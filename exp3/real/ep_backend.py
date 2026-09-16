# -*- coding: utf-8 -*-
"""EP 真机 PickPlace 后端 —— 把 exec_contract.PickExecutor 的六段接到 ep/drive/rm.py 的原语上。

契约不变：goal 只有 cell_id + cls，六段还是 descend/grasp/lift/carry/release/retract，
reason 枚举还是 sort_core.taxonomy 那一套。真机与仿真的差别**只在这两个缝隙的实现**
（scan 见 camera.py，pick_place 见本文件）。

三条设计决定，都是 EP 这个平台的特殊性逼出来的：

1. **按 id 查位姿，不用 ctx 里的表坐标。**
   ctx.A/ctx.B 是桌面系 (x_m, y_m) —— 那是识别/几何那条线的坐标系（相机标定出来的）。
   EP 底盘是里程计系（odom，以上电位姿为原点），两个系之间没有已知的对应关系，也**没有
   必要建立**：goal 只给 cell_id+cls（contract v1 就是这么定的），执行侧拿 id 去
   ep_waypoints.json 查底盘位姿与臂姿态就够了。两套坐标因此各管各的，永远不用互相换算 ——
   这是"换格/换料盒不改接口"那句话在真机侧的实际收益。

2. **held/placed 只能人工确认。**
   EP 没有任何物块位姿感知（红外/视觉感知不在骨架范围）。`assume_ok` 只给离线演练，真机跑
   必须 operator_confirm —— 对齐实验二 judge 口径。人工确认的是"夹住没有/放正没有"，
   **不是**判类别、也不是触发动作（docx §六 禁的是后者：类别由识别给、动作由任务控制给）。

3. **任何一段失败都要立即收尾。**
   单段失败 → 映射成 taxonomy 的 action reason（STAGE_REASON），并立刻 safe_reset
   （开爪/抬到高/回 HOME，尽力而为）。理由：EP 是开环的，失败之后机械臂停在哪儿没人知道，
   不收拾干净就接着跑下一个物体，等于拖着机械臂横扫桌面。
"""
import time

from ros2.task_control.exec_contract import PickExecutor, resolve_goal
from sort_core.task import OK
from sort_core.taxonomy import (REASON_DROPPED, REASON_EXEC_ERROR, REASON_GRASP_FAILED,
                                REASON_UNREACHABLE, STAGE_APPROACH, STAGE_GRASP, STAGE_LIFT,
                                STAGE_MOVE_TO_BIN, STAGE_PLACE, STAGE_RETRACT, STAGE_NAMES)

# 段号 → 失败时的 action reason。真机上**每一段都只能靠异常失败**（只有 lift/release 会
# 返回 False，其余段没有可判的物理结果），所以这张表实际上是"按段解释那次异常"。
# 定表的总规则（比逐条记忆好记）：
#
#     **只报这一段的代码真能做出来的判断，不替谁宣布一个没人做过的判定。**
#
#   - 到不了格/料盒（descend/carry）= 底盘开环走偏、被挡、命令被拒 → unreachable_target
#   - 夹爪闭合动作本身失败（grasp）= 只有异常可能，是硬错 → exec_error
#         ← 这里曾经填 grasp_failed, 是错的：grasp() 只调 gripper_close()，**从没判过
#           held**；而 taxonomy 里 grasp_failed 的定义是"抬离后判定没夹起"。填它等于
#           凭空宣布一个没做过的判定。held 的判定在 lift 段（STAGE_LIFT），那才是它。
#   - 抬离后判 held 不过（lift）    = 真的没夹住 → grasp_failed
#   - 放置后判 placed 不过（release）= 掉/放偏/还在爪里 → dropped_or_missed_place
#   - 回不了位（retract）           = 物已放下但机械臂停在未知处 → exec_error
#         ← 仿真线这里填 unreachable_target, 两条线**故意不一致**：那个 reason 来自
#           规划器的可达性判定，而 EP **没有规划层**，没有任何东西产出过"不可达"这个
#           结论；报 exec_error（驱动硬错）才是如实描述。同一规则的另一面。
STAGE_REASON = {
    STAGE_APPROACH: REASON_UNREACHABLE,
    STAGE_GRASP: REASON_EXEC_ERROR,
    STAGE_LIFT: REASON_GRASP_FAILED,
    STAGE_MOVE_TO_BIN: REASON_UNREACHABLE,
    STAGE_PLACE: REASON_DROPPED,
    STAGE_RETRACT: REASON_EXEC_ERROR,
}

# 六段的调用顺序（与 PickPlace.action 的阶段号一致）
STAGE_ORDER = ((STAGE_APPROACH, 'descend'), (STAGE_GRASP, 'grasp'), (STAGE_LIFT, 'lift'),
               (STAGE_MOVE_TO_BIN, 'carry'), (STAGE_PLACE, 'release'),
               (STAGE_RETRACT, 'retract'))


class _NullLog:
    def text(self, msg):
        pass


class EPPickExecutor(PickExecutor):
    """六段 → EP 原语。

    rm 需要的最小面（ep/drive/rm.py 的 RM 与 real/dry_run.py 的 DryRunRM 都满足）：
        goto(x_m, y_m, z_deg) / arm_moveto(x_mm, y_mm, tag=) / gripper_open() / gripper_close()
    ep_cfg ：config_real/ep_waypoints.json 的内容（现场标定值，未标定为 null）
    ask    ：人工确认回调 (question) -> bool；judge 要求 operator_confirm 时**必须**给，
             没给就直接报错 —— 免得"以为有人在看，其实没人看"
    """

    def __init__(self, rm, ep_cfg, log=None, on_stage=None, ask=None):
        self.rm = rm
        self.cfg = ep_cfg or {}
        self.log = log or _NullLog()
        self.on_stage = on_stage
        self.ask = ask
        self.arm = self.cfg.get('arm') or {}
        self.chassis = self.cfg.get('chassis') or {}
        self.judge = self.cfg.get('judge') or {}
        self.cell_pose = {k: v for k, v in (self.cfg.get('cell_pose') or {}).items()
                          if k != '_note'}
        self.bin_pose = {k: v for k, v in (self.cfg.get('bin_pose') or {}).items()
                         if k != '_note'}
        # 人工介入计数：docx §六 要求"正常任务过程中不进行人工类别判断和动作触发"，
        # 这个数就是那一条的**凭据** —— 它只该等于 2×物体数(夹起/放正各问一次)，
        # 多了说明有人在替系统做决定。
        self.n_asks = 0

    # ---------- 小工具 ----------
    def _stage(self, no):
        if self.on_stage:
            self.on_stage(no, STAGE_NAMES.get(no, str(no)))
        self.log.text('[stage %d] %s' % (no, STAGE_NAMES.get(no, '?')))

    def _do(self, what, fn, *a, **kw):
        """跑一个 EP 原语；抛错时带上"哪一步"的上下文 —— 现场看 run.log 就知道停在哪。"""
        try:
            return fn(*a, **kw)
        except Exception as e:
            raise RuntimeError('%s 失败: %s: %s' % (what, type(e).__name__, e))

    def _try(self, what, fn, *a, **kw):
        """收尾用：失败只记 WARN，不往上抛（已经在失败路径上了）。"""
        try:
            return fn(*a, **kw)
        except Exception as e:
            self.log.text('WARN: %s 失败(臂可能不在安全位): %s' % (what, e))
            return None

    def _pose(self, table, key, what):
        p = table.get(key)
        if not p:
            raise RuntimeError('%s %r 的底盘位姿未标定(%r) —— 闸门本该拦住它, '
                               '说明 ep_waypoints.json 被改过' % (what, key, p))
        return p

    def _arm_pose(self, key, what):
        v = self.arm.get(key)
        if not v:
            raise RuntimeError('arm.%s 未标定 —— 用 ep/drive/env_check.py --jog 试出来再填' % key)
        return v

    def _grab_low(self):
        return self._arm_pose('grab_low_mm', '抓取低点')

    def _lift_high(self):
        return self._arm_pose('lift_high_mm', '抬升高点')

    def _release_low(self):
        """放置低点：没单独标就与抓取低点同高（料盒面高与桌面一致时的常规情形）。"""
        return self.arm.get('release_low_mm') or self._grab_low()

    def _home(self):
        p = self.chassis.get('home_pose')
        if not p:
            raise RuntimeError('chassis.home_pose 未标定')
        return p

    def _assume(self, what):
        return self.judge.get(what) == 'assume_ok'

    def _ask(self, tag, ctx):
        if self.ask is None:
            raise RuntimeError('judge.%s=operator_confirm 但没有人工确认回调(ask) —— '
                               '要么接键盘, 要么显式改成 assume_ok' % tag)
        if tag == 'held':
            q = 'HELD 确认: 夹住了 %s(网格 %s) 且没滑?' % (ctx['cls'], ctx['cell_id'])
        else:
            q = 'PLACED 确认: %s 落在 %s 里且没翻?' % (ctx['cls'], ctx['bin_id'])
        self.n_asks += 1
        return bool(self.ask(q))

    # ---------- 段外动作：预备 / 收尾 ----------
    def prepare(self):
        """一次 pick_place 开始前的状态归位：抬到高位 + 开爪。

        为什么必须有 —— retract 失败或操作员 Ctrl-C 之后，臂可能停在低处；这时候让底盘
        goto 就是拖着机械臂横扫桌面。归位失败**不吞**：直接抛，由 run_real 判成 exec_error
        停线（宁可整个任务停，也不带着未知臂位接着跑）。
        """
        self._do('预备-抬到高位', self.rm.arm_moveto, *self._lift_high(), tag='prepare_high')
        self._do('预备-开爪', self.rm.gripper_open)

    def safe_reset(self, why=''):
        """失败收尾（尽力而为，不抛）：开爪 → 抬到高 → 回 HOME。"""
        if why:
            self.log.text('收尾(%s): 尽力回到已知状态' % why)
        self._try('收尾-开爪', self.rm.gripper_open)
        hi = self.arm.get('lift_high_mm')
        if hi:
            self._try('收尾-抬到高位', self.rm.arm_moveto, *hi, tag='safe_high')
        home = self.chassis.get('home_pose')
        if home:
            self._try('收尾-回 HOME', self.rm.goto, *home)

    # ---------- 六段（对齐 PickPlace.action 的阶段号） ----------
    def descend(self, ctx):
        self._stage(STAGE_APPROACH)
        self._do('开到网格 %s' % ctx['cell_id'], self.rm.goto,
                 *self._pose(self.cell_pose, ctx['cell_id'], '网格'))
        self._do('下探到抓取低点', self.rm.arm_moveto, *self._grab_low(), tag='descend')
        return True, '已到 %s 抓取位' % ctx['cell_id']

    def grasp(self, ctx):
        self._stage(STAGE_GRASP)
        self._do('夹爪压紧', self.rm.gripper_close)
        return True, '已压紧 %s' % ctx['cell_id']

    def lift(self, ctx):
        self._stage(STAGE_LIFT)
        retry = int(self.judge.get('retry_grab', 1))
        note = ''
        for att in range(1 + retry):
            self._do('抬到高位', self.rm.arm_moveto, *self._lift_high(), tag='lift')
            # 等一下再问人：刚抬起来物体还在晃，这时候问 HELD 容易误判
            time.sleep(float(self.arm.get('lift_wait_s', 1.0)))
            if self._assume('held'):
                return True, '已抬起（held 未判，assume_ok）'
            if self._ask('held', ctx):
                return True, '已抬起（操作员确认夹住）'
            note = 'HELD 未确认（没夹起/滑落）'
            if att < retry:
                self.log.text('HELD=n，重抓（剩 %d 次）：开爪→下探→压紧→提起' % (retry - att))
                self._do('重抓-开爪', self.rm.gripper_open)
                self._do('重抓-下探', self.rm.arm_moveto, *self._grab_low(), tag='retry_descend')
                self._do('重抓-压紧', self.rm.gripper_close)
        return False, note

    def carry(self, ctx):
        self._stage(STAGE_MOVE_TO_BIN)
        self._do('开到料盒 %s' % ctx['bin_id'], self.rm.goto,
                 *self._pose(self.bin_pose, ctx['bin_id'], '料盒'))
        return True, '已到料盒 %s' % ctx['bin_id']

    def release(self, ctx):
        self._stage(STAGE_PLACE)
        self._do('下放到放置低点', self.rm.arm_moveto, *self._release_low(), tag='release')
        time.sleep(float(self.arm.get('drop_wait_s', 1.0)))
        self._do('开爪释放', self.rm.gripper_open)
        if self._assume('placed'):
            return True, '已释放（placed 未判，assume_ok）'
        if self._ask('placed', ctx):
            return True, '已释放（操作员确认落正）'
        return False, 'PLACED 未确认（落偏/翻了/还在爪里）'

    def retract(self, ctx):
        """退回安全位。这一段**自带重试**：物已经放下了，为一次读位抖动把整轮判失败不值得；
        但连试 retry_retract+1 次都回不去 → 真抛（臂停在未知处，必须停线）。"""
        self._stage(STAGE_RETRACT)
        tries = 1 + int(self.judge.get('retry_retract', 1))
        last = None
        for att in range(tries):
            try:
                self._do('抬到高位', self.rm.arm_moveto, *self._lift_high(), tag='retract')
                self._do('回 HOME', self.rm.goto, *self._home())
                return True, '已退回 HOME' + ('（第 %d 次成功）' % (att + 1) if att else '')
            except Exception as e:
                last = e
                self.log.text('WARN: retract 第 %d/%d 次失败: %s' % (att + 1, tries, e))
                time.sleep(0.5)
        raise RuntimeError('回安全位连续 %d 次失败: %s' % (tries, last))


class EPPickPlace:
    """TaskController 的 pick_place 缝隙：(cell_id, cls) -> ('ok', note) | (action_reason, note)。

    职责只有三件：goal 校验（复用 resolve_goal，与仿真侧同一份）、按顺序驱动六段、
    把段的成败翻译成 taxonomy 的 reason。**不做**类别判断、不做目标选择 —— 那是
    TaskController 的事，这一层只管"把这个动作做出来"。
    """

    def __init__(self, executor, grid, bins, log=None, prepare=True):
        self.ex = executor
        self.grid = grid
        self.bins = bins
        self.log = log or _NullLog()
        self.prepare = prepare

    def __call__(self, cell_id, cls):
        ctx, err = resolve_goal(self.grid, self.bins, cell_id, cls)
        if err:
            return REASON_EXEC_ERROR, 'goal 非法: %s' % err
        if self.prepare:
            try:
                self.ex.prepare()
            except Exception as e:
                return REASON_EXEC_ERROR, '预备归位失败，拒绝动作（臂不在安全位）: %s' % e
        notes = {}
        for no, name in STAGE_ORDER:
            try:
                ok, note = getattr(self.ex, name)(ctx)
            except Exception as e:
                return self._fail(no, str(e))
            if not ok:
                return self._fail(no, note)
            notes[name] = note
        # 成功也要把关键段的说明带上：
        #   held/placed —— 是"人工确认"还是"assume"的**凭据**（docx §六 禁止人工替系统做
        #                 判断，验收时要查得出来）；
        #   retract     —— 记着"第 N 次成功"。重试过一次说明这个点位已经在临界上，
        #                  是下一轮要调的对象，别让它淹在 6 条一模一样的成功记录里。
        return OK, '%s→%s 完成; held=%s; placed=%s; retract=%s' % (
            cell_id, ctx['bin_id'], notes.get('lift', '?'),
            notes.get('release', '?'), notes.get('retract', '?'))

    def _fail(self, stage, detail):
        note = 'stage=%d(%s) %s' % (stage, STAGE_NAMES.get(stage, '?'), detail)
        self.log.text('失败: %s' % note)
        self.ex.safe_reset(note)
        return STAGE_REASON[stage], note
