# -*- coding: utf-8 -*-
"""C4PickExecutor —— 用 c4_2 直驱原语实现 PickExecutor（多格多料盒版单次取放）。

它**组合**一个 C4Driver 实例（c4_2/drive/c4_cycle.py）当“robot”，只调它的运动原语，不再
照搬 C4Driver.run() 的固定 A/B 周期。与 c4_2 cycle 的三点差异（联调时最容易踩）：

  1. **不 reset_block**：c4_2 每周期把方块删了在 A_center 重生；实验3 的物体由仿真场景摆在各格、
     任务控制选哪格就夹哪格 —— 这里只做『到格→夹→抬→搬→放→退』，不碰 spawn/delete。
  2. **多 A/B**：A(格中心)、B(料盒中心) 由 server 的 resolve_goal 现查放 ctx，本类不读 config。
  3. **判 held/placed 不做死**：sim 接触受限(见 c4_2 注释)时 held 判定不可靠，默认 assume
     (对齐 c4_2 --rehearsal 口径)。真判据在 Jetson 联调时 override _judge_hold/_judge_place
     （读物体位姿/或人工确认），本文件保持无 ROS import、可离线用假 robot 测时序。

robot 需要的最小运动面（C4Driver 均满足；返回签名照 c4_2/drive/c4_cycle.py）：
    go_cartesian(xyz, label)          -> (ok:bool, path|None)   # plan_segment 直线(锁姿态)
    go_chain(chain, label)            -> (ok:bool, st, att)     # 关节链/单点 JTC
    grip_set(vals, label, settle=None)-> (ok:bool, st)          # 夹爪
全部几何/关节参数集中在 motion（构造入参），联调由甲按标定覆盖；默认值参考 c4_2/config_sim.json。
"""
from ros2.task_control.exec_contract import PickExecutor


class C4PickExecutor(PickExecutor):
    """motion 字段（构造入参，见模块注释；除 *_assume 外都是数值/关节列）：
        home            : 回零关节列(可选，None 则 retract 不回零)
        gripper_open/close : 张开/压紧的夹爪关节指令(可选)
        grasp_z         : 爪中(mid frame)抓取高度，默认 0.038(c4_2 sim grasp_mid_z)
        lift_dz         : 抬离增量，默认 0.1(c4_2 sim lift_dz)
        place_z         : 料盒放置时爪中高度，默认 = grasp_z
        lift_retry      : 抬离后判未夹起时，就地重压再抬的次数，默认 1
        press_settle / release_settle : 夹紧/张开后的沉降时间(s)
        hold_assume / place_assume    : True=判 held/placed 一律通过(彩排口径)
    """

    def __init__(self, robot, motion):
        self.r = robot
        self.m = motion or {}
        self._zg = self.m.get('grasp_z', 0.038)
        self._lift_dz = self.m.get('lift_dz', 0.1)
        self._place_z = self.m.get('place_z', self._zg)
        self._open = self.m.get('gripper_open')
        self._close = self.m.get('gripper_close')
        self._press_settle = self.m.get('press_settle', 4.0)
        self._release_settle = self.m.get('release_settle', 3.0)
        self._lift_retry = int(self.m.get('lift_retry', 1))
        self._hold_assume = bool(self.m.get('hold_assume', True))
        self._place_assume = bool(self.m.get('place_assume', True))

    # ---------- 目标几何（全部从 ctx；motion 只给 z 语义） ----------
    def _targets(self, ctx):
        A, B = ctx['A'], ctx['B']
        zg = min(self._zg, A['z_pick'])          # 抓取高别高于格的安全高(pick_z_m)
        A_above = (A['x'], A['y'], A['z_pick'])  # 到格上方的安全高度
        A_grasp = (A['x'], A['y'], zg)
        A_lift = (A['x'], A['y'], zg + self._lift_dz)
        B_place = (B['x'], B['y'], self._place_z)
        return A_above, A_grasp, A_lift, B_place

    # ---------- 判据（默认 assume；Jetson 联调 override 接真感知） ----------
    def _judge_hold(self, ctx):
        return self._hold_assume, 'held assumed（联调时 override _judge_hold 接真感知）'

    def _judge_place(self, ctx):
        return self._place_assume, 'placed assumed（联调时 override _judge_place）'

    # ---------- 6 段动作 ----------
    def descend(self, ctx):
        A_above, A_grasp, _, _ = self._targets(ctx)
        if self._open is not None:
            ok, _ = self.r.grip_set(self._open, 'OPEN@descend')
            if not ok:
                return False, '开爪执行失败'
        ok, _ = self.r.go_cartesian(A_above, 'TO_ABOVE_A')
        if not ok:
            return False, '到格 %s 上方失败 (A=%s)' % (ctx['cell_id'], A_above)
        ok, _ = self.r.go_cartesian(A_grasp, 'DOWN_TO_A')
        if not ok:
            return False, '下探到 %s 失败 (grasp_z=%.3f)' % (ctx['cell_id'], A_grasp[2])
        return True, 'approached %s' % ctx['cell_id']

    def grasp(self, ctx):
        if self._close is None:
            return False, 'motion 未给 gripper_close'
        ok, _ = self.r.grip_set(self._close, 'PRESS', settle=self._press_settle)
        if not ok:
            return False, '夹爪压紧执行失败'
        return True, 'pressed %s' % ctx['cell_id']

    def lift(self, ctx):
        _, _, A_lift, _ = self._targets(ctx)
        attempts = 1 + self._lift_retry            # 首次 + 重压重试次数
        for att in range(attempts):
            ok, _ = self.r.go_cartesian(A_lift, 'LIFT_A')
            if not ok:
                return False, '抬离规划/执行失败 (A_lift=%s)' % (A_lift,)
            held, note = self._judge_hold(ctx)
            if held:
                return True, 'lifted (%s)' % note
            # 只在还有下一次机会时就地重压（c4_2 同款 re-PRESS 逻辑）
            if att < attempts - 1 and self._close is not None:
                self.r.grip_set(self._close, 're-PRESS', settle=2.0)
        return False, '未夹起: %s' % note

    def carry(self, ctx):
        _, _, _, B_place = self._targets(ctx)
        ok, _ = self.r.go_cartesian(B_place, 'CARRY_TO_B')
        if not ok:
            return False, '搬运到料盒失败 (B=%s)' % (B_place,)
        return True, 'carried to %s' % ctx['bin_id']

    def release(self, ctx):
        if self._open is None:
            return False, 'motion 未给 gripper_open'
        ok, _ = self.r.grip_set(self._open, 'RELEASE', settle=self._release_settle)
        if not ok:
            return False, '张开放置执行失败'
        if self._place_assume:
            return True, 'released (placed assumed)'
        placed, note = self._judge_place(ctx)
        return placed, note

    def retract(self, ctx):
        A_above, _, _, _ = self._targets(ctx)
        ok, _ = self.r.go_cartesian(A_above, 'RISE_AWAY')
        if not ok:
            return False, '退回安全高失败'
        home = self.m.get('home')
        if home is None:
            return True, 'retracted（motion 无 home，未回零）'
        ok, st, _ = self.r.go_chain(home, 'HOME')
        return ok, 'HOME st=%s' % st
