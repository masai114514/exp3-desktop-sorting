#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboMaster SDK 薄封装(EP 工程形态)。所有真机调用集中在 RM 类, 便于换 API/加日志。

设计要点:
- robomaster 官方 SDK 只能现场 pip/源码安装, 本机离线没有 —— 这里**延迟导入**并在缺包时给
  明确提示; 平台差异/属性名差异用 env_check.py 真机探测兜底。
- 官方文档里 机械臂 模块名的占位是 robotic_arm, 但多机汇总 API 里也叫 robotic_arm; 个别
  示例/镜像写法是 ep_robot.arm。无法离线坐实 → RM.resolve 按 config.arm.attr_candidates
  逐个 hasattr, 探测到哪个用哪个并在日志里写明(env_check 首跑会打印)。
- **位置读数只能靠订阅**(2026-09-13 对着官方 0.1.1.68 源码 + 导入内省坐实): SDK 里
  `chassis.get_position()` / `arm.get_position()` **都不存在**(`Chassis`/`RoboticArm` 只有
  `sub_position`/`unsub_position`)。所以本封装在 connect() 时订阅一次, 之后"读位置" = 取最近
  一次推送 + 判新鲜度(见 _PushCache)。原先按同步 getter 写, 真机上第一次 goto 就会
  AttributeError、臂读数则静默变 NaN。
- chassis 里程计与 move 均相对底盘系; goto() 读当前 odom 做误差纠正(单次 move + 复核), 容忍漂移。
  订阅用 `cs=1`(以**上电**位姿为原点, 见 config.chassis.odom_cs): 标定读的 HOME/A/B 就是上电
  累计值, 两端必须同口径 —— `cs=0` 是"以订阅那一刻的位置为原点", 拿它标定/复现都会错位。
- moveto/open/close 返回值是否 Action(要 wait_for_completed)在不同 SDK 版本不一致 → _finish
  统一处理: 有 wait_for_completed 就等, 没有就当同步已完成; 另外 action 的**成功位**
  (has_succeeded) 单独看 —— wait_for_completed 早返回不代表动作成功(见 _finish)。
"""
import sys
import threading
import time

try:
    from robomaster import robot
    _SDK_OK = True
except Exception as e:            # 未安装/半装都进这里
    robot = None
    _SDK_ERR = str(e)
    _SDK_OK = False

# 单独 import: 拿不到 conn 也不该把整个 SDK 判成不可用(离线测试的替身包就只有 robot),
# 拿不到时 _sdk_conn_type 会退化为 sys.intern, 一样能过 SDK 的 `is` 比较。
try:
    from robomaster import conn as _conn
except Exception:
    _conn = None


# 连接类型**必须**用 SDK 自己那个字符串对象 —— conn.request_connection 里是
#     if conn_type is CONNECTION_WIFI_AP: / elif ... is CONNECTION_WIFI_STA: ...
# 用 `is` 比的(不是 ==)。而我们从 config_ep.json 读出来的 conn_type 是 json.load 新建的
# 字符串,与 SDK 模块里的字面量**不是同一个对象**(实测 `json 读出 is 常量 → False`) ——
# 三个分支一个都不命中, proxy_addr 根本没赋值, 连接必失败。
# 甲在现场能连上, 只因为他脚本里写的是源码字面量(字面量会被驻留)。
# 所以这里把 config 里的写法映射到 SDK 的常量; 万一 import 不到就退化为驻留(interning),
# 驻留后与模块里的字面量同对象, 一样能过 `is`。
_CONN_TYPE_ALIASES = {
    'ap': 'CONNECTION_WIFI_AP',
    'sta': 'CONNECTION_WIFI_STA',
    'rndis': 'CONNECTION_USB_RNDIS',
}


def _sdk_conn_type(raw):
    """把 config 里的 'ap'/'sta'/'rndis' 映射成 SDK 自己的连接类型常量。"""
    s = str(raw).strip().lower()
    name = _CONN_TYPE_ALIASES.get(s)
    if _SDK_OK and name and hasattr(_conn, name):
        return getattr(_conn, name)
    return sys.intern(s)

# 机械臂坐标的编码: SDK 的 ArmSubject.decode 用 struct.unpack('<II') —— **无符号**,
# 于是负的 mm 会以接近 2**32 的样子出现(同款 SDK 的官方示例处理方式一致, 社区记作
# signed_int32_mm)。不做这一步, 臂一旦走到负坐标, 我们就会拿 4.29e9 当读数去比对。
_UINT32 = 2 ** 32
_INT32_MAX = 2 ** 31


def _signed_mm(v):
    """把无符号回读还原成有符号 mm(仅对 ≥2**31 的高半区做补码还原)。"""
    iv = int(v)
    return iv - _UINT32 if iv >= _INT32_MAX else iv


def _finish(ret, timeout=60, log=None, what=''):
    """Action 对象就 wait_for_completed; 返回 None 就当同步完成。

    额外看 action 的成功位: 实测 `wait_for_completed` 47ms 就返回 True、而臂还要 0.5s 才动
    (见 _settle_arm), 所以"等待返回"不等于"动作成功"。厂商把结果放在 has_succeeded 里,
    这里不吞掉失败 —— 只告警, 由上层决定怎么办(标定时看到就别照读回值填 config)。
    """
    if ret is not None and hasattr(ret, 'wait_for_completed'):
        ret.wait_for_completed(timeout=timeout)
        if hasattr(ret, 'has_succeeded') and ret.has_succeeded is not True:
            (log or _NullLog()).text('WARN: {} 动作未成功(state={}) —— 别把这次读回当到位值'
                                     .format(what or 'action', getattr(ret, 'state', '?')))
    return ret


class _PushCache:
    """最近一次订阅推送的缓存(SDK 0.1.1.68 没有同步 getter, 位置只能订阅)。

    为什么不用"读一次就问一次": 位置是 5Hz 推上来的, 读的多半是同一个样本。所以除了值,
    还要留 **序号**(_count) —— 判"连续两次一致"时只有新样本才算数(_settle_arm 用)。
    """

    def __init__(self, label, convert=None):
        self.label = label
        self._convert = convert or (lambda v: [float(x) for x in v])
        self._lock = threading.Lock()
        self._value = None
        self._at = 0.0
        self._count = 0
        self._error = None

    def callback(self, value, *args, **kwargs):
        """订阅回调: 回调只收一个 tuple(DDS subject 的 data_info), 不是拆开的多个参数。"""
        with self._lock:
            try:
                self._value = self._convert(value)
                self._error = None
            except Exception as e:          # 坏样本不复用上一个好值, 免得更错
                self._value = None
                self._error = str(e)
            self._at = time.monotonic()
            self._count += 1

    def read_sample(self, max_age=1.0, wait=0.0):
        """取最新推送值, 连序号一起返回: (值, 序号)。wait>0 时最多等这么久等**第一**个样本。

        max_age 是"新鲜度"闸门: 推送断了(掉线/订阅失效)就报错, 别把几秒前的旧值当真值用。
        值和序号在同一次加锁里取出, 免得判"新样本"时比到一个撕裂的组合。
        """
        deadline = time.monotonic() + max(0.0, wait)
        while True:
            with self._lock:
                v, at, n, err = self._value, self._at, self._count, self._error
            if err:
                raise RuntimeError('{} 推送数据无效: {}'.format(self.label, err))
            if v is not None:
                age = time.monotonic() - at
                if age <= max_age:
                    return list(v), n
                if time.monotonic() >= deadline:
                    raise RuntimeError('{} 最后一次推送在 {:.1f}s 前(推送断了? 超过 max_age={}s)'
                                       .format(self.label, age, max_age))
            elif time.monotonic() >= deadline:
                raise RuntimeError('{} 一个样本都没收到({}s): 订阅没生效? EP 掉线?'
                                   .format(self.label, max(0.0, wait)))
            time.sleep(0.05)

    def read(self, max_age=1.0, wait=0.0):
        """只要值(不关心序号)的简写。"""
        return self.read_sample(max_age=max_age, wait=wait)[0]


class RM:
    """持有 ep_robot + 各模块句柄; 失败抛 RuntimeError(带可读信息)。"""

    # 位置推送滞后(实测值见 _settle_arm 注释): 轮询上限与"算稳定"的容差(mm)
    # 超时给得宽: 正常动作靠"已偏离旧值 + 连续两次一致"提前返回(实测 ~0.4s), 这个上限只在
    # 行程被钳制/推送异常时才吃到。给太紧(2.0s)的代价是: lift_high→grab_low 这种几十 mm 的
    # 大行程会误报"未稳定", 然后把旧读数当结果返回 —— 那正是这个函数要防的事。
    ARM_SETTLE_TIMEOUT_S = 6.0
    ARM_SETTLE_TOL_MM = 1.0
    # "没到位"的告警阈值: 比抖动量级宽一档(实测稳态 ±1mm 抖动 + x 有 +1mm 漂移),
    # 超过它才算真的没到 —— 用途见 arm_moveto 末尾。
    ARM_MISS_TOL_MM = 3.0

    def __init__(self, cfg, log=None):
        self.cfg = cfg
        self.log = log or _NullLog()
        self.ep = None
        self.chassis = None
        self.arm = None
        self.arm_attr = None
        self.gripper = None
        self.grip_state = '?'
        # 位置只能订阅(见模块 docstring): 两个缓存由 connect() 里的 sub_position 喂
        self._odom = _PushCache('chassis odom')
        self._arm_pos = _PushCache('arm x/y', convert=lambda v: [_signed_mm(x) for x in v])
        self._subscribed = []

    # ---------- 连接 / 探测 ----------
    def connect(self):
        if not _SDK_OK:
            raise RuntimeError(
                'robomaster SDK 未安装/导入失败({}); EP 真机环境请先装官方 SDK'
                '(DJI 开发者站或 github dji-sdk/RoboMaster-SDK, python setup.py install),'
                ' 再跑 env_check.py'.format(_SDK_ERR if _SDK_OK is False else '?'))
        rt = self.cfg['robot']
        conn_type = _sdk_conn_type(rt['conn_type'])
        self.log.text('连接 EP: initialize(conn_type={})'.format(conn_type))
        ep = robot.Robot()
        kwargs = dict(conn_type=conn_type)
        for k in ('proto_type',):
            if rt.get(k):
                kwargs[k] = rt[k]
        ok = ep.initialize(**kwargs)
        if not ok:
            ep.close()
            raise RuntimeError('ep.initialize({}) 失败 —— 检查热点/网段/是否工程形态开机'.format(kwargs))
        self.ep = ep
        self.chassis = ep.chassis
        self._resolve_modules()
        self._subscribe_positions()
        return self

    def _subscribe_positions(self):
        """订阅底盘/机械臂位置推送 —— 这是本 SDK 拿位置的**唯一**途径。

        cs 用 config.chassis.odom_cs(默认 1 = 以**上电**位姿为原点): 标定时读的 HOME/A/B
        就是上电累计值, 标定与运行必须同口径。若 SDK 版本没有 cs 形参, 退化为不带 cs 订阅
        并明确告警 —— 那种情况下原点取决于订阅时刻, 标定前务必先确认。
        """
        ch = self.cfg.get('chassis', {})
        cs = int(ch.get('odom_cs', 1))
        freq = int(ch.get('sub_freq_hz', 5))
        try:
            ok = self.chassis.sub_position(cs=cs, freq=freq, callback=self._odom.callback)
        except TypeError:
            self.log.text('WARN: chassis.sub_position 不接受 cs 形参, 退化为默认原点(订阅时刻); '
                          '标定与运行都要在同一步之后订阅, 否则 HOME/A/B 会错位')
            ok = self.chassis.sub_position(freq=freq, callback=self._odom.callback)
        if ok is not True:
            self.log.text('WARN: chassis.sub_position 订阅未确认(odom_cs={}) —— 后面读 odom 会报'
                          '"一个样本都没收到"'.format(cs))
        else:
            self._subscribed.append(('chassis', self.chassis.unsub_position))
        aok = self.arm.sub_position(freq=freq, callback=self._arm_pos.callback)
        if aok is not True:
            self.log.text('WARN: arm.sub_position 订阅未确认 —— 后面读臂位置会报"一个样本都没收到"')
        else:
            self._subscribed.append(('arm', self.arm.unsub_position))
        self.log.text('位置订阅: chassis(cs={}) / arm, {}Hz'.format(cs, freq))
        # 首样本: 让"连上了但还没推上来"与"连不上"分开报, 也确认 cs 形参真的生效
        try:
            pose = self._odom.read(wait=5.0)
            xy = self._arm_pos.read(wait=5.0)
            self.log.text('首个位置样本: odom={} arm={} mm'.format(
                [round(v, 3) for v in pose], xy))
        except RuntimeError as e:
            self.log.text('WARN: 首样本未到 —— {}'.format(e))

    def _resolve_modules(self):
        cands = self.cfg['arm'].get('attr_candidates') or ['robotic_arm', 'arm']
        found = None
        for name in cands:
            if hasattr(self.ep, name):
                found = name
                break
        if found is None:
            raise RuntimeError(
                '在 ep_robot 上找不到机械臂属性(candidates={}); 确认这是**工程形态**(带 2 轴臂'
                '+ 夹爪)。步兵形态只有云台, 不适用。'.format(cands))
        self.arm = getattr(self.ep, found)
        self.arm_attr = found
        self.gripper = self.ep.gripper
        self.log.text('模块就绪: arm 经 ep_robot.{} (候选 {})'.format(found, cands))

    def probe(self):
        """env_check 用: 打印各模块可用的关键方法。"""
        out = {}
        for mod, name in ((self.chassis, 'chassis'), (self.arm, 'arm'), (self.gripper, 'gripper')):
            if mod is None:
                out[name] = None
                continue
            have = sorted(m for m in dir(mod) if not m.startswith('_'))
            out[name] = have
        return out

    # ---------- 底盘 ----------
    def read_chassis_pose(self, max_age=1.5):
        """当前 odom (x_m, y_m, z_deg) —— 取最近一次订阅推送(SDK 无同步 getter)。"""
        if self.chassis is None:
            raise RuntimeError('chassis 未就绪')
        return self._odom.read(max_age=max_age)

    def chassis_move(self, dx, dy, dz=0.0, xy_speed=None, z_speed=None):
        ch = self.cfg['chassis']
        _finish(self.chassis.move(
            x=dx, y=dy, z=dz,
            xy_speed=xy_speed if xy_speed is not None else ch['xy_speed'],
            z_speed=z_speed if z_speed is not None else ch.get('z_speed', 60.0)))

    def goto(self, x, y, z=None):
        """开到里程计目标位姿; 每段读 odom 做误差纠正(至多 2 次补正)。"""
        ch = self.cfg['chassis']
        tol = ch.get('move_tol_m', 0.03)
        for attempt in range(3):
            cur = self.read_chassis_pose()
            err_x = x - cur[0]
            err_y = y - cur[1]
            if err_x * err_x + err_y * err_y <= tol * tol:
                break
            if z is None:
                err_z = 0.0
            else:
                err_z = z - cur[2]
                err_z = (err_z + 180) % 360 - 180      # 最短转角
            self.log.text('goto 补正#{}: err=({:.3f},{:.3f},{:.1f}deg)'.format(
                attempt + 1, err_x, err_y, err_z))
            self.chassis_move(err_x, err_y, err_z)
        return self.read_chassis_pose()

    # ---------- 机械臂 ----------
    def arm_moveto(self, x_mm, y_mm, tag=''):
        if self.arm is None:
            raise RuntimeError('arm 未就绪(非工程形态?)')
        before = self.read_arm_xy()
        try:
            ret = self.arm.moveto(x=int(x_mm), y=int(y_mm))
        except TypeError:                             # 老版本只吃位置参数
            ret = self.arm.moveto(int(x_mm), int(y_mm))
        _finish(ret, log=self.log, what='arm.moveto({},{})'.format(int(x_mm), int(y_mm)))
        pos = self._settle_arm(before, target=(x_mm, y_mm))
        self.log.text('arm_moveto{}({},{}mm) -> 读回 {}'.format(
            '[' + tag + ']' if tag else '', int(x_mm), int(y_mm), pos))
        # 只判"稳定"不够: 命令超出行程被钳制、或动作根本没执行时, 读数同样会稳定下来,
        # 于是返回一个"看着挺确定"的值 —— 标定时照它填 config 就把错的值当标定值了。
        miss = max(abs(pos[0] - x_mm), abs(pos[1] - y_mm))
        if miss > self.ARM_MISS_TOL_MM:
            self.log.text('WARN: arm 未到位 —— 要求 ({},{}), 读回 {} (差 {:.0f}mm)。'
                          '多半是超出行程被钳制; 标定时**别**把这个读回值当到位值填 config, '
                          '换个更靠内的目标再试'.format(
                              int(x_mm), int(y_mm), pos, miss))
        return pos

    def _settle_arm(self, before, timeout=None, tol=None, need_stable=2, target=None):
        """等位置推送追上动作, 返回稳定后的读数(超时则返回最后一次读数并告警)。

        为什么必须等 —— 实测(2026-09-13, EP_Arm_5mm_Test_v2 的 result.json, 100 条采样):
          指令 22:20:23.297 → wait_for_completed 47ms 就返回 action_succeeded, 但 SDK 快照里
          仍是旧位姿 x:74,y:57; 位置推送到 .593 才 74,58, 到 .984 才 75,62。
          即"动作完成"比"位置推送更新"早约 0.5s(推送周期约 200ms)。
        紧跟 _finish 就读会拿到动作【前】的旧位姿 —— 标定说明 §2 让人"记下读回的 x/y 填
        config", 照这样做就会把旧值当到位值填进去。

        判据 = 读数已偏离动作前的值 且 连续 need_stable 个**新推送样本**两两一致。
        三个要点都是有依据的, 少一个就出前面那种错:
        - "偏离动作前的值": 滞后期间旧值本身就是稳定的, 只判"稳定"会把动作前的位姿当成到位值。
        - "新样本"而不是"新一次读取": 位置是 5Hz 推上来的, 读的是缓存; 轮询比推送快时同一个
          样本会被读到两次, 拿它当"两次一致"等于没判(所以比 read_sample 的序号)。
        - `target` 给了就先看"是不是本来就在目标上"(空动作/极小步), 是就直接返回 —— 否则
          空动作永远等不到"偏离旧值", 白白烧满 timeout 再报一次假 WARN。
        另: 偏差判据用 > tol 而非 !=, 因为稳态本身有 ±1mm 抖动。
        """
        timeout = self.ARM_SETTLE_TIMEOUT_S if timeout is None else timeout
        tol = self.ARM_SETTLE_TOL_MM if tol is None else tol
        t0 = time.monotonic()
        prev, prev_seq = self.read_arm_xy_sample()
        if target is not None and not any(abs(a - b) > tol for a, b in zip(prev, before)) \
                and max(abs(prev[0] - target[0]), abs(prev[1] - target[1])) <= self.ARM_MISS_TOL_MM:
            return prev                                    # 本来就在目标附近, 空动作
        moved = any(abs(a - b) > tol for a, b in zip(prev, before))
        stable = 0
        while time.monotonic() - t0 < timeout:
            cur, seq = self.read_arm_xy_sample(wait=0.3)
            if seq == prev_seq:
                continue                                   # 还没等到新推送, 这次不算
            prev_seq = seq
            if not moved:
                if any(abs(a - b) > tol for a, b in zip(cur, before)):
                    moved, stable = True, 0                # 刚动起来, 重新数稳定
                prev = cur
                continue
            if all(abs(a - b) <= tol for a, b in zip(cur, prev)):
                stable += 1
                if stable >= need_stable:
                    return cur
            else:
                stable = 0
            prev = cur
        self.log.text('WARN: arm 读数 {}s 内未稳定(行程被钳制? 推送断了?); '
                      '返回最后读数 {} —— 别拿它当到位值填 config'.format(timeout, prev))
        return prev

    def arm_recenter(self):
        if self.arm is None:
            raise RuntimeError('arm 未就绪')
        ret = self.arm.recenter()
        _finish(ret, log=self.log, what='arm.recenter')

    def read_arm_xy_sample(self, max_age=1.0, wait=0.0):
        """(臂 x/y mm, 推送序号) —— 取最近一次订阅推送, 序号给判"新样本"的调用方。"""
        if self.arm is None:
            raise RuntimeError('arm 未就绪')
        return self._arm_pos.read_sample(max_age=max_age, wait=wait)

    def read_arm_xy(self, max_age=1.0):
        """臂 x/y (mm)。原先走 SDK 的 arm.get_position() —— 该方法**不存在**, 只会静默返回
        NaN(被 hasattr 兜住), 真机上臂读数等于没有; 现在改读订阅推送(见模块 docstring)。"""
        return self.read_arm_xy_sample(max_age=max_age)[0]

    # ---------- 夹爪 ----------
    def gripper_open(self):
        gp = self.cfg['gripper']
        _finish(self.gripper.open(power=gp['open_power']), log=self.log, what='gripper.open')
        self.grip_state = 'open'
        time.sleep(gp.get('act_wait_s', 1.5))

    def gripper_close(self):
        gp = self.cfg['gripper']
        _finish(self.gripper.close(power=gp['close_power']), log=self.log, what='gripper.close')
        self.grip_state = 'closed'
        time.sleep(gp.get('act_wait_s', 1.5))

    def disconnect(self):
        """先退订阅再关连接: 订阅的回调在断连后仍可能被调到, 退干净少一类退出期异常。"""
        for name, unsub in reversed(self._subscribed):
            try:
                unsub()
            except Exception as e:
                self.log.text('{} 退订异常(可忽略): {}'.format(name, e))
        self._subscribed = []
        if self.ep is not None:
            try:
                self.ep.close()
            except Exception as e:
                self.log.text('ep.close() 异常(可忽略): {}'.format(e))
            self.ep = None


class _NullLog:
    def text(self, msg):
        print(msg, flush=True)

    def event(self, kind, **kw):
        pass
