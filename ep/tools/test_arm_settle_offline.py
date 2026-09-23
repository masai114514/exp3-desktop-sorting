#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证 ep/drive/rm.py 的位置读数与 _settle_arm: 用假 SDK 复现 5mm 实测的时序。

有网/无网、有机器人/没机器人 都能跑:  python3 ep/tools/test_arm_settle_offline.py

假 SDK 按 2026-09-13 EP_Arm_5mm_Test_v2 的真实时序建模(证据见
docs/真机侧证据_EP_20260913/):
  - 位置**只能订阅**(官方 0.1.1.68 没有 chassis/arm.get_position), 5Hz 推送, 所以有滞后;
  - moveto() 返回的 Action 在 ~47ms 就报 action_succeeded(不等真到位);
  - 臂坐标走 struct.unpack('<II') 无符号编码, 负值以接近 2**32 出现;
  - 到位后读数在 ±1mm 抖动, x 有 +1mm 偏移。

跑的场景:
  A 反证: 紧跟 wait_for_completed 就读 = 拿到**动作前**的旧位姿(修复前的行为);
  B 正题: _settle_arm 必须等到读数偏离旧值并稳定, 返回到位后的读回(轮询比推送快时
          也**不能**把同一个样本当成"两次一致" —— 那样会停在半路, B 的落点检查会抓到);
  C 兜底: 目标不可达(行程被钳制) → 超时告警 + 不把旧值伪装成到位值;
  D 编码: 无符号回读(负 mm)被还原成有符号。
"""
import os
import sys
import time
import tempfile
import textwrap

FAKE = textwrap.dedent('''
    import threading, time

    LAG_S = 0.5          # 动作开始到"位置推送"反映出来 的滞后(实测 ~0.5s)
    TRAVEL_S = 0.3       # 本段行程耗时(5mm 量级)
    PUSH_HZ = 5.0        # 位置推送周期 200ms
    X_DRIFT = 1          # 实测 x 从 74 漂到 75

    class _Action:
        def __init__(self): self.state, self.has_succeeded = 'action_idle', None
        def wait_for_completed(self, timeout=60):
            time.sleep(0.047)          # 实测 ACTION_ELAPSED_S = 0.047
            self.state, self.has_succeeded = 'action_succeeded', True
            return True

    class _Arm:
        """臂: 位置只能订阅推送(对齐官方 0.1.1.68 —— 它没有 get_position)。"""
        def __init__(self):
            self._true = [74.0, 57.0]   # 机械臂真实位置
            self._pushed = [74.0, 57.0] # 最近一次推送出去的值
            self._target = None
            self._t0 = None
            self._from = None
            self._lock = threading.Lock()
            self._stop = False
        def sub_position(self, freq=5, callback=None, *a, **kw):
            self._cb, self._freq = callback, float(freq)
            threading.Thread(target=self._pusher, daemon=True).start()
            return True
        def unsub_position(self):
            self._stop = True
            return True
        def moveto(self, x, y):
            # EP 会按行程把目标钳制在可达范围(垂直 ~0.15m); 如实建模, 否则测不出"被钳制"
            with self._lock:
                self._from = list(self._true)
                self._target = [min(max(float(x), 0.0), 220.0), min(max(float(y), 0.0), 150.0)]
                self._t0 = time.monotonic()
            return _Action()
        def inject_unsigned(self, x, y):
            """直接塞一个"无符号编码"的样本(负坐标在线上就是这个样子)。"""
            self._cb((int(x) & 0xFFFFFFFF, int(y) & 0xFFFFFFFF))
        def _pusher(self):
            while not self._stop:
                time.sleep(1.0 / self._freq)
                with self._lock:
                    t, target, t0, src = time.monotonic(), self._target, self._t0, self._from
                    if target is not None:
                        el = t - t0
                        if el <= LAG_S:
                            pass                              # 推送还没反映动作
                        elif el <= LAG_S + TRAVEL_S:
                            k = (el - LAG_S) / TRAVEL_S
                            self._true = [src[i] + (target[i] - src[i]) * k for i in (0, 1)]
                        else:
                            self._true = list(target)
                    # x 的 +1mm 是在动作之后出现的(实测 74->75), 动作之前不含它
                    v = [self._true[0] + (X_DRIFT if target is not None else 0), self._true[1]]
                    # 到位后 ±1mm 抖动(实测 61<->62)
                    if target is not None and self._true == target:
                        v[1] += 1 if int(t * self._freq) % 2 else 0
                    self._pushed = [round(v[0]), round(v[1])]
                # 线上是无符号: 负值补码到 2**32 那一段
                self._cb((int(self._pushed[0]) & 0xFFFFFFFF, int(self._pushed[1]) & 0xFFFFFFFF))

    class _Chassis:
        def sub_position(self, cs=0, freq=5, callback=None, *a, **kw):
            self._cb = callback
            threading.Thread(target=self._pusher, kwargs={'freq': float(freq)}, daemon=True).start()
            return True
        def unsub_position(self): return True
        def _pusher(self, freq=5.0):
            while True:
                time.sleep(1.0 / freq)
                self._cb((0.0, 0.0, 0.0))

    class _Gripper:
        def open(self, power=None): return _Action()
        def close(self, power=None): return _Action()

    class _Robot:
        def __init__(self):
            self.chassis = _Chassis()
            self.robotic_arm = _Arm()
            self.gripper = _Gripper()
        def initialize(self, conn_type=None, **kw): return True
        def close(self): return True

    def Robot():            # noqa: N802  —— 对齐官方 SDK 的工厂函数名
        return _Robot()
''')

tmp = tempfile.mkdtemp(prefix='fakerobo_')
os.makedirs(os.path.join(tmp, 'robomaster'))
with open(os.path.join(tmp, 'robomaster', '__init__.py'), 'w') as f:
    f.write('')
with open(os.path.join(tmp, 'robomaster', 'robot.py'), 'w') as f:
    f.write(FAKE)
sys.path.insert(0, tmp)

# 仓库根 = ep/tools/ 往上三层; 只用到 ep/drive/rm.py, 不需要连机器人
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'ep', 'drive'))
import rm as rmmod  # noqa: E402

CFG = {'robot': {'conn_type': 'ap'},
       'arm': {'attr_candidates': ['robotic_arm', 'arm'], 'grab_low': [74, 62],
               'lift_high': [74, 120]},
       'gripper': {'open_power': 60, 'close_power': 60, 'act_wait_s': 0.1},
       'chassis': {'xy_speed': 0.5, 'z_speed': 60.0, 'move_tol_m': 0.03, 'odom_cs': 1}}

LOG = []


class Log:
    def text(self, m):
        LOG.append(m)
        print('   |', m)

    def event(self, kind, **kw):
        pass


def new_rm():
    LOG.clear()
    r = rmmod.RM(CFG, Log())
    r.connect()
    return r


fails = []

print('\n== 0 读数通路: 位置来自订阅推送(不是不存在的 get_position) ==')
r = new_rm()
xy = r.read_arm_xy()
odom = r.read_chassis_pose()
print('   arm =', xy, ' odom =', odom)
if xy == [74, 57]:
    print('   ✓ 订阅首样本拿到了基线位姿')
else:
    fails.append('0: 订阅读数不对: {}'.format(xy))
if any('一个样本都没收到' in m or '推送数据无效' in m for m in LOG):
    fails.append('0: 读数报错(订阅没生效?)')

print('\n== A 反证: 紧跟 _finish 就读(修复前的行为) ==')
r = new_rm()
before = r.read_arm_xy()
rmmod._finish(r.arm.moveto(x=74, y=62))          # wait_for_completed 立刻返回
immediate = r.read_arm_xy()
print('   动作前 =', before, ' 立刻读 =', immediate)
if immediate == before:
    print('   ✓ 复现: 立刻读到的是动作【前】的旧值 —— 正是 993ba9b 修掉的坑')
else:
    fails.append('A: 没能复现滞后(假 SDK 时序不对?)')

print('\n== B 正题: _settle_arm ==')
r = new_rm()
before = r.read_arm_xy()
t0 = time.monotonic()
pos = r.arm_moveto(74, 62, tag='to_target')
dt = time.monotonic() - t0
print('   返回 =', pos, ' 用时 {:.2f}s'.format(dt))
ok_depart = pos != before
ok_near = abs(pos[0] - 74 - 1) <= 1.5 and abs(pos[1] - 62) <= 1.5   # 允许 x 漂移 1mm + ±1 抖动
ok_waited = dt >= 0.5
print('   ✓ 偏离旧值' if ok_depart else '   ✗ 仍等于旧值')
print('   ✓ 落在目标±容差(含实测 x+1mm 漂移; 停半路就会偏出这里)'
      if ok_near else '   ✗ 偏离目标太多: {}'.format(pos))
print('   ✓ 确实等了(>{:.1f}s)'.format(0.5) if ok_waited else '   ✗ 没等(滞后没被吃掉)')
for c, m in ((ok_depart, 'B: 返回旧值'), (ok_near, 'B: 未落到目标'), (ok_waited, 'B: 未等待滞后')):
    if not c:
        fails.append(m)

print('\n== C 兜底: 目标不可达(行程被钳制到 150) ==')
r = new_rm()
t0 = time.monotonic()
pos = r.arm_moveto(74, 999, tag='unreachable')    # 钳到 150, 永远到不了 999
dt = time.monotonic() - t0
warned = any('未到位' in m for m in LOG)
print('   返回 =', pos, ' 用时 {:.2f}s'.format(dt))
print('   ✓ "未到位"告警已打印' if warned else '   ✗ 没告警 —— 钳制被当成正常到位了')
print('   ✓ 返回的是钳制位置(读数本身可信, 且已标警)'
      if abs(pos[1] - 150) < 3 else '   ✗ 返回值不可信: {}'.format(pos))
for c, m in ((warned, 'C: 无告警'), (abs(pos[1] - 150) < 3, 'C: 返回值不可信')):
    if not c:
        fails.append(m)

print('\n== D 编码: 无符号回读还原为有符号 mm ==')
r = new_rm()
r.arm.unsub_position()                            # 先停推送, 免得注入的样本被下一帧盖掉
time.sleep(0.3)
r.arm.inject_unsigned(-6, 58)                     # 线上就是 4294967290
xy = r.read_arm_xy()
print('   线上 4294967290,58 -> 读回', xy)
if xy == [-6, 58]:
    print('   ✓ 补码还原正确(不做这步会把 4.29e9 当读数)')
else:
    fails.append('D: 无符号未还原: {}'.format(xy))
for raw, want in ((4294967290, -6), (74, 74), (2 ** 31, -(2 ** 31)), (2 ** 32 - 1, -1)):
    got = rmmod._signed_mm(raw)
    if got != want:
        fails.append('D: _signed_mm({}) = {} != {}'.format(raw, got, want))

print('\n' + ('RESULT: ALL_OK —— 四个场景都符合预期' if not fails
             else 'RESULT: FAIL —— ' + '; '.join(fails)))
sys.exit(1 if fails else 0)
