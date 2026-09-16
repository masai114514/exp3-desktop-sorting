# -*- coding: utf-8 -*-
"""--dry-run 用的假 EP：不连 SDK、不动真机，只把原语按顺序打出来 + 模拟到位读数。

用途是**现场演练**：点位/臂姿态/相机标定都填完之后，先空跑一遍，看动作链是不是想的那样
（开去哪个格、臂下探到多少、再去哪个料盒），确认无误再上真机。它也是离线单测的替身。

它**不是验收依据** —— run_real.py 会在 result.json 旁边写 real_run.json 标 dry_run=true，
run.log 里也打大字提醒。真机验收视频必须用真机跑出来的那一份。

fail_pred(method, call_index, args) -> bool：测试/演练时注入失败用（第几次调哪个原语炸）。
"""
import time


class DryRunRM:
    """满足 EPPickExecutor 要的最小原语面：goto / arm_moveto / gripper_open / gripper_close。

    行为上唯一"真"的地方是**状态**：odom 与臂读数会跟着指令走，所以真实链路里读回值的代码
    路径(比如 rm.goto 的误差纠正、arm_moveto 的到位判断)在这里也走得到 —— 这正是一次
    演练要验证的东西。RM 内部的等待(夹爪 act_wait_s 之类)在这里不真等。
    """

    def __init__(self, log=None, fail_pred=None, sleep=False):
        self.log = log
        self.fail_pred = fail_pred
        self.sleep = bool(sleep)
        self.calls = []                 # [(method, args)]，测试/演练后可以整体打印
        self._n = {}                    # method -> 调用次数
        self.odom = [0.0, 0.0, 0.0]
        self.arm_xy = [0, 0]
        self.grip = 'open'
        self.connected = False

    # ---------- 内部 ----------
    def _log_call(self, method, *a, **kw):
        n = self._n.get(method, 0) + 1
        self._n[method] = n
        self.calls.append((method, a))
        if self.log:
            extra = ' '.join('%s=%s' % (k, v) for k, v in kw.items() if k != 'tag')
            self.log.text('[DRY] %s(%s)%s' % (method, ', '.join(str(x) for x in a),
                                              (' ' + extra) if extra else ''))
        if self.fail_pred and self.fail_pred(method, n, a):
            raise RuntimeError('[DRY] 注入失败: %s 第 %d 次调用' % (method, n))

    def _wait(self, s):
        if self.sleep:
            time.sleep(s)

    # ---------- 原语（与 ep/drive/rm.py 的 RM 同签名） ----------
    def connect(self):
        self.connected = True
        if self.log:
            self.log.text('[DRY] connect()：演练模式，不连 EP')
        return self

    def goto(self, x, y, z=None):
        self._log_call('goto', x, y, z)
        self.odom = [float(x), float(y), float(z if z is not None else self.odom[2])]
        return list(self.odom)

    def arm_moveto(self, x_mm, y_mm, tag=''):
        self._log_call('arm_moveto', int(x_mm), int(y_mm), tag=tag)
        self.arm_xy = [int(x_mm), int(y_mm)]
        return list(self.arm_xy)

    def gripper_open(self):
        self._log_call('gripper_open')
        self.grip = 'open'

    def gripper_close(self):
        self._log_call('gripper_close')
        self.grip = 'closed'

    def read_chassis_pose(self, max_age=1.5):
        return list(self.odom)

    def read_arm_xy(self, max_age=1.0):
        return list(self.arm_xy)

    def disconnect(self):
        self.connected = False
        if self.log:
            self.log.text('[DRY] disconnect()：共 %d 次原语调用' % len(self.calls))

    def dump(self):
        """把调用序列打出来（演练末尾用；一眼扫完动作链对不对）。"""
        lines = ['[DRY] 动作链 %d 步:' % len(self.calls)]
        for i, (m, a) in enumerate(self.calls, 1):
            lines.append('  %2d. %s(%s)' % (i, m, ', '.join(str(x) for x in a)))
        return '\n'.join(lines)
