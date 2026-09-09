# -*- coding: utf-8 -*-
"""C4PickExecutor 的离线单测：假 robot 验证动作时序/失败传播 + resolve_goal 契约。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
不需要 ROS —— executor/契约层是无 ROS import 的（c4_executor / exec_contract）。
"""
import unittest

from ros2.task_control.exec_contract import resolve_goal
from ros2.task_control.c4_executor import C4PickExecutor

# 一个可复用的 ctx（resolve_goal 的结果形状）
CTX = {'cell_id': 'c1', 'cls': 'cup', 'bin_id': 'bin_cup',
       'A': {'x': 0.09, 'y': -0.09, 'z_pick': 0.138},
       'B': {'x': 0.04, 'y': 0.2}}


def motion(**over):
    m = dict(
        home=[0.0, -1.571, 0.0, -1.571, 1.571, -1.571],
        gripper_open=[0.025, -0.025], gripper_close=[-0.006, 0.006],
        grasp_z=0.038, lift_dz=0.1, place_z=0.038,
        press_settle=4.0, release_settle=3.0,
        hold_assume=True, place_assume=True,
    )
    m.update(over)
    return m


class FakeRobot:
    """记录所有运动调用的假 robot；fail_label 指定哪个 label 直接失败。"""

    def __init__(self):
        self.calls = []          # [(label, kind, arg)]
        self.fail_label = None

    def _ok(self, label, kind, arg):
        self.calls.append((label, kind, arg))
        if label == self.fail_label:
            return False                       # 失败分支也要返回可解包形状，见各方法
        return True

    def go_cartesian(self, xyz, label):
        return (self._ok(label, 'cart', tuple(round(v, 3) for v in xyz)),
                [tuple(xyz)] if self.fail_label != label else None)

    def go_chain(self, chain, label):
        ok = self._ok(label, 'chain', tuple(chain))
        return ok, 'ok' if ok else None, 4

    def grip_set(self, vals, label, settle=None):
        return self._ok(label, 'grip', (tuple(vals), settle)), 'ok'


class TestFullPickPlaceSequence(unittest.TestCase):
    def test_happy_sequence_order(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion())
        ctx = CTX
        for stage, expect_ok in (('descend', True), ('grasp', True), ('lift', True),
                                 ('carry', True), ('release', True), ('retract', True)):
            ok, note = getattr(e, stage)(ctx)
            self.assertTrue(ok, '%s failed: %s' % (stage, note))
        labels = [c[0] for c in r.calls]
        # 预期运动次序：开爪→到格上方→下探→压紧→抬→(判held)→搬→放→(判placed)→退回→回零
        idx = {lab: labels.index(lab) for lab in
               ('OPEN@descend', 'TO_ABOVE_A', 'DOWN_TO_A', 'PRESS', 'LIFT_A',
                'CARRY_TO_B', 'RELEASE', 'RISE_AWAY', 'HOME')}
        self.assertTrue(idx['OPEN@descend'] < idx['TO_ABOVE_A'] < idx['DOWN_TO_A']
                        < idx['PRESS'] < idx['LIFT_A'] < idx['CARRY_TO_B']
                        < idx['RELEASE'] < idx['RISE_AWAY'] < idx['HOME'])
        # 到格/下探用格中心，搬运落点用料盒中心
        down = [c for c in r.calls if c[0] == 'DOWN_TO_A'][0][2]
        self.assertEqual(down[0], round(CTX['A']['x'], 3))
        self.assertEqual(down[1], round(CTX['A']['y'], 3))
        carry = [c for c in r.calls if c[0] == 'CARRY_TO_B'][0][2]
        self.assertEqual(carry[0], round(CTX['B']['x'], 3))


class TestFailurePropagation(unittest.TestCase):
    def test_descend_fails_when_grip_open_fails(self):
        r = FakeRobot(); r.fail_label = 'OPEN@descend'
        ok, note = C4PickExecutor(r, motion()).descend(CTX)
        self.assertFalse(ok)
        self.assertIn('开爪', note)

    def test_carry_fails_on_plan_error(self):
        r = FakeRobot(); r.fail_label = 'CARRY_TO_B'
        ok, _ = C4PickExecutor(r, motion()).carry(CTX)
        self.assertFalse(ok)

    def test_lift_returns_false_when_not_held(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion(hold_assume=False))
        ok, note = e.lift(CTX)          # 判据假但 assume 关 → 判定不过
        self.assertFalse(ok)
        self.assertIn('未夹起', note)

    def test_lift_retries_press_before_giving_up(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion(hold_assume=False, lift_retry=1))
        e.lift(CTX)
        labels = [c[0] for c in r.calls]
        self.assertEqual(labels.count('LIFT_A'), 2)          # 抬→判败→重压→再抬→判败
        self.assertEqual(labels.count('re-PRESS'), 1)

    def test_release_false_when_place_judge_fails(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion(place_assume=False))
        ok, _ = e.release(CTX)
        self.assertFalse(ok)


class TestResolveGoal(unittest.TestCase):
    GRID = {'cells': [{'id': 'c1', 'row': 0, 'col': 0,
                       'x0': 0.06, 'y0': -0.135, 'x1': 0.12, 'y1': -0.045}],
            'pick_z_m': 0.138}
    BINS = {'bins': {'bin_cup': {'cls': 'cup', 'x_m': 0.04, 'y_m': 0.2}},
            'class_to_bin': {'cup': 'bin_cup'}}

    def test_ok(self):
        ctx, err = resolve_goal(self.GRID, self.BINS, 'c1', 'cup')
        self.assertIsNone(err)
        self.assertEqual(ctx['cell_id'], 'c1')
        self.assertEqual(ctx['A']['z_pick'], 0.138)
        self.assertEqual(ctx['B']['x'], 0.04)

    def test_unknown_cell(self):
        ctx, err = resolve_goal(self.GRID, self.BINS, 'c99', 'cup')
        self.assertIsNone(ctx)
        self.assertIn('cell_id', err)

    def test_cls_without_bin(self):
        ctx, err = resolve_goal(self.GRID, self.BINS, 'c1', 'stapler')
        self.assertIsNone(ctx)
        self.assertIn('stapler', err)


if __name__ == '__main__':
    unittest.main()
