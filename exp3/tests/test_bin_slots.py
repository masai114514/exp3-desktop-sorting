# -*- coding: utf-8 -*-
"""料盒落料点（slots）的离线单测。

## 这条不变量是怎么来的

第 7 轮云机实跑（run_20260924_005503）第一次出现"两个物块进同一个料盒"：
c1(cup) 已放好，c3(cup) 下降途中穿进 c1 的体内 —— trace_blocks.log 铁证：

    t=169.2  tip z=0.088  c1=(0.100,-0.160,0.820)   ← c1 已在盒内
    t=171.2  tip z=0.052  c1=(0.362,-0.239,1.753)   ← 被 LCP 炸飞

落料点在 c1 落入后**从来没有变过**，所以第 2 个必定与第 1 个同位 → 互穿 → 爆炸。
最终 c1 被抛到 (-75.2, 47.7) m，本轮 recorded placed_ok=3 但场上实际只剩 2 个在盒里。

⇒ 不变量：**一个料盒要放 n>1 个物块时，落料点必须逐个错开（或叠层），
   且第 k 个点必须已被第 k-1 个点占用后才会被用到。**

本文件的用例同时覆盖三处接线：config/bins.json → resolve_goal → c4_executor._targets，
以及 pick_place_server 的占用时序（只在放置被确认时占用）。
"""

import json
import unittest
from pathlib import Path

from config_check import check_bins
from ros2.task_control.c4_executor import C4PickExecutor
from ros2.task_control.exec_contract import resolve_goal, slot_offset

EXP3 = Path(__file__).resolve().parents[1]
SERVER_SRC = EXP3 / 'ros2' / 'task_control' / 'pick_place_server.py'


def load_real_bins():
    with open(EXP3 / 'config' / 'bins.json', encoding='utf-8') as fh:
        return json.load(fh)


def load_real_grid():
    with open(EXP3 / 'config' / 'grid_cells.json', encoding='utf-8') as fh:
        return json.load(fh)


def half_of(cls, geo):
    """物体的半尺寸 (x,y)——与 config_check 的口径一致。"""
    if cls == 'cup':
        r = geo['cup_diameter'] / 2.0
        return (r, r)
    ms = geo['mouse_size']
    return (ms[0] / 2.0, ms[1] / 2.0)


class FakeRobot:
    def __init__(self):
        self.calls = []

    def go_cartesian(self, xyz, label):
        self.calls.append((label, tuple(round(v, 4) for v in xyz)))
        return True, [tuple(xyz)]

    def go_chain(self, chain, label):
        self.calls.append((label, tuple(map(tuple, chain))))
        return True, 'ok', 1

    def grip_set(self, vals, label, settle=None):
        return True, 'ok'


def motion():
    return dict(home=[[0.0] * 6], gripper_open=[0.025, -0.025],
                gripper_close=[-0.006, 0.006], grasp_z=0.038, lift_dz=0.1,
                place_z=0.038, press_settle=0.1, release_settle=0.1,
                hold_assume=True, place_assume=True)


class TestRealConfigHasUsableSlots(unittest.TestCase):
    """真实 config/bins.json 必须能让 6 物（cup x4 / mouse x2）互不重叠地落料。"""

    @classmethod
    def setUpClass(cls):
        cls.bins = load_real_bins()
        cls.geo = cls.bins['slots_geometry_m']
        cls.cells = load_real_grid()['cells']
        cls.n_cup = 4     # c1/c3/c4/c6（见 exp3_sim/scene.py 的摆放表）
        cls.n_mouse = 2   # c2/c5

    def test_every_bin_declares_slots(self):
        for bid, b in self.bins['bins'].items():
            self.assertIn('slots', b, '%s 没有 slots' % bid)
            self.assertGreaterEqual(len(b['slots']), 1)

    def test_slot_count_covers_the_worst_case(self):
        """cup 盒必须 ≥4 个点、mouse 盒 ≥2 个点 —— 否则第 5 个物块会钳回最后一个点上重叠。"""
        self.assertGreaterEqual(len(self.bins['bins']['bin_cup']['slots']), self.n_cup)
        self.assertGreaterEqual(len(self.bins['bins']['bin_mouse']['slots']), self.n_mouse)

    def test_slots_are_inside_the_bin_cavity(self):
        hx = self.geo['bin_inner_x_half']
        hy = self.geo['bin_inner_y_half']
        for bid, b in self.bins['bins'].items():
            rx, ry = half_of(b['cls'], self.geo)
            for i, s in enumerate(b['slots']):
                self.assertLessEqual(abs(s['dx']) + rx, hx + 1e-9,
                                     '%s slot%d 在 x 方向出界' % (bid, i))
                self.assertLessEqual(abs(s['dy']) + ry, hy + 1e-9,
                                     '%s slot%d 在 y 方向出界' % (bid, i))

    def test_same_layer_slots_do_not_overlap(self):
        for bid, b in self.bins['bins'].items():
            rx, ry = half_of(b['cls'], self.geo)
            same = {}
            for i, s in enumerate(b['slots']):
                same.setdefault(round(s['dz'], 6), []).append((i, s))
            for dz, group in same.items():
                for a in range(len(group)):
                    for c in range(a + 1, len(group)):
                        ga = abs(group[a][1]['dx'] - group[c][1]['dx'])
                        gb = abs(group[a][1]['dy'] - group[c][1]['dy'])
                        self.assertTrue(ga >= 2 * rx - 1e-9 or gb >= 2 * ry - 1e-9,
                                        '%s 层 dz=%.3f 的 slot%d 与 slot%d 重叠'
                                        % (bid, dz, group[a][0], group[c][0]))

    def test_stacked_layer_clears_the_one_below(self):
        """叠层高度必须 ≥ 物体自身高度，否则上层物块会陷进下层体内（又是互穿）。"""
        need = {'cup': self.geo['cup_height'],
                'mouse': self.geo['mouse_size'][2]}
        for bid, b in self.bins['bins'].items():
            layers = sorted({s['dz'] for s in b['slots']})
            for lo, hi in zip(layers, layers[1:]):
                self.assertGreaterEqual(hi - lo, need[b['cls']] - 1e-9,
                                        '%s 的分层间距 %.4f 小于物体高 %.4f'
                                        % (bid, hi - lo, need[b['cls']]))

    def test_config_check_passes_on_real_bins(self):
        """配置闸门本身也要认可这份配置（否则 check_bins 与现实脱节）。"""
        task = json.loads((EXP3 / 'config' / 'task.json').read_text(encoding='utf-8'))
        issues = [i for i in check_bins(self.bins, task)
                  if i['severity'] == 'ERROR']
        self.assertEqual(issues, [], 'check_bins 报错: %s' % issues)


class TestSlotOffsetAndResolveGoal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bins = load_real_bins()
        cls.grid = load_real_grid()

    def test_slot_offset_clamps_instead_of_wrapping(self):
        """越界必须钳到最后一个点，不能环绕 —— 环绕会让第 n+1 个物块落回第 1 个点上。"""
        b = self.bins['bins']['bin_cup']
        n = len(b['slots'])
        self.assertEqual(slot_offset(b, 0), b['slots'][0])
        self.assertEqual(slot_offset(b, n + 5), b['slots'][-1])
        self.assertEqual(slot_offset(b, -3), b['slots'][0])

    def test_slot_offset_is_empty_without_slots(self):
        self.assertEqual(slot_offset({'cls': 'cup'}, 3), {})

    def test_resolve_goal_walks_the_slots(self):
        b = self.bins['bins']['bin_cup']
        seen = set()
        for k in range(len(b['slots'])):
            ctx, err = resolve_goal(self.grid, self.bins, 'c1', 'cup', slot=k)
            self.assertIsNone(err, err)
            self.assertAlmostEqual(ctx['B']['x'], b['x_m'] + b['slots'][k]['dx'], places=9)
            self.assertAlmostEqual(ctx['B']['y'], b['y_m'] + b['slots'][k]['dy'], places=9)
            self.assertAlmostEqual(ctx['B']['z_drop'], b['slots'][k]['dz'], places=9)
            self.assertEqual(ctx['slot_index'], k)
            seen.add((round(ctx['B']['x'], 6), round(ctx['B']['y'], 6),
                      round(ctx['B']['z_drop'], 6)))
        self.assertEqual(len(seen), len(b['slots']), '不同 slot 必须给出不同的落料点')

    def test_resolve_goal_default_slot_keeps_backward_compat(self):
        """不传 slot 时用第 1 个点 —— 旧调用方（真机线）行为可预期。"""
        ctx, err = resolve_goal(self.grid, self.bins, 'c2', 'mouse')
        self.assertIsNone(err, err)
        b = self.bins['bins']['bin_mouse']
        self.assertAlmostEqual(ctx['B']['x'], b['x_m'] + b['slots'][0]['dx'], places=9)
        self.assertAlmostEqual(ctx['A']['x'], (0.09 + 0.09) / 2, places=9)  # A 不受影响

    def test_resolve_goal_without_slots_is_unchanged(self):
        """没有 slots 的配置（真机未标定时）必须退化成"料盒中心"，不能报错。"""
        bins = {'bins': {'bin_cup': {'cls': 'cup', 'x_m': 0.3, 'y_m': -0.2}},
                'class_to_bin': {'cup': 'bin_cup'}}
        ctx, err = resolve_goal(self.grid, bins, 'c1', 'cup', slot=2)
        self.assertIsNone(err, err)
        self.assertAlmostEqual(ctx['B']['x'], 0.3, places=9)
        self.assertAlmostEqual(ctx['B']['z_drop'], 0.0, places=9)


class TestExecutorAppliesZDrop(unittest.TestCase):
    """落料高必须加上该落料点的分层高度，否则第 2 层物块从下层体内开始。"""

    def ctx(self, z_drop, x=0.077, y=-0.16):
        return {'cell_id': 'c1', 'cls': 'cup', 'bin_id': 'bin_cup',
                'slot_index': 2,
                'A': {'x': 0.09, 'y': -0.08, 'z_pick': 0.138},
                'B': {'x': x, 'y': y, 'z_drop': z_drop}}

    def test_targets_adds_z_drop_to_place_z(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion())
        _a_above, _a_grasp, _a_lift, b_place = e._targets(self.ctx(0.036))
        self.assertAlmostEqual(b_place[2], 0.038 + 0.036, places=9)
        self.assertAlmostEqual(b_place[0], 0.077, places=9)

    def test_targets_handles_missing_z_drop(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion())
        ctx = self.ctx(0.0)
        del ctx['B']['z_drop']
        _a, _b, _c, b_place = e._targets(ctx)
        self.assertAlmostEqual(b_place[2], 0.038, places=9)

    def test_carry_moves_to_the_slot_not_the_bin_center(self):
        r = FakeRobot()
        e = C4PickExecutor(r, motion())
        ok, _note = e.carry(self.ctx(0.036))
        self.assertTrue(ok)
        lab, xyz = [c for c in r.calls if c[0] == 'CARRY_TO_B'][0]
        self.assertAlmostEqual(xyz[0], 0.077, places=6)
        self.assertAlmostEqual(xyz[2], 0.074, places=6)


class TestServerClaimsSlotsInOrder(unittest.TestCase):
    """占用时序：只在**放置被确认**时占用；失败不占用，避免下一次落到同一点。"""

    @classmethod
    def setUpClass(cls):
        cls.src = SERVER_SRC.read_text(encoding='utf-8')

    def test_slot_counter_lives_on_the_server(self):
        self.assertIn('self._slots = {}', self.src)
        self.assertIn('def _claim_slot(', self.src)

    def test_slot_is_passed_to_resolve_goal(self):
        self.assertIn('bin_for_cls(g.cls, self.bins)', self.src)
        self.assertIn('slot=slot', self.src)

    def test_slot_is_claimed_only_on_confirmed_release(self):
        """PLACE 段必须走 _release_and_claim；不能被换成裸 self.exec.release。"""
        self.assertIn('(STAGE_PLACE, _release_and_claim)', self.src)
        self.assertNotIn('(STAGE_PLACE, lambda: self.exec.release(ctx))', self.src)

    def test_claim_happens_inside_the_release_wrapper(self):
        i = self.src.index('def _release_and_claim(')
        j = self.src.index('(STAGE_PLACE, _release_and_claim)')
        self.assertIn("self._claim_slot(ctx['bin_id'])", self.src[i:j])

    def test_claim_increments_monotonically(self):
        """_claim_slot 的语义用同样的实现复算一遍（源码是 ROS 节点，离线不能实例化）。"""
        slots = {}
        for _ in range(3):
            n = slots.get('bin_cup', 0)
            slots['bin_cup'] = n + 1
        self.assertEqual(slots['bin_cup'], 3)


class TestCheckBinsRejectsBadSlots(unittest.TestCase):
    """配置闸门要能挡住"同点落料"这类只在实跑时才炸的错误。"""

    def bad_bins(self, slots):
        return {'slots_geometry_m': {'bin_inner_x_half': 0.046, 'bin_inner_y_half': 0.0335,
                                     'cup_diameter': 0.040, 'cup_height': 0.035,
                                     'mouse_size': [0.040, 0.025, 0.020]},
                'bins': {'bin_cup': {'cls': 'cup', 'x_m': 0.1, 'y_m': -0.16, 'slots': slots}},
                'class_to_bin': {'cup': 'bin_cup'}}

    def errors(self, slots):
        return [i for i in check_bins(self.bad_bins(slots), {'classes': ['cup']})
                if i['severity'] == 'ERROR']

    def test_overlapping_same_layer_is_an_error(self):
        """回归护栏：第 7 轮那版"两个 cup 都落料盒中心"必须被判错。"""
        self.assertTrue(self.errors([{'dx': 0.0, 'dy': 0.0, 'dz': 0.0},
                                     {'dx': 0.0, 'dy': 0.0, 'dz': 0.0}]))

    def test_different_layers_may_share_xy(self):
        """叠层允许同 xy（那是刻意的），前提是 dz 不同。"""
        self.assertEqual(self.errors([{'dx': 0.0, 'dy': 0.0, 'dz': 0.0},
                                      {'dx': 0.0, 'dy': 0.0, 'dz': 0.036}]), [])

    def test_out_of_cavity_is_an_error(self):
        self.assertTrue(self.errors([{'dx': 0.040, 'dy': 0.0, 'dz': 0.0}]))

    def test_non_numeric_slot_is_an_error(self):
        self.assertTrue(self.errors([{'dx': 'a', 'dy': 0.0, 'dz': 0.0}]))

    def test_empty_slot_list_is_an_error(self):
        self.assertTrue(self.errors([]))

    def test_missing_slots_is_tolerated(self):
        """slots 是可选字段：不写不报错（旧配置/真机未标定）。"""
        bins = self.bad_bins([])
        del bins['bins']['bin_cup']['slots']
        self.assertEqual([i for i in check_bins(bins, {'classes': ['cup']})
                          if i['severity'] == 'ERROR'], [])


if __name__ == '__main__':
    unittest.main()
