# -*- coding: utf-8 -*-
"""仿真场景摆放表 + 异常轮空格的离线单测。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
不需要 ROS —— exp3_sim/scene.py 与 sort_core 都是无 ROS import 的。

本文件锁住三条【判据】，它们都是在云机上被反复问到、但很容易被后人改坏的东西：

  1. 摆放表与 `_object_sdf()` 的几何**必须同步**：object_z 就是物块半高，
     改了几何没改这里会让物块悬空或陷进桌面（云机上踩过）。
     这里用**源码文本解析**核对 —— pick_place_server.py import rclpy，离线不能执行它。
  2. 异常轮的空格必须是**场景事实**（那一格真的不 spawn），不是配置断言；
     因为任务书 §六「空网格应被跳过」的证据形式是"那一格没有任何记录"。
  3. 异常轮的 verdict **必然是 TRIAL**，这是"样本不足不能判定"的正确语义，
     所以必须与记分轮分开跑。这条用 verdict_for() 直接锁住，防止有人当成 bug 去"修"。
"""
import re
import unittest
from pathlib import Path

from ros2.exp3_sim.exp3_sim.scene import (CARRY_Z_DEFAULT, CARRY_Z_FRACTIONS,
                                         CARRY_Z_IK_CEILING, CARRY_Z_SAFETY_MARGIN,
                                         CELL_IDS, NOMINAL_CLS,
                                         SCENE_OBJECTS, carry_z_candidates,
                                         scene_objects)
from sort_core.logging_util import verdict_for
from sort_core.taxonomy import (VERDICT_FAIL, VERDICT_PASS, VERDICT_RUNNING,
                                VERDICT_TRIAL)

_SERVER_SRC = (Path(__file__).resolve().parent.parent
               / 'ros2' / 'exp3_sim' / 'exp3_sim' / 'pick_place_server.py')


class TestSceneTable(unittest.TestCase):
    def test_six_cells_by_default(self):
        objs = scene_objects()
        self.assertEqual(len(objs), 6)
        self.assertEqual([o[0].split('_')[1] for o in objs], list(CELL_IDS))

    def test_two_classes_only(self):
        """任务书 §六 要求"自动分类 >=2 类"，本场景用的正是 cup / mouse 两类。"""
        self.assertEqual(sorted({o[1] for o in SCENE_OBJECTS}), ['cup', 'mouse'])

    def test_nominal_map_covers_all_cells(self):
        self.assertEqual(set(NOMINAL_CLS), set(CELL_IDS))

    def test_grid_is_two_by_three(self):
        """2x3 矩形：x 两列、y 三行，与 c4_2/config_sim.json 的格子中心一致。"""
        self.assertEqual(len({o[2] for o in SCENE_OBJECTS}), 2)
        self.assertEqual(len({o[3] for o in SCENE_OBJECTS}), 3)

    def test_objects_do_not_overlap(self):
        """同格不重物：模型名唯一。"""
        names = [o[0] for o in SCENE_OBJECTS]
        self.assertEqual(len(names), len(set(names)))


class TestAnomalyRoundEmptyCell(unittest.TestCase):
    def test_empty_cell_removes_exactly_that_cell(self):
        for cell in CELL_IDS:
            with self.subTest(cell=cell):
                objs = scene_objects(cell)
                self.assertEqual(len(objs), 5)
                left = [o[0].split('_')[1] for o in objs]
                self.assertNotIn(cell, left)
                self.assertEqual(left, [c for c in CELL_IDS if c != cell])

    def test_empty_cell_keeps_positions_of_the_rest(self):
        """空格不得让邻格挪位 —— 否则"空格"变成了"换场景"，证据就不等价了。"""
        full = {o[0]: (o[2], o[3], o[4]) for o in SCENE_OBJECTS}
        for cell in CELL_IDS:
            for obj in scene_objects(cell):
                with self.subTest(cell=cell, obj=obj[0]):
                    self.assertEqual((obj[2], obj[3], obj[4]), full[obj[0]])

    def test_empty_and_none_are_both_normal_round(self):
        for value in ('', None, '   '):
            with self.subTest(value=value):
                self.assertEqual(len(scene_objects(value)), 6)

    def test_invalid_cell_rejected(self):
        for bad in ('c0', 'c7', 'c3 ', 'C3', 'block_c3', 'cup'):
            with self.subTest(bad=bad):
                if bad.strip() in CELL_IDS:
                    continue
                with self.assertRaises(ValueError):
                    scene_objects(bad)

    def test_whitespace_is_tolerated_for_valid_cell(self):
        """launch 参数从命令行来，容易带空格；' c3 ' 应当等价于 'c3'。"""
        self.assertEqual([o[0] for o in scene_objects(' c3 ')],
                         [o[0] for o in scene_objects('c3')])


class TestVerdictSemanticsForAnomalyRound(unittest.TestCase):
    """锁住"异常轮必然 TRIAL" —— 这是语义，不是缺陷，不要去"修"。"""

    def test_five_objects_on_table_yields_trial(self):
        # 场上 5 个物块 ⇒ 首轮成功映射进网格的格数 objects_seen=5 < required_total=6
        self.assertEqual(verdict_for(5, 5, 6, 5, True), VERDICT_TRIAL)

    def test_trial_is_not_a_failure(self):
        self.assertNotEqual(verdict_for(5, 5, 6, 5, True), VERDICT_FAIL)

    def test_full_round_can_pass(self):
        self.assertEqual(verdict_for(6, 6, 6, 5, True), VERDICT_PASS)

    def test_full_round_with_too_few_placed_fails(self):
        self.assertEqual(verdict_for(4, 6, 6, 5, True), VERDICT_FAIL)

    def test_running_before_final(self):
        self.assertEqual(verdict_for(3, 6, 6, 5, False), VERDICT_RUNNING)


class TestCarryZLadder(unittest.TestCase):
    """★ 锁住 carry_z 阶梯。这条护栏记录的是**两次云机实跑换来的教训**。

    教训一（第 5 轮）：把倍率整体抬到 (0.87, 0.74) ⇒ 绝对 (0.125, 0.112)，
      0.125 对 6/6 格不可达，cup 那 4 格会"抬到料盒上方失败"。

    教训二（第 6 轮，run_20260924_004129）：保留 0.112 一档（作为最高候选）
      **仍然是错的**，因为它在 IK 边界上：
        - mouse 格搬运腿确实能到 0.112，但回程腿 RISE_AWAY 从那里出发必失败
          （实测 LIMIT q5=2.176，c2 放置成功后倒在回程）；
        - 更狠的是跟踪过冲（瞄准 0.112、到位 0.113）会把 q0 顶出可行域，
          于是**下一步规划从非法起点出发**而必然失败（实测 CARRY_TO_B q5=2.369）。
      ⇒ 不变量：**任何一档都必须离该料盒的实测边界至少 CARRY_Z_SAFETY_MARGIN**。
        在边界上取点，成功是偶然、失败是必然。
    """

    GRASP_Z = 0.038
    LIFT_DZ = 0.10

    @classmethod
    def setUpClass(cls):
        cls.src = _SERVER_SRC.read_text(encoding='utf-8')

    def ladder(self, bin_id=None):
        return carry_z_candidates(self.GRASP_Z, self.LIFT_DZ, bin_id)

    def test_ladder_is_descending(self):
        """必须从高到低：_go_above_bin 取第一个可达的，高的一档净空更大。"""
        for bid in (None,) + tuple(CARRY_Z_IK_CEILING):
            zs = self.ladder(bid)
            self.assertEqual(zs, sorted(zs, reverse=True))

    def test_every_rung_keeps_margin_below_its_bin_ceiling(self):
        """★ 核心不变量：每一档都要离该料盒的 IK 边界留出余量。"""
        for bid, ceiling in CARRY_Z_IK_CEILING.items():
            limit = ceiling - CARRY_Z_SAFETY_MARGIN
            for z in self.ladder(bid):
                self.assertLessEqual(
                    z, limit + 1e-9,
                    '%s: carry_z=%.4f 离边界 %.3f 不足余量 %.3f —— '
                    '跟踪过冲会把 q0 顶出可行域，下一步规划必失败'
                    % (bid, z, ceiling, CARRY_Z_SAFETY_MARGIN))

    def test_top_rung_is_the_documented_default(self):
        """第一档 = CARRY_Z_DEFAULT = 0.088（第 4 轮云机实跑验证过的取值）。"""
        self.assertAlmostEqual(self.ladder()[0], CARRY_Z_DEFAULT, places=9)
        self.assertAlmostEqual(CARRY_Z_DEFAULT, 0.088, places=9)

    def test_regression_rejected_ladder_round5(self):
        """回归护栏：第 5 轮那个 [0.125, 0.112] 版本必须已经被否决。"""
        bad = [self.GRASP_Z + self.LIFT_DZ * f for f in (0.87, 0.74)]
        self.assertNotEqual(self.ladder(), bad)
        for z in bad:
            self.assertGreater(z, CARRY_Z_IK_CEILING['bin_cup'] - CARRY_Z_SAFETY_MARGIN)

    def test_regression_rejected_ladder_round6(self):
        """回归护栏：**边界值本身**不能再出现在阶梯里（0.112 / 0.096 都算）。

        这是第 6 轮实跑买来的教训，比第 5 轮那条更严 —— 当时 0.096 看着"能跑"，
        但它正好等于 cup 格边界，一旦跟踪过冲就变成 round3 那种
        "从非法起点规划"的必然失败。
        """
        zs = [round(z, 4) for z in self.ladder()]
        for boundary in (CARRY_Z_IK_CEILING['bin_cup'], CARRY_Z_IK_CEILING['bin_mouse']):
            self.assertNotIn(boundary, zs,
                             'carry_z=%.3f 是实测 IK 边界值，不能作为目标（脆）' % boundary)

    def test_bottom_rung_matches_round4_known_good(self):
        """保底档必须留下第 4 轮实测能跑的值 0.073，否则失去回退能力。"""
        zs = [round(z, 4) for z in self.ladder()]
        self.assertIn(0.088, zs)
        self.assertIn(0.073, zs)

    def test_top_rung_still_clears_cup_tops_while_carrying(self):
        """下界：第一档仍要让**携带中的物块底面**高过 cup 顶面，否则搬运途中会撞邻格。

        物块挂在刀尖下方：mouse 0.028、cup 0.020（取更吃亏的 0.028）。
        cup 顶面相对桌面 = 0.035 ⇒ 相对 base 0.0345（桌面即 base_z）。
        """
        table_top, cup_h, mouse_drop = 0.0, 0.0345, 0.028
        clearance = self.ladder()[0] - mouse_drop - cup_h
        self.assertGreater(clearance, 0.020,
                           '第一档 %.3f 携带 mouse 时对 cup 顶面只剩 %.3f m 净空'
                           % (self.ladder()[0], clearance))

    def test_ceiling_covers_both_bins(self):
        """两个料盒都要有实测边界；mouse 的上限高于 cup（差异来自 y 方向）。"""
        self.assertIn('bin_cup', CARRY_Z_IK_CEILING)
        self.assertIn('bin_mouse', CARRY_Z_IK_CEILING)
        self.assertGreater(CARRY_Z_IK_CEILING['bin_mouse'],
                           CARRY_Z_IK_CEILING['bin_cup'])

    def test_server_uses_the_shared_ladder_and_passes_bin_id(self):
        """服务端必须复用共享阶梯，并把 bin_id 传下去（否则按 bin 封顶失效）。"""
        self.assertIn('carry_z_candidates(', self.src)
        self.assertIn("_go_above_bin(B_place, 'CARRY_ABOVE_BIN', ctx.get('bin_id'))",
                      self.src)
        self.assertIn("_go_above_bin(B_place, 'RISE_ABOVE_BIN', ctx.get('bin_id'))",
                      self.src)


class TestObjectZMatchesCollisionGeometry(unittest.TestCase):
    """object_z 必须等于 _object_sdf() 里几何的半高 —— 源码文本解析核对（不能在离线执行它）。"""

    @classmethod
    def setUpClass(cls):
        cls.src = _SERVER_SRC.read_text(encoding='utf-8')

    def _half_height(self, cls_name):
        if cls_name == 'cup':
            m = re.search(r'<cylinder><radius>([\d.]+)</radius><length>([\d.]+)</length></cylinder>',
                          self.src)
            self.assertIsNotNone(m, 'cup 的 cylinder 几何没找到（_object_sdf 被改过了？）')
            return float(m.group(2)) / 2.0
        m = re.search(r'<box><size>([\d.]+) ([\d.]+) ([\d.]+)</size></box>', self.src)
        self.assertIsNotNone(m, 'mouse 的 box 几何没找到（_object_sdf 被改过了？）')
        return float(m.group(3)) / 2.0

    def test_object_z_is_half_height(self):
        for name, cls_name, _x, _y, object_z in SCENE_OBJECTS:
            with self.subTest(obj=name):
                self.assertAlmostEqual(object_z, self._half_height(cls_name), places=6)

    def test_cup_is_taller_than_mouse(self):
        """cup 顶面 0.8345 高于料盒壁顶 0.830 ⇒ carry 的下界约束来自 cup。"""
        cup_top = max(o[4] for o in SCENE_OBJECTS if o[1] == 'cup') * 2 + 0.80
        self.assertGreater(cup_top, 0.83)

    def test_source_still_declares_empty_cell_parameter(self):
        self.assertIn("declare_parameter('empty_cell'", self.src)
        self.assertIn('scene_objects(empty_cell)', self.src)


if __name__ == '__main__':
    unittest.main()
