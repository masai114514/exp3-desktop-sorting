# -*- coding: utf-8 -*-
"""EPPickExecutor / EPPickPlace 的离线断言（假 EP，不连 SDK）。

重点不是"能跑通"（那是 test_run_real 的活），而是**失败路径分得对**：
EP 是开环平台，失败最贵的不是抓空，而是抓空之后系统还以为自己知道机械臂在哪。
所以这里逐段注入失败，确认 (a) 段号与 reason 的对应、(b) 每次失败都收了尾。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
"""
import unittest

from real.dry_run import DryRunRM
from real.ep_backend import (EPPickExecutor, EPPickPlace, STAGE_ORDER, STAGE_REASON)
from ros2.task_control.exec_contract import PickExecutor
from sort_core.task import OK
from sort_core.taxonomy import (REASON_DROPPED, REASON_EXEC_ERROR, REASON_GRASP_FAILED,
                                REASON_UNREACHABLE, STAGE_APPROACH, STAGE_GRASP, STAGE_LIFT,
                                STAGE_MOVE_TO_BIN, STAGE_PLACE, STAGE_RETRACT)


def grid_fixture():
    return {'cells': [
        {'id': 'c1', 'row': 0, 'col': 0, 'x0': 0.06, 'y0': -0.045, 'x1': 0.12, 'y1': 0.045},
        {'id': 'c2', 'row': 0, 'col': 1, 'x0': 0.06, 'y0': 0.045, 'x1': 0.12, 'y1': 0.135},
    ], 'pick_z_m': 0.138, 'object_z_m': 0.025}


def bins_fixture():
    return {'bins': {'bin_cup': {'cls': 'cup', 'x_m': 0.04, 'y_m': 0.2},
                     'bin_mouse': {'cls': 'mouse', 'x_m': 0.18, 'y_m': 0.2}},
            'class_to_bin': {'cup': 'bin_cup', 'mouse': 'bin_mouse'}}


def ep_fixture(judge='assume_ok'):
    """演练用假标定值：只要求互不相同、量级合理（真值靠现场 odom/jog 读）。

    wait 全给 0 —— 真机那些 1.0/1.5s 的等待是为了让人和设备稳下来，单测里只是拖时间。
    """
    return {
        'chassis': {'home_pose': [0.0, 0.0, 0.0], 'move_tol_m': 0.03},
        'arm': {'grab_low_mm': [70, 60], 'lift_high_mm': [70, 150],
                'release_low_mm': None, 'lift_wait_s': 0.0, 'drop_wait_s': 0.0},
        'gripper': {'open_power': 60, 'close_power': 60, 'act_wait_s': 0.0},
        'cell_pose': {'c1': [0.2, -0.05, 0.0], 'c2': [0.2, 0.05, 0.0]},
        'bin_pose': {'bin_cup': [0.3, -0.15, 0.0], 'bin_mouse': [0.3, 0.15, 0.0]},
        'judge': {'held': judge, 'placed': judge, 'retry_grab': 1, 'retract_retry_note': None},
    }


def asker(answers):
    """按剧本回答的人工确认回调（多于剧本的提问一律 True）。"""
    seq = list(answers)

    def ask(q):
        return seq.pop(0) if seq else True
    return ask


def fail_on(method, n):
    """让某个原语的第 n 次调用抛错。"""
    return lambda m, i, a: (m == method and i == n)


class TestContract(unittest.TestCase):
    def test_implements_all_six_stages(self):
        ex = EPPickExecutor(DryRunRM(), ep_fixture())
        self.assertIsInstance(ex, PickExecutor)
        for _, name in STAGE_ORDER:
            self.assertTrue(callable(getattr(ex, name)), name)

    def test_stage_order_matches_action_definition(self):
        self.assertEqual([n for _, n in STAGE_ORDER],
                         ['descend', 'grasp', 'lift', 'carry', 'release', 'retract'])
        self.assertEqual([s for s, _ in STAGE_ORDER],
                         [STAGE_APPROACH, STAGE_GRASP, STAGE_LIFT, STAGE_MOVE_TO_BIN,
                          STAGE_PLACE, STAGE_RETRACT])

    def test_every_stage_has_a_reason(self):
        for s, _ in STAGE_ORDER:
            self.assertIn(s, STAGE_REASON)
        self.assertEqual(STAGE_REASON[STAGE_APPROACH], REASON_UNREACHABLE)
        self.assertEqual(STAGE_REASON[STAGE_LIFT], REASON_GRASP_FAILED)
        self.assertEqual(STAGE_REASON[STAGE_PLACE], REASON_DROPPED)
        self.assertEqual(STAGE_REASON[STAGE_RETRACT], REASON_EXEC_ERROR)


class TestHappyPath(unittest.TestCase):
    def test_primitive_sequence(self):
        rm = DryRunRM()
        pp = EPPickPlace(EPPickExecutor(rm, ep_fixture()), grid_fixture(), bins_fixture())
        res, note = pp('c1', 'cup')
        self.assertEqual(res, OK, note)
        self.assertEqual([c[0] for c in rm.calls],
                         ['arm_moveto', 'gripper_open',              # prepare 归位
                          'goto', 'arm_moveto',                       # descend
                          'gripper_close',                            # grasp
                          'arm_moveto',                               # lift
                          'goto',                                     # carry
                          'arm_moveto', 'gripper_open',               # release
                          'arm_moveto', 'goto'])                      # retract
        # 下探到抓取低点、抬到高位、放置低点回落（未单独标 → 与抓取同高）
        self.assertIn(('arm_moveto', (70, 60)), [(m, a[:2]) for m, a in rm.calls])
        self.assertIn(('arm_moveto', (70, 150)), [(m, a[:2]) for m, a in rm.calls])
        self.assertEqual(rm.calls[-1], ('goto', (0.0, 0.0, 0.0)))       # 回到 HOME
        self.assertEqual(rm.grip, 'open')

    def test_goal_is_cell_id_not_coordinates(self):
        """执行侧按 id 查位姿 —— ctx 里的表坐标换了(识别标定变了)不影响动作。"""
        rm = DryRunRM()
        g = grid_fixture()
        g['cells'][0].update(x0=9.0, y0=9.0, x1=9.1, y1=9.1)   # 表坐标改到天上去
        pp = EPPickPlace(EPPickExecutor(rm, ep_fixture()), g, bins_fixture())
        res, _ = pp('c1', 'cup')
        self.assertEqual(res, OK)
        self.assertIn(('goto', (0.2, -0.05, 0.0)), rm.calls)     # 走的还是 cell_pose

    def test_stage_callback_reports_six_stages(self):
        seen = []
        ex = EPPickExecutor(DryRunRM(), ep_fixture(), on_stage=lambda n, name: seen.append(n))
        EPPickPlace(ex, grid_fixture(), bins_fixture())('c2', 'mouse')
        self.assertEqual(seen, [STAGE_APPROACH, STAGE_GRASP, STAGE_LIFT,
                                STAGE_MOVE_TO_BIN, STAGE_PLACE, STAGE_RETRACT])


class TestFailureMapping(unittest.TestCase):
    """逐段注入失败 → 段号/reason 对应 + 一定收了尾。"""

    def _run(self, fail_pred, cls='cup'):
        rm = DryRunRM(fail_pred=fail_pred)
        ep = ep_fixture()
        pp = EPPickPlace(EPPickExecutor(rm, ep), grid_fixture(), bins_fixture())
        before = len(rm.calls)
        res, note = pp('c1', cls)
        return res, note, rm, rm.calls[before:]

    def test_descend_goto_fail_unreachable(self):
        res, note, _, _ = self._run(fail_on('goto', 1))
        self.assertEqual(res, REASON_UNREACHABLE)
        self.assertIn('stage=1(approach)', note)

    def test_descend_arm_fail_unreachable(self):
        res, note, _, _ = self._run(fail_on('arm_moveto', 2))
        self.assertEqual(res, REASON_UNREACHABLE)
        self.assertIn('下探', note)

    def test_gripper_close_fail_exec_error(self):
        """夹爪闭合动作失败 → exec_error（**不是** grasp_failed）。

        grasp() 只调 gripper_close()，没有判 held 的能力；taxonomy 里 grasp_failed 是
        "抬离后判定没夹起"，判定在 lift 段（见 test_lift_fail_grasp_failed）。
        这里填 grasp_failed 会凭空宣布一个没做过的判定。
        """
        res, note, _, _ = self._run(fail_on('gripper_close', 1))
        self.assertEqual(res, REASON_EXEC_ERROR)
        self.assertIn('stage=2(grasp)', note)

    def test_lift_fail_grasp_failed(self):
        res, note, _, _ = self._run(fail_on('arm_moveto', 3))
        self.assertEqual(res, REASON_GRASP_FAILED)
        self.assertIn('stage=3(lift)', note)

    def test_carry_fail_unreachable(self):
        res, note, _, _ = self._run(fail_on('goto', 2))
        self.assertEqual(res, REASON_UNREACHABLE)
        self.assertIn('stage=4(move_to_bin)', note)

    def test_release_fail_dropped(self):
        res, note, _, _ = self._run(fail_on('gripper_open', 2))
        self.assertEqual(res, REASON_DROPPED)
        self.assertIn('stage=5(place)', note)

    def test_retract_fail_once_then_ok(self):
        """retract 第一次失败、重试成功 → 整轮仍算成功（物已放下，不为一次读位抖动判失败）。"""
        # arm_moveto 第 5 次 = retract 首次抬到高位（前 4 次：prepare/descend/lift/release）
        res, note, _, _ = self._run(fail_on('arm_moveto', 5))
        self.assertEqual(res, OK, note)
        self.assertIn('第 2 次成功', note)

    def test_retract_fail_twice_exec_error(self):
        """连续两次回不了位 → exec_error：物已放下但机械臂停在未知处，宁停不猜。"""
        res, note, _, _ = self._run(lambda m, i, a: m == 'arm_moveto' and i >= 5)
        self.assertEqual(res, REASON_EXEC_ERROR)
        self.assertIn('stage=6(retract)', note)

    def test_every_failure_ends_in_safe_reset(self):
        """任何一段失败，最后一个动作都该是回 HOME（收尾），不是停在半路。"""
        for pred in (fail_on('goto', 1), fail_on('gripper_close', 1),
                     fail_on('goto', 2), fail_on('gripper_open', 2)):
            with self.subTest(pred=pred):
                _, _, rm, tail = self._run(pred)
                self.assertIn(('goto', (0.0, 0.0, 0.0)), rm.calls[-len(tail):])
                self.assertEqual(rm.calls[-1], ('goto', (0.0, 0.0, 0.0)))

    def test_prepare_fail_refuses_to_move(self):
        """预备归位失败（臂位置未知）→ 直接 exec_error，一个底盘动作都不发。"""
        rm = DryRunRM(fail_pred=fail_on('arm_moveto', 1))
        pp = EPPickPlace(EPPickExecutor(rm, ep_fixture()), grid_fixture(), bins_fixture())
        res, note = pp('c1', 'cup')
        self.assertEqual(res, REASON_EXEC_ERROR)
        self.assertIn('预备归位失败', note)
        self.assertNotIn('goto', [m for m, _ in rm.calls])


class TestJudgement(unittest.TestCase):
    def test_held_no_then_yes_retries(self):
        """HELD 先否再是 → 就地重抓一次后成功（不整轮判失败）。"""
        rm = DryRunRM()
        ex = EPPickExecutor(rm, ep_fixture(judge='operator_confirm'),
                            ask=asker([False, True, True]))
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c1', 'cup')
        self.assertEqual(res, OK, note)
        self.assertEqual(rm.calls.count(('gripper_close', ())), 2)   # 压紧两次
        self.assertIn('held=已抬起（操作员确认夹住）', note)
        self.assertEqual(ex.n_asks, 3)                              # held 否 / held 是 / placed 是

    def test_held_always_no_grasp_failed(self):
        rm = DryRunRM()
        ex = EPPickExecutor(rm, ep_fixture(judge='operator_confirm'),
                            ask=asker([False, False]))
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c1', 'cup')
        self.assertEqual(res, REASON_GRASP_FAILED)
        self.assertIn('HELD 未确认', note)

    def test_placed_no_dropped(self):
        rm = DryRunRM()
        ex = EPPickExecutor(rm, ep_fixture(judge='operator_confirm'), ask=asker([True, False]))
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c1', 'cup')
        self.assertEqual(res, REASON_DROPPED)
        self.assertIn('stage=5(place)', note)

    def test_operator_confirm_without_ask_is_loud(self):
        """要人工确认却没给键盘 → 报错，而不是"安静地当成成功"。"""
        ex = EPPickExecutor(DryRunRM(), ep_fixture(judge='operator_confirm'), ask=None)
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c1', 'cup')
        self.assertEqual(res, REASON_GRASP_FAILED)
        self.assertIn('没有人工确认回调', note)

    def test_judge_count_is_2_per_object(self):
        ex = EPPickExecutor(DryRunRM(), ep_fixture(judge='operator_confirm'), ask=asker([]))
        pp = EPPickPlace(ex, grid_fixture(), bins_fixture())
        pp('c1', 'cup')
        pp('c2', 'mouse')
        self.assertEqual(ex.n_asks, 4)      # 2 物 × (held + placed)


class TestGoalValidation(unittest.TestCase):
    def test_unknown_cell(self):
        ex = EPPickExecutor(DryRunRM(), ep_fixture())
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c9', 'cup')
        self.assertEqual(res, REASON_EXEC_ERROR)
        self.assertIn('goal 非法', note)
        self.assertEqual(ex.rm.calls, [])

    def test_unknown_class(self):
        ex = EPPickExecutor(DryRunRM(), ep_fixture())
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c1', 'stapler')
        self.assertEqual(res, REASON_EXEC_ERROR)
        self.assertIn('goal 非法', note)

    def test_cell_pose_gate_left_open(self):
        """ep_waypoints 少一格（闸门本该拦住）→ 现场也要给出可读的错，不是 KeyError。"""
        ep = ep_fixture()
        ep['cell_pose']['c1'] = None
        ex = EPPickExecutor(DryRunRM(), ep)
        res, note = EPPickPlace(ex, grid_fixture(), bins_fixture())('c1', 'cup')
        self.assertEqual(res, REASON_UNREACHABLE)
        self.assertIn('未标定', note)
        self.assertNotIn('KeyError', note)


if __name__ == '__main__':
    unittest.main()
