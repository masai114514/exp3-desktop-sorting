# -*- coding: utf-8 -*-
"""标定工具 real/calibrate.py 的离线断言（假 EP，不连任何硬件）。

标定是**唯一一处"人会手抖、代价是撞桌"**的环节，所以这里盯的是它的防错，而不是它写没写对：
  1. **钳制陷阱**：读回值和目标差太多时必须**拒绝记录**（被 SDK 安静钳制的值填进表，
     真机上就是一个永远够不着、还只打 WARN 的点位）；
  2. **不误覆盖**：已有值的槽必须 --force 才改（现场改错一格 = 撞一次桌）；
  3. **写进去的是什么**：臂是 2 元列表 `[x_mm, y_mm]` 而不是标量；
  4. **封表条件**：本表没填全就不许封，且 --seal 一定要留 by/venue；
  5. **失效语义**：--invalidate 只作废底盘位姿，臂/夹爪照旧（这条讲错了会让人白重标一遍）。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
"""
import contextlib
import inspect
import io
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

from real import calibrate, ep_conn
from real.dry_run import DryRunRM

_EXP3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SHIPPED = os.path.join(_EXP3, 'config_real')

BY, VENUE, DATE = '测试', '离线夹具', '2026-09-16'


# ---------------------------------------------------------------- 假机器人
class FakeRM(DryRunRM):
    """在 DryRunRM 上加两个标定才用得到的钩子：可控的 odom / 臂读回。

    `arm_readback` 默认**照抄要求值**（模拟臂走到了）；要模拟"被钳制"，就把它设成
    一个和目标差很远的固定值 —— 真机上位姿会"稳定地停在"那儿，SDK 只打 WARN 不抛异常。
    """

    ARM_MISS_TOL_MM = 3.0            # 真机 rm.py 上就是这个类属性

    def __init__(self, log=None, odom=(0.31, -0.12, 8.0), arm_readback=None):
        DryRunRM.__init__(self, log)
        self._odom = list(odom)
        self.arm_readback = arm_readback
        self.moves = []

    def read_chassis_pose(self):
        return list(self._odom)

    def arm_moveto(self, x, y, tag=None):
        self.moves.append((int(x), int(y)))
        return True

    def read_arm_xy(self):
        if self.arm_readback is not None:
            return list(self.arm_readback)
        return list(self.moves[-1]) if self.moves else [0, 0]


def setup_dir(dst):
    for name in ('task.json', 'grid_cells.json', 'bins.json', 'ep_waypoints.json'):
        shutil.copy(os.path.join(_SHIPPED, name), os.path.join(dst, name))
    return dst


def rd(d, n):
    with open(os.path.join(d, n), encoding='utf-8') as f:
        return json.load(f)


def wr(d, n, data):
    with open(os.path.join(d, n), 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def fill_all_but(d, skip=()):
    """把 EP 表填到"只差 skip 里那几项"，模拟标定做到一半。"""
    ep = rd(d, 'ep_waypoints.json')
    for s in calibrate._CELL_SLOTS:
        ep['cell_pose'][s] = [0.30, 0.0, 0.0]
    for s in calibrate._BIN_SLOTS:
        ep['bin_pose'][s] = [0.05, 0.35, 0.0]
    ep['chassis']['home_pose'] = [0.0, 0.0, 0.0]
    ep['arm']['grab_low_mm'] = [74, 60]
    ep['arm']['lift_high_mm'] = [74, 150]
    ep['gripper']['open_power'] = 60
    ep['gripper']['close_power'] = 65
    for key in skip:
        group, field = key.split('.')
        ep[group][field] = None
    wr(d, 'ep_waypoints.json', ep)
    return ep


def run_cli(argv, fake=None, assume_yes=False, patch_connect=True):
    """跑 calibrate.main，把连机器人这一步换成假 EP。→ (code, 输出)

    这里的输出 = 工具自己 print 的 + **被我们吞掉的那句 SystemExit**。
    工具用 `raise SystemExit('…')` 报错，而那句话平时是解释器在退出时打出来的 ——
    我们在这里 catch 了它，不自己收进返回值就会两手空空，于是"拒绝的理由"全丢，
    只剩下一个非 0 退出码。断言就只剩"它退出了"，测不出它为什么退。
    """
    out, err = io.StringIO(), io.StringIO()
    code, msg = 0, ''
    ctx = mock.patch.object(ep_conn, 'connect', lambda *a, **k: fake) if patch_connect \
        else contextlib.nullcontext()
    with ctx, contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            code = calibrate.main(argv, assume_yes=assume_yes)
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
            msg = '' if e.code is None else str(e)
    return code, out.getvalue() + err.getvalue() + msg


# ---------------------------------------------------------------- 逐项
class TestProgress(unittest.TestCase):
    def test_shipped_table_reports_everything_missing(self):
        """出厂表：进度要如实报"差 11 项"（9 底盘 + grab/lift 两档）。

        夹爪两档出厂是 60（不是 null），release_low_mm 的 null **本身合法** —— 两者都不算缺。
        """
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td])
        self.assertEqual(code, 1)                      # 还没标完 ⇒ 非 0
        self.assertIn('★未标', out)
        self.assertIn('还差 11 项', out)
        # release 那一行不能出现在 ★未标 里 —— 否则人会去编一个值填上
        starred = [ln.split()[0] for ln in out.splitlines() if '★未标' in ln]
        self.assertNotIn('release_low_mm', starred)
        self.assertIn('（未填=同 grab）', out)

    def test_progress_and_seal_agree_on_what_is_missing(self):
        """--progress 的项数必须和 --seal 的判据一致 —— 两边口径不一样，
        人就会照 --progress 去补一个其实不该补的项（比如 release）。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            ep = rd(td, 'ep_waypoints.json')
            n_seal = len(calibrate._ep_missing(ep))
            code, out = run_cli(['--config-dir', td])
        self.assertEqual(n_seal, 11)
        self.assertIn('还差 %d 项' % n_seal, out)

    def test_fully_filled_table_points_at_seal(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td])
        self.assertEqual(code, 0)
        self.assertIn('已填全', out)
        self.assertIn('--seal', out)


class TestRecordPose(unittest.TestCase):
    def test_records_odom_list_into_the_right_slot(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fake = FakeRM(odom=(0.31, -0.12, 8.0))
            code, out = run_cli(['--config-dir', td, '--record', 'c3'], fake, assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')

        self.assertEqual(code, 0, out)
        self.assertEqual(ep['cell_pose']['c3'], [0.31, -0.12, 8.0])
        self.assertIsNone(ep['cell_pose']['c4'])       # 别的槽不许动
        self.assertIn('本次上电', out)                  # 有效期必须当场说清

    def test_home_and_home_pose_are_the_same_slot(self):
        """两个写法指同一个槽 —— 别名写错了现场会以为标了两个点，其实只有一个。"""
        for alias in ('home', 'home_pose'):
            with tempfile.TemporaryDirectory() as td:
                setup_dir(td)
                run_cli(['--config-dir', td, '--record', alias], FakeRM(), assume_yes=True)
                ep = rd(td, 'ep_waypoints.json')
            self.assertEqual(ep['chassis']['home_pose'], [0.31, -0.12, 8.0], alias)

    def test_bin_pose_slots(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            run_cli(['--config-dir', td, '--record', 'bin_mouse'], FakeRM(), assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(ep['bin_pose']['bin_mouse'], [0.31, -0.12, 8.0])
        self.assertIsNone(ep['bin_pose']['bin_cup'])

    def test_refuses_to_overwrite_without_force(self):
        """已有值 → 拒绝覆盖。现场改错一格就是撞一次桌，必须让人显式 --force。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td, '--record', 'c1'], FakeRM(),
                                assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 1)
        self.assertIn('已经有值', out)
        self.assertIn('--force', out)
        self.assertEqual(ep['cell_pose']['c1'], [0.30, 0.0, 0.0])   # 原值没动

    def test_force_overwrites_and_says_what_it_replaced(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td, '--record', 'c1', '--force'],
                                FakeRM(odom=(0.9, 0.9, 0.0)), assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 0, out)
        self.assertEqual(ep['cell_pose']['c1'], [0.9, 0.9, 0.0])
        self.assertIn('旧值', out)

    def test_declining_the_confirmation_writes_nothing(self):
        """问"车现在就在这个位置吗"答 n → 不写表（现场摆错一格时靠这个兜住）。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            with mock.patch('builtins.input', return_value='n'):
                code, out = run_cli(['--config-dir', td, '--record', 'c1'], FakeRM())
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 1)
        self.assertIn('没记', out)
        self.assertIsNone(ep['cell_pose']['c1'])

    def test_unknown_slot_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--record', 'c9'], FakeRM(),
                                assume_yes=True)
        self.assertEqual(code, 1)
        self.assertIn('未知槽名', out)


class TestRecordArm(unittest.TestCase):
    def test_writes_a_two_element_list_not_a_scalar(self):
        """★臂位姿是 [x_mm, y_mm] 2 元列表。写成标量的话，驱动 arm_moveto(*v) 会直接 TypeError。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--record-arm', 'grab', '74', '120'],
                                FakeRM(), assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')

        self.assertEqual(code, 0, out)
        self.assertEqual(ep['arm']['grab_low_mm'], [74, 120])
        self.assertIsInstance(ep['arm']['grab_low_mm'], list)

    def test_release_slot_maps_to_its_own_field(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            run_cli(['--config-dir', td, '--record-arm', 'release', '74', '70'],
                    FakeRM(), assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(ep['arm']['release_low_mm'], [74, 70])
        self.assertIsNone(ep['arm']['grab_low_mm'])       # 别串档

    def test_refuses_a_clamped_readback(self):
        """★核心防错：读回值和目标差 20mm ⇒ 多半是超行程被 SDK 钳制，**拒绝记录**。

        这条要是退化了，把钳制值填进表的人在真机上会得到一个"永远够不着、只打 WARN、
        现象像抓取不稳"的档位 —— 实验二花了不少时间才定位到这类问题。
        """
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fake = FakeRM(arm_readback=(54, 120))          # 要求 74，只到 54（差 20mm）
            code, out = run_cli(['--config-dir', td, '--record-arm', 'grab', '74', '120'],
                                fake, assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')

        self.assertEqual(code, 1)
        self.assertIn('钳制', out)
        self.assertIn('别把这个数填进表', out)
        self.assertIsNone(ep['arm']['grab_low_mm'])        # 没写进去

    def test_force_records_the_clamped_value_anyway(self):
        """确实要强行记录时留个后门，但得显式 --force。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fake = FakeRM(arm_readback=(54, 120))
            code, out = run_cli(['--config-dir', td, '--record-arm', 'grab', '74', '120',
                                 '--force'], fake, assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 0, out)
        self.assertEqual(ep['arm']['grab_low_mm'], [74, 120])

    def test_small_readback_drift_is_accepted(self):
        """稳态本来就有 ±1mm 抖动 ⇒ 差 1mm 属于正常，不该拦。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--record-arm', 'lift', '74', '150'],
                                FakeRM(arm_readback=(75, 150)), assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 0, out)
        self.assertEqual(ep['arm']['lift_high_mm'], [74, 150])

    def test_refuses_to_overwrite_existing_arm_value(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td, '--record-arm', 'grab', '70', '60'],
                                FakeRM(), assume_yes=True)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 1)
        self.assertIn('已经有值', out)
        self.assertEqual(ep['arm']['grab_low_mm'], [74, 60])

    def test_unknown_arm_slot_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--record-arm', 'hover', '74', '120'],
                                FakeRM(), assume_yes=True)
        self.assertEqual(code, 1)
        self.assertIn('未知臂档', out)

    def test_non_integer_arm_values_are_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--record-arm', 'grab', '7.4', '120'],
                                FakeRM(), assume_yes=True)
        self.assertEqual(code, 1)
        self.assertIn('整数', out)


class TestGripper(unittest.TestCase):
    def test_power_out_of_range_is_rejected(self):
        """官方范围 1..100 —— 越界值不该被写进表，也不该拿去驱动。"""
        for bad in ('0', '101', '-5'):
            with tempfile.TemporaryDirectory() as td:
                setup_dir(td)
                code, out = run_cli(['--config-dir', td, '--record-grip', 'close', bad])
            self.assertEqual(code, 1, bad)
            self.assertIn('越界', out)

    def test_record_grip_writes_and_reports_the_old_value(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--record-grip', 'close', '75'])
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 0, out)
        self.assertEqual(ep['gripper']['close_power'], 75)
        self.assertEqual(ep['gripper']['open_power'], 60)      # 另一档不动

    def test_trial_grip_does_not_write_the_table(self):
        """--grip 只是"试一次"，**不写表** —— 写表得用 --record-grip，两步分开。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fake = FakeRM()
            code, out = run_cli(['--config-dir', td, '--grip', 'close', '--power', '80'], fake)
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 0, out)
        self.assertEqual(ep['gripper']['close_power'], 60)     # 表里还是原值
        self.assertIn('--record-grip', out)                    # 告诉人下一步怎么记


class TestSeal(unittest.TestCase):
    def test_incomplete_table_cannot_be_sealed(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            code, out = run_cli(['--config-dir', td, '--seal', '--by', BY, '--venue', VENUE])
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 1)
        self.assertIn('还不能封表', out)
        self.assertEqual(ep['calibration']['status'], 'uncalibrated')

    def test_seal_needs_by_and_venue(self):
        """留档是给别人查的：出问题时要知道找谁、在哪儿标的。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td, '--seal', '--by', BY])
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 1)
        self.assertIn('--venue', out)
        self.assertEqual(ep['calibration']['status'], 'uncalibrated')

    def test_seal_stamps_who_when_where(self):
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td, '--seal', '--by', BY, '--venue', VENUE,
                                 '--date', DATE])
            ep = rd(td, 'ep_waypoints.json')
        self.assertEqual(code, 0, out)
        # note 是随表出厂的解释文字，封表不会删它（值得保留给下一个人看），所以逐键比
        self.assertEqual({k: ep['calibration'][k] for k in ('status', 'by', 'date', 'venue')},
                         dict(status='calibrated', by=BY, date=DATE, venue=VENUE))
        self.assertIn('require-calibrated', out)          # 下一步要给出来

    def test_seal_then_gate_passes(self):
        """封表 → `--require-calibrated` 的 EP 那一份归零（其余三份本来就没填，只查 EP）。"""
        import config_check
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            run_cli(['--config-dir', td, '--seal', '--by', BY, '--venue', VENUE, '--date', DATE])
            ep = rd(td, 'ep_waypoints.json')
            issues = config_check.check_ep_waypoints(ep, rd(td, 'grid_cells.json'),
                                                     rd(td, 'bins.json'))
        self.assertEqual([i['msg'] for i in issues], [])


class TestInvalidate(unittest.TestCase):
    def test_invalidates_chassis_but_not_arm_or_gripper(self):
        """★换场地/重新上电：odom 原点跟着上电位姿走 ⇒ 9 个底盘位姿全废，臂/夹爪不受影响。

        这条讲错了会让人白重标一遍臂两档（现场很贵）。
        """
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            ep_before = rd(td, 'ep_waypoints.json')
            code, out = run_cli(['--config-dir', td, '--invalidate'])
            ep = rd(td, 'ep_waypoints.json')

        self.assertEqual(code, 0, out)
        self.assertEqual(ep['calibration']['status'], 'uncalibrated')
        self.assertIn('9 个底盘位姿', out)
        self.assertIn('不用重标', out)
        # 底盘位姿的**值**原样留着（重标时是覆盖，不是从 null 开始），臂/夹爪一字未动
        self.assertEqual(ep['cell_pose'], ep_before['cell_pose'])
        self.assertEqual(ep['arm'], ep_before['arm'])
        self.assertEqual(ep['gripper'], ep_before['gripper'])

    def test_counts_nine_poses_not_ten(self):
        """home / home_pose 是同一个槽 ⇒ 数出来是 9，不是 10。"""
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            fill_all_but(td)
            code, out = run_cli(['--config-dir', td, '--invalidate'])
        self.assertEqual(code, 0, out)
        self.assertIn('9 个底盘位姿', out)
        self.assertNotIn('10 个底盘位姿', out)


class TestConnectionPlumbing(unittest.TestCase):
    def test_calibrate_and_run_real_share_one_connect_helper(self):
        """标定和跑任务必须走同一个连接函数 —— 抄第二份一定会漂（两份 config 拼法不一样）。

        这里只钉住"calibrate 用的是 ep_conn.connect"这一条：真正拼 config 的逻辑
        由 ep_conn 自己的测试覆盖。
        """
        with tempfile.TemporaryDirectory() as td:
            setup_dir(td)
            seen = []

            def spy(ep_cfg, log=None, arm=None, gripper=None):
                seen.append(dict(arm=arm, gripper=gripper))
                return FakeRM()

            with mock.patch.object(ep_conn, 'connect', spy):
                run_cli(['--config-dir', td, '--grip', 'close', '--power', '80'],
                        patch_connect=False)
                run_cli(['--config-dir', td, '--record-arm', 'grab', '74', '120'],
                        assume_yes=True, patch_connect=False)

        self.assertEqual(len(seen), 2)
        # --grip --power 用**覆盖**传下去（试夹不能先改表）
        self.assertEqual(seen[0]['gripper'], {'close_power': 80})
        self.assertEqual(seen[1]['arm'], None)


class TestFieldNames(unittest.TestCase):
    """工具写的字段名必须就是出厂表里那些 —— 名字对不上时，驱动读不到就报"未标定"，
    人会以为是自己没标，而不是工具写错了地方。"""

    def test_writes_into_fields_the_shipped_table_defines(self):
        ep = rd(_SHIPPED, 'ep_waypoints.json')
        for field in calibrate._ARM_FIELD.values():
            self.assertIn(field, ep['arm'], field)
        for field in calibrate._GRIP_FIELD.values():
            self.assertIn(field, ep['gripper'], field)

    def test_backend_reads_the_same_arm_fields(self):
        """反向核对：ep_backend 真的去读这两档（它是**消费**这些字段的一方）。"""
        from real import ep_backend
        for field in ('grab_low_mm', 'lift_high_mm'):
            self.assertIn(field, inspect.getsource(ep_backend), field)


if __name__ == '__main__':
    unittest.main()
