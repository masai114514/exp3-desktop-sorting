# -*- coding: utf-8 -*-
"""真机入口 run_real.py 的离线端到端断言（假 EP + mock 检测器，不连任何硬件）。

这里验的是三件在真机上没法反复试的事：
  1. **闸门在前**：没标定时必须退出，且是"连机器人之前"就退出；
  2. **链路通**：相机→检测→任务控制→EP 这一段，在 dry-run 下能跑完整轮并落对日志；
  3. **异常路径**：未识别物体被跳过、且不打断其余物体的分类。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
"""
import contextlib
import io
import json
import os
import shutil
import tempfile
import unittest

from real import run_real

_EXP3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SHIPPED = os.path.join(_EXP3, 'config_real')

BY, DATE, VENUE = '测试', '2026-09-15', '离线夹具'


def make_calibrated_config(dst):
    """把 config_real/ 拷一份并**按标定流程填完** —— 夹具即"标定完成后长什么样"。

    填的就是现场那几步：删占位标记 + 回填实测值 + 记 status/by/date/venue。
    夹具里的坐标是**编的**（离线没有真桌面），所以这里量的是"闸门与链路的逻辑"，
    不是"点位准不准" —— 后者只能现场验，这也正是闸门存在的理由。
    """
    for name in ('task.json', 'grid_cells.json', 'bins.json', 'ep_waypoints.json'):
        shutil.copy(os.path.join(_SHIPPED, name), os.path.join(dst, name))

    def rd(n):
        with open(os.path.join(dst, n), encoding='utf-8') as f:
            return json.load(f)

    def wr(n, d):
        with open(os.path.join(dst, n), 'w', encoding='utf-8') as f:
            json.dump(d, f, ensure_ascii=False, indent=2)

    task, grid, bins, ep = rd('task.json'), rd('grid_cells.json'), rd('bins.json'), \
        rd('ep_waypoints.json')

    task['camera']['calib'].pop('_note', None)
    task['camera']['calibration'] = dict(status='calibrated', by=BY, date=DATE, venue=VENUE)

    for c in grid['cells']:
        c.pop('_note', None)
    grid['calibration'] = dict(status='calibrated', by=BY, date=DATE, venue=VENUE)

    for b in bins['bins'].values():
        b.pop('_note', None)
    bins['calibration'] = dict(status='calibrated', by=BY, date=DATE, venue=VENUE)

    # 假 odom：按格中心推一个"互不相同、量级合理"的底盘位姿。真值现场读。
    for c in grid['cells']:
        cx = (c['x0'] + c['x1']) / 2.0
        cy = (c['y0'] + c['y1']) / 2.0
        ep['cell_pose'][c['id']] = [round(0.26 - cx, 3), round(cy, 3), 0.0]
    for bid, b in bins['bins'].items():
        ep['bin_pose'][bid] = [round(0.26 - b['x_m'], 3), round(b['y_m'], 3), 0.0]
    ep['chassis']['home_pose'] = [0.0, 0.0, 0.0]
    ep['arm']['grab_low_mm'] = [70, 60]
    ep['arm']['lift_high_mm'] = [70, 150]
    # 这两档是**给人看的**稳定时间(抬起来等物体不晃了再问 y/n)，不是标定值 ——
    # 夹具里归零，否则每跑一个测试就要空等 6 物 × 2s。
    ep['arm']['lift_wait_s'] = 0.0
    ep['arm']['drop_wait_s'] = 0.0
    ep['calibration'] = dict(status='calibrated', by=BY, date=DATE, venue=VENUE)

    for n, d in (('task.json', task), ('grid_cells.json', grid),
                 ('bins.json', bins), ('ep_waypoints.json', ep)):
        wr(n, d)
    return dst


def run_main(args):
    """跑 run_real.main 并捕获 stdout → (exit_code, 输出文本)。"""
    buf = io.StringIO()
    code = 0
    with contextlib.redirect_stdout(buf):
        try:
            code = run_real.main(args)
        except SystemExit as e:
            code = e.code if isinstance(e.code, int) else (0 if e.code is None else 1)
    return code, buf.getvalue()


def read_json(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def find_run_dir(log_dir):
    runs = sorted(d for d in os.listdir(log_dir) if d.startswith('run_'))
    assert runs, '没生成 run_* 目录'
    return os.path.join(log_dir, runs[-1])


class TestGate(unittest.TestCase):
    def test_shipped_config_is_refused(self):
        """出厂状态(未标定)必须被拦下 —— 这是本次改动的核心，别让它悄悄退化。"""
        code, out = run_main(['--config-dir', _SHIPPED, '--dry-run'])
        self.assertEqual(code, 1)
        self.assertIn('未通过真机标定闸门', out)
        self.assertIn('拒绝启动', out)

    def test_gate_runs_before_touching_camera(self):
        """闸门没过时不该走到建相机那一步 —— 这里故意给一个不存在的图，若顺序反了会报读图失败。"""
        code, out = run_main(['--config-dir', _SHIPPED, '--camera',
                              'image:/definitely/not/here.jpg', '--dry-run'])
        self.assertEqual(code, 1)
        self.assertIn('闸门', out)
        self.assertNotIn('读不了图片', out)

    def test_partially_calibrated_is_refused(self):
        """只填了网格、没填 EP 位姿 → 照样拦（分开记标定的意义就在这）。"""
        with tempfile.TemporaryDirectory() as td:
            make_calibrated_config(td)
            p = os.path.join(td, 'ep_waypoints.json')
            ep = read_json(p)
            ep['cell_pose']['c6'] = None
            with open(p, 'w', encoding='utf-8') as f:
                json.dump(ep, f, ensure_ascii=False)
            code, out = run_main(['--config-dir', td, '--dry-run'])
        self.assertEqual(code, 1)
        self.assertIn('cell_pose 有 1 个位姿还是 null', out)
        self.assertIn('c6', out)


class TestDryRunHappyPath(unittest.TestCase):
    def test_six_objects_pass(self):
        with tempfile.TemporaryDirectory() as td:
            make_calibrated_config(td)
            logd = os.path.join(td, 'logs')
            code, out = run_main(['--config-dir', td, '--dry-run', '--camera', 'none',
                                  '--detector', 'mock', '--assume-judge', '--no-pause',
                                  '--log-dir', logd])
            self.assertEqual(code, 0, out)
            rd = find_run_dir(logd)
            res = read_json(os.path.join(rd, 'result.json'))
            ctx = read_json(os.path.join(rd, 'real_run.json'))

        self.assertEqual(res['verdict'], 'PASS')
        self.assertEqual(res['placed_ok'], 6)
        self.assertEqual(res['objects_seen'], 6)
        self.assertEqual(res['exit_status'], 'no_target')
        # 演练标记必须落盘 —— 这一份不能冒充真机验收凭据
        self.assertTrue(ctx['dry_run'])
        self.assertGreater(ctx['dry_run_primitive_calls'], 0)
        self.assertEqual(ctx['calibration']['ep']['by'], BY)

    def test_records_carry_judgement_provenance(self):
        with tempfile.TemporaryDirectory() as td:
            make_calibrated_config(td)
            logd = os.path.join(td, 'logs')
            run_main(['--config-dir', td, '--dry-run', '--camera', 'none', '--assume-judge',
                      '--no-pause', '--log-dir', logd])
            rd = find_run_dir(logd)
            with open(os.path.join(rd, 'records.jsonl'), encoding='utf-8') as f:
                recs = [json.loads(ln) for ln in f if ln.strip()]

        execs = [r for r in recs if r['decision'] == 'exec']
        self.assertEqual(len(execs), 6)
        for r in execs:
            self.assertTrue(r['grasp_ok'] and r['place_ok'])
            self.assertIn('held=', r['detail'])        # 夹起/放正的来源要能查
            self.assertIn('assume_ok', r['detail'])    # 这次是 assume，就该如实写 assume

    def test_operator_confirm_path_counts_two_asks_per_object(self):
        """真机默认口径（operator_confirm）：替代键盘回答，确认每物只问两次。"""
        asks = []
        old = run_real._ask_stdin
        run_real._ask_stdin = lambda q: (asks.append(q), True)[1]
        try:
            with tempfile.TemporaryDirectory() as td:
                make_calibrated_config(td)
                logd = os.path.join(td, 'logs')
                code, out = run_main(['--config-dir', td, '--dry-run', '--camera', 'none',
                                      '--no-pause', '--log-dir', logd])
                ctx = read_json(os.path.join(find_run_dir(logd), 'real_run.json'))
        finally:
            run_real._ask_stdin = old

        self.assertEqual(code, 0, out)
        self.assertEqual(ctx['operator_confirms'], 12)      # 6 物 × (HELD + PLACED)
        self.assertEqual(ctx['judge']['held'], 'operator_confirm')
        self.assertEqual(sum('HELD' in q for q in asks), 6)
        self.assertEqual(sum('PLACED' in q for q in asks), 6)
        # 人工只被问了"夹住没有/放正没有"，没被问类别 —— docx §六 那一条
        for q in asks:
            self.assertNotIn('是 cup 还是 mouse', q)


class TestAbnormal(unittest.TestCase):
    def _scene(self, td, extra):
        p = os.path.join(td, 'scene.json')
        base = read_json(os.path.join(_EXP3, 'real', 'scenes', 'demo_6obj.json'))
        base['extra_dets'] = extra
        with open(p, 'w', encoding='utf-8') as f:
            json.dump(base, f, ensure_ascii=False)
        return p

    def test_unrecognized_object_is_skipped_not_fatal(self):
        """桌上多一个没训练过的类：跳过它，其余 6 个照常分类（docx §六 的异常项之一）。"""
        with tempfile.TemporaryDirectory() as td:
            make_calibrated_config(td)
            scene = self._scene(td, [{'cls': 'stapler', 'conf': 0.9,
                                      'bbox': [200, 200, 212, 212]}])
            logd = os.path.join(td, 'logs')
            code, out = run_main(['--config-dir', td, '--dry-run', '--camera', 'none',
                                  '--detector', 'mock', '--scene', scene, '--assume-judge',
                                  '--no-pause', '--log-dir', logd])
            res = read_json(os.path.join(find_run_dir(logd), 'result.json'))

        self.assertEqual(code, 0, out)
        self.assertEqual(res['placed_ok'], 6)                    # 6 个该放的都放了
        self.assertEqual(res['reasons'].get('unrecognized_object'), 1)
        self.assertEqual(res['exit_status'], 'no_exec')          # 剩下那个跳过项 → 空转收尾

    def test_low_confidence_is_unrecognized(self):
        """置信度低于 conf_min=0.5 的检测 → unrecognized_object（不是"看不见"）。"""
        with tempfile.TemporaryDirectory() as td:
            make_calibrated_config(td)
            scene = self._scene(td, [{'cls': 'cup', 'conf': 0.3,
                                      'bbox': [200, 200, 212, 212]}])
            logd = os.path.join(td, 'logs')
            run_main(['--config-dir', td, '--dry-run', '--camera', 'none', '--detector',
                      'mock', '--scene', scene, '--assume-judge', '--no-pause',
                      '--log-dir', logd])
            res = read_json(os.path.join(find_run_dir(logd), 'result.json'))
        self.assertEqual(res['reasons'].get('unrecognized_object'), 1)
        self.assertEqual(res['placed_ok'], 6)

    def test_out_of_grid_detection_is_skipped(self):
        """画面里但不在网格上的检测 → out_of_grid（铺在桌面上网格外的东西不会被误抓）。"""
        with tempfile.TemporaryDirectory() as td:
            make_calibrated_config(td)
            scene = self._scene(td, [{'cls': 'mouse', 'conf': 0.9,
                                      'bbox': [600, 400, 612, 412]}])   # 角落，网格外
            logd = os.path.join(td, 'logs')
            run_main(['--config-dir', td, '--dry-run', '--camera', 'none', '--detector',
                      'mock', '--scene', scene, '--assume-judge', '--no-pause',
                      '--log-dir', logd])
            res = read_json(os.path.join(find_run_dir(logd), 'result.json'))
        self.assertEqual(res['reasons'].get('out_of_grid'), 1)
        self.assertEqual(res['placed_ok'], 6)

    def test_grasp_failure_stops_after_fail_limit(self):
        """操作员一直说"没夹住" → 就地重抓一次仍不行 → 连续 3 次后 safety_stop 止损。

        这条线是真机的安全底线：夹不住时**不能无限重试**（EP 开环跑，每试一次都是真的
        在动），到 fail_limit 就停。这里同时验了重抓逻辑（每轮压紧 2 次 = 首次 + 重抓）。
        """
        closes = []
        old = run_real._ask_stdin
        # HELD 一律否；PLACED 一律是（走不到那一步，但别让回调意外消费掉）
        run_real._ask_stdin = lambda q: (closes.append(q), not q.startswith('HELD'))[1]
        try:
            with tempfile.TemporaryDirectory() as td:
                make_calibrated_config(td)
                logd = os.path.join(td, 'logs')
                run_main(['--config-dir', td, '--dry-run', '--camera', 'none',
                          '--detector', 'mock', '--no-pause', '--log-dir', logd])
                res = read_json(os.path.join(find_run_dir(logd), 'result.json'))
        finally:
            run_real._ask_stdin = old

        self.assertEqual(res['placed_ok'], 0)
        self.assertEqual(res['exit_status'], 'safety_stop')
        self.assertEqual(res['reasons'].get('grasp_failed'), 3)   # = fail_limit
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertEqual(sum('HELD' in q for q in closes), 6)     # 3 轮 × (首次 + 重抓)


class TestScanSeam(unittest.TestCase):
    def test_camera_failure_raises_instead_of_looking_empty(self):
        """取不到帧必须抛 —— 返回 [] 会被 TaskController 当成"桌面已空、正常结束"。"""
        from real.camera import make_scan

        class Bad:
            def grab(self):
                raise RuntimeError('掉线')

        class D:
            def detect(self, frame):
                return []

        scan = make_scan(Bad(), D())
        with self.assertRaises(RuntimeError):
            scan()


if __name__ == '__main__':
    unittest.main()
