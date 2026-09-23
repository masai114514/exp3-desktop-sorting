# -*- coding: utf-8 -*-
import tempfile
import os
import unittest

from sort_core.task import TaskController
from sort_core.logging_util import Exp3RunLog, make_run_dir, verdict_for
from sort_core.taxonomy import (REASON_NO_TARGET, REASON_NO_EXEC, REASON_SAFETY_STOP,
                                REASON_UNRECOGNIZED, REASON_OUT_OF_GRID,
                                VERDICT_PASS, VERDICT_TRIAL)

CELLS = [
    {'id': 'c1', 'row': 0, 'col': 0, 'x0': 0.0, 'y0': 0.0, 'x1': 0.1, 'y1': 0.1},
    {'id': 'c2', 'row': 0, 'col': 1, 'x0': 0.0, 'y0': 0.1, 'x1': 0.1, 'y1': 0.2},
]
BINS = {'class_to_bin': {'cup': 'bin_cup', 'mouse': 'bin_mouse'}}


def make_cfg(expected_total=6, pass_line=5, conf_min=0.5, fail_limit=3,
             no_exec_limit=3, max_rounds=60):
    return {
        'expected_total': expected_total, 'pass_line': pass_line,
        'conf_min': conf_min, 'fail_limit': fail_limit,
        'no_exec_limit': no_exec_limit, 'max_rounds': max_rounds,
        'camera': {'image_width_px': 500, 'image_height_px': 500,
                   'calib': {'mode': 'rectilinear', 'offset_px': [0.0, 0.0],
                             'scale_px_per_m': [1000.0, 1000.0]}},
    }


def bbox_at(px, py, hw=5):
    return [px - hw, py - hw, px + hw, py + hw]


def det_for_cell(cell_id, cls, conf=0.9):
    # 像素坐标 = 1000 * 桌面坐标(米)
    cell = next(c for c in CELLS if c['id'] == cell_id)
    px, py = (cell['x0'] + cell['x1']) / 2 * 1000, (cell['y0'] + cell['y1']) / 2 * 1000
    return {'cls': cls, 'conf': conf, 'bbox': bbox_at(px, py)}


class _TableBackend:
    """模拟一张有 6 个位置/若干物体的桌面。pick 成功即从桌面移除。"""

    def __init__(self, objects, pick_result='ok'):
        self.objects = list(objects)      # [(cell_id, cls)]
        self.pick_result = pick_result
        self.picked = []

    def scan(self):
        return [det_for_cell(cid, cls) for (cid, cls) in self.objects]

    def pick_place(self, cell_id, cls):
        if self.pick_result == 'ok' and (cell_id, cls) in self.objects:
            self.objects.remove((cell_id, cls))
            self.picked.append((cell_id, cls))
            return 'ok', 'picked+placed'
        return self.pick_result, 'fail'


class TestAllSorted(unittest.TestCase):
    def test_no_target_when_table_cleared(self):
        cfg = make_cfg(expected_total=2, pass_line=2)
        tb = _TableBackend([('c1', 'cup'), ('c2', 'mouse')])
        ctl = TaskController(cfg, BINS, CELLS, tb.scan, tb.pick_place)
        s = ctl.run()
        self.assertEqual(s['exit_status'], REASON_NO_TARGET)
        self.assertEqual(s['placed_ok'], 2)
        self.assertEqual(s['objects_seen'], 2)
        self.assertEqual(sorted(tb.picked), [('c1', 'cup'), ('c2', 'mouse')])


class TestObjectsSeen(unittest.TestCase):
    def test_out_of_grid_bin_detection_does_not_inflate_objects_seen(self):
        cfg = make_cfg(expected_total=2, pass_line=1, no_exec_limit=2)
        objects = [('c1', 'cup')]

        def scan():
            dets = [det_for_cell(cid, cls) for cid, cls in objects]
            if not objects:
                dets.append({'cls': 'cup', 'conf': 0.9, 'bbox': bbox_at(450, 450)})
            return dets

        def pick_place(cell_id, cls):
            objects.remove((cell_id, cls))
            return 'ok', 'picked+placed'

        ctl = TaskController(cfg, BINS, CELLS, scan, pick_place)
        summary = ctl.run()

        self.assertEqual(summary['placed_ok'], 1)
        self.assertEqual(summary['objects_seen'], 1)
        self.assertEqual(
            verdict_for(summary['placed_ok'], summary['objects_seen'],
                        cfg['expected_total'], cfg['pass_line'], final=True),
            VERDICT_TRIAL)


class TestSafetyStop(unittest.TestCase):
    def test_stop_after_consecutive_grasp_fails(self):
        cfg = make_cfg(fail_limit=3)
        tb = _TableBackend([('c1', 'cup')], pick_result='grasp_failed')
        stopped = []
        ctl = TaskController(cfg, BINS, CELLS, tb.scan, tb.pick_place,
                             safe_stop=lambda r: stopped.append(r))
        s = ctl.run()
        self.assertEqual(s['exit_status'], REASON_SAFETY_STOP)
        self.assertEqual(stopped, [REASON_SAFETY_STOP])
        self.assertEqual(s['placed_ok'], 0)
        self.assertEqual(s['reasons'].get('grasp_failed'), 3)


class TestUnrecognizedOnly(unittest.TestCase):
    def test_no_exec_when_only_unknown_class(self):
        cfg = make_cfg(no_exec_limit=3)

        def scan():
            return [{'cls': 'stapler', 'conf': 0.9, 'bbox': bbox_at(50, 50)}]

        ctl = TaskController(cfg, BINS, CELLS, scan, lambda *a: ('ok', 'x'))
        s = ctl.run()
        self.assertEqual(s['exit_status'], REASON_NO_EXEC)
        self.assertEqual(s['placed_ok'], 0)
        # 每个 (cell,cls) 只记一次跳过
        self.assertEqual(s['reasons'].get(REASON_UNRECOGNIZED), 1)


class TestOutOfGrid(unittest.TestCase):
    def test_out_of_grid_skipped(self):
        cfg = make_cfg(no_exec_limit=2)

        def scan():
            return [{'cls': 'mouse', 'conf': 0.9, 'bbox': bbox_at(450, 450)}]  # 无网格

        ctl = TaskController(cfg, BINS, CELLS, scan, lambda *a: ('ok', 'x'))
        s = ctl.run()
        self.assertEqual(s['exit_status'], REASON_NO_EXEC)
        self.assertEqual(s['reasons'].get(REASON_OUT_OF_GRID), 1)


class TestLogging(unittest.TestCase):
    def test_logger_writes_records_and_result(self):
        with tempfile.TemporaryDirectory() as d:
            run_dir = make_run_dir(d)
            log = Exp3RunLog(run_dir)
            cfg = make_cfg(expected_total=2, pass_line=2)
            tb = _TableBackend([('c1', 'cup'), ('c2', 'mouse')])
            ctl = TaskController(cfg, BINS, CELLS, tb.scan, tb.pick_place, log=log)
            s = ctl.run()
            verdict = log.write_result(
                s['placed_ok'], s['objects_seen'], cfg['expected_total'],
                cfg['pass_line'], s['exit_status'], s['reasons'])
            self.assertEqual(verdict, VERDICT_PASS)
            self.assertTrue(os.path.exists(os.path.join(run_dir, 'records.jsonl')))
            self.assertTrue(os.path.exists(os.path.join(run_dir, 'result.json')))
            with open(os.path.join(run_dir, 'records.jsonl'), encoding='utf-8') as f:
                lines = [ln for ln in f if ln.strip()]
            self.assertEqual(len(lines), 2)


if __name__ == '__main__':
    unittest.main()
