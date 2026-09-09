# -*- coding: utf-8 -*-
"""config_check 的坏配置断言：喂各类错填，确认能 ERROR/WARN 拦下。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
真实 config 应为零问题（config_check.py 直接跑也验证）。
"""
import unittest

from config_check import (check_grid, check_bins, check_camera, check_all,
                          ERROR, WARN)


def valid_grid(cells=None):
    cells = cells or [
        {'id': 'c1', 'row': 0, 'col': 0, 'x0': 0.0, 'y0': 0.0, 'x1': 0.1, 'y1': 0.1},
        {'id': 'c2', 'row': 0, 'col': 1, 'x0': 0.0, 'y0': 0.1, 'x1': 0.1, 'y1': 0.2},
    ]
    return {'cells': cells, 'pick_z_m': 0.1, 'object_z_m': 0.02}


def valid_bins():
    return {
        'bins': {'bin_cup': {'cls': 'cup', 'x_m': 0.2, 'y_m': 0.05},
                 'bin_mouse': {'cls': 'mouse', 'x_m': 0.2, 'y_m': 0.15}},
        'class_to_bin': {'cup': 'bin_cup', 'mouse': 'bin_mouse'},
    }


def valid_task(classes=('cup', 'mouse'), expected_total=2):
    return {
        'classes': list(classes), 'expected_total': expected_total, 'pass_line': 2,
        'camera': {'image_width_px': 640, 'image_height_px': 480,
                   'calib': {'mode': 'rectilinear',
                             'offset_px': [320.0, 240.0],
                             'scale_px_per_m': [1000.0, 1000.0]}},
    }


def errs(issues):
    return [it['msg'] for it in issues if it['severity'] == ERROR]


class TestClean(unittest.TestCase):
    def test_real_config_zero_issues(self):
        # 直接读真实 config 文件，保证当前占位/回填基线是干净的
        from sort_core.config import load_all
        c = load_all()
        self.assertEqual(check_all(c['task'], c['grid'], c['bins']), [])

    def test_valid_fixture_zero_issues(self):
        self.assertEqual(check_all(valid_task(), valid_grid(), valid_bins()), [])


class TestGrid(unittest.TestCase):
    def test_duplicate_id(self):
        g = valid_grid([{'id': 'c1', 'row': 0, 'col': 0, 'x0': 0, 'y0': 0, 'x1': .1, 'y1': .1},
                        {'id': 'c1', 'row': 1, 'col': 0, 'x0': .2, 'y0': 0, 'x1': .3, 'y1': .1}])
        self.assertIn('重复', ' '.join(errs(check_grid(g))))

    def test_overlap_warns(self):
        g = valid_grid([{'id': 'c1', 'row': 0, 'col': 0, 'x0': 0, 'y0': 0, 'x1': .2, 'y1': .2},
                        {'id': 'c2', 'row': 0, 'col': 1, 'x0': .1, 'y0': .1, 'x1': .3, 'y1': .3}])
        issues = check_grid(g)
        self.assertTrue(any(it['severity'] == WARN and '重叠' in it['msg'] for it in issues))

    def test_fewer_cells_than_expected_warns(self):
        g = valid_grid([{'id': 'c1', 'row': 0, 'col': 0, 'x0': 0, 'y0': 0, 'x1': .1, 'y1': .1}])
        issues = check_grid(g, valid_task(expected_total=6))
        self.assertTrue(any(it['severity'] == WARN and 'expected_total' in it['msg']
                            for it in issues))

    def test_x0_ge_x1(self):
        g = valid_grid([{'id': 'c1', 'row': 0, 'col': 0, 'x0': .2, 'y0': 0, 'x1': .1, 'y1': .1}])
        self.assertIn('x0>=x1', ' '.join(errs(check_grid(g))))


class TestBins(unittest.TestCase):
    def test_missing_target_bin(self):
        b = valid_bins(); del b['bins']['bin_mouse']
        issues = check_bins(b, valid_task())
        self.assertTrue(any('bin_mouse' in m for m in errs(issues)))

    def test_cls_mismatch(self):
        b = valid_bins(); b['bins']['bin_cup']['cls'] = 'mouse'
        issues = check_bins(b, valid_task())
        self.assertTrue(any('cls' in m for m in errs(issues)))

    def test_classes_unmapped(self):
        t = valid_task(classes=['cup', 'mouse', 'stapler'])
        issues = check_bins(valid_bins(), t)
        self.assertTrue(any('unrecognized' in m or 'stapler' in m for m in errs(issues)))


class TestCamera(unittest.TestCase):
    def test_zero_scale_error(self):
        t = valid_task()
        t['camera']['calib']['scale_px_per_m'] = [0.0, 1000.0]
        issues = check_camera(t, valid_grid(), valid_bins())
        self.assertTrue(any('0' in m and 'scale' in m for m in errs(issues)))

    def test_workspace_out_of_view_error(self):
        t = valid_task()
        t['camera']['calib']['offset_px'] = [5000.0, 5000.0]
        issues = check_camera(t, valid_grid(), valid_bins())
        self.assertTrue(any('完全看不到' in m for m in errs(issues)))


if __name__ == '__main__':
    unittest.main()
