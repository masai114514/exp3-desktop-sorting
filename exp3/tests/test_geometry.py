# -*- coding: utf-8 -*-
import unittest

from sort_core.geometry import (bbox_center, px_to_table, point_to_cell,
                                locate, cell_center)
from sort_core import config as cfgmod

# 合成桌面：两个 0.1m 见方网格
CELLS = [
    {'id': 'c1', 'row': 0, 'col': 0, 'x0': 0.0, 'y0': 0.0, 'x1': 0.1, 'y1': 0.1},
    {'id': 'c2', 'row': 0, 'col': 1, 'x0': 0.0, 'y0': 0.1, 'x1': 0.1, 'y1': 0.2},
]
RECT = {'mode': 'rectilinear', 'offset_px': [0.0, 0.0], 'scale_px_per_m': [1000.0, 1000.0]}
IMG = (500, 500)


def bbox_at(px, py, hw=5):
    return [px - hw, py - hw, px + hw, py + hw]


class TestBbox(unittest.TestCase):
    def test_center(self):
        self.assertEqual(bbox_center([0, 0, 10, 20]), (5.0, 10.0))

    def test_cell_center(self):
        self.assertEqual(cell_center(CELLS[0]), (0.05, 0.05))


class TestRectilinear(unittest.TestCase):
    def test_px_to_table(self):
        x, y = px_to_table(410, 240, RECT)
        self.assertAlmostEqual(x, 0.410)
        self.assertAlmostEqual(y, 0.240)

    def test_scale_zero_rejected(self):
        bad = dict(RECT, scale_px_per_m=[0.0, 1000.0])
        self.assertIsNone(px_to_table(410, 240, bad))


class TestHomography(unittest.TestCase):
    def test_homography_equals_rectilinear_solution(self):
        # x=0.001*(px-320), y=0.001*(py-240)，与(320,240)+1000px/m 的正俯视解一致
        H = {'mode': 'homography', 'H': [[0.001, 0.0, -0.32],
                                         [0.0, 0.001, -0.24],
                                         [0.0, 0.0, 1.0]]}
        x, y = px_to_table(410, 240, H)
        self.assertAlmostEqual(x, 0.09)
        self.assertAlmostEqual(y, 0.0)


class TestPointToCell(unittest.TestCase):
    def test_inside(self):
        self.assertEqual(point_to_cell(0.05, 0.05, CELLS)['id'], 'c1')
        self.assertEqual(point_to_cell(0.05, 0.15, CELLS)['id'], 'c2')

    def test_outside(self):
        self.assertIsNone(point_to_cell(0.2, 0.05, CELLS))


class TestLocate(unittest.TestCase):
    def test_hit_cell(self):
        r = locate(bbox_at(50, 50), RECT, CELLS, IMG)   # c1 中心像素
        self.assertTrue(r['in_view'])
        self.assertEqual(r['cell']['id'], 'c1')

    def test_in_view_but_no_cell(self):
        r = locate(bbox_at(450, 450), RECT, CELLS, IMG)  # (0.45,0.45) 在桌面平面但无网格
        self.assertTrue(r['in_view'])
        self.assertIsNone(r['cell'])

    def test_out_of_view(self):
        r = locate(bbox_at(550, 50), RECT, CELLS, IMG)   # px 超出画面
        self.assertFalse(r['in_view'])
        self.assertIsNone(r['cell'])


class TestShippedConfigs(unittest.TestCase):
    def test_configs_load_and_have_expected_shape(self):
        c = cfgmod.load_all()
        self.assertGreaterEqual(len(c['grid']['cells']), 4)
        self.assertIn('cup', c['bins']['class_to_bin'])
        self.assertEqual(c['task']['pass_line'], 5)


if __name__ == '__main__':
    unittest.main()
