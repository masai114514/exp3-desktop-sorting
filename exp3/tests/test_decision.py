# -*- coding: utf-8 -*-
import unittest

from sort_core.decision import classify, bin_for_cls, sort_by_cell
from sort_core.taxonomy import REASON_UNRECOGNIZED, REASON_OUT_OF_GRID

BINS = {'class_to_bin': {'cup': 'bin_cup', 'mouse': 'bin_mouse'},
        'bins': {'bin_cup': {'cls': 'cup'}, 'bin_mouse': {'cls': 'mouse'}}}

C1 = {'id': 'c1', 'row': 0, 'col': 0, 'x0': 0.0, 'y0': 0.0, 'x1': 0.1, 'y1': 0.1}
C2 = {'id': 'c2', 'row': 0, 'col': 1, 'x0': 0.0, 'y0': 0.1, 'x1': 0.1, 'y1': 0.2}
CONF = 0.5


def hit(cell):
    return {'cell': cell}


class TestBin(unittest.TestCase):
    def test_mapping(self):
        self.assertEqual(bin_for_cls('cup', BINS), 'bin_cup')
        self.assertIsNone(bin_for_cls('stapler', BINS))


class TestClassify(unittest.TestCase):
    def test_executable(self):
        d = classify(hit(C1), 'cup', 0.9, CONF, BINS)
        self.assertTrue(d['executable'])
        self.assertEqual(d['cell']['id'], 'c1')
        self.assertIsNone(d['reason'])

    def test_unknown_class_keeps_cell(self):
        d = classify(hit(C2), 'stapler', 0.9, CONF, BINS)
        self.assertFalse(d['executable'])
        self.assertEqual(d['reason'], REASON_UNRECOGNIZED)
        self.assertEqual(d['cell']['id'], 'c2')   # 位置保留，便于记录/统计

    def test_low_conf_unrecognized(self):
        d = classify(hit(C1), 'mouse', 0.3, CONF, BINS)
        self.assertFalse(d['executable'])
        self.assertEqual(d['reason'], REASON_UNRECOGNIZED)

    def test_out_of_grid(self):
        d = classify({'cell': None}, 'mouse', 0.9, CONF, BINS)
        self.assertFalse(d['executable'])
        self.assertEqual(d['reason'], REASON_OUT_OF_GRID)
        self.assertIsNone(d['cell'])


class TestSort(unittest.TestCase):
    def test_row_col_order(self):
        a = {'cell': {'id': 'c2', 'row': 1, 'col': 0}, 'cls': 'cup'}
        b = {'cell': {'id': 'c1', 'row': 0, 'col': 2}, 'cls': 'mouse'}
        self.assertEqual([x['cell']['id'] for x in sort_by_cell([a, b])], ['c1', 'c2'])


if __name__ == '__main__':
    unittest.main()
