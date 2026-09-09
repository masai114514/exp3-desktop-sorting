# -*- coding: utf-8 -*-
"""真实 config 上的端到端场景断言（跑 examples/run_demo 的逻辑，不依赖 ROS）。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
注意：读的是 config/*.json 现值；若甲改几何/物体数使这些场景语义变掉，这里需同步。
"""
import tempfile
import unittest

from examples.run_demo import run_once
from sort_core.taxonomy import (VERDICT_PASS, REASON_NO_TARGET, REASON_SAFETY_STOP,
                                REASON_NO_EXEC, REASON_UNRECOGNIZED, REASON_OUT_OF_GRID,
                                REASON_GRASP_FAILED)


class TestHappy(unittest.TestCase):
    def test_all_sorted_pass(self):
        with tempfile.TemporaryDirectory() as d:
            s, v = run_once('happy', d)
        self.assertEqual(v, VERDICT_PASS)
        self.assertEqual(s['exit_status'], REASON_NO_TARGET)
        self.assertEqual(s['placed_ok'], 6)
        self.assertEqual(s['objects_seen'], 6)


class TestRecovery(unittest.TestCase):
    def test_fail_once_then_ok_still_pass(self):
        with tempfile.TemporaryDirectory() as d:
            s, v = run_once('recovery', d)
        self.assertEqual(v, VERDICT_PASS)
        self.assertEqual(s['placed_ok'], 6)
        # 每物先 1 次 grasp_failed 再成功 → 6 次失败但全部救回
        self.assertEqual(s['reasons'].get(REASON_GRASP_FAILED), 6)


class TestSafetyStop(unittest.TestCase):
    def test_consecutive_fail_hits_limit(self):
        with tempfile.TemporaryDirectory() as d:
            s, v = run_once('safety_stop', d)
        self.assertEqual(s['exit_status'], REASON_SAFETY_STOP)
        self.assertEqual(s['placed_ok'], 0)
        self.assertEqual(s['reasons'].get(REASON_GRASP_FAILED), 3)  # fail_limit


class TestUnrecognizedOnly(unittest.TestCase):
    def test_no_exec_when_all_unknown(self):
        with tempfile.TemporaryDirectory() as d:
            s, v = run_once('unrecognized', d)
        self.assertEqual(s['exit_status'], REASON_NO_EXEC)
        # 6 个不同 (cell,cls) 各记一次跳过
        self.assertEqual(s['reasons'].get(REASON_UNRECOGNIZED), 6)


class TestOutOfGrid(unittest.TestCase):
    def test_no_exec_when_nothing_in_grid(self):
        with tempfile.TemporaryDirectory() as d:
            s, v = run_once('out_of_grid', d)
        self.assertEqual(s['exit_status'], REASON_NO_EXEC)
        self.assertGreaterEqual(s['reasons'].get(REASON_OUT_OF_GRID, 0), 1)


if __name__ == '__main__':
    unittest.main()
