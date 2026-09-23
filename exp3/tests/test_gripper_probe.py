# -*- coding: utf-8 -*-
"""夹爪探针 real/gripper_probe.py 的离线断言（假爪，不连任何硬件）。

这个探针**存在的意义**就是"我们不知道固件的 status 从什么推出来的"，所以这里盯的是
**它会不会把三种情况认错** —— 认错的代价是答辩上承诺一条做不到的判据：

  1. **state**：空合到 closed、夹物停 normal ⇒ 判 state（最好，不需要阈值）；
  2. **time** ：终态一样但用时明显不同 ⇒ 判 time，并给一个落在两者中间的阈值；
  3. **unusable**：终态一样、用时也分不开 ⇒ **必须老实说这条判据不成立**，不许硬判；
  4. 订阅一个样本都没收到（sub_status 返回 False / 固件不推）⇒ 也要判 unusable，
     不是"样本不足"含糊过去；
  5. 轨迹的读取口径：终态是最后一个样本；`settle_time` 在"从没到过 target"时**返回 None**，
     这个 None 正是 state 与 time 的分界，不能被兜底吃掉；
  6. 标签贴反（空合 normal、夹物 closed）要能被发现。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
"""
import os
import tempfile
import unittest

from real import gripper_probe as gp

_EXP3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def trace(*pairs):
    """[('t', status), ...] → trace。"""
    t = gp.new_trace()
    for t_rel, st in pairs:
        gp.add_sample(t, t_rel, st)
    return t


class TestTraceReading(unittest.TestCase):
    def test_终态是最后一个样本(self):
        t = trace((0.03, gp.NORMAL), (0.42, gp.CLOSED))
        self.assertEqual(gp.final_status(t), gp.CLOSED)

    def test_空轨迹终态是None而不是报错(self):
        self.assertIsNone(gp.final_status(gp.new_trace()))

    def test_到终态用时取首次到达(self):
        t = trace((0.03, gp.NORMAL), (0.42, gp.CLOSED), (0.90, gp.CLOSED))
        self.assertAlmostEqual(gp.settle_time(t), 0.42, places=3)

    def test_从没到过target时用时是None而不是兜底(self):
        """夹物轨迹停在 normal ⇒ settle_time(target=closed) 必须是 None。
        这个 None 正是 state 与 time 的分界；兜底成"最后样本时间"会把两种情况抹平。"""
        t = trace((0.03, gp.NORMAL), (0.30, gp.NORMAL))
        self.assertIsNone(gp.settle_time(t, target=gp.CLOSED))
        self.assertAlmostEqual(gp.settle_time(t), 0.03, places=3)   # 到"终态"是 0.03

    def test_summarize三元组(self):
        t = trace((0.03, gp.NORMAL), (0.42, gp.CLOSED))
        self.assertEqual(gp.summarize(t), (gp.CLOSED, 0.42, 2))


class TestClassify(unittest.TestCase):
    def _empty_ok(self, n=3):
        return [trace((0.03, gp.NORMAL), (0.40 + 0.01 * i, gp.CLOSED)) for i in range(n)]

    def _obj_stall(self, n=3):
        return [trace((0.03, gp.NORMAL), (0.30, gp.NORMAL)) for i in range(n)]

    def test_假设a_空合到closed夹物停normal_判state(self):
        r = gp.classify(self._empty_ok(), self._obj_stall())
        self.assertEqual(r['verdict'], 'state')
        self.assertIsNone(r['threshold_s'])

    def test_空合到closed夹物停在opened也算状态可分(self):
        r = gp.classify(self._empty_ok(), [trace((0.03, gp.OPENED))] * 3)
        self.assertEqual(r['verdict'], 'state')

    def test_假设b_终态都是closed但用时差得开_判time并给中间阈值(self):
        empty = [trace((0.03, gp.NORMAL), (0.40, gp.CLOSED))] * 3
        obj = [trace((0.03, gp.NORMAL), (1.20, gp.CLOSED))] * 3
        r = gp.classify(empty, obj)
        self.assertEqual(r['verdict'], 'time')
        self.assertAlmostEqual(r['threshold_s'], (0.40 + 1.20) / 2.0, places=3)

    def test_终态相同且用时分不开_必须老实判unusable(self):
        """差得不够（1.2 倍 < margin 1.5）就不能判 —— 硬判等于现场瞎猜一个阈值。"""
        empty = [trace((0.03, gp.NORMAL), (0.40, gp.CLOSED))] * 3
        obj = [trace((0.03, gp.NORMAL), (0.48, gp.CLOSED))] * 3
        r = gp.classify(empty, obj)
        self.assertEqual(r['verdict'], 'unusable')
        self.assertIsNone(r['threshold_s'])

    def test_margin可调(self):
        empty = [trace((0.03, gp.NORMAL), (0.40, gp.CLOSED))] * 3
        obj = [trace((0.03, gp.NORMAL), (0.48, gp.CLOSED))] * 3
        self.assertEqual(gp.classify(empty, obj, margin=1.1)['verdict'], 'time')

    def test_一个样本都没收到_判unusable且点名订阅(self):
        """sub_status 返回 False / 固件根本不推 —— 这是个**结论**，不是"样本不足"。"""
        r = gp.classify([gp.new_trace()], self._obj_stall())
        self.assertEqual(r['verdict'], 'unusable')
        self.assertIn('订阅', r['reason'])

    def test_两组缺一组_判need_more(self):
        self.assertEqual(gp.classify([], self._obj_stall())['verdict'], 'need_more')
        self.assertEqual(gp.classify(self._empty_ok(), [])['verdict'], 'need_more')

    def test_标签贴反要被发现(self):
        """空合停在 normal、夹物到 closed —— 方向不对，不能说"状态可分"。"""
        r = gp.classify(self._obj_stall(), self._empty_ok())
        self.assertEqual(r['verdict'], 'unusable')
        self.assertIn('方向不对', r['reason'])

    def test_单次也能判_但reason里带次数以便自知样本少(self):
        r = gp.classify([trace((0.03, gp.NORMAL), (0.40, gp.CLOSED))], self._obj_stall(1))
        self.assertEqual(r['verdict'], 'state')


class TestRecorder(unittest.TestCase):
    def test_未arm之前来的样本不计入轨迹(self):
        """订阅一建立就可能立刻推来一个"当前状态"，那不是这次开合的轨迹。"""
        t = [0.0]
        rec = gp.GripRecorder(clock=lambda: t[0])
        rec(gp.OPENED)                       # 还没 arm
        self.assertEqual(len(rec.trace['samples']), 0)
        self.assertEqual(rec.n_total, 1)     # 但总接收数要如实记
        rec.arm()
        rec(gp.NORMAL)
        self.assertEqual(len(rec.trace['samples']), 1)

    def test_时间戳相对arm时刻(self):
        t = [100.0]
        rec = gp.GripRecorder(clock=lambda: t[0])
        rec.arm()
        t[0] = 100.42
        rec(gp.CLOSED)
        self.assertAlmostEqual(rec.trace['samples'][0][0], 0.42, places=3)

    def test_未知状态原样记下不丢(self):
        rec = gp.GripRecorder(clock=lambda: 0.0)
        rec.arm()
        rec('weird')
        self.assertTrue(rec.trace['samples'][0][1].startswith('unknown'))


class TestPersistence(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='probe_')

    def test_存了能读回来并按标签分组(self):
        gp.save_run(self.d, 'empty', 60, 20, 4.0, [(trace((0.4, gp.CLOSED)), gp.CLOSED, 0.4)])
        gp.save_run(self.d, 'bottle', 60, 20, 4.0, [(trace((0.3, gp.NORMAL)), gp.NORMAL, 0.3)])
        labels = gp.load_labels(self.d)
        self.assertEqual(sorted(labels), ['bottle', 'empty'])
        self.assertEqual(gp.final_status(labels['empty'][0]), gp.CLOSED)

    def test_空目录读回来是空字典而不是报错(self):
        self.assertEqual(gp.load_labels(os.path.join(self.d, 'nonexistent')), {})

    def test_power记进去了_因为换power等于换一套物理行为(self):
        gp.save_run(self.d, 'empty', 75, 20, 4.0, [(trace((0.4, gp.CLOSED)), gp.CLOSED, 0.4)])
        import json
        with open(os.path.join(self.d, 'label_empty.json'), encoding='utf-8') as f:
            self.assertEqual(json.load(f)['power'], 75)


class TestDryRun(unittest.TestCase):
    """--dry-run 的假爪必须**演示出两种不同的轨迹**，否则用户看不到输出差异。"""

    def _collect(self, label):
        import time
        rm = gp._FakeRM(label)
        rec = gp.GripRecorder()
        self.assertTrue(rm.gripper.sub_status(freq=20, callback=rec))
        rm.gripper_open()
        time.sleep(0.5)
        rec.arm()
        rm.gripper_close()
        time.sleep(0.6)
        return rec.trace

    def test_假爪empty到closed(self):
        self.assertEqual(gp.final_status(self._collect('empty')), gp.CLOSED)

    def test_假爪实物停在normal(self):
        self.assertEqual(gp.final_status(self._collect('bottle')), gp.NORMAL)

    def test_假爪两组能被判成state(self):
        r = gp.classify([self._collect('empty')], [self._collect('bottle')])
        self.assertEqual(r['verdict'], 'state')


class TestCli(unittest.TestCase):
    def test_report在没有记录时给可执行的下一步(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(SystemExit) as cm:
                gp.main(['--report', '--out-root', d])
            self.assertIn('先跑', str(cm.exception))

    def test_report只有一组时拒绝下结论(self):
        with tempfile.TemporaryDirectory() as d:
            gp.save_run(d, 'empty', 60, 20, 4.0, [(trace((0.4, gp.CLOSED)), gp.CLOSED, 0.4)])
            with self.assertRaises(SystemExit) as cm:
                gp.main(['--report', '--out-root', d])
            self.assertIn('都测完', str(cm.exception))

    def test_report两组齐了给结论(self):
        with tempfile.TemporaryDirectory() as d:
            gp.save_run(d, 'empty', 60, 20, 4.0,
                        [(trace((0.03, gp.NORMAL), (0.40, gp.CLOSED)), gp.CLOSED, 0.40)] * 3)
            gp.save_run(d, 'bottle', 60, 20, 4.0,
                        [(trace((0.03, gp.NORMAL), (0.30, gp.NORMAL)), gp.NORMAL, 0.03)] * 3)
            self.assertEqual(gp.main(['--report', '--out-root', d]), 0)

    def test_report判unusable时返回非零_免得被当成成功(self):
        with tempfile.TemporaryDirectory() as d:
            gp.save_run(d, 'empty', 60, 20, 4.0,
                        [(trace((0.03, gp.NORMAL), (0.40, gp.CLOSED)), gp.CLOSED, 0.40)] * 3)
            gp.save_run(d, 'bottle', 60, 20, 4.0,
                        [(trace((0.03, gp.NORMAL), (0.48, gp.CLOSED)), gp.CLOSED, 0.48)] * 3)
            self.assertEqual(gp.main(['--report', '--out-root', d]), 1)

    def test_不给label也不给report时报错(self):
        with self.assertRaises(SystemExit):
            gp.main([])


class TestShippedConfigIsReadable(unittest.TestCase):
    def test_能读出货真价实的config_real(self):
        ep = gp.load_ep(os.path.join(_EXP3, 'config_real'))
        self.assertIn('gripper', ep)

    def test_读不到时给的是可执行的提示(self):
        with self.assertRaises(SystemExit) as cm:
            gp.load_ep('/nonexistent-dir')
        self.assertIn('config_real', str(cm.exception))


if __name__ == '__main__':
    unittest.main()
