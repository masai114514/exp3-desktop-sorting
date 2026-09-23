# -*- coding: utf-8 -*-
"""取景/识别自检 real/vision_check.py 的离线断言（不连相机、不加载模型）。

这个工具是现场开跑前的**最后一道人工闸门** —— 它的判词直接决定"这个取景能不能开工"。
判词说错的方向只有两个，且都会造成实实在在的损失：

  - **把危险说成能跑**：整桌取景下 mouse 召回 0–20%（域探针实测），跑了就是大面积
    `unrecognized_object`，一轮白跑，还得重来。
  - **把能量化的东西说成"感觉不对"**：现场没有数，就只能靠猜高度和距离。

所以这里把**判据本身**钉住：三个区间（<60 / 60–80 / ≥80）的边界、"有效检出"必须按
`task.conf_min` 过滤（低于它的会被判 unrecognized，不算数）、以及"一个都没检出"要单独成判词。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
"""
import unittest

from real import vision_check as vc


def row(cls, conf, w, h=None):
    return {'cls': cls, 'conf': conf, 'w': w, 'h': h if h is not None else w * 0.6}


def row_net(cls, conf, w_net, w_raw=None):
    """带网络等效宽的检出（真实的调用方长这样）。"""
    return {'cls': cls, 'conf': conf, 'w': w_raw if w_raw is not None else w_net * 3,
            'h': 10, 'w_net': w_net}


class TestNetWidth(unittest.TestCase):
    """等效像素宽的折算。域探针的判据是"网络实际看到多少像素"，不是原图里多少像素。

    ultralytics predict 会把整帧缩到 imgsz=640（长边对齐）。所以 640 宽的行车相机
    （现场唯一正常情形）折算系数恒为 1；而一张 1920×1080 的照片会被缩到 1/3 ——
    不折的话，工具会把一个"送进网络只剩三分之一"的取景报成"宽裕"。
    """

    def test_640x480相机折算系数为1(self):
        self.assertAlmostEqual(vc.frame_scale((640, 480)), 1.0)
        self.assertAlmostEqual(vc.net_width(100, (640, 480)), 100.0)

    def test_大图按长边折算(self):
        # 1920×1080 → 长边 1920 → 缩到 640（系数 1/3）
        self.assertAlmostEqual(vc.frame_scale((1920, 1080)), 640.0 / 1920.0)
        self.assertAlmostEqual(vc.net_width(300, (1920, 1080)), 100.0)

    def test_竖图按高度折算(self):
        # 480×640 → 长边 640 → 系数 1
        self.assertAlmostEqual(vc.net_width(200, (480, 640)), 200.0)

    def test_退化尺寸不炸(self):
        self.assertEqual(vc.net_width(50, (0, 0)), 50.0)
        self.assertEqual(vc.frame_scale((0, 0)), 1.0)

    def test_折算会真的改变判词(self):
        """原框 300px 在 1920 宽的图里，网络只看到 100px —— 判词从"宽裕"变"稳"。"""
        self.assertEqual(vc.judge_width(300)[0], '✓')
        self.assertEqual(vc.judge_width(vc.net_width(300, (1920, 1080)))[0], '✓')
        # 原框 150px 折算后只剩 50px ⇒ 落入危险区
        self.assertEqual(vc.judge_width(150)[0], '✓')
        self.assertEqual(vc.judge_width(vc.net_width(150, (1920, 1080)))[0], '✗')


class TestJudgeWidth(unittest.TestCase):
    def test_三个区间的边界(self):
        self.assertEqual(vc.judge_width(None)[0], '?')
        self.assertEqual(vc.judge_width(0.0)[0], '✗')
        self.assertEqual(vc.judge_width(59.9)[0], '✗')
        self.assertEqual(vc.judge_width(vc.W_DANGER)[0], '!')          # 60 起算"边缘"
        self.assertEqual(vc.judge_width(79.9)[0], '!')
        self.assertEqual(vc.judge_width(vc.W_OK)[0], '✓')              # 80 起算"稳"
        self.assertEqual(vc.judge_width(119.9)[0], '✓')
        self.assertEqual(vc.judge_width(vc.W_PLENTY)[0], '✓')

    def test_宽裕区会提醒别裁出画外(self):
        mark, why = vc.judge_width(200)
        self.assertEqual(mark, '✓')
        self.assertIn('裁', why)

    def test_危险区的判词点明这是mouse那个量级(self):
        mark, why = vc.judge_width(40)
        self.assertEqual(mark, '✗')
        self.assertIn('认不出来', why)


class TestSummarize(unittest.TestCase):
    def test_有效检出按conf_min过滤(self):
        rows = [row('cup', 0.9, 100), row('cup', 0.3, 100), row('cup', 0.8, 90)]
        s = vc.summarize(rows, conf_min=0.5)
        self.assertEqual(s['cup']['n'], 3)
        self.assertEqual(s['cup']['n_ok'], 2)                 # 0.3 那条不算数
        self.assertAlmostEqual(s['cup']['median_px'], 95.0)   # 中位数只取有效的两条

    def test_全部低于conf_min就是一次有效检出都没有(self):
        rows = [row('mouse', 0.2, 120), row('mouse', 0.31, 130)]
        s = vc.summarize(rows, conf_min=0.5)
        self.assertEqual(s['mouse']['n_ok'], 0)
        self.assertEqual(s['mouse']['mark'], '✗')
        self.assertIn('一次有效检出都没有', s['mouse']['verdict'])

    def test_按类分开统计(self):
        rows = [row('cup', 0.9, 100), row('mouse', 0.9, 50)]
        s = vc.summarize(rows, conf_min=0.5)
        self.assertEqual(set(s), {'cup', 'mouse'})
        self.assertEqual(s['cup']['mark'], '✓')
        self.assertEqual(s['mouse']['mark'], '✗')

    def test_空输入(self):
        self.assertEqual(vc.summarize([], 0.5), {})

    def test_优先用网络等效宽(self):
        """同一条检出：原框 300px，但图是 1920 宽的 ⇒ 该按 100px 判，不是 300px。"""
        r = row_net('cup', 0.9, w_net=vc.net_width(300, (1920, 1080)), w_raw=300)
        s = vc.summarize([r], conf_min=0.5)
        self.assertAlmostEqual(s['cup']['median_px'], 100.0)

    def test_没有等效宽就退回原框宽(self):
        s = vc.summarize([row('cup', 0.9, 95)], conf_min=0.5)
        self.assertAlmostEqual(s['cup']['median_px'], 95.0)


class TestOverall(unittest.TestCase):
    def test_一个都没检出(self):
        mark, why = vc.overall_verdict({})
        self.assertEqual(mark, '✗')
        self.assertIn('没检出', why)

    def test_有一类危险就别开跑(self):
        s = vc.summarize([row('cup', 0.9, 100), row('mouse', 0.9, 30)], conf_min=0.5)
        mark, why = vc.overall_verdict(s)
        self.assertEqual(mark, '✗')
        self.assertIn('mouse', why)
        self.assertIn('别开跑', why)

    def test_有一类边缘要给继续收窄的建议(self):
        s = vc.summarize([row('cup', 0.9, 100), row('mouse', 0.9, 70)], conf_min=0.5)
        mark, why = vc.overall_verdict(s)
        self.assertEqual(mark, '!')
        self.assertIn('收窄', why)

    def test_两类都稳才算通过(self):
        s = vc.summarize([row('cup', 0.9, 110), row('mouse', 0.9, 95)], conf_min=0.5)
        mark, why = vc.overall_verdict(s)
        self.assertEqual(mark, '✓')
        self.assertIn('绿色区', why)

    def test_只看有效检出_低分框救不了场(self):
        """mouse 检出 10 个但全部 conf<0.5 ⇒ 结论仍必须是不能跑。"""
        rows = [row('cup', 0.9, 110)] + [row('mouse', 0.35, 120) for _ in range(10)]
        s = vc.summarize(rows, conf_min=0.5)
        mark, _ = vc.overall_verdict(s)
        self.assertEqual(mark, '✗')


class TestCalibUsable(unittest.TestCase):
    def test_homography有H才算可用(self):
        self.assertTrue(vc._calib_usable({'mode': 'homography',
                                          'H': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}))
        self.assertFalse(vc._calib_usable({'mode': 'homography'}))
        self.assertFalse(vc._calib_usable({'mode': 'homography', 'H': [[1, 0], [0, 1]]}))

    def test_task_json里那份rectilinear占位要被认出来(self):
        self.assertFalse(vc._calib_usable({'mode': 'rectilinear',
                                           'offset_px': [320.0, 240.0],
                                           'scale_px_per_m': [1000.0, 1000.0]}))

    def test_真的rectilinear算可用(self):
        self.assertTrue(vc._calib_usable({'mode': 'rectilinear',
                                          'offset_px': [318.0, 239.0],
                                          'scale_px_per_m': [1120.0, 1105.0]}))

    def test_空的或坏形状一律不可用(self):
        for bad in (None, {}, [], {'mode': 'weird'}):
            self.assertFalse(vc._calib_usable(bad), bad)


class TestCellOf(unittest.TestCase):
    """格号那一栏：标定好了要能算出落在哪一格，坏输入不能把工具打挂。"""

    CELLS = [{'id': 'c1', 'x0': 0.0, 'y0': 0.0, 'x1': 0.1, 'y1': 0.1}]
    CALIB = {'mode': 'rectilinear', 'offset_px': [0.0, 0.0], 'scale_px_per_m': [1000.0, 1000.0]}

    def test_框中心落在格里(self):
        bbox = [40, 40, 60, 60]          # 中心 (50,50) px → (0.05,0.05) m → c1
        self.assertEqual(vc._cell_of(bbox, self.CALIB, self.CELLS, (500, 500)), 'c1')

    def test_在画面里但不在任何格(self):
        bbox = [200, 200, 220, 220]      # (0.21,0.21) m → 不在 c1
        self.assertEqual(vc._cell_of(bbox, self.CALIB, self.CELLS, (500, 500)), 'out_of_grid')

    def test_框中心出画(self):
        bbox = [-40, -40, -20, -20]
        self.assertEqual(vc._cell_of(bbox, self.CALIB, self.CELLS, (500, 500)), 'out_of_frame')

    def test_坏calib不抛异常只给横杠(self):
        self.assertEqual(vc._cell_of([1, 2, 3, 4], {'mode': 'nope'}, self.CELLS, (500, 500)), '-')


class TestConstants(unittest.TestCase):
    def test_边界常量与域探针一致(self):
        """改这三个数就是改判据 —— 钉住它们，改动必须是有意的。"""
        self.assertEqual((vc.W_DANGER, vc.W_OK, vc.W_PLENTY), (60.0, 80.0, 120.0))

    def test_推理门槛与run_real一致(self):
        self.assertEqual(vc.CONF_DEFAULT, 0.25)


if __name__ == '__main__':
    unittest.main()
