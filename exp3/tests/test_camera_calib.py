# -*- coding: utf-8 -*-
"""相机 H 标定工具 real/camera_calib.py 的离线断言（不碰相机、不碰硬件）。

这个工具填的是**四份标定里唯一一份几何**（其余三份是读数）。它错了，检测框算出来的
格号就整体错 —— 而现象看起来像"底盘漂了"，排查方向会被带偏。所以这里盯的是：

  1. **H 的方向不能反**：解出来必须与 `geometry.px_to_table` 同向（像素 → 桌面）。
     反了的话 4 个点照样能"解出来"，但每一格都镜像 —— 单看误差是看不出来的。
  2. **回环要准**：给一批点 → 解 H → 用 H 定位，必须回到原点（合成数据下是 1e-9 级）。
  3. **4 个点的残差恒为 0**，这件事必须被**说出来**（工具会警告），
     因为"0 误差"很容易被当成质量证明。
  4. **超容差不写盘**：宁可挡住，也不把错的 H 写进 task.json ——
     写进去之后闸门拦不住它（闸门只查"填了没有"）。
  5. **写盘要顺手把占位清掉**：`calib._note` 必须删、rectilinear 字段必须挪干净，
     否则闸门会一直点这份 config。

跑法：cd exp3 && python3 -m unittest discover -s tests -t . -v
"""
import contextlib
import io
import json
import math
import os
import tempfile
import unittest

from real import camera_calib as cc
from sort_core.geometry import px_to_table, table_to_px

try:
    import numpy                                        # noqa: F401
    HAS_NUMPY = True
except ImportError:                                     # pragma: no cover
    HAS_NUMPY = False

# 一份"像样"的真值 H（像素 → 桌面）：640x480 的画面映到 ~0.15m×0.15m 的作业面 + 一点透视
H_TRUE = [[2.4e-4, 1.0e-5, -0.075],
          [7.0e-6, -2.1e-4, 0.095],
          [1.2e-7, 1.8e-7, 1.0]]
CALIB_TRUE = {'mode': 'homography', 'H': H_TRUE}

# 桌面上的参考点（米）：四角 + 两个中间点（≥5 点才有可测误差）
TABLE_PTS = [(-0.020, -0.040), (-0.020, 0.100), (0.060, -0.040),
             (0.060, 0.100), (0.020, 0.030), (0.040, -0.010)]


def pairs_from_truth(pts=None):
    """真值 H → [(px, py, x_m, y_m)]（用 table_to_px 反投影出像素）。"""
    out = []
    for x, y in (pts or TABLE_PTS):
        px, py = table_to_px(x, y, CALIB_TRUE)
        out.append((px, py, x, y))
    return out


def write_config(d, points_list, tol_cell=0.10):
    """搭一个最小的 config_real/：task.json（带占位 calib）+ grid_cells.json + 点位文件。"""
    task = {
        'classes': ['cup', 'mouse'], 'conf_min': 0.5,
        'camera': {
            'image_width_px': 640, 'image_height_px': 480,
            'calib': {'mode': 'rectilinear', '_note': '★示例占位, 待回填(乙)。',
                      'offset_px': [320.0, 240.0], 'scale_px_per_m': [1000.0, 1000.0]},
            'calibration': {'status': 'uncalibrated', 'by': None, 'date': None, 'venue': None,
                            'note': '乙负责。'},
        },
    }
    grid = {'cells': [{'id': 'c1', 'x0': 0.0, 'y0': 0.0, 'x1': tol_cell, 'y1': tol_cell}],
            'calibration': {'status': 'uncalibrated'}}
    tp = os.path.join(d, 'task.json')
    gp = os.path.join(d, 'grid_cells.json')
    pp = os.path.join(d, 'camera_calib_points.json')
    with open(tp, 'w', encoding='utf-8') as f:
        json.dump(task, f, ensure_ascii=False, indent=2)
    with open(gp, 'w', encoding='utf-8') as f:
        json.dump(grid, f, ensure_ascii=False, indent=2)
    with open(pp, 'w', encoding='utf-8') as f:
        json.dump({'image': 'calib_frame.jpg', 'image_size': [640, 480],
                   'points': [{'label': 'P%d' % (i + 1), 'px': px, 'py': py,
                               'x_m': x, 'y_m': y} for i, (px, py, x, y) in enumerate(points_list)]},
                  f, ensure_ascii=False, indent=2)
    return tp, gp, pp


def read_json(p):
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def write_json(p, obj):
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


def run(argv):
    """跑 CLI 并吞掉 stdout（现场那套彩色输出不必进测试日志）。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = cc.main(argv)
    return rc, buf.getvalue()


@unittest.skipUnless(HAS_NUMPY, '需要 numpy（装 opencv-python / ultralytics 时已带上）')
class TestSolve(unittest.TestCase):
    def test_至少4个点才能解(self):
        with self.assertRaises(SystemExit):
            cc.solve_h(pairs_from_truth(TABLE_PTS[:3]))

    def test_h33归一为1(self):
        H = cc.solve_h(pairs_from_truth())
        self.assertAlmostEqual(H[2][2], 1.0, places=12)

    def test_方向不能反_像素到桌面(self):
        """★ 核心断言：解出的 H 必须与 geometry.px_to_table 同向。

        把方向搞反（桌面→像素）的 H 也能"解出来"，但每一格会镜像 ——
        误差表上照样可能是 0。所以这里逐点验回环，而不是只看残差。
        """
        pairs = pairs_from_truth()
        H = cc.solve_h(pairs)
        for px, py, x_m, y_m in pairs:
            gx, gy = px_to_table(px, py, {'mode': 'homography', 'H': H})
            self.assertAlmostEqual(gx, x_m, places=9)
            self.assertAlmostEqual(gy, y_m, places=9)

    def test_与真值H逐元素接近(self):
        H = cc.solve_h(pairs_from_truth())
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(H[i][j], H_TRUE[i][j], places=9)

    def test_反投影残差在噪声下仍然小(self):
        """给像素加 0.5px 抖动（就是手点鼠标的量级），误差应该还在毫米级。"""
        pairs = []
        for k, (px, py, x, y) in enumerate(pairs_from_truth()):
            pairs.append((px + 0.5 * ((-1) ** k), py + 0.5, x, y))
        H = cc.solve_h(pairs)
        errs = cc.reprojection_errors(pairs, H)
        self.assertLess(max(e['err_m'] for e in errs), 0.005)     # < 5mm

    def test_4个点的残差恒为0_这才说明误差测不出来(self):
        pairs = pairs_from_truth(TABLE_PTS[:4])
        errs = cc.reprojection_errors(pairs, cc.solve_h(pairs))
        self.assertLess(max(e['err_m'] for e in errs), 1e-9)

    def test_小于5点不给LOOCV(self):
        self.assertIsNone(cc.loocv_errors(pairs_from_truth(TABLE_PTS[:4])))
        self.assertEqual(len(cc.loocv_errors(pairs_from_truth())), len(TABLE_PTS))

    def test_LOOCV在真值点上接近0(self):
        loo = cc.loocv_errors(pairs_from_truth())
        self.assertLess(max(e['err_m'] for e in loo), 1e-9)

    def test_共线点会告警(self):
        line = [(0.0, 0.0), (0.02, 0.0), (0.04, 0.0), (0.03, 0.0)]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            cc._warn_degenerate(pairs_from_truth(line))
        self.assertIn('一条线', buf.getvalue())


@unittest.skipUnless(HAS_NUMPY, '需要 numpy')
class TestTolerance(unittest.TestCase):
    def test_默认容差是半个格子宽(self):
        with tempfile.TemporaryDirectory() as d:
            write_config(d, pairs_from_truth(), tol_cell=0.10)
            tol, why = cc.default_tol_m(d)
            self.assertAlmostEqual(tol, 0.05)
            self.assertIn('格子', why)

    def test_量不到格子就兜底(self):
        with tempfile.TemporaryDirectory() as d:
            tol, why = cc.default_tol_m(d)
            self.assertAlmostEqual(tol, cc.DEFAULT_TOL_M)
            self.assertIn('兜底', why)


@unittest.skipUnless(HAS_NUMPY, '需要 numpy')
class TestWriteAndVerify(unittest.TestCase):
    def test_写盘清掉占位并填齐留档字段(self):
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            rc, out = run(['--config-dir', d, '--points', pp, '--solve', '--write',
                           '--by', '乙', '--venue', '实验室'])
            self.assertEqual(rc, 0, out)
            task = read_json(tp)
            calib = task['camera']['calib']
            self.assertEqual(calib['mode'], 'homography')
            self.assertEqual(len(calib['H']), 3)
            self.assertNotIn('_note', calib)                  # 闸门靠它认"没回填"
            self.assertNotIn('offset_px', calib)              # rectilinear 字段留着会让人看错模式
            self.assertNotIn('scale_px_per_m', calib)
            ccal = task['camera']['calibration']
            self.assertEqual(ccal['status'], 'calibrated')
            self.assertEqual((ccal['by'], ccal['venue']), ('乙', '实验室'))
            self.assertTrue(ccal['date'])
            self.assertIn('points_signature', ccal)

    def test_写盘后闸门不再点camera那一项(self):
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            run(['--config-dir', d, '--points', pp, '--solve', '--write',
                 '--by', '乙', '--venue', '实验室'])
            import config_check
            task = read_json(tp)
            grid = read_json(gp)
            out = config_check.check_calibration(task, grid, {})
            cam = [i for i in out if i.get('who') == 'camera']
            self.assertEqual(cam, [], 'camera 那一项不该再有 issue：%s' % cam)

    def test_超容差拒绝写盘(self):
        with tempfile.TemporaryDirectory() as d:
            bad = list(pairs_from_truth())
            bad[0] = (bad[0][0] + 40.0, bad[0][1], bad[0][2] + 0.12, bad[0][3])   # 手点歪了
            tp, gp, pp = write_config(d, bad, tol_cell=0.04)
            with open(tp, encoding='utf-8') as f:
                before = f.read()
            rc, out = run(['--config-dir', d, '--points', pp, '--solve', '--write',
                           '--by', '乙', '--venue', '实验室'])
            self.assertEqual(rc, 1)
            self.assertIn('不写盘', out)
            with open(tp, encoding='utf-8') as f:
                self.assertEqual(f.read(), before)     # 文件没被动过

    def test_verify通过时报markers(self):
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            run(['--config-dir', d, '--points', pp, '--solve', '--write',
                 '--by', '乙', '--venue', '实验室'])
            rc, out = run(['--config-dir', d, '--points', pp, '--verify'])
            self.assertEqual(rc, 0, out)
            self.assertIn('markers=6/6', out)

    def test_verify发现相机被动过(self):
        """SOP §2.5 那条待办：地面场地相机被碰**没有报警**，只能自己复验。

        这里模拟"相机被挪了一下" —— 挪相机是**整幅画面平移**，所以所有点的像素一起动。
        位移要够大才越得过默认容差（半个格子宽）：本组合成数据里 1px ≈ 0.24mm，
        所以 ~210px 才等于 5cm。**这条恰好说明默认容差是"格内能不能站对"的口径，
        不是"相机有没有被碰"的口径** —— 想更灵敏就 `--tol-m` 收紧。
        """
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            run(['--config-dir', d, '--points', pp, '--solve', '--write',
                 '--by', '乙', '--venue', '实验室'])
            obj = read_json(pp)
            for p in obj['points']:
                p['px'] += 250.0                                  # 整幅平移 = 相机动了
            write_json(pp, obj)
            rc, out = run(['--config-dir', d, '--points', pp, '--verify'])
            self.assertEqual(rc, 1)
            self.assertIn('先别跑', out)
            self.assertIn('相机被碰', out)

    def test_verify可以用tol_m收紧灵敏度(self):
        """默认容差偏松（半个格子宽）。想抓"轻微被碰"就自己收紧。"""
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            run(['--config-dir', d, '--points', pp, '--solve', '--write',
                 '--by', '乙', '--venue', '实验室'])
            obj = read_json(pp)
            for p in obj['points']:
                p['px'] += 40.0                                   # ≈9.6mm
            write_json(pp, obj)
            rc, out = run(['--config-dir', d, '--points', pp, '--verify', '--tol-m', '0.005'])
            self.assertEqual(rc, 1, out)
            self.assertIn('markers=', out)

    def test_verify会提示点位文件被改过(self):
        """signature 不匹配 ⇒ 上面的数仅供参考，不能拿来判断相机动没动。"""
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            run(['--config-dir', d, '--points', pp, '--solve', '--write',
                 '--by', '乙', '--venue', '实验室'])
            obj = read_json(pp)
            obj['points'][3]['x_m'] += 0.001                      # 只挪参考点、不动相机
            write_json(pp, obj)
            rc, out = run(['--config-dir', d, '--points', pp, '--verify'])
            self.assertIn('不是同一批', out)

    def test_verify在还没H时给出下一步(self):
        with tempfile.TemporaryDirectory() as d:
            tp, gp, pp = write_config(d, pairs_from_truth())
            rc, out = run(['--config-dir', d, '--points', pp, '--verify'])
            self.assertEqual(rc, 1)
            self.assertIn('还没有 homography', out)

    def test_点位没填全时不写盘(self):
        with tempfile.TemporaryDirectory() as d:
            pts = pairs_from_truth()
            tp, gp, pp = write_config(d, pts)
            obj = read_json(pp)
            obj['points'][2]['x_m'] = None
            write_json(pp, obj)
            rc, out = run(['--config-dir', d, '--points', pp, '--solve', '--write',
                           '--by', '乙', '--venue', '实验室'])
            self.assertEqual(rc, 1)
            self.assertIn('没填全', out)


@unittest.skipUnless(HAS_NUMPY, '需要 numpy')
class TestPointFile(unittest.TestCase):
    def test_点数不足直接拒绝(self):
        with tempfile.TemporaryDirectory() as d:
            pp = os.path.join(d, 'p.json')
            write_json(pp, {'points': [{'px': 1, 'py': 2, 'x_m': 0.1, 'y_m': 0.1}]})
            with self.assertRaises(SystemExit):
                cc.load_points(pp)

    def test_init建的模板是空的但形状对(self):
        with tempfile.TemporaryDirectory() as d:
            pp = os.path.join(d, 'p.json')
            rc, out = run(['--config-dir', d, '--points', pp, '--init', '--n', '5'])
            self.assertEqual(rc, 0)
            obj = read_json(pp)
            self.assertEqual(len(obj['points']), 5)
            for p in obj['points']:
                self.assertIsNone(p['px'])
                self.assertIsNone(p['x_m'])

    def test_init的点数下限是4(self):
        with tempfile.TemporaryDirectory() as d:
            pp = os.path.join(d, 'p.json')
            run(['--config-dir', d, '--points', pp, '--init', '--n', '2'])
            self.assertEqual(len(read_json(pp)['points']), 4)

    def test_签名只跟数有关(self):
        p = {'px': 1.0, 'py': 2.0, 'x_m': 0.1, 'y_m': 0.2}
        self.assertEqual(cc._point_signature([(1, p)]), [[1.0, 2.0, 0.1, 0.2]])

    def test_签名能分辨改过的点(self):
        a = {'px': 1.0, 'py': 2.0, 'x_m': 0.1, 'y_m': 0.2}
        b = dict(a, px=1.5)
        self.assertNotEqual(cc._point_signature([(1, a)]), cc._point_signature([(1, b)]))


class TestMathHelpers(unittest.TestCase):
    """不依赖 numpy 的部分。"""

    def test_单位H下方向是像素直通(self):
        """H=I ⇒ 桌面坐标逐位等于像素。这条把"两个方向"钉住：
        若哪天把 H 反着用（桌面→像素），这里的 err_m / err_px 会当场对不上。"""
        H = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        pts = [(1.0, 2.0, 1.0, 2.0), (5.0, 6.0, 0.1, 0.2)]
        e = cc.reprojection_errors(pts, H)
        self.assertAlmostEqual(e[0]['err_px'], 0.0)
        self.assertAlmostEqual(e[0]['err_m'], 0.0)
        self.assertAlmostEqual(e[1]['err_px'], math.hypot(4.9, 5.8), places=9)
        self.assertAlmostEqual(e[1]['err_m'], math.hypot(4.9, 5.8), places=9)


if __name__ == '__main__':
    unittest.main()
