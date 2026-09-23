#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验3 相机标定工具（乙那份）—— 把「摆参考点 → 读像素 → 解 H」变成一条命令。

    python3 real/camera_calib.py --init --n 5                    # 建一份点位模板
    python3 real/camera_calib.py --grab calib_frame.jpg          # 抓一帧存盘（架好相机后先做这个）
    python3 real/camera_calib.py --pick                          # 在图上点出参考点 → 写 px/py
    # 把卷尺量出来的 x_m/y_m 填进那份 json（4 个角 + 至少 1 个校验点）
    python3 real/camera_calib.py --solve                         # 解 H，报误差
    python3 real/camera_calib.py --solve --write --by 乙 --venue 实验室   # 通过就写进 task.json
    python3 real/camera_calib.py --verify                        # ★标定后复验：现在还对不对得上

## 这一步在整条链路上的位置

`config_real/task.json` 的 `camera.calib` 是 `sort_core/geometry.px_to_table` 的输入 ——
检测框中心(像素) → 桌面坐标(米) → 落在哪个格子。**它错了，每一格都会偏**，而现象看起来像
"底盘漂了"。四份标定里只有这一份是**几何**，不是读数。

## H 的方向（别搞反）

本工具解出的 H 与 `geometry.px_to_table` 同向：**像素 → 桌面(机器人系)**。

    [x_m, y_m, w] = H @ [px, py, 1]      （再各自除以 w）

`table_to_px`（反投影）内部用的是它的逆，不用你操心。

## 为什么至少要 4 个点、为什么建议 5 个

H 有 8 个自由度，**4 个点刚好 8 条约束 ⇒ 残差恒为 0**，"误差"这个数在 4 点下**测不出来**。
所以点 5 个（网格纸四角 + 一个中间点）时，工具会报真正的重投影误差，并且给留一交叉验证(LOCV)。
4 个点也能跑，但工具会明确告诉你**这份 H 的误差没被验证过** —— 别把"0 误差"当质量证明。

## 判据

`config_real/task.json` 的 `calibration.note` 写的是「反投影误差 ≤ 半个格子宽」。
所以默认容差 = `grid_cells.json` 里格子宽的一半（量不到就用 3cm），可用 `--tol-m` 覆盖。
超容差不写盘（除非 `--force`）—— 与 `calibrate.py` 拒绝记录被钳制的臂读数是同一个态度：
**宁可挡住，也别把错的数写进去**，写进去之后闸门是拦不住的（它只查"填了没有"）。
"""
import argparse
import datetime
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXP3 = os.path.dirname(_HERE)
for p in (_EXP3, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from sort_core.geometry import px_to_table, table_to_px      # noqa: E402

DEFAULT_TOL_M = 0.03          # 量不到格子宽时的兜底：3cm
MIN_POINTS = 4                # H 的 8 个自由度：4 点是最低下限（残差恒 0）
GOOD_POINTS = 5               # ≥5 点误差才可测


# ---------------------------------------------------------------- 路径与读写
def points_path(config_dir):
    return os.path.join(config_dir, 'camera_calib_points.json')


def task_path(config_dir):
    return os.path.join(config_dir, 'task.json')


def _load_json(p):
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def _save_json(p, obj):
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
        f.write('\n')


def load_points(path):
    """读点位文件并做**结构性**校验（不算数）。返回 (obj, 已填满的点列表)。

    只校验"形状对不对"，不判断"数准不准" —— 后者是人的事（卷尺量的）。
    """
    if not os.path.isfile(path):
        raise SystemExit('读不到点位文件 %s\n（先跑 --init 建一份，再 --pick 点像素）' % path)
    obj = _load_json(path)
    pts = obj.get('points') or []
    if len(pts) < MIN_POINTS:
        raise SystemExit('点位文件里只有 %d 个点，H 至少要 %d 个（建议 %d 个：四角 + 1 个校验点）'
                         % (len(pts), MIN_POINTS, GOOD_POINTS))
    ready, pending = [], []
    for i, p in enumerate(pts, 1):
        ok = all(p.get(k) is not None for k in ('x_m', 'y_m', 'px', 'py'))
        (ready if ok else pending).append((i, p))
    return obj, ready, pending


def _point_signature(ready):
    """点集的几何签名，用来判断"这份 H 是不是这对点解出来的"。"""
    return [[round(float(p['px']), 3), round(float(p['py']), 3),
             round(float(p['x_m']), 6), round(float(p['y_m']), 6)] for _, p in ready]


# ---------------------------------------------------------------- DLT
def _require_numpy():
    try:
        import numpy as np
    except ImportError:
        raise SystemExit('需要 numpy —— pip3 install numpy（装 opencv-python / ultralytics 时已带上）')
    return np


def _normalize(np, pts):
    """Hartley 归一化：把质心移到原点、均方距离缩到 sqrt(2)。

    不做这一步的话，像素坐标在几百的量级、桌面坐标在十分之一的量级，
    DLT 的矩阵条件数会很难看 —— 4 个点的小系统照样能解出"看起来对"的 H，
    但一多几个点就开始飘。
    """
    c = pts.mean(axis=0)
    d = float(np.sqrt(((pts - c) ** 2).sum(axis=1)).mean())
    s = (math.sqrt(2.0) / d) if d > 1e-12 else 1.0
    T = np.array([[s, 0.0, -s * c[0]],
                  [0.0, s, -s * c[1]],
                  [0.0, 0.0, 1.0]], dtype=float)
    return T, (pts - c) * s


def solve_h(pairs):
    """点对 → H（像素 → 桌面，3×3、row-major、h33 归一到 1）。

    pairs: [(px, py, x_m, y_m), ...]，至少 4 个。
    """
    np = _require_numpy()
    if len(pairs) < MIN_POINTS:
        raise SystemExit('解 H 至少要 %d 个点，现在只有 %d 个' % (MIN_POINTS, len(pairs)))

    src = np.array([[float(p[0]), float(p[1])] for p in pairs], dtype=float)
    dst = np.array([[float(p[2]), float(p[3])] for p in pairs], dtype=float)

    T_src, src_n = _normalize(np, src)
    T_dst, dst_n = _normalize(np, dst)

    rows = []
    for (px, py), (x, y) in zip(src_n, dst_n):
        rows.append([-px, -py, -1.0, 0.0, 0.0, 0.0, x * px, x * py, x])
        rows.append([0.0, 0.0, 0.0, -px, -py, -1.0, y * px, y * py, y])
    A = np.array(rows, dtype=float)

    # 最小奇异值对应的右奇异向量 = A h ≈ 0 的最小二乘解
    _, _, vt = np.linalg.svd(A)
    Hn = vt[-1].reshape(3, 3)

    H = np.linalg.inv(T_dst) @ Hn @ T_src
    if abs(H[2, 2]) < 1e-12:
        raise SystemExit('解出的 H 退化（h33≈0）—— 参考点是不是共线 / 重复了？')
    H = H / H[2, 2]
    return [[float(v) for v in row] for row in H]


def _calib_from_H(H):
    return {'mode': 'homography', 'H': H}


def reprojection_errors(pairs, H):
    """逐点误差。返回 list of dict(表->像 px 误差, 像->表 m 误差)。

    两个方向都报，因为"错多少像素"和"错多少米"不是一回事：同一个像素误差在画面
    近端和远端对应的实际距离不同。**判据用米**（闸门/格子是按米判的）。
    """
    calib = _calib_from_H(H)
    out = []
    for px, py, x_m, y_m in pairs:
        got_px = table_to_px(float(x_m), float(y_m), calib)
        err_px = (math.hypot(got_px[0] - float(px), got_px[1] - float(py))
                  if got_px else float('inf'))
        got_t = px_to_table(float(px), float(py), calib)
        err_m = (math.hypot(got_t[0] - float(x_m), got_t[1] - float(y_m))
                 if got_t else float('inf'))
        out.append({'err_px': err_px, 'err_m': err_m})
    return out


def loocv_errors(pairs):
    """留一交叉验证：每次拿掉一个点、用其余点解 H，再预测被拿掉的那个点。

    ≥5 点时才有意义（4 点解 H 刚好定解，留一后剩 3 点欠定）。
    这是对"这份 H 到了第 5、第 6 个位置还准不准"最诚实的估计 ——
    全点重投影误差会随点数增加而变小，看起来总是比实际好。
    """
    if len(pairs) < GOOD_POINTS:
        return None
    errs = []
    for i in range(len(pairs)):
        rest = pairs[:i] + pairs[i + 1:]
        H = solve_h(rest)
        e = reprojection_errors([pairs[i]], H)[0]
        errs.append(e)
    return errs


# ---------------------------------------------------------------- 容差
def default_tol_m(config_dir):
    """默认容差 = 半个格子宽（task.json 的 calibration.note 口径）。量不到就用 3cm。"""
    try:
        grid = _load_json(os.path.join(config_dir, 'grid_cells.json'))
        widths = [abs(float(c['x1']) - float(c['x0'])) for c in (grid.get('cells') or [])
                  if c.get('x0') is not None and c.get('x1') is not None]
        if widths:
            widths.sort()
            half = widths[len(widths) // 2] / 2.0
            return half, '半个格子宽（格子 %.1f cm）' % (widths[len(widths) // 2] * 100)
    except (IOError, ValueError, KeyError, TypeError):
        pass
    return DEFAULT_TOL_M, '兜底 %.0f cm（量不到格子宽）' % (DEFAULT_TOL_M * 100)


# ---------------------------------------------------------------- 动作
def cmd_init(config_dir, path, n):
    if os.path.isfile(path) and not _overwrite_ok(path):
        return 1
    n = max(int(n), MIN_POINTS)
    obj = {
        '_note': ('参考点：桌面上 4 个（或更多）已知点。x_m/y_m 用卷尺量，'
                  'px/py 用 --pick 在图上点。建议四角 + 1 个中间点：'
                  '4 个点时 H 是恰好定解，残差恒为 0，误差**测不出来**。'),
        'image': 'calib_frame.jpg',
        'image_size': None,
        'points': [{'label': 'P%d' % (i + 1), 'x_m': None, 'y_m': None,
                    'px': None, 'py': None} for i in range(n)],
    }
    _save_json(path, obj)
    print('已建点位模板：%s（%d 个点）' % (path, n))
    print('下一步：')
    print('  1) 架好相机（架完**不许再动**）→ python3 real/camera_calib.py --grab calib_frame.jpg')
    print('  2) python3 real/camera_calib.py --pick      # 在图上按顺序点出这几个点')
    print('  3) 卷尺量出各点的桌面坐标，填进 json 的 x_m/y_m')
    print('  4) python3 real/camera_calib.py --solve')
    return 0


def _overwrite_ok(path):
    ans = input('%s 已存在，覆盖？(y/n) > ' % path).strip().lower()
    return ans in ('y', 'yes')


def cmd_grab(camera_spec, out_path, size):
    from real.camera import open_camera
    cam = open_camera(camera_spec, size)
    try:
        frame = cam.grab()
    finally:
        cam.close()
    import cv2
    if not cv2.imwrite(out_path, frame):
        raise SystemExit('写不了图片：%s' % out_path)
    h, w = frame.shape[:2]
    print('已抓帧：%s  (%dx%d)' % (out_path, w, h))
    print('★ 这一帧就是标定基准。架完相机**不许再动** —— 地面场地碰一下相机，'
          '格子不动相机动，H 当场作废且没有报警（见 1_现场SOP.md §2）。')
    return 0


def cmd_pick(image_path, path, obj, n_expected):
    """在图上点参考点 → 写进 px/py。有 GUI 就用窗口点；没有就退回手输像素。"""
    try:
        import cv2
    except ImportError:
        raise SystemExit('--pick 需要 opencv —— pip3 install opencv-python')
    img = cv2.imread(image_path)
    if img is None:
        raise SystemExit('读不了图片 %s（先 --grab 抓一帧？）' % image_path)
    h, w = img.shape[:2]
    pts = obj.setdefault('points', [])
    n = int(n_expected or len(pts) or MIN_POINTS)
    while len(pts) < n:
        pts.append({'label': 'P%d' % (len(pts) + 1), 'x_m': None, 'y_m': None,
                    'px': None, 'py': None})

    clicked = []
    try:
        win = 'camera_calib: click %d points (r=reset, q/enter=done)' % n
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < n:
                clicked.append((float(x), float(y)))

        cv2.setMouseCallback(win, on_mouse)
        print('在窗口里按顺序点 %d 个点（点完按 q 或 Enter 结束，r 重来）。' % n)
        while True:
            vis = img.copy()
            for i, (x, y) in enumerate(clicked):
                cv2.drawMarker(vis, (int(x), int(y)), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(vis, pts[i].get('label') or ('P%d' % (i + 1)),
                            (int(x) + 8, int(y) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(vis, 'clicked %d/%d' % (len(clicked), n), (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(win, vis)
            k = cv2.waitKey(20) & 0xFF
            if k == ord('r'):
                clicked[:] = []
            elif k in (ord('q'), 13, 27):
                break
        cv2.destroyAllWindows()
    except Exception as e:                                        # 没有显示环境
        print('窗口点选不可用（%s）。改用**手输像素**。' % e)
        print('提示：把图放进能看像素坐标的工具（Preview/PS/画图）里，读出 x y 再敲进来。')
        clicked = []
        for i in range(n):
            s = input('  %s 的像素 (px py，留空跳过) > ' % (pts[i].get('label') or ('P%d' % (i + 1))))
            if not s.strip():
                continue
            try:
                x, y = (float(v) for v in s.replace(',', ' ').split()[:2])
            except ValueError:
                print('    没读懂，跳过这个点。')
                continue
            clicked.append((x, y))

    for i, (x, y) in enumerate(clicked[:n]):
        pts[i]['px'], pts[i]['py'] = round(x, 2), round(y, 2)
        print('  %-4s px=%8.2f  py=%8.2f' % (pts[i].get('label') or ('P%d' % (i + 1)), x, y))

    obj['image'] = os.path.basename(image_path)
    obj['image_size'] = [int(w), int(h)]
    _save_json(path, obj)
    print('\n已写入 %s（%d 个点的像素坐标）' % (path, len(clicked[:n])))
    print('★ 别忘了把卷尺量出来的 x_m/y_m 填进去 —— 只有 px/py 是解不出 H 的。')
    return 0


def _report(pairs, H, tol_m, label='重投影误差'):
    errs = reprojection_errors(pairs, H)
    print('\n%s（逐点）：' % label)
    print('  %-5s %10s %12s' % ('点', '表→像(px)', '像→表(cm)'))
    for i, e in enumerate(errs, 1):
        print('  %-5s %10.2f %12.2f' % ('P%d' % i, e['err_px'], e['err_m'] * 100))
    worst = max(e['err_m'] for e in errs)
    rms_m = math.sqrt(sum(e['err_m'] ** 2 for e in errs) / len(errs))
    rms_px = math.sqrt(sum(e['err_px'] ** 2 for e in errs) / len(errs))
    print('  %-5s %10.2f %12.2f' % ('RMS', rms_px, rms_m * 100))
    return errs, worst, rms_m


def cmd_solve(config_dir, path, tol_m_override, write, by, venue, date, force):
    obj, ready, pending = load_points(path)
    if pending:
        print('★ 还有 %d 个点没填全（%s）—— 先补齐再解。'
              % (len(pending), '、'.join(p.get('label') or ('P%d' % i) for i, p in pending)))
        return 1

    pairs = [(p['px'], p['py'], p['x_m'], p['y_m']) for _, p in ready]
    _warn_degenerate(pairs)
    H = solve_h(pairs)

    tol_m, why = (tol_m_override, '--tol-m 指定') if tol_m_override else default_tol_m(config_dir)
    print('参考点 %d 个，H 的 8 个自由度%s。' %
          (len(pairs), '有 %d 个冗余' % (len(pairs) * 2 - 8) if len(pairs) > MIN_POINTS else '刚好定解'))
    errs, worst, rms_m = _report(pairs, H, tol_m)

    if len(pairs) < GOOD_POINTS:
        print('\n⚠ 只有 %d 个点：H 恰好定解（8 约束 = 8 自由度）⇒ 上面的残差**恒为 0**，'
              '\n  它证明不了精度。想要一个能看的误差数，再加 1 个点（第 5 个）。' % len(pairs))
    else:
        loo = loocv_errors(pairs)
        worst_loo = max(e['err_m'] for e in loo)
        rms_loo = math.sqrt(sum(e['err_m'] ** 2 for e in loo) / len(loo))
        print('\n留一交叉验证（拿掉一点、用其余点解、预测被拿掉的）：')
        print('  最差 %.2f cm    RMS %.2f cm' % (worst_loo * 100, rms_loo * 100))
        print('  ← 这个数才是"到了没参与解算的位置还准不准"；全点 RMS 会随点数变小，偏乐观。')

    print('\n判据：%s（容差 %.2f cm，%s）' % ('反投影误差 ≤ 半个格子宽', tol_m * 100, why))
    ok = worst <= tol_m

    if not ok:
        print('\n✗ 最差 %.2f cm > 容差 %.2f cm。**不写盘。**' % (worst * 100, tol_m * 100))
        print('  常见原因：① 像素点选错点（点的不是同一个角）② 卷尺量错/记错单位（cm 当成 m）')
        print('           ③ 点基本共线（H 退化，看有没有退化告警）④ 相机在抓帧后动过')
        if not force:
            return 1
        print('  （--force 已给，继续写盘 —— 想清楚：错的 H 会让**每一格**都偏，'
              '且闸门拦不住它。）')

    if not write:
        print('\n✓ 通过。要写进 config_real/task.json：')
        print('  python3 real/camera_calib.py --solve --write --by <你的名字> --venue <场地>')
        return 0 if ok else 1

    if not by or not venue:
        raise SystemExit('--write 要 --by <名字> --venue <场地> —— 四份记录分开署名，'
                         '出问题才知道卡在谁那里')
    date = date or datetime.date.today().isoformat()
    task = _load_json(task_path(config_dir))
    cam = task.setdefault('camera', {})
    calib = cam.setdefault('calib', {})
    old = {k: calib.get(k) for k in ('mode',)}
    calib.pop('_note', None)                       # 占位标记必须删掉（闸门靠它认"没回填"）
    calib['mode'] = 'homography'
    calib['H'] = H
    for k in ('offset_px', 'scale_px_per_m'):      # rectilinear 的字段留着只会让人看错模式
        calib.pop(k, None)
    task['camera']['calibration'] = {
        'status': 'calibrated', 'by': by, 'date': date, 'venue': venue,
        # 点集签名：--verify 用它判断"这批点是不是当初解 H 的那批"。
        # 不存它的话，改了点位文件再 --verify 会拿旧 H 对新点，报出来的误差没法解释。
        'points_signature': _point_signature(ready),
        'note': ('H 由 real/camera_calib.py 解出；参考点 %d 个；'
                 '最差反投影 %.2f cm、RMS %.2f cm（容差 %.2f cm）。'
                 '原点是 %s 上量的 %d 个点。'
                 % (len(pairs), worst * 100, rms_m * 100, tol_m * 100,
                    os.path.basename(path), len(pairs))),
    }
    _save_json(task_path(config_dir), task)
    print('\n已写盘：%s' % task_path(config_dir))
    print('  camera.calib.mode = homography（原 %s）' % (old.get('mode'),))
    print('  camera.calib.H    = 3×3（已删 _note / rectilinear 字段）')
    print('  camera.calibration = calibrated  by=%s  date=%s  venue=%s' % (by, date, venue))
    print('\n下一步：')
    print('  python3 config_check.py --config-dir config_real --require-calibrated   # 四份一起查')
    print('  ※ 这份只解决"像素↔桌面"。**识别认不认得出**是另一件事 —— '
          '跑一轮 python3 real/vision_check.py 看目标在画面里的等效像素宽（域探针的结论：'
          '等效 80–120px 才稳，整桌取景时 mouse 只有 0–20%）。')
    return 0


def _warn_degenerate(pairs):
    """点在桌面上共线 / 太集中 ⇒ H 病态。提前说，别等解出来才从误差里猜。"""
    xs = [float(p[2]) for p in pairs]
    ys = [float(p[3]) for p in pairs]
    span_x, span_y = max(xs) - min(xs), max(ys) - min(ys)
    if min(span_x, span_y) < 0.01:
        print('⚠ 参考点在桌面上几乎是**一条线**（x 跨度 %.3fm，y 跨度 %.3fm）—— '
              'H 会病态。用四角，别用同一边上的几个点。' % (span_x, span_y))


def cmd_verify(config_dir, path, tol_m_override, force):
    """★标定后复验：相机被碰过没有报警，所以跑之前自己查一遍。

    `1_现场SOP.md` §2.5 记的那条待办（"我们工具链里还没有这一步"）就是这个命令：
    拿**同一批参考点**重投影，看 H 还对不对得上。对不上 ⇒ 相机动了，重标再跑。
    """
    obj, ready, pending = load_points(path)
    task = _load_json(task_path(config_dir))
    calib = (task.get('camera') or {}).get('calib') or {}
    if calib.get('mode') != 'homography' or not calib.get('H'):
        print('task.json 里还没有 homography 的 H（当前 mode=%r）。' % calib.get('mode'))
        print('先解一份：python3 real/camera_calib.py --solve --write --by <名字> --venue <场地>')
        return 1

    pairs = [(p['px'], p['py'], p['x_m'], p['y_m']) for _, p in ready]
    tol_m, why = (tol_m_override, '--tol-m 指定') if tol_m_override else default_tol_m(config_dir)
    errs, worst, rms_m = _report(pairs, calib['H'], tol_m, label='当前 H 的重投影误差')
    n_ok = sum(1 for e in errs if e['err_m'] <= tol_m)
    print('\nmarkers=%d/%d（容差 %.2f cm，%s）' % (n_ok, len(errs), tol_m * 100, why))

    saved = ((task.get('camera') or {}).get('calibration') or {}).get('points_signature')
    if saved is not None and saved != _point_signature(ready):
        print('⚠ 这批点和写盘时的不是同一批（点位文件被改过）—— '
              '上面的数分不清"相机动了"和"参考点挪了"，仅供参考。')
        print('  要判断相机有没有被碰：把这批点**原样**（同一个文件、同一批数）再跑一次 --verify。')

    if worst <= tol_m:
        print('✓ 相机还对得上这张 H —— 可以用它开跑。')
        return 0
    print('\n✗ 对不上了（最差 %.2f cm）。**先别跑。**' % (worst * 100))
    print('  地面场地最常见的解释：**相机被碰过/支架被挪过** —— 格子不动相机动，'
          'H 当场作废，而且没有任何提示（现象是"每格都偏一点"，很容易误判成底盘漂了）。')
    print('  做法：重新 --grab → --pick → 对卷尺坐标复核 → --solve --write。')
    print('  确认 H 没变、只是参考点被移动过（纸被踢跑）：把参考点重新摆回原处再 --pick。')
    return 1 if not force else 0


# ---------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(
        description='实验3 相机 H（单应）标定工具 —— 像素 → 桌面坐标（乙那份标定）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='参考点：≥%d 个（建议 %d：四角 + 1 个校验点）。4 点时 H 恰好定解、误差测不出来。'
               % (MIN_POINTS, GOOD_POINTS))
    ap.add_argument('--config-dir', default=os.path.join(_EXP3, 'config_real'))
    ap.add_argument('--points', default='', help='点位文件（默认 <config-dir>/camera_calib_points.json）')
    ap.add_argument('--init', action='store_true', help='建一份点位模板')
    ap.add_argument('--n', type=int, default=GOOD_POINTS, help='--init 建几个点（默认 5）')
    ap.add_argument('--grab', metavar='OUT.jpg', help='抓一帧存盘（架好相机后第一件事）')
    ap.add_argument('--camera', default='cv2:0', help='配 --grab：none | cv2:0 | image:/path')
    ap.add_argument('--pick', action='store_true', help='在图上点出参考点 → 写 px/py')
    ap.add_argument('--image', default='', help='配 --pick：图片路径（默认取点位文件里的 image）')
    ap.add_argument('--solve', action='store_true', help='解 H 并报误差')
    ap.add_argument('--write', action='store_true', help='配 --solve：通过后写进 config_real/task.json')
    ap.add_argument('--verify', action='store_true', help='★标定后复验：现有 H 还对得上吗')
    ap.add_argument('--tol-m', type=float, default=None, help='容差（米），默认半个格子宽')
    ap.add_argument('--by', default='', help='配 --write：标定人')
    ap.add_argument('--venue', default='', help='配 --write：场地')
    ap.add_argument('--date', default='', help='配 --write：日期（默认今天）')
    ap.add_argument('--force', action='store_true', help='超容差也写盘（想清楚再开）')
    args = ap.parse_args(argv)

    path = args.points or points_path(args.config_dir)

    if args.init:
        return cmd_init(args.config_dir, path, args.n)

    if args.grab:
        try:
            task = _load_json(task_path(args.config_dir))
            size = (int(task['camera']['image_width_px']), int(task['camera']['image_height_px']))
        except (IOError, KeyError, ValueError, TypeError):
            size = (640, 480)
        return cmd_grab(args.camera, args.grab, size)

    if args.pick:
        obj = _load_json(path) if os.path.isfile(path) else {'points': []}
        img = args.image or os.path.join(os.path.dirname(os.path.abspath(path)),
                                         obj.get('image') or '')
        if not args.image and not obj.get('image'):
            raise SystemExit('--pick 要指一张图：--image <图>（或先在点位文件里写 image 字段）')
        return cmd_pick(img, path, obj, args.n)

    if args.verify:
        return cmd_verify(args.config_dir, path, args.tol_m, args.force)

    if args.solve:
        return cmd_solve(args.config_dir, path, args.tol_m, args.write,
                         args.by, args.venue, args.date or None, args.force)

    ap.print_help()
    print('\n常用顺序：--init → --grab → --pick →（填 x_m/y_m）→ --solve → --solve --write')
    return 0


if __name__ == '__main__':
    sys.exit(main())
