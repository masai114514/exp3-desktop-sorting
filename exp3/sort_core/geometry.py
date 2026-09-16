# -*- coding: utf-8 -*-
"""网格判定：检测框中心 → 桌面(机器人系)点 → 所在网格 cell。

标定支持两模式(task.json camera.calib)：
- 'rectilinear'：正俯视相机(仿真常见)。px = offset_x + scale_x * x_m, py = offset_y + scale_y * y_m
- 'homography' ：任意俯视/透视相机(真机常见)。H @ [px, py, 1] -> [x_m, y_m, w]，除以 w
全部为纯函数，离线可测；★甲在场景落地后把真实标定回填 config。
"""
from .taxonomy import REASON_OUT_OF_GRID


def bbox_center(bbox_xyxy):
    """bbox = [x1, y1, x2, y2]（像素）→ (px, py) 中心。"""
    x1, y1, x2, y2 = bbox_xyxy
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def px_to_table(px, py, calib):
    """像素 → 桌面(机器人系)点 (x_m, y_m)；映射失败(奇异/越界)返回 None。"""
    mode = calib.get('mode', 'rectilinear')
    if mode == 'rectilinear':
        cx, cy = calib['offset_px']
        sx, sy = calib['scale_px_per_m']
        # 防除零
        if abs(sx) < 1e-9 or abs(sy) < 1e-9:
            return None
        return (px - cx) / sx, (py - cy) / sy
    if mode == 'homography':
        H = calib['H']  # 3x3 list
        a, b, c = H[0]
        d, e, f = H[1]
        g, h, i = H[2]
        w = g * px + h * py + i
        if abs(w) < 1e-9:
            return None
        return ((a * px + b * py + c) / w, (d * px + e * py + f) / w)
    raise ValueError('unknown camera.calib.mode: %r' % mode)


def _inv3(m):
    """3x3 求逆（奇异返回 None）。与 config_check._inv3 是同一份算法 —— 那边刻意**不**
    import sort_core（自检器要能在只有 config 的情况下独立跑），所以这里不共用。"""
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        return None
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]


def table_to_px(x_m, y_m, calib):
    """桌面(机器人系)点 → 像素。px_to_table 的逆；映射不可用(奇异/退化)返回 None。

    用途是**反投影**：把"物体在哪个格"变成"它该出现在画面哪儿"。真机链路里给 mock 检测器
    及各种合成/回放用（真检测器不需要它 —— 它本来就输出像素）。
    """
    mode = calib.get('mode', 'rectilinear')
    if mode == 'rectilinear':
        cx, cy = calib['offset_px']
        sx, sy = calib['scale_px_per_m']
        if abs(sx) < 1e-9 or abs(sy) < 1e-9:
            return None
        return cx + sx * x_m, cy + sy * y_m
    if mode == 'homography':
        # calib['H'] 定义成 px→robot（见 px_to_table），所以正向投影要用它的逆
        inv = _inv3(calib['H'])
        if inv is None:
            return None
        a, b, c = inv[0]
        d, e, f = inv[1]
        g, h, i = inv[2]
        w = g * x_m + h * y_m + i
        if abs(w) < 1e-9:
            return None
        return ((a * x_m + b * y_m + c) / w, (d * x_m + e * y_m + f) / w)
    raise ValueError('unknown camera.calib.mode: %r' % mode)


def point_in_cell(x_m, y_m, cell):
    return (cell['x0'] <= x_m <= cell['x1']) and (cell['y0'] <= y_m <= cell['y1'])


def point_to_cell(x_m, y_m, cells):
    """桌面点 → 网格 dict；不在任何网格返回 None。按 config 顺序返回第一个命中。"""
    for c in cells:
        if point_in_cell(x_m, y_m, c):
            return c
    return None


def cell_center(cell):
    return ((cell['x0'] + cell['x1']) / 2.0, (cell['y0'] + cell['y1']) / 2.0)


def locate(bbox_xyxy, calib, cells, image_size):
    """检测框 → 定位结果。

    返回 dict：
      {cell: dict|None, x_m: float|None, y_m: float|None, in_view: bool}
    - 框中心出画面/映射失败 → in_view False, cell None（调用方记 out_of_grid）
    - 在画面但不在任何网格 → cell None（reason=out_of_grid）
    """
    w, h = image_size
    px, py = bbox_center(bbox_xyxy)
    if not (0 <= px <= w and 0 <= py <= h):
        return {'cell': None, 'x_m': None, 'y_m': None, 'in_view': False}
    xy = px_to_table(px, py, calib)
    if xy is None:
        return {'cell': None, 'x_m': None, 'y_m': None, 'in_view': False}
    x_m, y_m = xy
    return {'cell': point_to_cell(x_m, y_m, cells), 'x_m': x_m, 'y_m': y_m, 'in_view': True}
