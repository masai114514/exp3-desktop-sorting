#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""exp3 config 回填自检器 —— 甲/乙回填坐标前先跑一遍，防止字段填错导致联调返工。

校验四组契约（与 sort_core / exp3 README 口径一致）：
  grid   : grid_cells.json —— id/row/col 唯一、x0<x1/y0<y1、无重叠、取放高度在
  bins   : bins.json —— class_to_bin↔bins 双向对得上、坐标在
  camera : task.json camera —— 模式/尺寸合法，且『网格+料盒要能被相机看到』
  cross  : 类名对齐（classes⊆class_to_bin）、格数 vs expected_total（同格放两物会卡 done_cells）

用法（在 exp3/ 目录）：
  python3 config_check.py             # 只读 config/*.json，有 ERROR 则退出码非 0
  python3 config_check.py --strict    # WARN 也算失败（给队友回填前当门禁）
纯函数 check_* 可在单测里喂坏配置（tests/test_config_check.py）。
"""
import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR = os.path.join(_HERE, 'config')

EPS = 1e-9
ERROR, WARN = 'ERROR', 'WARN'


# ---------- 工具 ----------
def _num(x, what):
    return isinstance(x, (int, float)) and not isinstance(x, bool) \
        and math.isfinite(x) or False


def _inv3(m):
    """3x3 矩阵求逆（H 是 px→robot；robot→px 用其逆）。奇异返回 None。"""
    a, b, c = m[0]; d, e, f = m[1]; g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < EPS:
        return None
    inv = [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
           [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
           [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]
    return inv


def _fwd_px(x, y, calib):
    """桌面(机器人系) → 像素；映射不可用返回 None。rectilinear 用 offset/scale，homography 用 H 逆。"""
    mode = calib.get('mode', 'rectilinear')
    if mode == 'rectilinear':
        cx, cy = calib['offset_px']; sx, sy = calib['scale_px_per_m']
        if abs(sx) < EPS or abs(sy) < EPS:
            return None
        return cx + sx * x, cy + sy * y
    if mode == 'homography':
        Hinv = _inv3(calib['H'])
        if Hinv is None:
            return None
        a, b, c = Hinv[0]; d, e, f = Hinv[1]; g, h, i = Hinv[2]
        w = g * x + h * y + i
        if abs(w) < EPS:
            return None
        return ((a * x + b * y + c) / w, (d * x + e * y + f) / w)
    return None


# ---------- 各组检查 ----------
def check_grid(grid, task=None):
    out = []
    cells = (grid or {}).get('cells')
    if not isinstance(cells, list) or not cells:
        return [dict(severity=ERROR, who='grid', msg='cells 缺失或为空')]
    seen_id, seen_rc, seen_xy = set(), set(), []
    expected = (task or {}).get('expected_total')
    for c in cells:
        cid = c.get('id')
        if not cid or cid in seen_id:
            out.append(dict(severity=ERROR, who='grid',
                            msg='cell id 缺失或重复: %r' % (cid,)))
        seen_id.add(cid)
        for k in ('row', 'col'):
            if not isinstance(c.get(k), int) or c.get(k) < 0:
                out.append(dict(severity=ERROR, who='grid',
                                msg='cell %s 的 %s 需为非负整数: %r' % (cid, k, c.get(k))))
        rc = (c.get('row'), c.get('col'))
        if rc in seen_rc:
            out.append(dict(severity=ERROR, who='grid',
                            msg='(row,col) 重复: %s (cell %s)' % (rc, cid)))
        seen_rc.add(rc)
        for k in ('x0', 'y0', 'x1', 'y1'):
            if not _num(c.get(k), k):
                out.append(dict(severity=ERROR, who='grid',
                                msg='cell %s 的 %s 需为数值: %r' % (cid, k, c.get(k))))
        if _num(c.get('x0'), 'x') and _num(c.get('x1'), 'x') and c['x0'] >= c['x1']:
            out.append(dict(severity=ERROR, who='grid', msg='cell %s: x0>=x1' % cid))
        if _num(c.get('y0'), 'y') and _num(c.get('y1'), 'y') and c['y0'] >= c['y1']:
            out.append(dict(severity=ERROR, who='grid', msg='cell %s: y0>=y1' % cid))
        if all(_num(c.get(k), k) for k in ('x0', 'y0', 'x1', 'y1')):
            seen_xy.append((cid, (c['x0'], c['y0'], c['x1'], c['y1'])))
    for i in range(len(seen_xy)):
        for j in range(i + 1, len(seen_xy)):
            ca, (ax0, ay0, ax1, ay1) = seen_xy[i]
            cb, (bx0, by0, bx1, by1) = seen_xy[j]
            ox = min(ax1, bx1) - max(ax0, bx0)
            oy = min(ay1, by1) - max(ay0, by0)
            if ox > EPS and oy > EPS:
                out.append(dict(severity=WARN, who='grid',
                                msg='%s 与 %s 重叠 %.3g×%.3g m（两物落重叠区会判定歧义）'
                                    % (ca, cb, ox, oy)))
    for k in ('pick_z_m', 'object_z_m'):
        v = (grid or {}).get(k)
        if not _num(v, k) or v <= 0:
            out.append(dict(severity=ERROR, who='grid',
                            msg='顶层 %s 需为正数: %r' % (k, v)))
    if expected and len(cells) < expected:
        out.append(dict(severity=WARN, who='grid',
                        msg='格数(%d) < expected_total(%d)：同格两物会被 done_cells 卡住只能取一'
                            % (len(cells), expected)))
    return out


def check_bins(bins, task=None):
    out = []
    bd = (bins or {}).get('bins')
    c2b = (bins or {}).get('class_to_bin')
    if not isinstance(bd, dict) or not bd:
        return [dict(severity=ERROR, who='bins', msg='bins 缺失或为空')]
    if not isinstance(c2b, dict):
        return [dict(severity=ERROR, who='bins', msg='class_to_bin 缺失')]
    # 双向一致：class_to_bin → bins 存在且 cls 对上
    for cls, bid in c2b.items():
        if bid not in bd:
            out.append(dict(severity=ERROR, who='bins',
                            msg='class_to_bin[%r]=%r 但 bins 里没有该料盒' % (cls, bid)))
            continue
        if bd[bid].get('cls') != cls:
            out.append(dict(severity=ERROR, who='bins',
                            msg='class_to_bin[%r]=%r 但 bins[%r].cls=%r'
                                % (cls, bid, bid, bd[bid].get('cls'))))
    # bins 几何 + 单一类别归属
    by_cls = {}
    for bid, b in bd.items():
        for k in ('x_m', 'y_m'):
            if not _num(b.get(k), k):
                out.append(dict(severity=ERROR, who='bins',
                                msg='bin %s 的 %s 需为数值: %r' % (bid, k, b.get(k))))
        by_cls.setdefault(b.get('cls'), []).append(bid)
    for cls, bids in by_cls.items():
        if len(bids) > 1:
            out.append(dict(severity=WARN, who='bins',
                            msg='多个料盒同属一类 %r: %s（分类仍成立但浪费格）' % (cls, bids)))
    # 与 task.classes 对齐
    classes = (task or {}).get('classes') or []
    if classes:
        unmapped = [c for c in classes if c not in c2b]
        if unmapped:
            out.append(dict(severity=ERROR, who='bins',
                            msg='task.classes 里无料盒映射（该类别全被判 unrecognized）: %s' % unmapped))
        extra = [c for c in c2b if c not in classes]
        if extra:
            out.append(dict(severity=WARN, who='bins',
                            msg='class_to_bin 有 classes 之外类别: %s' % extra))
    return out


def _samples(grid, bins):
    """取『必须被相机看到』的采样点：各格四角+中心 + 各料盒中心。"""
    pts = []
    for c in (grid or {}).get('cells', []):
        if all(_num(c.get(k), k) for k in ('x0', 'y0', 'x1', 'y1')):
            for (px_, py_) in ((c['x0'], c['y0']), (c['x0'], c['y1']),
                               (c['x1'], c['y0']), (c['x1'], c['y1'])):
                pts.append((px_, py_))
            pts.append(((c['x0'] + c['x1']) / 2, (c['y0'] + c['y1']) / 2))
    for b in ((bins or {}).get('bins') or {}).values():
        if _num(b.get('x_m'), 'x') and _num(b.get('y_m'), 'y'):
            pts.append((b['x_m'], b['y_m']))
    return pts


def check_camera(task, grid, bins):
    out = []
    cam = (task or {}).get('camera') or {}
    W, H = cam.get('image_width_px'), cam.get('image_height_px')
    if not _num(W, 'W') or not _num(H, 'H') or W <= 0 or H <= 0:
        return [dict(severity=ERROR, who='camera',
                     msg='camera.image_width_px/height_px 需为正数: %r/%r' % (W, H))]
    calib = cam.get('calib') or {}
    mode = calib.get('mode', 'rectilinear')
    if mode not in ('rectilinear', 'homography'):
        return [dict(severity=ERROR, who='camera', msg='未知 calib.mode: %r' % mode)]
    if mode == 'rectilinear':
        off, sc = calib.get('offset_px'), calib.get('scale_px_per_m')
        if not isinstance(off, list) or len(off) != 2 or not all(_num(v, 'off') for v in off):
            return [dict(severity=ERROR, who='camera', msg='rectilinear 需 offset_px=[cx,cy]')]
        if not isinstance(sc, list) or len(sc) != 2 or not all(_num(v, 'sc') for v in sc):
            return [dict(severity=ERROR, who='camera', msg='rectilinear 需 scale_px_per_m=[sx,sy]')]
        if abs(sc[0]) < EPS or abs(sc[1]) < EPS:
            return [dict(severity=ERROR, who='camera', msg='scale_px_per_m 含 0（px_to_table 会全部判失败）')]
        for v in sc:
            if v < 0:
                out.append(dict(severity=WARN, who='camera',
                                msg='scale_px_per_m 为负(%r)：图像会镜像，确认是有意' % v))
    else:
        Hm = calib.get('H')
        if (not isinstance(Hm, list) or len(Hm) != 3
                or any(len(r) != 3 or not all(_num(v, 'H') for v in r) for r in Hm)):
            return [dict(severity=ERROR, who='camera', msg='homography 需 H=3×3 数值')]
        if _inv3(Hm) is None:
            return [dict(severity=ERROR, who='camera', msg='H 不可逆（奇异）')]
    # 可见性：网格+料盒必须能投影进画面
    pts = _samples(grid, bins)
    inside = 0
    bad = []
    for (x, y) in pts:
        r = _fwd_px(x, y, calib)
        if r is None:
            bad.append('(%.3g,%.3g)→不可投影' % (x, y))
        else:
            (px_, py_) = r
            if 0 <= px_ <= W and 0 <= py_ <= H:
                inside += 1
            else:
                bad.append('(%.3g,%.3g)→px(%.0f,%.0f) 出画面' % (x, y, px_, py_))
    if not pts:
        return out
    if inside == 0:
        out.append(dict(severity=ERROR, who='camera',
                        msg='相机完全看不到网格/料盒：%s' % '；'.join(bad[:4])))
    elif inside < len(pts):
        out.append(dict(severity=WARN, who='camera',
                        msg='网格/料盒 %d/%d 点出画面（外点可能永远扫不到）：%s'
                            % (len(pts) - inside, len(pts), '；'.join(bad[:4]))))
    return out


def check_all(task, grid, bins):
    return (check_grid(grid, task) + check_bins(bins, task)
            + check_camera(task, grid, bins))


def _load(name):
    with open(os.path.join(_CONFIG_DIR, name), encoding='utf-8') as f:
        return json.load(f)


def main(argv=None):
    ap = argparse.ArgumentParser(description='exp3 config 回填自检')
    ap.add_argument('--config-dir', default=_CONFIG_DIR)
    ap.add_argument('--strict', action='store_true', help='WARN 也算失败')
    a = ap.parse_args(argv)

    try:
        task = _load('task.json'); grid = _load('grid_cells.json'); bins = _load('bins.json')
    except FileNotFoundError as e:
        print('读 config 失败: %s（--config-dir 指定别的目录？）' % e)
        return 2

    issues = check_all(task, grid, bins)
    if not issues:
        print('config 自检：全部通过（grid/bins/camera/cross 契约一致）')
        return 0
    n_err = sum(1 for it in issues if it['severity'] == ERROR)
    for it in issues:
        print('[%s] %-6s %s' % (it['severity'], it['who'], it['msg']))
    print('共 %d 项问题（ERROR %d / WARN %d）'
          % (len(issues), n_err, len(issues) - n_err))
    fail = n_err > 0 or (a.strict and issues)
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
