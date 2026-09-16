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


def _load(name, config_dir=None):
    with open(os.path.join(config_dir or _CONFIG_DIR, name), encoding='utf-8') as f:
        return json.load(f)


def _issue(sev, who, msg):
    return dict(severity=sev, who=who, msg=msg)


# ---------- 真机线：EP 执行侧标定表 ----------
# 臂 moveto 的宽松上界(mm)。官方行程 水平 ~0.22m / 垂直 ~0.15m, 这里只拦明显写错的量级
# （和 ep/core/selfcheck.py 同一口径, 真正的到位值靠现场 jog 试出来）。
ARM_RANGE_MM = 500
GRIP_POWER = (1, 100)
EP_ARM_KEYS = ('grab_low_mm', 'lift_high_mm', 'release_low_mm')
# 必须实填的臂姿态。release_low_mm **不在**这里: null 它有确定含义(= 与 grab_low 同高,
# 见 config_real/ep_waypoints.json 的 _release_note), 是部署选择而不是"还没量" ——
# 料盒面高与桌面一致时本来就不该再标一个值出来。
EP_ARM_REQUIRED = ('grab_low_mm', 'lift_high_mm')
EP_JUDGE_VALUES = ('operator_confirm', 'assume_ok')


def _pose3(v):
    return isinstance(v, list) and len(v) == 3 and all(_num(x, 'p') for x in v)


def _xy2(v):
    return isinstance(v, list) and len(v) == 2 and all(_num(x, 'p') for x in v)


def check_ep_waypoints(ep, grid=None, bins=None):
    """ep_waypoints.json 结构与对齐检查（未标定项 null 合法 —— 那道闸在 check_calibration）。

    主要防的是**键对不上**：cell_pose 少一个 c6、bin_pose 拼成 bin_cup2、arm 键名写成
    grab_low（少了 _mm）。这类错不会在启动时暴露，而是跑到第 5 个物体、或者第一次换料盒时
    才当场炸 —— 现场时间最贵，提前在这里按 grid/bins 的 id 对齐一遍。
    """
    out = []
    if not isinstance(ep, dict):
        return [_issue(ERROR, 'ep', '内容不是 JSON 对象')]
    for k in ('chassis', 'arm', 'gripper', 'cell_pose', 'bin_pose', 'judge', 'calibration'):
        if k not in ep:
            out.append(_issue(ERROR, 'ep', '缺顶层键: %s' % k))
    cells = (grid or {}).get('cells') or []
    bins_d = (bins or {}).get('bins') or {}

    # 底盘位姿：每个格/每个料盒都要有键（值可以为 null = 未标定）
    for ep_key, want, who in (('cell_pose', [c.get('id') for c in cells], '格'),
                              ('bin_pose', list(bins_d.keys()), '料盒')):
        m = ep.get(ep_key)
        if not isinstance(m, dict):
            out.append(_issue(ERROR, 'ep', '%s 缺失或不是对象' % ep_key))
            continue
        for wid in want:
            if wid not in m:
                out.append(_issue(ERROR, 'ep', '%s 缺 %s %r —— 任务跑到它时会当场失败'
                                           % (ep_key, who, wid)))
            elif m[wid] is not None and not _pose3(m[wid]):
                out.append(_issue(ERROR, 'ep', '%s[%r] 应为 [x_m, y_m, z_deg] 三元组或 null: %r'
                                           % (ep_key, wid, m[wid])))
        for k in m:
            if k != '_note' and k not in want:
                out.append(_issue(WARN, 'ep', '%s 有多余键 %r（grid/bins 里没这个 id，'
                                           '拼错了？）' % (ep_key, k)))
    ch = ep.get('chassis') or {}
    if ch.get('home_pose') is not None and not _pose3(ch.get('home_pose')):
        out.append(_issue(ERROR, 'ep', 'chassis.home_pose 应为三元组或 null: %r'
                                       % (ch.get('home_pose'),)))
    if not _num(ch.get('move_tol_m'), 'tol') or not (0 < ch.get('move_tol_m', 0) < 0.2):
        out.append(_issue(WARN, 'ep', 'chassis.move_tol_m 建议 0.01..0.2（底盘开环，给太紧会'
                                      '一直补正）: %r' % ch.get('move_tol_m')))

    # 臂姿态
    am = ep.get('arm') or {}
    for k in EP_ARM_KEYS:
        v = am.get(k)
        if v is None:
            continue
        if not _xy2(v):
            out.append(_issue(ERROR, 'ep', 'arm.%s 应为 [x_mm, y_mm] 或 null: %r' % (k, v)))
            continue
        for x in v:
            if abs(x) > ARM_RANGE_MM:
                out.append(_issue(ERROR, 'ep', 'arm.%s(%smm) 超出宽松界 ±%dmm —— 超官方行程会被'
                                               '钳制' % (k, x, ARM_RANGE_MM)))
    if _xy2(am.get('grab_low_mm')) and tuple(am['grab_low_mm']) == tuple(am.get('lift_high_mm') or ()):
        out.append(_issue(ERROR, 'ep', 'arm.grab_low_mm == lift_high_mm：没有下探行程，'
                                       '两档高度要分别 jog 标定'))

    # 夹爪
    gp = ep.get('gripper') or {}
    for k in ('open_power', 'close_power'):
        v = gp.get(k)
        if not (isinstance(v, int) and not isinstance(v, bool)
                and GRIP_POWER[0] <= v <= GRIP_POWER[1]):
            out.append(_issue(ERROR, 'ep', 'gripper.%s=%r 应取整数 %d..%d'
                                           % (k, v, GRIP_POWER[0], GRIP_POWER[1])))

    # 判据
    jd = ep.get('judge') or {}
    for k in ('held', 'placed'):
        if jd.get(k) not in EP_JUDGE_VALUES:
            out.append(_issue(ERROR, 'ep', 'judge.%s=%r 应为 %s 之一（EP 无物块感知，'
                                           'assume_ok 只给离线演练）'
                                           % (k, jd.get(k), '/'.join(EP_JUDGE_VALUES))))

    cal = ep.get('calibration')
    if not isinstance(cal, dict) or cal.get('status') not in ('calibrated', 'uncalibrated'):
        out.append(_issue(ERROR, 'ep', 'calibration.status 应为 calibrated/uncalibrated: %r'
                                       % (cal or {}).get('status')))
    return out


# ---------- 真机线：未标定拒跑闸门 ----------
# 占位标记：_note 里还留着这些词 = 那一项的坐标还是从模板抄的，没人量过。
# 为什么不能只靠数值域判：占位值**是合法数值**，能过 check_grid/check_bins/check_camera 的
# 全部检查。实验二就栽在这 —— config_ep.json 里编的 A/B 恰好通过全部结构检查，"是否已标定"
# 这条从来没有真正起过作用（见 ep/core/selfcheck.py check_calibrated 的注释）。
_PLACEHOLDER_MARKS = ('示例', '待回填', 'placeholder', 'TODO')


def _markers(notes):
    """[(标签, 文本)] 里还留着占位标记的 → [标签]。"""
    return [label for label, txt in notes
            if isinstance(txt, str) and any(m in txt for m in _PLACEHOLDER_MARKS)]


def check_calibration(task, grid, bins, ep=None):
    """真机驱动前的硬闸门：识别 / 几何 / EP 三侧的现场标定都做完了吗。

    刻意与 check_all 分开，因为**仿真线必须能在未标定时照跑**（甲的场景就是从 config/ 里的
    占位几何起步的）。所以：
      - config_check.py 默认只把标定状态当**提示**打印（同 ep/core/selfcheck.py 的做法）；
      - --require-calibrated 才把它算失败 —— run_real.py 走这条，拒绝启动时连机器人都不连。

    三处**分别**记标定状态是刻意的：相机标定是乙的活、网格/料盒是现场卷尺量的、EP 位姿是
    组长 jog 出来的。分开记才知道卡在谁那里，也才能只在某一项上返工。
    """
    out = []
    # 1) 相机（乙）
    cam = (task or {}).get('camera') or {}
    calib = cam.get('calib') or {}
    ccal = cam.get('calibration')
    if not isinstance(ccal, dict):
        out.append(_issue(ERROR, 'camera',
                          'task.json camera 里没有 calibration 块 —— 这份 config 不是真机线'
                          '（config_real/）的，或者标定记录还没建'))
    else:
        if ccal.get('status') != 'calibrated':
            out.append(_issue(ERROR, 'camera',
                              'calibration.status=%r ≠ "calibrated" —— 相机标定没做完（乙）'
                              % ccal.get('status')))
        for k in ('by', 'date', 'venue'):
            if not ccal.get(k):
                out.append(_issue(ERROR, 'camera', 'calibration.%s 为空（标定人/日期/场地要'
                                                   '留档）' % k))
    mk = _markers([('camera.calib', calib.get('_note'))])
    if mk:
        out.append(_issue(ERROR, 'camera', '仍留着占位标记：%s —— 把标定值填进 calib 并删掉 '
                                           '_note（真机斜俯视支架多半要换成 mode=homography）'
                                           % '、'.join(mk)))

    # 2) 网格（现场量）
    gcal = (grid or {}).get('calibration')
    if not isinstance(gcal, dict) or gcal.get('status') != 'calibrated':
        out.append(_issue(ERROR, 'grid', 'grid_cells.json 未标定（calibration.status=%r）—— '
                                         '现场卷尺量出每格两角坐标回填 x0/y0/x1/y1'
                                         % (gcal or {}).get('status')))
    mk = _markers([('cell %s' % c.get('id'), c.get('_note'))
                   for c in ((grid or {}).get('cells') or [])])
    if mk:
        out.append(_issue(ERROR, 'grid', '仍留着占位标记：%s —— 删掉各 cell 的 _note，'
                                         '坐标没量过就别改 status' % '、'.join(mk)))

    # 3) 料盒（现场量）
    bcal = (bins or {}).get('calibration')
    if not isinstance(bcal, dict) or bcal.get('status') != 'calibrated':
        out.append(_issue(ERROR, 'bins', 'bins.json 未标定（calibration.status=%r）—— 现场量'
                                         '料盒中心坐标回填 x_m/y_m'
                                         % (bcal or {}).get('status')))
    mk = _markers([('bin %s' % b, b_.get('_note'))
                   for b, b_ in (((bins or {}).get('bins') or {}).items())])
    if mk:
        out.append(_issue(ERROR, 'bins', '仍留着占位标记：%s —— 删掉各 bin 的 _note'
                                         % '、'.join(mk)))

    # 4) EP 执行侧（组长）—— 只在这一侧能用 null 表达"没标定"
    if ep is None:
        out.append(_issue(ERROR, 'ep', '没有 ep_waypoints.json —— 真机执行侧位姿表缺失，'
                                       'run_real.py 无从知道该把 EP 开到哪'))
        return out
    ecal = ep.get('calibration')
    if not isinstance(ecal, dict) or ecal.get('status') != 'calibrated':
        out.append(_issue(ERROR, 'ep', 'ep_waypoints.json 未标定（calibration.status=%r）—— '
                                       '按 ep/标定说明.md 现场标 EP 位姿'
                                       % (ecal or {}).get('status')))
    else:
        for k in ('by', 'date', 'venue'):
            if not ecal.get(k):
                out.append(_issue(ERROR, 'ep', 'calibration.%s 为空（换场地/重新上电必须重标，'
                                               '要能查到是谁什么时候标的）' % k))
    ch = ep.get('chassis') or {}
    if ch.get('home_pose') is None:
        out.append(_issue(ERROR, 'ep', 'chassis.home_pose 仍是 null（未标定）—— 用 '
                                       'ep/drive/env_check.py --odom 读 odom 回填'))
    for ep_key in ('cell_pose', 'bin_pose'):
        miss = [k for k, v in (ep.get(ep_key) or {}).items()
                if k != '_note' and v is None]
        if miss:
            out.append(_issue(ERROR, 'ep', '%s 有 %d 个位姿还是 null：%s'
                                       % (ep_key, len(miss), '、'.join(sorted(miss)))))
    miss = [k for k in EP_ARM_REQUIRED if (ep.get('arm') or {}).get(k) is None]
    if miss:
        out.append(_issue(ERROR, 'ep', 'arm 还有未标定项：%s —— 用 '
                                       'ep/drive/env_check.py --jog 逐档试出来'
                                       % '、'.join(miss)))
    return out


def check_all_with_ep(task, grid, bins, ep=None):
    """结构/数值域全量（含 EP 位姿表）。标定闸门是另一件事 —— check_calibration。"""
    return check_all(task, grid, bins) + check_ep_waypoints(ep, grid, bins)


def main(argv=None):
    ap = argparse.ArgumentParser(description='exp3 config 回填自检')
    ap.add_argument('--config-dir', default=_CONFIG_DIR,
                    help='config 目录（仿真线 config/ | 真机线 config_real/）')
    ap.add_argument('--strict', action='store_true', help='WARN 也算失败')
    ap.add_argument('--require-calibrated', action='store_true',
                    help='标定未做完即失败 —— 真机驱动前必须过（run_real.py 同口径）')
    a = ap.parse_args(argv)

    try:
        task = _load('task.json', a.config_dir)
        grid = _load('grid_cells.json', a.config_dir)
        bins = _load('bins.json', a.config_dir)
    except FileNotFoundError as e:
        print('读 config 失败: %s（--config-dir 指定别的目录？）' % e)
        return 2
    # ep_waypoints.json 只有真机线有；仿真线目录里没有 → ep=None，标定闸门会如实报"缺失"
    ep_path = os.path.join(a.config_dir, 'ep_waypoints.json')
    ep = _load('ep_waypoints.json', a.config_dir) if os.path.exists(ep_path) else None

    issues = check_all_with_ep(task, grid, bins, ep) if ep is not None \
        else check_all(task, grid, bins)

    n_err = sum(1 for it in issues if it['severity'] == ERROR)
    for it in issues:
        print('[%s] %-6s %s' % (it['severity'], it['who'], it['msg']))
    if not issues:
        print('config 自检：全部通过（grid/bins/camera/cross 契约一致%s）'
              % ('/ep 位姿表' if ep is not None else ''))
    else:
        print('共 %d 项问题（ERROR %d / WARN %d）'
              % (len(issues), n_err, len(issues) - n_err))

    # 标定闸门只在真机线有意义：仿真线（config/）没有 ep_waypoints.json，它本来就该在
    # 占位几何上跑 —— 那种情况下不打扰。--require-calibrated 例外：那时"缺位姿表"本身
    # 就是要报出来的错。
    if ep is None and not a.require_calibrated:
        return 1 if (n_err > 0 or (a.strict and issues)) else 0

    cal = check_calibration(task, grid, bins, ep)
    if not cal:
        c = (task.get('camera') or {}).get('calibration') or {}
        print('标定状态：已标定（by=%s date=%s venue=%s）' % (c.get('by'), c.get('date'),
                                                              c.get('venue')))
    else:
        head = '未标定' if a.require_calibrated \
            else '未标定（真机线 run_real.py 会拒绝启动）'
        print('\n⚠ %s —— %d 项：' % (head, len(cal)))
        for it in cal:
            print('  [%s] %-6s %s' % (it['severity'], it['who'], it['msg']))

    fail = n_err > 0 or (a.strict and issues) or (a.require_calibrated and bool(cal))
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
