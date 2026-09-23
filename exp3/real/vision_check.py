#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验3 取景 + 识别自检（现场开跑前**必跑一次**）—— 回答一个问题：

    **在这个取景下，best.pt 到底认不认得出桌上的东西？**

    python3 real/vision_check.py                       # 用 USB 相机连抓 10 帧
    python3 real/vision_check.py --frames 20           # 多抓几帧看稳不稳
    python3 real/vision_check.py --camera image:test.jpg   # 用一张存图（在家先试布置）
    python3 real/vision_check.py --out frames/         # 存带框的标注图，肉眼复核

## 为什么需要这个工具

`exp3_domain_probe/SUMMARY_zoom.md`（2026-09-21 实测、960 行合成帧）的结论**很硬**：

| 网络看到的等效像素宽 | cup 召回 | mouse 召回 |
|---|---|---|
| 40px（整桌入镜） | 90% | **0–20%** |
| 60px | 100% | 10–30% |
| 80px | 100% | 20–40% |
| 100px | 100% | **30–60%** |

⇒ **mouse 在整桌取景下基本认不出来**；而且**事后裁切/放大救不回来**（信息在降采样时就没了，
`probe_crop.py` 证明了这一点）。唯一有效的手段是**采集时就让目标占更多像素** ——
把取景收窄到只框住工作区。

问题是：**这件事没法在家拍板**（取决于现场架多高、多远、工作区多大）。
所以现场必须有一个 30 秒就能跑完的检查，而不是等跑完一轮看日志才发现 mouse 全丢。

## 它报什么

- 每个检出：类别 / 置信度 / **像素宽高** / 框中心 /（标定好的话）落在哪一格
- 按类汇总：检出数、**有效检出数**（conf ≥ `task.conf_min`，低于它的会被判成
  `unrecognized_object`，所以这才是"真正算数的那些"）、等效像素宽的中位数
- 结论：按域探针的三个区间（**<60 危险 / 60–80 边缘 / ≥80 稳**）逐类给判词
- 标注图存到 `--out`（默认为 `vision_check_frames/`），肉眼复核框对不对

## 它不做什么

不连 EP、不过标定闸门、不改任何 config —— 纯只读，随时可跑。
（相机没标定也能跑：格号那一栏会显示 `-`，其余照报。）
"""
import argparse
import datetime
import json
import os
import statistics
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXP3 = os.path.dirname(_HERE)
for p in (_EXP3, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

from sort_core.geometry import locate                            # noqa: E402

# 域探针的三个区间（等效像素宽，单位 px）。改这里 = 改判据，所以写成常量并注明出处。
W_DANGER = 60.0        # < 60：整桌取景那个量级，mouse 基本认不出
W_OK = 80.0            # ≥ 80：主表里的"绿色区"起点
W_PLENTY = 120.0       # ≥ 120：宽裕（但取景太紧会把物体裁出画外，SUMMARY_zoom 的下半张表）
CONF_DEFAULT = 0.25    # 与 run_real.py 的 YoloDetector 一致：压低门槛，让低分框也发得出来
IMGSZ = 640            # ultralytics predict 的默认输入边长 —— 网络**实际**看到的是缩到这个尺寸的图


# ---------------------------------------------------------------- 判据（纯函数，可离线测）
def net_width(w, frame_size, imgsz=IMGSZ):
    """框的原始像素宽 → **网络实际看到的**等效像素宽。

    为什么必须折这一下：`model.predict(frame)` 内部会把整帧 letterbox 到 imgsz（默认 640，
    长边对齐）。所以能喂给网络的信息量由"物体在**缩放后**那张图里占多少像素"决定，
    不是原图里的像素数。域探针的主表就是按 640 宽的合成帧报的，两边口径必须一致 ——
    否则一张 1920×1080 的照片会被报成"宽裕"，实际送进网络只剩三分之一。

    正常现场（相机出 640×480）这一项恒等于 1，折算不改任何数。
    """
    h, w_frame = int(frame_size[1]), int(frame_size[0])
    long_side = max(h, w_frame)
    if long_side <= 0:
        return float(w)
    return float(w) * float(imgsz) / float(long_side)


def frame_scale(frame_size, imgsz=IMGSZ):
    h, w_frame = int(frame_size[1]), int(frame_size[0])
    long_side = max(h, w_frame)
    return (float(imgsz) / float(long_side)) if long_side > 0 else 1.0
def judge_width(median_px):
    """等效像素宽 → (记号, 判词)。"""
    if median_px is None:
        return '?', '没有有效检出'
    if median_px < W_DANGER:
        return '✗', '太小（<%.0fpx）——整桌取景那个量级，这个类很可能整轮都认不出来' % W_DANGER
    if median_px < W_OK:
        return '!', '边缘（%.0f–%.0fpx）——时好时坏，别指望它稳' % (W_DANGER, W_OK)
    if median_px < W_PLENTY:
        return '✓', '稳（≥%.0fpx，域探针的绿色区）' % W_OK
    return '✓', '宽裕（≥%.0fpx）——注意取景是否已把物体裁出画外' % W_PLENTY


def summarize(rows, conf_min):
    """逐类汇总。

    rows: [{'cls':.., 'conf':.., 'w_net':.., ...}, ...]（一次前向一条）
    返回 {cls: {'n','n_ok','widths_ok','median_px','mark','verdict'}}

    ★ 统计的是 **w_net**（网络实际看到的等效宽，见 net_width），不是原框宽 ——
    判据来自域探针的 640 宽合成帧，口径必须一致。没有 w_net 就退回原框宽（便于单测）。
    """
    out = {}
    for r in rows:
        d = out.setdefault(r['cls'], {'n': 0, 'n_ok': 0, 'widths_ok': []})
        d['n'] += 1
        if r['conf'] >= conf_min:
            d['n_ok'] += 1
            d['widths_ok'].append(r.get('w_net', r['w']))
    for cls, d in out.items():
        med = statistics.median(d['widths_ok']) if d['widths_ok'] else None
        mark, verdict = judge_width(med)
        if d['n_ok'] == 0:
            mark, verdict = '✗', '一次有效检出都没有（conf 全部 < %.2f）' % conf_min
        d['median_px'] = med
        d['mark'] = mark
        d['verdict'] = verdict
    return out


def overall_verdict(per_class):
    """整轮判词：两类都要能稳才算"可以开跑"。"""
    if not per_class:
        return '✗', '一个物体都没检出 —— 取景/对焦/相机没开？'
    bad = [c for c, d in per_class.items() if d['mark'] == '✗']
    edge = [c for c, d in per_class.items() if d['mark'] == '!']
    if bad:
        return '✗', ('这些类在这个取景下认不出来：%s —— **别开跑**。'
                     '把相机拉近/抬低到只框住工作区（目标占到 80–120px）再验一次。'
                     % '、'.join(sorted(bad)))
    if edge:
        return '!', ('这些类是边缘状态：%s —— 先把取景再收窄一点，或者先接受"这几类会掉"。'
                     % '、'.join(sorted(edge)))
    return '✓', '两类都在域探针的绿色区 —— 取景可以用。'


# ---------------------------------------------------------------- 动作
def _load_cfg(config_dir):
    def rd(name):
        p = os.path.join(config_dir, name)
        try:
            with open(p, encoding='utf-8') as f:
                return json.load(f)
        except IOError:
            return {}
    return rd('task.json'), rd('grid_cells.json')


def _calib_usable(calib):
    """这份 calib 能不能用来算格号（占位值不算）。"""
    if not isinstance(calib, dict):
        return False
    if calib.get('mode') == 'homography':
        H = calib.get('H')
        return isinstance(H, list) and len(H) == 3 and all(len(r) == 3 for r in H)
    off, sc = calib.get('offset_px'), calib.get('scale_px_per_m')
    return (isinstance(off, list) and isinstance(sc, list) and len(off) == 2 and len(sc) == 2
            and not (sc == [1000.0, 1000.0] and off == [320.0, 240.0]))   # 就是 task.json 那个占位


def _cell_of(bbox, calib, cells, size):
    try:
        r = locate(bbox, calib, cells, size)
    except (ValueError, KeyError, TypeError):
        return '-'
    if not r.get('in_view'):
        return 'out_of_frame'
    return r['cell']['id'] if r.get('cell') else 'out_of_grid'


def _annotate(cv2, frame, rows):
    vis = frame.copy()
    for r in rows:
        x1, y1, x2, y2 = [int(v) for v in r['bbox']]
        color = (0, 200, 0) if r['conf'] >= r.get('conf_min', 0) else (0, 165, 255)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        label = '%s %.2f %dpx' % (r['cls'], r['conf'], int(r['w']))
        cv2.putText(vis, label, (x1, max(14, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return vis


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='实验3 取景 + 识别自检：best.pt 在这个取景下认不认得出？',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config-dir', default=os.path.join(_EXP3, 'config_real'))
    ap.add_argument('--camera', default='cv2:0', help='none 不适用 | cv2:0 | image:/path')
    ap.add_argument('--model', default='',
                    help='best.pt 路径（默认找 <仓库>/../weights/best.pt 与 <仓库>/weights/best.pt）')
    ap.add_argument('--frames', type=int, default=10, help='抓几帧（默认 10；image: 模式下忽略）')
    ap.add_argument('--conf', type=float, default=CONF_DEFAULT, help='推理 conf 门槛（默认 0.25）')
    ap.add_argument('--out', default='', help='标注图输出目录（默认 ./vision_check_frames）')
    ap.add_argument('--no-save', action='store_true', help='不存图')
    args = ap.parse_args(argv)

    task, grid = _load_cfg(args.config_dir)
    cam_cfg = (task.get('camera') or {})
    size = (int(cam_cfg.get('image_width_px') or 640), int(cam_cfg.get('image_height_px') or 480))
    conf_min = float(task.get('conf_min') or 0.5)
    cells = grid.get('cells') or []
    calib = cam_cfg.get('calib') or {}
    usable = _calib_usable(calib)

    try:
        import cv2
    except ImportError:
        raise SystemExit('需要 opencv —— pip3 install opencv-python')

    from real.camera import open_camera, YoloDetector

    if not args.model:
        repo = os.path.dirname(_EXP3)
        for cand in (os.path.join(repo, 'weights', 'best.pt'),
                     os.path.join(_EXP3, 'weights', 'best.pt'),
                     os.path.join(_HERE, 'best.pt')):
            if os.path.isfile(cand):
                args.model = cand
                break
    if not args.model or not os.path.isfile(args.model):
        raise SystemExit('找不到 best.pt。用 --model 指一下（见 实验3_真机线_队友清单 里的权重那节）')

    print('=== 实验3 取景 + 识别自检 ===')
    print('相机      : %s   请求 %dx%d' % (args.camera, size[0], size[1]))
    print('模型      : %s' % args.model)
    print('推理 conf : %.2f   task.conf_min = %.2f（低于它的会被判 unrecognized_object）'
          % (args.conf, conf_min))
    print('相机标定  : %s' % ('已填，能算格号' if usable else '未标定/占位 —— 格号一栏显示 "-"，不影响其余'))
    print()

    det = YoloDetector(args.model, conf_min=args.conf)
    print('模型类别表: %s' % det.names)
    names = set(det.names.values())
    classes = list(task.get('classes') or [])
    if classes:
        missing = [c for c in classes if c not in names]
        extra = [n for n in names if n not in classes]
        if missing:
            print('★ 模型缺这些类：%s —— 它们一个都认不出来。' % missing)
        if extra:
            print('! 模型多出这些类：%s —— 会走到 unrecognized_object（模型类别表与 task.classes 不一致）。'
                  % extra)
    print()

    cam = open_camera(args.camera, size)
    out_dir = args.out or 'vision_check_frames'
    n_frames = 1 if args.camera.startswith('image:') else max(1, int(args.frames))
    rows, per_frame = [], []
    size_warned = False
    try:
        for i in range(n_frames):
            frame = cam.grab()
            if frame is None:
                raise SystemExit('相机返回空帧（--camera none 不能用在这个工具上）')
            fh, fw = frame.shape[:2]
            scale = frame_scale((fw, fh))
            if (fw, fh) != size and not size_warned:
                size_warned = True
                print('⚠ 帧尺寸 %dx%d ≠ config 里的 %dx%d。' % (fw, fh, size[0], size[1]))
                print('  真机链路里 Cv2Camera **会直接报错**（标定是按后者做的，尺寸不符定位整体错位）——')
                print('  现在还能跑只因为 --camera image: 不做尺寸校验。下面已按网络实际看到的等效宽折算。')
                print()
            raw = det.detect(frame)
            frows = []
            for d in raw:
                x1, y1, x2, y2 = [float(v) for v in d['bbox']]
                frows.append({'cls': d['cls'], 'conf': float(d['conf']), 'bbox': [x1, y1, x2, y2],
                              'w': x2 - x1, 'h': y2 - y1, 'conf_min': conf_min,
                              'w_net': net_width(x2 - x1, (fw, fh)), 'scale': scale,
                              'cell': _cell_of([x1, y1, x2, y2], calib, cells, size)
                              if usable else '-'})
            rows.extend(frows)
            per_frame.append(frows)

            print('帧 %-3d 检出 %d 个%s' % (i + 1, len(frows),
                                          '' if not frows else ':'))
            for r in frows:
                flag = '' if r['conf'] >= conf_min else '  ← 低于 conf_min，会被判 unrecognized'
                print('    %-6s conf=%.2f  等效宽 %5.0fpx  原框 %4.0f×%-4.0f  中心(%4.0f,%4.0f)  '
                      '格=%-11s%s'
                      % (r['cls'], r['conf'], r['w_net'], r['w'], r['h'],
                         (r['bbox'][0] + r['bbox'][2]) / 2, (r['bbox'][1] + r['bbox'][3]) / 2,
                         r['cell'], flag))
            if not args.no_save:
                os.makedirs(out_dir, exist_ok=True)
                path = os.path.join(out_dir, 'frame_%02d.jpg' % (i + 1))
                cv2.imwrite(path, _annotate(cv2, frame, frows))
    finally:
        cam.close()

    per_class = summarize(rows, conf_min)
    print('\n' + '=' * 78)
    print('按类汇总（共 %d 帧）' % n_frames)
    print('%-8s %6s %8s %12s  %s' % ('类', '检出', '有效', '等效像素宽中位', '判词'))
    for cls in sorted(per_class):
        d = per_class[cls]
        med = '%.0f px' % d['median_px'] if d['median_px'] is not None else '-'
        print('%-8s %6d %8d %12s  %s %s' % (cls, d['n'], d['n_ok'], med, d['mark'], d['verdict']))

    mark, verdict = overall_verdict(per_class)
    print('\n结论 %s %s' % (mark, verdict))
    print('''
判据出处：exp3_domain_probe/SUMMARY_zoom.md（960 行合成帧，采自本机真机背景照）
  等效像素宽 <60px：mouse 召回 0–20%   |  80px：20–40%  |  100px：30–60%
  事后裁切/放大救不回来（probe_crop.py）⇒ 唯一手段是**采集时收窄取景**。''')
    if not args.no_save:
        print('标注图：%s（肉眼复核框准不准、有没有把整桌都拍进去）' % os.path.abspath(out_dir))
    print('复核完再决定：收窄取景 → 重跑本工具 → 通过了才谈标定和跑轮。')
    return 0 if mark != '✗' else 1


if __name__ == '__main__':
    sys.exit(main())
