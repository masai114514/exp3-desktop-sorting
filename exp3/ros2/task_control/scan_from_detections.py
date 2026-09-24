#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ScanFromDetections —— 把 Detection2DArray 转成 TaskController.scan() 缝隙的 det 列表。

TaskController 缝隙签名：scan() -> [ {cls, conf, bbox:[x1,y1,x2,y2]} ]（像素 xyxy）。
本节点持有识别话题的『最新一帧』并转成该格式；latest() 即 seam 的 scan 实现。

bbox 口径（与乙的 detection_d2a_node 骨架同源，动手前先 ros2 interface show 核对）：
    Detection2D.bbox.center(size) 归一化 [0,1]、原点左上 → 按 config 图像尺寸还原像素 xyxy。
    若队伍口径改成像素直出，把 _norm_to_px 关掉(或 param bbox_normalized=False)。
    class/conf 从 results[0].hypothesis 读，vision_msgs 新旧版本用 hasattr 兼容。
"""
from collections import namedtuple

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

Frame = namedtuple('Frame', 'dets stamp_ns')   # None stamp 表示还没收到首帧


def _norm_to_px(x, img_w):
    return x * float(img_w)


def dets_from_msg(msg, img_w, img_h):
    """Detection2DArray → [ {cls, conf, bbox:[x1,y1,x2,y2]} ]（像素）。"""
    out = []
    for d in msg.detections:
        center = d.bbox.center
        if hasattr(center, 'position'):
            cx_norm, cy_norm = center.position.x, center.position.y
        else:
            cx_norm, cy_norm = center.x, center.y
        if hasattr(d.bbox, 'size_x'):
            width_norm, height_norm = d.bbox.size_x, d.bbox.size_y
        else:
            width_norm = d.bbox.size.width
            height_norm = d.bbox.size.height
        cx = _norm_to_px(cx_norm, img_w)
        cy = _norm_to_px(cy_norm, img_h)
        w = _norm_to_px(width_norm, img_w)
        h = _norm_to_px(height_norm, img_h)
        if not d.results:
            continue
        hyp = d.results[0].hypothesis
        # 新旧版本兼容（同 detection_d2a_node）：优先 class_name/class_probabilities，
        # 退路 class_id/score 再按固定表换名。乙实现应已把 class_name 填好。
        cls, conf = _read_hypothesis(hyp)
        if cls is None:
            continue
        out.append({'cls': cls, 'conf': conf,
                    'bbox': [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]})
    return out


def _read_hypothesis(hyp):
    """→ (cls_name, conf)；读不到返回 (None, None)。class_id→名 的对照表按乙节点输出对齐。"""
    if hasattr(hyp, 'class_probabilities') and hyp.class_probabilities:
        cp = hyp.class_probabilities[0]
        return cp.class_name, float(cp.probability)
    if hasattr(hyp, 'class_id'):
        # ★ class_id 在 vision_msgs 里是 string。真值检测器直接填 'cup'/'mouse'，
        #   但旧链路（乙的节点）可能填的是数字 id —— 所以两种都要认。
        #   曾经只写 int(hyp.class_id) ⇒ truth_detector 一发帧就
        #   ValueError: invalid literal for int() with base 10: 'cup'，
        #   sim_task 因此"等首帧超时"而死（run_20260924_1115）。
        cid = hyp.class_id
        if isinstance(cid, str):
            return (cid or None), float(getattr(hyp, 'score', 0.0))
        try:
            return _id_to_name.get(int(cid)), float(getattr(hyp, 'score', 0.0))
        except (TypeError, ValueError):
            return None, None
    return None, None


# class_id → class_name：0=cup 1=mouse（与 best.pt 一致；若乙直接发 class_name 就只走上面分支）
_id_to_name = {0: 'cup', 1: 'mouse'}


class ScanFromDetections(Node):
    """订阅 Detection2DArray，保最新一帧；latest() 给 TaskController 当 scan。

    语义：还没收到首帧时 latest() 返回 None（控制器要等，别把『没数据』当『桌面已空』）；
    收到过帧后返回该帧像素 det 列表（可能是 []，即识别没看到目标 → 桌面已空）。
    """

    def __init__(self, image_topic='detections_d2a',
                 image_width_px=640, image_height_px=480):
        super().__init__('scan_from_detections')
        self._img = (int(image_width_px), int(image_height_px))
        self._frame = Frame([], None)          # 占位：stamp None = 未收到首帧
        self.create_subscription(Detection2DArray, image_topic, self._on_msg, 10)

    def _on_msg(self, msg):
        w, h = self._img
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._frame = Frame(dets_from_msg(msg, w, h), stamp)

    def frame(self):
        """拿到整帧（含 stamp_ns）。

        ★ 为什么需要 stamp：实测 run_20260924_005503 里，物块互穿引爆 ODE 之后
        **相机话题仍在以 ~48 fps 发帧，但画面内容冻结** —— 于是后续每轮扫描都读到
        同一份陈旧检测结果，控制器把它当成"桌面上只剩盒里的那个杯"，连续 3 轮无
        可执行目标 ⇒ `no_exec` 提前收尾，`placed_ok` 卡在 3。
        只靠 dets 内容分辨不出"真没看到"和"画面卡住了"，必须比对帧戳。
        """
        return self._frame

    def latest(self):
        """TaskController.scan：None=还没数据（等待）；list=该帧识别结果。"""
        return self._frame.dets if self._frame.stamp_ns is not None else None


def wait_first_frame(node, scan, timeout=20.0):
    """阻塞到 scan() 返回非 None（收到识别首帧）；超时抛 RuntimeError。"""
    import time
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if scan() is not None:
            return
        rclpy.spin_once(node, timeout_sec=0.5)
    raise RuntimeError('等待识别首帧超时 %ss' % timeout)
