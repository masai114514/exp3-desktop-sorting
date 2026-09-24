#!/usr/bin/env python3
"""真值投影检测器（仿真专用）：/gazebo/model_states → Detection2DArray。

★ 为什么需要它，以及它替换了哪一步
------------------------------------------------------------------
仿真里原先用 HSV 检测器吃 Gazebo 相机画面（color_detector.py）。但实测
run_20260924_110405 暴露了一条与算法无关的故障：**Gazebo 的相机渲染管线会卡死**
—— 帧戳照常推进（173.9s→181.4s→189.1s），画面内容却一字不变：

    轮次    dets  内容
    round1   6    c1..c6 全部正确映射（此时臂静止在停靠位，渲染健康）
    round2   6    c1 已被取走却仍在原像素被检出 ⇒ 内容滞后
    round3   6    c4 漂到 (461,340)，开始出现 OUT
    round4   1    只剩料盒里那个 mouse，且随后三轮完全相同 ⇒ 渲染已死

物理侧完全正常（trace_blocks 实测物块坐标精确、臂位姿实时），所以卡的是
**渲染**这一段。根因是这台容器没有 GPU，Mesa 走 llvmpipe 软渲染
（nvidia-smi 显存 0 MiB，460 个 llvmpipe 线程），越跑越跟不上。

于是把"像素→检测框"这一步换成真值投影：
    model_states 世界坐标 → table_to_px（**同一份已标定的投影**）→ 像素框
**相机标定、投影、像素→桌面→格子的映射链路一点都没动**，动的只是"框从哪来"。
这样既能继续验证「规划-取放-落料」这条主线，也把故障隔离在渲染这一段。

★ 如实披露
------------------------------------------------------------------
这是**理想感知**（无漏检、无噪声、无遮挡），比真实相机强。所以它只能用来
验证规划与执行，不能用来证明"检测算法有效"——那份证据得靠 HSV 检测器在
渲染健康时（round 1）的 6/6 正确映射，两者在报告里必须分开陈述。

可用 `--noise-px` 注入像素噪声，让定位带上检测误差（默认 0 = 理想）。
"""
import math
import random

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

# 各类别的**水平**外形尺寸（米），用于生成检测框；z 不参与（俯视只看 xy 投影）
# 与 pick_place_server._object_sdf 的几何保持一致。
CLS_FOOTPRINT_M = {
    'cup': (0.040, 0.040),      # 圆柱 r=0.020 ⇒ 直径 0.040
    'mouse': (0.040, 0.025),    # 盒 0.040 x 0.025 x 0.020
}


def _load_cls_table():
    """block_c1..block_c6 → 类别。单一真源是 exp3_sim/scene.py。"""
    try:
        from exp3_sim.scene import NOMINAL_CLS
        return dict(NOMINAL_CLS)
    except Exception:                                  # pragma: no cover
        return {'c1': 'cup', 'c2': 'mouse', 'c3': 'cup',
                'c4': 'cup', 'c5': 'mouse', 'c6': 'cup'}


class TruthDetector(Node):
    def __init__(self, output_topic='/detections_d2a', image_width_px=640,
                 image_height_px=480, noise_px=0.0, rate_hz=10.0):
        super().__init__('truth_detector')
        self._img = (float(image_width_px), float(image_height_px))
        self._noise = float(noise_px)
        self._cls = _load_cls_table()
        self._models = None
        self.pub = self.create_publisher(Detection2DArray, output_topic, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self._on_models, 10)
        period = 1.0 / float(rate_hz) if rate_hz > 0 else 0.1
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'truth detector ready: %dx%d noise=%.1fpx -> %s'
            % (image_width_px, image_height_px, self._noise, output_topic))

    def _on_models(self, msg):
        self._models = msg

    def _tick(self):
        msg = self._models
        if msg is None:
            return
        w, h = self._img
        try:
            from sort_core.geometry import table_to_px
        except Exception as exc:                       # pragma: no cover
            self.get_logger().error('cannot import table_to_px: %s' % exc)
            return
        try:
            calib = self._calib
        except AttributeError:
            self.get_logger().warn('calib not set; skipping frame')
            return

        out = Detection2DArray()
        now = self.get_clock().now().to_msg()
        out.header.stamp = now
        out.header.frame_id = 'overhead'
        for name, pose in zip(msg.name, msg.pose):
            if not name.startswith('block_'):
                continue
            cid = name[len('block_'):]
            cls = self._cls.get(cid)
            if cls is None or cls not in CLS_FOOTPRINT_M:
                continue
            fx, fy = CLS_FOOTPRINT_M[cls]
            try:
                cx, cy = table_to_px(pose.position.x, pose.position.y, calib)
            except Exception:
                continue
            if self._noise > 0.0:
                cx += random.uniform(-self._noise, self._noise)
                cy += random.uniform(-self._noise, self._noise)
            sx, sy = calib['scale_px_per_m']
            if calib.get('mode', 'rectilinear') != 'rectilinear':
                continue                               # 只支持线性标定
            half_w = fx * abs(sx) / 2.0
            half_h = fy * abs(sy) / 2.0
            # ★ 必须是**归一化**坐标：下游 scan_from_detections.dets_from_msg 会做
            #   `cx = center.x * image_width` 还原成像素（color_detector 也是这么填的）。
            #   曾直接填像素 ⇒ 被再乘一次 640/480，检测点变成 (174720, 70560) 这种
            #   天文数字，locate() 判"出画面" ⇒ 6 个全 OFF。
            det = Detection2D()
            det.header = out.header
            det.bbox.center.position.x = float(cx / w)
            det.bbox.center.position.y = float(cy / h)
            det.bbox.center.theta = 0.0
            det.bbox.size_x = float(2 * half_w / w)
            det.bbox.size_y = float(2 * half_h / h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls
            hyp.hypothesis.score = 0.99
            det.results.append(hyp)
            out.detections.append(det)
        self.pub.publish(out)


def main(args=None):
    import argparse
    import json
    import os
    import sys

    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.environ.get('EXP3_CAMERA_JSON', ''))
    ap.add_argument('--output-topic', default='/detections_d2a')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--noise-px', type=float, default=0.0)
    ap.add_argument('--rate', type=float, default=10.0)
    ns, _unknown = ap.parse_known_args(args)

    rclpy.init(args=args)
    node = TruthDetector(ns.output_topic, ns.width, ns.height, ns.noise_px, ns.rate)

    # 标定：优先用 task.yaml 里的 camera.calib（与 HSV 链路**同一份**标定）
    calib = None
    if ns.config and os.path.isfile(ns.config):
        data = json.load(open(ns.config, encoding='utf-8'))
        calib = ((data.get('camera') or {}).get('calib')
                 or (data.get('task') or {}).get('camera', {}).get('calib'))
    if calib is None:
        root = os.environ.get('EXP3_ROOT')
        if root:
            sys.path.insert(0, root)
            from sort_core.config import load_all
            calib = load_all()['task']['camera']['calib']
    if calib is None:
        node.get_logger().error('no camera calib found')
        return 1
    node._calib = calib
    node.get_logger().info('calib mode=%s offset=%s scale=%s'
                           % (calib.get('mode'), calib.get('offset_px'),
                              calib.get('scale_px_per_m')))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
