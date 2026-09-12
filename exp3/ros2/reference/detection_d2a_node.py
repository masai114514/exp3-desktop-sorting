#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detection_d2a_node.py —— 乙的参考骨架（实验3 接口改造版检测节点）

旧节点 object_detection_lab/ros2_ws/src/detection_pkg/detection_pkg/detection_node.py
把结果发 std_msgs/String(JSON)。本骨架把这条链路改成实验3 口径：
    sensor_msgs/Image  ──cv_bridge──▶ cv::Mat ──YOLO(best.pt)──▶ vision_msgs/Detection2DArray

本文件是「怎么改」的模板，不是最终成品：字段命名/归一化口径等以组长为准（见
exp3/README.md 的接口契约）。仿真联调前先在真模型上跑通 Image→Detection2DArray。

构建运行（ROS2 Humble，Jetson / WSL2 均可）：
    sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge
    cd ~/ws_mecharm/src && cp detection_d2a_node.py ./ && cd ~/ws_mecharm && colcon build
    ros2 run detection_d2a_node detection_d2a_node --ros-args \
        -p model_path:=/path/to/best.pt -p image_topic:=/camera/image_raw
    # 另终端: ros2 topic echo /detections_d2a

依赖检测：vision_msgs 各版本字段不一致，先跑：
    ros2 interface show vision_msgs/msg/ObjectHypothesis
    ros2 interface show vision_msgs/msg/Detection2D
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# vision_msgs：老版(≤3.x) ObjectHypothesis{class_id, score}；
# 新版(≥4.x) 以 class_probabilities: ClassProbability[] 为主，class_id/score 标记废弃。
# 用 hasattr 探测，两头兼容；动手前务必 ros2 interface show 确认本机字段。
from vision_msgs.msg import (
    Detection2DArray, Detection2D, BoundingBox2D,
    ObjectHypothesisWithPose, ObjectHypothesis,
)
try:                                  # 新版才有的类型；老版没有就跳过
    from vision_msgs.msg import ClassProbability
    HAS_CLASS_PROB = True
except ImportError:
    HAS_CLASS_PROB = False

from ultralytics import YOLO

CLASS_NAMES = ['cup', 'mouse']        # 0=cup, 1=mouse —— 与 best.pt / config/bins.json 对齐


def make_hypothesis(cls_id, cls_name, score):
    """按本机 vision_msgs 版本字段填充 ObjectHypothesis。"""
    hyp = ObjectHypothesis()
    filled = []
    # 老版路径：class_id(数值) + score
    if hasattr(hyp, 'class_id'):
        hyp.class_id = cls_id
        hyp.score = float(score)
        filled.append('class_id')
    # 新版路径：class_probabilities(带类别名的概率数组)
    if HAS_CLASS_PROB and hasattr(hyp, 'class_probabilities'):
        cp = ClassProbability(class_name=cls_name, probability=float(score))
        hyp.class_probabilities = [cp]
        filled.append('class_probabilities')
    # 若两种都没填上：版本超预期，打印出来让队长定口径
    if not filled:
        raise RuntimeError('无法识别本机 ObjectHypothesis 字段，先 ros2 interface show vision_msgs/msg/ObjectHypothesis')
    return hyp


class DetectionD2aNode(Node):
    def __init__(self):
        super().__init__('detection_d2a_node')
        self.declare_parameter('model_path', '/path/to/best.pt')  # 改成你的权重实际路径
        self.declare_parameter('image_topic', '/camera/image_raw')  # 仿真相机/真相机话题
        self.declare_parameter('out_topic', 'detections_d2a')
        self.declare_parameter('conf', 0.25)     # 推理门限（任务验收线 conf_min=0.5 由下游判，见 config/task.json）
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', '0')    # 'cpu' / '0'
        self.declare_parameter('frame_id', 'camera')  # header.frame_id，跟图像话题对齐即可

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.get_logger().info(f'加载模型: {model_path}')
        self.model = YOLO(model_path)

        self.pub = self.create_publisher(Detection2DArray, out_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(f'订阅 {image_topic} → 发布 {out_topic}')

    def _on_image(self, img: Image):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        results = self.model(frame, imgsz=self.imgsz, conf=self.conf,
                             device=self.device, verbose=False)[0]
        H, W = frame.shape[:2]

        msg = Detection2DArray()
        msg.header = img.header
        msg.header.frame_id = self.frame_id
        if results.boxes is None:
            self.pub.publish(msg)
            return

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            score = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = results.names[cls_id] if results.names else CLASS_NAMES[cls_id]

            # —— bbox 口径：已定为「归一化 [0,1]、原点左上、中心 + 尺寸」（2026-09-12）——
            # 注意：vision_msgs 本身不强制归一化还是像素（.msg 表达不了这个约束），
            # 所以这是【队伍口径】不是规范要求；答辩若被问到，照这个说，别说成"spec 规定的"。
            # 消费侧 scan_from_detections.py 按 config/task.json 的图像尺寸(640x480)还原像素
            # xyxy，所以仿真相机分辨率必须与 config 一致，改分辨率要三人同步。
            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0
            w_px = x2 - x1
            h_px = y2 - y1

            d = Detection2D()
            d.header = msg.header
            d.id = f'{cls_name}_{int(cx_px)}_{int(cy_px)}'
            d.bbox = BoundingBox2D()
            d.bbox.center.x = cx_px / float(W)   # 归一化 [0,1]，原点左上
            d.bbox.center.y = cy_px / float(H)
            d.bbox.size.width = w_px / float(W)
            d.bbox.size.height = h_px / float(H)

            ohwp = ObjectHypothesisWithPose()
            ohwp.hypothesis = make_hypothesis(cls_id, cls_name, score)
            d.results = [ohwp]
            msg.detections.append(d)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionD2aNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
