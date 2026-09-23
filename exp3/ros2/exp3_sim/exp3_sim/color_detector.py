#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesis, ObjectHypothesisWithPose)

try:
    from vision_msgs.msg import ClassProbability
except ImportError:
    ClassProbability = None


class ColorDetector(Node):
    """Image-based detector for the color-coded Gazebo test objects."""

    def __init__(self):
        super().__init__('exp3_sim_color_detector')
        self.declare_parameter('image_topic', '/overhead/image_raw')
        self.declare_parameter('output_topic', '/detections_d2a')
        self.declare_parameter('min_area_px', 100.0)
        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_area = float(self.get_parameter('min_area_px').value)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, output_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info('HSV detector: %s -> %s' % (image_topic, output_topic))

    @staticmethod
    def _hypothesis(cls_name, score):
        hyp = ObjectHypothesis()
        cls_id = '0' if cls_name == 'cup' else '1'
        filled = False
        if ClassProbability is not None and hasattr(hyp, 'class_probabilities'):
            hyp.class_probabilities = [ClassProbability(class_name=cls_name,
                                                        probability=float(score))]
            filled = True
        if hasattr(hyp, 'class_id'):
            hyp.class_id = cls_id
            hyp.score = float(score)
            filled = True
        if not filled:
            raise RuntimeError('Unsupported vision_msgs ObjectHypothesis fields')
        wrapped = ObjectHypothesisWithPose()
        wrapped.hypothesis = hyp
        return wrapped

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning('cv_bridge: %s' % exc)
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {
            'cup': cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0, 105, 55]), np.array([14, 255, 255])),
                cv2.inRange(hsv, np.array([168, 105, 55]), np.array([180, 255, 255]))),
            'mouse': cv2.inRange(hsv, np.array([96, 100, 50]),
                                 np.array([138, 255, 255])),
        }
        out = Detection2DArray()
        out.header = msg.header
        height, width = frame.shape[:2]
        for cls_name, mask in masks.items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    np.ones((3, 3), dtype=np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                detection = Detection2D()
                detection.header = msg.header
                detection.id = '%s_%d_%d' % (cls_name, x + w // 2, y + h // 2)
                detection.bbox = BoundingBox2D()
                cx_norm = (x + w / 2.0) / float(width)
                cy_norm = (y + h / 2.0) / float(height)
                if hasattr(detection.bbox.center, 'position'):
                    detection.bbox.center.position.x = cx_norm
                    detection.bbox.center.position.y = cy_norm
                    detection.bbox.center.theta = 0.0
                else:
                    detection.bbox.center.x = cx_norm
                    detection.bbox.center.y = cy_norm
                if hasattr(detection.bbox, 'size_x'):
                    detection.bbox.size_x = w / float(width)
                    detection.bbox.size_y = h / float(height)
                else:
                    detection.bbox.size.width = w / float(width)
                    detection.bbox.size.height = h / float(height)
                detection.results = [self._hypothesis(cls_name, 0.99)]
                out.detections.append(detection)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
