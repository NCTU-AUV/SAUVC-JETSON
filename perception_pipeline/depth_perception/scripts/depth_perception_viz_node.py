#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from depth_perception.msg import PerceptionArray


class DepthPerceptionVizNode(Node):

    def __init__(self):
        super().__init__('depth_perception_viz')

        self.bridge = CvBridge()
        self.image = None
        self.objects = []

        # ── Declare parameters ─────────────────────────────────────
        self.declare_parameter('viz_image_topic',      '/yolov8_processed_image')
        self.declare_parameter('viz_perception_topic', '/perception_array')
        self.declare_parameter('viz_native_width',     1280)
        self.declare_parameter('viz_native_height',    720)

        viz_image_topic      = self.get_parameter('viz_image_topic').value
        viz_perception_topic = self.get_parameter('viz_perception_topic').value
        self.viz_native_width  = self.get_parameter('viz_native_width').value
        self.viz_native_height = self.get_parameter('viz_native_height').value

        self.create_subscription(
            Image,
            viz_image_topic,
            self.image_cb,
            10
        )

        self.create_subscription(
            PerceptionArray,
            viz_perception_topic,
            self.perception_cb,
            10
        )

    def image_cb(self, msg: Image):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.draw()

    def perception_cb(self, msg: PerceptionArray):
        self.objects = msg.objects

    def draw(self):
        if self.image is None:
            return

        img = self.image.copy()

        h, w = img.shape[:2]
        # Coordinates in perception_array are scaled to the native RealSense image.
        # Scale them back to the visualization window size (usually the 640x640 YOLO overlay).
        scale_x = w / float(self.viz_native_width)
        scale_y = h / float(self.viz_native_height)

        for obj in self.objects:
            cx = obj.cx * scale_x
            cy = obj.cy * scale_y
            bw = obj.width * scale_x
            bh = obj.height * scale_y

            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)

            color = (0, 255, 0) if obj.valid else (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            label = f"{obj.class_name} {obj.distance:.2f}m"
            
            cv2.putText(
                 img,
                 label,
                 (x1, y2+15),
                 cv2.FONT_HERSHEY_SIMPLEX,
                 0.5,
                 color,
                 2
            )

        cv2.imshow('Depth Perception Viz', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = DepthPerceptionVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
