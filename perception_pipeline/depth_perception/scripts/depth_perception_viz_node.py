#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from orca_interface.msg import PerceptionArray


class DepthPerceptionVizNode(Node):

    def __init__(self):
        super().__init__('depth_perception_viz')

        self.bridge = CvBridge()
        self.image = None
        self.objects = []

        # ── Declare parameters ─────────────────────────────────────
        self.declare_parameter('viz_image_topic',      '/yolov8_processed_image')
        self.declare_parameter('viz_perception_topic', '/orca/perception_array')
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
        # Coordinates in PerceptionObject are in YOLO input tensor space
        # Scale them to the visualization window size

        for obj in self.objects:

            x1 = int(obj.cx - obj.width / 2)
            y1 = int(obj.cy - obj.height / 2)
            x2 = int(obj.cx + obj.width / 2)
            y2 = int(obj.cy + obj.height / 2)

            # Green = stable, Yellow = valid (depth ok, not stable), Red = no depth
            if obj.is_stable:
                color = (0, 255, 0)
            elif obj.valid:
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)

            # cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            dist_str = f'{obj.distance:.2f}m' if obj.distance >= 0 else 'no depth'
            label = f'{obj.label} {dist_str}'

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
