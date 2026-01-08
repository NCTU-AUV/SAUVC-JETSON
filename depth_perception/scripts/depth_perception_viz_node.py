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

        self.create_subscription(
            Image,
            '/yolov8_processed_image',
            self.image_cb,
            10
        )

        self.create_subscription(
            PerceptionArray,
            '/perception_array',
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

        for obj in self.objects:
            x1 = int(obj.cx - obj.width / 2)
            y1 = int(obj.cy - obj.height / 2)
            x2 = int(obj.cx + obj.width / 2)
            y2 = int(obj.cy + obj.height / 2)

            color = (0, 255, 0) if obj.valid else (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            label = f"{obj.class_name} {obj.distance:.2f}m"
            cv2.putText(
                img,
                label,
                (x1, y2 + 15),
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
