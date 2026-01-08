#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header

from depth_perception.msg import PerceptionArray, PerceptionObject


class DepthPerceptionNode(Node):

    def __init__(self):
        super().__init__('depth_perception')

        self.bridge = CvBridge()
        self.depth_img = None

        self.create_subscription(
            Image,
            '/realsense/depth_image',
            self.depth_cb,
            10
        )

        self.create_subscription(
            Detection2DArray,
            '/detections_output',
            self.detection_cb,
            10
        )

        self.pub = self.create_publisher(
            PerceptionArray,
            '/perception_array',
            10
        )

    def depth_cb(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detection_cb(self, msg: Detection2DArray):
        if self.depth_img is None:
            return

        h, w = self.depth_img.shape
        out = PerceptionArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'realsense'

        for det in msg.detections:
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)
            bw = int(det.bbox.size_x)
            bh = int(det.bbox.size_y)

            x1 = max(0, cx - bw // 2)
            x2 = min(w - 1, cx + bw // 2)
            y1 = max(0, cy - bh // 2)
            y2 = min(h - 1, cy + bh // 2)

            cls = det.results[0].hypothesis.class_id

            obj = PerceptionObject()
            obj.class_name = cls
            obj.cx = float(cx)
            obj.cy = float(cy)
            obj.width = float(bw)
            obj.height = float(bh)

            # ===== Gate depth estimation =====
            valid = False
            z = float('nan')

            if cls == '2':
                column_depths = []

                for u in range(x1, x2, 3):
                    col = self.depth_img[y1:y2, u]
                    col = col[np.isfinite(col)]
                    col = col[col > 0.3]  # remove noise

                    if len(col) > 30:
                        column_depths.append((u, np.percentile(col, 30)))

                if len(column_depths) >= 2:
                    left = min(column_depths, key=lambda x: x[0])
                    right = max(column_depths, key=lambda x: x[0])

                    width_pix = right[0] - left[0]
                    depth_sym = abs(left[1] - right[1])

                    if width_pix > bw * 0.4 and depth_sym < 0.4:
                        z = (left[1] + right[1]) / 2.0
                        valid = True

            # ===== Non-gate objects =====
            else:
                roi = self.depth_img[y1:y2, x1:x2]
                roi = roi[np.isfinite(roi)]
                roi = roi[roi > 0.3]
                if len(roi) > 50:
                    z = float(np.percentile(roi, 30))
                    valid = True

            obj.distance = float(z) if valid else -1.0
            obj.valid = valid

            out.objects.append(obj)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
