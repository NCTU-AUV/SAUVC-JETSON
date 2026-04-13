#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection2DArray

from depth_perception.msg import PerceptionArray, PerceptionObject


class DepthPerceptionNode(Node):

    def __init__(self):
        super().__init__('depth_perception')

        self.bridge = CvBridge()
        self.depth_img = None

        # ── Declare parameters ─────────────────────────────────────
        self.declare_parameter('depth_image_topic',       '/orca/aligned_depth_to_color/image_raw')
        self.declare_parameter('detections_topic',        '/detections_output')
        self.declare_parameter('mode_topic',              '/orca/camera_mode')
        self.declare_parameter('perception_output_topic', '/perception_array')
        self.declare_parameter('yolo_input_width',        640)
        self.declare_parameter('yolo_input_height',       640)

        depth_image_topic       = self.get_parameter('depth_image_topic').value
        detections_topic        = self.get_parameter('detections_topic').value
        mode_topic              = self.get_parameter('mode_topic').value
        perception_output_topic = self.get_parameter('perception_output_topic').value
        self.yolo_input_width   = self.get_parameter('yolo_input_width').value
        self.yolo_input_height  = self.get_parameter('yolo_input_height').value

        self.current_mode = 'realsense'

        # ── QoS profiles ──────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            Image,
            depth_image_topic,
            self.depth_cb,
            sensor_qos
        )

        self.create_subscription(
            Detection2DArray,
            detections_topic,
            self.detection_cb,
            reliable_qos
        )

        self.create_subscription(
            String,
            mode_topic,
            self.mode_cb,
            mode_qos
        )

        # ── Publisher ──────────────────────────────────────────────
        self.pub = self.create_publisher(
            PerceptionArray,
            perception_output_topic,
            reliable_qos
        )

    def mode_cb(self, msg: String):
        if msg.data in ('realsense', 'usb'):
            if self.current_mode != msg.data:
                self.get_logger().info(
                    f'Depth perception mode: {self.current_mode} -> {msg.data}')
                self.current_mode = msg.data
        else:
            self.get_logger().warn(
                f'Unknown camera mode: {msg.data}')

    def depth_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        
        if msg.encoding in ['16UC1', 'mono16'] or cv_img.dtype == np.uint16:
            # RealSense 16UC1 depth goes in millimeters, convert to meters
            self.depth_img = cv_img.astype(np.float32) / 1000.0
        else:
            self.depth_img = cv_img.astype(np.float32)

    def detection_cb(self, msg: Detection2DArray):
        # ── Gate: only process when RealSense is active ───────
        if self.current_mode != 'realsense':
            out = PerceptionArray()
            out.header = Header()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'realsense'
            self.pub.publish(out)
            return

        if self.depth_img is None:
            return

        h, w = self.depth_img.shape
        out = PerceptionArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'realsense'

        for det in msg.detections:
            # Detections are relative to the YOLO input tensor (configurable size)
            # Need to scale them up to the native depth image dimensions (w, h)
            scale_x = w / float(self.yolo_input_width)
            scale_y = h / float(self.yolo_input_height)

            cx = int(det.bbox.center.position.x * scale_x)
            cy = int(det.bbox.center.position.y * scale_y)
            bw = int(det.bbox.size_x * scale_x)
            bh = int(det.bbox.size_y * scale_y)

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
                        column_depths.append(
                            (u, np.percentile(col, 30)))

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
                # Sample the center 50% region to avoid large background depths at the edges
                cw, ch = bw * 0.5, bh * 0.5
                c_x1 = max(0, int(cx - cw // 2))
                c_x2 = min(w - 1, int(cx + cw // 2))
                c_y1 = max(0, int(cy - ch // 2))
                c_y2 = min(h - 1, int(cy + ch // 2))

                roi = self.depth_img[c_y1:c_y2, c_x1:c_x2]
                roi = roi[np.isfinite(roi)]
                roi = roi[roi > 0.3]

                # Fallback to full bbox if center crop has no valid depth
                if len(roi) < 5:
                    self.get_logger().info(f"Fallback to bbox: length of center crop roi was {len(roi)}")
                    roi = self.depth_img[y1:y2, x1:x2]
                    roi = roi[np.isfinite(roi)]
                    roi = roi[roi > 0.3]
                
                if len(roi) > 0:
                    # 10th percentile gives robust front edge of the object
                    z = float(np.percentile(roi, 10))
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
