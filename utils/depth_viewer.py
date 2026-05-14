#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from threading import Lock


class DepthInspector(Node):
    def __init__(self):
        super().__init__('depth_inspector')

        self.bridge = CvBridge()
        self.lock = Lock()

        self.color_img = None
        self.depth_img = None

        self.cursor_x = 0
        self.cursor_y = 0

        # Subscribers
        self.create_subscription(
            Image,
            '/orca_auv/color/image_raw',
            self.color_callback,
            10
        )

        self.create_subscription(
            Image,
            '/orca_auv/depth/image_raw',
            self.depth_callback,
            10
        )

        # OpenCV window
        self.window_name = "Color Image (Depth Inspector)"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        # Timer for display
        self.create_timer(0.03, self.update_display)  # ~30 FPS

    def color_callback(self, msg):
        with self.lock:
            self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        with self.lock:
            # Keep raw depth (likely uint16 in mm)
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def mouse_callback(self, event, x, y, flags, param):
        self.cursor_x = x
        self.cursor_y = y

    def update_display(self):
        with self.lock:
            if self.color_img is None or self.depth_img is None:
                return

            display = self.color_img.copy()

            h, w = display.shape[:2]

            if 0 <= self.cursor_x < w and 0 <= self.cursor_y < h:
                depth_raw = self.depth_img[self.cursor_y, self.cursor_x]

                # Convert to meters (assuming mm input)
                if depth_raw == 0:
                    depth_m = 0.0
                else:
                    depth_m = float(depth_raw) / 1000.0

                text = f"Depth: {depth_m:.3f} m | {self.cursor_x}, {self.cursor_y}"

                # Draw marker
                cv2.circle(display, (self.cursor_x, self.cursor_y), 5, (0, 0, 255), -1)

                # Draw text
                cv2.putText(
                    display,
                    text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            cv2.imshow(self.window_name, display)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthInspector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()