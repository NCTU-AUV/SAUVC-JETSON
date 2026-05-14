#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo


class BottomCamPublisher(Node):
    def __init__(self):
        super().__init__('bottom_cam_publisher')

        # Parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'bottom_cam')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)

        camera_index = self.get_parameter('camera_index').value
        self.frame_id = self.get_parameter('frame_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # OpenCV camera
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            raise RuntimeError('Failed to open USB camera')

        # Publishers
        self.image_pub = self.create_publisher(
            Image, '/orca/usb_cam/image_raw', 10
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/orca/usb_cam/camera_info', 10
        )

        self.bridge = CvBridge()

        # CameraInfo (simple placeholder – no calibration)
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = width
        self.camera_info_msg.height = height
        self.camera_info_msg.distortion_model = 'plumb_bob'
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [
            1.0, 0.0, width / 2.0,
            0.0, 1.0, height / 2.0,
            0.0, 0.0, 1.0
        ]
        self.camera_info_msg.p = [
            1.0, 0.0, width / 2.0, 0.0,
            0.0, 1.0, height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Timer
        period = 1.0 / fps
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('Bottom camera publisher started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        now = self.get_clock().now().to_msg()

        # Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = now
        image_msg.header.frame_id = self.frame_id

        # CameraInfo message
        self.camera_info_msg.header.stamp = now
        self.camera_info_msg.header.frame_id = self.frame_id

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(self.camera_info_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BottomCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
