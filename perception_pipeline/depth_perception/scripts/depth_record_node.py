#!/usr/bin/env python3
"""Depth → 8-bit grayscale JPEG, for the bag recorder.

The bag used to carry the depth stream raw (32FC1 metres, 640x480 x 30 Hz
≈ 37 MB/s — on its own about 95% of an image-enabled bag). That bought
metric depth you could re-run `_estimate_gate` against, and cost roughly
2.3 GB/min. This node trades that away deliberately: it renders depth the
same way the Web GUI's DEPTH panel does and records the *picture*, which
is what post-run review actually looks at.

The rendering matches what `web_video_server` serves the GUI, so a bag
frame and the live panel show the same thing:

    mono8 = clip((depth_m - min) / (max - min), 0, 1) * 255

Near is dark, far is bright, out-of-range clips at the ends. `+inf` — what
a depth camera returns for a ray that hit nothing — clips to the far end
(white), matching `web_video_server`; genuine no-data (NaN, or the 0 a
16UC1 sensor writes) reads black. Default window is 0-10 m, the same one
`gui_node.py` pins on the live stream.

One deliberate difference from `web_video_server`: the window here is
always in **metres**. 16UC1 sources (the RealSense publishes aligned
depth in millimetres) are converted before scaling, so the same
min/max_depth_m means the same distances on the real robot and in sim.

What this is NOT good for: anything metric. 8 bits over 10 m is ~4 cm
per code before JPEG touches it, and JPEG is lossy on top of that. Once
a run is recorded this way the depth is for looking at, not for
reprocessing.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class DepthRecordNode(Node):

    def __init__(self):
        super().__init__('depth_record')

        self.bridge = CvBridge()

        # ── Declare parameters ─────────────────────────────────────
        self.declare_parameter('depth_image_topic',
                               '/orca/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_topic', '/orca/record/depth/compressed')
        # 0-10 m mirrors the GUI. Note the pool runs out to ~16 m, so the far
        # wall saturates white at this setting — raise max_depth_m if a run
        # needs the background readable, at the cost of near-field contrast.
        self.declare_parameter('min_depth_m', 0.0)
        self.declare_parameter('max_depth_m', 10.0)
        # 80 rather than the OpenCV default 95: depth renders are smooth
        # gradients, so the extra bytes buy almost nothing visible.
        self.declare_parameter('jpeg_quality', 80)

        depth_image_topic = self.get_parameter('depth_image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        if self.max_depth <= self.min_depth:
            raise ValueError(
                f'max_depth_m ({self.max_depth}) must exceed '
                f'min_depth_m ({self.min_depth})')

        # BEST_EFFORT to match the depth publishers (see depth_perception_node);
        # a RELIABLE subscription would simply never connect to them.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # RELIABLE on the way out, matching the `image_transport republish`
        # nodes that feed the other three /orca/record/* topics — the recorder
        # inherits the offered QoS, and dropping frames there is pointless
        # when the whole point of the topic is the recording.
        record_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(CompressedImage, output_topic, record_qos)
        self.create_subscription(Image, depth_image_topic, self._depth_cb, sensor_qos)

        self.get_logger().info(
            f'Recording depth as mono8 JPEG: {depth_image_topic} → {output_topic} '
            f'[{self.min_depth:.1f}-{self.max_depth:.1f} m, q{self.jpeg_quality}]')

    def _depth_cb(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Same normalisation as depth_perception_node._depth_cb — 16-bit
        # sources carry millimetres, float sources carry metres.
        if msg.encoding in ('16UC1', 'mono16') or depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        # np.nan_to_num before the scaling, not after: NaN survives clip()
        # untouched, and casting NaN to uint8 is undefined in numpy (it warns
        # and yields whatever the platform's C cast does, which is not always
        # 0).
        #
        # +inf maps to the FAR end, not to 0. Gazebo's depth camera returns
        # +inf for every ray that hits nothing — 5% of a typical pool frame,
        # looking up through open water — and `web_video_server` renders those
        # white, because cv::min(inf, max) is max. Folding them to black
        # instead would draw the emptiest part of the scene as if it were
        # pressed against the lens, which is both wrong and the single most
        # misleading thing this image could say. NaN is different: that is
        # "no data", not "far", and stays black, as does the 0 a 16UC1 sensor
        # writes for no-return.
        depth_m = np.nan_to_num(depth_m, nan=0.0,
                                posinf=self.max_depth, neginf=0.0)

        scaled = (depth_m - self.min_depth) * (255.0 / (self.max_depth - self.min_depth))
        mono8 = np.clip(scaled, 0.0, 255.0).astype(np.uint8)

        ok, buf = cv2.imencode(
            '.jpg', mono8, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn('JPEG encode failed, dropping frame',
                                   throttle_duration_sec=5.0)
            return

        out = CompressedImage()
        out.header = msg.header
        # image_transport's format convention, same as the other three
        # /orca/record/* feeds, so `republish compressed raw` can decode it.
        out.format = 'mono8; jpeg compressed mono8'
        out.data = buf.tobytes()
        self.pub.publish(out)


def main():
    # Both halves of this matter on shutdown, and neither is optional.
    #
    # SIGINT from launch does NOT surface as KeyboardInterrupt here: rclpy's
    # signal handler tears the context down underneath the executor, so spin()
    # raises ExternalShutdownException. Catching only KeyboardInterrupt lets it
    # escape, and then the finally clause calls rclpy.shutdown() on a context
    # that is already down, which raises RCLError on top of it. The result is
    # a double traceback on every clean `make stop` — noise that would later
    # hide a real failure in the launch log.
    rclpy.init()
    node = DepthRecordNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
