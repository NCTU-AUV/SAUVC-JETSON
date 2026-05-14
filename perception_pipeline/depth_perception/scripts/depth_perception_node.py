#!/usr/bin/env python3
"""
depth_perception_node.py
────────────────────────
Unified perception funnel for the SAUVC AUV.

Both camera modes (realsense / usb) flow through this single node and are
published on /orca/perception_array for the decision node to consume.

  realsense mode
    ─ subscribes depth image + detections + camera_info
    ─ solves 3-D pose (X, Y, Z) and distance for every detection
    ─ runs per-class depth strategies (gate vs general objects)
    ─ populates pose, distance, valid

  usb mode
    ─ subscribes detections only
    ─ emits PerceptionObjects with distance = -1.0, empty pose, valid = False
    ─ is_stable can still become True once the tracker accumulates enough frames

In both modes the node maintains a per-label stability tracker:
  is_stable = True when
      detected_in_last_N_frames >= stability_min_hits
      AND std_dev(position) over window < stability_pos_std_thresh
      AND confidence >= stability_conf_thresh

All parameters are declared as ROS parameters and should be set via
  orca_perception/config/perception_params.yaml
"""

from multiprocessing import get_logger
import collections
import math

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
)

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header, String
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Pose, Quaternion

from orca_interface.msg import PerceptionArray, PerceptionObject


# ---------------------------------------------------------------------------
# Stability tracker
# ---------------------------------------------------------------------------
class _ObjectTracker:
    """Rolling-window stability tracker for a single object label."""

    def __init__(self, window: int, min_hits: int,
                 pos_std_thresh: float, conf_thresh: float):
        self._window = window
        self._min_hits = min_hits
        self._pos_std_thresh = pos_std_thresh
        self._conf_thresh = conf_thresh

        # deques length-limited to window
        self._hits: collections.deque[bool] = collections.deque(maxlen=window)
        self._positions: collections.deque[tuple] = collections.deque(maxlen=window)
        self._confidences: collections.deque[float] = collections.deque(maxlen=window)

    def update(self, detected: bool,
               pos: tuple = (float('nan'), float('nan'), float('nan')),
               conf: float = 0.0) -> bool:
        """
        Push a new observation and return the current is_stable value.
        Call with detected=False when the label is absent in this frame.
        """
        self._hits.append(detected)
        if detected:
            self._positions.append(pos)
            self._confidences.append(conf)

        if sum(self._hits) < self._min_hits:
            return False

        if not self._confidences or max(self._confidences) < self._conf_thresh:
            return False

        # Position std-dev check (only if we have 3-D positions)
        valid_pos = [p for p in self._positions
                     if all(math.isfinite(v) for v in p)]
        if len(valid_pos) >= 2:
            arr = np.array(valid_pos)
            std = float(np.mean(np.std(arr, axis=0)))
            if std >= self._pos_std_thresh:
                return False

        return True


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------
class DepthPerceptionNode(Node):

    def __init__(self):
        super().__init__('depth_perception')

        self.bridge = CvBridge()
        self.depth_img = None
        self._cx = None   # optical centre x (from camera_info)
        self._cy_p = None  # optical centre y
        self._fx = None
        self._fy = None

        # ── Declare parameters ─────────────────────────────────────
        self.declare_parameter('depth_image_topic',
                               '/orca/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/orca/color/camera_info')
        self.declare_parameter('detections_topic',        '/detections_output')
        self.declare_parameter('mode_topic',              '/orca/camera_mode')
        self.declare_parameter('perception_output_topic', '/orca/perception_array')
        self.declare_parameter('yolo_input_width',        640)
        self.declare_parameter('yolo_input_height',       640)

        # class-id → label mapping (index = class id)
        default_names = [
            'blue_drum', 'blue_flare', 'gate', 'orange_flare',
            'red_drum', 'red_flare', 'yellow_flare',
        ]
        self.declare_parameter('class_names', default_names)

        # depth estimation thresholds
        self.declare_parameter('depth_min_range',       0.3)
        self.declare_parameter('gate_symmetry_thresh',  0.4)
        self.declare_parameter('gate_width_min_ratio',  0.4)
        self.declare_parameter('gate_col_step',         3)
        self.declare_parameter('gate_col_min_samples',  2)
        self.declare_parameter('gate_col_min_points',   30)
        self.declare_parameter('obj_center_crop_ratio', 0.5)
        self.declare_parameter('obj_percentile',        10)
        self.declare_parameter('obj_min_roi_points',    5)

        # stability tracker parameters
        self.declare_parameter('stability_window',          10)
        self.declare_parameter('stability_min_hits',         7)
        self.declare_parameter('stability_pos_std_thresh',  0.15)
        self.declare_parameter('stability_conf_thresh',     0.35)

        # ── Read parameters ────────────────────────────────────────
        depth_image_topic       = self.get_parameter('depth_image_topic').value
        camera_info_topic       = self.get_parameter('camera_info_topic').value
        detections_topic        = self.get_parameter('detections_topic').value
        mode_topic              = self.get_parameter('mode_topic').value
        perception_output_topic = self.get_parameter('perception_output_topic').value
        self.yolo_input_width   = self.get_parameter('yolo_input_width').value
        self.yolo_input_height  = self.get_parameter('yolo_input_height').value

        names_list = self.get_parameter('class_names').value
        self._names: dict[int, str] = {i: n for i, n in enumerate(names_list)}
        self.get_logger().info(
            f'Loaded {len(self._names)} class labels: '
            + ', '.join(f'{i}:{v}' for i, v in self._names.items()))

        self._depth_min_range      = self.get_parameter('depth_min_range').value
        self._gate_sym_thresh      = self.get_parameter('gate_symmetry_thresh').value
        self._gate_w_ratio         = self.get_parameter('gate_width_min_ratio').value
        self._gate_col_step        = self.get_parameter('gate_col_step').value
        self._gate_col_min_samples = self.get_parameter('gate_col_min_samples').value
        self._gate_col_min_pts     = self.get_parameter('gate_col_min_points').value
        self._crop_ratio           = self.get_parameter('obj_center_crop_ratio').value
        self._obj_pct              = self.get_parameter('obj_percentile').value
        self._obj_min_pts          = self.get_parameter('obj_min_roi_points').value

        stab_window     = self.get_parameter('stability_window').value
        stab_min_hits   = self.get_parameter('stability_min_hits').value
        stab_pos_std    = self.get_parameter('stability_pos_std_thresh').value
        stab_conf_thresh = self.get_parameter('stability_conf_thresh').value

        self._stab_kwargs = dict(
            window=stab_window,
            min_hits=stab_min_hits,
            pos_std_thresh=stab_pos_std,
            conf_thresh=stab_conf_thresh,
        )
        # label → _ObjectTracker  (created on first sight)
        self._trackers: dict[str, _ObjectTracker] = {}

        self.current_mode = 'realsense'

        # ── QoS profiles ──────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriptions ─────────────────────────────────────────
        self.create_subscription(
            Image, depth_image_topic, self._depth_cb, sensor_qos)

        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_cb, sensor_qos)

        self.create_subscription(
            Detection2DArray, detections_topic, self._detection_cb, reliable_qos)

        self.create_subscription(
            String, mode_topic, self._mode_cb, mode_qos)

        # ── Publisher ──────────────────────────────────────────────
        self.pub = self.create_publisher(
            PerceptionArray, perception_output_topic, reliable_qos)

        self.get_logger().info(
            f'DepthPerceptionNode ready — output: {perception_output_topic}')

    # ── Callbacks ──────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        if msg.data in ('realsense', 'usb'):
            if self.current_mode != msg.data:
                self.get_logger().info(
                    f'Camera mode: {self.current_mode} → {msg.data}')
                self.current_mode = msg.data
        else:
            self.get_logger().warn(f'Unknown camera mode: {msg.data!r}')

    def _camera_info_cb(self, msg: CameraInfo):
        """Extract intrinsics from the first valid camera_info message."""
        if self._fx is not None:
            return  # already have them
        K = msg.k  # row-major 3×3
        self._fx   = K[0]
        self._fy   = K[4]
        self._cx   = K[2]
        self._cy_p = K[5]
        self.get_logger().info(
            f'Camera intrinsics received: '
            f'fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy_p:.1f}')

    def _depth_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding in ('16UC1', 'mono16') or cv_img.dtype == np.uint16:
            self.depth_img = cv_img.astype(np.float32) / 1000.0
        else:
            self.depth_img = cv_img.astype(np.float32)

    # ── Main detection callback ─────────────────────────────────────

    def _detection_cb(self, msg: Detection2DArray):
        out = PerceptionArray()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.current_mode

        detected_labels: set[str] = set()

        for det in msg.detections:
            # ── Resolve class id → label ──────────────────────────
            raw_id = det.results[0].hypothesis.class_id
            try:
                class_idx = int(raw_id)
            except (ValueError, TypeError):
                class_idx = -1
            label = self._names.get(class_idx, str(raw_id))
            confidence = float(det.results[0].hypothesis.score)

            # ── 2-D bbox (in YOLO tensor space) ──────────────────
            cx_yolo = float(det.bbox.center.position.x)
            cy_yolo = float(det.bbox.center.position.y)
            bw_yolo = float(det.bbox.size_x)
            bh_yolo = float(det.bbox.size_y)

            obj = PerceptionObject()
            obj.label      = label
            obj.confidence = confidence
            obj.cx         = cx_yolo
            obj.cy         = cy_yolo
            obj.width      = bw_yolo
            obj.height     = bh_yolo
            obj.pose       = Pose()
            obj.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            obj.distance   = -1.0
            obj.valid      = False

            # ── Depth estimation (realsense mode only) ────────────
            if self.current_mode == 'realsense' and self.depth_img is not None:
                h, w = self.depth_img.shape

                # Scale YOLO-space coords → depth image pixel coords
                scale_x = w / float(self.yolo_input_width)
                scale_y = h / float(self.yolo_input_height)

                cx_d = int(cx_yolo * scale_x)
                cy_d = int(cy_yolo * scale_y)
                bw_d = int(bw_yolo * scale_x)
                bh_d = int(bh_yolo * scale_y)

                x1 = max(0, cx_d - bw_d // 2)
                x2 = min(w - 1, cx_d + bw_d // 2)
                y1 = max(0, cy_d - bh_d // 2)
                y2 = min(h - 1, cy_d + bh_d // 2)

                # Gate labels use gate-specific column sampling regardless of class index.
                if label == 'gate':
                    x1_ = max(x1-15, 0)
                    x2_ = min(x2+15, w-1)
                    z, valid = self._estimate_gate(x1_, x2_, y1, y2, x2-x1)
                else:
                    z, valid = self._estimate_object(cx_d, cy_d, bw_d, bh_d,
                                                     x1, x2, y1, y2, w, h)

                obj.valid    = valid
                obj.distance = float(z) if valid else -1.0

                if valid and self._fx is not None:
                    # Back-project centre pixel into camera frame
                    X = (cx_d - self._cx)   * z / self._fx
                    Y = (cy_d - self._cy_p) * z / self._fy
                    Z = z
                    obj.pose.position.x = float(X)
                    obj.pose.position.y = float(Y)
                    obj.pose.position.z = float(Z)
                    pos3d = (float(X), float(Y), float(Z))
                else:
                    pos3d = (float('nan'), float('nan'), float('nan'))
            else:
                pos3d = (float('nan'), float('nan'), float('nan'))

            # ── Stability tracking ────────────────────────────────
            tracker = self._get_tracker(label)
            obj.is_stable = tracker.update(
                detected=True, pos=pos3d, conf=confidence)
            detected_labels.add(label)

            out.objects.append(obj)

        # Advance trackers for labels not seen this frame
        for label, tracker in self._trackers.items():
            if label not in detected_labels:
                tracker.update(detected=False)

        self.pub.publish(out)

    # ── Depth estimation helpers ────────────────────────────────────

    def _estimate_gate(self, x1, x2, y1, y2, bw_d):
        """
        Gate-specific depth: sample columns, check left-right symmetry.
        Returns (z_metres, valid_bool).
        """
        column_depths = []
        for u in range(x1, x2, self._gate_col_step):
            col = self.depth_img[y1:y2, u]
            col = col[np.isfinite(col)]
            col = col[col > self._depth_min_range]
            col = col[col < 30]
            if len(col) >= self._gate_col_min_pts:
                column_depths.append((u, float(np.percentile(col, 10))))

        if len(column_depths) < self._gate_col_min_samples:
            # self.get_logger().info(f'Gate rejected: only {len(column_depths)} valid columns (need {self._gate_col_min_samples})')
            return float('nan'), False

        left = min(column_depths[:15], key=lambda x: x[1])
        right = min(column_depths[len(column_depths)-15:len(column_depths)], key=lambda x: x[1])
        # self.get_logger().info(f'Left: {left}, Right: {right}')

        width_pix  = right[0] - left[0]
        depth_sym  = abs(left[1] - right[1])

        if width_pix > bw_d * self._gate_w_ratio and \
                depth_sym < self._gate_sym_thresh:
            z = (left[1] + right[1]) / 2.0
            return z, True

        # self.get_logger().info(f'Gate rejected: width_pix={width_pix:.1f} vs {bw_d * self._gate_w_ratio:.1f}, '
        #     f'depth_sym={depth_sym:.2f} vs {self._gate_sym_thresh:.2f}')
        return float('nan'), False

    def _estimate_object(self, cx_d, cy_d, bw_d, bh_d,
                         x1, x2, y1, y2, w, h):
        """
        General object depth: sample the centre crop of the bounding box.
        Returns (z_metres, valid_bool).
        """
        cw = bw_d * self._crop_ratio
        ch = bh_d * self._crop_ratio
        c_x1 = max(0, int(cx_d - cw / 2))
        c_x2 = min(w - 1, int(cx_d + cw / 2))
        c_y1 = max(0, int(cy_d - ch / 2))
        c_y2 = min(h - 1, int(cy_d + ch / 2))

        roi = self.depth_img[c_y1:c_y2, c_x1:c_x2]
        roi = roi[np.isfinite(roi)]
        roi = roi[roi > self._depth_min_range]

        if len(roi) < self._obj_min_pts:
            # Fallback: full bounding box
            self.get_logger().debug(
                f'Fallback to full bbox (centre crop had {len(roi)} pts)')
            roi = self.depth_img[y1:y2, x1:x2]
            roi = roi[np.isfinite(roi)]
            roi = roi[roi > self._depth_min_range]

        if len(roi) > 0:
            z = float(np.percentile(roi, self._obj_pct))
            return z, True

        return float('nan'), False

    # ── Tracker factory ────────────────────────────────────────────

    def _get_tracker(self, label: str) -> _ObjectTracker:
        if label not in self._trackers:
            self._trackers[label] = _ObjectTracker(**self._stab_kwargs)
        return self._trackers[label]


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = DepthPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
