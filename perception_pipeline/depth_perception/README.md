# depth_perception

Unified perception funnel for the SAUVC AUV — converts raw YOLOv8 detections into enriched, stability-tracked `PerceptionObject` messages.

## Overview

This package contains two Python nodes:

| Node | Script | Role |
|------|--------|------|
| `depth_perception` | `depth_perception_node.py` | Main perception funnel: subscribes to depth image + detections, performs 3-D pose estimation (RealSense mode) or 2-D passthrough (USB mode), runs multi-frame stability tracking, and publishes `PerceptionArray`. |
| `depth_perception_viz` | `depth_perception_viz_node.py` | Optional visualiser: overlays distance + stability info on the YOLO-annotated image. |

## Dual-Mode Operation

| Mode | Input | Output |
|------|-------|--------|
| **RealSense** | Depth image + detections + camera intrinsics | `PerceptionObject` with 3-D pose (`X, Y, Z`), distance, `valid=True/False` |
| **USB** | Detections only | `PerceptionObject` with `distance=-1.0`, `valid=False` |

In both modes the **multi-frame stability tracker** runs per-label and sets `is_stable=True` when:
- Object detected in ≥ `stability_min_hits` of the last `stability_window` frames
- Position standard deviation < `stability_pos_std_thresh`
- Confidence ≥ `stability_conf_thresh`

## Depth Estimation Strategies

- **Gate** (class index 2): Samples columns across the bounding box, checks left-right depth symmetry.
- **General objects**: Centre-crop percentile depth with full-bbox fallback.

## Subscribed Topics

| Topic (default) | Type |
|-----------------|------|
| `/orca/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` |
| `/orca/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/detections_output` | `vision_msgs/Detection2DArray` |
| `/orca/decision/camera_mode` | `std_msgs/String` |

## Published Topics

| Topic (default) | Type |
|-----------------|------|
| `/orca/perception_array` | `orca_interface/PerceptionArray` |

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `depth_min_range` | 0.3 | Discard depth below this (m) |
| `gate_symmetry_thresh` | 0.4 | Max L/R depth asymmetry for gate (m) |
| `obj_center_crop_ratio` | 0.5 | Fraction of bbox for centre-crop sampling |
| `obj_percentile` | 10 | Depth percentile for foreground estimation |
| `stability_window` | 10 | Rolling window length (frames) |
| `stability_min_hits` | 7 | Min detections in window |
| `stability_pos_std_thresh` | 0.15 | Max position std-dev (m) |
| `stability_conf_thresh` | 0.35 | Min confidence threshold |

All parameters are configurable via the central YAML config (see `orca_perception`).

## Dependencies

- `rclpy`, `cv_bridge`, `numpy`, `opencv`
- `sensor_msgs`, `std_msgs`, `vision_msgs`, `geometry_msgs`
- `orca_interface`
