# depth_perception

Unified perception funnel for the SAUVC AUV — converts raw YOLOv8 detections into enriched, stability-tracked `PerceptionObject` messages.

## Overview

This package contains three Python nodes:

| Node | Script | Role |
|------|--------|------|
| `depth_perception` | `depth_perception_node.py` | Main perception funnel: subscribes to depth image + detections, performs 3-D pose estimation (RealSense mode) or 2-D passthrough (USB mode), runs multi-frame stability tracking, and publishes `PerceptionArray`. |
| `depth_perception_viz` | `depth_perception_viz_node.py` | Optional visualiser: overlays distance + stability info on the YOLO-annotated image. |
| `depth_record` | `depth_record_node.py` | Renders depth to 8-bit grayscale JPEG for the bag recorder. Launched only under `record_images:=true`. |

## Depth in the bag

`depth_record_node.py` exists because the bag needs depth to be *small*, not
*exact*. Raw 32FC1 at 640x480 x 30 Hz measures 35.94 MB/s and used to be ~95%
of an image-enabled bag; the grayscale render measures 0.30 MB/s at
10.0 KB/frame — **123x smaller** (sim, 2026-08-19).

It applies the same mapping `web_video_server` serves the Web GUI's DEPTH
panel, so a bag frame and the live panel look identical:

```
mono8 = clip((depth_m - min_depth_m) / (max_depth_m - min_depth_m), 0, 1) * 255
```

Near is dark, far is bright. 16UC1 sources are converted from millimetres
first, so the window is always in metres regardless of whether the frames come
from the RealSense or from Gazebo.

**`+inf` is far, not invalid.** A depth camera returns `+inf` for any ray that
hits nothing — 5% of a typical pool frame — and those pixels saturate white,
matching `cv::min(inf, max)` in `web_video_server`. Folding them to black
instead would paint the emptiest part of the scene as if it were pressed
against the lens. Genuine no-data (NaN, or the 0 a 16UC1 sensor writes for
no-return) stays black.

**This throws away the metric depth.** 8 bits over 10 m is ~4 cm per code and
JPEG is lossy on top, so a bag recorded this way cannot be replayed through
`_estimate_gate`. `/orca/perception_array` is still recorded, so what the gate
logic concluded survives — only the input it concluded from does not. If a
particular run needs metric depth, record
`/orca/aligned_depth_to_color/image_raw` separately.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `depth_image_topic` | `/orca/aligned_depth_to_color/image_raw` | Source. Set by `perception.launch.py` from `/depth_perception`'s value and **not** overridable from YAML, so sim/real is resolved in one place. |
| `output_topic` | `/orca/record/depth/compressed` | Fixed name `record_topics.yaml` subscribes to; also set by the launch file rather than YAML. |
| `min_depth_m` | 0.0 | Near end of the ramp (black) |
| `max_depth_m` | 10.0 | Far end (white). Matches the GUI; the pool runs to ~16 m, so the far wall saturates. |
| `jpeg_quality` | 80 | Depth renders are smooth gradients — higher buys little |

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
