# perception_pipeline

Parent directory for all perception-related packages of the Orca AUV.

| Package | Description |
|---------|-------------|
| [`camera_selector`](camera_selector/) | Runtime camera switching composable node (RealSense ↔ USB). |
| [`isaac_ros_yolov8`](isaac_ros_yolov8/) | YOLOv8 TensorRT decoder and bounding-box visualiser (adapted from NVIDIA Isaac ROS). |
| [`depth_perception`](depth_perception/) | Dual-mode depth estimation and multi-frame stability tracking; publishes `PerceptionArray`. |
| [`orca_perception`](orca_perception/) | Meta-package: single launch file + YAML config that orchestrates the entire pipeline. |

## Data Flow

```
RealSense / USB  ──►  camera_selector  ──►  DNN encoder  ──►  TensorRT
                                                                  │
                                                          yolov8_decoder
                                                                  │
                                                       depth_perception
                                                                  │
                                                    /orca/perception_array  ──►  decision node
```

See each package's README for detailed topic, parameter, and launch documentation.
