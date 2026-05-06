# orca_perception

Unified perception pipeline launcher for the SAUVC AUV.

This meta-package contains **no nodes of its own** — it provides a single launch file and YAML configurations that orchestrate all perception components into a coherent pipeline:

```
 Camera streams
      │
      ▼
┌─────────────────────────────────────────────────┐
│          perception_container (composable)       │
│                                                  │
│  camera_selector ──► DNN encoder ──► TensorRT    │
│                                          │       │
│                                  yolov8_decoder   │
└──────────────────────────────────────┬──────────┘
                                       │ /detections_output
                                       ▼
                              depth_perception  (Python, standalone)
                                       │
                                       ▼
                            /orca/perception_array
```

## Launch

```bash
# With default config (perception_params.yaml)
ros2 launch orca_perception perception.launch.py

# With Gazebo simulation config
ros2 launch orca_perception perception.launch.py \
  config_file:=src/perception_pipeline/orca_perception/config/simulation_params.yaml

# Enable visualization overlay
ros2 launch orca_perception perception.launch.py use_viz:=true
```

## Configuration Files

| File | Description |
|------|-------------|
| `config/perception_params.yaml` | Default parameters for real-world deployment |
| `config/simulation_params.yaml` | Parameters tuned for Gazebo simulation (different camera topics, model paths) |

Both files share the same structure — a single YAML that configures **all** nodes:

| Section | Configures |
|---------|-----------|
| `launch.use_viz` | Enable/disable visualisation nodes |
| `class_names` | Ordered class labels (shared by decoder, visualiser, depth node) |
| `dnn_image_encoder` | Input/network image dimensions, normalisation |
| `/tensor_rt` | ONNX/TensorRT model paths, binding names |
| `/yolov8_decoder_node` | Confidence/NMS thresholds, number of classes |
| `/camera_selector_node` | Input/output camera topics, default mode |
| `/depth_perception` | Depth estimation thresholds, stability tracker params |
| `/depth_perception_viz` | Visualiser image topic and native resolution |
| `/yolov8_visualizer` | Class names for bounding box labels |

## Dependencies (runtime)

- `orca_interface`, `camera_selector`, `depth_perception`, `isaac_ros_yolov8`
- `isaac_ros_tensor_rt`, `isaac_ros_dnn_image_encoder`
- `launch`, `launch_ros`
