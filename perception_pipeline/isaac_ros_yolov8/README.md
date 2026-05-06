# isaac_ros_yolov8

YOLOv8 object detection decoder and visualiser, adapted from [NVIDIA Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection) for the Orca AUV.

## Nodes

| Node | Language | Role |
|------|----------|------|
| `YoloV8DecoderNode` | C++ (composable) | Decodes TensorRT inference output into `vision_msgs/Detection2DArray`. Runs as a component inside the perception container for zero-copy transport. |
| `isaac_ros_yolov8_visualizer.py` | Python | Draws bounding boxes and class labels on the image and publishes to `/yolov8_processed_image`. |

## Parameters

### Decoder Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `confidence_threshold` | 0.25 | Minimum detection confidence |
| `nms_threshold` | 0.45 | Non-maximum suppression IoU threshold |
| `num_classes` | 7 | Number of classes in the model |

### Visualizer Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `names` | `[blue_drum, blue_flare, gate, ...]` | Ordered class label list (index = class id) |

## Launch Files (deprecated)

| Launch File | Description |
|-------------|-------------|
| `isaac_ros_yolov8_core.launch.py` | Core detection pipeline |
| `isaac_ros_yolov8_visualize.launch.py` | Detection + visualisation (RealSense) |
| `isaac_ros_yolov8_visualize_gazebo.launch.py` | Detection + visualisation (Gazebo) |
| `yolov8_tensor_rt.launch.py` | TensorRT inference (RealSense) |
| `yolov8_tensor_rt_gazebo.launch.py` | TensorRT inference (Gazebo) |

> **Note**: In the unified pipeline, these launch files are **not used directly**. The decoder and encoder are launched by `orca_perception/perception.launch.py` as composable nodes.

## Dependencies

- `rclcpp`, `rclcpp_components`
- `isaac_ros_nitros`, `isaac_ros_managed_nitros`, `isaac_ros_tensor_list_interfaces`
- `isaac_ros_tensor_rt`, `isaac_ros_dnn_image_encoder`
- `vision_msgs`, `sensor_msgs`, `cv_bridge`, `opencv`
