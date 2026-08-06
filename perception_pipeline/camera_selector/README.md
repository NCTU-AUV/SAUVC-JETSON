# camera_selector

Runtime camera switching node for the SAUVC AUV perception pipeline.

Subscribes to both **RealSense** and **USB** camera image/info streams and republishes the currently selected camera's data on a unified output topic. The active camera can be switched at runtime via the mode topic.

## Node: `camera_selector_node`

Implemented as a **composable node** (C++ component) for zero-copy intra-process image transport when loaded in a shared container.

### Subscribed Topics

| Topic (default) | Type | Description |
|-----------------|------|-------------|
| `/orca/color/image_raw` | `sensor_msgs/Image` | RealSense colour image |
| `/orca/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense camera intrinsics |
| `/orca/usb_cam/image_raw` | `sensor_msgs/Image` | USB camera image |
| `/orca/usb_cam/camera_info` | `sensor_msgs/CameraInfo` | USB camera intrinsics |
| `/orca/decision/camera_mode` | `std_msgs/String` | Mode command: `"realsense"` or `"usb"` |

### Published Topics

| Topic (default) | Type | Description |
|-----------------|------|-------------|
| `/orca/selected/image_raw` | `sensor_msgs/Image` | Selected camera image |
| `/orca/selected/camera_info` | `sensor_msgs/CameraInfo` | Selected camera info (timestamp synced) |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `realsense_image_topic` | `/orca/color/image_raw` | RealSense image input |
| `realsense_info_topic` | `/orca/color/camera_info` | RealSense info input |
| `usb_image_topic` | `/orca/usb_cam/image_raw` | USB camera image input |
| `usb_info_topic` | `/orca/usb_cam/camera_info` | USB camera info input |
| `mode_topic` | `/orca/decision/camera_mode` | Mode selection topic |
| `selected_image_topic` | `/orca/selected/image_raw` | Unified image output |
| `selected_info_topic` | `/orca/selected/camera_info` | Unified info output |
| `default_mode` | `realsense` | Initial camera mode |

## Dependencies

- `rclcpp`, `rclcpp_components`
- `sensor_msgs`, `std_msgs`
