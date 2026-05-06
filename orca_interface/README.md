# orca_interface

Shared ROS 2 message definitions for the Orca AUV system.

This package contains no nodes — it solely provides custom `.msg` types consumed by the **perception pipeline** and the **decision system**.

## Messages

| Message | Description |
|---------|-------------|
| `PerceptionObject.msg` | Single detected object with 2-D bbox (YOLO pixel coords), 3-D pose (RealSense mode), depth distance, single-frame validity flag, and multi-frame stability flag. |
| `PerceptionArray.msg` | Stamped array of `PerceptionObject` published by `depth_perception` on `/orca/perception_array`. |
| `DecisionStatus.msg` | Telemetry message from the decision node containing mission phase, current BT action, target lock state, mission time, camera mode, and debug string. |

## Dependencies

- `std_msgs`
- `geometry_msgs`
- `rosidl_default_generators` / `rosidl_default_runtime`

## Build

```bash
colcon build --packages-select orca_interface
source install/setup.bash
```

After building, the messages are available as `orca_interface/msg/PerceptionObject`, etc. in both C++ and Python.
