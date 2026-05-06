# orca_decision

Autonomous mission decision system for the Orca AUV, built on **BehaviorTree.CPP v3**.

The node subscribes to perception output (`orca_interface/PerceptionArray`) and IMU data, maintains a **World Model** of tracked objects, executes a configurable **Behavior Tree** for mission orchestration, and outputs `geometry_msgs/Wrench` commands for thrust allocation.

## Architecture

```
/orca/perception_array ──► ┌──────────────┐
                           │              │ ──► /orca_auv/control/wrench_sources/decision  (Wrench)
/orca_auv/sensors/imu  ──► │ decision_node│ ──► /orca_auv/control/targets/depth_m          (Float64)
                           │              │ ──► /orca/decision/status                      (DecisionStatus)
/orca/decision/start   ──► └──────────────┘ ──► /orca/decision/camera_mode                 (String)
```

### Core Components

| Component | File | Role |
|-----------|------|------|
| **DecisionNode** | `src/decision_node.cpp` | ROS 2 node — loads BT, subscribes to sensors, publishes wrench at 50 Hz, ticks BT at 10 Hz. |
| **WorldModel** | `src/world_model.cpp` | Thread-safe tracker: fuses IMU dead-reckoning with perception to maintain object positions in a world frame. Stale objects are removed after `perception_timeout_sec`. |
| **WrenchAdapter** | `src/wrench_adapter.cpp` | Low-pass filtered wrench generator: converts `MotionCommand` (surge/sway/heave/yaw) to `geometry_msgs/Wrench` with configurable gains `k_surge`, `k_sway`, `k_yaw`. |

### Behavior Tree Nodes

| Node | Type | Description |
|------|------|-------------|
| `SearchTarget` | Action | Rotate to search for a target label. |
| `ApproachTarget` | Action | Approach a detected target until within a specified distance. |
| `FinalAlignTarget` | Action | Fine yaw+lateral alignment to the target. |
| `BlindForward` | Action | Drive forward for a fixed duration with optional heading lock. |
| `TurnToYaw` | Action | Turn to an absolute yaw angle. |
| `SetCamera` | Action | Switch the active camera mode (`realsense` / `usb`). |
| `SetDepth` | Action | Publish a desired depth target. |
| `AvoidObstacle` | Condition/Action | Reactive obstacle avoidance with sticky cooldown. |

## Configuration

### Parameters (`config/decision_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `k_surge` | 1.0 | Surge force gain |
| `k_sway` | 1.0 | Sway force gain |
| `k_yaw` | 0.5 | Yaw torque gain |
| `motion_lowpass_alpha` | 0.2 | Low-pass filter smoothing factor |
| `max_velocity_clamp` | 1.5 | Max estimated velocity (m/s) |
| `perception_timeout_sec` | 1.0 | Drop tracked objects unseen for this long |
| `velocity_decay` | 0.95 | Per-step velocity decay (drag model) |
| `tree_xml_file` | `config/trees.xml` | Path to BehaviorTree XML |
| `main_tree_id` | `QualificationMission` | Root tree to execute |
| `align_yaw_threshold` | 0.1 | Yaw alignment tolerance (rad) |
| `align_distance_threshold` | 0.5 | Distance alignment tolerance (m) |

### Mission Trees (`config/trees.xml`)

- **QualificationMission** — Submerge → pass gate → U-turn → return → surface.
- **FinalMission** — Extended mission with obstacle avoidance.
- **PassGateProcedure** — Reusable sub-tree: search → approach → align → blind forward.

## Launch

```bash
ros2 launch orca_decision decision.launch.py
```

The launch file automatically remaps:

| Internal Topic | Remapped To |
|----------------|-------------|
| `/orca/decision/wrench` | `/orca_auv/control/wrench_sources/decision` |
| `/orca/decision/desired_depth` | `/orca_auv/control/targets/depth_m` |

### Start a Mission

```bash
ros2 topic pub --once /orca/decision/start_mission std_msgs/msg/Bool 'data: true'
```

### Monitor

```bash
# Decision status (mission phase, current action, target lock, etc.)
ros2 topic echo /orca/decision/status

# Wrench output
ros2 topic echo /orca_auv/control/wrench_sources/decision
```

## Dependencies

- `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`
- `orca_interface` (custom messages)
- `behaviortree_cpp_v3`
- `tf2`
