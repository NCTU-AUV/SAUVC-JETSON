# SAUVC-JETSON

ROS 2 workspace for the **Orca AUV**, running on Jetson Orin NX with Isaac ROS 3.2.

This repository contains the full autonomy stack: from camera input through object detection and depth estimation, to BehaviorTree-based mission execution.

## System Architecture

```
                         ┌──────────────────────────────────────────────────────────┐
                         │              perception_container (composable)           │
                         │                                                          │
  RealSense / USB  ──►   │  camera_selector ──► DNN encoder ──► TensorRT ──► YOLOv8 │
                         └──────────────────────────────────────┬───────────────────┘
                                                                │  /detections_output
                                                                ▼
                                                       depth_perception  (Python)
                                                                │
                                                                ▼
                                                     /orca/perception_array
                                                                │
                 /orca_auv/sensors/imu  ──►  ┌──────────────────┘
                                             ▼
                                       decision_node  (BehaviorTree.CPP)
                                             │
                            ┌────────────────┼────────────────────┐
                            ▼                ▼                    ▼
              wrench_sources/decision   targets/depth_m    decision/status
```

## Packages

Refer to README in each package folder for more details.

| Package | Location | Description |
|---------|----------|-------------|
| [`orca_interface`](orca_interface/) | `src/orca_interface` | Shared message definitions (`PerceptionObject`, `PerceptionArray`, `DecisionStatus`) |
| [`camera_selector`](perception_pipeline/camera_selector/) | `src/perception_pipeline/camera_selector` | Runtime RealSense ↔ USB camera switching (composable node) |
| [`isaac_ros_yolov8`](perception_pipeline/isaac_ros_yolov8/) | `src/perception_pipeline/isaac_ros_yolov8` | YOLOv8 TensorRT decoder + visualiser |
| [`depth_perception`](perception_pipeline/depth_perception/) | `src/perception_pipeline/depth_perception` | Depth estimation + multi-frame stability tracking |
| [`orca_perception`](perception_pipeline/orca_perception/) | `src/perception_pipeline/orca_perception` | Unified perception launcher & YAML config |
| [`orca_decision`](orca_decision/) | `src/orca_decision` | BehaviorTree.CPP mission decision system |

## Quick Start

### 1. Prerequisites

Follow the [Isaac ROS Getting Started](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html) guide to set up your workspace for Isaac ROS 3.2.
Complete **"Compute Setup"** and **"Developer Environment Setup"** (mind your platform: x86 / Jetson).

### 2. Clone the Repository

```bash
cd $ISAAC_ROS_WS/src
git clone --recurse-submodules git@github.com:NCTU-AUV/SAUVC-JETSON.git

# Flatten into src/
mv SAUVC-JETSON/* .
mv SAUVC-JETSON/.g* .
rm -rf SAUVC-JETSON
```

### 3. Configure Docker Image Layers

```bash
touch ~/.isaac_ros_common-config
echo "CONFIG_IMAGE_KEY=ros2_humble.realsense.orca25" >> ~/.isaac_ros_common-config
```

### 4. Set Up Alias & Enter Container

```bash
echo "alias isa='cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && ./scripts/run_dev.sh'" >> ~/.bashrc
source ~/.bashrc
isa    # builds the image on first run (~40 min), then enters bash
```

> **Do not build this image on the Jetson if you can avoid it.** The image is
> ~20 GB of content, 86% of which comes from NVIDIA's base layer (Triton +
> CUDA devel + PyTorch + TensorRT) — nothing we can trim. Build it once on an
> x86 machine, push it to a registry, and `pull` everywhere else:
>
> ```bash
> # on the x86 build machine
> ./scripts/orca_registry.sh build --arm64
> ./scripts/orca_registry.sh push
>
> # on the Jetson
> ./scripts/orca_registry.sh pull
> isa    # now reuses the pulled image instead of rebuilding
> ```
>
> Also: **never edit `docker/Dockerfile.base`, `.aarch64` or `.ros2_humble`.**
> `build_image_layers.sh` looks up NVIDIA's prebuilt images by an md5 of the
> Dockerfile chain — changing any of those files invalidates the lookup and
> forces a multi-hour from-scratch build of the 17.8 GB base. Project-specific
> changes belong in `docker/Dockerfile.orca25`, the outermost layer, which does
> not affect that hash.

To open additional terminals inside the **same container**, simply run `isa` from any new terminal.

### 5. Build the Workspace

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Full Pipeline

After entering the container with `isa`, open **three terminals** (each running `isa` to attach to the same container).

### Terminal 1 — Perception Pipeline

```bash
ros2 launch orca_perception perception.launch.py \
  config_file:=src/perception_pipeline/orca_perception/config/simulation_params.yaml
```

This launches the full perception stack: camera selector → DNN encoder → TensorRT → YOLOv8 decoder → depth perception.

### Terminal 2 — Decision Node

```bash
ros2 launch orca_decision decision.launch.py
```

This starts the BehaviorTree-based decision node, subscribing to perception output and IMU.

### Terminal 3 — Start Mission & Monitor

```bash
# Trigger mission start
ros2 topic pub --once /orca/decision/start_mission std_msgs/msg/Bool 'data: true'
```

Monitor the system with:

```bash
# Decision status (mission phase, current BT action, target lock, etc.)
ros2 topic echo /orca/decision/status

# Wrench output sent to thruster allocation
ros2 topic echo /orca_auv/control/wrench_sources/decision
```

---

## Gazebo Simulation

Simulation lives in its own repository and its own container:
[SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation). Start the
Gazebo world there, then run the perception + decision pipeline here with
`simulation_params.yaml` — the two containers share one ROS 2 graph.

Gazebo is deliberately **not** installed in this image any more (it used to be,
duplicating the simulation container for no benefit). The old in-repo
`sauvc_sim` package now lives in [`legacy/`](legacy/) and is not built.

> Cross-container discovery requires `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and
> the DDS transport to match across every container. In particular
> `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` — without it Fast DDS advertises shared
> memory locators that the other containers cannot reach, and participants
> match but data never flows, with no error message at all.

## Useful Debug Commands

```bash
# List all active nodes / topics
ros2 node list
ros2 topic list

# Inspect a specific topic
ros2 topic info /orca/perception_array
ros2 topic echo /orca/perception_array

# Check node parameters
ros2 param list /decision_node
ros2 param get /decision_node main_tree_id
```
