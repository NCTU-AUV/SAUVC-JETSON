# depth_perception

Depth-based perception package for SAUVC AUV missions.  
This package fuses 2D object detections with depth images to produce
geometry-aware perception results suitable for decision-making and control.

---

## 1. Overview

`depth_perception` is a ROS2 package built with **ament_cmake** and
implemented using **Python nodes**.  

Key design principles:

- Single source of truth for depth estimation
- Robust gate-aware depth logic
- Clear separation between perception, visualization, and decision logic
- ROS2-native custom message interfaces

---

## 2. Package Architecture

```
┌──────────────────────────┐
│     YOLOv8 Detection     │
│    (isaac_ros_yolov8)    │
└───────────┬──────────────┘
            │
            │  /detections_output
            │  (vision_msgs/Detection2DArray)
            ▼
┌──────────────────────────────────────┐
│       depth_perception package       │
│                                      │
│  ┌────────────────────────────────┐  │
│  │  depth_estimator_node (Python) │  │
│  │                                │  │
│  │  Inputs:                       │  │
│  │   - /detections_output         │◄─┼── YOLO bbox
│  │   - /realsense/depth_image     │◄─┼── Depth image
│  │                                │  │
│  │  Internal logic:               │  │
│  │   - Gate-specific:             │  │
│  │    Scan depth columns in bbox  │  │
│  │    Setect left/right candidates│  │
│  │    Enforce width consistency   │  │
│  │    Enforce depth symmetry      │  │
│  │   - Non-gate objects:          │  │
│  │    ROI-based depth percentile  │  │
│  │                                │  │
│  │  Output:                       │  │
│  │   - /perception_array          │──┼──►
│  │     (PerceptionArray.msg)      │  │
│  └────────────────────────────────┘  │
│                                      │
│  ┌────────────────────────────────┐  │
│  │ depth_estimator_viz_node       │  │
│  │        (Python, optional)      │  │
│  │                                │  │
│  │  Inputs:                       │  │
│  │   - /perception_array          │◄─┼── Depth results
│  │   - /yolov8_processed_image    │◄─┼── RGB image
│  │                                │  │
│  │  Function:                     │  │
│  │   - Draw bbox                  │  │
│  │   - Annotate depth & validity  │  │
│  │   - No depth computation       │  │
│  │                                │  │
│  │  Output:                       │  │
│  │   - OpenCV window (debug only) │  │
│  │                                │  │
│  │  > disabled during competition │  │
│  └────────────────────────────────┘  │
│                                      │
└──────────────────────────────────────┘
            │
            │  /perception_array
            ▼
┌──────────────────────────┐
│    Decision / FSM Node   │
└──────────────────────────┘
```

---

## 3. Custom Message Definitions

### 3.1 PerceptionObject.msg

```text
string class_name

float32 cx
float32 cy
float32 width
float32 height

float32 distance
bool valid
````

---

### 3.2 PerceptionArray.msg

```text
std_msgs/Header header
PerceptionObject[] objects
```

---

## 4. Build & Run

### Build

```bash
colcon build --packages-select depth_perception --symlink-install
source install/setup.bash
```

### Verify interfaces

```bash
ros2 interface show depth_perception/msg/PerceptionArray
```

### Launch

```bash
ros2 launch depth_perception depth_estimation.launch.py use_viz:=true
```

Disable visualization for competition runs:

```bash
ros2 launch depth_perception depth_estimation.launch.py use_viz:=false
```
