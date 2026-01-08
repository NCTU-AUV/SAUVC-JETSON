# fsm_decision

FSM-based decision node for AUV, designed for SAUVC.

This package implements a **mission-driven finite state machine (FSM)** that executes modular actions (primitive and perception-driven) and directly outputs low-level control commands (`/fsm/cmd_vel`, arm, hand).

The design goal is to allow:
- Clear separation between **decision logic**, **actions**, and **perception**
- Easy mission reconfiguration via separated YAML file
- Safe and debuggable execution during competition

---

## 1. Package Architecture Overview

### 1.1 High-Level Concept
```
Mission YAML
    ↓
Mission Loader
    ↓
[ Action List ]
    ↓
FSM Node (fsm_node.py)
    ↓
Actions (update / success / failure)
    ↓
Control Topics (/fsm/cmd_vel, /fsm/arm, /fsm/hand)
```
The FSM **does not implement behaviors itself**.  
All behaviors are encapsulated inside **Action classes**.

---

### 1.2 Directory Structure
```
fsm_decision/
├── fsm_decision/
│ ├── fsm_node.py           # Main FSM node
│ ├── mission_loader.py     # YAML → Action instances
│ │
│ ├── actions/
│ │ ├── action_base.py      # Base class for all actions
│ │ ├── hover.py            # Primitive action example
│ │ └── search_target.py    # Perception-driven action example
│ │
│ └── missions/
│ │ └── test.yaml           # Example mission
│ │
│ └── utils/                # shared tools (preserved, don't contain any files yet)
│   ├── time.py               # e.g. Time helpers
│   └── geometry.py           # e.g. Geometry utilities
│
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
---

### 1.3 FSM Responsibilities

The FSM node is responsible for:
- Managing FSM states:
  - `INIT`
  - `IDLE`
  - `LOAD_MISSION`
  - `INIT_ACTION`
  - `EXEC_ACTION`
  - `SAFE_HALT`
- Calling `Action.on_enter()`
- Calling `Action.update()` at **10 Hz**
- Checking `is_success()` / `is_failed()`
- Publishing:
  - `/fsm/cmd_vel` (`geometry_msgs/Twist`)
  - `/fsm/arm` (`std_msgs/Int8`)
  - `/fsm/hand` (`std_msgs/Int8`)
  - `/fsm/status` (debug)

The FSM **does not**:
- Interpret perception data
- Implement control laws
- Contain mission-specific logic

---

### 1.4 Action Concept

Each action represents **one logical behavior**.

Examples:
- Hover for 3 seconds
- Rotate to search for a gate
- Align yaw to a detected object

Actions are:
- Modular
- Testable in isolation
- Reusable across missions

---

## 2. Quick Start
### 2.1 Build the Package
```bash
colcon build --packages-select fsm_decision --symlink_install
source install/setup.bash
```

### 2.2 Example Mission
```
# missions/test.yaml
mission_name: test

actions:
  - type: Hover
    duration_s: 3.0

  - type: SearchTarget
    target_type: 2
```

### 2.3 Run the FSM Node
Run `ros2 run fsm_decision fsm_node`. After started in the next step,  you should see logs similar to:
```
[INFO] [1767868027.697050601] [fsm_decision]: FSM Decision Node initialized
[INFO] [1767868027.760994869] [fsm_decision]: FSM INIT
[INFO] [1767868037.015802622] [fsm_decision]: Start mission requested
[INFO] [1767868037.161144107] [fsm_decision]: Loading mission
[INFO] [1767868037.261317072] [fsm_decision]: Init action [0] Hover
[INFO] [1767868037.265780553] [fsm_decision]: [HoverAction] Start hovering for 3.0 s
[INFO] [1767868040.461076911] [fsm_decision]: Action [0] done
[INFO] [1767868040.562379908] [fsm_decision]: Init action [1] SearchTarget(2)
[INFO] [1767868040.563808194] [fsm_decision]: [SearchTarget] Searching for target: 2
[INFO] [1767868040.960341634] [fsm_decision]: Action [1] done
[INFO] [1767868041.061596730] [fsm_decision]: Mission completed
```

### 2.4 Start Mission Manually
Trigger the mission execution: </br>
`ros2 topic pub /fsm/start_mission std_msgs/Bool "{data: true}"`

### 2.5 Observe Outputs
```
ros2 topic echo /fsm/status
ros2 topic echo /fsm/cmd_vel
```

## 3. Intended Usage
- Competition mission logic → YAML
- Behavior logic → Actions
- Safety and sequencing → FSM

New behaviors should never be implemented inside fsm_node.py.</br>
See ACTION_DEVELOPMENT_GUIDE.md for extension instructions.