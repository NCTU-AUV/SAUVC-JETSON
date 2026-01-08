# Action Development Guide
This document explains how to design and implement new actions in the `fsm_decision` package.

Target audience:
- New team members
- Developers unfamiliar with the FSM architecture


## 1. Action Philosophy
An **Action** represents a single, self-contained behavior.

Examples:
- Hover for a duration
- Rotate to search for a target
- Align yaw to a detected object

FSM controls *when* actions run.  
Actions control *how* the vehicle behaves.

## 2. Action Base Class Contract
All actions **must inherit** from `ActionBase`.
```python
class ActionBase:
    def on_enter(self, context):
        ...

    def update(self, context):
        ...
        return cmd_vel, arm_cmd, hand_cmd

    def is_success(self, context) -> bool:
        ...

    def is_failed(self, context) -> bool:
        ...
```

- Context Object (Important)</br>
Actions do not subscribe to ROS topics. They receive a context object from the FSM,

## 3. Action Categories
### 3.1 Primitive Actions
Primitive actions do not depend on perception.
- Hover
- MoveForward
- RotateYaw
- HoldDepth

### 3.2 Perception-Driven Actions
Perception-driven actions react to sensor input.
- SearchTarget
- AlignToTarget
- ApproachTarget

## 4. Common Mistakes to Avoid
❌ Subscribing to ROS topics inside actions</br>
❌ Publishing inside actions</br>
❌ Changing FSM state inside actions</br>
❌ Long blocking computations in update()

## 5. Adding a New Action (Checklist)
1. Create new file in fsm_decision/actions/
2. Inherit from ActionBase
3. Implement action class methods
4. Import action and register action in ACTION_REGISTRY inside `mission_loader.py`
5. Add example YAML entry
6. Test with /fsm/start_mission

## 6. Design Principle Summary
- FSM controls sequence
- Actions control behavior
- Perception controls information
- YAML controls mission