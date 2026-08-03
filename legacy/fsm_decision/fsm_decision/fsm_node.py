#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool
from decision_interface.msg import FSMStatus
from depth_perception.msg import PerceptionArray

from fsm_decision.mission_loader import MissionLoader

# 假設你之後會有這些
# from fsm_msgs.msg import FSMStatus
# from perception_msgs.msg import PerceptionArray
# from control_msgs.msg import ControlFeedback
# from fsm_decision.actions.hover import HoverAction
# from fsm_decision.actions.searchtarget import SearchTargetAction


FSM_TICK_HZ = 10.0
FSM_DT = 1.0 / FSM_TICK_HZ

isaac_ros_ws = os.environ.get("ISAAC_ROS_WS")
if isaac_ros_ws is None:
    raise EnvironmentError("ISAAC_ROS_WS environment variable is not set")

class FSMState(Enum):
    INIT = 0
    IDLE = 1
    LOAD_MISSION = 2
    INIT_ACTION = 3
    EXEC_ACTION = 4
    ACTION_DONE = 5
    SAFE_HALT = 99


class FSMNode(Node):

    def __init__(self):
        super().__init__('fsm_decision')

        # =====================
        # Publishers (FSM namespace)
        # =====================
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/fsm/cmd_vel', 10
        )
        self.arm_pub = self.create_publisher(
            Int8, '/fsm/arm', 10
        )
        self.hand_pub = self.create_publisher(
            Int8, '/fsm/hand', 10
        )

        self.status_pub = self.create_publisher(
            FSMStatus, '/fsm/status', 10
        )

        # =====================
        # Subscribers
        # =====================
        self.perception_sub = self.create_subscription(
            PerceptionArray,
            '/perception_array',
            self.perception_cb,
            10
        )

        # self.control_fb_sub = self.create_subscription(
        #     ControlFeedback,
        #     '/control_feedback',
        #     self.control_feedback_cb,
        #     10
        # )

        self.start_mission_sub = self.create_subscription(
            Bool,
            '/fsm/start_mission',
            self.start_mission_cb,
            10
        )

        # =====================
        # FSM internal state
        # =====================
        self.fsm_state = FSMState.INIT
        self.current_action = None
        self.current_action_idx = -1
        self.mission_actions = []

        self.start_mission_requested = False
        self.mission_active = False

        self.mission_loader = MissionLoader(
            mission_dir=os.path.join(
                isaac_ros_ws,
                "src/fsm_decision/fsm_decision/missions"
            )
        )

        self.last_cmd_vel = Twist()
        self.latest_perception = None
        self.latest_control_fb = None
        self.progress = 0.0
        self.msg = ''

        # FSM timer
        self.timer = self.create_timer(FSM_DT, self.fsm_tick)

        self.get_logger().info('FSM Decision Node initialized')

    # =====================
    # Callbacks
    # =====================
    def perception_cb(self, msg):
        self.latest_perception = msg

    def control_feedback_cb(self, msg):
        self.latest_control_fb = msg

    def start_mission_cb(self, msg: Bool):
        if msg.data:
            if not self.mission_active:
                self.get_logger().info('Start mission requested')
                self.start_mission_requested = True
            else:
                self.get_logger().warn('Mission already active, ignore start')


    # =====================
    # FSM main loop
    # =====================
    def fsm_tick(self):
        """
        Called at fixed rate (10 Hz).
        """
        # ---------- Global safety check ----------
        if self.latest_control_fb is not None:
            if self.latest_control_fb.kill_switch == 1:
                self.enter_safe_halt('Kill switch activated')
                return

        # ---------- FSM State Machine ----------
        if self.fsm_state == FSMState.INIT:
            self.handle_init()

        elif self.fsm_state == FSMState.IDLE:
            self.handle_idle()

        elif self.fsm_state == FSMState.LOAD_MISSION:
            self.handle_load_mission()

        elif self.fsm_state == FSMState.INIT_ACTION:
            self.handle_init_action()

        elif self.fsm_state == FSMState.EXEC_ACTION:
            self.handle_exec_action()

        elif self.fsm_state == FSMState.ACTION_DONE:
            self.handle_action_done()

        elif self.fsm_state == FSMState.SAFE_HALT:
            self.handle_safe_halt()

        else:
            self.get_logger().error('Unknown FSM state')
        
        self.publish_status()

    # =====================
    # FSM state handlers
    # =====================
    def handle_init(self):
        self.get_logger().info('FSM INIT')
        self.fsm_state = FSMState.IDLE

    def handle_idle(self):
        if self.start_mission_requested:
            self.start_mission_requested = False
            self.mission_active = True
            self.fsm_state = FSMState.LOAD_MISSION

    def handle_load_mission(self):
        self.get_logger().info('Loading mission')
        try:
            self.mission_actions = self.mission_loader.load(
                'test.yaml'
            )
        except Exception as e:
            self.enter_safe_halt(str(e))
            return
        
        self.current_action_idx = 0
        self.fsm_state = FSMState.INIT_ACTION

    def handle_init_action(self):
        if self.current_action_idx >= len(self.mission_actions):
            self.get_logger().info('Mission completed')
            # self.mission_active = False
            self.fsm_state = FSMState.IDLE
            return

        self.current_action = self.mission_actions[self.current_action_idx]
        self.get_logger().info(
            f'Init action [{self.current_action_idx}] '
            f'{self.current_action.name}'
        )

        self.current_action.on_enter(self)
        self.fsm_state = FSMState.EXEC_ACTION

    def handle_exec_action(self):
        cmd_vel, arm_cmd, hand_cmd = self.current_action.update(self)

        # 發布 action 指令（如果有）
        if cmd_vel is not None:
            self.cmd_vel_pub.publish(cmd_vel)
            self.last_cmd_vel = cmd_vel
        else:
            # 保持或發 0，避免 watchdog
            self.cmd_vel_pub.publish(self.last_cmd_vel)

        if arm_cmd is not None:
            self.arm_pub.publish(arm_cmd)

        if hand_cmd is not None:
            self.hand_pub.publish(hand_cmd)

        # Action 結束條件
        if self.current_action.is_success(self):
            self.fsm_state = FSMState.ACTION_DONE

        elif self.current_action.is_failed(self):
            self.enter_safe_halt('Action failed')

    def handle_action_done(self):
        self.get_logger().info(
            f'Action [{self.current_action_idx}] done'
        )
        self.progress = 0.0
        self.msg = ''
        self.current_action_idx += 1
        self.fsm_state = FSMState.INIT_ACTION

    def handle_safe_halt(self):
        # 持續輸出 0 cmd_vel
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    # =====================
    # Safety
    # =====================
    def enter_safe_halt(self, reason: str):
        self.get_logger().error(f'SAFE_HALT: {reason}')
        self.fsm_state = FSMState.SAFE_HALT

    def publish_status(self):
        status = FSMStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.fsm_state = self.fsm_state.name
        status.current_action = self.mission_actions[self.current_action_idx].name if self.current_action_idx >=0 and self.current_action_idx < len(self.mission_actions) else "None"
        status.action_index = self.current_action_idx
        status.action_total = len(self.mission_actions)
        status.progress = self.progress
        status.message = self.msg
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
