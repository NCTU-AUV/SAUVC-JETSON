"""Decision node only. For the full autonomy stack use autonomy.launch.py.

The node itself publishes and subscribes under a fixed ``/orca/...`` prefix;
everything that crosses into the control stack is remapped here onto the
vehicle namespace, so the namespace lives in exactly one place.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('orca_decision')
    config_file = os.path.join(pkg_share, 'config', 'decision_params.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value=os.environ.get('ORCA_NAMESPACE', 'orca_auv'),
            description='Vehicle namespace the control stack publishes under'),
        # 要跑哪一棵樹。留空就用 decision_params.yaml 裡的 main_tree_id。
        # 沒有這個參數的話，換一棵樹就得改 YAML、再 build 一次才會進 install
        # space —— 新生練習時每次換樹都要付一次 colcon 的代價。
        DeclareLaunchArgument(
            'main_tree_id',
            default_value='',
            description=('BehaviorTree ID to run, overriding decision_params.yaml '
                         '(e.g. StudentSimpleQualMission). Empty keeps the YAML value.')),
    ]

    def create_nodes(context, *args, **kwargs):
        ns = LaunchConfiguration('namespace').perform(context).strip('/')
        main_tree_id = LaunchConfiguration('main_tree_id').perform(context).strip()

        # 後面的 dict 覆蓋前面 YAML 的同名參數；留空時整個不加，YAML 說了算。
        parameters = [config_file]
        if main_tree_id:
            parameters.append({'main_tree_id': main_tree_id})

        decision_node = Node(
            package='orca_decision',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=parameters,
            remappings=[
                # Attitude/heading source. Without this the world model never
                # leaves its construction pose: TurnToYaw never converges,
                # heading lock is a no-op and GoToPose reports instant success.
                ('/orca/imu/data', f'/{ns}/sensors/imu'),
                ('/orca/decision/wrench', f'/{ns}/control/wrench_sources/decision'),
                ('/orca/decision/desired_depth', f'/{ns}/control/targets/depth_m'),
                # Ball electromagnet. Without this remap GrabBall and DropBall
                # published into the void — nothing anywhere subscribed to
                # /orca/decision/hand, so both were no-ops that waited a second
                # and reported SUCCESS. The control stack's actuator topic is
                # actuators/electromagnet/enabled, which the STM32 firmware
                # picks up over micro-ROS; the Web GUI publishes onto the same
                # topic, so the two command paths now agree.
                ('/orca/decision/hand', f'/{ns}/actuators/electromagnet/enabled'),
            ],
            # NOTE: /orca/decision/arm (ExtendArm/RetractArm, std_msgs/Int32)
            # still has no destination — the control stack has no arm actuator
            # topic at all, so there is nothing to remap onto. Those two nodes
            # remain no-ops until the arm's interface is defined.
        )

        return [decision_node]

    return LaunchDescription(launch_args + [OpaqueFunction(function=create_nodes)])
