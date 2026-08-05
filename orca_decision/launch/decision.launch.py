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
    ]

    def create_nodes(context, *args, **kwargs):
        ns = LaunchConfiguration('namespace').perform(context).strip('/')

        decision_node = Node(
            package='orca_decision',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                # Attitude/heading source. Without this the world model never
                # leaves its construction pose: TurnToYaw never converges,
                # heading lock is a no-op and GoToPose reports instant success.
                ('/orca/imu/data', f'/{ns}/sensors/imu'),
                ('/orca/decision/wrench', f'/{ns}/control/wrench_sources/decision'),
                ('/orca/decision/desired_depth', f'/{ns}/control/targets/depth_m'),
            ],
        )

        return [decision_node]

    return LaunchDescription(launch_args + [OpaqueFunction(function=create_nodes)])
