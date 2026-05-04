import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('orca_decision')
    config_file = os.path.join(pkg_share, 'config', 'decision_params.yaml')

    decision_node = Node(
        package='orca_decision',
        executable='decision_node',
        name='decision_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/orca/decision/wrench', '/orca_auv/control/wrench_sources/decision'),
            ('/orca/decision/desired_depth', '/orca_auv/control/targets/depth_m')
        ]
    )

    return LaunchDescription([
        decision_node
    ])
