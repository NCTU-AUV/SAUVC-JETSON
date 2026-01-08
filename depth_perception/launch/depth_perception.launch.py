from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_viz = LaunchConfiguration('use_viz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_viz',
            default_value='true',
            description='Enable visualization'
        ),

        Node(
            package='depth_perception',
            executable='depth_perception_node.py',
            name='depth_perception',
            output='screen'
        ),

        Node(
            package='depth_perception',
            executable='depth_perception_viz_node.py',
            name='depth_perception_viz',
            output='screen',
            condition=IfCondition(use_viz)
        ),
    ])
