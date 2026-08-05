"""Standalone depth_perception, for debugging it apart from the full pipeline.

The nodes run on their own declared defaults unless params_file is given. Those
defaults target the real robot; for the simulator pass orca_perception's
simulation_params.yaml, or use orca_perception/perception.launch.py instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'use_viz',
            default_value='true',
            description='Enable visualization'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional parameter YAML; empty means use node defaults'
        ),
    ]

    def create_nodes(context, *args, **kwargs):
        params_file = LaunchConfiguration('params_file').perform(context)
        parameters = [params_file] if params_file else []

        return [
            Node(
                package='depth_perception',
                executable='depth_perception_node.py',
                name='depth_perception',
                output='screen',
                parameters=parameters,
            ),
            Node(
                package='depth_perception',
                executable='depth_perception_viz_node.py',
                name='depth_perception_viz',
                output='screen',
                parameters=parameters,
                condition=IfCondition(LaunchConfiguration('use_viz')),
            ),
        ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=create_nodes)])
