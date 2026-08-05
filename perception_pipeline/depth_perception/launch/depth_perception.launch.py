"""Standalone depth_perception, for debugging it apart from the full pipeline.

The nodes run on their own declared defaults unless params_file is given, and
those defaults target the real robot.

Do NOT pass orca_perception's simulation_params.yaml or perception_params.yaml
here. Neither is a valid rcl parameter file: both carry non-node top-level keys
(`launch:`, `model_profile:`, `model_profiles:`, `class_names:`,
`dnn_image_encoder:`), and rcl_yaml_param_parser rejects the whole file with
"Cannot have a value before ros__parameters at line 8", so both nodes die inside
rclpy.init(). params_file here only accepts a plain ros__parameters file.

For the simulator use orca_perception/perception.launch.py instead — it extracts
each node's ros__parameters sub-dict and passes it inline, which is what makes
those YAMLs usable.
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
