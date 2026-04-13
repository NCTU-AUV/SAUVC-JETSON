# Copyright 2024 NCTU-AUV
# SPDX-License-Identifier: Apache-2.0
#
# Unified perception pipeline for SAUVC AUV:
#   camera_selector  ──►  YOLOv8 (TensorRT)  ──►  depth_perception
#
# All NITROS-capable nodes run inside a single composable container
# for zero-copy intra-process communication.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────
    launch_args = [
        DeclareLaunchArgument(
            'model_file_path',
            default_value=os.path.join(
                '/workspaces/isaac_ros-dev', 'src', 'best.onnx'),
            description='Absolute path to the YOLOv8 ONNX model'),
        DeclareLaunchArgument(
            'engine_file_path',
            default_value=os.path.join(
                '/workspaces/isaac_ros-dev', 'src', 'best.plan'),
            description='Absolute path to the TensorRT engine file'),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.25',
            description='YOLOv8 confidence threshold'),
        DeclareLaunchArgument(
            'nms_threshold',
            default_value='0.45',
            description='YOLOv8 NMS IOU threshold'),
        DeclareLaunchArgument(
            'use_viz',
            default_value='false',
            description='Enable depth perception visualization'),
        DeclareLaunchArgument(
            'force_engine_update',
            default_value='False',
            description='Force TensorRT engine rebuild'),
    ]

    # ── Resolve launch configurations ─────────────────────────────
    model_file_path = LaunchConfiguration('model_file_path')
    engine_file_path = LaunchConfiguration('engine_file_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    nms_threshold = LaunchConfiguration('nms_threshold')
    use_viz = LaunchConfiguration('use_viz')
    force_engine_update = LaunchConfiguration('force_engine_update')

    # ── Composable nodes ──────────────────────────────────────────

    # 1) Camera selector — zero-copy forwarding in the container
    camera_selector_node = ComposableNode(
        name='camera_selector_node',
        package='camera_selector',
        plugin='camera_selector::CameraSelectorNode',
        parameters=[],
    )

    # 2) TensorRT inference node
    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'model_file_path': model_file_path,
            'engine_file_path': engine_file_path,
            'output_binding_names': ['output0'],
            'output_tensor_names': ['output_tensor'],
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['images'],
            'verbose': False,
            'force_engine_update': force_engine_update,
        }],
    )

    # 3) YOLOv8 decoder node
    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[{
            'confidence_threshold': confidence_threshold,
            'nms_threshold': nms_threshold,
        }],
    )

    # ── Composable container (shared process) ─────────────────────
    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_selector_node,
            tensor_rt_node,
            yolov8_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # ── DNN image encoder (attaches to our container) ─────────────
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    dnn_image_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch',
                         'dnn_image_encoder.launch.py')),
        launch_arguments={
            'input_image_width': '640',
            'input_image_height': '640',
            'network_image_width': '640',
            'network_image_height': '640',
            'image_mean': '[0.0, 0.0, 0.0]',
            'image_stddev': '[1.0, 1.0, 1.0]',
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'perception_container',
            'dnn_image_encoder_namespace': 'yolov8_encoder',
            'image_input_topic': '/orca/selected/image_raw',
            'camera_info_input_topic': '/orca/selected/camera_info',
            'tensor_output_topic': '/tensor_pub',
        }.items(),
    )

    # ── YOLOv8 visualizer (optional) ────────────────────────────────
    yolov8_visualizer_node = Node(
        package='isaac_ros_yolov8',
        executable='isaac_ros_yolov8_visualizer.py',
        name='yolov8_visualizer',
        output='screen',
        condition=IfCondition(use_viz),
    )

    # ── Image viewer (optional) ─────────────────────────────────────
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view',
        arguments=['/yolov8_processed_image'],
        condition=IfCondition(use_viz),
    )

    # ── Depth perception (Python — runs as standalone process) ────
    depth_perception_node = Node(
        package='depth_perception',
        executable='depth_perception_node.py',
        name='depth_perception',
        output='screen',
    )

    # ── Depth perception visualizer (optional) ────────────────────
    depth_perception_viz_node = Node(
        package='depth_perception',
        executable='depth_perception_viz_node.py',
        name='depth_perception_viz',
        output='screen',
        condition=IfCondition(use_viz),
    )

    # ── Assemble launch description ───────────────────────────────
    return LaunchDescription(
        launch_args + [
            perception_container,
            dnn_image_encoder_launch,
            depth_perception_node,
            depth_perception_viz_node,
            yolov8_visualizer_node,
            image_view_node,
        ]
    )
