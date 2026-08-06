# Copyright 2024 NCTU-AUV
# SPDX-License-Identifier: Apache-2.0
#
# Unified perception pipeline for SAUVC AUV:
#   camera_selector  ──►  YOLOv8 (TensorRT)  ──►  depth_perception
#                                                         │
#                                              /orca/perception_array
#
# All NITROS-capable nodes run inside a single composable container
# for zero-copy intra-process communication.
# depth_perception runs as a standalone Python process.
#
# Usage:
#   ros2 launch orca_perception perception.launch.py
#   ros2 launch orca_perception perception.launch.py config_file:=/path/to/params.yaml
#   ros2 launch orca_perception perception.launch.py use_viz:=true          # X-dependent viewers
#   ros2 launch orca_perception perception.launch.py detection_overlay:=false  # drop the GUI overlay

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pkg_share = get_package_share_directory('orca_perception')
    default_config = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────
    launch_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Absolute path to the pipeline parameter YAML file'),
        DeclareLaunchArgument(
            'use_viz',
            default_value='',           # empty → fall back to YAML value
            description=('Debug viewers that open a window and therefore need an X server: '
                         'depth_perception_viz and rqt_image_view. Override the YAML value '
                         '(true/false); leave empty to use it.')),
        DeclareLaunchArgument(
            'detection_overlay',
            default_value='true',
            description=('Publish /yolov8_processed_image (detections drawn on the frame). '
                         'This is what the control GUI streams, so it defaults on — the node '
                         'only publishes a topic and needs no display, unlike use_viz.')),
        DeclareLaunchArgument(
            'namespace',
            default_value=os.environ.get('ORCA_NAMESPACE', 'orca_auv'),
            description=('Vehicle namespace. Expands $(ns) in the YAML topic names — '
                         'used for topics that come from the vehicle (camera, depth), '
                         'not for the pipeline-internal /orca/... topics.')),
    ]

    def create_nodes(context, *args, **kwargs):
        config_file = LaunchConfiguration('config_file').perform(context)
        use_viz_override = LaunchConfiguration('use_viz').perform(context)
        namespace = LaunchConfiguration('namespace').perform(context).strip('/')

        # ── Load YAML ─────────────────────────────────────────────
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f) or {}

        # ── Resolve use_viz: CLI arg wins; fall back to YAML ──────
        #
        # use_viz covers only the viewers that open a window: depth_perception_viz
        # calls cv2.imshow inside its subscription callback and rqt_image_view is
        # a Qt app, so both die on a host with no reachable X server — which is
        # every HEADLESS=true run, because that path skips xhost_grant while
        # DISPLAY stays set in the container.
        #
        # The YOLO overlay is deliberately NOT gated by it. That node only
        # publishes /yolov8_processed_image and never touches a display, and the
        # control GUI streams that topic, so folding it in here would make the
        # GUI's detection view silently empty on every headless run.
        if use_viz_override in ('true', 'True', '1'):
            use_viz = True
        elif use_viz_override in ('false', 'False', '0'):
            use_viz = False
        else:
            use_viz = bool(cfg.get('launch', {}).get('use_viz', False))

        use_viz_str = 'true' if use_viz else 'false'
        overlay_str = LaunchConfiguration('detection_overlay').perform(context)

        default_class_names = [
            'blue_drum', 'blue_flare', 'gate', 'orange_flare',
            'red_drum', 'red_flare', 'yellow_flare',
        ]

        # ── Model profile selection ───────────────────────────────
        # New configs can select one model profile and have it override the
        # TensorRT model/engine paths, decoder class count, and shared labels.
        # Older configs without model_profiles continue to use their node params
        # and top-level class_names unchanged.
        profile = {}
        model_profiles = cfg.get('model_profiles', {})
        if model_profiles:
            profile_name = cfg.get('model_profile', 'finals')
            if profile_name not in model_profiles:
                valid_profiles = ', '.join(sorted(model_profiles.keys()))
                raise RuntimeError(
                    f'Unknown model_profile {profile_name!r}. '
                    f'Valid profiles: {valid_profiles}')
            profile = model_profiles[profile_name] or {}

        # ── Class names shared by depth_perception + visualiser ────
        class_names = profile.get(
            'class_names',
            cfg.get('class_names', default_class_names))

        # ── DNN encoder dimensions from YAML ──────────────────────
        enc_cfg = cfg.get('dnn_image_encoder', {})
        enc_args = {
            'input_image_width':  str(enc_cfg.get('input_image_width',  640)),
            'input_image_height': str(enc_cfg.get('input_image_height', 640)),
            'network_image_width':  str(enc_cfg.get('network_image_width',  640)),
            'network_image_height': str(enc_cfg.get('network_image_height', 640)),
            'image_mean':   str(enc_cfg.get('image_mean',   '[0.0, 0.0, 0.0]')),
            'image_stddev': str(enc_cfg.get('image_stddev', '[1.0, 1.0, 1.0]')),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'perception_container',
            'dnn_image_encoder_namespace': 'yolov8_encoder',
            'image_input_topic': '/orca/selected/image_raw',
            'camera_info_input_topic': '/orca/selected/camera_info',
            'tensor_output_topic': '/tensor_pub',
        }

        # ── Helper: extract ros__parameters dict for a named node ──
        # We extract each node's ros__parameters sub-dict and pass it inline
        # to avoid rcl_yaml_param_parser tripping over non-standard top-level keys.
        #
        # `$(ns)` in any string value expands to the vehicle namespace. Only the
        # topics that actually come from the vehicle need it — in simulation the
        # Gazebo bridge publishes camera and depth under ORCA_NAMESPACE, so they
        # follow it, while the /orca/... pipeline-internal topics never do. On
        # the real robot the RealSense driver sits under a fixed /orca prefix
        # that is unrelated to the vehicle namespace, which is why
        # perception_params.yaml has no $(ns) at all.
        #
        # Writing the namespace literally made ORCA_NAMESPACE a half-working
        # knob: decision.launch.py remapped correctly onto the new namespace
        # while every perception node stayed subscribed to /orca_auv/..., so
        # /orca/perception_array went permanently empty with no error anywhere
        # and make status still looked healthy.
        def expand(value):
            if isinstance(value, str):
                return value.replace('$(ns)', namespace)
            if isinstance(value, list):
                return [expand(item) for item in value]
            return value

        def node_params(name):
            section = cfg.get(f'/{name}', cfg.get(name, {}))
            return {key: expand(value)
                    for key, value in section.get('ros__parameters', {}).items()}

        tensor_rt_params = node_params('tensor_rt')
        yolov8_decoder_params = node_params('yolov8_decoder_node')

        if profile:
            if 'model_file_path' in profile:
                tensor_rt_params['model_file_path'] = profile['model_file_path']
            if 'engine_file_path' in profile:
                tensor_rt_params['engine_file_path'] = profile['engine_file_path']
            if 'num_classes' in profile:
                yolov8_decoder_params['num_classes'] = int(profile['num_classes'])

        # ── Composable nodes ──────────────────────────────────────

        # 1) Camera selector — zero-copy forwarding in the container
        camera_selector_node = ComposableNode(
            name='camera_selector_node',
            package='camera_selector',
            plugin='camera_selector::CameraSelectorNode',
            parameters=[node_params('camera_selector_node')],
        )

        # 2) TensorRT inference node
        tensor_rt_node = ComposableNode(
            name='tensor_rt',
            package='isaac_ros_tensor_rt',
            plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
            parameters=[tensor_rt_params],
        )

        # 3) YOLOv8 decoder node
        yolov8_decoder_node = ComposableNode(
            name='yolov8_decoder_node',
            package='isaac_ros_yolov8',
            plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
            parameters=[yolov8_decoder_params],
        )

        # ── Composable container (shared process) ─────────────────
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

        # ── DNN image encoder (attaches to our container) ─────────
        encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
        dnn_image_encoder_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(encoder_dir, 'launch',
                             'dnn_image_encoder.launch.py')),
            launch_arguments=enc_args.items(),
        )

        # ── Depth perception (Python — standalone process) ─────────
        # Merge YAML ros__parameters with the top-level class_names so the node
        # gets the same label list as the visualiser without duplication in YAML.
        dp_params = node_params('depth_perception')
        dp_params['class_names'] = class_names

        depth_perception_node = Node(
            package='depth_perception',
            executable='depth_perception_node.py',
            name='depth_perception',
            output='screen',
            parameters=[dp_params],
        )

        # ── Depth perception visualiser (optional) ─────────────────
        dp_viz_params = node_params('depth_perception_viz')
        depth_perception_viz_node = Node(
            package='depth_perception',
            executable='depth_perception_viz_node.py',
            name='depth_perception_viz',
            output='screen',
            parameters=[dp_viz_params],
            condition=IfCondition(use_viz_str),
        )

        # ── YOLOv8 visualiser (optional) ──────────────────────────
        # Inject class_names from top-level so both nodes share the same list
        viz_params = node_params('yolov8_visualizer')
        viz_params['names'] = class_names

        yolov8_visualizer_node = Node(
            package='isaac_ros_yolov8',
            executable='isaac_ros_yolov8_visualizer.py',
            name='yolov8_visualizer',
            output='screen',
            parameters=[viz_params],
            condition=IfCondition(overlay_str),
        )

        # ── Image viewer (optional) ────────────────────────────────
        image_view_node = Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            arguments=['/yolov8_processed_image'],
            condition=IfCondition(use_viz_str),
        )

        return [
            perception_container,
            dnn_image_encoder_launch,
            depth_perception_node,
            depth_perception_viz_node,
            yolov8_visualizer_node,
            image_view_node,
        ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=create_nodes)])
