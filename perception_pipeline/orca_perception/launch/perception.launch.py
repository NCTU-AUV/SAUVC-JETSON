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
#   ros2 launch orca_perception perception.launch.py record_images:=true       # compressed feeds for the bag

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
        DeclareLaunchArgument(
            'record_images',
            default_value='false',
            description=('Republish the four camera feeds compressed, under the fixed '
                         '/orca/record/* names the bag recorder subscribes to. Off by '
                         'default: it costs CPU and only the recorder consumes it.')),
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

        # ── Compressed feeds for the bag recorder (optional) ───────
        #
        # The recorder runs in the control container, which has only the core
        # image_transport package — the compression plugins live here, in the
        # Isaac image. So the republishing has to happen on this side; the
        # recorder just subscribes to the CompressedImage topics, which needs
        # nothing beyond sensor_msgs.
        #
        # Sources come out of the same config the pipeline itself uses, so the
        # sim/real topic split is already resolved and is not spelled out a
        # second time here. Outputs are fixed names under /orca/record, which
        # is what makes one recorder topic list work in both cases.
        record_nodes = []
        if IfCondition(LaunchConfiguration('record_images')).evaluate(context):
            selector_params = node_params('camera_selector_node')
            # Colour feeds go through image_transport. Depth cannot: the
            # compressedDepth plugin quantises 32FC1 into uint16 against a hard
            # 10 m ceiling this build exposes no parameter for, and the pool
            # runs out to 16 m, so the far wall would come back invalid.
            # depth_record_node handles it instead — see below.
            feeds = [
                ('front', selector_params.get('realsense_image_topic'), 'compressed'),
                ('bottom', selector_params.get('usb_image_topic'), 'compressed'),
            ]
            # Only worth republishing if something is drawing it.
            if IfCondition(LaunchConfiguration('detection_overlay')).evaluate(context):
                feeds.append(('detections', '/yolov8_processed_image', 'compressed'))

            for name, source, transport in feeds:
                if not source:
                    raise RuntimeError(
                        f'record_images: no source topic for {name} in {config_file}')
                record_nodes.append(Node(
                    package='image_transport',
                    executable='republish',
                    name=f'record_republish_{name}',
                    arguments=['raw', transport],
                    # republish resolves its output as out/<transport>; remapping
                    # the bare `out` does not reach it, because ROS 2 remapping
                    # matches whole names rather than prefixes.
                    remappings=[
                        ('in', source),
                        (f'out/{transport}', f'/orca/record/{name}/{transport}'),
                    ],
                    output='screen',
                ))

            # Depth is recorded as a picture, not as measurements: the same
            # 0-10 m grayscale render the GUI's DEPTH panel shows, JPEG'd.
            # Raw 32FC1 was ~37 MB/s, about 95% of an image-enabled bag; this
            # is roughly two orders of magnitude less. The trade is real —
            # a bag recorded this way can no longer be replayed through
            # _estimate_gate, because 8 bits over 10 m is ~4 cm per code and
            # everything past 10 m saturates. /orca/perception_array is still
            # recorded, so what the gate logic *concluded* is preserved even
            # though the input it concluded it from is not.
            depth_params = node_params('depth_perception')
            depth_source = depth_params.get('depth_image_topic')
            if not depth_source:
                raise RuntimeError(
                    f'record_images: no depth_image_topic for depth_perception '
                    f'in {config_file}')
            record_nodes.append(Node(
                package='depth_perception',
                executable='depth_record_node.py',
                name='depth_record',
                parameters=[{
                    # YAML first, wiring second — the two topic names are
                    # structural, not tuning. The source has to track
                    # depth_perception's so sim/real stays resolved in one
                    # place, and the output name is what record_topics.yaml
                    # subscribes to, so a YAML edit renaming either would
                    # silently record nothing. min/max_depth_m and
                    # jpeg_quality stay configurable.
                    **node_params('depth_record'),
                    'depth_image_topic': depth_source,
                    'output_topic': '/orca/record/depth/compressed',
                }],
                output='screen',
            ))

        return [
            perception_container,
            dnn_image_encoder_launch,
            depth_perception_node,
            depth_perception_viz_node,
            yolov8_visualizer_node,
            image_view_node,
        ] + record_nodes

    return LaunchDescription(launch_args + [OpaqueFunction(function=create_nodes)])
