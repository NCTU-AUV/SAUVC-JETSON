"""The whole autonomy stack: perception pipeline + decision node.

This is the entry point the Makefile and the docs should use. Launching
decision.launch.py on its own brings up the BehaviorTree with nothing
publishing /orca/perception_array, so every search/approach node runs to its
timeout while the graph still looks healthy — the failure that made this file
necessary.

Usage:
  ros2 launch orca_decision autonomy.launch.py
  ros2 launch orca_decision autonomy.launch.py perception_config:=simulation_params.yaml
  ros2 launch orca_decision autonomy.launch.py use_perception:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


TRUE_VALUES = ('true', 'True', '1')


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value=os.environ.get('ORCA_NAMESPACE', 'orca_auv'),
            description='Vehicle namespace the control stack publishes under'),
        DeclareLaunchArgument(
            'use_perception',
            default_value='true',
            description='Start the perception pipeline alongside the decision node'),
        DeclareLaunchArgument(
            'perception_config',
            default_value='',
            description=('Perception parameter YAML. Either an absolute path, or a '
                         'bare file name resolved against orca_perception/config '
                         '(e.g. simulation_params.yaml). Empty uses the pipeline default.')),
        DeclareLaunchArgument(
            'use_viz',
            default_value='',
            description='Override use_viz from the perception YAML (true/false)'),
    ]

    def create_actions(context, *args, **kwargs):
        namespace = LaunchConfiguration('namespace').perform(context)
        use_perception = LaunchConfiguration('use_perception').perform(context)
        perception_config = LaunchConfiguration('perception_config').perform(context)
        use_viz = LaunchConfiguration('use_viz').perform(context)

        actions = []

        if use_perception in TRUE_VALUES:
            perception_share = get_package_share_directory('orca_perception')

            # A bare name is resolved against the pipeline's own config dir so
            # callers do not have to know the install-space layout.
            if perception_config and os.sep not in perception_config:
                perception_config = os.path.join(
                    perception_share, 'config', perception_config)

            if perception_config and not os.path.isfile(perception_config):
                raise RuntimeError(
                    f'perception_config not found: {perception_config}')

            perception_args = {}
            if perception_config:
                perception_args['config_file'] = perception_config
            if use_viz:
                perception_args['use_viz'] = use_viz

            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(perception_share, 'launch', 'perception.launch.py')),
                launch_arguments=perception_args.items(),
            ))

        decision_share = get_package_share_directory('orca_decision')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(decision_share, 'launch', 'decision.launch.py')),
            launch_arguments={'namespace': namespace}.items(),
        ))

        return actions

    return LaunchDescription(launch_args + [OpaqueFunction(function=create_actions)])
