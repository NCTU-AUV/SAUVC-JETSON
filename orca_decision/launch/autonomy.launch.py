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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
        DeclareLaunchArgument(
            'record_images',
            default_value='false',
            description=('Republish the camera feeds compressed under /orca/record/* '
                         'for the bag recorder. Must be set together with the control '
                         "stack's own record_images — the two stacks launch separately.")),
    ]

    def create_actions(context, *args, **kwargs):
        namespace = LaunchConfiguration('namespace').perform(context)
        perception_config = LaunchConfiguration('perception_config').perform(context)
        use_viz = LaunchConfiguration('use_viz').perform(context)

        # launch 自己的條件語意：strip + lower，接受 true/1/false/0，其餘 raise。
        # 原本是拿 ('true', 'True', '1') 這個手寫 tuple 做比對，於是
        # make sim PERCEPTION=TRUE（或 yes、on、帶尾隨空白）會靜默走 false 分支，
        # 整條感知管線不啟動而沒有任何錯誤 —— Makefile 照樣印 perception=TRUE，
        # 而那正是這個檔案存在的理由。要失敗就要大聲失敗。
        use_perception = IfCondition(
            LaunchConfiguration('use_perception')).evaluate(context)

        actions = []

        if use_perception:
            perception_share = get_package_share_directory('orca_perception')

            # A bare name is resolved against the pipeline's own config dir so
            # callers do not have to know the install-space layout.
            if perception_config and os.sep not in perception_config:
                perception_config = os.path.join(
                    perception_share, 'config', perception_config)

            if perception_config and not os.path.isfile(perception_config):
                raise RuntimeError(
                    f'perception_config not found: {perception_config}')

            # namespace 一定要一起傳。少了它，decision_node 會正確 remap 到新的
            # namespace，而整條感知管線仍訂閱舊的 —— 兩邊都活著、都沒有錯誤，
            # 只有 /orca/perception_array 永遠是空的。
            perception_args = {
                'namespace': namespace,
                'record_images': LaunchConfiguration('record_images').perform(context),
            }
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
