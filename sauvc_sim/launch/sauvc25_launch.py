import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the world file from the package
    pkg_sauvc_sim = get_package_share_directory('sauvc_sim')
    world_file = os.path.join(pkg_sauvc_sim, 'worlds', 'sauvc25.world')

    # Launch Gazebo with the world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r']
        }.items()
    )

    # Single bridge node for all topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        remappings=[
            ('/cmd_vel', '/fsm/cmd_vel'),
        ],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/realsense/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/realsense/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/realsense/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/realsense/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/bottom_cam@sensor_msgs/msg/Image@gz.msgs.Image',
            '/side_cam@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/pool_world/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        output='screen'
    )

    # Entity spawner node
    entity_spawner = Node(
        package='sauvc_sim',
        executable='entity_spawner.py',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        entity_spawner,
    ])
