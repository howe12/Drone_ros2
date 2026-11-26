"""E4 2D SLAM launch: odom->TF + slam_toolbox (use_sim_time, 需栈桥接 /clock)
用法:
  ros2 launch drone_sim slam_2d.launch.py
  # 前提: run_simulation.sh xtd2 gz_x500_lidar_2d walls 已启动
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_sim')
    ns_arg = DeclareLaunchArgument('ns', default_value='/x500_lidar_2d_0')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_2d.yaml')

    odom_tf = Node(
        package='drone_sim',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{
            'odom_topic': [LaunchConfiguration('ns'), '/odometry'],
            'use_sim_time': True,
        }],
    )

    # 静态 TF: base_link -> 激光坐标系 (gz scan frame_id = <model>/link/lidar_2d_v2)
    # 安装位姿来自 x500_lidar_2d/model.sdf: .12 0 .26
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        arguments=['0.12', '0', '0.26', '0', '0', '0',
                   'base_link', 'x500_lidar_2d_0/link/lidar_2d_v2'],
        parameters=[{'use_sim_time': True}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([ns_arg, odom_tf, lidar_tf, slam])
