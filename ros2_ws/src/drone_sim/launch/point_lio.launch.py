"""E5 Point-LIO 3D 激光惯性里程计 launch
用法 (栈需 walls + gz_x500_lidar_3d):
  ros2 launch drone_sim point_lio.launch.py
输出: /Odometry 里程计, /cloud_registered 配准点云, /path 轨迹
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_sim')
    ns_arg = DeclareLaunchArgument('ns', default_value='/x500_lidar_3d_0')

    adapter = Node(
        package='drone_sim',
        executable='lidar3d_adapter',
        name='lidar3d_adapter',
        output='screen',
        parameters=[{
            'input_topic': [LaunchConfiguration('ns'), '/lidar_3d/points'],
            'output_topic': '/velodyne_points',
            'use_sim_time': True,
        }],
    )

    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'point_lio_gz.yaml'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([ns_arg, adapter, point_lio])
