"""E6 VINS-Fusion 单目+IMU VIO launch

链路: /x500_mono_cam_0/camera/image_raw + /x500_mono_cam_0/imu
      → vins_node (单目惯性紧耦合) → /vins_estimator/odometry, /vins_estimator/path
      → vio_evaluator 与 gz 真值对比

用法 (栈: xtd2 gz_x500_mono_cam baylands):
  ros2 launch drone_sim vins_mono.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_sim')
    ns = LaunchConfiguration('ns')
    cfg = os.path.join(pkg, 'config', 'vins_mono_imu.yaml')

    vins = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        arguments=[cfg],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cam0/image_raw', [ns, '/camera/image_raw']),
            ('/imu0', [ns, '/imu']),
        ],
    )

    ev = Node(
        package='drone_sim',
        executable='vio_evaluator',
        name='vio_evaluator',
        output='screen',
        parameters=[{
            'vins_topic': '/vins_estimator/odometry',
            'truth_topic': [ns, '/pose'],
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='/x500_mono_cam_0'),
        vins, ev,
    ])
