"""E2 航点任务 launch
用法:
  ros2 launch drone_sim waypoint_mission.launch.py                       # classic, 正方形
  ros2 launch drone_sim waypoint_mission.launch.py mission:=circle
  ros2 launch drone_sim waypoint_mission.launch.py prefix:=/x500_depth_0 mission:=spiral
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_sim')
    mission_arg = DeclareLaunchArgument('mission', default_value='square')
    prefix_arg = DeclareLaunchArgument('prefix', default_value='')
    speed_arg = DeclareLaunchArgument('speed', default_value='1.5')

    return LaunchDescription([
        mission_arg, prefix_arg, speed_arg,
        Node(
            package='drone_sim',
            executable='waypoint_mission',
            name='drone_sim_waypoint_mission',
            output='screen',
            parameters=[{
                'mission_file': [os.path.join(pkg, 'config', 'mission_'),
                                 LaunchConfiguration('mission'), '.yaml'],
                'topic_prefix': LaunchConfiguration('prefix'),
                'cruise_speed': LaunchConfiguration('speed'),
            }],
        ),
    ])
