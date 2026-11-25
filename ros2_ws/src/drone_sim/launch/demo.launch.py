"""启动 drone_sim offboard 演示节点 (需先启动 MicroXRCEAgent 与 PX4 SITL)"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_sim',
            executable='offboard_demo',
            name='drone_sim_offboard_demo',
            output='screen',
            parameters=[{
                'takeoff_altitude': 5.0,
                'circle_radius': 3.0,
                'circle_speed': 0.3,
                'circle_duration': 25.0,
            }],
        ),
    ])
