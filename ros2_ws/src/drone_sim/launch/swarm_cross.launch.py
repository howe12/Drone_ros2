"""E9 集群交叉飞行 demo: 3 机三角形交叉飞行

布局 (世界系, 高度 2.5m):
  x500_0: (-3, 0)  ->  ( 3, 0)
  x500_1: ( 3, 0)  ->  ( 0, 4)
  x500_2: ( 0, 4)  ->  (-3, 0)
三机路径在中心区交叉, 分布式势场避碰应保持 > 安全半径 (默认 2.0m) 间距。

用法:
  ros2 launch drone_sim swarm_cross.launch.py
  ros2 launch drone_sim swarm_cross.launch.py num:=3 safety_dist:=2.0
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    num_arg = DeclareLaunchArgument('num', default_value='3')
    world_arg = DeclareLaunchArgument('world', default_value='aruco')
    safety_arg = DeclareLaunchArgument('safety_dist', default_value='2.5')
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir', default_value=os.path.expanduser('/develop/IDEA/Drone_ros2/PX4-Autopilot'))

    multi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('drone_sim'), 'launch', 'multi_vehicle.launch.py'])),
        launch_arguments={
            'num': LaunchConfiguration('num'),
            'world': LaunchConfiguration('world'),
            'spawn': '-3,0;3,0;0,4',
            'px4_dir': LaunchConfiguration('px4_dir'),
        }.items())

    agents = []
    layout = [
        ('/x500_0', 3.0, 0.0, '/x500_1,/x500_2', 1, 0.0),
        ('/x500_1', 0.0, 4.0, '/x500_0,/x500_2', 2, 0.7),
        ('/x500_2', -3.0, 0.0, '/x500_0,/x500_1', 3, 1.4),
    ]
    for ns, tx, ty, nbrs, tsys, dz in layout:
        agents.append(Node(
            package='drone_sim', executable='swarm_agent', output='screen',
            name=f'swarm_agent_{ns.strip("/")}',
            parameters=[{
                'px4_ns': ns, 'target_x': tx, 'target_y': ty, 'target_z': 2.5 + dz,
                'target_system': tsys, 'log_file': f'/develop/IDEA/Drone_ros2/logs/e9_{ns.strip("/")}.log',
                'neighbors': nbrs, 'safety_dist': LaunchConfiguration('safety_dist'),
                'use_sim_time': True,
            }],
        ))

    return LaunchDescription([num_arg, world_arg, safety_arg, px4_dir_arg, multi] + agents)
