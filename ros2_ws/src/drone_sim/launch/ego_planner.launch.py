"""E7 ego-planner 局部规划避障全链路 launch

链路: 深度相机 → grid_map 局部占据栅格 → ego-planner B样条规划 → traj_server
      → /position_cmd → ego_bridge → PX4 TrajectorySetpoint(Offboard)

用法 (栈: xtd2 gz_x500_depth e7_obstacles):
  ros2 launch drone_sim ego_planner.launch.py
  ros2 launch drone_sim ego_planner.launch.py goal_x:=6.0 goal_y:=0.0 goal_z:=2.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')

    # 深度相机 (OakD-Lite): FOV 1.274 rad, 640x480 → fx=fy≈432.6
    fx = '432.0'
    fy = '432.0'
    cx = '320.0'
    cy = '240.0'

    ego = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name='ego_planner_node',
        output='screen',
        remappings=[
            ('odom_world', [ns, '/odometry']),
            ('grid_map/depth', [ns, '/depth_camera/image_raw']),
            ('grid_map/odom', [ns, '/odometry']),
            ('grid_map/pose', [ns, '/pose_gt']),
            ('position_cmd', '/position_cmd'),
        ],
        parameters=[{
            'use_sim_time': True,
            'fsm/flight_type': 2,            # 2 = 手动/外部触发 (由 bridge 发目标)
            'fsm/thresh_replan_time': 1.0,
            'fsm/thresh_no_replan_meter': 1.0,
            'fsm/planning_horizon': 7.5,
            'fsm/planning_horizen_time': 3.0,
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': False,
            'fsm/fail_safe': True,
            'fsm/waypoint_num': 1,
            'fsm/waypoint0_x': goal_x,
            'fsm/waypoint0_y': goal_y,
            'fsm/waypoint0_z': goal_z,
            'fsm/waypoint1_x': goal_x,
            'fsm/waypoint1_y': goal_y,
            'fsm/waypoint1_z': goal_z,
            'grid_map/resolution': 0.1,
            'grid_map/map_size_x': 20.0,
            'grid_map/map_size_y': 20.0,
            'grid_map/map_size_z': 5.0,
            'grid_map/local_update_range_x': 5.5,
            'grid_map/local_update_range_y': 5.5,
            'grid_map/local_update_range_z': 4.5,
            'grid_map/obstacles_inflation': 0.12,
            'grid_map/local_map_margin': 10,
            'grid_map/ground_height': -0.5,
            'grid_map/cx': float(cx),
            'grid_map/cy': float(cy),
            'grid_map/fx': float(fx),
            'grid_map/fy': float(fy),
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.15,
            'grid_map/depth_filter_maxdist': 5.0,
            'grid_map/depth_filter_mindist': 0.2,
            'grid_map/depth_filter_margin': 2,
            'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 2,
            'grid_map/p_hit': 0.65,
            'grid_map/p_miss': 0.35,
            'grid_map/p_min': 0.12,
            'grid_map/p_max': 0.90,
            'grid_map/p_occ': 0.80,
            'grid_map/min_ray_length': 0.1,
            'grid_map/max_ray_length': 4.5,
            'grid_map/virtual_ceil_height': 2.9,
            'grid_map/visualization_truncate_height': 1.8,
            'grid_map/show_occ_time': False,
            'grid_map/pose_type': 1,
            'grid_map/frame_id': 'world',
            'manager/drone_id': 0,
            'manager/max_vel': 1.0,
            'manager/max_acc': 3.0,
            'manager/control_points_distance': 0.4,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': 7.5,
            'manager/use_distinctive_trajs': True,
            'optimization/lambda_smooth': 1.0,
            'optimization/lambda_collision': 0.5,
            'optimization/lambda_feasibility': 0.1,
            'optimization/lambda_fitness': 1.0,
            'optimization/dist0': 0.5,
            'optimization/max_vel': 1.0,
            'optimization/max_acc': 3.0,
            'bspline/limit_vel': 1.0,
            'bspline/limit_acc': 3.0,
            'bspline/limit_ratio': 1.1,
        }],
    )

    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='traj_server',
        output='screen',
        remappings=[('planning/bspline', 'planning/bspline')],
        parameters=[{
            'use_sim_time': True,
            'traj_server/time_forward': 1.0,
        }],
    )

    bridge = Node(
        package='drone_sim',
        executable='ego_bridge',
        name='ego_bridge',
        output='screen',
        parameters=[{
            'px4_ns': ns,
            'target_system': 1,
            'position_cmd_topic': '/position_cmd',
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='/x500_depth_0'),
        DeclareLaunchArgument('goal_x', default_value='6.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_z', default_value='2.0'),
        ego, traj_server, bridge,
    ])
