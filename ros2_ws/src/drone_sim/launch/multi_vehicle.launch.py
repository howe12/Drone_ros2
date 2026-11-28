"""E3 多机 launch: N 架 x500 (每机 gz模型+PX4实例+通信节点+桥接), 单 Agent 单世界
用法:
  ros2 launch drone_sim multi_vehicle.launch.py num:=2 world:=aruco
之后分别启动领导机任务与跟随机编队:
  ros2 launch drone_sim waypoint_mission.launch.py prefix:=/x500_0 mission:=square
  ros2 run drone_sim formation_follower --ros-args -p leader_ns:=/x500_0 -p follower_ns:=/x500_1
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    num_arg = DeclareLaunchArgument('num', default_value='2')
    world_arg = DeclareLaunchArgument('world', default_value='aruco')
    model_arg = DeclareLaunchArgument('model', default_value='gz_x500')
    # px4_dir 默认指向**本仓库内的 PX4**（仓库根/PX4-Autopilot），而不是要求开发者
    # 把 PX4 放在家目录。也可用环境变量 PX4_DIR 覆盖。
    _repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              '..', '..', '..', '..'))
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir', default_value=os.environ.get(
            'PX4_DIR', os.path.join(_repo_root, 'PX4-Autopilot')))
    # E9: 自定义初始位置 "x,y;x,y;..." (空则用默认 i*2 一字排开)
    spawn_arg = DeclareLaunchArgument('spawn', default_value='')

    xtd2_launch_share = FindPackageShare('xtd2_launch')

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            xtd2_launch_share, 'launch', 'gz_launch.py'])),
        launch_arguments={'world': LaunchConfiguration('world')}.items())

    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent udp4 -p 8888'], output='screen',
        name='microxrceagent', shell=True)

    ld = LaunchDescription([num_arg, world_arg, model_arg, px4_dir_arg, spawn_arg,
                            world_launch, agent])

    # num 是 launch 参数, 无法在 Python 层直接 range —— 用 OpaqueFunction
    from launch.actions import OpaqueFunction

    def spawn_vehicles(context):
        n = int(context.launch_configurations['num'])
        model = context.launch_configurations['model']
        world = context.launch_configurations['world']
        px4_dir = context.launch_configurations['px4_dir']
        spawn = context.launch_configurations.get('spawn', '')
        base = model[3:] if model.startswith('gz_') else model
        pts = []
        if spawn:
            for item in spawn.split(';'):
                if ',' in item:
                    x, y = item.split(',')[:2]
                    pts.append((float(x), float(y)))
        actions = []
        for i in range(n):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('xtd2_launch'), 'launch',
                    'xtd2_vehicle_spawn_launch.py'])),
                launch_arguments={
                    'world_name': world,
                    'model': model,
                    'id': str(i),
                    'namespace': f'{base}_{i}',
                    'px4_dir': px4_dir,
                    'x': str(pts[i][0] if i < len(pts) else i * 2.0),
                    'y': str(pts[i][1] if i < len(pts) else 0.0), 'z': '0',
                    'roll': '0', 'pitch': '0', 'yaw': '0',
                }.items()))
        return actions

    ld.add_action(OpaqueFunction(function=spawn_vehicles))
    return ld
