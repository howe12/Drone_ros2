#!/usr/bin/env python3
"""
E2 航点任务节点 (Issue #2)
==========================
YAML 航点文件 / 内置轨迹模式 (square/circle/spiral/figure8) → 平滑 setpoint 流
→ 到点判定 → 自动降落。RViz 可视化: nav_msgs/Path 实飞轨迹 + MarkerArray 航点。

用法:
  ros2 run drone_sim waypoint_mission --ros-args \
      -p mission_file:=/path/mission.yaml -p topic_prefix:=/x500_depth_0
  # topic_prefix 为空 = classic 平台; /x500_0 等 = xtd2 平台命名空间

YAML 格式 (二选一):
  takeoff_altitude: 5.0
  cruise_speed: 1.5
  waypoints: [{x: 3, y: 0, z: -5, yaw: 0.0, hold: 2.0}, ...]
  --- 或 ---
  pattern: circle        # square | circle | spiral | figure8
  pattern_args: {radius: 3.0, points: 24}
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)
from drone_sim.px4_interface import PX4Interface


def generate_pattern(name, args, alt):
    """内置轨迹模式生成航点列表 (NED, z 向上为负)"""
    pts = []
    n = int(args.get('points', 24))
    if name == 'square':
        s = float(args.get('size', 3.0))
        corners = [(s, 0), (s, s), (0, s), (0, 0)]
        for x, y in corners:
            pts.append({'x': x, 'y': y, 'z': -alt, 'hold': 1.0})
    elif name == 'circle':
        r = float(args.get('radius', 3.0))
        for i in range(n):
            th = 2 * math.pi * i / n
            pts.append({'x': r * math.cos(th), 'y': r * math.sin(th),
                        'z': -alt, 'yaw': th + math.pi / 2, 'hold': 0.0})
    elif name == 'spiral':
        r = float(args.get('radius', 3.0))
        turns = float(args.get('turns', 2.0))
        dz = float(args.get('dz', 2.0))
        for i in range(n):
            f = i / max(n - 1, 1)
            th = 2 * math.pi * turns * f
            pts.append({'x': r * f * math.cos(th), 'y': r * f * math.sin(th),
                        'z': -(alt + dz * f), 'yaw': th + math.pi / 2, 'hold': 0.0})
    elif name == 'figure8':
        s = float(args.get('size', 3.0))
        for i in range(n):
            th = 2 * math.pi * i / n
            pts.append({'x': s * math.sin(th), 'y': s * math.sin(th) * math.cos(th),
                        'z': -alt, 'hold': 0.0})
    else:
        raise ValueError(f'未知轨迹模式: {name}')
    return pts


class WaypointMission(Node):
    def __init__(self):
        super().__init__('drone_sim_waypoint_mission')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('topic_prefix', '')          # ''=classic, '/x500_0'=xtd2
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('cruise_speed', 1.5)         # m/s
        self.declare_parameter('arrive_tol', 0.35)          # 到点容差 m
        self.declare_parameter('pattern', 'square')
        self.declare_parameter('target_system', 1)   # 多机时 = 实例号+1         # 无 mission_file 时的默认模式

        prefix = self.get_parameter('topic_prefix').value
        self.alt = self.get_parameter('takeoff_altitude').value
        self.speed = self.get_parameter('cruise_speed').value
        self.tol = self.get_parameter('arrive_tol').value

        # ---- 加载任务 ----
        mf = self.get_parameter('mission_file').value
        if mf:
            with open(mf) as f:
                cfg = yaml.safe_load(f)
            self.alt = float(cfg.get('takeoff_altitude', self.alt))
            self.speed = float(cfg.get('cruise_speed', self.speed))
            if 'waypoints' in cfg:
                self.wps = cfg['waypoints']
            else:
                self.wps = generate_pattern(cfg.get('pattern', 'square'),
                                            cfg.get('pattern_args', {}), self.alt)
        else:
            self.wps = generate_pattern(self.get_parameter('pattern').value,
                                        {'size': 3.0, 'radius': 3.0}, self.alt)
        for wp in self.wps:
            wp.setdefault('z', -self.alt)
            wp.setdefault('hold', 0.0)
        self.get_logger().info(f'任务加载: {len(self.wps)} 个航点, 高度 {self.alt}m, '
                               f'巡航 {self.speed}m/s, 前缀 "{prefix}"')

        # ---- PX4 统一接口 (QoS/心跳/解锁/EKF 门控全在 px4_interface 内) ----
        self.px4 = PX4Interface(
            self, ns=prefix,
            target_system=self.get_parameter('target_system').value,
            on_position=self.cb_pos)

        # RViz 可视化 (reliable 默认 QoS)
        self.pub_path = self.create_publisher(Path, f'{prefix}/mission/path', 10)
        self.pub_markers = self.create_publisher(MarkerArray, f'{prefix}/mission/waypoints', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.publish_waypoint_markers()

        self.state = 'INIT'
        self.tick = 0
        self.wp_idx = 0
        self.hold_until = None
        self.last_cmd_tick = -100
        self.target = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        # EKF 原点稳定性门控: 重置计数器变化时禁止解锁 (防止 z 原点跳变导致"贴地爬行")
        self.last_reset = None
        self.reset_stable_ticks = 0
        self.create_timer(0.1, self.loop)

    # ---- 状态访问 (委托给接口) ----
    @property
    def status(self):
        return self.px4.status

    @property
    def pos(self):
        return self.px4.pos

    # ---- 回调 ----
    def cb_pos(self, msg):
        # 位置由 PX4Interface 保存, 这里只做轨迹记录
        if self.tick % 5 == 0 and msg.z_valid:   # 2Hz 记录实飞轨迹
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = msg.x
            p.pose.position.y = msg.y
            p.pose.position.z = msg.z
            self.path_msg.poses.append(p)
            self.path_msg.header.stamp = p.header.stamp
            self.pub_path.publish(self.path_msg)

    # ---- 可视化 ----
    def publish_waypoint_markers(self):
        arr = MarkerArray()
        for i, wp in enumerate(self.wps):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wp['x'])
            m.pose.position.y = float(wp['y'])
            m.pose.position.z = float(wp['z'])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 0.9
            arr.markers.append(m)
            t = Marker()
            t.header.frame_id = 'map'
            t.ns = 'wp_labels'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(wp['x'])
            t.pose.position.y = float(wp['y'])
            t.pose.position.z = float(wp['z']) + 0.4
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'WP{i}'
            arr.markers.append(t)
        self.pub_markers.publish(arr)

    # ---- 发布工具 ----
    def publish_mode(self):
        pass   # 心跳由 publish_setpoint -> px4.heartbeat_position 统一发布

    def publish_setpoint(self):
        self.px4.heartbeat_position(*self.target, yaw=self.target_yaw)

    def send_command(self, command, p1=0.0, p2=0.0, p7=0.0):
        self.px4.send_command(command, p1, p2, p7=p7)

    def request_offboard_and_arm(self):
        if self.tick - self.last_cmd_tick < 10:
            return
        self.last_cmd_tick = self.tick
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('请求 Offboard + 解锁 ...')

    # ---- 主循环 ----
    def loop(self):
        self.tick += 1
        self.publish_mode()
        self.publish_setpoint()

        if self.status is None:
            if self.tick % 50 == 0:
                self.get_logger().warn('等待 PX4 状态 ...')
            return
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.state == 'INIT':
            # EKF 原点稳定门控 (由 PX4Interface 统一跟踪)
            if self.tick > 20 and self.px4.ekf_stable:
                self.request_offboard_and_arm()
            elif self.tick % 50 == 0:
                self.get_logger().info('等待 EKF 原点稳定 ...')
            if armed and offboard:
                self.state = 'TAKEOFF'
                self.target = [0.0, 0.0, -self.alt]
                self.get_logger().info(f'已解锁, 起飞至 {self.alt}m')

        elif self.state == 'TAKEOFF':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            if self.pos and self.pos.z_valid and abs(self.pos.z + self.alt) < 0.3:
                self.state = 'MISSION'
                self.wp_idx = 0
                self.get_logger().info('到达起飞高度, 开始航点任务')

        elif self.state == 'MISSION':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            wp = self.wps[self.wp_idx]
            goal = [float(wp['x']), float(wp['y']), float(wp['z'])]
            # setpoint 自身沿路径以巡航速度推进 (开环 carrot, 不重锚定到机位置,
            # 否则 setpoint 永远只领先一点点, 飞机会以极低速度蠕动)
            d_sp = [goal[i] - self.target[i] for i in range(3)]
            dist_sp = math.sqrt(sum(v * v for v in d_sp))
            if dist_sp > 0.02:
                step = min(self.speed * 0.1, dist_sp)
                self.target = [self.target[i] + d_sp[i] / dist_sp * step for i in range(3)]
            if 'yaw' in wp:
                self.target_yaw = float(wp['yaw'])
            elif dist_sp > 0.1:
                self.target_yaw = math.atan2(d_sp[1], d_sp[0])
            # 到点判定用飞机实际位置
            if self.pos and self.pos.z_valid:
                cur = [self.pos.x, self.pos.y, self.pos.z]
                dist = math.sqrt(sum((goal[i] - cur[i]) ** 2 for i in range(3)))
                if dist <= self.tol:
                    if self.hold_until is None:
                        self.hold_until = self.tick + int(float(wp['hold']) * 10)
                        self.get_logger().info(
                            f'WP{self.wp_idx} 到达 ({goal[0]:.1f},{goal[1]:.1f},{goal[2]:.1f}) '
                            f'误差 {dist:.2f}m')
                    if self.tick >= self.hold_until:
                        self.hold_until = None
                        self.wp_idx += 1
                        if self.wp_idx >= len(self.wps):
                            self.state = 'LAND'
                            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                            self.get_logger().info('全部航点完成, 降落')

        elif self.state == 'LAND':
            if not armed:
                self.state = 'DONE'
                self.get_logger().info('已降落并上锁, 航点任务完成 ✔')

        elif self.state == 'DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
