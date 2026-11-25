#!/usr/bin/env python3
"""
drone_sim offboard 演示节点
=========================
链路: 本节点 --ROS2--> MicroXRCEAgent(UDP 8888) <--uXRCE-DDS--> PX4 SITL <--> Gazebo

状态机: INIT(预流setpoint) -> 请求Offboard+解锁 -> TAKEOFF(起飞)
        -> CIRCLE(画圆) -> LAND(降落) -> DONE

坐标系: PX4 本地坐标为 NED (北-东-地), 高度向上为 z 负方向。
参考: https://github.com/Jaeyoung-Lim/px4-offboard (BSD-3)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardDemo(Node):
    def __init__(self):
        super().__init__('drone_sim_offboard_demo')

        # ---- 可调参数 ----
        self.declare_parameter('takeoff_altitude', 5.0)   # 起飞高度 (m)
        self.declare_parameter('circle_radius', 3.0)      # 画圆半径 (m)
        self.declare_parameter('circle_speed', 0.3)       # 画圆角速度 (rad/s)
        self.declare_parameter('circle_duration', 25.0)   # 画圆时长 (s)
        self.alt = self.get_parameter('takeoff_altitude').value
        self.radius = self.get_parameter('circle_radius').value
        self.omega = self.get_parameter('circle_speed').value
        self.circle_time = self.get_parameter('circle_duration').value

        # PX4 uXRCE-DDS 话题使用 best-effort QoS, 订阅端必须匹配
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 发布端用 BEST_EFFORT: PX4 uXRCE 话题本身是 best-effort 语义;
        # 默认 RELIABLE 在 FastDDS SHM 缓冲满时可能阻塞死锁 (实测踩坑)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.cb_status, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.cb_local_pos, qos)

        self.status = None
        self.local_pos = None
        self.state = 'INIT'
        self.tick = 0
        self.circle_start = None
        self.last_cmd_tick = -100
        self.target = [0.0, 0.0, 0.0]   # 当前目标位置 (NED)
        self.target_yaw = 0.0

        # 10 Hz 控制循环 (offboard 要求 setpoint 流 > 2 Hz)
        self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f'drone_sim 演示启动: 起飞 {self.alt} m -> 半径 {self.radius} m 画圆 '
            f'{self.circle_time} s -> 降落')

    # ---- 订阅回调 ----
    def cb_status(self, msg):
        self.status = msg

    def cb_local_pos(self, msg):
        self.local_pos = msg

    # ---- 发布工具 ----
    def publish_mode(self):
        m = OffboardControlMode()
        m.position = True          # 位置控制模式
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.thrust_and_torque = False   # px4_msgs 1.15 字段 (1.16+ 改名为 actuator)
        m.direct_actuator = False
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(m)

    def publish_setpoint(self):
        sp = TrajectorySetpoint()
        sp.position = self.target
        sp.yaw = self.target_yaw
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_sp.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0, p3=0.0, p7=0.0):
        c = VehicleCommand()
        c.command = command
        c.param1, c.param2, c.param3, c.param7 = p1, p2, p3, p7
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(c)

    def request_offboard_and_arm(self):
        # 每 1 s 重试一次, 直到进入 offboard 且解锁
        if self.tick - self.last_cmd_tick < 10:
            return
        self.last_cmd_tick = self.tick
        # DO_SET_MODE: param1=1 自定义模式, param2=6 = OFFBOARD
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('请求 Offboard 模式 + 解锁 ...')

    # ---- 主循环 ----
    def loop(self):
        self.tick += 1
        # offboard 心跳与 setpoint 必须持续发布
        self.publish_mode()
        self.publish_setpoint()

        if self.status is None:
            if self.tick % 50 == 0:
                self.get_logger().warn('等待 PX4 状态... (MicroXRCEAgent / SITL 是否已启动?)')
            return

        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.state == 'INIT':
            # 先预流 2 s setpoint, PX4 才允许切入 offboard
            if self.tick > 20:
                self.request_offboard_and_arm()
            if armed and offboard:
                self.state = 'TAKEOFF'
                self.target = [0.0, 0.0, -self.alt]
                self.get_logger().info(f'已解锁并进入 Offboard, 起飞至 {self.alt} m')

        elif self.state == 'TAKEOFF':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            if self.local_pos is not None and self.local_pos.z_valid:
                if abs(self.local_pos.z + self.alt) < 0.3:
                    self.state = 'CIRCLE'
                    self.circle_start = self.tick
                    self.get_logger().info('到达目标高度, 开始画圆')

        elif self.state == 'CIRCLE':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            t = (self.tick - self.circle_start) * 0.1
            theta = self.omega * t
            self.target = [
                self.radius * math.cos(theta),
                self.radius * math.sin(theta),
                -self.alt,
            ]
            self.target_yaw = theta + math.pi / 2  # 机头沿切线方向
            if t >= self.circle_time:
                self.state = 'LAND'
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().info('画圆完成, 开始降落')

        elif self.state == 'LAND':
            # 以上锁判定降落完成 (不同机型出生点高度偏移不同, 地面 z 不一定≈0,
            # 例如 gz_x500 出生点抬高后地面 z≈-0.85)
            if not armed:
                self.state = 'DONE'
                self.get_logger().info('已降落并上锁, 演示完成 ✔')

        elif self.state == 'DONE':
            pass  # 保持节点运行, 可继续观察话题


def main(args=None):
    rclpy.init(args=args)
    node = OffboardDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
