#!/usr/bin/env python3
"""
drone_sim offboard 演示节点 (参考实现 —— 展示如何用 PX4Interface)
=================================================================
链路: 本节点 --ROS2--> MicroXRCEAgent(UDP 8888) <--uXRCE-DDS--> PX4 SITL <--> Gazebo

状态机: INIT(预流setpoint) -> 请求Offboard+解锁 -> TAKEOFF(起飞)
        -> CIRCLE(画圆) -> LAND(降落) -> DONE

所有 PX4 交互 (QoS/心跳/解锁/EKF 门控/坐标系) 统一走 px4_interface.PX4Interface,
本文件只保留"任务逻辑", 约 60 行 —— 这是写新节点的推荐范式。
"""

import math

import rclpy
from rclpy.node import Node

from drone_sim.px4_interface import PX4Interface


class OffboardDemo(Node):
    def __init__(self):
        super().__init__('drone_sim_offboard_demo')

        # ---- 可调参数 ----
        self.declare_parameter('takeoff_altitude', 5.0)   # 起飞高度 (m)
        self.declare_parameter('circle_radius', 3.0)      # 画圆半径 (m)
        self.declare_parameter('circle_speed', 0.3)       # 画圆角速度 (rad/s)
        self.declare_parameter('circle_duration', 25.0)   # 画圆时长 (s)
        self.declare_parameter('px4_ns', '')               # ''=相对话题(launch remap); '/x500_0'=绝对
        self.declare_parameter('target_system', 1)
        self.alt = self.get_parameter('takeoff_altitude').value
        self.radius = self.get_parameter('circle_radius').value
        self.omega = self.get_parameter('circle_speed').value
        self.circle_time = self.get_parameter('circle_duration').value

        # px4_ns='' 时用相对话题 /fmu/... (由 launch remap); 也可直接指定 '/x500_0'
        self.px4 = PX4Interface(
            self, ns=self.get_parameter('px4_ns').value,
            target_system=self.get_parameter('target_system').value)

        self.state = 'INIT'
        self.tick = 0
        self.circle_start = None
        self.target = (0.0, 0.0, 0.0)   # (x, y, z_down)
        self.target_yaw = 0.0

        self.create_timer(0.1, self.loop)   # 10 Hz (>2Hz 是 offboard 要求)
        self.get_logger().info(
            f'drone_sim 演示启动: 起飞 {self.alt} m -> 半径 {self.radius} m 画圆 '
            f'{self.circle_time} s -> 降落')

    def loop(self):
        self.tick += 1
        # 心跳 + 设定点 (必须持续)
        self.px4.heartbeat_position(*self.target, yaw=self.target_yaw)

        if self.px4.status is None:
            if self.tick % 50 == 0:
                self.get_logger().warn('等待 PX4 状态... (MicroXRCEAgent / SITL 是否已启动?)')
            return

        if self.state == 'INIT':
            if self.tick > 20:          # 预流 2s setpoint 后才请求
                self.px4.request_offboard_and_arm()
            if self.px4.armed and self.px4.offboard:
                self.state = 'TAKEOFF'
                self.target = (0.0, 0.0, -self.alt)
                self.get_logger().info(f'已解锁并进入 Offboard, 起飞至 {self.alt} m')

        elif self.state == 'TAKEOFF':
            if not (self.px4.armed and self.px4.offboard):
                self.state = 'INIT'
                return
            if abs(self.px4.alt() - self.alt) < 0.3:
                self.state = 'CIRCLE'
                self.circle_start = self.tick
                self.get_logger().info('到达目标高度, 开始画圆')

        elif self.state == 'CIRCLE':
            if not (self.px4.armed and self.px4.offboard):
                self.state = 'INIT'
                return
            t = (self.tick - self.circle_start) * 0.1
            theta = self.omega * t
            self.target = (self.radius * math.cos(theta),
                           self.radius * math.sin(theta),
                           -self.alt)
            self.target_yaw = theta + math.pi / 2   # 机头沿切线
            if t >= self.circle_time:
                self.state = 'LAND'
                self.px4.land()
                self.get_logger().info('画圆完成, 开始降落')

        elif self.state == 'LAND':
            # 以上锁判定降落 (不同机型地面 z 偏移不同, 不能以 z≈0 判断)
            if not self.px4.armed:
                self.state = 'DONE'
                self.get_logger().info('已降落并上锁, 演示完成 ✔')

        elif self.state == 'DONE':
            pass


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
