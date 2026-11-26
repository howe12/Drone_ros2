#!/usr/bin/env python3
"""
E7 ego-planner → PX4 桥接节点
订阅 ego_planner traj_server 的 /position_cmd (world/ENU, z 向上),
转为 PX4 TrajectorySetpoint (NED, z 向下) 并管理 Offboard 心跳/解锁。

用法:
  ros2 run drone_sim ego_bridge --ros-args -p px4_ns:=/x500_depth_0 -p target_system:=1
"""
import math


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleStatus, VehicleLocalPosition)
from quadrotor_msgs.msg import PositionCommand


class EgoBridge(Node):
    def __init__(self):
        super().__init__('drone_sim_ego_bridge')
        self.declare_parameter('px4_ns', '/x500_depth_0')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('position_cmd_topic', '/position_cmd')
        self.declare_parameter('cmd_timeout', 2.0)
        ns = self.get_parameter('px4_ns').value
        self.target_system = self.get_parameter('target_system').value
        cmd_topic = self.get_parameter('position_cmd_topic').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        # ego-planner 输出用默认 RELIABLE, 这里必须匹配
        cmd_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=50)

        self.pub_mode = self.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', pub_qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', pub_qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), sub_qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'pos', m), sub_qos)
        self.create_subscription(PositionCommand, cmd_topic, self.cb_cmd, cmd_qos)
        # ego-planner PRESET_TARGET 模式需要 /traj_start_trigger 启动
        self.pub_trigger = self.create_publisher(PoseStamped, '/traj_start_trigger', 10)

        self.status = None
        self.pos = None
        self.cmd = None
        self.cmd_stamp = 0.0
        self.armed_once = False
        self.hold = [0.0, 0.0, -2.0]
        self.create_timer(1.0 / 30.0, self.loop)
        self.get_logger().info(f'ego 桥接启动: {cmd_topic} -> {ns}/fmu/in/trajectory_setpoint')

    def cb_cmd(self, msg):
        self.cmd = msg
        self.cmd_stamp = self.get_clock().now().nanoseconds / 1e9

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def send_command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.now_s() * 1e6)
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = self.target_system
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def loop(self):
        # 心跳: offboard 控制模式
        mode = OffboardControlMode()
        mode.timestamp = int(self.now_s() * 1e6)
        mode.position = True
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.now_s() * 1e6)
        yaw = 0.0
        if self.cmd is not None and (self.now_s() - self.cmd_stamp) < self.cmd_timeout:
            # ENU(z上) → NED(z下)
            # ego 世界系 gz ENU (x=东,y=北,z=上) → PX4 local NED (x=北,y=东,z=下)
            # 关键: 这是坐标轴交换(ENU->NED), 不是航向旋转
            sp.position = [self.cmd.position.y, self.cmd.position.x, -self.cmd.position.z]
            vx = max(-2.0, min(2.0, self.cmd.velocity.y))
            vy = max(-2.0, min(2.0, self.cmd.velocity.x))
            vz = max(-2.0, min(2.0, -self.cmd.velocity.z))
            sp.velocity = [vx, vy, vz]
            yaw = self.pos.heading if (self.pos is not None and self.pos.z_valid) else 0.0
            self.hold = list(sp.position)
        else:
            sp.position = self.hold
            sp.velocity = [0.0, 0.0, 0.0]
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

        # 触发 ego-planner 开始规划 (PRESET_TARGET: 1Hz 重发, 直到收到轨迹)
        if self.cmd is None:
            trig = PoseStamped()
            trig.header.stamp = self.get_clock().now().to_msg()
            trig.header.frame_id = 'world'
            self.pub_trigger.publish(trig)

        # 收到轨迹后: 请求 offboard + 解锁
        if self.cmd is not None and not self.armed_once:
            self.armed_once = True
            self.get_logger().info('收到 ego 轨迹, 请求 Offboard + 解锁')
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        if self.cmd is not None and self.armed_once and self.status is not None:
            if self.status.arming_state != 2:
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = EgoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
