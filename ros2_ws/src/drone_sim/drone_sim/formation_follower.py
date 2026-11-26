#!/usr/bin/env python3
"""
E3 编队跟随节点 (Issue #3)
==========================
跟随机保持与领导机的固定偏移 (leader-follower 编队)。
领导机由 waypoint_mission 驱动; 本节点独立完成跟随机的
解锁/offboard/起飞/跟随/降落全流程, 并周期性报告编队误差。

用法:
  ros2 run drone_sim formation_follower --ros-args \
      -p leader_ns:=/x500_0 -p follower_ns:=/x500_1 \
      -p offset_x:=-2.0 -p offset_y:=2.0
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)


class FormationFollower(Node):
    def __init__(self):
        super().__init__('drone_sim_formation_follower')
        self.declare_parameter('leader_ns', '/x500_0')
        self.declare_parameter('follower_ns', '/x500_1')
        self.declare_parameter('offset_x', -2.0)
        self.declare_parameter('offset_y', 2.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('cruise_speed', 2.5)   # 需快于领导机以收敛队形
        self.declare_parameter('spawn_x', 2.0)        # 跟随机出生点 (世界系, 仅日志用)
        self.declare_parameter('target_system', 2)    # 跟随机 MAV_SYS_ID = 实例号+1 (关键!)
        self.target_system = self.get_parameter('target_system').value

        lns = self.get_parameter('leader_ns').value
        fns = self.get_parameter('follower_ns').value
        self.offset = [self.get_parameter('offset_x').value,
                       self.get_parameter('offset_y').value,
                       self.get_parameter('offset_z').value]
        self.alt = self.get_parameter('takeoff_altitude').value
        self.speed = self.get_parameter('cruise_speed').value
        self.spawn_x = self.get_parameter('spawn_x').value

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

        self.pub_mode = self.create_publisher(
            OffboardControlMode, f'{fns}/fmu/in/offboard_control_mode', pub_qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, f'{fns}/fmu/in/trajectory_setpoint', pub_qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f'{fns}/fmu/in/vehicle_command', pub_qos)

        self.leader_pos = None
        self.leader_status = None
        self.pos = None
        self.status = None
        self.create_subscription(VehicleLocalPosition, f'{lns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'leader_pos', m), sub_qos)
        self.create_subscription(VehicleStatus, f'{lns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'leader_status', m), sub_qos)
        self.create_subscription(VehicleLocalPosition, f'{fns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'pos', m), sub_qos)
        self.create_subscription(VehicleStatus, f'{fns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), sub_qos)

        self.state = 'INIT'
        self.tick = 0
        self.last_cmd_tick = -100
        self.target = [0.0, 0.0, 0.0]   # 本地坐标原点在出生点
        self.target_yaw = 0.0
        self.create_timer(0.1, self.loop)
        self.get_logger().info(
            f'编队跟随启动: {lns} -> {fns}, 偏移 {self.offset}, 巡航 {self.speed}m/s')

    def publish_mode(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(m)

    def publish_setpoint(self):
        sp = TrajectorySetpoint()
        sp.position = self.target
        sp.yaw = self.target_yaw
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_sp.publish(sp)

    def send_command(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.command = command
        c.param1, c.param2 = p1, p2
        c.target_system = self.target_system   # 必须匹配该机 MAV_SYS_ID, 否则被拒收
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(c)

    def loop(self):
        self.tick += 1
        self.publish_mode()
        self.publish_setpoint()

        if self.status is None:
            if self.tick % 50 == 0:
                self.get_logger().warn('等待跟随机 PX4 状态 ...')
            return
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.state == 'INIT':
            if self.tick > 20:
                if self.tick - self.last_cmd_tick >= 10:
                    self.last_cmd_tick = self.tick
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.get_logger().info('跟随机请求 Offboard + 解锁 ...')
            if armed and offboard:
                self.state = 'TAKEOFF'
                # 本地坐标原点在出生点, 起飞目标为原点正上方 (不是世界系 spawn_x)
                self.target = [0.0, 0.0, -self.alt]
                self.get_logger().info(f'跟随机起飞至 {self.alt}m')

        elif self.state == 'TAKEOFF':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            if self.pos and self.pos.z_valid and abs(self.pos.z + self.alt) < 0.3:
                self.state = 'FOLLOW'
                self.get_logger().info('跟随机到达高度, 开始编队跟随')

        elif self.state == 'FOLLOW':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            if self.leader_pos is None or not self.leader_pos.z_valid:
                return
            lp = self.leader_pos
            goal = [lp.x + self.offset[0], lp.y + self.offset[1], lp.z + self.offset[2]]
            # setpoint 开环推进 (E2 教训: 不重锚定到机位置)
            d = [goal[i] - self.target[i] for i in range(3)]
            dist_sp = math.sqrt(sum(v * v for v in d))
            if dist_sp > 0.02:
                step = min(self.speed * 0.1, dist_sp)
                self.target = [self.target[i] + d[i] / dist_sp * step for i in range(3)]
            self.target_yaw = math.atan2(lp.y - self.pos.y, lp.x - self.pos.x) \
                if self.pos else 0.0
            # 编队误差报告 (2s 一次)
            if self.tick % 20 == 0 and self.pos and self.pos.z_valid:
                err = math.sqrt((self.pos.x - goal[0]) ** 2 +
                                (self.pos.y - goal[1]) ** 2 +
                                (self.pos.z - goal[2]) ** 2)
                self.get_logger().info(
                    f'编队误差 {err:.2f}m | 领导({lp.x:.1f},{lp.y:.1f},{lp.z:.1f}) '
                    f'跟随({self.pos.x:.1f},{self.pos.y:.1f},{self.pos.z:.1f})')
            # 领导机降落完成 (上锁) → 跟随机降落
            if self.leader_status is not None and \
               self.leader_status.arming_state != VehicleStatus.ARMING_STATE_ARMED and \
               lp.z > -0.5:
                self.state = 'LAND'
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.get_logger().info('领导机已降落, 跟随机开始降落')

        elif self.state == 'LAND':
            if not armed:
                self.state = 'DONE'
                self.get_logger().info('跟随机已降落上锁, 编队任务完成 ✔')

        elif self.state == 'DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FormationFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
