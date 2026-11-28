#!/usr/bin/env python3
"""
E11 VTOL 过渡段控制 demo

流程: 解锁+Offboard(多旋翼位置控制) -> 垂直爬升到 cruise_alt
      -> 发 DO_VTOL_TRANSITION(FW=4) 进入固定翼前飞
      -> 观察固定翼段 (空速/高度) cruise_time 秒
      -> 发 DO_VTOL_TRANSITION(MC=3) 转回多旋翼
      -> NAV_LAND 垂直降落

用法 (栈: xtd2 gz_standard_vtol walls):
  ros2 run drone_sim vtol_demo --ros-args -p px4_ns:=/standard_vtol_0
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus, VehicleLocalPosition, VehicleAirData)


from drone_sim.paths import log_path


class VtolDemo(Node):
    def __init__(self):
        super().__init__('drone_sim_vtol_demo')
        self.declare_parameter('px4_ns', '/standard_vtol_0')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('cruise_alt', 20.0)
        self.declare_parameter('fw_leg_time', 18.0)
        self.declare_parameter('log_file', log_path('e11_vtol.log'))
        ns = self.get_parameter('px4_ns').value
        self.target_system = self.get_parameter('target_system').value
        self.alt = self.get_parameter('cruise_alt').value
        self.fw_time = self.get_parameter('fw_leg_time').value
        self.log_path = self.get_parameter('log_file').value

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_mode = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', pub_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', pub_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), sub_qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'pos', m), sub_qos)
        self.create_subscription(VehicleAirData, f'{ns}/fmu/out/vehicle_air_data',
                                 lambda m: setattr(self, 'air', m), sub_qos)

        self.status = None
        self.pos = None
        self.air = None
        self.state = 'INIT'
        self.started = time.time()
        self.state_t0 = time.time()
        self.target = [0.0, 0.0, -self.alt]
        self.logf = open(self.log_path, 'w', buffering=1)
        self.create_timer(0.1, self.loop)
        self.get_logger().info(f'VTOL demo: {ns}, 巡航高度 {self.alt}m')

    def dlog(self, s):
        self.logf.write(f'[{time.time()-self.started:6.1f}s] {s}\n')
        self.logf.flush()

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = self.target_system
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def alt_now(self):
        return -self.pos.z if (self.pos and self.pos.z_valid) else 0.0

    def loop(self):
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = list(self.target)
        sp.yaw = 0.0
        self.pub_sp.publish(sp)

        if self.state == 'INIT':
            if self.pos is not None and self.pos.z_valid:
                self.state = 'TAKEOFF'
                self.state_t0 = time.time()
                self.dlog('状态: TAKEOFF (多旋翼垂直爬升)')
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            return

        if self.state == 'TAKEOFF':
            self.target = [0.0, 0.0, -self.alt]
            if not (self.status and self.status.arming_state == 2):
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            if self.alt_now() > self.alt - 2.0:
                self.state = 'TRANSITION_FW'
                self.state_t0 = time.time()
                self.dlog(f'到达 {self.alt_now():.1f}m, 切 POSCTL 并发 VTOL 过渡 -> 固定翼')
                # 先切 POSCTL (PX4 才允许 VTOL 过渡到固定翼前飞)
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 22.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 4.0)  # 4=FW
            elif time.time() - self.state_t0 > 60:
                self.dlog('起飞超时, 直接降落')
                self.state = 'LAND'
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return

        if self.state == 'TRANSITION_FW':
            # 过渡期间保持位置设定点 (MC 段), 等待 PX4 完成转换
            if time.time() - self.state_t0 > 1.5:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 4.0)
            if time.time() - self.state_t0 > 4.0:
                self.state = 'FW_CRUISE'
                self.state_t0 = time.time()
                self.dlog('状态: FW_CRUISE (固定翼前飞)')
            return

        if self.state == 'FW_CRUISE':
            # 固定翼段: 记录空速/高度/姿态
            if int(time.time() - self.state_t0) % 2 == 0:
                pass
            if time.time() - self.state_t0 > self.fw_time:
                self.state = 'TRANSITION_MC'
                self.state_t0 = time.time()
                self.dlog(f'固定翼段结束 (空速 {self.airspeed():.1f}m/s), 转回多旋翼')
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 3.0)  # 3=MC
            return

        if self.state == 'TRANSITION_MC':
            if time.time() - self.state_t0 > 3.0:
                self.state = 'LAND'
                self.state_t0 = time.time()
                self.dlog('状态: LAND (垂直降落)')
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return

        if self.state == 'LAND':
            if self.status and self.status.arming_state != 2:
                self.dlog(f'已降落上锁, 最终高度 {self.alt_now():.2f}m')
                self.state = 'DONE'
            return

    def airspeed(self):
        # vehicle_air_data 无空速, 用水平速度近似
        if self.pos is None:
            return 0.0
        return math.sqrt(self.pos.vx ** 2 + self.pos.vy ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = VtolDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
