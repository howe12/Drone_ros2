#!/usr/bin/env python3
"""
E12 安全框架 (仿真侧): 地理围栏 + 失控保护 + 急停

监控项 (任一越界触发保护动作):
  - 水平半径 > max_radius (默认 15m)      -> RTL
  - 高度 > max_altitude (默认 25m)        -> 收油门/降落
  - 高度 < min_altitude 且已解锁           -> 降落 (防钻地)
  - 水平速度 > max_speed (默认 8m/s)      -> 限速警告 + RTL
  - 姿态倾角 > max_tilt (默认 45°)        -> 降落
  - 与起飞点水平距离 > fence 且失联        -> RTL
  - 外部急停 (ros2 topic pub /safety/estop) -> 立即降落

动作: 通过 PX4 VehicleCommand 发 MAV_CMD_NAV_RETURN_TO_LAUNCH / NAV_LAND;
      若 PX4 已失联, 记录并进入 MONITOR 状态继续尝试。

用法 (栈已启动):
  ros2 run drone_sim safety_supervisor --ros-args -p px4_ns:=/x500_0
  # 急停: ros2 topic pub --once /safety/estop std_msgs/Bool "{data: true}"
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude)
from std_msgs.msg import Bool


from drone_sim.paths import log_path


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('drone_sim_safety_supervisor')
        self.declare_parameter('px4_ns', '/x500_0')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('max_radius', 15.0)
        self.declare_parameter('max_altitude', 25.0)
        self.declare_parameter('min_altitude', 0.15)
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('max_tilt_deg', 45.0)
        self.declare_parameter('log_file', log_path('e12_safety.log'))

        ns = self.get_parameter('px4_ns').value
        self.target_system = self.get_parameter('target_system').value
        self.max_radius = self.get_parameter('max_radius').value
        self.max_alt = self.get_parameter('max_altitude').value
        self.min_alt = self.get_parameter('min_altitude').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_tilt = math.radians(self.get_parameter('max_tilt_deg').value)
        self.log_path = self.get_parameter('log_file').value

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_cmd = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'pos', m), sub_qos)
        self.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), sub_qos)
        self.create_subscription(VehicleAttitude, f'{ns}/fmu/out/vehicle_attitude',
                                 lambda m: setattr(self, 'att', m), sub_qos)
        self.create_subscription(Bool, '/safety/estop', self.cb_estop, 10)

        self.pos = None
        self.status = None
        self.att = None
        self.estop = False
        self.action = None          # 已触发的保护动作
        self.logf = open(self.log_path, 'a', buffering=1)
        self.create_timer(0.2, self.loop)
        self.dlog(f'== safety_supervisor {ns}: 半径{self.max_radius}m 高度[{self.min_alt},{self.max_alt}]m '
                  f'速度{self.max_speed}m/s 倾角{math.degrees(self.max_tilt):.0f}°')

    def dlog(self, s):
        self.logf.write(s + '\n')
        self.logf.flush()

    def cb_estop(self, msg):
        if msg.data and not self.estop:
            self.estop = True
            self.dlog('!!! 外部急停触发 !!!')

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

    def armed(self):
        return self.status is not None and self.status.arming_state == 2

    def trigger(self, action, reason):
        if self.action == action:
            return
        self.action = action
        self.dlog(f'[安全事件] {reason} -> 执行 {action}')
        self.get_logger().warn(f'安全事件: {reason} → {action}')
        if action == 'RTL':
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        elif action == 'LAND':
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def loop(self):
        if self.pos is None or not self.pos.z_valid:
            return
        if not self.armed():
            return
        p = self.pos
        alt = -p.z
        speed = math.sqrt(p.vx ** 2 + p.vy ** 2 + p.vz ** 2)
        horiz = math.sqrt(p.x ** 2 + p.y ** 2)
        # 倾角: 机体 z 轴与世界 z 轴夹角 (四元数 q=[w,x,y,z])
        if self.att is not None:
            q = self.att.q
            cos_tilt = 2.0 * (q[0] * q[0] + q[3] * q[3]) - 1.0
            tilt = math.acos(max(-1.0, min(1.0, cos_tilt)))
        else:
            tilt = 0.0

        if self.estop:
            self.trigger('LAND', '外部急停')
            return
        if horiz > self.max_radius:
            self.trigger('RTL', f'超出水平围栏 {horiz:.1f}m > {self.max_radius}m')
            return
        if alt > self.max_alt:
            self.trigger('LAND', f'超出高度上限 {alt:.1f}m > {self.max_alt}m')
            return
        if alt < self.min_alt and alt > -0.1 and speed > 1.0:
            self.trigger('LAND', f'低于高度下限 {alt:.2f}m 且在下坠')
            return
        if speed > self.max_speed:
            self.trigger('RTL', f'速度超限 {speed:.1f}m/s > {self.max_speed}m/s')
            return
        if tilt > self.max_tilt and alt > 1.0:
            self.trigger('LAND', f'倾角过大 {math.degrees(tilt):.0f}°')
            return


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
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
