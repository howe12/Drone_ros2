#!/usr/bin/env python3
"""
E9 分布式集群智能体 (多机避碰 + 交叉飞行)

每个智能体:
  - 从 gz 世界系位姿 (/<ns>/pose) 获取自己和邻居的真实位置
  - 势场法: 目标吸引 + 邻居斥力 (安全半径内)
  - 速度控制 (PX4 Offboard velocity), 世界系速度按航向转换到机体系
  - 到达目标后悬停

用法 (3 机交叉):
  ros2 run drone_sim swarm_agent --ros-args \
    -p px4_ns:=/x500_0 -p target_x:=3.0 -p target_y:=0.0 -p target_z:=2.5 \
    -p neighbors:=/x500_1,/x500_2
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus, VehicleLocalPosition)
from geometry_msgs.msg import PoseArray


from drone_sim.paths import log_path


class SwarmAgent(Node):
    def __init__(self):
        super().__init__('drone_sim_swarm_agent')
        self.declare_parameter('px4_ns', '/x500_0')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.5)
        self.declare_parameter('neighbors', '')
        self.declare_parameter('safety_dist', 2.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('goal_tol', 0.4)
        self.declare_parameter('timeout', 90.0)
        self.declare_parameter('log_file', log_path('e9_agent.log'))

        self.ns = self.get_parameter('px4_ns').value
        self.target_system = self.get_parameter('target_system').value
        self.goal = (self.get_parameter('target_x').value,
                     self.get_parameter('target_y').value,
                     self.get_parameter('target_z').value)
        nbrs = self.get_parameter('neighbors').value
        self.nbr_ns = [s for s in nbrs.split(',') if s]
        self.safe = self.get_parameter('safety_dist').value
        self.vmax = self.get_parameter('max_speed').value
        self.tol = self.get_parameter('goal_tol').value
        self.timeout = self.get_parameter('timeout').value
        self.log_path = self.get_parameter('log_file').value

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_mode = self.create_publisher(OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', pub_qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', pub_qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, f'{self.ns}/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(VehicleStatus, f'{self.ns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), sub_qos)
        self.create_subscription(VehicleLocalPosition, f'{self.ns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'pos', m), sub_qos)

        # 世界系位姿 (自己和邻居)
        self.world = None
        self.nbr_world = {}
        self.create_subscription(PoseArray, f'{self.ns}/pose', self.cb_own, qos_profile_sensor_data)
        for n in self.nbr_ns:
            self.create_subscription(PoseArray, f'{n}/pose',
                                     lambda m, name=n: self.cb_nbr(m, name), qos_profile_sensor_data)

        self.status = None
        self.pos = None
        self.arrived = False
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.min_sep = 99.0
        self.logf = open(self.log_path, 'a', buffering=1)
        self.create_timer(0.05, self.loop)   # 20Hz
        self.dlog(f'--- swarm_agent {self.ns}: goal={self.goal} 邻居={self.nbr_ns} 安全半径={self.safe}m')

    def dlog(self, s):
        self.logf.write(s + '\n')
        self.logf.flush()

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cb_own(self, msg):
        if msg.poses:
            p = msg.poses[0].position
            self.world = (p.x, p.y, p.z)

    def cb_nbr(self, msg, name):
        if msg.poses:
            p = msg.poses[0].position
            self.nbr_world[name] = (p.x, p.y, p.z)

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
        if self.world is None:
            return
        t = self.now_s()
        # 心跳 (速度控制)
        mode = OffboardControlMode()
        mode.timestamp = int(t * 1e6)
        mode.velocity = True
        self.pub_mode.publish(mode)

        wx, wy, wz = self.world
        gx, gy, gz = self.goal
        # --- 势场 ---
        dx, dy, dz = gx - wx, gy - wy, gz - wz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < self.tol:
            self.arrived = True
        vx = vy = vz = 0.0
        if not self.arrived:
            # 目标吸引 (限速)
            k_att = 0.9
            vx, vy, vz = k_att * dx, k_att * dy, k_att * dz
            # 邻居斥力
            for name, (nx, ny, nz) in self.nbr_world.items():
                ex, ey, ez = wx - nx, wy - ny, wz - nz
                d = math.sqrt(ex * ex + ey * ey + ez * ez)
                self.min_sep = min(self.min_sep, d)
                if 0.05 < d < self.safe:
                    k_rep = 1.2 * (1.0 / d - 1.0 / self.safe) * (self.safe ** 2)
                    vx += k_rep * ex / d
                    vy += k_rep * ey / d
                    vz += 0.8 * k_rep * ez / d
            # 限速
            sp = math.sqrt(vx * vx + vy * vy + vz * vz)
            if sp > self.vmax:
                s = self.vmax / sp
                vx, vy, vz = vx * s, vy * s, vz * s

        # --- 世界系速度 → 机体系 (PX4 local: x=北/世界y, y=东/世界x, 含航向旋转) ---
        h = self.pos.heading if (self.pos is not None and self.pos.z_valid) else 0.0
        v_lx = vx * math.cos(h) + vy * math.sin(h)
        v_ly = vx * math.sin(h) - vy * math.cos(h)
        v_lz = -vz

        sp = TrajectorySetpoint()
        sp.timestamp = int(t * 1e6)
        sp.position = [float('nan'), float('nan'), float('nan')]
        sp.velocity = [v_lx, v_ly, v_lz]
        sp.yaw = float(h)
        self.pub_sp.publish(sp)

        # 解锁 + Offboard
        if self.status is None or self.status.arming_state != 2:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

        if int(t * 2) % 2 == 0:   # 2Hz 日志
            self.dlog(f'{self.ns} pos=({wx:.2f},{wy:.2f},{wz:.2f}) goal_dist={dist:.2f} '
                      f'v=({vx:.2f},{vy:.2f},{vz:.2f}) min_sep={self.min_sep:.2f} '
                      f'arrived={self.arrived}')

        if self.arrived and (t - self.t0) > 3.0:
            self.dlog(f'{self.ns} DONE: 到达目标, 最小间距 {self.min_sep:.2f}m, 用时 {t-self.t0:.1f}s')


def main(args=None):
    rclpy.init(args=args)
    node = SwarmAgent()
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
