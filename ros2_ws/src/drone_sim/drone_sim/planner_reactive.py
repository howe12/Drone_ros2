#!/usr/bin/env python3
"""
E7 自研反应式避障规划器 (替代 ego-planner 的框架依赖)

思路:
  1) **启动自标定**: 依次给机体系 x/y 方向一个已知速度, 用 gz 世界位姿测量产生的世界位移,
     从而学习 机体系速度 -> 世界速度 的 2x2 映射 M (不依赖任何航向/坐标约定假设)
  2) **目标吸引**: 世界系目标误差 -> 世界速度 -> 经 M^-1 转为机体系速度
  3) **深度避障**: 深度图分块找最近障碍, 在**机体系**内产生横向推力 + 前向减速
     (避障量直接在机体系, 不需要坐标系转换)
  4) 输出 PX4 Offboard 速度设定点

用法 (栈: xtd2 gz_x500_depth e7_obstacles):
  ros2 run drone_sim planner_reactive --ros-args -p px4_ns:=/x500_depth_0 \
      -p target_x:=6.0 -p target_y:=0.0 -p target_z:=1.2
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus, VehicleLocalPosition)
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image


class PlannerReactive(Node):
    def __init__(self):
        super().__init__('drone_sim_planner_reactive')
        self.declare_parameter('px4_ns', '/x500_depth_0')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_x', 6.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 1.2)
        self.declare_parameter('cruise_z', 1.2)
        self.declare_parameter('max_speed', 1.2)
        self.declare_parameter('safe_dist', 1.8)
        self.declare_parameter('goal_tol', 0.5)
        self.declare_parameter('log_file', '/develop/IDEA/Drone_ros2/logs/e7_reactive.log')

        ns = self.get_parameter('px4_ns').value
        self.target_system = self.get_parameter('target_system').value
        self.goal = (self.get_parameter('target_x').value,
                     self.get_parameter('target_y').value,
                     self.get_parameter('target_z').value)
        self.cruise_z = self.get_parameter('cruise_z').value
        self.vmax = self.get_parameter('max_speed').value
        self.safe = self.get_parameter('safe_dist').value
        self.tol = self.get_parameter('goal_tol').value
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
        self.create_subscription(PoseArray, f'{ns}/pose', self.cb_world, qos_profile_sensor_data)
        self.create_subscription(Image, f'{ns}/depth_camera/image_raw', self.cb_depth,
                                 qos_profile_sensor_data)

        self.status = None
        self.pos = None
        self.world = None            # (x,y,z) gz 世界系
        self.world_hist = []         # 用于测速
        self.depth = None            # 分块最小深度 (前方)
        self.M = None                # 机体系速度 -> 世界速度 2x2
        self.state = 'INIT'
        self.t_state = 0.0
        self.calib_start = None
        self.calib_v1 = None
        self.detour_until = 0.0
        self.detour_dir = np.array([0.0, 0.0])
        self.detour_count = 0
        self.avoid_until = 0.0
        self.logf = open(self.log_path, 'w', buffering=1)
        self.create_timer(0.05, self.loop)   # 20Hz
        self.dlog(f'== planner_reactive: goal={self.goal} 安全距离={self.safe}m')

    # ---------------- 基础 ----------------
    def dlog(self, s):
        self.logf.write(s + '\n')
        self.logf.flush()

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cb_world(self, msg):
        if not msg.poses:
            return
        p = msg.poses[0].position
        self.world = (p.x, p.y, p.z)
        self.world_hist.append((self.now(), p.x, p.y, p.z))
        if len(self.world_hist) > 60:
            self.world_hist.pop(0)

    def world_vel(self, dt=1.0):
        """最近 dt 秒的世界系平均速度"""
        if len(self.world_hist) < 4:
            return None
        t1, x1, y1, z1 = self.world_hist[-1]
        for (t0, x0, y0, z0) in reversed(self.world_hist):
            if t1 - t0 >= dt:
                d = t1 - t0
                return ((x1 - x0) / d, (y1 - y0) / d, (z1 - z0) / d)
        return None

    def cb_depth(self, msg):
        try:
            d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        except Exception:
            return
        h, w = d.shape
        # 下采样 6x4 块, 取每块最小有效深度
        grid = []
        for gy in range(4):
            row = []
            for gx in range(6):
                blk = d[gy * h // 4:(gy + 1) * h // 4, gx * w // 6:(gx + 1) * w // 6]
                valid = blk[np.isfinite(blk) & (blk > 0.1) & (blk < 20)]
                row.append(float(valid.min()) if valid.size else 99.0)
            grid.append(row)
        self.depth = grid

    def send_command(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.now() * 1e6)
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = self.target_system
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def publish_velocity(self, vx_b, vy_b, vz_b, yaw=0.0):
        mode = OffboardControlMode()
        mode.timestamp = int(self.now() * 1e6)
        mode.velocity = True
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.now() * 1e6)
        sp.position = [float('nan'), float('nan'), float('nan')]
        sp.velocity = [float(vx_b), float(vy_b), float(vz_b)]
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    # ---------------- 深度: 左右净空 ----------------
    def side_clearance(self):
        """返回 (左侧最小深度, 右侧最小深度)"""
        if self.depth is None:
            return 99.0, 99.0
        left = min(self.depth[gy][gx] for gy in range(2) for gx in range(3))
        right = min(self.depth[gy][gx] for gy in range(2) for gx in range(3, 6))
        return left, right

    def front_min(self):
        """正前方障碍距离; 只用图像上半部 (下半部常年是地面回波, 会误判)"""
        if self.depth is None:
            return 99.0
        return min(min(row[1:5]) for row in self.depth[:2])

    # ---------------- 深度避障 (机体系) ----------------
    def avoidance_body(self):
        """返回 (前向减速, 横向推力) 机体系"""
        if self.depth is None:
            return 0.0, 0.0
        fwd_slow = 0.0
        lat = 0.0
        for gy in range(4):
            for gx in range(6):
                dmin = self.depth[gy][gx]
                if dmin > self.safe:
                    continue
                # 水平位置: -1(左) .. +1(右); 深度相机右为正 (图像 x 向右)
                lat_pos = (gx - 2.5) / 2.5
                w = (1.0 - dmin / self.safe)      # 越近越强
                w = max(0.0, min(1.0, w))
                fwd_slow += w
                lat -= lat_pos * w * 1.5           # 障碍在右 -> 向左推
        return min(1.0, fwd_slow), max(-1.0, min(1.0, lat))

    # ---------------- 主循环 ----------------
    def loop(self):
        if self.world is None:
            return
        t = self.now()
        wx, wy, wz = self.world
        gx, gy, gz = self.goal

        # 解锁 + offboard (起步阶段)
        if self.status is None or self.status.arming_state != 2:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

        if self.state == 'INIT':
            self.publish_velocity(0.0, 0.0, -0.6)     # 垂直爬升
            if abs(wz - self.cruise_z) < 0.35:
                self.state = 'CALIB1'
                self.t_state = t
                self.dlog(f'到达巡航高度 {wz:.2f}m, 开始自标定')
            return

        if self.state == 'CALIB1':
            if t - self.t_state < 1.8:
                self.publish_velocity(0.6, 0.0, 0.0)   # 机体系 +x
                return
            v = self.world_vel(1.2)
            if v is None:
                self.t_state = t - 0.5
                return
            self.calib_v1 = (v[0] / 0.6, v[1] / 0.6)   # 单位机体系 x 速度 -> 世界速度
            self.state = 'CALIB2'
            self.t_state = t
            self.dlog(f'标定 x: 机体系+x -> 世界({self.calib_v1[0]:.2f},{self.calib_v1[1]:.2f})')
            return

        if self.state == 'CALIB2':
            if t - self.t_state < 1.8:
                self.publish_velocity(0.0, 0.6, 0.0)   # 机体系 +y
                return
            v = self.world_vel(1.2)
            if v is None:
                self.t_state = t - 0.5
                return
            c2 = (v[0] / 0.6, v[1] / 0.6)
            self.M = np.array([[self.calib_v1[0], c2[0]],
                               [self.calib_v1[1], c2[1]]])
            det = np.linalg.det(self.M)
            self.dlog(f'标定 y: 机体系+y -> 世界({c2[0]:.2f},{c2[1]:.2f}); det={det:.2f}')
            if abs(det) < 0.2:
                self.dlog('标定矩阵退化, 重试')
                self.state = 'CALIB1'
                self.t_state = t
                return
            self.Minv = np.linalg.inv(self.M)
            self.state = 'RUN'
            self.t_state = t
            self.dlog('标定完成, 开始避障飞行')
            return

        if self.state == 'RUN':
            ex, ey, ez = gx - wx, gy - wy, gz - wz
            dist = math.sqrt(ex * ex + ey * ey + ez * ez)

            # --- 避障: 前方受阻则选一侧做世界系绕行 ---
            fmin = self.front_min()
            if fmin < self.safe and t >= self.detour_until - 0.1 and self.detour_count < 6:
                left, right = self.side_clearance()
                # 机体系 -y 方向在世界系的方向 (由标定矩阵得到)
                left_world = -self.M @ np.array([0.0, 1.0])
                left_world = left_world / (np.linalg.norm(left_world) + 1e-6)
                self.detour_dir = left_world if left >= right else -left_world
                self.detour_until = t + 2.5
                self.detour_count += 1
                self.avoid_until = t + 12.0
                self.dlog(f'遇障 fmin={fmin:.2f}m 左{left:.2f}/右{right:.2f} -> 绕行 '
                          f'({self.detour_dir[0]:.2f},{self.detour_dir[1]:.2f})')

            if t < self.detour_until and self.detour_count <= 6:
                vw = np.array([self.detour_dir[0], self.detour_dir[1]]) * self.vmax * 0.9
            else:
                vw = np.array([0.9 * ex, 0.9 * ey])
                n = np.linalg.norm(vw)
                if n > self.vmax:
                    vw = vw * self.vmax / n
            vz = 0.9 * ez
            vb = self.Minv @ vw
            vx_b, vy_b = float(vb[0]), float(vb[1])
            vz_b = float(-max(-0.8, min(0.8, vz)))
            if dist < self.tol:
                vx_b = vy_b = vz_b = 0.0
            self.publish_velocity(vx_b, vy_b, vz_b)
            if int(t * 2) % 2 == 0:
                self.dlog(f'pos=({wx:.2f},{wy:.2f},{wz:.2f}) goal_dist={dist:.2f} '
                          f'fmin={fmin:.2f} vb=({vx_b:.2f},{vy_b:.2f})')
            if dist < self.tol and (t - self.t_state) > 5:
                self.dlog(f'到达目标, 误差 {dist:.2f}m')
                self.state = 'HOLD'


def main(args=None):
    rclpy.init(args=args)
    node = PlannerReactive()
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
