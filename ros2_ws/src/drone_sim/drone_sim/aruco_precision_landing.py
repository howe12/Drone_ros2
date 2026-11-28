#!/usr/bin/env python3
"""
E8 ArUco 精准降落节点 (Issue #8)
================================
下视相机检测 ArUco 靶标 → 像素误差视觉伺服 → 分级下降 → 降落在靶标中心。

坐标映射 (下视相机 pitch=90°, 无偏航):
  图像右 = 世界 -y, 图像下 = 世界 -x  (gz 相机 +x 视轴, 经 R_y(90) 朝下)
  因此 drone 修正: dx = -k*(v-cy), dy = -k*(u-cx)

用法:
  ros2 run drone_sim aruco_precision_landing --ros-args -p camera_ns:=/x500_mono_cam_down_0
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus,
)


from drone_sim.paths import log_path


class ArucoPrecisionLanding(Node):
    def __init__(self):
        super().__init__('drone_sim_aruco_precision_landing')
        self.declare_parameter('camera_ns', '/x500_mono_cam_down_0')
        self.declare_parameter('marker_id', 42)
        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('align_tol_px', 25.0)     # 居中容差(像素)
        self.declare_parameter('servo_gain', 0.004)      # 像素→米 增益
        self.declare_parameter('servo_sign_x', -1.0)     # ey→机体系x (实测: 图像下=世界-x)
        self.declare_parameter('servo_sign_y', 1.0)      # ex→机体系y (实测: 图像右=世界+y)
        self.declare_parameter('target_system', 1)

        ns = self.get_parameter('camera_ns').value
        self.marker_id = self.get_parameter('marker_id').value
        self.alt = self.get_parameter('takeoff_altitude').value
        self.tol_px = self.get_parameter('align_tol_px').value
        self.gain = self.get_parameter('servo_gain').value
        self.sign_x = self.get_parameter('servo_sign_x').value
        self.sign_y = self.get_parameter('servo_sign_y').value
        self.target_system = self.get_parameter('target_system').value

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)

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
        self.create_subscription(Image, f'{ns}/camera/image_raw', self.cb_image,
                                 qos_profile_sensor_data)
        # 世界系真值位姿 (gz 桥接), 用于落点精度验证 (局部坐标原点会漂移)
        self.world_pose = None
        self.create_subscription(PoseArray, f'{ns}/pose',
                                 lambda m: setattr(self, 'world_pose', m.poses[0]) if m.poses else None,
                                 qos_profile_sensor_data)

        self.status = None
        self.pos = None
        self.marker_px = None      # (u, v) 靶标中心像素
        self.img_size = None
        self.last_detect_tick = -1000
        self.state = 'INIT'
        self.tick = 0
        self.last_cmd_tick = -100
        self.target = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        self.descend_stage = 0     # 0=3m 1=1.5m 2=0.6m
        self.centered_ticks = 0
        self.stage_alt = [self.alt, 1.5, 0.6]
        self.px_center = None      # 像素基准 (光心, 原点无关)
        self.calib_samples = []
        self.err_window = []       # 最终阶段误差滑窗 (统计居中判定)
        self.create_timer(0.1, self.loop)
        # 直接文件日志 (绕过 C stdout 管道缓冲, 实时可查)
        # 路径可移植: 跟随仓库位置, 见 drone_sim/paths.py
        self.dbg = open(log_path('e8_debug.log'), 'a', buffering=1)
        self.get_logger().info(f'ArUco 精准降落启动: 靶标 ID{self.marker_id}, 起飞 {self.alt}m')

    def dlog(self, s):
        self.dbg.write(f'[{self.tick}] {s}\n')
        self.dbg.flush()

    # ---- 视觉 ----
    def cb_image(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.img_size = (msg.width, msg.height)
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is not None and self.marker_id in ids.flatten():
            idx = list(ids.flatten()).index(self.marker_id)
            c = corners[idx][0]
            self.marker_px = (float(c[:, 0].mean()), float(c[:, 1].mean()))
            self.last_detect_tick = self.tick
        else:
            self.marker_px = None

    # ---- 发布 ----
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
        c.target_system = self.target_system
        c.target_component = 1
        c.source_system = c.source_component = 1
        c.from_external = True
        c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(c)

    def pixel_error(self):
        if self.marker_px is None or self.img_size is None:
            return None
        w, h = self.img_size
        cx, cy = self.px_center if self.px_center else (w / 2, h / 2)
        return (self.marker_px[0] - cx, self.marker_px[1] - cy)

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
            if self.tick > 20 and self.tick - self.last_cmd_tick >= 10:
                self.last_cmd_tick = self.tick
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.get_logger().info('请求 Offboard + 解锁 ...')
            if armed and offboard:
                self.state = 'TAKEOFF'
                self.target = [0.0, 0.0, -self.alt]
                self.get_logger().info(f'起飞至 {self.alt}m (靶标正上方)')

        elif self.state == 'TAKEOFF':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            if self.pos and self.pos.z_valid and abs(self.pos.z + self.alt) < 0.25:
                self.state = 'ALIGN'
                self.descend_stage = 0
                # 不用局部坐标标定: EKF 原点会重置 (xy_reset_counter>0),
                # 直接以图像光心为基准, 伺服平衡点=靶标在光轴上=靶标正上方 (原点无关)
                self.px_center = (self.img_size[0] / 2, self.img_size[1] / 2) if self.img_size else None
                self.get_logger().info('到达高度, 开始靶标对齐 (光心基准)')

        elif self.state == 'ALIGN':
            if not (armed and offboard):
                self.state = 'INIT'
                return
            err = self.pixel_error()
            if err is None:
                self.centered_ticks = 0
                if self.tick % 30 == 0:
                    self.get_logger().warn('未检测到靶标, 保持位置')
                return
            ex, ey = err
            final_stage = self.descend_stage == len(self.stage_alt) - 1
            # 最终阶段误差滑窗 (每 tick 采样, 统计居中判定用)
            if final_stage:
                self.err_window.append(math.sqrt(ex * ex + ey * ey))
                if len(self.err_window) > 50:
                    self.err_window.pop(0)
            # 高度自适应: 像素→米 比例随高度线性变化 (3m 为基准)
            cur_alt = abs(self.pos.z) if (self.pos and self.pos.z_valid) else self.alt
            alt_scale = max(cur_alt, 0.3) / self.alt
            gain_eff = self.gain * alt_scale
            tol_eff = self.tol_px / alt_scale     # 低空放宽像素容差(对应相同米制精度)
            if final_stage:
                tol_eff = min(tol_eff, 40.0)      # 最终阶段收紧: ≈5cm 落点精度
                gain_eff *= 0.5                   # 低空减速防极限环振荡
            if self.tick % 20 == 0:
                self.dlog(f'err=({ex:.0f},{ey:.0f})px tol={tol_eff:.0f} stage={self.descend_stage} '
                          f'h={math.degrees(self.pos.heading):.0f}deg '
                          f'target=({self.target[0]:.2f},{self.target[1]:.2f},{self.target[2]:.2f}) '
                          f'pos=({self.pos.x:.2f},{self.pos.y:.2f},{self.pos.z:.2f})' if self.pos else
                          f'err=({ex:.0f},{ey:.0f})px stage={self.descend_stage}')
            if abs(ex) < tol_eff and abs(ey) < tol_eff:
                self.centered_ticks += 1
                # 持续居中 1.5s 才进下一阶段 (防振荡误触发)
                if self.centered_ticks >= 15:
                    self.centered_ticks = 0
                    if self.descend_stage < len(self.stage_alt) - 1:
                        self.descend_stage += 1
                        self.target[2] = -self.stage_alt[self.descend_stage]
                        self.err_window = []
                        self.dlog(f'居中完成, 下降至 {self.stage_alt[self.descend_stage]}m')
                    else:
                        self.state = 'LAND'
                        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        self.dlog('最终对齐完成, 降落')
            else:
                self.centered_ticks = 0
                # 最终阶段: 统计居中判定 (极限环振荡下瞬时容差永不满足,
                # 用 5s 滑窗均值判定; PX4 降落保持 XY, 落点偏差≈悬停偏差)
                if final_stage and len(self.err_window) >= 50:
                    mean_err = sum(self.err_window) / len(self.err_window)
                    if mean_err < 80.0:
                        self.state = 'LAND'
                        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        self.dlog(f'统计居中达标 (5s均值 {mean_err:.0f}px), 降落')
                        return
                # 像素→机体系修正, 再按实际偏航旋转到世界系 (关键: EKF heading 可能≠0)
                cb_x = self.sign_x * gain_eff * ey
                cb_y = self.sign_y * gain_eff * ex
                h = self.pos.heading if (self.pos and self.pos.z_valid) else 0.0
                corr_x = math.cos(h) * cb_x - math.sin(h) * cb_y
                corr_y = math.sin(h) * cb_x + math.cos(h) * cb_y
                norm = math.sqrt(corr_x * corr_x + corr_y * corr_y)
                max_step = 0.008 if final_stage else 0.02
                if norm > max_step:
                    corr_x *= max_step / norm
                    corr_y *= max_step / norm
                if self.tick - self.last_detect_tick <= 5:
                    self.target[0] = max(-6.0, min(6.0, self.target[0] + corr_x))
                    self.target[1] = max(-6.0, min(6.0, self.target[1] + corr_y))
                self.target[2] = -self.stage_alt[self.descend_stage]

        elif self.state == 'LAND':
            if not armed:
                self.state = 'DONE'
                if self.world_pose:
                    wx, wy = self.world_pose.position.x, self.world_pose.position.y
                    dist = math.sqrt(wx * wx + wy * wy)
                    self.dlog(f'已降落上锁, 世界系落点 ({wx:.3f},{wy:.3f}), '
                              f'距靶标中心 {dist:.3f}m')
                else:
                    self.dlog('已降落上锁 (无世界系位姿)')

        elif self.state == 'DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPrecisionLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
