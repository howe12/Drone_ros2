#!/usr/bin/env python3
"""
PX4 uXRCE-DDS 统一接口 (平台层核心)
=====================================

**为什么存在**: 此前 8 个节点 (offboard_demo / waypoint_mission / formation_follower /
aruco_precision_landing / bt_mission / swarm_agent / planner_reactive / safety_supervisor)
各自实现了一份几乎相同但**语义不一致**的 PX4 接口, 直接导致过这些真实故障:
  - target_system 默认值不一致 -> 多机时 2 号机无法解锁
  - QoS 不一致 (RELIABLE vs BEST_EFFORT) -> 与 Point-LIO 订阅端连不上
  - 只有部分节点实现 EKF 原点门控 -> 遇 GPS 漂移时"贴地爬行"
  - 坐标系转换各自手写 (ENU/NED/航向) -> 飞反、打转

本模块把上述内容收敛为**唯一实现**, 供所有节点与课程示例复用。

设计约定 (踩坑后固化, 请勿随意更改):
  1. 发布端 QoS 必须 BEST_EFFORT + VOLATILE
     (RELIABLE 在 FastDDS SHM 缓冲满时会硬死锁, 实测过一次整机卡死)
  2. 订阅端 QoS 用 BEST_EFFORT + TRANSIENT_LOCAL, 与 PX4 侧一致
  3. offboard 心跳频率须 > 2Hz (本模块默认 10Hz, 由调用方定时器驱动)
  4. 解锁/切模式前应先预流 setpoint >= 1s (PX4 要求)
  5. 高度: PX4 z 向下为正; 本模块 alt() 返回**向上为正**的实用值
  6. 世界系(gz/ENU) <-> 机体系(PX4/NED) 转换见 to_ned()/to_local()

用法 (推荐, 组合而非继承):
    class MyMission(Node):
        def __init__(self):
            super().__init__('my_mission')
            self.px4 = PX4Interface(self, ns='/x500_0', target_system=1)
            self.create_timer(0.1, self.loop)
        def loop(self):
            self.px4.heartbeat_position(x=0, y=0, z_up=3.0)   # 持续心跳+设定点
            if self.px4.armed and self.px4.offboard:
                ...
"""

import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
)

# ---------------------------------------------------------------- 常量 (勿用魔法数字)
ARMED = VehicleStatus.ARMING_STATE_ARMED          # == 2
NAV_OFFBOARD = VehicleStatus.NAVIGATION_STATE_OFFBOARD  # == 14
NAV_LAND = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
NAV_RTL = VehicleStatus.NAVIGATION_STATE_AUTO_RTL

CMD_DO_SET_MODE = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
CMD_ARM_DISARM = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
CMD_NAV_LAND = VehicleCommand.VEHICLE_CMD_NAV_LAND
CMD_NAV_RTL = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
CMD_NAV_TAKEOFF = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
CMD_VTOL_TRANSITION = VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION

MAIN_MODE_OFFBOARD = 6.0
MAIN_MODE_POSCTL = 22.0

# ---------------------------------------------------------------- QoS 预设
# 发布: PX4 侧是 best-effort 语义; RELIABLE 会在 SHM 满时死锁 (实测)
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
# 订阅: 与 PX4 侧发布保持一致
QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class PX4Interface:
    """一个 PX4 实例 (一个 uXRCE-DDS 命名空间) 的统一控制接口。"""

    def __init__(self, node, ns='', target_system=1, subscribe_attitude=False,
                 on_status=None, on_position=None):
        """
        Args:
            node: rclpy Node (提供 create_publisher/subscription/timer 与时钟)
            ns:   ''            -> 相对话题 /fmu/... (单机 demo, 走 launch remap)
                  '/x500_0'     -> 绝对话题 /x500_0/fmu/...
            target_system: PX4 MAV_SYS_ID, 多机时必须 = 实例号 + 1 (踩过的坑)
            subscribe_attitude: 需要真实倾角时置 True (如安全框架)
            on_status / on_position: 可选回调, 供节点做自己的记录/可视化
        """
        self.node = node
        self.ns = ns.rstrip('/') if ns else ''
        self.target_system = int(target_system)

        def t(suffix):
            return f'{self.ns}/fmu/{suffix}' if self.ns else f'/fmu/{suffix}'

        self.pub_mode = node.create_publisher(OffboardControlMode, t('in/offboard_control_mode'), QOS_PUB)
        self.pub_sp = node.create_publisher(TrajectorySetpoint, t('in/trajectory_setpoint'), QOS_PUB)
        self.pub_cmd = node.create_publisher(VehicleCommand, t('in/vehicle_command'), QOS_PUB)

        self._on_status = on_status
        self._on_position = on_position
        self.status = None
        self.pos = None
        self.att = None
        self._reset = None
        self._stable_ticks = 0

        node.create_subscription(VehicleStatus, t('out/vehicle_status'),
                                 self._cb_status, QOS_SUB)
        node.create_subscription(VehicleLocalPosition, t('out/vehicle_local_position'),
                                 self._cb_pos, QOS_SUB)
        if subscribe_attitude:
            node.create_subscription(VehicleAttitude, t('out/vehicle_attitude'),
                                     self._cb_att, QOS_SUB)

    # ---------------- 订阅 ----------------
    def _cb_status(self, msg):
        self.status = msg
        if self._on_status is not None:
            self._on_status(msg)

    def _cb_pos(self, msg):
        self.pos = msg
        # EKF 原点稳定性门控 (E5 踩坑: 原点跳变会让"地面=local z=-5", 飞机贴地爬行)
        cur = (msg.xy_reset_counter, msg.z_reset_counter)
        if cur != self._reset:
            self._reset = cur
            self._stable_ticks = 0
        else:
            self._stable_ticks += 1
        if self._on_position is not None:
            self._on_position(msg)

    def _cb_att(self, msg):
        self.att = msg

    # ---------------- 状态查询 ----------------
    @property
    def ready(self):
        """是否已收到 PX4 状态与位置"""
        return self.status is not None and self.pos is not None

    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == ARMED

    @property
    def offboard(self):
        return self.status is not None and self.status.nav_state == NAV_OFFBOARD

    @property
    def nav_state(self):
        return self.status.nav_state if self.status is not None else -1

    @property
    def ekf_stable(self):
        """EKF 重置计数器 5s (10Hz*50) 未变。解锁前必须为 True。"""
        return self._stable_ticks >= 50

    def alt(self):
        """相对起飞点的高度, 向上为正 (m)。未就绪返回 0.0"""
        if self.pos is None or not self.pos.z_valid:
            return 0.0
        return -self.pos.z

    def heading(self):
        return self.pos.heading if (self.pos is not None and self.pos.z_valid) else 0.0

    def tilt_rad(self):
        """机体 z 轴与世界 z 轴夹角 (需 subscribe_attitude=True)"""
        if self.att is None:
            return 0.0
        q = self.att.q  # [w, x, y, z]
        cos_tilt = 2.0 * (q[0] * q[0] + q[3] * q[3]) - 1.0
        return math.acos(max(-1.0, min(1.0, cos_tilt)))

    def distance_to(self, x, y, z_up):
        """当前位姿到 NED 目标 (x, y, z_down) 的三维距离"""
        if self.pos is None:
            return float('inf')
        dz = self.pos.z - z_up
        return math.sqrt((self.pos.x - x) ** 2 + (self.pos.y - y) ** 2 + dz * dz)

    # ---------------- 坐标系转换 ----------------
    @staticmethod
    def enu_to_ned(x, y, z_up):
        """gz 世界系 ENU (x=东, y=北, z=上) -> PX4 局部 NED (x=北, y=东, z=下)"""
        return float(y), float(x), -float(z_up)

    @staticmethod
    def body_to_world_vel(vx_body, vy_body, heading):
        """机体系速度 -> 世界系速度 (按航向旋转)"""
        c, s = math.cos(heading), math.sin(heading)
        return c * vx_body + s * vy_body, s * vx_body - c * vy_body

    # ---------------- 发布 ----------------
    def _now_us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def heartbeat_position(self, x, y, z_down, yaw=0.0):
        """位置控制心跳: 必须先于/同时于解锁调用 (需持续 >2Hz)"""
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        mode.position = True
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        sp.position = [float(x), float(y), float(z_down)]
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    def heartbeat_velocity(self, vx, vy, vz_down, yaw=0.0):
        """速度控制心跳 (位置字段置 NaN 以免冲突)"""
        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        mode.velocity = True
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        sp.position = [float('nan')] * 3
        sp.velocity = [float(vx), float(vy), float(vz_down)]
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    def heartbeat_hold(self):
        """原地悬停 (基于当前位姿)"""
        if self.pos is None:
            self.heartbeat_position(0.0, 0.0, -2.0)
            return
        self.heartbeat_position(self.pos.x, self.pos.y, self.pos.z, self.pos.heading)

    def send_command(self, command, p1=0.0, p2=0.0, p3=0.0, p7=0.0):
        c = VehicleCommand()
        c.timestamp = self._now_us()
        c.command = command
        c.param1, c.param2, c.param3, c.param7 = float(p1), float(p2), float(p3), float(p7)
        c.target_system = self.target_system     # 多机必须匹配 MAV_SYS_ID
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    # ---------------- 常用动作 ----------------
    def set_offboard_mode(self):
        self.send_command(CMD_DO_SET_MODE, 1.0, MAIN_MODE_OFFBOARD)

    def set_posctl_mode(self):
        self.send_command(CMD_DO_SET_MODE, 1.0, MAIN_MODE_POSCTL)

    def arm(self):
        self.send_command(CMD_ARM_DISARM, 1.0, 0.0)

    def disarm(self):
        self.send_command(CMD_ARM_DISARM, 0.0, 0.0)

    def takeoff(self, altitude_m):
        self.send_command(CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, float(altitude_m))

    def land(self):
        self.send_command(CMD_NAV_LAND)

    def rtl(self):
        self.send_command(CMD_NAV_RTL)

    def vtol_transition_to_fw(self):
        """切固定翼: 必须先切 POSCTL, 否则 PX4 不执行过渡 (E11 踩坑)"""
        self.set_posctl_mode()
        self.send_command(CMD_VTOL_TRANSITION, 4.0)

    def vtol_transition_to_mc(self):
        self.send_command(CMD_VTOL_TRANSITION, 3.0)

    def request_offboard_and_arm(self, require_ekf_stable=True):
        """请求 Offboard + 解锁。

        Args:
            require_ekf_stable: True 时 EKF 未稳定则**不解锁**(只切模式),
                                避免 GPS 漂移预检拒绝后"贴地爬行"
        Returns:
            (已请求模式, 已请求解锁)
        """
        self.set_offboard_mode()
        if require_ekf_stable and not self.ekf_stable:
            return True, False
        self.arm()
        return True, True
