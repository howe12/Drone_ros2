#!/usr/bin/env python3
"""
E10 行为树任务层 (py_trees + PX4 Offboard)

任务用 XML 描述 (config/mission_bt.xml), 节点类型:
  复合: <sequence> <selector> <parallel>
  动作: <Takeoff altitude> <GotoWaypoint x y z> <Land/> <RTL/> <Hover seconds>
        <EmergencyLand/> <WaitArmed/>
  条件: <AltitudeAbove meters/> <Timeout seconds> (装饰器: 子树超时则 FAILURE)

根节点建议: <selector> = [ 任务序列, 应急降落 ] —— 任务失败自动切应急分支。

用法:
  ros2 run drone_sim bt_mission --ros-args -p px4_ns:=/x500_0 -p mission_xml:=<path>
"""
import math
import os
import time
import xml.etree.ElementTree as ET

import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleStatus, VehicleLocalPosition)


# ---------------------------------------------------------------- PX4 接口
from drone_sim.paths import log_path


class DroneInterface:
    """与 PX4 交互的共享接口 (心跳/解锁/设定点/状态)"""

    def __init__(self, node, ns='/x500_0', target_system=1):
        self.node = node
        self.target_system = target_system
        self.status = None
        self.pos = None
        self.target = [0.0, 0.0, -2.0]
        self.offboard_requested = False
        self.log_fn = None      # 由 BtMission 注入的日志回调
        # EKF 原点稳定性门控 (重置计数器 5s 不变才允许解锁)
        self._last_reset = None
        self._stable_ticks = 0

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_mode = node.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', pub_qos)
        self.pub_sp = node.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', pub_qos)
        self.pub_cmd = node.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', pub_qos)
        node.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status',
                                 lambda m: setattr(self, 'status', m), sub_qos)
        node.create_subscription(VehicleLocalPosition,
                                 f'{ns}/fmu/out/vehicle_local_position',
                                 lambda m: setattr(self, 'pos', m), sub_qos)

    # -- 状态查询
    @property
    def armed(self):
        return self.status is not None and self.status.arming_state == 2

    @property
    def offboard(self):
        return self.status is not None and self.status.nav_state == 14

    def alt(self):
        """相对起飞点高度 (米, 向上为正)"""
        if self.pos is None or not self.pos.z_valid:
            return 0.0
        return -self.pos.z

    def remain(self, x, y, z):
        """到目标点水平/垂直距离 (传入 NED z)"""
        if self.pos is None:
            return 99.0
        dz = self.pos.z - z
        return math.sqrt((self.pos.x - x) ** 2 + (self.pos.y - y) ** 2 + dz * dz)

    # -- 控制
    def now_us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

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

    def _update_ekf_stability(self):
        if self.pos is None:
            return
        cur = (self.pos.xy_reset_counter, self.pos.z_reset_counter)
        if cur != self._last_reset:
            self._last_reset = cur
            self._stable_ticks = 0
        else:
            self._stable_ticks += 1

    def debug(self, msg):
        if self.log_fn is not None:
            self.log_fn(msg)

    @property
    def ekf_stable(self):
        return self._stable_ticks >= 50      # 10Hz * 5s

    def heartbeat(self):
        """10Hz 心跳 + 当前设定点"""
        self._update_ekf_stability()
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = list(self.target)
        sp.yaw = 0.0
        self.pub_sp.publish(sp)

    def request_offboard_arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    def land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


# ---------------------------------------------------------------- 动作节点
class Takeoff(py_trees.behaviour.Behaviour):
    def __init__(self, name, iface, altitude=3.0):
        super().__init__(name)
        self.iface = iface
        self.altitude = float(altitude)
        self.started = 0.0

    def initialise(self):
        self.started = time.time()
        self.iface.request_offboard_arm()
        self.iface.target = [0.0, 0.0, -self.altitude]

    def update(self):
        # 持续重试解锁直到超时; EKF 原点未稳定时先等待 (防 GPS 漂移预检拒绝)
        if not (self.iface.armed and self.iface.offboard):
            if time.time() - self.started < 12 and not self.iface.ekf_stable:
                return py_trees.common.Status.RUNNING
            self.iface.request_offboard_arm()
        if abs(self.iface.alt() - self.altitude) < 0.6:
            return py_trees.common.Status.SUCCESS
        if time.time() - self.started > 40:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


class GotoWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, iface, x, y, z, tol=0.4, timeout=45.0):
        super().__init__(name)
        self.iface = iface
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.tol = tol
        self.timeout = timeout
        self.started = 0.0

    def initialise(self):
        self.started = time.time()
        self.iface.target = [self.x, self.y, -self.z]

    def update(self):
        self.iface.target = [self.x, self.y, -self.z]
        rem = self.iface.remain(self.x, self.y, -self.z)
        self.iface.debug(f'  GotoWaypoint {self.name}: remain={rem:.2f} elapsed={time.time()-self.started:.1f}')
        if rem < self.tol:
            return py_trees.common.Status.SUCCESS
        if time.time() - self.started > self.timeout:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


class Hover(py_trees.behaviour.Behaviour):
    def __init__(self, name, iface, seconds=3.0):
        super().__init__(name)
        self.iface = iface
        self.seconds = float(seconds)
        self.started = 0.0

    def initialise(self):
        self.started = time.time()
        if self.iface.pos is not None:
            self.iface.target = [self.iface.pos.x, self.iface.pos.y, self.iface.pos.z]

    def update(self):
        if time.time() - self.started >= self.seconds:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class Land(py_trees.behaviour.Behaviour):
    def __init__(self, name, iface):
        super().__init__(name)
        self.iface = iface
        self.cmd_sent = False

    def initialise(self):
        self.cmd_sent = False

    def update(self):
        if not self.cmd_sent:
            self.iface.land()
            self.cmd_sent = True
        if not self.iface.armed:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class RTL(py_trees.behaviour.Behaviour):
    """返航: 回到起飞点上方再降落"""

    def __init__(self, name, iface):
        super().__init__(name)
        self.iface = iface
        self.phase = 'go'
        self.cmd_sent = False

    def initialise(self):
        self.phase = 'go'
        self.cmd_sent = False

    def update(self):
        if self.phase == 'go':
            self.iface.target = [0.0, 0.0, -(max(self.iface.alt(), 2.0))]
            if self.iface.remain(0.0, 0.0, -max(self.iface.alt(), 2.0)) < 0.5:
                self.phase = 'land'
            return py_trees.common.Status.RUNNING
        if not self.cmd_sent:
            self.iface.land()
            self.cmd_sent = True
        if not self.iface.armed:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class EmergencyLand(py_trees.behaviour.Behaviour):
    """应急降落: 立即发 NAV_LAND, 直到上锁"""

    def __init__(self, name, iface):
        super().__init__(name)
        self.iface = iface
        self.cmd_sent = False

    def initialise(self):
        self.cmd_sent = False
        if self.iface.node is not None:
            self.iface.node.get_logger().error('!!! 应急降落触发 !!!')

    def update(self):
        if not self.cmd_sent:
            self.iface.land()
            self.cmd_sent = True
        if not self.iface.armed:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class WaitArmed(py_trees.behaviour.Behaviour):
    def __init__(self, name, iface, timeout=20.0):
        super().__init__(name)
        self.iface = iface
        self.timeout = timeout
        self.started = 0.0

    def initialise(self):
        self.started = time.time()

    def update(self):
        if self.iface.armed:
            return py_trees.common.Status.SUCCESS
        if time.time() - self.started > self.timeout:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


# ---------------------------------------------------------------- 条件/装饰器
class AltitudeAbove(py_trees.behaviour.Behaviour):
    def __init__(self, name, iface, meters=2.0):
        super().__init__(name)
        self.iface = iface
        self.meters = float(meters)

    def update(self):
        ok = self.iface.alt() > self.meters
        return py_trees.common.Status.SUCCESS if ok else py_trees.common.Status.FAILURE


class Timeout(py_trees.behaviour.Behaviour):
    """子树在给定秒数内未完成则 FAILURE (自包含实现, 自行 tick 子树)"""

    def __init__(self, child, seconds=60.0):
        super().__init__(name=f'Timeout({seconds}s)')
        self.child = child
        self.seconds = float(seconds)
        self.started = 0.0

    def initialise(self):
        self.started = time.time()
        self.child.stop(py_trees.common.Status.INVALID)

    def update(self):
        el = time.time() - self.started
        iface = getattr(self.child, 'iface', None)
        if iface is not None:
            iface.debug(f'  Timeout: elapsed={el:.1f}/{self.seconds} child={self.child.status.name}')
        if el > self.seconds:
            self.child.stop(py_trees.common.Status.INVALID)
            return py_trees.common.Status.FAILURE
        # 手动驱动子树 (与 Behaviour.tick 语义一致, 避免版本差异)
        if self.child.status != py_trees.common.Status.RUNNING:
            self.child.initialise()
        st = self.child.update()
        self.child.status = st
        return st


# ---------------------------------------------------------------- XML 构建
def build_node(elem, iface, index=[0]):
    tag = elem.tag
    kids = list(elem)
    index[0] += 1
    name = elem.get('name') or f'{tag}_{index[0]}'
    if tag == 'sequence':
        children = [build_node(c, iface) for c in kids]
        return py_trees.composites.Sequence(name=name, memory=True, children=children)
    if tag == 'selector':
        children = [build_node(c, iface) for c in kids]
        return py_trees.composites.Selector(name=name, memory=False, children=children)
    if tag == 'parallel':
        children = [build_node(c, iface) for c in kids]
        policy = py_trees.common.ParallelPolicy.SuccessOnAll()
        return py_trees.composites.Parallel(name=name, policy=policy, children=children)
    if tag == 'Takeoff':
        return Takeoff(name, iface, altitude=elem.get('altitude', 3.0))
    if tag == 'GotoWaypoint':
        return GotoWaypoint(name, iface, elem.get('x', 0), elem.get('y', 0), elem.get('z', 3),
                            tol=float(elem.get('tol', 0.4)),
                            timeout=float(elem.get('timeout', 45.0)))
    if tag == 'Hover':
        return Hover(name, iface, seconds=elem.get('seconds', 3.0))
    if tag == 'Land':
        return Land(name, iface)
    if tag == 'RTL':
        return RTL(name, iface)
    if tag == 'EmergencyLand':
        return EmergencyLand(name, iface)
    if tag == 'WaitArmed':
        return WaitArmed(name, iface, timeout=float(elem.get('timeout', 20.0)))
    if tag == 'AltitudeAbove':
        return AltitudeAbove(name, iface, meters=elem.get('meters', 2.0))
    if tag == 'Timeout':
        child = build_node(kids[0], iface)
        return Timeout(child, seconds=elem.get('seconds', 60.0))
    raise ValueError(f'未知节点类型: {tag}')


# ---------------------------------------------------------------- 主节点
class BtMission(Node):
    def __init__(self):
        super().__init__('drone_sim_bt_mission')
        self.declare_parameter('px4_ns', '/x500_0')
        self.declare_parameter('mission_xml', '')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('log_file', log_path('e10_bt.log'))
        ns = self.get_parameter('px4_ns').value
        xml_path = self.get_parameter('mission_xml').value
        target = self.get_parameter('target_system').value
        self.log_path = self.get_parameter('log_file').value

        self.iface = DroneInterface(self, ns=ns, target_system=target)
        self.iface.log_fn = lambda m: None   # 稍后绑定到 dlog
        with open(xml_path) as f:
            root_elem = ET.parse(f).getroot()
        mission_elem = root_elem if root_elem.tag != 'mission' else (list(root_elem)[0])
        self.root = build_node(mission_elem, self.iface)
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup(timeout=15)

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.logf = open(self.log_path, 'w', buffering=1)
        self.iface.log_fn = self.dlog
        self.tick = 0
        self.done = False
        self.create_timer(0.1, self.loop)
        self.get_logger().info(f'行为树任务启动: {xml_path} (ns={ns})')

    def dlog(self, s):
        self.logf.write(s + '\n')
        self.logf.flush()

    def loop(self):
        if self.done:
            return
        self.iface.heartbeat()
        self.tree.tick()
        self.tick += 1
        if self.tick % 10 == 0:      # 1Hz: 记录树状态
            self.dlog(f'--- tick {self.tick} alt={self.iface.alt():.2f} '
                      f'armed={self.iface.armed} offboard={self.iface.offboard}')
            for line in py_trees.display.ascii_tree(self.root, show_status=True).splitlines():
                self.dlog('  ' + line)
        if self.root.status in (py_trees.common.Status.SUCCESS,
                                py_trees.common.Status.FAILURE):
            self.dlog(f'=== 任务结束: {self.root.status.name} (tick {self.tick}) ===')
            self.get_logger().info(f'任务结束: {self.root.status.name}')
            if self.iface.armed:
                self.iface.land()
            self.done = True
            self.logf.close()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BtMission()
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
