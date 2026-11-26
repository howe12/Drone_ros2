#!/usr/bin/env python3
"""
E5 点云适配器: gz 16线激光 PointCloud2 (xyz) → PointXYZIRT (带 ring/time)
Point-LIO velodyne handler 需要 ring(线号) 与 time(点内时间偏移) 字段,
gz 桥接的点云只有 xyz, 本节点按几何关系补算:
  ring = 垂直角映射 (±15° → 0..15)
  time = 方位角比例 × 扫描周期 (10Hz → 0.1s)
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class Lidar3dAdapter(Node):
    def __init__(self):
        super().__init__('drone_sim_lidar3d_adapter')
        self.declare_parameter('input_topic', '/x500_lidar_3d_0/lidar_3d/points')
        self.declare_parameter('output_topic', '/velodyne_points')
        self.declare_parameter('scan_lines', 16)
        self.declare_parameter('vfov_deg', 30.0)     # ±15°
        self.declare_parameter('scan_period', 0.1)   # 10Hz
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.lines = self.get_parameter('scan_lines').value
        self.vfov = math.radians(self.get_parameter('vfov_deg').value)
        self.period = self.get_parameter('scan_period').value

        # Point-LIO 订阅端用默认 RELIABLE QoS, 发布端必须匹配 (BEST_EFFORT 会无法连接)
        self.pub = self.create_publisher(PointCloud2, out, 10)
        self.create_subscription(PointCloud2, inp, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'点云适配: {inp} -> {out} (ring/time 补算)')

    def cb(self, msg):
        # 解析 xyz (假设 offsets 0/4/8, float32)
        n = msg.width * msg.height
        if n == 0:
            return
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        xyz = np.zeros((n, 3), dtype=np.float32)
        for i, f in enumerate(msg.fields[:3]):
            xyz[:, i] = data[:, f.offset:f.offset + 4].copy().view(np.float32).flatten()
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        r = np.sqrt(x * x + y * y + z * z)
        # 关键: 过滤 inf/nan (gz 无回波点为 inf, 会破坏 PCL/ikd-tree 近邻搜索)
        valid = np.isfinite(xyz).all(axis=1) & (r > 0.3) & (r < 60.0)
        xyz = xyz[valid]
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        r = r[valid]
        n = len(xyz)
        if n == 0:
            return
        r_safe = np.maximum(r, 1e-6)
        # ring: 垂直角量化到最近波束 (16 线均布 ±15°)
        vert = np.arcsin(np.clip(z / r_safe, -1, 1))
        ring = np.clip(np.round((vert + self.vfov / 2) / (self.vfov / (self.lines - 1))).astype(np.uint16),
                       0, self.lines - 1)
        # time: 方位角 [-π, π] → [0, period)
        az = np.arctan2(y, x)
        t = ((az + math.pi) / (2 * math.pi) * self.period).astype(np.float32)
        intensity = np.zeros(n, dtype=np.float32)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 24
        out.row_step = 24 * n
        out.is_dense = True
        buf = np.zeros((n, 6), dtype=np.float32)
        buf[:, 0:3] = xyz
        buf[:, 3] = intensity
        buf[:, 5] = t
        raw = buf.view(np.uint8)
        raw[:, 16:18] = ring.view(np.uint8).reshape(n, 2)
        out.data = raw.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3dAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
