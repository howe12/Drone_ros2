#!/usr/bin/env python3
"""
E6 VIO 精度评估: 对比 VINS-Fusion 里程计与 gz 真值位姿
订阅: /vins_estimator/odometry (nav_msgs/Odometry) + 真值 (geometry_msgs/PoseArray 或 Odometry)
输出: 实时误差日志 + 汇总 (RMSE, 最大误差)

用法:
  ros2 run drone_sim vio_evaluator --ros-args -p vins_topic:=/vins_estimator/odometry \
      -p truth_topic:=/x500_mono_cam_0/pose
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray


class VioEvaluator(Node):
    def __init__(self):
        super().__init__('drone_sim_vio_evaluator')
        self.declare_parameter('vins_topic', '/vins_estimator/odometry')
        self.declare_parameter('truth_topic', '/x500_mono_cam_0/pose')
        vt = self.get_parameter('vins_topic').value
        tt = self.get_parameter('truth_topic').value

        self.vins = None
        self.truth = None
        self.samples = []     # (vx, vy, vz, tx, ty, tz)
        self.create_subscription(Odometry, vt, self.cb_vins, qos_profile_sensor_data)
        self.create_subscription(PoseArray, tt, self.cb_truth, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'VIO 评估: {vt} vs {tt}')

    def cb_vins(self, msg):
        p = msg.pose.pose.position
        self.vins = (p.x, p.y, p.z)

    def cb_truth(self, msg):
        if not msg.poses:
            return
        p = msg.poses[0].position
        self.truth = (p.x, p.y, p.z)
        if self.vins is not None:
            self.samples.append((*self.vins, *self.truth))

    def report(self):
        n = len(self.samples)
        if n == 0:
            self.get_logger().warn('尚无忌位对比样本')
            return
        import numpy as np
        a = np.array(self.samples[-2000:])
        # VINS 系与真值系存在初始偏航/平移, 这里比较轨迹增量误差 (相对位移)
        v_rel = a[:, :3] - a[0, :3]
        t_rel = a[:, 3:] - a[0, 3:]
        err = np.linalg.norm(v_rel - t_rel, axis=1)
        self.get_logger().info(
            f'样本 {n} | 轨迹增量误差 RMSE {float(np.sqrt((err**2).mean())):.3f} m '
            f'| 最大 {float(err.max()):.3f} m | 当前 {err[-1]:.3f} m')


def main(args=None):
    rclpy.init(args=args)
    node = VioEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
