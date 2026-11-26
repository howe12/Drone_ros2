#!/usr/bin/env python3
"""
E4 辅助节点: 里程计 → TF (odom -> base_link)
slam_toolbox 需要 TF 树 map->odom->base_link; map->odom 由 SLAM 发布,
odom->base_link 由本节点从桥接的 nav_msgs/Odometry 转发。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('drone_sim_odom_to_tf')
        self.declare_parameter('odom_topic', '/x500_lidar_2d_0/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'odom->TF: {topic} ({self.odom_frame}->{self.base_frame})')

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
