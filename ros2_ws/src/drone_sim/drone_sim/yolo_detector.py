#!/usr/bin/env python3
"""
E8 YOLO 目标检测节点 (Issue #8)
订阅相机图像 → YOLOv8 推理 → 发布标注图像 + 检测结果日志。
用法:
  ros2 run drone_sim yolo_detector --ros-args -p image_topic:=/x500_mono_cam_down_0/camera/image_raw
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class YoloDetector(Node):
    def __init__(self):
        super().__init__('drone_sim_yolo_detector')
        self.declare_parameter('image_topic', '/x500_mono_cam_0/camera/image_raw')
        self.declare_parameter('model', '')              # 空=用包内 yolov8n.pt
        self.declare_parameter('infer_every_n', 5)     # 每 N 帧推理一次
        self.declare_parameter('conf', 0.4)
        topic = self.get_parameter('image_topic').value

        import os
        from ament_index_python.packages import get_package_share_directory
        model_path = self.get_parameter('model').value or os.path.join(
            get_package_share_directory('drone_sim'), 'models', 'yolov8n.pt')
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.conf = self.get_parameter('conf').value
        self.every_n = self.get_parameter('infer_every_n').value
        self.frame_n = 0

        self.pub_annotated = self.create_publisher(Image, '~/annotated', 10)
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'YOLO 检测启动: {topic}, 模型 {self.get_parameter("model").value}')

    def cb(self, msg):
        self.frame_n += 1
        if self.frame_n % self.every_n != 0:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        results = self.model(img, conf=self.conf, verbose=False)
        names = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                names.append(f'{r.names[cls]}:{float(box.conf[0]):.2f}')
        if names:
            self.get_logger().info(f'检测: {", ".join(names)}')
        annotated = results[0].plot()
        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = annotated.tobytes()
        self.pub_annotated.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
