#!/usr/bin/env python3
"""
录制图像话题为视频（用于感知类场景：YOLO 标注图、相机画面等）
=================================================================
Gazebo 窗口录制看不到 YOLO 的检测框 —— 那是发布在图像话题上的，
必须直接录话题。

用法:
    /usr/bin/python3 tools/record_image_topic.py \
        --topic /drone_sim_yolo_detector/annotated \
        --out videos/vision.yolo_detect_raw.mp4 \
        --seconds 30

    # 录完自动转 GIF（供 GitHub README 内联播放）
    bash scripts/make_gif.sh videos/vision.yolo_detect_raw.mp4 videos/vision.yolo_detect.gif

说明:
    - 用 cv2.VideoWriter(mp4v)，帧率按实际收帧率统计，避免卡顿感
    - 只保留最近 seconds 秒；到时间自动退出
"""
import argparse
import os
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--fps', type=float, default=10.0, help='写入帧率上限')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    rclpy.init()
    node = Node('img_topic_recorder')
    bridge = CvBridge()
    writer = {'w': None, 'frames': 0}
    t0 = time.time()

    def cb(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        if writer['w'] is None:
            h, w = frame.shape[:2]
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            writer['w'] = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*'mp4v'),
                                          args.fps, (w, h))
            print(f'  开始录制 {w}x{h} @ {args.fps}fps -> {args.out}')
        writer['w'].write(frame)
        writer['frames'] += 1

    node.create_subscription(Image, args.topic, cb, qos_profile_sensor_data)
    print(f'  订阅 {args.topic}，录制 {args.seconds:.0f}s ...')

    while rclpy.ok() and time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    if writer['w'] is not None:
        writer['w'].release()
    dur = time.time() - t0
    print(f"  完成: {writer['frames']} 帧 / {dur:.1f}s "
          f"({writer['frames'] / max(dur, 1e-3):.1f} fps 实际)")
    if writer['frames'] == 0:
        print('  ✗ 没收到任何图像帧（话题名或节点未运行？）')
        return 1
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
