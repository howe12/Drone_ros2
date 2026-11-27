# 03 传感器话题桥接

> 难度 L1 · 对应实验 E1（Issue #1，已完成）· 视觉/SLAM/避障章的前置

## 学习目标

- 会按任务选机型（相机/深度/2D激光/3D激光），一条命令换装
- 理解 Gazebo→ROS2 的传感器桥接（ros_gz_bridge + 命名空间推导）
- 用 `ros2 topic hz` 验证每种传感器的真实频率

## 3.1 机型库：按传感器选飞机

| 机型 | 传感器 | 启动命令 | 用途章节 |
|---|---|---|---|
| `gz_x500` | IMU | `run_simulation.sh xtd2` | 控制入门 |
| `gz_x500_mono_cam` | 前置单目相机 | `run_simulation.sh xtd2 gz_x500_mono_cam` | 05 YOLO / 08 VIO |
| `gz_x500_depth` | OakD 深度相机 | `run_simulation.sh xtd2 gz_x500_depth` | 09 避障 |
| `gz_x500_mono_cam_down` | 下视相机（自建机架4013） | `run_simulation.sh xtd2 gz_x500_mono_cam_down` | 06 精准降落 |
| `gz_x500_lidar_2d` | 2D 激光（自建机架4012） | `run_simulation.sh xtd2 gz_x500_lidar_2d` | 07 2D SLAM |
| `gz_x500_lidar_3d` | 16线3D激光（自建机架4014） | `run_simulation.sh xtd2 gz_x500_lidar_3d` | 07 3D SLAM |

命名空间自动推导：`gz_x500_mono_cam` → `/x500_mono_cam_0`（去掉 `gz_` 前缀 + 实例号）。

## 3.2 传感器话题速查（实测频率）

| 话题（`{ns}` = 机型命名空间） | 类型 | 实测 |
|---|---|---|
| `{ns}/camera/image_raw` | sensor_msgs/Image | 23.6 Hz, 1280×960 |
| `{ns}/camera/camera_info` | CameraInfo | 内参 fx=fy=539.94 |
| `{ns}/imu` | sensor_msgs/Imu | 253 Hz（含重力） |
| `{ns}/scan` | sensor_msgs/LaserScan | 30.4 Hz, 270°/30m |
| `{ns}/depth_camera/image_raw` | Image（深度） | 25.6 Hz |
| `{ns}/depth_camera/points` | PointCloud2 | 28.1 Hz |
| `{ns}/lidar_3d/points` | PointCloud2（16线） | 见 07 章 |
| `{ns}/pose` | PoseArray | **gz 世界系真值**（判分用） |
| `/clock` | Clock | 仿真时钟（use_sim_time 全链路） |

验证方法：

```bash
ros2 topic hz /x500_mono_cam_0/camera/image_raw
ros2 topic echo /x500_mono_cam_0/camera/camera_info --once   # 看内参
```

![相机画面](images/07_camera_frame.png)

## 3.3 桥接是怎么工作的

```
Gazebo 传感器插件 --gz.transport--> ros_gz_bridge(parameter_bridge) --DDS--> ROS2 话题
```

- 桥接配置是 yaml 模板（`xtd2_launch/launch/launch_config/`），按命名空间实例化
- 课程补丁 0007/0008：新增机架 4012/4013/4014、传感器桥接扩展
- 补丁 0011：`/clock` + 带协方差里程计桥接（SLAM 需要）
- **Humble 的 ros_gz 必须源码编译对接 Harmonic**（二进制只支持 Fortress）——setup 脚本已处理

## 3.4 故障实验室

1. **QoS 连不上**：传感器话题是 BEST_EFFORT，订阅端用默认 RELIABLE 会静默收不到数据（不报错！）。`ros2 topic echo` 能收到而节点收不到时，先查 QoS。
2. **点云含 inf**：无回波点产生 inf，直接喂给 LIO 会破坏 ikd-tree 发散（07 章的 adapter 过滤）。
3. **点云缺 ring/time 字段**：gz 点云没有线号/时间戳，3D LIO 需要补齐（`lidar3d_adapter.py`）。

## 练习

1. 启动深度相机机型，用 `ros2 topic hz` 验证深度图与点云频率，与上表对照。
2. `ros2 topic echo {ns}/pose --once` 找到飞机世界系位置——记住它，后面判分全靠它。
3. 思考：为什么 IMU 频率(253Hz)远高于相机(23.6Hz)？这对 VIO 意味着什么？（08 章揭晓）

[上一章](course/02-offboard.md) · [下一章：航点任务与场景画廊 →](course/04-waypoints.md)
