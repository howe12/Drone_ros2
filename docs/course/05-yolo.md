# 05 YOLO 目标检测

> 难度 L2 · 对应实验 E8（检测部分）· 场景 `vision.yolo_detect`

## 学习目标

- 把机载相机图像接进 YOLOv8，跑通实时检测
- 理解"系统 Python vs conda Python"的 ROS2 混合环境坑（本章最大教学点）

## 5.1 跑起来

```bash
# 栈: 前置相机机型 + 户外纹理世界
bash scripts/run_simulation.sh xtd2 gz_x500_mono_cam baylands

# 检测节点（新终端）:
bash scripts/run_yolo.sh /x500_mono_cam_0/camera/image_raw
```

![YOLO 检测](images/18_e8_yolo_test.jpg)

样本图实测：`bus: 0.87`、`person ×3`（yolov8n 权重已入包，无需下载）。

### 判分指标

| 指标 | 阈值 | 取数 |
|---|---|---|
| 启动成功 | ≥1 | 日志含"YOLO 检测启动" |
| 无异常退出 | ≥1 | 日志无 Traceback / ModuleNotFound |

## 5.2 为什么用 run_yolo.sh 包装（真实工程坑）

ROS2 Humble 的 rclpy 装在**系统 Python 3.10**；而 ultralytics/torch 装在 **conda 环境**。
直接 `ros2 run` 会 `ModuleNotFound`，直接用 conda python 又找不到 rclpy。

课程的解法（`scripts/run_yolo.sh`）：**conda python + LD_PRELOAD 系统库**，两边兼得。
这是 ROS2 + 深度学习混合栈的通用模式，值得记住：

```
rclpy(系统python) ←→ 推理(conda python)   # 用包装脚本桥接, 不硬改环境
```

## 5.3 检测节点结构

`drone_sim/yolo_detector.py`：订阅 Image → cv_bridge 转 cv2 → YOLOv8 推理 → 发布检测框。
权重在包内 `config/`，离线可跑。

## 故障实验室

- `ModuleNotFound: ultralytics` → 没用 run_yolo.sh 包装
- 图像话题收不到 → 检查机型是否带相机、命名空间是否匹配（03 章 QoS 坑）

## 练习

1. 把相机对准 worlds 里的车辆/人形模型，记录置信度变化。
2. 思考：检测框如何变成"飞向目标"的控制量？（下一章 ArUco 伺服就是完整答案）

[上一章](course/04-waypoints.md) · [下一章：ArUco 精准降落 →](course/06-precision-landing.md)
