#!/usr/bin/env bash
# E8 YOLO 检测节点包装器
# ultralytics/torch 装在 conda 环境, 而 ROS 入口脚本用系统 python;
# conda 的 libstdc++ 缺 GLIBCXX_3.4.30, 需 LD_PRELOAD 系统库。
#
# 用法:
#   bash scripts/run_yolo.sh [image_topic] [model] [classes]
#   bash scripts/run_yolo.sh /x500_mono_cam_0/camera/image_raw
#   bash scripts/run_yolo.sh /x500_mono_cam_0/camera/image_raw yolov8s-worldv2.pt "tree, person, car"
#
# 说明:
#   - model 为空 -> 用包内 yolov8n.pt (COCO 80 类)
#   - model 含 "world" -> 用开放词表模型, 可用 classes 指定任意文本类别(如 tree)
set -u
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
set +u; source "$ROOT/ros2_ws/install/setup.bash"; set -u
TOPIC="${1:-/x500_mono_cam_0/camera/image_raw}"
MODEL="${2:-}"
CLASSES="${3:-}"
CONDA_PY="$HOME/miniconda3/bin/python3"
[ -x "$CONDA_PY" ] || CONDA_PY=python3

ARGS=(-p "image_topic:=$TOPIC")
[ -n "$MODEL" ] && ARGS+=(-p "model:=$ROOT/ros2_ws/src/drone_sim/models/$MODEL")
[ -n "$CLASSES" ] && ARGS+=(-p "classes:=$CLASSES")

LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  "$CONDA_PY" "$ROOT/ros2_ws/install/drone_sim/lib/drone_sim/yolo_detector" \
  --ros-args "${ARGS[@]}"
