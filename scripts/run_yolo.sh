#!/usr/bin/env bash
# E8 YOLO 检测节点包装器
# ultralytics/torch 装在 conda 环境, 而 ROS 入口脚本用系统 python;
# conda 的 libstdc++ 缺 GLIBCXX_3.4.30, 需 LD_PRELOAD 系统库。
# 用法: bash scripts/run_yolo.sh [image_topic]
source /opt/ros/humble/setup.bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/ros2_ws/install/setup.bash"
TOPIC="${1:-/x500_mono_cam_0/camera/image_raw}"
CONDA_PY="$HOME/miniconda3/bin/python3"
[ -x "$CONDA_PY" ] || CONDA_PY=python3
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  "$CONDA_PY" "$ROOT/ros2_ws/install/drone_sim/lib/drone_sim/yolo_detector" \
  --ros-args -p image_topic:="$TOPIC"
