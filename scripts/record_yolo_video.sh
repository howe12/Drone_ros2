#!/usr/bin/env bash
# 录制 YOLO 检测结果视频
#
# YOLO 的检测框画在 /drone_sim_yolo_detector/annotated 上，不在 Gazebo 窗口里，
# 所以必须直接录这个图像话题（用 tools/record_image_topic.py）。
#
#   bash scripts/record_yolo_video.sh [机型] [世界]
#
# 产出:
#   videos/vision.yolo_detect.mp4 + .gif
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
MODEL="${1:-gz_x500_mono_cam}"
WORLD="${2:-baylands}"
TOPIC="/drone_sim_yolo_detector/annotated"
SECS="${YOLO_REC_SECONDS:-30}"
export DISPLAY="${DISPLAY:-:0}"
# 必须 source ROS 与工作空间: 否则看不到 px4_msgs 等话题, 就绪检测会永远失败(踩过)
# 注意: ROS 的 setup.bash 引用了未定义变量, 与 set -u 冲突, 需临时关闭
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export PATH="$HOME/.local/bin:/usr/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
PY=/usr/bin/python3
mkdir -p videos logs/grade

echo "== 启动仿真栈 ($MODEL / $WORLD)"
env DISPLAY="$DISPLAY" bash scripts/run_simulation.sh xtd2 "$MODEL" "$WORLD" \
    > logs/grade/yolo_stack.log 2>&1 &
STACK=$!

ready=0
for _ in $(seq 1 36); do
  sleep 5
  if timeout 5 ros2 topic list 2>/dev/null | grep -q "fmu/out/vehicle_status"; then ready=1; break; fi
done
if [ "$ready" -ne 1 ]; then
  echo "  ✗ 仿真栈未就绪，退出"; kill -INT "$STACK" 2>/dev/null; exit 1
fi
echo "  PX4 就绪"

echo "== 启动 YOLO 检测"
bash scripts/run_yolo.sh "/x500_mono_cam_0/camera/image_raw" \
    > logs/grade/yolo_node.log 2>&1 &
YOLO=$!
sleep 8

# 让飞机飞起来，画面有变化（否则录到的是静止地面）
timeout 60 ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_mono_cam_0 \
    -p takeoff_altitude:=3.0 -p circle_duration:=20.0 > logs/grade/yolo_flight.log 2>&1 &
FLIGHT=$!

echo "== 录制 $TOPIC ($SECS s)"
$PY tools/record_image_topic.py --topic "$TOPIC" --out videos/vision.yolo_detect.mp4 \
    --seconds "$SECS" 2>&1 | tail -3

# 收尾
kill -INT "$FLIGHT" 2>/dev/null; kill -INT "$YOLO" 2>/dev/null
kill -INT "$STACK" 2>/dev/null
sleep 4
pkill -9 -x gz 2>/dev/null; pkill -9 -x MicroXRCEAgent 2>/dev/null

if [ -s videos/vision.yolo_detect.mp4 ]; then
  bash scripts/make_gif.sh videos/vision.yolo_detect.mp4 videos/vision.yolo_detect.gif
else
  echo "  ✗ 未生成视频"
  exit 1
fi
