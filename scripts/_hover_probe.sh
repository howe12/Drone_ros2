#!/usr/bin/env bash
# 对照实验: 原地悬停 20s, 录原始相机画面
# 目的: 判断飞行画面顶部两块"暗色长条"到底是机体自身部件还是场景物体
#   - 若悬停时暗条仍在(无人机不移动), 且随航向固定 -> 机体/渲染问题
#   - 若悬停时暗条消失 -> 是场景里的物体(如远处的黑色树)
set -u
ROOT=/develop/IDEA/Drone_ros2
cd "$ROOT"
set +u; source /opt/ros/humble/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export DISPLAY=:0
export PATH="$HOME/.local/bin:/usr/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

bash scripts/clean_sim.sh --quiet

bash scripts/run_simulation.sh xtd2 gz_x500_mono_cam yolo_targets > logs/grade/hover_stack.log 2>&1 &
for _ in $(seq 1 30); do
  sleep 5
  timeout 5 ros2 topic list 2>/dev/null | grep -q "fmu/out/vehicle_status" && break
done

# 几乎不动的"悬停"（半径 5cm）
timeout 120 ros2 run drone_sim offboard_demo --ros-args \
  -p px4_ns:=/x500_mono_cam_0 -p takeoff_altitude:=4.0 \
  -p circle_radius:=0.05 -p circle_speed:=0.5 -p circle_duration:=35.0 \
  > logs/grade/hover_flight.log 2>&1 &

for _ in $(seq 1 30); do
  sleep 2
  z=$(timeout 6 ros2 topic echo /x500_mono_cam_0/fmu/out/vehicle_local_position --once 2>/dev/null | awk '/^z:/ {print $2; exit}')
  if [ -n "${z:-}" ]; then
    alt=$(awk -v z="$z" 'BEGIN{printf "%.2f", -z}')
    awk -v a="$alt" 'BEGIN{exit !(a>2.5)}' && { echo "  已升空 ${alt}m, 开始录"; break; }
  fi
done

/usr/bin/python3 tools/record_image_topic.py --topic /x500_mono_cam_0/camera/image_raw \
  --out /tmp/hover.mp4 --seconds 20 2>&1 | tail -2

bash scripts/clean_sim.sh --quiet
