#!/usr/bin/env bash
# 录制 YOLO 检测视频（必须等飞机真正升空后才开录）
#
#   bash scripts/record_yolo_video.sh [机型] [世界] [模型文件] [类别]
#
# 例:
#   # COCO 80 类, 专用目标世界(人/车)
#   bash scripts/record_yolo_video.sh gz_x500_mono_cam yolo_targets
#   # 开放词表, 在 baylands 里检测树木
#   bash scripts/record_yolo_video.sh gz_x500_mono_cam baylands yolov8s-worldv2.pt "tree, person, car"
#
# 产出: videos/vision.yolo_detect.mp4 + .gif
#
# ⚠️ 教训:
#   1. 之前不等飞机起飞就开录 -> 录到的是**地面静止画面**(帧 5 与帧 105 完全相同),
#      看起来像"检测成功"其实飞机没动。必须等 armed && 高度>2m。
#   2. 不先清场会连到上一轮实例 -> 录到别的世界。用 clean_sim.sh。
#   3. 脚本要 source ROS(set -u 需临时关闭), 否则就绪/状态检测永远失败。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
MODEL_AC="${1:-gz_x500_mono_cam}"
WORLD="${2:-yolo_targets}"
YOLO_MODEL="${3:-}"
CLASSES="${4:-}"
SECS="${YOLO_REC_SECONDS:-40}"
NS="/${MODEL_AC#gz_}_0"          # gz_x500_mono_cam -> /x500_mono_cam_0
export DISPLAY="${DISPLAY:-:0}"
PY=/usr/bin/python3
mkdir -p videos logs/grade

set +u; source /opt/ros/humble/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export PATH="$HOME/.local/bin:/usr/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

bash scripts/clean_sim.sh --quiet

echo "== 启动仿真栈 ($MODEL_AC / $WORLD)"
env DISPLAY="$DISPLAY" bash scripts/run_simulation.sh xtd2 "$MODEL_AC" "$WORLD" \
    > logs/grade/yolo_stack.log 2>&1 &
STACK=$!

ready=0
for _ in $(seq 1 40); do
  sleep 5
  if timeout 5 ros2 topic list 2>/dev/null | grep -q "fmu/out/vehicle_status"; then ready=1; break; fi
done
if [ "$ready" -ne 1 ]; then
  echo "  ✗ 仿真栈未就绪"; bash scripts/clean_sim.sh --quiet; exit 1
fi
echo "  PX4 就绪"

echo "== 启动 YOLO 检测"
bash scripts/run_yolo.sh "$NS/camera/image_raw" "$YOLO_MODEL" "$CLASSES" \
    > logs/grade/yolo_node.log 2>&1 &
YOLO=$!
sleep 10
grep -aE "检测启动|开放词表" logs/grade/yolo_node.log | head -2 | cut -c1-130

echo "== 起飞并沿 +x 直线掠过目标 (3.5m)"
# 用专门航线: 从 -4m 飞到 +16m, 依次经过 行人(x=5) 与 车(x=8/14),
# 目标全程在正前方 -> 不会像绕圈那样只在部分时段入镜
timeout 150 ros2 launch drone_sim waypoint_mission.launch.py \
    prefix:="$NS" mission:=yolo_pass speed:=1.2 \
    > logs/grade/yolo_flight.log 2>&1 &
FLIGHT=$!

# ★ 关键: 等飞机真的升空(已解锁 且 高度>2m), 否则录到的是地面静止画面
echo "== 等待升空 (armed + 高度>2m) ..."
airborne=0
for _ in $(seq 1 45); do
  sleep 2
  st=$(timeout 6 ros2 topic echo "$NS/fmu/out/vehicle_local_position" --once 2>/dev/null)
  z=$(echo "$st" | awk '/^z:/ {print $2; exit}')
  arm=$(timeout 6 ros2 topic echo "$NS/fmu/out/vehicle_status" --once 2>/dev/null | awk '/^arming_state:/ {print $2; exit}')
  if [ -n "${z:-}" ] && [ -n "${arm:-}" ]; then
    alt=$(awk -v z="$z" 'BEGIN{printf "%.2f", -z}')
    if [ "$arm" = "2" ] && awk -v a="$alt" 'BEGIN{exit !(a>2.0)}'; then
      echo "  ✓ 已升空: 高度 ${alt}m, armed=$arm"; airborne=1; break
    fi
  fi
done
if [ "$airborne" -ne 1 ]; then
  echo "  ✗ 飞机未升空, 放弃录制 (不录地面静止画面)"
  tail -3 logs/grade/yolo_flight.log | cut -c1-110
  kill -INT "$FLIGHT" 2>/dev/null; kill -INT "$YOLO" 2>/dev/null
  bash scripts/clean_sim.sh --quiet; exit 2
fi

echo "== 录制检测结果 ($SECS s)"
$PY tools/record_image_topic.py --topic "/drone_sim_yolo_detector/annotated" \
    --out videos/vision.yolo_detect.mp4 --seconds "$SECS" 2>&1 | tail -3

kill -INT "$FLIGHT" 2>/dev/null; kill -INT "$YOLO" 2>/dev/null
bash scripts/clean_sim.sh --quiet

if [ -s videos/vision.yolo_detect.mp4 ]; then
  bash scripts/make_gif.sh videos/vision.yolo_detect.mp4 videos/vision.yolo_detect.gif
else
  echo "  ✗ 未生成视频"; exit 1
fi
