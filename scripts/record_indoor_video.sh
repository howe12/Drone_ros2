#!/usr/bin/env bash
# 录制室内飞行视频（带**边界自检**，不安全就拒绝起飞）
#
#   bash scripts/record_indoor_video.sh [世界] [机型]
#   环境变量:
#     SPAWN_X/Y/Z      出生点（默认 0 0 0.3，大厅中心）
#     CIRCLE_RADIUS    画圆半径（默认 2.0）
#     TAKEOFF_ALT      起飞高度（默认 2.5）
#     CIRCLE_DURATION  画圆时长（默认 30）
#     HALL_HALF        大厅半边长（默认 10.0，对应 20x20）
#     CEIL             天花板高度（默认 4.6）
#     SECONDS_CAP      录像最长秒数（默认 70）
#
# 产出: videos/indoor_hall.flight.mp4 + .gif
#
# 为什么要有边界自检:
#   offboard_demo 的**画圆圆心 = 起飞点**。早先我用出生点 (-4,-9) + 半径 2.0 m，
#   圆周就会到 y=-11，而南墙内表面在 y=-9.85 —— 实测**撞墙**了。
#   这类错误靠肉眼很难提前发现，所以做成硬性前置检查（退出码 3）。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
WORLD="${1:-indoor_hall}"
MODEL="${2:-gz_x500_mono_cam}"

SX="${SPAWN_X:-0}"; SY="${SPAWN_Y:-0}"; SZ="${SPAWN_Z:-0.3}"
R="${CIRCLE_RADIUS:-2.0}"; ALT="${TAKEOFF_ALT:-2.5}"; DUR="${CIRCLE_DURATION:-30}"
HALL_HALF="${HALL_HALF:-10.0}"; CEIL="${CEIL:-4.6}"
CAP="${SECONDS_CAP:-70}"
WALL_T=0.3
export DISPLAY="${DISPLAY:-:0}"
mkdir -p videos logs/grade

# ---------- 边界自检 ----------
/usr/bin/python3 - "$SX" "$SY" "$R" "$ALT" "$HALL_HALF" "$CEIL" "$WALL_T" <<'PY'
import sys, math
sx, sy, r, alt, half, ceil, wall = (float(x) for x in sys.argv[1:8])
inner = half - wall/2.0
xmax, ymax = abs(sx) + r, abs(sy) + r
ok = True
print(f'大厅内表面: x,y ∈ ±{inner:.2f} m ; 天花板 {ceil:.2f} m')
print(f'圆周外包:   x ≤ {xmax:.2f} m, y ≤ {ymax:.2f} m ; 起飞高度 {alt:.2f} m')
if xmax > inner:
    print(f'✗ 圆周在 x 方向越墙 {xmax-inner:.2f} m'); ok = False
if ymax > inner:
    print(f'✗ 圆周在 y 方向越墙 {ymax-inner:.2f} m'); ok = False
if alt > ceil - 0.6:
    print(f'✗ 起飞高度距天花板不足 0.6 m (余 {ceil-alt:.2f} m)'); ok = False
if ok:
    print(f'✓ 边界自检通过 (距墙 ≥ {min(inner-xmax, inner-ymax):.2f} m)')
sys.exit(0 if ok else 3)
PY
check=$?
if [ "$check" -ne 0 ]; then
  echo
  echo "✗ 边界自检未通过，拒绝起飞（避免撞墙/撞顶）。"
  echo "  调整示例: SPAWN_X=0 SPAWN_Y=0 CIRCLE_RADIUS=2.0 bash scripts/record_indoor_video.sh"
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export PATH="$HOME/.local/bin:/usr/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"

bash scripts/clean_sim.sh --quiet

echo "== 启动仿真栈 ($MODEL / $WORLD), 出生点 ($SX, $SY, $SZ)"
env DISPLAY="$DISPLAY" SPAWN_X="$SX" SPAWN_Y="$SY" SPAWN_Z="$SZ" \
  bash scripts/run_simulation.sh xtd2 "$MODEL" "$WORLD" > logs/grade/indoor_stack.log 2>&1 &
for _ in $(seq 1 40); do
  sleep 5
  timeout 5 ros2 topic list 2>/dev/null | grep -q "fmu/out/vehicle_status" && break
done
timeout 5 ros2 topic list 2>/dev/null | grep -q "fmu/out/vehicle_status" || {
  echo "  ✗ 仿真栈未就绪"; bash scripts/clean_sim.sh --quiet; exit 1; }
echo "  PX4 就绪"

# 把 Gazebo 窗口抬到前面（否则 x11grab 抓不到/抓错窗口）
WID=$(/usr/bin/python3 tools/raise_gz_window.py --wait 40 2>/dev/null | awk '{print $1}')
if [ -z "${WID:-}" ]; then
  echo "  ✗ 未找到 Gazebo 窗口"; bash scripts/clean_sim.sh --quiet; exit 1
fi
echo "  录制窗口: $WID"

echo "== 起飞 ${ALT}m + 画圆 ${DUR}s（半径 ${R}m）"
NS="/${MODEL#gz_}_0"
# 参数必须传成浮点字面量: 传 "30" 会被 ROS CLI 推断为 INTEGER,
# 而节点声明的是 DOUBLE, 会抛 InvalidParameterTypeException 直接崩溃(踩过两次)
ALT_F=$(awk -v v="$ALT" 'BEGIN{printf "%.2f", v}')
R_F=$(awk -v v="$R" 'BEGIN{printf "%.2f", v}')
DUR_F=$(awk -v v="$DUR" 'BEGIN{printf "%.1f", v}')
timeout $((DUR + 120)) ros2 run drone_sim offboard_demo --ros-args \
  -p px4_ns:="$NS" -p takeoff_altitude:="$ALT_F" \
  -p circle_radius:="$R_F" -p circle_speed:=0.25 \
  -p circle_duration:="$DUR_F" \
  > logs/grade/indoor_flight.log 2>&1 &
FLIGHT=$!

# 等真正升空再开录（避免录到地面静止画面）
echo "== 等待升空 ..."
airborne=0
for _ in $(seq 1 40); do
  sleep 2
  z=$(timeout 6 ros2 topic echo "$NS/fmu/out/vehicle_local_position" --once 2>/dev/null | awk '/^z:/{print $2;exit}')
  arm=$(timeout 6 ros2 topic echo "$NS/fmu/out/vehicle_status" --once 2>/dev/null | awk '/^arming_state:/{print $2;exit}')
  if [ -n "${z:-}" ] && [ "${arm:-}" = "2" ]; then
    alt_now=$(awk -v z="$z" 'BEGIN{printf "%.2f", -z}')
    awk -v a="$alt_now" -v t="$ALT" 'BEGIN{exit !(a > t*0.7)}' && {
      echo "  ✓ 已升空 ${alt_now}m"; airborne=1; break; }
  fi
done
if [ "$airborne" -ne 1 ]; then
  echo "  ✗ 未升空, 放弃录制"
  kill -INT "$FLIGHT" 2>/dev/null || true
  bash scripts/clean_sim.sh --quiet; exit 2
fi

# 让 GUI 相机跟随无人机（否则固定机位下飞机在画面里很小/出画）
for _ in $(seq 1 10); do
  if timeout 5 gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
       --reptype gz.msgs.Boolean --timeout 2000 --req "data: \"${NS#/}\"" >/dev/null 2>&1; then
    echo "  镜头跟随: ${NS#/}"; break
  fi
  sleep 2
done

echo "== 录制（窗口 $WID, 上限 ${CAP}s）"
# 与 grade_video.sh 一致的抓窗参数: -window_id 后接 -i $DISPLAY, 不要 -video_size
ffmpeg -loglevel error -nostdin -y -f x11grab -window_id "$WID" -framerate 12 \
  -i "$DISPLAY" -pix_fmt yuv420p -c:v libx264 -preset ultrafast -crf 28 \
  "$ROOT/videos/indoor_hall.flight.mp4" > /dev/null 2>&1 &
FF=$!
sleep $((DUR + 12))
kill -INT "$FF" 2>/dev/null || true
wait "$FF" 2>/dev/null || true
kill -INT "$FLIGHT" 2>/dev/null || true
bash scripts/clean_sim.sh --quiet

if [ -s videos/indoor_hall.flight.mp4 ]; then
  bash scripts/make_gif.sh videos/indoor_hall.flight.mp4 videos/indoor_hall.flight.gif
else
  echo "  ✗ 未生成视频"; exit 1
fi
