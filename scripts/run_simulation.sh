#!/usr/bin/env bash
# 一键启动仿真: MicroXRCEAgent + PX4 SITL (Gazebo Classic GUI) + drone_sim 演示
# 用法: bash scripts/run_simulation.sh
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PATH=/usr/bin:$HOME/.local/bin:$PATH
export LD_LIBRARY_PATH=$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

echo "[1/3] 启动 MicroXRCEAgent (UDP 8888)..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 1

echo "[2/3] 启动 PX4 SITL + Gazebo (首次会自动编译)..."
cd "$ROOT/PX4-Autopilot"
make px4_sitl gazebo-classic &
PX4_PID=$!

echo "等待 PX4 就绪 (30s)..."
sleep 30

echo "[3/3] 启动 drone_sim offboard 演示 (起飞->画圆->降落)..."
ros2 launch drone_sim demo.launch.py &
DEMO_PID=$!

cleanup() {
    kill "$DEMO_PID" "$AGENT_PID" 2>/dev/null || true
    # make 会派生 gzserver/gzclient/px4, 一并清理
    pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
}
trap cleanup EXIT
wait
