#!/usr/bin/env bash
# 可执行判定入口 (学生/课程 agent 直接用这个, 不用关心环境)
#
#   bash scripts/grade.sh --list
#   bash scripts/grade.sh --scenario ctrl.takeoff_hover_land --runs 3
#   bash scripts/grade.sh --scenario ctrl.waypoint_square --runs 3 --md report.md
#
# 说明: 必须用系统 python (/usr/bin/python3), conda 的 libstdc++ 版本过低会加载不了 rclpy
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"

# MicroXRCEAgent 安装在 ~/.local/bin (用户级 pip), 缺了它仿真栈会 exit 127
export PATH="$HOME/.local/bin:/usr/bin:$PATH"
# MicroXRCEAgent 动态库在 ~/.local/lib (缺了会报 libmicroxrcedds_agent.so.3.0 not found)
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
mkdir -p "$ROOT/logs/grade"
exec /usr/bin/python3 -u "$ROOT/tools/grade.py" "$@"   # -u: 无缓冲便于看进度
