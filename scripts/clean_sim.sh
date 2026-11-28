#!/usr/bin/env bash
# 清理所有仿真相关进程（安全：不会误杀自己）
#
#   bash scripts/clean_sim.sh          # 清理并报告结果
#   bash scripts/clean_sim.sh --quiet  # 静默（供其他脚本调用）
#
# ⚠️ 教训（本项目真实踩过）:
#   1. 用 `pkill -x gz` 清理**无效** —— 真正的进程是 `ruby .../bin/gz sim ...`,
#      精确名匹配不到，残留实例会继续占着话题/端口，导致新测试录到旧世界的画面。
#   2. 模式里必须用括号写法（如 `gz si[m]`）: 这样正则匹配 "gz sim",
#      而命令行里出现的是字面量 `gz si[m]`, 不会匹配到自己 → 不会自杀。
#   3. 之前多次"同一命令里既有 pkill 又有程序名"导致把自己杀掉，务必分开写。
set -u
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1
say() { [ "$QUIET" -eq 1 ] || printf '%s\n' "$*"; }

# 注意: pgrep -c 无匹配时仍会打印 0 并返回非零, 不能用 `|| echo 0` (会得到 "0\n0")
count() { local n; n=$(pgrep -cf "$1" 2>/dev/null); echo "${n:-0}"; }
# 精确进程名匹配, 避免 -f 匹配到命令行文本(如本脚本自身/调用者)造成误报
count_exact() { local n; n=$(pgrep -xc "$1" 2>/dev/null); echo "${n:-0}"; }

before_gz=$(count "gz si[m]")
before_px4=$(count "px4_sitl_default/bin/p[x]4")
say "清理前: gz=$before_gz px4=$before_px4"

# 1) gz 仿真（ruby 启动器 / 服务端 / GUI）
pkill -f "gz si[m]"            2>/dev/null
pkill -f "gz-sim-serve[r]"     2>/dev/null
pkill -f "gz-sim-gu[i]"        2>/dev/null
sleep 1
pkill -9 -f "gz si[m]"         2>/dev/null
pkill -9 -f "gz-sim-serve[r]"  2>/dev/null

# 2) PX4 SITL
pkill -f "px4_sitl_default/bin/p[x]4" 2>/dev/null
sleep 1
pkill -9 -f "px4_sitl_default/bin/p[x]4" 2>/dev/null

# 3) DDS Agent
pkill -x MicroXRCEAgent 2>/dev/null
sleep 1
pkill -9 -x MicroXRCEAgent 2>/dev/null

# 4) 我们自己的节点 / launch / 桥接
for pat in "ros2 launc[h]" "parameter_bridg[e]" "waypoint_missio[n]" \
           "offboard_dem[o]" "planner_reactiv[e]" "swarm_agen[t]" \
           "formation_followe[r]" "aruco_precision_landin[g]" "bt_missio[n]" \
           "safety_superviso[r]" "vtol_dem[o]" "yolo_detecto[r]" \
           "lidar3d_adapte[r]" "odom_to_t[f]" "point_lio_nod[e]"; do
  pkill -f "$pat" 2>/dev/null
done
sleep 2

after_gz=$(count "gz si[m]")
after_px4=$(count "px4_sitl_default/bin/p[x]4")
after_agent=$(count_exact "MicroXRCEAgent")
say "清理后: gz=$after_gz px4=$after_px4 agent=$after_agent"

if [ "$after_gz" -eq 0 ] && [ "$after_px4" -eq 0 ] && [ "$after_agent" -eq 0 ]; then
  say "✅ 无残留"
  exit 0
else
  say "⚠️ 仍有残留，建议人工检查:"
  pgrep -af "gz si[m]" | head -5
  exit 1
fi
