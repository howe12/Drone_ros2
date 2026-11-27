#!/usr/bin/env bash
# 批量验证所有场景并汇总结果 (串行执行, 避免仿真栈互相冲突)
#
#   bash scripts/grade_all.sh              # 全部场景
#   bash scripts/grade_all.sh plan.obstacle_avoidance ctrl.waypoint_square
#
# 输出:
#   logs/grade/batch.log        总日志
#   logs/grade/summary.md       汇总表 (场景 / 指标 / 结果)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
LOG=logs/grade/batch.log
SUM=logs/grade/summary.md
mkdir -p logs/grade
: > "$LOG"

# 默认顺序: 先测风险最高的(多机/新代码路径), 再测其余
DEFAULT="multi.formation swarm.cross_3uav ctrl.waypoint_square vision.aruco_precision_landing \
slam.lio_odometry mission.behavior_tree safety.geofence vtol.transition vision.yolo_detect vio.vins_mono"

if [ "$#" -gt 0 ]; then SCENARIOS="$*"; else SCENARIOS="$DEFAULT"; fi

echo "# 场景批量判定汇总 ($(date '+%Y-%m-%d %H:%M'))" > "$SUM"
echo "" >> "$SUM"
echo "| 场景 | 结论 | 指标 | 耗时 |" >> "$SUM"
echo "|---|---|---|---|" >> "$SUM"

for sid in $SCENARIOS; do
  echo "=========== $sid ===========" | tee -a "$LOG"
  start=$(date +%s)
  OUT="logs/grade/${sid}.out"
  # 写文件而非命令替换: 残留子进程持有管道会让 $( ) 永久阻塞 (踩过)
  timeout 900 bash scripts/grade.sh --scenario "$sid" --runs 1 \
      --md "logs/grade/${sid}.md" > "$OUT" 2>&1
  rc=$?
  cat "$OUT" >> "$LOG"
  dur=$(( $(date +%s) - start ))
  verdict=$(grep -aE "^结论:" "$OUT" | tail -1 | sed 's/^结论: //')
  metrics=$(grep -aE "^  指标:" "$OUT" | tail -1 | sed 's/^  指标: //')
  [ "$rc" -eq 124 ] && verdict="TIMEOUT (900s)" 
  [ -z "$verdict" ] && verdict="NO RESULT (见 batch.log)"
  [ -z "$metrics" ] && metrics="-"
  echo "| \`$sid\` | $verdict | \`$metrics\` | ${dur}s |" >> "$SUM"
  echo "--- $sid -> $verdict (${dur}s)" | tee -a "$LOG"
done

echo "" >> "$SUM"
echo "完成时间: $(date '+%Y-%m-%d %H:%M')" >> "$SUM"
echo "完毕: $SUM" | tee -a "$LOG"
