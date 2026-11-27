#!/usr/bin/env bash
# 录屏 + 判定: 记录仿真画面并同时跑自动验收
#
#   bash scripts/grade_video.sh vision.aruco_precision_landing
#   bash scripts/grade_video.sh multi.formation swarm.cross_3uav
#
# 产出:
#   videos/<scenario>.mp4    仿真画面 (1280x720, 10fps, 便于快速回看)
#   logs/grade/<scenario>.out 判定输出
#   logs/grade/<scenario>.md  判定报告
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p videos logs/grade
export DISPLAY="${DISPLAY:-:0}"

# 录屏参数: 缩放一半 + 10fps + ultrafast, 兼顾清晰度与 CPU 占用 (仿真本身很吃 CPU)
REC_OPTS=(-f x11grab -framerate 6 -video_size 2560x1440 -i "$DISPLAY"
          -vf scale=1280:-2 -c:v libx264 -preset ultrafast -crf 30 -pix_fmt yuv420p)

record_start() {   # $1=输出文件  $2=最长秒数(保险)
  # 直接后台起 ffmpeg 并记录它自己的 PID。
  # 教训: 之前用 timeout 包装, kill 打到 timeout 而非 ffmpeg, wait 会一直卡住
  ffmpeg -loglevel error -nostdin -y "${REC_OPTS[@]}" "$1" > /dev/null 2>&1 &
  echo $!
}

for sid in "$@"; do
  echo "=========== $sid ==========="
  VID="videos/${sid}.mp4"
  OUT="logs/grade/${sid}.out"

  REC_PID=$(record_start "$VID" 900)
  sleep 1

  timeout 850 bash scripts/grade.sh --scenario "$sid" --runs 1 \
      --md "logs/grade/${sid}.md" > "$OUT" 2>&1
  rc=$?

  # 停止录制: SIGINT 让 ffmpeg 正常写 moov, 3s 未退则强杀
  kill -INT "$REC_PID" 2>/dev/null
  for _ in 1 2 3 4 5 6; do
    kill -0 "$REC_PID" 2>/dev/null || break
    sleep 0.5
  done
  kill -9 "$REC_PID" 2>/dev/null
  wait "$REC_PID" 2>/dev/null
  sleep 1

  verdict=$(grep -aE "^结论:" "$OUT" | tail -1 | sed 's/^结论: //')
  metrics=$(grep -aE "^  指标:" "$OUT" | tail -1 | sed 's/^  指标: //')
  size=$(du -h "$VID" 2>/dev/null | cut -f1)
  echo "  结果: ${verdict:-NO RESULT}"
  echo "  指标: ${metrics:--}"
  echo "  视频: $VID (${size:-无})"
  [ "$rc" -eq 124 ] && echo "  ⚠ 超时"
done
