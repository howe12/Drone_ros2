#!/usr/bin/env bash
# 录屏 + 判定（只录 Gazebo 仿真窗口，不录桌面）
#
#   bash scripts/grade_video.sh vision.aruco_precision_landing
#
# 产出:
#   videos/<scenario>.mp4       仿真画面 (原速)
#   videos/<scenario>.gif       页面内可直接播放的压缩预览 (时间压缩, 便于 GitHub README 内联播放)
#   logs/grade/<scenario>.out   判定输出
#   logs/grade/<scenario>.md    判定报告
#
# 关键教训:
#   1. 直接录全屏会把浏览器/终端一起录进去 -> 必须只录 Gazebo 窗口
#      (曾生成过一段全是 GitHub 页面的"仿真视频")
#   2. X11 对被遮挡窗口抓图会报 BadMatch -> 录之前先置顶 (tools/raise_gz_window.py)
#   3. 之前用 timeout 包装 ffmpeg, kill 打到 timeout 而非 ffmpeg, wait 会卡死 -> 直接起 ffmpeg
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
mkdir -p videos logs/grade
export DISPLAY="${DISPLAY:-:0}"
PY=/usr/bin/python3

bash scripts/clean_sim.sh --quiet    # 每次录制前清场, 避免连到上一轮的实例

for sid in "$@"; do
  echo "=========== $sid ==========="
  VID="videos/${sid}.mp4"
  OUT="logs/grade/${sid}.out"

  # 1) 后台开始判定
  timeout 850 bash scripts/grade.sh --scenario "$sid" --runs 1 \
      --md "logs/grade/${sid}.md" > "$OUT" 2>&1 &
  GRADE_PID=$!

  # 2) 等 Gazebo 窗口出现并置顶 (最多 180s)
  WINFO=$($PY tools/raise_gz_window.py --wait 300 2>/dev/null)
  WID=$(echo "$WINFO" | awk '{print $1}')

  if [ -z "${WID:-}" ]; then
    echo "  ⚠ 未找到 Gazebo 窗口, 本次不录屏 (删除可能存在的旧视频, 避免张冠李戴)"
    rm -f "$VID" "videos/${sid}.gif"
    wait "$GRADE_PID"
    REC_OK=0
  else
    # 2.5) 尽力让镜头跟随被测无人机 (服务不存在则忽略, 不影响录制)
    NS=$(grep -aE "^  namespaces:" -A1 "catalog/scenarios.yaml" >/dev/null 2>&1; \
         $PY - "$sid" <<'PYEOF2' 2>/dev/null
import sys, yaml
sid = sys.argv[1]
for s in yaml.safe_load(open('catalog/scenarios.yaml'))['scenarios']:
    if s['id'] == sid:
        ns = (s.get('namespaces') or ['/x500_0'])[0]
        print(ns.strip('/'))
        break
PYEOF2
)
    if [ -n "${NS:-}" ]; then
      for _ in 1 2 3 4 5 6 7 8 9 10; do
        if timeout 5 gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
             --reptype gz.msgs.Boolean --timeout 2000 --req "data: \"$NS\"" >/dev/null 2>&1; then
          echo "  镜头跟随: $NS"; break
        fi
        sleep 2
      done
    fi

    # 3) 只录该窗口
    ffmpeg -loglevel error -nostdin -y -f x11grab -window_id "$WID" -framerate 8 \
           -i "$DISPLAY" -pix_fmt yuv420p -c:v libx264 -preset ultrafast -crf 30 \
           "$VID" > /dev/null 2>&1 &
    REC_PID=$!

    # 4) 等判定结束
    wait "$GRADE_PID"

    # 5) 停录: SIGINT 正常收尾, 3s 未退则强杀
    kill -INT "$REC_PID" 2>/dev/null
    for _ in 1 2 3 4 5 6; do
      kill -0 "$REC_PID" 2>/dev/null || break
      sleep 0.5
    done
    kill -9 "$REC_PID" 2>/dev/null
    wait "$REC_PID" 2>/dev/null
    REC_OK=1
  fi

  verdict=$(grep -aE "^结论:" "$OUT" | tail -1 | sed 's/^结论: //')
  metrics=$(grep -aE "^  指标:" "$OUT" | tail -1 | sed 's/^  指标: //')
  size=$(du -h "$VID" 2>/dev/null | cut -f1)
  echo "  结果: ${verdict:-NO RESULT}"
  echo "  指标: ${metrics:--}"
  echo "  视频: $VID (${size:-无})"

  # 6) 生成 GIF 预览 (压缩到约 15s, GitHub README 可内联播放)
  if [ "${REC_OK:-0}" = "1" ] && [ -f "$VID" ] && [ -s "$VID" ]; then
    d=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$VID" 2>/dev/null | cut -d. -f1)
    factor=$($PY -c "print(round(max(1.0, ${d:-60}/15.0), 2))")
    ffmpeg -loglevel error -y -i "$VID" \
      -vf "setpts=PTS/${factor},fps=8,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer" \
      -loop 0 "videos/${sid}.gif" 2>/dev/null
    echo "  GIF:  videos/${sid}.gif ($(du -h "videos/${sid}.gif" 2>/dev/null | cut -f1), 快放 ${factor}x)"
  fi
  sleep 3
done
