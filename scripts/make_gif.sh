#!/usr/bin/env bash
# MP4 -> GIF（时间压缩到约 15s，供 GitHub README 内联播放）
#   bash scripts/make_gif.sh videos/xxx.mp4 videos/xxx.gif [目标秒数]
set -u
IN="${1:?用法: make_gif.sh <in.mp4> <out.gif> [秒数]}"
OUT="${2:?缺少输出路径}"
TARGET="${3:-15}"
PY=/usr/bin/python3
d=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$IN" 2>/dev/null | cut -d. -f1)
factor=$($PY -c "print(round(max(1.0, ${d:-60}/${TARGET}), 2))")
ffmpeg -loglevel error -y -i "$IN" \
  -vf "setpts=PTS/${factor},fps=8,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer" \
  -loop 0 "$OUT"
echo "  GIF: $OUT ($(du -h "$OUT" | cut -f1), 快放 ${factor}x, 源 ${d}s)"
