#!/usr/bin/env bash
# 从 Gazebo Fuel 下载目标检测用的模型（人/车）
#
#   bash scripts/fetch_fuel_models.sh
#
# 为什么单独下载而不入库:
#   这两个模型合计约 38MB（person_standing 28MB + yolo_car 9.8MB），
#   放进 patch 会把仓库撑大，所以改为按需下载 + 校验。
#
# 下载后会在 third_party/XTDrone2/xtd2_gz_sim/models/ 下生成:
#   person_standing/   (OpenRobotics/Standing person, CC0)
#   yolo_car/          (GoogleResearch/CITY_TAXI_POLICE_CAR)
#
# 配套世界: xtd2_gz_sim/worlds/yolo_targets.sdf (补丁 0019)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODELS="$ROOT/third_party/XTDrone2/xtd2_gz_sim/models"
TMP="${TMPDIR:-$ROOT/logs}/fuel_dl"
mkdir -p "$TMP" "$MODELS"

# Fuel 在国内直连可能失败，可用代理: PROXY=http://127.0.0.1:7897 bash scripts/fetch_fuel_models.sh
PROXY="${PROXY:-}"
CURL=(curl -sL --max-time 300)
if [ -n "$PROXY" ]; then
  CURL+=(-x "$PROXY")
  echo "== 使用代理: $PROXY"
fi

fetch() {   # $1=owner  $2=model(可含空格, 用%20)  $3=version  $4=目标目录名
  local owner="$1" name="$2" ver="$3" dest="$4"
  local zip="$TMP/$dest.zip"
  if [ -d "$MODELS/$dest" ]; then
    echo "  ✓ 已存在: $dest"
    return 0
  fi
  echo "== 下载 $owner/$name (v$ver)"
  "${CURL[@]}" -o "$zip" \
    "https://fuel.gazebosim.org/1.0/$owner/models/$name/$ver/$name.zip" || {
      echo "  ✗ 下载失败（网络？可设 PROXY=... 重试）"; return 1; }
  [ -s "$zip" ] || { echo "  ✗ 空文件"; return 1; }
  mkdir -p "$MODELS/$dest"
  unzip -o -q "$zip" -d "$MODELS/$dest" || { echo "  ✗ 解包失败"; return 1; }
  rm -f "$zip"
  # 统一 model.config 里的名字，避免与目录名不一致
  if [ -f "$MODELS/$dest/model.config" ]; then
    python3 - "$MODELS/$dest/model.config" "$dest" <<'PY'
import re, sys
p, name = sys.argv[1], sys.argv[2]
s = open(p).read()
s = re.sub(r'<name>[^<]*</name>', f'<name>{name}</name>', s, count=1)
open(p, 'w').write(s)
PY
  fi
  echo "  ✓ 完成: $dest ($(du -sh "$MODELS/$dest" | cut -f1))"
}

ok=0
fetch OpenRobotics "Standing%20person" 1 person_standing || ok=1
fetch GoogleResearch "CITY_TAXI_POLICE_CAR" 1 yolo_car || ok=1
# 树木: 让画面有自然背景 (COCO 不含 tree 类, 但场景更真实; 配合 YOLO-World 可直接检出)
fetch OpenRobotics "Oak%20tree" 1 oak_tree || ok=1
fetch OpenRobotics "Pine%20Tree" 1 pine_tree || ok=1
# 室内大厅 (indoor_hall 世界) 的家具
fetch OpenRobotics "SquareShelf" 1 SquareShelf || ok=1
fetch OpenRobotics "Desk" 1 Desk || ok=1
fetch OpenRobotics "VisitorChair" 1 VisitorChair || ok=1
fetch OpenRobotics "Cardboard%20box" 1 cardboard_box || ok=1
fetch OpenRobotics "Euro%20pallet" 1 euro_pallet || ok=1
fetch OpenRobotics "Pallet_Rack_Section" 1 pallet_rack_section || ok=1
fetch OpenRobotics "Fridge" 1 Fridge || ok=1

if [ "$ok" -eq 0 ]; then
  echo
  echo "✅ 模型就绪。接着执行:"
  echo "   cd ros2_ws && colcon build --symlink-install --base-paths src ../third_party/XTDrone2 --packages-select xtd2_gz_sim"
  echo "   bash scripts/run_simulation.sh xtd2 gz_x500_mono_cam yolo_targets"
else
  echo "⚠️ 部分模型下载失败"
  exit 1
fi
