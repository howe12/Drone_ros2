#!/usr/bin/env bash
# 让 Gazebo Fuel 模型可离线加载（把 SDF 里的 https 绝对 URI 改成相对本地路径）
#
#   bash scripts/fix_fuel_offline.sh              # 处理全部已缓存的 Fuel 模型
#   bash scripts/fix_fuel_offline.sh baylands     # 只处理名字包含 baylands 的模型
#
# 背景:
#   从 Fuel 下载的模型会缓存在 ~/.gz/fuel/，但它的 model.sdf 里网格与贴图 URI 是
#   **https 绝对地址**（如 https://fuel.gazebosim.org/1.0/.../files/media/x.DAE）。
#   于是每次加载都要联网；在网络受限时贴图/网格取不到，模型就会渲染成**纯黑剪影**
#   （本项目的 baylands 世界里的树就是这种情况）。
#   把 URI 改成相对路径后，gz 直接用缓存里的本地文件，无需联网。
#
# 安全性: 每个 model.sdf 改动前备份为 model.sdf.bak；已是相对路径的会跳过（幂等）。
set -u
FUEL_ROOT="${GZ_FUEL_CACHE:-$HOME/.gz/fuel/fuel.gazebosim.org}"
FILTER="${1:-}"

if [ ! -d "$FUEL_ROOT" ]; then
  echo "✗ 未找到 Fuel 缓存目录: $FUEL_ROOT" >&2
  echo "  先跑一次用到 Fuel 世界的场景（如 baylands），让 gz 自动下载。" >&2
  exit 1
fi

echo "== Fuel 缓存: $FUEL_ROOT"
[ -n "$FILTER" ] && echo "== 过滤: 只处理名字含 '$FILTER' 的模型"

changed=0; skipped=0; total=0
while IFS= read -r sdf; do
  total=$((total+1))
  dir=$(dirname "$sdf")
  name=$(basename "$(dirname "$dir")")          # .../models/<name>/<version>/model.sdf
  if [ -n "$FILTER" ] && ! echo "$name" | grep -qi "$FILTER"; then
    skipped=$((skipped+1)); continue
  fi
  if ! grep -q "https://fuel.gazebosim.org" "$sdf" 2>/dev/null; then
    skipped=$((skipped+1)); continue
  fi
  n_before=$(grep -c "https://fuel.gazebosim.org" "$sdf")
  [ -f "$sdf.bak" ] || cp "$sdf" "$sdf.bak"
  # 把 "https://fuel.gazebosim.org/1.0/<owner>/models/<name>/<ver>/files/" 前缀去掉,
  # 使 URI 变为相对模型目录的路径 (media/xxx.DAE)
  sed -i -E 's#https://fuel\.gazebosim\.org/1\.0/[^/]+/models/[^/]+/[0-9]+/files/##g' "$sdf"
  n_after=$(grep -c "https://fuel.gazebosim.org" "$sdf")
  echo "  ✓ $name: 远程 URI $n_before -> $n_after (已改相对路径, 备份 $sdf.bak)"
  changed=$((changed+1))
done < <(find "$FUEL_ROOT" -name model.sdf -not -name "*.bak" 2>/dev/null)

echo
echo "扫描 $total 个模型, 修改 $changed 个, 跳过 $skipped 个"
if [ "$changed" -eq 0 ]; then
  echo "（没有需要修改的：可能已是相对路径，或该目录下没有缓存的 Fuel 模型）"
fi
echo
echo "提示: 树模型即便改成本地路径仍可能是黑色 —— 那是 DAE 内部材质问题，"
echo "      与 URI 无关；需要换带贴图的资产或修网格材质。"
