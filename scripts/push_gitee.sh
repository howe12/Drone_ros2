#!/usr/bin/env bash
# 推送到 Gitee（凭据不落盘到仓库）
#
# 用法 A：私人令牌（推荐，最快）
#   GITEE_REPO=https://gitee.com/<你的用户名>/Drone_ros2.git \
#   GITEE_TOKEN=<私人令牌> \
#   bash scripts/push_gitee.sh
#
#   或直接给带凭据的 URL（脚本会用完即抹掉）：
#   GITEE_REPO=https://<用户名>:<令牌>@gitee.com/<用户名>/Drone_ros2.git \
#   bash scripts/push_gitee.sh
#
# 用法 B：SSH（需先把公钥加到 Gitee → 设置 → SSH 公钥）
#   GITEE_REPO=git@gitee.com:<用户名>/Drone_ros2.git bash scripts/push_gitee.sh
#
# 说明：
#   - 只会推送到名为 gitee 的 remote，不影响 origin(GitHub)
#   - 若 GITEE_REPO 带令牌，推送结束后会把 remote URL 重置为不含令牌的形式
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

REPO="${GITEE_REPO:-}"
TOKEN="${GITEE_TOKEN:-}"
BRANCH="${GITEE_BRANCH:-main}"
REMOTE="${GITEE_REMOTE:-gitee}"

if [ -z "$REPO" ]; then
  echo "用法: GITEE_REPO=<仓库地址> [GITEE_TOKEN=<令牌>] bash scripts/push_gitee.sh" >&2
  exit 2
fi

# 若给了 TOKEN 但 URL 里没有凭据，则拼进去（仅本次使用）
if [ -n "$TOKEN" ] && ! echo "$REPO" | grep -q '@'; then
  REPO_CLEAN=$(echo "$REPO" | sed -E 's#^https://##')
  PUSH_URL="https://${TOKEN}@${REPO_CLEAN}"
  CLEAN_URL="https://${REPO_CLEAN}"
else
  PUSH_URL="$REPO"
  CLEAN_URL=$(echo "$REPO" | sed -E 's#//[^@]*@#//#')
fi

echo "== 目标: $(echo "$CLEAN_URL")"
echo "== 分支: $BRANCH"

# 配置/更新 remote
if git remote | grep -qx "$REMOTE"; then
  git remote set-url "$REMOTE" "$PUSH_URL"
else
  git remote add "$REMOTE" "$PUSH_URL"
fi

# 推送（网络不稳，最多重试 6 次，代理/直连交替）
ok=0
for i in $(seq 1 6); do
  if [ $((i % 2)) -eq 1 ]; then
    env -u http_proxy -u https_proxy -u all_proxy \
      git -c http.proxy= -c https.proxy= push -u "$REMOTE" "$BRANCH" 2>&1 | tail -3
    [ "${PIPESTATUS[0]}" -eq 0 ] && { ok=1; break; }
  else
    https_proxy=http://127.0.0.1:7897 http_proxy=http://127.0.0.1:7897 \
      git push -u "$REMOTE" "$BRANCH" 2>&1 | tail -3
    [ "${PIPESTATUS[0]}" -eq 0 ] && { ok=1; break; }
  fi
  echo "  第 $i 次失败，20s 后重试"
  sleep 20
done

# 无论成败都抹掉 URL 中的令牌，避免留在 .git/config
git remote set-url "$REMOTE" "$CLEAN_URL"

if [ "$ok" -eq 1 ]; then
  echo "✅ 推送成功: $CLEAN_URL ($BRANCH)"
else
  echo "❌ 推送失败，请检查仓库地址/令牌权限（需 projects 或 write_repository 权限）" >&2
  exit 1
fi
