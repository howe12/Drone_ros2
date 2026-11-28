#!/usr/bin/env bash
# 一键盘点/修复 Gitee 同步配置（任何新克隆、新环境、新 agent 都跑这一条）
#
#   bash scripts/setup_gitee_sync.sh          # 检查并自动修复
#   bash scripts/setup_gitee_sync.sh --check   # 只检查, 不改动
#
# 做的事:
#   1. 确认 gitee 远端存在（缺失则添加, 地址可用 GITEE_URL 覆盖）
#   2. 确认 origin 的 push URL 列表里同时有 GitHub 和 Gitee（缺失则追加）
#   3. 安装 post-push 钩子（core.hooksPath -> scripts/git-hooks）
#   4. 校验 SSH 到 gitee.com 是否可用（不可用给出明确修复指引）
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

GITEE_URL="${GITEE_URL:-git@gitee.com:huo-haijie/Drone_ros2.git}"
GITHUB_URL="${GITHUB_URL:-https://github.com/howe12/Drone_ros2.git}"
problems=0

say() { printf '%s\n' "$*"; }

say "== 1. gitee 远端"
if git remote get-url gitee >/dev/null 2>&1; then
  say "   ✓ 已存在: $(git remote get-url gitee)"
else
  problems=$((problems+1))
  say "   ✗ 缺失"
  if [ "$CHECK_ONLY" -eq 0 ]; then
    git remote add gitee "$GITEE_URL"
    say "   + 已添加: $GITEE_URL"
    problems=0
  fi
fi

say "== 2. origin 的 push 目标（需要同时包含 GitHub 与 Gitee）"
pushes=$(git remote get-url --push --all origin 2>/dev/null)
has_gh=0; has_gt=0
echo "$pushes" | grep -q 'github\.com' && has_gh=1
echo "$pushes" | grep -q 'gitee\.com' && has_gt=1
say "   GitHub: $([ $has_gh -eq 1 ] && echo ✓ || echo ✗)"
say "   Gitee : $([ $has_gt -eq 1 ] && echo ✓ || echo ✗)"
if [ "$CHECK_ONLY" -eq 0 ]; then
  if [ $has_gt -eq 0 ]; then
    git remote set-url --add --push origin "$GITEE_URL"
    say "   + 已把 Gitee 追加为 origin 的推送目标"
  fi
  if [ $has_gh -eq 0 ]; then
    git remote set-url --add --push origin "$GITHUB_URL"
    say "   + 已把 GitHub 追加为 origin 的推送目标"
  fi
fi

say "== 3. post-push 钩子（防止只推了 GitHub 就忘了 Gitee）"
cur_hooks=$(git config --get core.hooksPath || true)
if [ "$cur_hooks" = "scripts/git-hooks" ]; then
  say "   ✓ 已启用: core.hooksPath=scripts/git-hooks"
else
  problems=$((problems+1))
  say "   ✗ 未启用${cur_hooks:+ (当前: $cur_hooks)}"
  if [ "$CHECK_ONLY" -eq 0 ]; then
    git config core.hooksPath scripts/git-hooks
    say "   + 已设置 core.hooksPath=scripts/git-hooks"
    problems=$((problems-1))
  fi
fi

say "== 4. Gitee SSH 连通性"
# 注意: gitee 的欢迎信息带 ANSI 颜色码, 必须先剥离再匹配
ssh_out=$(timeout 15 ssh -o BatchMode=yes -o StrictHostKeyChecking=no -T git@gitee.com 2>&1 | sed -E 's/\x1b\[[0-9;]*m//g')
if echo "$ssh_out" | grep -q "successfully authenticated"; then
  who=$(echo "$ssh_out" | grep -o '@[a-zA-Z0-9_-]*' | head -1 | tr -d '@')
  say "   ✓ SSH 可用 (账号: ${who:-未知})"
else
  problems=$((problems+1))
  say "   ✗ SSH 认证失败 —— 需要把公钥加到 Gitee「设置 → SSH 公钥」"
  say "     本机公钥: $(cat ~/.ssh/id_ed25519.pub 2>/dev/null | cut -c1-60)..."
fi

say ""
if [ "$problems" -eq 0 ]; then
  say "✅ 配置完整：以后 git push origin main 会同时更新 GitHub 和 Gitee"
else
  say "⚠️ 还有 $problems 项需要处理（用 bash scripts/setup_gitee_sync.sh 自动修复）"
  exit 1
fi
