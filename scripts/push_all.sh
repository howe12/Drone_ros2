#!/usr/bin/env bash
# 一次推送到两个远端（GitHub + Gitee）
#
#   bash scripts/push_all.sh
#
# 说明:
#   - origin = GitHub (HTTPS, 网络不稳, 代理/直连交替重试)
#   - gitee  = Gitee  (SSH, 需已配置公钥)
#   - 任一失败不影响另一个, 最后汇总结果并以非零退出码提示
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
BRANCH="${BRANCH:-main}"

push_github() {
  for i in $(seq 1 6); do
    if [ $((i % 2)) -eq 1 ]; then
      env -u http_proxy -u https_proxy -u all_proxy \
        git -c http.proxy= -c https.proxy= push origin "$BRANCH" 2>&1 | tail -1
      [ "${PIPESTATUS[0]}" -eq 0 ] && return 0
    else
      https_proxy=http://127.0.0.1:7897 http_proxy=http://127.0.0.1:7897 \
        git push origin "$BRANCH" 2>&1 | tail -1
      [ "${PIPESTATUS[0]}" -eq 0 ] && return 0
    fi
    echo "  GitHub 第 $i 次失败, 20s 后重试" >&2
    sleep 20
  done
  return 1
}

push_gitee() {
  for i in $(seq 1 3); do
    timeout 600 git push gitee "$BRANCH" 2>&1 | tail -1
    [ "${PIPESTATUS[0]}" -eq 0 ] && return 0
    echo "  Gitee 第 $i 次失败, 20s 后重试" >&2
    sleep 20
  done
  return 1
}

echo "== 本地 HEAD: $(git rev-parse --short HEAD)  分支: $BRANCH"

gh_ok=0; gt_ok=0
echo "== 推送到 GitHub (origin) ..."
push_github && gh_ok=1
echo "== 推送到 Gitee (gitee) ..."
push_gitee && gt_ok=1

echo
echo "GitHub: $([ $gh_ok -eq 1 ] && echo ✅ || echo ❌)"
echo "Gitee : $([ $gt_ok -eq 1 ] && echo ✅ || echo ❌)"
[ $gh_ok -eq 1 ] && [ $gt_ok -eq 1 ]
