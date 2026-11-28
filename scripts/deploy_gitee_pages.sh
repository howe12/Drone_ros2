#!/usr/bin/env bash
# ============================================================
# Gitee Pages 部署助手
# 用法:
#   bash scripts/deploy_gitee_pages.sh --check   # 检查仓库就绪 + 探测站点是否已上线
#   bash scripts/deploy_gitee_pages.sh           # 打印开通步骤并打开 Gitee Pages 设置页
#
# 说明: Gitee Pages 免费版必须【实名认证 + 网页手动开通 + 每次推送后手动点"更新"】，
#       这些是账号侧操作，无法用脚本/API 代办（本仓库无 Gitee API token）。
#       本脚本负责: 把仓库侧准备到位、给出精确步骤、开通后验证站点。
# ============================================================
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OWNER="huo-haijie"
REPO="Drone_ros2"
PAGES_URL="https://${OWNER}.gitee.io/${REPO}/"
SERVICE_URL="https://gitee.com/${OWNER}/${REPO}/pages"

check_ready() {
    echo "== 1. 仓库侧就绪检查"
    local ok=1
    for f in docs/index.html docs/vendor/docsify.min.js docs/vendor/vue.css docs/README.md docs/_sidebar.md; do
        if [ -f "$ROOT/$f" ]; then echo "   ✓ $f"; else echo "   ✗ 缺少 $f"; ok=0; fi
    done
    if grep -q "jsdelivr\|googleapis" "$ROOT/docs/index.html"; then
        echo "   ✗ index.html 仍依赖外部 CDN（国内可能加载失败）"; ok=0
    else
        echo "   ✓ index.html 已用本地 vendor，无外部 CDN 依赖"
    fi
    [ "$ok" = 1 ] && echo "   仓库侧就绪 ✅" || echo "   仓库侧未就绪 ❌"
}

probe_site() {
    echo "== 2. 探测 Gitee Pages 站点"
    local code
    code=$(curl -s -o /dev/null --max-time 20 -w "%{http_code}" "$PAGES_URL" 2>/dev/null || echo 000)
    if [ "$code" = 200 ]; then
        echo "   ✅ 已上线: $PAGES_URL (HTTP $code)"
    else
        echo "   ⏳ 尚未上线 (HTTP $code): $PAGES_URL"
        echo "      需先在网页开通 Gitee Pages（见下方步骤）"
    fi
}

print_steps() {
    cat <<EOF

== Gitee Pages 开通步骤（需你在浏览器操作，账号侧无法代办）

1. 打开仓库 Pages 设置页:
   $SERVICE_URL
   （或: Gitee 仓库首页 → 顶部「服务」→「Gitee Pages」）

2. 若提示实名认证，先完成实名认证（免费版必需）。

3. 部署配置:
   - 部署分支: main
   - 部署目录: docs/
   - 点击「启动」/「部署」

4. 部署成功后站点地址:
   $PAGES_URL

5. ⚠️ 以后每次 git push 后，回到该设置页手动点「更新」，
   Gitee Pages 才会重新部署（不会自动跟随推送）。

== 开通后验证
   bash scripts/deploy_gitee_pages.sh --check
EOF
}

case "${1:-}" in
    --check)
        check_ready
        probe_site
        ;;
    *)
        check_ready
        probe_site
        print_steps
        # 尝试打开浏览器（无图形环境时忽略）
        if command -v xdg-open >/dev/null 2>&1; then
            xdg-open "$SERVICE_URL" >/dev/null 2>&1 || true
        fi
        ;;
esac
