# Gitee 同步要求（交接给课程 agent / 任何协作者）

> 一句话：**在本仓库里执行 `git push origin main` 就会同时更新 GitHub 和 Gitee。**
> 但"双远端"是**本地 git 配置**，不在代码里 —— 换克隆、换机器、换 agent 都要先跑一次 `bash scripts/setup_gitee_sync.sh`。

---

## 1. 凭据条件（缺一不可）

| 条件 | 说明 | 怎么自查 |
|---|---|---|
| Gitee 账号 | `huo-haijie`（仓库 `gitee.com/huo-haijie/Drone_ros2`）| 见下方 SSH 命令 |
| SSH 公钥已注册 | 公钥加到 Gitee「设置 → SSH 公钥」 | `ssh -T git@gitee.com` 返回 `successfully authenticated` |
| 仓库内已配置双推送 | `origin` 的 push URL 列表含 GitHub + Gitee | `git remote get-url --push --all origin` |
| post-push 钩子已启用 | `core.hooksPath=scripts/git-hooks` | `git config --get core.hooksPath` |

**一键检查 + 自动修复**（推荐每个新环境先跑）：

```bash
bash scripts/setup_gitee_sync.sh          # 检查并修复
bash scripts/setup_gitee_sync.sh --check  # 只检查，不改动
```

正常输出应包含：

```
== 2. origin 的 push 目标（需要同时包含 GitHub 与 Gitee）
   GitHub: ✓
   Gitee : ✓
== 3. post-push 钩子（防止只推了 GitHub 就忘了 Gitee）
   ✓ 已启用: core.hooksPath=scripts/git-hooks
== 4. Gitee SSH 连通性
   ✓ SSH 可用 (账号: huo-haijie)
✅ 配置完整：以后 git push origin main 会同时更新 GitHub 和 Gitee
```

---

## 2. 推送方式（三选一，推荐第 1 种）

```bash
# ① 推荐: 一次双发（依赖 origin 的双 push URL 配置）
git push origin main

# ② 容错版: 逐远端重试、互不影响、分别报告（GitHub 网络差时用）
bash scripts/push_all.sh

# ③ 兜底: 显式分别推
git push origin main && git push gitee main
```

**不要**只执行 `git push gitee main` —— 那样 GitHub 会落后，两边版本不一致。

---

## 3. 提交后必须做的校验

```bash
git rev-parse --short HEAD                 # 本地
git ls-remote origin main | cut -c1-7      # GitHub
git ls-remote gitee  main | cut -c1-7      # Gitee
```

**三者必须相同**。不相同就说明有一边没推上去，用 `bash scripts/push_all.sh` 补推。

---

## 4. 给课程 agent 的工作流约定

每轮内容更新（新增章节 / 改图 / 补练习）完成后：

1. `git add -A && git commit -m "<类型>: <说明>"`
2. `git push origin main`（或 `bash scripts/push_all.sh`）
3. 跑第 3 节的三个哈希校验，确认两边一致
4. 若推送失败：
   - GitHub 失败 → 用 `bash scripts/push_all.sh`（内置代理/直连交替重试）
   - Gitee 失败 → 先 `bash scripts/setup_gitee_sync.sh` 复查，再看第 5 节

---

## 5. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `git push` 只更新了 GitHub | origin 的 push URL 只有 GitHub | `bash scripts/setup_gitee_sync.sh` |
| `Permission denied (publickey)` | 公钥没在 Gitee 注册 | 把 `~/.ssh/id_ed25519.pub` 内容加到 Gitee |
| `404 not found` | 仓库不存在或路径拼错 | 确认 `gitee.com/huo-haijie/Drone_ros2` 已创建 |
| `git push` 返回非零但 GitHub 成功了 | 多 URL 推送时 Gitee 失败 | 用 `bash scripts/push_all.sh` 单独补推 Gitee |
| Gitee 页面显示"仓库审核中" | Gitee 对公开仓库的内容审核 | 等几分钟；必要时按提示确认 |
| 新克隆里 `gitee` remote 不存在 | 远端配置不随仓库复制 | 跑一次 `bash scripts/setup_gitee_sync.sh` |

---

## 6. 机制说明（为什么这样做）

- **主手段**：`origin` 配置了两个 push URL（GitHub + Gitee），一次 `git push` 双发。
- **保险丝**：`.git/hooks/post-push`（由 `scripts/git-hooks/post-push` 提供）——
  若检测到 origin 只配了 GitHub，推送结束后会自动补推 Gitee。
  临时跳过：`GITEE_SYNC_SKIP=1 git push ...`
- **为什么不写死在代码里**：git 远端配置存于 `.git/config`，不随仓库分发；
  hook 也需要显式启用 `core.hooksPath`。所以"装一次"这一步无法完全省略，
  但 `setup_gitee_sync.sh` 把它压成了一条命令。

---

## 7. 课程网站托管（Gitee Pages 已停止服务）

### 7.1 现状：Gitee Pages 不可用

**Gitee Pages 已停止服务**（2025 年确认）：

- 仓库「服务」菜单中已无 Gitee Pages 入口（实测截图确认）
- 直接访问 `gitee.com/<owner>/<repo>/pages` 返回 404
- 官方反馈区 issue：[Gitee Pages服务是停用了吗](https://gitee.com/oschina/git-osc/issues/ID1EVM)（状态「已完成」）

**结论：课程站点不能部署在 Gitee Pages 上。** 本仓库已把 `docs/vendor/` 的 docsify 本地化（不依赖 jsdelivr / Google Fonts），这份准备对任何静态托管平台都通用，不会浪费。

代码镜像仍可通过 Gitee 获取（`git clone https://gitee.com/huo-haijie/Drone_ros2.git`），只是"网页托管"需要换平台。

### 7.2 替代方案对比

| 方案 | 国内访问 | 成本 | 自动部署 | 备注 |
|---|---|---|---|---|
| **GitHub Pages**（现状） | 不稳定（时快时慢/需代理） | 免费 | ✅ 推送即部署 | 已上线：`howe12.github.io/Drone_ros2` |
| **Cloudflare Pages** | 较好（全球 CDN） | 免费 | ✅ 连 GitHub 自动部署 | 推荐首选，`*.pages.dev` 免域名 |
| Vercel / Netlify | 差（默认域名常被墙） | 免费 | ✅ | 需自备域名才可用 |
| 阿里云 OSS / 腾讯云 COS | 最好 | 按量计费 | 需 CI 或手动上传 | 绑定自定义域名需**ICP 备案** |
| 自建 nginx | 取决于服务器 | 服务器成本 | 需 CI | 需公网 IP + 域名 |

### 7.3 推荐：Cloudflare Pages（步骤）

仓库已就绪（静态站，输出目录 `docs/`，无需构建命令）：

1. 用 GitHub 账号登录 <https://dash.cloudflare.com/> → Workers & Pages → Create → Pages → Connect to Git
2. 选择仓库 `howe12/Drone_ros2`，分支 `main`
3. 构建设置：**Framework preset = None**，**Build command 留空**，**Build output directory = `docs`**
4. 保存部署，得到形如 `https://drone-ros2.pages.dev` 的地址
5. 之后每次 push 自动重新部署（无需手动点更新）

> 如需自定义域名，在 Pages 项目 → Custom domains 里绑定，Cloudflare 会自动配 DNS。

### 7.4 已废弃：Gitee Pages 相关脚本

`scripts/deploy_gitee_pages.sh` 保留作为历史记录，其功能已随 Gitee Pages 停服失效，**不要再使用**。
