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

## 7. Gitee Pages（课程网站）

GitHub 侧课程站点走 GitHub Pages（`howe12.github.io/Drone_ros2`）。
Gitee Pages 是**另一套服务**，站点地址将是 `https://huo-haijie.gitee.io/Drone_ros2/`。

### 7.1 仓库侧已准备就绪（无需再改）

- docsify 及全部插件已**下载到本地 `docs/vendor/`**，`index.html` 不再依赖 jsdelivr 等外部 CDN
  （jsdelivr 与 Google Fonts 在国内不稳定，已移除）。GitHub / Gitee 两侧都能自包含加载。
- 部署目录用 `docs/`，docsify 用 hash 路由，相对路径在 Gitee Pages 下无需改配置。

一键检查就绪状态 + 探测站点是否上线：

```bash
bash scripts/deploy_gitee_pages.sh --check
```

### 7.2 开通步骤（账号侧操作，脚本/API 无法代办）

Gitee Pages 免费版必须**实名认证 + 网页手动开通**，本仓库无 Gitee API token，故以下步骤需你在浏览器完成：

1. 打开 `https://gitee.com/huo-haijie/Drone_ros2/pages`（或仓库首页 → 服务 → Gitee Pages）
2. 若提示实名认证，先完成实名认证
3. 部署分支选 `main`，部署目录选 `docs/`，点「启动/部署」
4. 成功后访问 `https://huo-haijie.gitee.io/Drone_ros2/`

运行 `bash scripts/deploy_gitee_pages.sh` 会打印上述步骤并尝试打开设置页。

### 7.3 每次推送后手动更新

**Gitee Pages 不会自动跟随推送**：每次 `git push origin main` 后，需回到 Pages 设置页手动点「更新」才会重新部署。
（GitHub Pages 是自动的；这是两个平台的差异。）
