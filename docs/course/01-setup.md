# 01 环境搭建与首飞

> 难度 L0 · 对应场景 `ctrl.takeoff_hover_land` · 预计：最小路径 10 分钟（环境已装好）／完整含编译 1–2 小时

## 本章问题

我们要让一架仿真四旋翼自动完成起飞、悬停和降落，并用判分系统证明它成功。

需要解决三个具体问题：

1. 怎么让整套环境跑起来，并让飞机动起来。
2. 飞机动起来之后，背后是哪几个程序在协作。
3. 如何判断一次飞行真的成功，而不是只看窗口里的动画。

本章点亮的这条数据链路，是后续所有实验的基础：第 7 章的 SLAM 建图、第 12 章的双机编队，用的都是同一条链路，只是换了机型和算法节点。

## 学习目标

| 维度 | 学习前 | 学习后 |
|---|---|---|
| 认知定位 | 以为仿真就是"打开 Gazebo 看个动画" | 理解 Gazebo 只是物理世界，飞控另有其人 |
| 链路理解 | 说不清 PX4、Gazebo、ROS2 谁跟谁说话 | 能画出 `Gazebo ↔ PX4 ↔ Agent ↔ 你的节点` 四段链路 |
| 环境能力 | 环境装到哪一步全靠运气 | 能用锁版本+补丁的方式，可复现地还原环境 |
| 动手能力 | 只会复制别人的命令 | 能改参数跑出不同高度/半径的飞行 |
| 排障能力 | 报错就懵 | 能定位"端口被占/缺库/解锁被拒"三类起步故障 |

## 前置知识

读者需要会 Linux 基本命令（`cd`、`ls`、`grep`）和 Python 基础语法。本章不要求 ROS2 或 PX4 经验。

**阅读顺序建议**：先按第 1 节跑通最小路径，看到飞机飞起来；再带着"刚才发生了什么"的疑问读第 2 节的原理；最后用第 3 节的案例做扩展练习。零基础读者不要在第 2 节停留过久——那里很多概念会在第 2 章系统展开。

---

## 1. 先让它飞起来（最小路径）

这一节的目标只有一个：**让飞机飞起来**。先不解释原理，先把结果拿到手。

### 1.1 前提：环境已安装

若环境尚未安装，先运行（耗时较长，可后台跑）：

```bash
bash scripts/install_gz_harmonic.sh   # 装 Gazebo Harmonic，需 sudo，已处理 /usr/bin/gz 冲突
bash scripts/setup_workspace.sh       # 克隆依赖 + 打补丁 + 全量编译
```

> 编译耗时 1–2 小时。若卡住，查看 `logs/` 下对应日志；无 GPU 的降级路径待补充（见 ROADMAP 待办）。

### 1.2 启动仿真栈

```bash
bash scripts/run_simulation.sh xtd2
```

等待 10–40 秒，日志出现 `Ready for takeoff!` 即就绪：

```text
[px4_launch-3] INFO  [gz_bridge] world: aruco, model name: x500_0, simulation model: x500
[microxrceagent-2] info | create_client | client_key: 0x00000001
[px4_launch-3] INFO  [commander] Ready for takeoff!
```

<!-- SCREENSHOT_PLACEHOLDER
id: shot-ch01-stack-ready
caption: 图 1.1 仿真栈启动后的 Gazebo Sim 窗口与终端日志（含 Ready for takeoff）
when: 运行 run_simulation.sh xtd2 之后，日志出现 Ready for takeoff! 时
what: 同时截 Gazebo Sim 窗口（能看到地面上的 x500 飞机与 aruco 靶标）和终端日志（能看到 Ready for takeoff!）
how: |
  WID=$(DISPLAY=:0 xwininfo -root -tree | grep -oE '0x[0-9a-f]+ "Gazebo Sim"' | head -1 | awk '{print $1}')
  DISPLAY=:0 import -window "$WID" docs/images/shot-ch01-stack-ready.png
-->
![图 1.1 栈就绪画面](images/PLACEHOLDER-shot-ch01-stack-ready.png)

### 1.3 让它飞（新终端）

```bash
source /opt/ros/humble/setup.bash
source <仓库路径>/ros2_ws/install/setup.bash
ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0 -p circle_duration:=0.0
```

日志出现 `演示完成 ✔` 即成功：

```text
[drone_sim_offboard_demo]: 请求 Offboard 模式 + 解锁 ...
[drone_sim_offboard_demo]: 已解锁并进入 Offboard, 起飞至 5.0 m
[drone_sim_offboard_demo]: 到达目标高度, 开始画圆
[drone_sim_offboard_demo]: 已降落并上锁, 演示完成 ✔
```

飞机起飞后的画面（真实运行截图）：

![首飞画面](images/04_xtd2_x500_takeoff.png)

### 1.4 用判分系统证明它成功

"看起来飞起来了"不算数。课程要求用指标判定：

```bash
python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1 --json out.json --md out.md
```

<!-- SCREENSHOT_PLACEHOLDER
id: shot-ch01-grade-report
caption: 图 1.2 grade.py 判分输出：三项指标与 PASS 汇总
when: 运行 grade.py 之后
what: 截取终端输出或 out.md 内容，需能看到三项指标（起飞高度误差、全程耗时、结束状态）与 3/3 PASS
how: |
  python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1 --md /tmp/out.md
  截终端输出，或 cat /tmp/out.md 后截图
-->
![图 1.2 判分报告](images/PLACEHOLDER-shot-ch01-grade-report.png)

---

## 2. 刚才发生了什么（回头理解原理）

现在你已经看到飞机飞起来了。这一节解释背后的机制。

### 2.1 仿真由哪几部分组成

| 组件 | 职责 | 运行形态 |
|---|---|---|
| **Gazebo Harmonic** | 物理世界：重力、碰撞、传感器、画面 | `gz sim` 进程 |
| **PX4 SITL** | 飞控固件：姿态控制、EKF、模式管理 | `px4` 进程（固件编译为 Linux 程序） |
| **ROS2 节点** | 任务逻辑：发指令、做感知 | 你写的节点，如 `offboard_demo` |

**为什么分成三块？**

1. **飞控不能被任务逻辑拖累**：姿态控制需要 250Hz–1kHz 稳定周期，和视觉算法同进程会因卡顿失稳。
2. **物理环境可替换**：换仿真器或接真机时，PX4 与你的节点都不用改——SITL 与真机共用同一份固件，差别只在数据来源。
3. **算法可独立叠加**：你的节点只依赖话题，因此第 7 章 SLAM、第 13 章集群可以逐步加，不必重写底层。

### 2.2 数据链路与 MicroXRCEAgent

PX4 用 uXRCE-DDS（嵌入式轻量协议），ROS2 用标准 DDS，两者协议不同，需要 `MicroXRCEAgent` 做中转：

<!-- IMAGE_PLACEHOLDER
id: fig-ch01-data-link
caption: 图 1.3 课程数据链路：Gazebo、PX4 SITL、MicroXRCEAgent 与 ROS2 节点之间的消息方向
alt: 数据链路示意图，Gazebo 提供传感器与执行器接口，PX4 SITL 经 uXRCE-DDS 连接 MicroXRCEAgent，Agent 再以 DDS 连接 ROS2 节点
prompt: |
  技术教材插图，白底，扁平化风格，中文标注，无真人无卡通无水印。横向四段架构图：
  第 1 段标 "Gazebo Harmonic（物理世界：模型、传感器、画面）"；
  第 2 段标 "PX4 SITL（飞控固件：姿态控制、EKF）"，与第 1 段之间双向箭头标 "传感器数据 / 电机指令"；
  第 3 段标 "MicroXRCEAgent（协议代理，UDP 8888）"，与第 2 段之间双向箭头标 "uXRCE-DDS"；
  第 4 段标 "ROS2 节点（你的任务程序）"，与第 3 段之间双向箭头标 "DDS 话题 /fmu/..."。
  第 4 段上方虚线框标注示例话题 "/x500_0/fmu/out/vehicle_status"。圆角矩形组件，箭头清晰，配色只用蓝灰。
-->
![图 1.3 数据链路](images/PLACEHOLDER-fig-ch01-data-link.png)

Agent 是**必需的中间环节**。没有它，节点看不到 `/fmu/...` 话题，PX4 也收不到指令。确认链路：

```bash
ros2 topic list | grep fmu      # 栈启动后应出现 /x500_0/fmu/... 系列话题
```

**三个常用话题**（先看这三个）：

| 话题 | 方向 | 内容 |
|---|---|---|
| `/x500_0/fmu/out/vehicle_status` | PX4 → 节点 | 是否解锁（`arming_state`）、当前模式（`nav_state`） |
| `/x500_0/fmu/out/vehicle_local_position` | PX4 → 节点 | 局部坐标 x/y/z（NED，z 向下为正） |
| `/x500_0/fmu/in/trajectory_setpoint` | 节点 → PX4 | 目标位置/速度 |

查看状态：`ros2 topic echo /x500_0/fmu/out/vehicle_status --once`。其中 `arming_state` 为 1 表示上锁、2 表示解锁；`nav_state` 为 14 表示 Offboard 模式。这两个字段是后续判断飞机状态的基础（第 2 章详述）。

### 2.3 一条启动命令做了什么

`bash scripts/run_simulation.sh xtd2` 按顺序拉起：

1. `gz sim`：加载 aruco 世界与 x500 模型
2. `MicroXRCEAgent udp4 -p 8888`：DDS 代理
3. `px4 -d -s .../rcS`：飞控固件，通过 gz 插件接入模型
4. `ros_gz_bridge` 与通信节点：把 Gazebo 传感器数据转成 ROS2 话题

脚本处理了启动顺序、命名空间推导和退出清理。手动启动若顺序错（如先 PX4 后 Agent），飞控会连不上代理。

### 2.4 首飞阶段与可观察状态

| 阶段 | 日志标志 | `vehicle_status` 变化 | Gazebo 画面 |
|---|---|---|---|
| 预流与请求 | `请求 Offboard 模式 + 解锁 ...` | `nav_state` 变为 Offboard(14) | 静止在地面 |
| 起飞 | `已解锁并进入 Offboard, 起飞至 5.0 m` | `arming_state` 由 1 变 2 | 螺旋桨转动，高度上升 |
| 到达高度 | `到达目标高度, 开始画圆` | 保持解锁与 Offboard | 悬停在约 5m |
| 降落上锁 | `已降落并上锁, 演示完成 ✔` | `arming_state` 回到 1 | 回到地面，桨停转 |

<!-- IMAGE_PLACEHOLDER
id: fig-ch01-flight-phases
caption: 图 1.4 首飞四个阶段对应的日志、状态位与画面变化
alt: 首飞阶段对照图，横轴为时间，标出预流请求、起飞、到达高度、降落上锁四阶段，对应日志标志、arming_state/nav_state 与画面
prompt: |
  技术教材插图，白底，扁平化风格，中文标注，无真人无卡通无水印。
  横向时间轴分四段："1 预流与请求"、"2 起飞"、"3 到达高度"、"4 降落上锁"。
  每段下方标注：
  1 "日志: 请求 Offboard 模式 + 解锁；状态: nav_state=14；画面: 静止地面"；
  2 "日志: 已解锁并进入 Offboard；状态: arming_state 1→2；画面: 高度上升"；
  3 "日志: 到达目标高度；状态: 保持解锁；画面: 悬停约 5m"；
  4 "日志: 已降落并上锁；状态: arming_state=1；画面: 回到地面"。
  右侧画简化四旋翼高度曲线（平—升—平—降），配色蓝灰，标注清晰。
-->
![图 1.4 首飞阶段](images/PLACEHOLDER-fig-ch01-flight-phases.png)

### 2.5 环境可复现

环境按锁定版本复现：PX4 v1.15.4、`px4_msgs` release/1.15、XTDrone2 锁定 commit，叠加 18 个补丁固化已踩过的坑（GStreamer 冲突、Qt5 缺失、QoS 死锁、多机工作目录冲突等）。实现见 `scripts/setup_workspace.sh` 与 `patches/`。

---

## 3. 扩展：案例与排障

下面四个案例来自本仓库实测记录，数据可在 `logs/grade/` 与 `catalog/scenarios.yaml` 对照。

### 案例 1：标准首飞（场景 `ctrl.takeoff_hover_land`）

**背景**：验证环境可用，并给出可核验结果。

**做法**：关闭画圆，只做起降：

```bash
ros2 run drone_sim offboard_demo --ros-args \
  -p px4_ns:=/x500_0 -p takeoff_altitude:=5.0 -p circle_duration:=0.0
```

**结果**（`logs/grade/demo8.md`，1 次运行）：

| 指标 | 中位数 | 阈值 | 结果 |
|---|---|---|---|
| 起飞高度误差 | 0.3 m | < 0.4 | PASS |
| 全程耗时 | 18.6 s | < 75 | PASS |
| 结束状态 | disarmed | disarmed | PASS |

汇总 3/3 通过。

**要点**：高度误差用 Gazebo 世界系真值算。若用 PX4 局部坐标，EKF 原点重置会让数字失真（第 4、7 章反复出现）。

### 案例 2：参数变体——改高度与画圆

**背景**：想飞 8m 并画 10 秒圆，确认参数化接口够用。

```bash
ros2 run drone_sim offboard_demo --ros-args \
  -p px4_ns:=/x500_0 -p takeoff_altitude:=8.0 \
  -p circle_radius:=5.0 -p circle_speed:=0.5 -p circle_duration:=10.0
```

`offboard_demo` 可调参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `takeoff_altitude` | 5.0 | 起飞高度（m） |
| `circle_radius` | 3.0 | 画圆半径（m） |
| `circle_speed` | 0.3 | 画圆角速度（rad/s） |
| `circle_duration` | 25.0 | 画圆时长（s，0 跳过） |
| `px4_ns` | `''` | 命名空间；多机写 `/x500_0` |
| `target_system` | 1 | 多机须等于 MAV_SYS_ID（实例号+1） |

**要点**：画圆耗时约 `2π / circle_speed`。参数写进命令而非源码，第 4 章航点、第 10 章行为树沿用此思路。

### 案例 3：故障复现——Agent 缺失导致"节点空转"

**背景**：只看到"等待 PX4 状态"，飞机不动，但不报明显错误。

```bash
pkill -x MicroXRCEAgent                 # 制造故障
ros2 topic list | grep fmu              # 无输出：话题消失
ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0
# 日志反复 "等待 PX4 状态..."，飞机不动
bash scripts/run_simulation.sh xtd2      # 重启栈恢复
```

<!-- SCREENSHOT_PLACEHOLDER
id: shot-ch01-no-topic
caption: 图 1.5 故障现象：Agent 停掉后 topic list 无 fmu 话题、节点打印"等待 PX4 状态"
when: 执行 pkill -x MicroXRCEAgent 后运行节点时
what: 截取终端，需同时看到 ros2 topic list 无输出 与 节点日志"等待 PX4 状态..."
how: |
  pkill -x MicroXRCEAgent
  ros2 topic list | grep fmu
  ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0
  截终端输出
-->
![图 1.5 故障现象](images/PLACEHOLDER-shot-ch01-no-topic.png)

**要点**：排障顺序为"先看代理在不在 → 再看话题在不在 → 最后看 QoS 与状态字段"。此路径在第 7、12 章同样适用。

### 案例 4：多机命名空间推导（为第 12 章预热）

机型名去掉 `gz_` 加实例号：

```text
gz_x500            → /x500_0
gz_x500_mono_cam   → /x500_mono_cam_0
gz_x500_lidar_3d   → /x500_lidar_3d_0
```

第 N 架实例号是 N，第二架 `/x500_1` 的 `target_system` 必须为 2。本章先用单机熟悉 `/x500_0`；第 12 章命名空间写错会直接导致"2 号机不解锁"。

---

## 任务卡

### 任务卡 1：复现一次完整首飞

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 飞机自动起飞到 5m、悬停、降落上锁，日志出现"演示完成 ✔" |
| 2 通信骨架/前置 | 栈已启动；节点发 `/x500_0/fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}`，收 `/x500_0/fmu/out/{vehicle_status, vehicle_local_position}` |
| 3 参考 prompt | "你是 ROS2 无人机助教。我运行 `ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0` 后飞机没起飞。请按'话题→QoS→模式→命令'顺序列排查步骤，每条给要执行的 ros2 命令，先给命令清单，不要讲理论。" |
| 4 执行步骤 | ① 启动栈等到 `Ready for takeoff!` ② 新终端 source 两层环境 ③ 跑命令 ④ 观察日志与 Gazebo ⑤ `ros2 topic echo /x500_0/fmu/out/vehicle_status --once` 看 `arming_state` 是否 1→2→1 |
| 5 验收标准 | □ 日志四行齐全 □ 窗口看到爬升与降落 □ `grade.py` 三项 PASS □ 能解释 `arming_state` 1 和 2 的含义 |
| 6 原理 + 扩学 | 教**整条链路可用性**。扩学：`circle_duration` 设 10，观察遥测 x/y 是否落在半径 3m 圆上 |

### 任务卡 2：参数改造

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 起飞到 8m 并画 10 秒圆 |
| 2 通信骨架/前置 | 同任务卡 1；旋钮都是 ROS2 参数，用 `-p 名字:=值` 覆盖 |
| 3 参考 prompt | "给我一条 `ros2 run` 命令：drone_sim 的 offboard_demo，起飞 8m、画圆 10s、半径 5m。说明每个 `-p` 参数的默认值和单位。" |
| 4 执行步骤 | 从源码 `grep declare_parameter offboard_demo.py` 找参数名 → 再跑 |
| 5 验收标准 | □ 到达 8m（遥测 z≈-8）□ 画圆半径≈5m □ `grade.py` 仍 PASS |
| 6 原理 + 扩学 | 教**参数化优于改代码**。扩学：加一个 `yaw_rate` 参数让飞机边飞边转 |

### 任务卡 3：故障注入（本章的"刺"）

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 亲手制造并修复一个起步故障 |
| 2 通信骨架/前置 | 先用 `pkill -x MicroXRCEAgent` 杀掉 Agent，再跑 offboard_demo |
| 3 参考 prompt | "我的 ROS2 节点一直打印'等待 PX4 状态...'，`ros2 topic list` 看不到 /x500_0/fmu 话题。请列出三个最可能原因和验证命令。" |
| 4 执行步骤 | ① 杀 Agent ② 跑节点观察"等待 PX4 状态" ③ `ros2 topic list \| grep fmu` 验证话题消失 ④ 重启 Agent 恢复 |
| 5 验收标准 | □ 能复现"无话题、节点空转" □ 能说出 Agent 在链路中的位置 □ 恢复后首飞 PASS |
| 6 原理 + 扩学 | 教**链路分段排障法**：先桥、再话题、最后 QoS 与模式 |

## AI 实验室

**先自己想**：如果飞机不动，但日志一直打印"等待 PX4 状态..."，你觉得问题最可能出在哪一段？先写下你的判断（代理、话题、还是模式？），再看下面。

**问 AI**：复制这段上下文 + 你的问题给 AI。

> 【课程环境】PX4 v1.15.4 / ROS2 Humble / Gazebo Harmonic / XTDrone2；命名空间 `/x500_0`；
> 统一接口 `drone_sim.px4_interface.PX4Interface`；发布端 QoS 为 BEST_EFFORT+VOLATILE。
> 要求：先给命令、再解释；不要假设版本；不确定请说明。
>
> 我的问题：运行 `offboard_demo` 后日志一直"等待 PX4 状态..."，飞机不动。请按"代理→话题→QoS→模式"给出排查步骤，每条附要执行的 ros2 命令。

**验证 AI 的答案**：本章范围内 AI 容易错的地方：

- 它可能让你用 `/fmu/...` 相对话题——本课程用绝对 `/x500_0/fmu/...`；
- 它可能忽略 Agent 是必需环节，直接让你查飞控；
- 它可能给出旧版字段或命令，与锁定的 v1.15.4 不符。

用下面命令核对 AI 的说法：

```bash
ros2 topic list | grep fmu                                  # 话题在不在
ps -ef | grep MicroXRCEAgent                                # 代理在不在
tail -20 logs/xtd2_sim.log | grep -iE "create_client|session"  # 会话建了没
```

**扩展挑战（用 AI 一起做）**：让 AI 帮你在 `offboard_demo` 上加一个 `hover_time` 参数——起飞后先悬停 N 秒再画圆。

验收：① 你自己改的代码能解释每一处；② 跑通并用 `grade.py` 判定仍 PASS；③ 能向同学讲清"参数怎么声明、怎么读、怎么影响流程"。

## ✅ 检查清单

- [ ] 我能独立复现环境，并知道编译失败时看哪个日志
- [ ] 我能启动仿真栈并用 `ros2 topic list | grep fmu` 确认链路已通
- [ ] 我能完成一次首飞，并用 `grade.py` 拿到 3/3 PASS 报告
- [ ] 我能改参数（高度/半径/角速度）飞出不同轨迹，并预判耗时变化
- [ ] 我能复现并修复"Agent 缺失导致话题消失"的故障
- [ ] 我能说出 Gazebo / PX4 SITL / ROS2 节点三者的分工

## 小结与下一步

本章完成了环境复现和一次自动起降，并用判分系统获得可核验的指标（高度误差 0.3m、耗时 18.6s、3/3 PASS）。

飞机已经能飞，但你还不知道它**为什么**听话：节点在请求解锁前做了什么判断、NED 坐标系下 `z` 为什么是负的、EKF 原点重置会怎样影响定位。这些是第 2 章的内容。

[下一章：Offboard 控制与 PX4Interface →](course/02-offboard.md)
