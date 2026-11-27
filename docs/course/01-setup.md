# 01 环境搭建与首飞

> 难度 L0 · 对应场景 `ctrl.takeoff_hover_land` · 预计 1–2 小时（含编译）

## 本章问题

我们要在一台装好 ROS2 Humble 的 Ubuntu 上，把课程环境完整复现出来，并让一架仿真四旋翼自动完成起飞、悬停和降落。

需要解决三个具体问题：

1. 仿真由哪几部分组成，各自负责什么（物理环境、飞控固件、任务程序）。
2. PX4 SITL、Gazebo 和 ROS2 节点之间如何通信，`MicroXRCEAgent` 在这条链路上的位置。
3. 如何判断一次飞行是否真的成功，而不是只看窗口里的动画。

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

读者需要会 Linux 基本命令（`cd`、`ls`、`grep`）和 Python 基础语法。本章不要求任何 ROS2 或 PX4 经验，下面这些概念会在正文中首次出现时就地解释。

## 本章涉及的新概念

| 术语 | 一句话解释 | 详细说明 |
|---|---|---|
| **Gazebo Harmonic** | 仿真器，负责物理、传感器和画面；课程里运行 `gz sim` | 1.1 节 |
| **PX4 SITL** | 飞控固件的"机上仿真"版本，和真机同一份源码，只是编译成 Linux 程序 | 1.1 节 |
| **ROS2 节点** | 你写的任务程序，通过话题收发消息 | 1.1 节 |
| **话题（Topic）** | 命名的消息通道；发布者写、订阅者读，例如 `/x500_0/fmu/out/vehicle_status` | 1.2 节 |
| **MicroXRCEAgent** | 把 PX4 的 uXRCE-DDS 协议转成 ROS2 能直接用的 DDS 话题的代理程序 | 1.2 节 |
| **命名空间** | 多机时区分不同飞机的前缀，如 `/x500_0`；由机型名去掉 `gz_` 加实例号得到 | 1.3 节 |
| **Offboard 模式** | 由外部程序接管控制权的飞行模式，本章先用，第 2 章讲机制 | 1.3 节 |
| **判分指标** | 可核验的阈值（如高度误差 < 0.4m），由 `tools/grade.py` 自动执行 | 实验 / 第 16 章 |

## 📖 主体内容

### 1.1 仿真由哪几部分组成

课程环境由三部分组成：

| 组件 | 职责 | 在运行时的形态 |
|---|---|---|
| **Gazebo Harmonic** | 物理世界：重力、碰撞、传感器数据、画面 | `gz sim` 进程，加载世界与机型模型 |
| **PX4 SITL** | 飞控固件：姿态控制、EKF 状态估计、模式管理 | `px4` 进程，固件编译为 Linux 程序 |
| **ROS2 节点** | 任务逻辑：发指令、做感知、跑算法 | 你写的节点，例如 `offboard_demo` |

Gazebo 里的模型只是飞机的物理替身，决定它怎么飞的是 PX4 中运行的固件逻辑。PX4 SITL 与真机固件来自同一份源码，因此在仿真里验证过的控制逻辑可以迁移到实机（迁移约束见第 15 章）。

**为什么需要分成三块，而不是写成一个程序？**

这是无人机工程的标准分工，原因有三点：

1. **飞控不该被任务逻辑拖累**。姿态控制需要 250Hz–1kHz 的稳定周期，如果和视觉算法跑在同一进程里，一次图像推理卡顿就可能让飞机失稳。PX4 作为独立进程，保证控制回路不受影响。
2. **物理环境要可替换**。Gazebo 负责重力和碰撞，换仿真器（或接真机）时，PX4 与你的节点都不用改——这正是 SITL 的意义：仿真与实机共用同一份固件。
3. **算法要能独立开发**。你的 ROS2 节点只依赖话题，不依赖飞控内部实现，因此第 7 章的 SLAM、第 13 章的集群可以逐步叠加，而不用重写底层。

理解这个分工后，就能明白后面所有实验都是在"第三块"上加东西，前两块保持不变。

**`px4` 进程与真机的关系**

SITL 是 Software-In-The-Loop 的缩写：把飞控固件编译成普通 Linux 程序，用仿真器替代真实传感器和执行器。因此：

- 固件里的控制律、EKF、模式管理逻辑，与真机完全一致；
- 差别只在数据来源：仿真里传感器数据来自 Gazebo，真机上来自实际硬件；
- 所以本章跑通的起飞逻辑，参数含义与真机一致，只是真机还需处理标定、延迟和振动（见第 15 章）。

### 1.2 数据链路与 MicroXRCEAgent

PX4 通过 uXRCE-DDS 与外部通信，ROS2 节点使用 DDS，两者之间需要代理：

<!-- IMAGE_PLACEHOLDER
id: fig-ch01-data-link
caption: 图 1.1 课程数据链路：Gazebo、PX4 SITL、MicroXRCEAgent 与 ROS2 节点之间的消息方向
alt: 数据链路示意图，Gazebo 提供传感器与执行器接口，PX4 SITL 经 uXRCE-DDS 连接 MicroXRCEAgent，Agent 再以 DDS 连接 ROS2 节点
prompt: |
  技术教材插图，白底，扁平化风格，中文标注，无真人无卡通无水印。横向四段架构图：
  第 1 段标 "Gazebo Harmonic（物理世界：模型、传感器、画面）"；
  第 2 段标 "PX4 SITL（飞控固件：姿态控制、EKF）"，与第 1 段之间用双向箭头标 "传感器数据 / 电机指令"；
  第 3 段标 "MicroXRCEAgent（协议代理，UDP 8888）"，与第 2 段之间双向箭头标 "uXRCE-DDS"；
  第 4 段标 "ROS2 节点（你的任务程序）"，与第 3 段之间双向箭头标 "DDS 话题 /fmu/..."。
  在第 4 段上方用虚线框标注示例话题 "/x500_0/fmu/out/vehicle_status"。
  组件用圆角矩形，箭头方向清晰，配色只用蓝灰两色。
-->
![图 1.1 课程数据链路](images/PLACEHOLDER-fig-ch01-data-link.png)

没有 Agent，节点看不到 `/fmu/...` 话题，PX4 也收不到指令。可以用下面的命令确认链路是否通：

```bash
ros2 topic list | grep fmu
```

栈未启动时该命令无输出；栈启动后应能看到 `/x500_0/fmu/...` 系列话题。

关于"话题"：可以把它理解成按名字区分的消息通道，程序向某个名字发布消息，其他程序从同一个名字读取。`/x500_0/fmu/out/vehicle_status` 就是 PX4 对外发布自身状态（是否解锁、当前模式）的通道。

**为什么 PX4 不能直接和 ROS2 说话？**

PX4 是一个飞控固件，它对外用的是 uXRCE-DDS 协议（嵌入式场景的轻量 DDS）；ROS2 节点之间用的是标准 DDS。两者协议不同，需要一个"翻译+中转"的进程，这就是 `MicroXRCEAgent`：

- 它监听 UDP 8888，与 PX4 建立会话；
- 把 PX4 发来的数据转成 ROS2 能直接订阅的 DDS 话题；
- 反向把 ROS2 发布的指令转给 PX4。

因此它是**必需的中间环节**，不是可选优化。可以用下面的命令确认会话是否建立（日志中应出现 `create_client` 与 `session established`）：

```bash
tail -20 logs/xtd2_sim.log | grep -iE "create_client|session established"
```

**常用话题先看这三个**

| 话题 | 方向 | 内容 | 本章用途 |
|---|---|---|---|
| `/x500_0/fmu/out/vehicle_status` | PX4 → 节点 | 是否解锁（`arming_state`）、当前模式（`nav_state`） | 判断飞机状态 |
| `/x500_0/fmu/out/vehicle_local_position` | PX4 → 节点 | 局部坐标 x/y/z（NED，z 向下为正） | 看高度变化 |
| `/x500_0/fmu/in/trajectory_setpoint` | 节点 → PX4 | 目标位置/速度 | 由 `offboard_demo` 发布 |

查看某个话题的实时数据：

```bash
ros2 topic echo /x500_0/fmu/out/vehicle_status --once
```

`arming_state` 为 1 表示上锁，2 表示解锁；`nav_state` 为 14 表示处于 Offboard 模式。这两个字段是后续所有实验判断状态的基础。

### 1.3 `run_simulation.sh xtd2` 做了什么

一条命令按顺序拉起四部分：

1. `gz sim`：加载 aruco 世界与 x500 模型。
2. `MicroXRCEAgent udp4 -p 8888`：DDS 代理。
3. `px4 -d -s .../rcS`：飞控固件，通过 gz 插件接入模型。
4. `ros_gz_bridge` 与通信节点：把 Gazebo 传感器数据转成 ROS2 话题。

脚本负责处理启动顺序、命名空间推导和退出清理。手动启动时若顺序错误（例如先启动 PX4、后启动 Agent），飞控会连不上代理。

### 1.4 环境可复现

环境按锁定版本复现：PX4 v1.15.4、`px4_msgs` release/1.15、XTDrone2 锁定 commit，并叠加 18 个补丁，把已踩过的坑固化（GStreamer 冲突、Qt5 缺失、QoS 死锁、多机工作目录冲突等）。对应实现见 `scripts/setup_workspace.sh` 与 `patches/`。

## 实验实战

**观察：确认仿真器与代理可用**

```bash
gz sim --version        # 期望: Gazebo Sim, version 8.15.0
ls -l ~/.local/bin/gz   # 期望: 指向 /usr/bin/gz-classic 的符号链接
```

**验证：启动整条链路**

```bash
bash scripts/run_simulation.sh xtd2
```

预期输出（节选，10–40 秒内陆续出现）：

```text
[px4_launch-3] INFO  [gz_bridge] world: aruco, model name: x500_0, simulation model: x500
[microxrceagent-2] info | create_client | client_key: 0x00000001
[px4_launch-3] INFO  [commander] Ready for takeoff!
```

出现 `Ready for takeoff!` 说明物理世界、飞控和 DDS 代理都已就绪。

**验证：让飞机自动飞一次**（新终端）

```bash
source /opt/ros/humble/setup.bash
source <仓库路径>/ros2_ws/install/setup.bash
ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0 -p circle_duration:=0.0
```

预期输出：

```text
[drone_sim_offboard_demo]: 请求 Offboard 模式 + 解锁 ...
[drone_sim_offboard_demo]: 已解锁并进入 Offboard, 起飞至 5.0 m
[drone_sim_offboard_demo]: 到达目标高度, 开始画圆
[drone_sim_offboard_demo]: 已降落并上锁, 演示完成 ✔
```

Gazebo 窗口中应看到飞机垂直爬升到 5m，随后降落。各阶段对应的可观察状态如下：

| 阶段 | 日志标志 | `vehicle_status` 变化 | Gazebo 画面 |
|---|---|---|---|
| 预流与请求 | `请求 Offboard 模式 + 解锁 ...` | `nav_state` 变为 Offboard(14) | 飞机静止在地面 |
| 起飞 | `已解锁并进入 Offboard, 起飞至 5.0 m` | `arming_state` 由 1 变 2 | 螺旋桨转动，高度上升 |
| 到达高度 | `到达目标高度, 开始画圆` | 保持解锁与 Offboard | 悬停在约 5m |
| 降落上锁 | `已降落并上锁, 演示完成 ✔` | `arming_state` 回到 1 | 回到地面，桨停转 |

<!-- IMAGE_PLACEHOLDER
id: fig-ch01-flight-phases
caption: 图 1.2 首飞四个阶段对应的日志、状态位与画面变化
alt: 首飞阶段对照图，横轴为时间，标出预流请求、起飞、到达高度、降落上锁四个阶段，并对应日志标志、arming_state/nav_state 与 Gazebo 画面
prompt: |
  技术教材插图，白底，扁平化风格，中文标注，无真人无卡通无水印。
  横向时间轴分为四段，依次标注："1 预流与请求"、"2 起飞"、"3 到达高度"、"4 降落上锁"。
  每段下方用三行小字注明：
  第 1 段 "日志: 请求 Offboard 模式 + 解锁；状态: nav_state=14；画面: 静止在地面"；
  第 2 段 "日志: 已解锁并进入 Offboard；状态: arming_state 1→2；画面: 高度上升"；
  第 3 段 "日志: 到达目标高度；状态: 保持解锁；画面: 悬停约 5m"；
  第 4 段 "日志: 已降落并上锁；状态: arming_state=1；画面: 回到地面"。
  在右侧画一个简化四旋翼高度曲线，随四段先平、上升、平、下降。配色只用蓝灰，标注清晰。
-->
![图 1.2 首飞阶段与可观察状态](images/PLACEHOLDER-fig-ch01-flight-phases.png)

![首飞截图](images/04_xtd2_x500_takeoff.png)

**分析：用判分系统给出可核验结果**

```bash
python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1 --json out.json --md out.md
```

`out.json` 中每个指标都有数值和 PASS/FAIL；判分规则见第 16 章。

**阅读**：打开 `ros2_ws/src/drone_sim/drone_sim/offboard_demo.py`，查看 `loop()` 中请求解锁前的条件判断。该逻辑在第 2 章展开。

## 场景案例

下面四个案例都来自本仓库的实测记录，数据可在 `logs/grade/` 与 `catalog/scenarios.yaml` 中对照。案例按"背景—做法—结果—要点"组织，帮助把套路迁移到你自己的任务上。

### 案例 1：标准首飞（对应场景 `ctrl.takeoff_hover_land`）

**背景**：需要验证整套环境可用，并给出可核验的飞行结果，而不是只看窗口里的动画。

**做法**：启动栈后运行 `offboard_demo`，关闭画圆（`circle_duration:=0.0`），让飞机只完成起飞—悬停—降落。

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

**要点**：这里的"高度误差 0.3m"是用 Gazebo 世界系真值算的。若改用 PX4 局部坐标，EKF 原点重置会让数字失真——这是第 4、7 章反复出现的教训。

### 案例 2：参数变体——改高度与画圆

**背景**：想让飞机飞 8m 并画 10 秒圆，确认参数化接口能覆盖不同任务，而不用改代码。

**做法**：用 `-p` 覆盖参数（完整参数见 `catalog/params.yaml`）：

```bash
ros2 run drone_sim offboard_demo --ros-args \
  -p px4_ns:=/x500_0 -p takeoff_altitude:=8.0 \
  -p circle_radius:=5.0 -p circle_speed:=0.5 -p circle_duration:=10.0
```

`offboard_demo` 的可调参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `takeoff_altitude` | 5.0 | 起飞高度（m） |
| `circle_radius` | 3.0 | 画圆半径（m） |
| `circle_speed` | 0.3 | 画圆角速度（rad/s） |
| `circle_duration` | 25.0 | 画圆时长（s，0 表示跳过） |
| `px4_ns` | `''` | 命名空间；单机留空由 launch 重映射，多机写 `/x500_0` |
| `target_system` | 1 | 多机时须等于 MAV_SYS_ID（实例号+1） |

**要点**：画圆耗时约为 `2π / circle_speed`，改角速度会明显改变总时长；把参数写进命令而不是源码，第 4 章航点任务、第 10 章行为树都沿用这个思路。

### 案例 3：故障复现——Agent 缺失导致"节点空转"

**背景**：某次运行只看到"等待 PX4 状态"，飞机不动，但不报明显错误。

**做法与现象**：

```bash
pkill -x MicroXRCEAgent                 # 制造故障：停掉代理
ros2 topic list | grep fmu              # 无输出：话题消失
ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0
# 日志反复出现 "等待 PX4 状态..."，飞机不动
bash scripts/run_simulation.sh xtd2      # 重启栈后恢复
```

**结果**：链路未通时，节点会一直等待状态消息；恢复 Agent 后首飞正常通过判分。

**要点**：排障顺序应为"先看代理在不在 → 再看话题在不在 → 最后看 QoS 与状态字段"。这个问题在多机（第 12 章）和 SLAM（第 7 章）里会以不同面貌出现，但排查路径相同。

### 案例 4：多机命名空间推导（为第 12 章预热）

**背景**：想知道多机时 `/x500_0`、`/x500_1` 这些前缀从哪来。

**做法**：机型名去掉 `gz_` 前缀，加下划线与实例号：

```text
gz_x500            → /x500_0
gz_x500_mono_cam   → /x500_mono_cam_0
gz_x500_lidar_3d   → /x500_lidar_3d_0
```

第 N 架飞机实例号就是 N，因此第二架是 `/x500_1`，其 `target_system` 必须为 2。

**要点**：本章先用单机熟悉 `/x500_0`；第 12 章双机编队时，命名空间写错会直接导致"2 号机不解锁"，是那一章的核心坑之一。

## 任务卡

### 任务卡 1：复现一次完整首飞

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 飞机自动起飞到 5m、悬停、降落上锁，日志出现"演示完成 ✔" |
| 2 通信骨架/前置 | 栈已启动（`run_simulation.sh xtd2`）；节点发 `/x500_0/fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}`，收 `/x500_0/fmu/out/{vehicle_status, vehicle_local_position}` |
| 3 参考 prompt | "你是 ROS2 无人机助教。我运行 `ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0` 后飞机没有起飞。请按'话题→QoS→模式→命���'顺序列出你的排查步骤，每条给出要执行的 ros2 命令，先给命令清单，不要讲理论。" |
| 4 执行步骤 | ① 启动栈并等到 `Ready for takeoff!` ② 新终端 source 两层环境 ③ 跑上面的命令 ④ 观察日志与 Gazebo 窗口 ⑤ 用 `ros2 topic echo /x500_0/fmu/out/vehicle_status --once` 看 `arming_state` 是否变 2 再变 1 |
| 5 验收标准 | □ 日志四行齐全 □ 窗口看到爬升与降落 □ `grade.py` 三项指标 PASS □ 能用一句话解释 `arming_state` 1 和 2 的含义 |
| 6 原理 + 扩学 | 教的是**整条链路可用性**。扩学：把 `circle_duration` 设为 10，观察画圆时遥测 x/y 是否落在半径 3m 的圆上 |

### 任务卡 2：参数改造（改动一个小旋钮）

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 让飞机起飞到 8m 并画 10 秒圆 |
| 2 通信骨架/前置 | 同任务卡 1；提示：所有旋钮都是 ROS2 参数，用 `-p 名字:=值` 覆盖 |
| 3 参考 prompt | "给我一条 `ros2 run` 命令：用 drone_sim 的 offboard_demo，起飞高度改 8m、画圆时长改 10s、画圆半径 5m。说明每个 `-p` 参数对应的默认值和单位。" |
| 4 执行步骤 | 先用 `ros2 param list` 看不到（节点未运行）→ 改从源码 `grep declare_parameter offboard_demo.py` 找参数名 → 再跑 |
| 5 验收标准 | □ 飞机到达 8m（遥测 z≈-8）□ 画圆半径≈5m □ `grade.py --scenario ctrl.takeoff_hover_land` 仍 PASS |
| 6 原理 + 扩学 | 教的是**参数化优于改代码**。扩学：改源码加一个 `yaw_rate` 参数，让飞机边飞边转 |

### 任务卡 3：故障注入（本章的"刺"）

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 亲手制造并修复一个起步故障 |
| 2 通信骨架/前置 | 故意制造：先用 `pkill -x MicroXRCEAgent` 杀掉 Agent，再重跑 offboard_demo |
| 3 参考 prompt | "我的 ROS2 节点一直打印'等待 PX4 状态...'，`ros2 topic list` 里看不到 /x500_0/fmu 话题。请列出三个最可能的原因和验证命令。" |
| 4 执行步骤 | ① 杀 Agent ② 跑节点观察"等待 PX4 状态" ③ 用 `ros2 topic list | grep fmu` 验证话题消失 ④ 重启 Agent 后恢复 |
| 5 验收标准 | □ 能复现"无话题、节点空转" □ 能说出 Agent 在链路中的位置 □ 恢复后首飞 PASS |
| 6 原理 + 扩学 | 教的是**链路分段排障法**：先看桥在不在，再看话题在不在，最后看 QoS 和模式 |

## ✅ 检查清单

- [ ] 我能画出 `Gazebo ↔ PX4 ↔ Agent ↔ 节点` 四段链路，并说出每段的作用
- [ ] 我能独立完成一次首飞，并解释日志里每一行的含义
- [ ] 我能用 `grade.py` 给自己的飞行打分
- [ ] 我能改参数飞出一个不同高度/半径的轨迹
- [ ] 我能复现并修复"Agent 缺失导致话题消失"的故障
- [ ] 我知道 `Ready for takeoff!` 出现的三个前提条件

## 小结与下一步

本章完成了环境复现和一次自动起降，并用判分系统获得可核验的指标。

下一步需要先理解飞机为什么听话：`offboard_demo.py` 在请求解锁前的条件判断、NED 坐标系下的 `z` 符号，以及 EKF 原点重置对定位的影响。这些内容在第 2 章说明。

[下一章：Offboard 控制与 PX4Interface →](course/02-offboard.md)
