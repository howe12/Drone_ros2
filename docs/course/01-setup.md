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

**命令小课堂（零基础先看这里）**：

- `sudo`：以管理员权限执行命令。安装软件需要它，会提示输入密码。
- **编译**：把源代码变成可执行程序的过程。PX4 是大型 C++ 工程，首次编译要 1–2 小时；之后只重编改动的部分，快得多。
- `source xxx/setup.bash`：把环境变量加载进**当前终端**，让它能找到 ROS2 命令和编译好的包。**每开一个新终端都要先 source**，否则 `ros2` 命令会报"找不到"。
- **工作空间**（`ros2_ws`）：放你的代码和编译产物的目录；`install/setup.bash` 是编译完成后生成的环境入口，source 它才能 `ros2 run` 你写的节点。

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

日志里有两个名词：**世界（world）**是场景——地面、障碍、靶标；`aruco` 是一个地面放着 ArUco 方形靶标的世界（第 6 章精准降落要用它）。**模型（model）**是飞机的三维外形加物理参数；`x500` 是 PX4 的标准四旋翼模型，"500"指 **500mm 轴距**（对角两个电机的间距），这是行业常见的机架尺寸级别。

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

**看懂这段日志的四件事**：

- **解锁 / 上锁（arm / disarm）**：解锁 = 允许电机转动，飞机才能飞；上锁 = 电机停转。出于安全，飞机在地面默认上锁，起飞前必须先解锁。
- **Offboard 模式**：一种"由外部程序接管控制"的飞行模式——你的节点发目标，PX4 负责执行。它为什么需要心跳和预流，第 2 章展开。
- **为什么高度写成 -5.0**：PX4 用 NED 坐标（北-东-地），**z 向下为正**，所以 5m 高度写作 z=-5；Gazebo 用相反的 ENU（z 向上为正）。先记住这一句，第 2 章细讲。
- **解锁被拒怎么办**：PX4 解锁前会做**预检**（preflight：EKF 是否收敛、传感器是否健康）。预检不过就拒绝解锁——等 10–20 秒让估计收敛再试，通常就好。

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
| **PX4 SITL** | 飞控固件：**姿态控制**（让飞机保持平稳）、**EKF**（融合传感器估计"我在哪、什么姿态"）、**模式管理**（决定此刻谁在控制飞机） | `px4` 进程（固件编译为 Linux 程序） |
| **ROS2 节点** | 任务逻辑：发指令、做感知 | 你写的节点，如 `offboard_demo` |

**先补两个背景事实**

无人机按升力来源主要分三类：**多旋翼**（多个螺旋桨直接向上拉，能悬停，续航短）、**固定翼**（机翼靠前进时的气流产生升力，不能悬停，续航长）、**复合翼 VTOL**（多旋翼垂直起降 + 固定翼巡航）。本课程从多旋翼开始，因为它能悬停、位置指令直接有效、失败现象明显，最适合学控制。分类对比图见[第 00 章](course/00-basics.md)。

四旋翼怎么飞？四个桨**两顺两逆**转（抵消反扭矩，机身才不自转），一切动作靠**转速差**：四桨同加速 = 上升；前后差速 = 俯仰；左右差速 = 横滚；顺逆两组打破平衡 = 转向。关键一点：**想水平移动，先倾斜姿态**——低头后升力向前倾，才产生向前的分力。所以飞控分两层：外层算"该倾斜多少"，内层算"电机怎么转才能保持这个倾斜"。

**表中三个职责到底指什么**

- **姿态控制**：姿态 = 飞机在空中的朝向，用三个角描述——横滚 Roll（左右侧倾）、俯仰 Pitch（机头抬低）、偏航 Yaw（原地转向）。姿态控制是内环，以数百 Hz 的频率让飞机保持目标朝向，直接输出电机转速。
- **EKF（扩展卡尔曼滤波）**：GPS 会漂移、IMU 会累积误差，各传感器的测量互相矛盾；EKF 把它们融合成"最可能真实"的位置/速度/姿态。副作用：GPS 跳变太大时它会**重置局部原点**，局部坐标随之"瞬移"——这是后面章节"贴地爬行"坑的根源，也是判分坚持用 Gazebo 世界系真值的原因。
- **模式管理**：飞控是个状态机，同一时刻只处于一种模式——人工（遥控）、Position（遥控+飞控保位置）、**Offboard（你的程序）**、RTL（自动返航）、LAND（自动降落）。为什么需要模式：安全兜底——程序崩了，飞控能切回安全模式，而不是跟着失控。

**为什么分成三块？**

1. **飞控不能被任务逻辑拖累**：姿态控制需要 250Hz–1kHz 稳定周期，和视觉算法同进程会因卡顿失稳。
2. **物理环境可替换**：换仿真器或接真机时，PX4 与你的节点都不用改——SITL 与真机共用同一份固件，差别只在数据来源。
3. **算法可独立叠加**：你的节点只依赖话题，因此第 7 章 SLAM、第 13 章集群可以逐步加，不必重写底层。

> 想更深入（内外环频率、EKF 数学直觉、飞控分层图、更多机型与命名由来），见[第 00 章 认识无人机与仿真](course/00-basics.md)；全部术语速查见[术语表](GLOSSARY.md)。本章内容已自足，不读第 00 章也能继续。

### 2.2 数据链路与 MicroXRCEAgent

先解释 **DDS**：一种"发布-订阅"通信标准——程序之间不直接调用，而是把消息发到**按名字区分的通道（话题）**，谁需要谁订阅。ROS2 底层就是它。PX4 跑在嵌入式硬件上，用的是它的轻量变体 **uXRCE-DDS**；两种"方言"不同，所以需要 `MicroXRCEAgent` 当翻译：

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
2. `MicroXRCEAgent udp4 -p 8888`：DDS 代理。`8888` 是**端口号**——同一台机器上区分不同服务的"门牌号"；排障时说的"端口被占"，就是别的程序已经用了这个门牌号
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

汇总 3/3 通过。（表中"中位数"：多次运行时取排序后的中间值，避免用单次碰巧的成绩判定；本例只跑 1 次，中位数即该次值。）

**要点**：高度误差用 Gazebo 世界系真值算。PX4 局部坐标经过 EKF 估计，原点重置会让坐标"瞬移"（见 2.1），数字随之失真——第 4、7 章会反复遇到。

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
| `px4_ns` | `''` | 命名空间（话题前缀）；单机留空由 launch 自动给话题改名，多机显式写 `/x500_0` |
| `target_system` | 1 | 目标飞机编号（MAV_SYS_ID）；多机时命令必须发给对应编号，第 N 架的编号是 N+1 |

**要点**：`circle_speed` 是**角速度**——每秒转多少弧度。0.3 rad/s ≈ 17°/s，转一整圈（2π≈6.28 弧度）约 21 秒，所以画圆耗时 ≈ `2π / circle_speed`。扩学里"观察遥测 x/y"的**遥测**，指飞机实时回传的数据（这里就是 `vehicle_local_position` 话题）。参数写进命令而非源码，第 4 章航点、第 10 章行为树沿用此思路。

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

**要点**：排障顺序为"先看代理在不在 → 再看话题在不在 → 最后看 QoS 与状态字段"。**QoS** 是 ROS2 的通信可靠性策略，两端设置不匹配会静默收不到消息（第 2 章详讲）。此路径在第 7、12 章同样适用。

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
| 3 AI 提示 | 先自己判断问题可能在代理/话题/模式哪一段，再问 AI：<br>"你是 ROS2 无人机助教。环境 PX4 v1.15.4 / ROS2 Humble / Gazebo Harmonic，命名空间 `/x500_0`，发布端 QoS 为 BEST_EFFORT+VOLATILE。我运行 `ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0` 后飞机没起飞。请按'话题→QoS→模式→命令'给排查步骤，每条附要执行的 ros2 命令，先给命令清单。" |
| 4 执行步骤 | ① 启动栈等到 `Ready for takeoff!` ② 新终端 source 两层环境 ③ 跑命令 ④ 观察日志与 Gazebo ⑤ `ros2 topic echo /x500_0/fmu/out/vehicle_status --once` 看 `arming_state` 是否 1→2→1<br>**验证 AI 的说法**：用 `ros2 topic list \| grep fmu`、`ps -ef \| grep MicroXRCEAgent` 确认代理与话题真实存在——AI 可能忽略 Agent 或用错话题前缀 |
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

## ✅ 检查清单

- [ ] 我能独立复现环境，并知道编译失败时看哪个日志
- [ ] 我能启动仿真栈并用 `ros2 topic list | grep fmu` 确认链路已通
- [ ] 我能完成一次首飞，并用 `grade.py` 拿到 3/3 PASS 报告
- [ ] 我能改参数（高度/半径/角速度）飞出不同轨迹，并预判耗时变化
- [ ] 我能复现并修复"Agent 缺失导致话题消失"的故障
- [ ] 我能说出 Gazebo / PX4 SITL / ROS2 节点三者的分工
- [ ] 若用了 AI 辅助，我能解释它给出的命令或代码，并用上面的命令验证过

> 本课程所有任务都可以用 AI 辅助（解释报错、生成片段、优化思路），方法见 [AI 协作指南](AI_GUIDE.md)。
> 注意：本课程锁版本且打过补丁，AI 的通用答案可能不适用——**要用命令验证**，不要直接照抄。

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
