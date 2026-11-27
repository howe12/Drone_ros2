# 01 环境搭建与首飞

> 难度 L0 · 对应场景 `ctrl.takeoff_hover_land` · 预计 1–2 小时（含编译）

## ❓ 驱动问题

你拿到一个 GitHub 仓库，README 第一行写着"三条命令就能飞"。你照着敲完，屏幕上确实开出了一个黑乎乎的 3D 窗口，
一个橙色的四旋翼模型停在地上——但接下来呢？

- 是谁在让螺旋桨转？窗口里那个软件，还是仓库里那堆 C++？
- `MicroXRCEAgent` 又是什么？没有它飞机就不动吗？
- 为什么日志里冒出一句 `Ready for takeoff!` 之后，就没有下文了？

从一台干净的 Ubuntu，到你亲眼看见一架飞机自己起飞、悬停、降落，中间到底要拧紧几颗螺丝？

**这就是本章要回答的问题。**

## 📚 上一章回顾

本课从零开始，无需回顾。

只提前告诉你一件事：我们会在这章亲手点亮**整条数据链路**，它是后面 15 章所有实验的地基——
第 7 章的 SLAM 建图、第 12 章的双机编队，用的都是同一条链路，只是换了几台飞机、多了几个算法节点。

## 🎯 学习目标

| 维度 | ❌ 学习前 | ✅ 学习后 |
|---|---|---|
| 认知定位 | 以为仿真就是"打开 Gazebo 看个动画" | 理解 Gazebo 只是物理世界，飞控另有其人 |
| 链路理解 | 说不清 PX4、Gazebo、ROS2 谁跟谁说话 | 能画出 `Gazebo ↔ PX4 ↔ Agent ↔ 你的节点` 四段链路 |
| 环境能力 | 环境装到哪一步全靠运气 | 能用锁版本+补丁的方式，可复现地还原环境 |
| 动手能力 | 只会复制别人的命令 | 能改参数跑出不同高度/半径的飞行 |
| 排障能力 | 报错就懵 | 能定位"端口被占/缺库/解锁被拒"三类起步故障 |

## 📖 主体内容

### 1.1 三块拼图：谁在飞

无人机仿真从来不是"一个软件"。它是三套东西咬合在一起：

| 拼图 | 角色 | 类比 |
|---|---|---|
| **Gazebo Harmonic** | 物理世界：重力、碰撞、传感器、画面 | 飞行场地 + 天气 + 你的眼睛 |
| **PX4 SITL** | 飞控固件：姿态控制、EKF 状态估计、模式管理 | 飞机上的飞控电脑 |
| **ROS2 + 你的节点** | 任务大脑：算航线、做视觉、发指令 | 站在地面拿遥控器的你 |

❌ 常见误解：以为 Gazebo 里那个模型是"飞机本体"。其实模型只是**物理替身**——
真正决定它怎么飞的是 PX4 里跑的固件代码（SITL 就是把固件编译成 Linux 程序来跑）。

✅ 正确理解：Gazebo 提供"身体与感官"，PX4 提供"小脑与脊髓"，你的 ROS2 节点提供"大脑"。

> 🔑 **PX4 SITL 与真机固件是同一份源码**——你在仿真里验证的飞行逻辑，下一章之后可以原样搬到真机。

### 1.2 第四段链路：为什么必须要一个 Agent

PX4 说的是 uXRCE-DDS 协议，ROS2 说的是 DDS 网络。两者需要一座桥：

```
Gazebo ──传感器/执行器── PX4 SITL ──uXRCE-DDS── MicroXRCEAgent ──DDS── 你的 ROS2 节点
```

❌ 常见误解：以为 Agent 是"可选的中间件"。
✅ 事实：**没有 Agent，你的节点看不到任何 `/fmu/...` 话题**，飞控也不会收到你的指令。

```bash
# 验证：栈启动前后各查一次，话题列表的差别就是 Agent 带来的
ros2 topic list | grep fmu
```

> 🔑 记住这条链路的顺序，第 9 章的避障、第 13 章的集群，都是在这条链路上加节点。

### 1.3 一条命令背后发生了什么

`bash scripts/run_simulation.sh xtd2` 看起来只有一行，实际按顺序拉起四样东西：

1. `gz sim`：加载 aruco 世界 + x500 模型（物理与渲染）
2. `MicroXRCEAgent udp4 -p 8888`：DDS 桥
3. `px4 -d -s .../rcS`：飞控固件，通过 gz 插件接入模型
4. `ros_gz_bridge` + 通信节点：把传感器转成 ROS2 话题

❌ 常见做法：手动一个个敲，顺序错了就卡住（Agent 没起就启动 PX4 → 飞控连不上）。
✅ 正确做法：用脚本统一拉起，它已经处理了依赖顺序、命名空间推导与退出清理。

### 1.4 可复现是硬要求

课程里所有环境都必须**锁版本**：PX4 v1.15.4、`px4_msgs` release/1.15、XTDrone2 锁定 commit。
再叠加 18 个补丁，把社区踩过的坑固化（GStreamer 冲突、Qt5 缺失、QoS 死锁、多机工作目录冲突……）。

❌ "我的机器能跑"：换个 CUDA/系统版本就玄学失败。
✅ "任何机器都能跑"：版本锁定 + 补丁 + 复现脚本 + 判分标准。

## 🔬 实验实战

**阶段 1 观察** — 先看清机器上装了什么

```bash
gz sim --version        # 期望: Gazebo Sim, version 8.15.0
ls -l ~/.local/bin/gz   # 期望: 指向 /usr/bin/gz-classic 的符号链接
```

**阶段 2 探索** — 启动整条链路，认人

```bash
bash scripts/run_simulation.sh xtd2
```

预期输出（节选，10–40 秒内陆续出现）：

```text
[px4_launch-3] INFO  [gz_bridge] world: aruco, model name: x500_0, simulation model: x500
[microxrceagent-2] info | create_client | client_key: 0x00000001
[px4_launch-3] INFO  [commander] Ready for takeoff!
```

看到 `Ready for takeoff!` 就说明：物理世界在跑、飞控已启动、DDS 桥已接通。

**阶段 3 交互** — 让飞机自己飞一次（新终端）

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

同时 Gazebo 窗口里应该看到飞机垂直爬升到 5m，然后缓慢降落。

![首飞](images/04_xtd2_x500_takeoff.png)

**阶段 4 导出** — 用判分系统给自己的飞行打分

```bash
python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1 --json out.json --md out.md
```

预期：终端打印指标表，`out.json` 里每个指标有数值与 PASS/FAIL。

**阶段 5 阅读** — 打开 `ros2_ws/src/drone_sim/drone_sim/offboard_demo.py`，
找到 `loop()` 里的状态机分支，回答：它在什么条件下才发解锁命令？

**阶段 6 反思** — 把四段链路（Gazebo → PX4 → Agent → 节点）讲给一个没学过 ROS 的同学，
对方能复述出"谁跟谁说话、为什么需要 Agent"，本阶段才算通过。

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

## ❓ 新问题

飞起来了，但你其实还没搞懂飞机**为什么**听话：为什么必须先发心跳？为什么它只听"设定点"不听"位置"？
更奇怪的是——为什么有的同学第一次飞，飞机会贴着地面"爬行"？

这三个现象指向同一套机制：Offboard 模式与 NED 坐标系。

[下一章：Offboard 控制与 PX4Interface →](course/02-offboard.md)
