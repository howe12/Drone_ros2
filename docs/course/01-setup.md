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

| 维度 | ❌ 学习前 | ✅ 学习后 |
|---|---|---|
| 认知定位 | 以为仿真就是"打开 Gazebo 看个动画" | 理解 Gazebo 只是物理世界，飞控另有其人 |
| 链路理解 | 说不清 PX4、Gazebo、ROS2 谁跟谁说话 | 能画出 `Gazebo ↔ PX4 ↔ Agent ↔ 你的节点` 四段链路 |
| 环境能力 | 环境装到哪一步全靠运气 | 能用锁版本+补丁的方式，可复现地还原环境 |
| 动手能力 | 只会复制别人的命令 | 能改参数跑出不同高度/半径的飞行 |
| 排障能力 | 报错就懵 | 能定位"端口被占/缺库/解锁被拒"三类起步故障 |

## 📖 主体内容

### 1.1 仿真由哪几部分组成

课程环境由三部分组成：

| 组件 | 职责 | 在运行时的形态 |
|---|---|---|
| **Gazebo Harmonic** | 物理世界：重力、碰撞、传感器数据、画面 | `gz sim` 进程，加载世界与机型模型 |
| **PX4 SITL** | 飞控固件：姿态控制、EKF 状态估计、模式管理 | `px4` 进程，固件编译为 Linux 程序 |
| **ROS2 节点** | 任务逻辑：发指令、做感知、跑算法 | 你写的节点，例如 `offboard_demo` |

Gazebo 里的模型只是飞机的物理替身，决定它怎么飞的是 PX4 中运行的固件逻辑。PX4 SITL 与真机固件来自同一份源码，因此在仿真里验证过的控制逻辑可以迁移到实机（迁移约束见第 15 章）。

### 1.2 数据链路与 MicroXRCEAgent

PX4 通过 uXRCE-DDS 与外部通信，ROS2 节点使用 DDS，两者之间需要代理：

```
Gazebo ──传感器/执行器── PX4 SITL ──uXRCE-DDS── MicroXRCEAgent ──DDS── 你的 ROS2 节点
```

没有 Agent，节点看不到 `/fmu/...` 话题，PX4 也收不到指令。可以用下面的命令确认链路是否通：

```bash
ros2 topic list | grep fmu
```

栈未启动时该命令无输出；栈启动后应能看到 `/x500_0/fmu/...` 系列话题。

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

Gazebo 窗口中应看到飞机垂直爬升到 5m，随后降落。

![首飞](images/04_xtd2_x500_takeoff.png)

**分析：用判分系统给出可核验结果**

```bash
python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1 --json out.json --md out.md
```

`out.json` 中每个指标都有数值和 PASS/FAIL；判分规则见第 16 章。

**阅读**：打开 `ros2_ws/src/drone_sim/drone_sim/offboard_demo.py`，查看 `loop()` 中请求解锁前的条件判断。该逻辑在第 2 章展开。

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
