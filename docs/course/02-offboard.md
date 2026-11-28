# 02 Offboard 控制与 PX4Interface

> 难度 L0–L1 · 代码导读章 · 后续所有实验的地基

## 本章问题

第 1 章你已经让飞机飞起来了，用的是现成的 `offboard_demo`。但换一个任务（比如飞到指定点后悬停），就要自己写节点。本章解决三个问题：

1. PX4 为什么愿意听一个外部程序的话，需要满足什么条件（Offboard 三要素）。
2. 为什么课程要求所有 PX4 交互都走 `PX4Interface`，而不是自己建发布者。
3. 怎么读懂 `offboard_demo` 的状态机，并照着写出自己的第一个任务节点。

理解这三件事后，第 4 章的航点、第 9 章的避障、第 12 章的编队，都是同一个套路的扩展。

## 学习目标

| 维度 | 学习前 | 学习后 |
|---|---|---|
| Offboard 机制 | 以为发个位置指令飞机就会动 | 知道需要心跳、设定点、模式/解锁三个动作且顺序不能错 |
| 坐标系 | 不明白 z 为什么是负的 | 能说清 NED 与 ENU 差异，知道 `alt()` 与 `pos.z` 的符号关系 |
| 接口使用 | 自己 `create_publisher` 连 PX4 | 统一使用 `PX4Interface`，避开 QoS/解锁/坐标四类历史故障 |
| 代码阅读 | 看不懂状态机 | 能照状态机改出"起飞—悬停—降落"节点 |
| 排障 | 遇到"贴地爬行""连不上"无思路 | 能用 EKF 门控、QoS、命令路由解释并定位 |

## 前置与回顾

第 1 章你已经：跑通整条链路、完成一次自动起降、用 `grade.py` 拿到 3/3 PASS。

本章依赖这些产出：`ros2 topic list | grep fmu` 能看到话题；`ros2 topic echo /x500_0/fmu/out/vehicle_status --once` 能看到 `arming_state` 和 `nav_state`。如果这两步还不通，先回第 1 章排障。

## 本章地图

| 步骤 | 你做什么 | 拿到的证据 |
|---|---|---|
| 1 | 观察飞行中 `arming_state` / `nav_state` 变化 | 状态值 1→2→1、`nav_state=14` |
| 2 | 用 `PX4Interface` 写 30 行 `hover_node` | 飞机起飞到 3m 再降落 |
| 3 | 理解 Offboard 三要素与顺序 | 能说出为什么必须先预流 |
| 4 | 理解 NED 与 `alt()` 的符号关系 | 能解释 `pos.z=-5` 与 `alt()=5` |
| 5 | 复现 QoS 死锁与 EKF 重置坑 | 能定位并修复 |

## 本章涉及的新概念

| 术语 | 一句话解释 | 详细说明 |
|---|---|---|
| **Offboard 模式** | 由外部程序接管控制权的飞行模式 | 2.2 节 |
| **心跳（Heartbeat）** | 持续发布的控制模式消息，证明"外部控制器还活着" | 2.2 节 |
| **设定点（Setpoint）** | 你希望飞机达到的目标位置/速度 | 2.2 节 |
| **预流（Pre-stream）** | 切 Offboard 前先持续发一段时间设定点 | 2.2 节 |
| **NED 坐标系** | 北-东-地，z 向下为正；与 Gazebo 的 ENU 相反 | 2.3 节 |
| **EKF** | PX4 内部的状态估计器，融合传感器算位置；原点可能被重置 | 2.4 节（门控） |
| **QoS** | ROS2 的通信可靠性策略；发布/订阅两端必须兼容 | 2.5 节 |
| **MAV_SYS_ID** | PX4 实例编号，多机时命令要发给对应编号（实例号+1） | 案例 4 / 第 12 章 |

**阅读顺序建议**：先按第 1 节跑通最小节点并观察状态变化；再读第 2 节理解"为什么必须这样"；最后用第 3 节案例练排障。

---

## 1. 先让它动起来（最小路径）

### 1.1 观察：飞机状态在飞行中如何变化

先启动栈（第 1 章已会）：

```bash
bash scripts/run_simulation.sh xtd2
```

新终端跑 `offboard_demo`，同时观察状态话题。开两个终端：

终端 A（飞）：

```bash
source /opt/ros/humble/setup.bash
source <仓库路径>/ros2_ws/install/setup.bash
ros2 run drone_sim offboard_demo --ros-args -p px4_ns:=/x500_0 -p circle_duration:=0.0
```

终端 B（看状态）：

```bash
ros2 topic echo /x500_0/fmu/out/vehicle_status --fields arming_state nav_state
```

你会看到 `arming_state` 从 1（上锁）变 2（解锁），`nav_state` 变为 14（Offboard），最后 `arming_state` 回到 1（降落上锁）。

<!-- SCREENSHOT_PLACEHOLDER
id: shot-ch02-status-echo
caption: 图 2.1 飞行中 vehicle_status 的 arming_state 与 nav_state 变化
when: 终端 A 跑 offboard_demo 期间，终端 B 执行上面 echo 命令时
what: 截取终端 B 输出，需能看到 arming_state 1→2→1 与 nav_state 变为 14 的过程
how: |
  ros2 topic echo /x500_0/fmu/out/vehicle_status --fields arming_state nav_state
  在飞机起飞后截取终端输出
-->
![图 2.1 状态变化](images/PLACEHOLDER-shot-ch02-status-echo.png)

**图 2.1　飞行过程中的状态变化：解锁后 `arming_state` 变 2，切 Offboard 后 `nav_state` 变 14。**

### 1.2 动手：写一个自己的最小节点

目标：起飞到 3m，悬停一会儿后降落。用课程统一接口 `PX4Interface`，约 30 行。

新建文件 `ros2_ws/src/drone_sim/drone_sim/hover_node.py`：

```python
import rclpy
from rclpy.node import Node
from drone_sim.px4_interface import PX4Interface

class HoverNode(Node):
    def __init__(self):
        super().__init__('hover_node')
        self.declare_parameter('target_system', 1)
        self.px4 = PX4Interface(
            self,
            ns='/x500_0',
            target_system=self.get_parameter('target_system').value,
        )
        self.create_timer(0.1, self.loop)   # 10Hz，必须 > 2Hz

    def loop(self):
        # 心跳 + 设定点：每次循环都要发，不能只在起飞前发一次
        self.px4.heartbeat_position(0.0, 0.0, -3.0, yaw=0.0)
        if not self.px4.ready:
            return
        if not (self.px4.armed and self.px4.offboard):
            if self.px4.ekf_stable:              # EKF 稳定才请求解锁
                self.px4.request_offboard_and_arm()
            return
        if abs(self.px4.alt() - 3.0) < 0.3:      # 到 3m 后降落
            self.px4.land()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HoverNode())

if __name__ == '__main__':
    main()
```

在 `setup.py` 的 `console_scripts` 注册：

```python
'hover_node = drone_sim.hover_node:main',
```

重新编译并运行：

```bash
cd ros2_ws && colcon build --symlink-install --packages-select drone_sim
source install/setup.bash
ros2 run drone_sim hover_node --ros-args -p target_system:=1
```

**注意**：编译终端只 source humble，不要 source 本工作空间（否则报 override 警告，见第 1 章）。

---

## 2. 刚才发生了什么（回头理解原理）

### 2.1 Offboard 是什么

Offboard 是 PX4 的一种飞行模式：控制权从遥控/飞控内部逻辑，交给外部程序（你的 ROS2 节点）。

但 PX4 不轻信外部程序，它要求三个动作，且**顺序不能颠倒**：

| 动作 | 对应话题/命令 | 要求 |
|---|---|---|
| 1 心跳 | `OffboardControlMode` | 持续发布，> 2Hz（课程用 10Hz） |
| 2 设定点 | `TrajectorySetpoint` | 持续发布目标；**切 Offboard 前就要预流** |
| 3 模式+解锁 | `VehicleCommand` | `DO_SET_MODE(176, param1=1, param2=6)` 切 Offboard；`ARM_DISARM(400, param1=1)` 解锁 |

<!-- IMAGE_PLACEHOLDER
id: fig-ch02-offboard-loop
caption: 图 2.2 Offboard 控制闭环：节点、MicroXRCEAgent、PX4、Gazebo 之间的消息方向与三要素
alt: Offboard 控制闭环图，ROS2 节点发布 OffboardControlMode 与 TrajectorySetpoint 经 Agent 到 PX4，PX4 返回 vehicle_status 与 vehicle_local_position，PX4 再驱动 Gazebo 中模型
prompt: |
  技术教材插图，白底，扁平化风格，中文标注，无真人无卡通无水印。
  左侧 "ROS2 节点（你的任务程序）"，中间 "MicroXRCEAgent（UDP 8888）"，右侧 "PX4 SITL"，下方 "Gazebo（物理世界）"。
  从节点到 PX4 两条向右箭头，分别标 "OffboardControlMode（心跳, 10Hz）" 与 "TrajectorySetpoint（目标）"，再一条标 "VehicleCommand（切 Offboard / 解锁）"；
  从 PX4 到节点两条向左箭头，标 "vehicle_status（arming_state / nav_state）" 与 "vehicle_local_position（x/y/z）"；
  PX4 与 Gazebo 之间双向箭头标 "传感器数据 / 电机指令"。
  在节点旁用编号 1/2/3 标出三个动作顺序，并注明"预流 → 请求 Offboard → 解锁"。圆角矩形，蓝灰配色。
-->
![图 2.2 Offboard 控制闭环](images/PLACEHOLDER-fig-ch02-offboard-loop.png)

**为什么必须先预流？** 如果先发切模式命令、之后才开始发设定点，PX4 会发现"外部控制器没有持续给目标"，判定不安全，拒绝切换或触发 Offboard loss failsafe（掉线保护）。所以 `offboard_demo` 里有一句 `if self.tick > 20`——以 10Hz 计，就是先流满 2 秒。

### 2.2 NED 坐标系：为什么 z 是负的

PX4 本地坐标是 **NED**（North-East-Down，北-东-地）：**z 向下为正**。

- 飞 5m 高：PX4 侧写作 `z = -5.0`
- Gazebo 世界系是 **ENU**（东-北-上）：z 向上为正

两者 z 符号相反、x/y 轴也互换，转换用接口提供的 `enu_to_ned()`。

接口里的两个易混字段：

| 写法 | 含义 | 符号 |
|---|---|---|
| `pos.z` | PX4 局部坐标原始值 | 向下为正，起飞后为负 |
| `alt()` | 接口计算的高度 | **向上为正**，起飞后为正数 |

所以判断高度用 `alt()`，不要用 `pos.z` 直接比大小。

<!-- 官方图：PX4-Autopilot 文档 docs/assets/advanced/ENU-NED.png，CC BY 4.0 -->
![图 2.3 NED 与 ENU 坐标对比](images/external/px4-enu-ned.png)

**图 2.3　官方画的 NED 与 ENU：同一个"上"，一个写成负数，一个写成正数。**

*图源：PX4-Autopilot 文档 `docs/assets/advanced/ENU-NED.png`，CC BY 4.0。记住结论：PX4 里飞 5m 高写 z=-5，Gazebo 里写 z=+5。*

### 2.3 为什么必须用 PX4Interface

课程早期 8 个节点各写一份 PX4 交互代码，造成过这些**真实故障**：

| 故障 | 根因 | 接口如何避免 |
|---|---|---|
| 多机 2 号机无法解锁 | `target_system` 默认值不一致 | 构造时传入，命令自动带正确编号 |
| 与 Point-LIO 连不上 / SHM 死锁 | QoS RELIABLE ↔ BEST_EFFORT 不匹配 | 发布端固定 BEST_EFFORT+VOLATILE |
| 遇 GPS 漂移"贴地爬行" | 只有部分节点做 EKF 原点门控 | 统一 `ekf_stable` 门控（计数器稳定 5s） |
| 飞反、打转 | 坐标系各写各的 | 提供 `enu_to_ned()` / `body_to_world_vel()` |

所以平台契约规定：**所有 PX4 交互走 `drone_sim/px4_interface.py`**（297 行，坑已固化）。

### 2.4 EKF 原点重置与门控

PX4 的 EKF 会融合传感器估计位置。当 GPS 漂移等原因触发时，它会**重置局部原点**（`xy_reset_counter` / `z_reset_counter` 跳变）。后果：

- 局部坐标发生"瞬移"，同一物理位置对应的 x/y/z 变了；
- 若任务用局部 z 判断"已起飞"，会误判，导致**贴地爬行**（E5 实验中实测）。

接口的对策是 `ekf_stable`：重置计数器 5 秒（10Hz × 50 拍）未变才返回真，且 `request_offboard_and_arm()` 默认只在它为真的情况下解锁。

### 2.5 QoS：发布与订阅两端要匹配

ROS2 的 QoS（服务质量策略）决定消息如何传递。课程里两套预设（见 `px4_interface.py` 第 66–80 行）：

| 用途 | 预设 | 设置 |
|---|---|---|
| 发布端（发给 PX4） | `QOS_PUB` | BEST_EFFORT + VOLATILE |
| 订阅端（收 PX4 状态） | `QOS_SUB` | BEST_EFFORT + TRANSIENT_LOCAL |

**不匹配会怎样**：发布端若用默认 RELIABLE，在 FastDDS 共享内存流控上可能**无限阻塞**——表现是节点 CPU 冻结、10Hz 心跳停发、飞机失去控制（这是本课程真实的死锁事故，已固化进接口）。

订阅端若用默认 RELIABLE 而 PX4 侧是 BEST_EFFORT，会**静默收不到数据**（不报错），比报错更难查。

### 2.6 心跳：为什么必须"持续"发

PX4 判断"外部控制器还活着"的方式很朴素：**看设定点话题有没有持续更新**。

| 参数 | 典型值 | 含义 |
|---|---|---|
| `COM_OF_LOSS_T` | 1.0 s | 多久没收到设定点就判定丢失 |
| 判定结果 | failsafe | 按 `COM_OBL_RC_ACT` 执行（悬停/降落/返航） |

**所以"发一次设定点"不够**——必须在超时内持续重发。课程节点用 10Hz 定时器，远超 1s 要求。

**为什么容易踩**：如果在循环里做了耗时操作（比如等图像处理结果），定时器被阻塞，心跳就断了。表现为"飞着飞着突然自己降落"。

**诊断**：

```bash
ros2 topic hz <ns>/fmu/in/trajectory_setpoint   # 应稳定 ≥ 2Hz
```

**与第 5 章直接相关**：YOLO 推理一帧几十毫秒甚至几秒，如果塞进同一个定时器，心跳必然断。**正确做法是分节点/分线程。**

### 2.7 模式编号：`param2=6.0` 不是 14

PX4 的 Offboard 在遥测里显示 `nav_state=14`，但**切模式的命令不是写 14**：

```python
self.send_command(VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
#                                     ↑    ↑
#                              主模式=1  子模式=6
```

| 量 | 值 | 含义 |
|---|---|---|
| `param1` | 1.0 | 主模式 = Custom（自定义） |
| `param2` | 6.0 | Custom 组内的编号 = Offboard |
| `nav_state` | 14 | 遥测里回报的**最终**状态编号 |

**这是两个不同的编号体系**：一个是"命令的入参"，一个是"状态的回报值"。混用会切不过去，而且报错不明显。

**切 Offboard 的两个前提**：

1. 已收到过设定点流（**预流**）；
2. 当前状态允许切换（地面上未解锁时不容许）。

这就是代码里"先发 20 拍再切模式"的原因：

```python
if self.tick > 20 and self.tick - self.last_cmd_tick >= 10:
    ...  # 先预流 20 拍（2 秒）
```

### 2.8 从 NED 到 ENU：一次完整的换算

`PX4Interface` 封装了转换，但你得知道它做了什么，否则调试时看不懂数字。

| 量 | ENU（gz） | NED（PX4） |
|---|---|---|
| 第一分量 | 东（x） | 北（x） |
| 第二分量 | 北（y） | 东（y） |
| 第三分量 | 上（z） | 下（z） |

所以：

```text
ned_x = enu_y
ned_y = enu_x
ned_z = -enu_z
```

**注意这是"交换"而不是"旋转"**——第 15 章那个飞出去 58 米的事故就是把它当旋转处理了。

**速度**要分两层：机体系速度 `(前, 右, 下)` 需按航向旋转才能变成世界系（`body_to_world_vel`）。

**自检方法**：让飞机朝北飞 1 米，看 NED 的 `x` 是否增加 1。如果 `y` 增加，说明你写成了旋转。

### 2.9 三种设定点：位置、速度、保持

`PX4Interface` 提供三个"心跳"方法，对应三种控制方式：

| 方法 | 发什么 | `OffboardControlMode` 字段 | 适合 |
|---|---|---|---|
| `heartbeat_position(x, y, z, yaw)` | 目标**位置** | `position = True` | 航点、定点悬停 |
| `heartbeat_velocity(vx, vy, vz, yaw)` | 目标**速度** | `velocity = True` | 视觉伺服、跟随、避障 |
| `heartbeat_hold()` | 保持当前位置 | 位置（当前值） | 暂停、等待 |

**关键点：发什么类型的设定点，就必须把 `OffboardControlMode` 对应字段置 True**。

```python
def heartbeat_velocity(self, vx, vy, vz_down, yaw=0.0):
    mode.velocity = True          # ← 必须与 TrajectorySetpoint 内容一致
```

**如果模式字段和设定点内容不一致会怎样**：飞控按"位置"解读你发的"速度"，结果飞机往错误的方向飞。**而且不报错**——这是又一处静默失败。

**怎么选**：

| 场景 | 选哪个 | 为什么 |
|---|---|---|
| 飞航点 | 位置 | 到点判定简单 |
| 精准降落伺服 | **速度** | 需要连续微调，位置设定点会跳变 |
| 编队跟随 | 位置（推进设定点） | 避免速度饱和 |
| 避障绕行 | **速度** | 需要快速改变方向 |

**对照课程各章**：第 4 章用位置（航点）、第 6 章用位置（分级下降）、第 9 章用**速度**（避障绕行）、第 12 章用位置（推进式跟随）。

**这一条比记 API 更重要**：选错类型，代码能跑、日志正常、飞机就是不对。

### 2.10 为什么"位置设定点"要"推进"而不是"跳变"

第 12 章的跟随者用了这段：

```python
step = min(speed * 0.1, dist_sp)
self.target = [self.target[i] + d[i] / dist_sp * step for i in range(3)]
```

**为什么不直接 `self.target = goal`**：位置设定点瞬间跳变几米，飞控会以最大加速度去追，表现为猛冲、过冲、甚至触发安全层。

**推进的本质**是给飞控一个"可跟随"的目标——每秒最多移动 `speed` 米。

**这等价于在位置环外面又套了一个"速度限制"**。你也可以用速度设定点直接表达，但推进式的好处是：**目标点本身是连续的，天然避免了位置跳变**。

**代价**：滞后。目标点永远比真实目标慢一点——这就是第 12 章"跟踪滞后"误差的来源。

---

## 3. 扩展：案例与排障

### 案例 1：状态机如何保证失败安全

`offboard_demo` 的状态转移（读 `offboard_demo.py`）：

```text
INIT ──(预流 2s + ekf_stable)──> 请求 Offboard + 解锁
  │ 已解锁且 Offboard
  ▼
TAKEOFF ──(|alt - 目标| < 0.3)──> CIRCLE ──(计时到)──> LAND ──(上锁)──> DONE
  └────── 丢失 Offboard/解锁 → 回 INIT 重来
```

<!-- IMAGE_PLACEHOLDER
id: fig-ch02-state-machine
caption: 图 2.4 offboard_demo 状态机：五个状态与转移条件
alt: 状态机图，INIT 经预流与 EKF 稳定后请求解锁进入 TAKEOFF，到目标高度进 CIRCLE，计时结束进 LAND，上锁后 DONE；任一状态丢失 Offboard 或解锁则回 INIT
prompt: |
  技术教材插图，白底，扁平化风格，中文标注，无真人无卡通无水印。
  横向五个圆角矩形状态框，依次："INIT"、"TAKEOFF"、"CIRCLE"、"LAND"、"DONE"，之间用箭头连接。
  箭头上标注转移条件：
  INIT→TAKEOFF 标 "预流 2s + ekf_stable，已解锁且 Offboard"；
  TAKEOFF→CIRCLE 标 "|alt - 目标| < 0.3"；
  CIRCLE→LAND 标 "画圆计时结束"；
  LAND→DONE 标 "已上锁（arming_state=1）"。
  从 TAKEOFF、CIRCLE 向下画虚线回到 INIT，标 "丢失 Offboard / 解锁 → 回 INIT"。
  配色蓝灰，转移条件文字清晰。
-->
![图 2.4 状态机](images/PLACEHOLDER-fig-ch02-state-machine.png)

**图 2.4　最小节点内部的状态机：预流 → 切模式 → 解锁 → 飞 → 降落。**

要点：

- 每个状态开头都检查 `armed and offboard`，掉了就回 INIT——**失败安全**，而不是继续盲飞；
- 降落判定用**上锁状态**而非 z 阈值（x500 出生点偏移，地面 z≈-0.85 而非 0，用 z 会误判）；
- 画圆用 `x=r·cos(θ), y=r·sin(θ), yaw=θ+π/2`（机头沿切线）。

### 案例 2：QoS 死锁复现（本章的"刺"）

**背景**：想验证"发布端 RELIABLE 会死锁"这一说法，亲手复现。

**做法**：临时把 `px4_interface.py` 里 `QOS_PUB` 的 `reliability` 改成 `RELIABLE`，重编后运行节点。

**现象**：

- 节点进程仍在，但 CPU 时间不再增长（`ps -o time -p <pid>` 观察）；
- 10Hz 心跳停发，飞机失去控制或触发保护；
- 不报明显异常，容易误判为"仿真卡了"。

<!-- SCREENSHOT_PLACEHOLDER
id: shot-ch02-qos-deadlock
caption: 图 2.5 QoS 死锁现象：节点 CPU 时间冻结、心跳停止
when: 把 QOS_PUB 改为 RELIABLE 重编后运行节点，观察 ps -o time 不再增长时
what: 截取终端，需能看到 ps -o time 两次查看值相同（CPU 冻结），或节点日志中心跳停止
how: |
  ps -o time -p <节点pid>; sleep 5; ps -o time -p <节点pid>   # 两次值相同即冻结
  截取终端输出
-->
![图 2.5 QoS 死锁现象](images/PLACEHOLDER-shot-ch02-qos-deadlock.png)

**修复**：改回 `BEST_EFFORT`。接口的 `QOS_PUB` 已固化正确配置。

### 案例 3：EKF 原点重置导致"贴地爬行"

**背景**：E5 实验中，飞机不爬升而是贴着地面移动。

**现象与原因**：

- EKF 重置后局部 z 原点跳变，任务里"z < -1 表示已起飞"的判断失效；
- 飞机以为自己已在空中，实际还在地面，于是沿地面"爬行"。

**对策**：使用 `ekf_stable` 门控（已固化），并且**位置/精度判定优先用 Gazebo 世界系真值**（第 16 章铁律三）。

### 案例 4：多机时 target_system 必须匹配

**背景**：第 12 章双机编队时，2 号机不解锁。

**原因**：MAVLink 命令按 `MAV_SYS_ID` 路由。第 N 架飞机的 SYS_ID 是 N+1，命令里的 `target_system` 写错（例如都写 1），命令会被 1 号机吃掉，2 号机收不到。

**对策**：`PX4Interface` 构造时传入正确的 `target_system`；第 12 章详述。

---

## 你可能卡在这里

| 现象 | 最可能原因 | 先跑这条命令 |
|---|---|---|
| 日志一直"等待 PX4 状态" | Agent 没起 / 命名空间错了 | `ros2 topic list \| grep fmu` |
| 解锁被拒 | EKF 未稳定（预检不过） | 等 10–20s，或查 `ekf_stable` |
| 切 Offboard 失败 | 没先预流设定点 | 确认 `tick > 20` 那段在跑 |
| 飞机贴地爬行 | EKF 原点重置 | `ros2 topic echo {ns}/fmu/out/vehicle_local_position --once` |
| 高度判断总反 | 用了 `pos.z` 而不是 `alt()` | 打印两者对比 |
| 节点 CPU 冻结、心跳停 | QoS RELIABLE 死锁 | 查 `QOS_PUB` 是否为 BEST_EFFORT |
| 自己写的节点收不到状态 | 订阅端 QoS 不匹配 | `ros2 topic info -v` 看两端 |

---

## 任务卡

### 任务卡 1：用最小范式写 hover_node

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 起飞到 3m，悬停后自动降落，日志出现降落/上锁 |
| 2 通信骨架/前置 | 栈已启动；用 `PX4Interface(ns='/x500_0', target_system=1)`；10Hz 定时器 |
| 3 AI 提示 | **先自己写**（本章 2.x 已给最小范式，照着改成 3m 悬停），写完再问：<br>"这是我自己写的 hover_node（贴代码）。请只指出三件事：有没有 10Hz 定时器、解锁前有没有检查 ekf_stable、心跳有没有每拍都发。**只列问题，不要给完整代码。**" |
| 4 执行步骤 | ① 新建 `hover_node.py` ② 在 `setup.py` 注册 ③ `colcon build --packages-select drone_sim` ④ `ros2 run drone_sim hover_node` ⑤ 另开终端 echo `vehicle_status` 观察 |
| 5 验收标准 | □ 飞机爬升到约 3m（终端 B 中 `arming_state` 1→2）□ 自动降落并上锁 □ 未出现"等待 PX4 状态" |
| 6 原理 + 扩学 | 教**Offboard 三要素与统一接口**。扩学：把悬停改成"到 3m 后前进 5m 再降落" |

### 任务卡 2：坐标符号验证

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 证明 `pos.z` 与 `alt()` 符号相反，并解释 |
| 2 通信骨架/前置 | 订阅 `/x500_0/fmu/out/vehicle_local_position`；接口 `alt()` |
| 3 AI 提示 | 先自己加这行打印、跑出真实数值，再让 AI 判对错：<br>"PX4 用 NED（z 向下为正），接口 alt() 向上为正。我起飞 5m 时实测 pos.z 与 alt() 分别是这两个数（贴数值）。请判断我的理解对不对、为什么符号相反。**不要写代码，只做判断。**" |
| 4 执行步骤 | ① 在 `hover_node` 的 loop 里加一行日志 ② 重编运行 ③ 记录起飞后的两个值 |
| 5 验收标准 | □ 能说出起飞 5m 时 `pos.z ≈ -5`、`alt() ≈ 5` □ 能解释 NED 与 ENU 的 z 方向差异 |
| 6 原理 + 扩学 | 教**坐标系符号**。扩学：用 `enu_to_ned()` 把 Gazebo 世界系目标转成 PX4 设定点 |

### 任务卡 3：故障注入——QoS 死锁

| 段 | 内容 |
|---|---|
| 1 要实现的功能 | 复现 RELIABLE 发布端死锁并修复 |
| 2 通信骨架/前置 | **无 AI 区：本卡禁止使用 AI**。备份 `px4_interface.py`，改 `QOS_PUB.reliability = RELIABLE` |
| 3 AI 提示 | 本卡不使用 AI。卡住可查 [故障实验室](faq.md)、读 `px4_interface.py` 开头的 QoS 注释、用 `ps -o time` 观察 CPU 时间。 |
| 4 执行步骤 | ① 改 QoS 重编 ② 运行节点 ③ `ps -o time -p <pid>` 两次对比 ④ 改回 BEST_EFFORT 重编验证恢复<br>**验证 AI 的说法**：用 `grep -n "QOS_PUB" ros2_ws/src/drone_sim/drone_sim/px4_interface.py` 核对课程实际配置，不要只按 AI 给的默认 RELIABLE 理解 |
| 5 验收标准 | □ 能复现 CPU 冻结 □ 能说明发布端与订阅端 QoS 应分别是什么 □ 改回后节点正常飞行 |
| 6 原理 + 扩学 | 教**QoS 匹配**。扩学：检查一个订阅端若用默认 RELIABLE 会怎样（静默收不到） |

## ✅ 检查清单

- [ ] 我能说出 Offboard 的三个动作及正确顺序
- [ ] 我能解释为什么切 Offboard 前必须先预流设定点
- [ ] 我能说清 NED 的 z 符号，以及 `pos.z` 与 `alt()` 的区别
- [ ] 我能用 `PX4Interface` 写出起飞—悬停—降落节点并跑通
- [ ] 我能解释 `ekf_stable` 门控在防"贴地爬行"中的作用
- [ ] 我能复现并修复 QoS RELIABLE 导致的死锁
- [ ] 我能关掉 AI，独立写出 hover_node 并跑通

> AI 可以解释 `actuator` 这类字段差异、指出你代码的问题，但**不替你写实现**；任务卡 3 起为无 AI 区。
> 规则见 [AI 使用指南](AI_GUIDE.md)。

- [ ] 我能说出 Offboard 的三个动作及正确顺序
- [ ] 我能解释为什么切 Offboard 前必须先预流设定点
- [ ] 我能说清 NED 的 z 符号，以及 `pos.z` 与 `alt()` 的区别
- [ ] 我能用 `PX4Interface` 写出起飞—悬停—降落节点并跑通
- [ ] 我能解释 `ekf_stable` 门控在防"贴地爬行"中的作用
- [ ] 我能复现并修复 QoS RELIABLE 导致的死锁

## 分层练习

**基础**

1. 目标高度 5m，NED 里 `z` 写多少？
   > 要点：`-5.0`。

2. `VEHICLE_CMD_DO_SET_MODE` 的 `param2=6.0` 表示什么？
   > 要点：Custom 组内的 Offboard 编号（不是 `nav_state=14`）。

3. 为什么切 Offboard 前要先发设定点？
   > 要点：PX4 要求"先有流再切模式"，否则拒绝切换。

**进阶**

4. 把定时器从 10Hz 改成 1Hz，观察心跳是否触发 failsafe。

5. 在 `loop()` 里加 `time.sleep(0.5)`，预测并验证现象（心跳断 → 自动降落）。

6. 用 `ros2 topic hz` 分别测发布与订阅两侧的频率，解释差异。

**挑战**

7. 写一个节点，同时支持"位置设定点"与"速度设定点"两种模式，用参数切换。
   > 要点：`OffboardControlMode` 里对应的布尔字段要同步切换。

8. 设计一个"心跳看门狗"：发现设定点发布频率掉到阈值以下就主动告警。

---

## 自查 12 问

1. Offboard 的字面意思与本质是什么？
2. Offboard 三要素是哪三个？
3. 为什么必须先预流设定点？
4. 解锁与切模式的命令分别是什么？
5. `nav_state=14` 与 `param2=6.0` 是什么关系？
6. NED 的 z 为什么向下？
7. `alt()` 与 `pos.z` 的关系是什么？
8. `ekf_stable` 门控防的是什么？
9. `QOS_PUB` 与 `QOS_SUB` 分别是什么组合？
10. QoS 不匹配的两种表现（阻塞 vs 静默）？
11. 心跳超时 `COM_OF_LOSS_T` 触发后会发生什么？
12. EKF 原点重置为什么会导致"贴地爬行"？

---

## 小结与下一步

本章把第 1 章"会用"推进到"会写"：Offboard 三要素、NED 坐标、统一接口与两类典型故障（QoS 死锁、EKF 原点重置）都已在代码层面固化。

现在你能控制单机按自己的逻辑飞。但真正的任务往往需要**多个航点**和轨迹可视化，也需要把状态机变成可配置的描述，而不是硬编码——这是第 3、4 章的内容（传感器与航点任务）。

[下一章：传感器话题桥接 →](course/03-sensors.md)

---

## 附录：本章命令与参数速查

### 观察状态

```bash
ros2 topic echo /x500_0/fmu/out/vehicle_status --once
ros2 topic echo /x500_0/fmu/out/vehicle_local_position --once
ros2 topic hz   /x500_0/fmu/out/vehicle_status
```

### 关键命令字

| 命令 | `param1` | `param2` | 含义 |
|---|---|---|---|
| `VEHICLE_CMD_DO_SET_MODE` | 1.0 | 6.0 | 切 Offboard |
| `VEHICLE_CMD_COMPONENT_ARM_DISARM` | 1.0 | 0.0 | 解锁 |
| `VEHICLE_CMD_COMPONENT_ARM_DISARM` | 0.0 | 0.0 | 上锁 |
| `VEHICLE_CMD_NAV_LAND` | — | — | 降落 |
| `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` | — | — | 返航 |

### 状态值对照

| 字段 | 值 | 含义 |
|---|---|---|
| `arming_state` | 1 | 已上锁（disarmed） |
| `arming_state` | 2 | 已解锁（armed） |
| `nav_state` | 14 | Offboard |
| `nav_state` | 22 | Position |
| `nav_state` | 18 | Land |

### QoS 预设

| 预设 | Reliability | Durability | 用于 |
|---|---|---|---|
| `QOS_PUB` | BEST_EFFORT | VOLATILE | 发布设定点/命令 |
| `QOS_SUB` | BEST_EFFORT | TRANSIENT_LOCAL | 订阅状态 |

### 模式切换的两个前提

1. 已预流设定点（≥ 20 拍 / 2 秒）；
2. 当前状态允许切换（地面上未解锁时不允许）。
