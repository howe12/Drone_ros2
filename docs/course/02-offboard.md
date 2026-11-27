# 02 Offboard 控制与 PX4Interface

> 难度 L0–L1 · 代码导读章 · 后续所有实验的地基

## 学习目标

- 理解 PX4 Offboard 模式的三要素与 NED 坐标系
- 掌握课程唯一控制入口 `PX4Interface`（为什么不允许自己建 publisher）
- 读懂 `offboard_demo.py` 状态机，能改出自己的第一个任务

## 2.1 Offboard 三要素

PX4 允许外部计算机（ROS2 节点）直接控制飞机，但有三条硬规则：

1. **心跳流**：`OffboardControlMode` 必须以 **>2Hz** 持续发布（课程用 10Hz），声明控制量类型（位置/速度/姿态…）
2. **设定点流**：`TrajectorySetpoint` 持续发布目标；**切入 Offboard 前就要预流**，否则 PX4 拒绝
3. **模式与解锁命令**：`VehicleCommand` 发 `DO_SET_MODE(176, param1=1, param2=6)` 切 Offboard，`ARM_DISARM(400, param1=1)` 解锁

数据链路（第 1 章已跑通）：

```
ROS2 节点 --DDS话题--> MicroXRCEAgent(UDP 8888) <--uXRCE-DDS--> PX4 SITL <--> Gazebo
```

### NED 坐标系（新手第一坑）

PX4 本地坐标是 **NED**（北-东-地）：**z 向下为正**。飞 5m 高 = `z = -5.0`。
而 Gazebo 世界系是 ENU（z 向上）。课程接口提供 `enu_to_ned()` 换算；
**精度验证一律用 gz 世界系真值**（第 16 章铁律三），因为 PX4 的 EKF 会重置局部原点。

## 2.2 为什么必须用 PX4Interface

课程早期 8 个节点各写一份 PX4 交互代码，造成过这些**真实故障**：

| 故障 | 根因 |
|---|---|
| 多机时 2 号机无法解锁 | `target_system` 默认值不一致（必须 = MAV_SYS_ID = 实例号+1） |
| 与 Point-LIO 连不上 / SHM 死锁 | QoS RELIABLE ↔ BEST_EFFORT 不匹配 |
| 遇 GPS 漂移"贴地爬行" | 只有部分节点做 EKF 原点重置门控 |
| 飞反、打转 | 坐标系各写各的 |

所以平台契约规定：**所有 PX4 交互走 `drone_sim/px4_interface.py`**（297 行，把坑全部固化）。

### 最小范式（约 30 行写一个任务节点）

```python
import rclpy
from rclpy.node import Node
from drone_sim.px4_interface import PX4Interface

class MyMission(Node):
    def __init__(self):
        super().__init__('my_mission')
        self.declare_parameter('target_system', 1)
        self.px4 = PX4Interface(
            self,
            ns='/x500_0',                # 多机命名空间; 单机可 ''
            target_system=self.get_parameter('target_system').value,
        )
        self.create_timer(0.1, self.loop)   # 10Hz, 必须 > 2Hz

    def loop(self):
        self.px4.heartbeat_position(0.0, 0.0, -3.0, yaw=0.0)  # 心跳+设定点
        if not self.px4.ready:
            return
        if not (self.px4.armed and self.px4.offboard):
            if self.px4.ekf_stable:         # EKF 稳定才解锁（门控固化）
                self.px4.request_offboard_and_arm()
            return
        if abs(self.px4.alt() - 3.0) < 0.3:
            self.px4.land()
```

### API 速查（完整表见[平台契约](../PLATFORM_API.md)）

| 成员 | 说明 |
|---|---|
| `ready` / `armed` / `offboard` | 状态就绪 / 已解锁 / Offboard 中 |
| `ekf_stable` | EKF 重置计数器 5s 未变，**解锁前必须为真** |
| `alt()` / `heading()` / `distance_to(x,y,z_up)` | 高度(向上正) / 航向 / 三维距离 |
| `heartbeat_position(x,y,z_down,yaw)` | 位置模式心跳 |
| `heartbeat_velocity(...)` / `heartbeat_hold()` | 速度模式 / 悬停 |
| `request_offboard_and_arm()` / `land()` / `rtl()` | 常用动作 |
| `vtol_transition_to_fw()` | VTOL 过渡（内部先切 POSCTL，坑已固化） |
| `enu_to_ned()` / `body_to_world_vel()` | 坐标换算 |

## 2.3 代码导读：offboard_demo 状态机

`ros2_ws/src/drone_sim/drone_sim/offboard_demo.py`：

```
INIT ──(预流2s + ekf_stable)──> 请求Offboard+解锁
  │ 已解锁且Offboard
  ▼
TAKEOFF ──(|alt-目标|<0.3)──> CIRCLE ──(计时到)──> LAND ──(上锁)──> DONE
  └────── 丢失Offboard/解锁 → 回 INIT 重来（失败安全）
```

要点：
- 每个状态都先检查 `armed and offboard`，掉了就回 INIT——**失败安全**而不是继续盲飞
- 降落判定用**上锁状态**而非 z 阈值（x500 出生点有偏移，地面 z≈-0.85 而非 0，z 阈值会误判）
- 画圆：`x=r·cos(θ), y=r·sin(θ), yaw=θ+π/2`（机头沿切线）

## 2.4 故障实验室（本章素材）

1. **QoS 死锁**：把发布端改回默认 RELIABLE，节点会在 FastDDS SHM 流控上无限阻塞（CPU 冻结、10Hz 心跳停发）。修复：BEST_EFFORT。
2. **EKF 原点重置**：GPS 漂移时 `xy_reset_counter` 跳变，局部坐标"瞬移"。若任务用局部 z 判断"已起飞"，会误判贴地爬行。修复：`ekf_stable` 门控 + 世界系真值。
3. **未预流就切 Offboard**：PX4 直接拒绝。修复：先流 2s 再发模式命令。

## 练习

1. 用最小范式写一个 `hover_node`：起飞 3m 悬停 10s 后降落（不许抄状态机，体会接口）。
2. 把 `offboard_demo` 的圆改成半径 5m、角速度 0.5，预测耗时并验证。
3. 思考：为什么 `target_system` 在多机时必须等于实例号+1？（提示：MAVLink 路由）

[上一章](course/01-setup.md) · [下一章：传感器话题桥接 →](course/03-sensors.md)
