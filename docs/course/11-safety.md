# 11 安全监督与地理围栏

> 难度 L5 · 对应实验 E12（安全部分）· 场景 `safety.geofence`

## 学习目标

- 独立安全监督层：围栏/速度/倾角/急停 → RTL 或 LAND
- 理解"监督者必须独立于任务"的安全架构

## 11.1 跑起来（围栏触发演示）

```bash
# 栈: gz_x500 + walls
bash scripts/run_simulation.sh xtd2 gz_x500 walls

# 终端1: 安全监督（半径 2.5m 围栏）
ros2 run drone_sim safety_supervisor --ros-args -p px4_ns:=/x500_0 -p max_radius:=2.5

# 终端2: 故意飞超出围栏的正方形任务
ros2 launch drone_sim waypoint_mission.launch.py prefix:=/x500_0 mission:=square
```

实测：飞机飞出 2.5m 半径时正确触发 **"超出水平围栏 3.1m -> RTL"**，无误报。

### 判分指标

| 指标 | 阈值 | 取数 |
|---|---|---|
| 围栏事件 | ≥1 | 日志含"超出水平围栏"等 |
| 误报 | <1 次 | 非围栏类安全事件数 |

## 11.2 监督层设计（`safety_supervisor.py`）

| 检查 | 触发动作 |
|---|---|
| 水平半径 > max_radius | RTL |
| 高度 > max_altitude | LAND |
| 速度超限 | 告警/限制 |
| 倾角超限 | LAND |
| 外部急停话题 | 立即 LAND |

架构要点：监督者**只订阅状态、只发安全命令**，不参与任务逻辑——任务节点崩了它还在。
这是实机部署的标准安全范式（第 15 章会再见到它）。

## 11.3 故障实验室：倾角不能用速度比近似

早期版本用"水平速度/垂直速度"近似倾角——悬停时垂直速度≈0，算出 **88° 假倾角**疯狂误报。
修复：订阅 `vehicle_attitude`，**用四元数算真实倾角**。

> 教训：安全层的每个量都要用权威来源，近似公式在边界工况会背叛你。

## 练习

1. 把 max_radius 调到 5m，验证正方形任务（3m 边）不再触发围栏。
2. 设计一个高度围栏实验：max_altitude:=4，任务起飞 5m，观察触发。
3. 思考：为什么监督层触发 RTL 而不是直接 disarm？

[上一章](course/10-bt.md) · [下一章：多机编队 →](course/12-formation.md)
