# 12 多机编队

> 难度 L6 · 对应实验 E3（Issue #3，已完成）· 场景 `multi.formation`
> 实测：双机同时起飞，队形误差 **0.14–0.29 m**

## 学习目标

- N 机仿真启动与命名空间机制（`PX4_UXRCE_DDS_NS`）
- leader-follower 编队控制
- 多机两大深坑：MAV_SYS_ID 路由与 parameters.bson 冲突

## 12.1 跑起来（双机编队）

```bash
# 一条 launch 拉起双机全套（各自独立 gz 模型 + PX4 实例 + Agent 会话）:
ros2 launch drone_sim multi_vehicle.launch.py num:=2 world:=aruco \
  px4_dir:="$PWD/PX4-Autopilot"    # 仓库根目录下的 PX4（clone 后自带的路径）

# 终端2: 领航机飞正方形
ros2 launch drone_sim waypoint_mission.launch.py prefix:=/x500_0 mission:=square

# 终端3: 跟随机保持编队偏移
ros2 run drone_sim formation_follower --ros-args \
  -p leader_ns:=/x500_0 -p follower_ns:=/x500_1 -p target_system:=2
```

![双机编队](images/13_e3_formation.png)

### 判分指标

| 指标 | 阈值 | 取数 |
|---|---|---|
| 队形误差 | < 0.6 m | 跟随者与(领航者+偏移)的中位距离 |
| 双机同时起飞 | ≥1 | 两机解锁时刻相差 <8s |

## 12.2 多机机制

```
每架飞机 = 独立 PX4 实例(-i N) + 独立命名空间 /x500_N + 独立工作目录
         + MAV_SYS_ID = N+1 + DDS 话题前缀 /x500_N/fmu/...
```

- `num:=3` 即三机；spawn 队形可自定义（第 13 章集群用它摆三角形）
- 跟随机订阅领航机**世界系位姿**（`/x500_0/pose`），加偏移后作为自己的目标

## 12.3 故障实验室（多机两大深坑）

1. **target_system 必须 = 实例号+1**：MAVLink 命令按 MAV_SYS_ID 路由。
   2 号机 SYS_ID=2，命令里写默认 1 → 命令被 1 号机吃掉，2 号机永远不解锁。
2. **parameters.bson 冲突**（补丁 0010）：多个 PX4 实例共享工作目录会互写参数文件，
   参数互相覆盖、行为错乱。修复：每实例独立工作目录。

## 练习

1. 三机编队（num:=3），两个跟随机不同偏移，观察队形。
2. 把 follower 的 target_system 故意写错成 1，复现"2 号机不解锁"，再看日志定位。
3. 思考：为什么编队用世界系位姿而不是 PX4 局部位置？（提示：各机 EKF 原点独立重置）

[上一章](course/11-safety.md) · [下一章：三机集群交叉 →](course/13-swarm.md)
