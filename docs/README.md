# Drone ROS2 开源课程

> **从零到集群的仿真无人机之路** —— 基于 PX4 v1.15 + ROS2 Humble + Gazebo Harmonic + XTDrone2

## 这是一门什么课

一门**全部实验实测验证**的仿真无人机课程。不是"理论上可以跑"，而是每一章都对应仓库里
真实飞过的场景：起飞画圆、航点任务、YOLO 检测、ArUco 精准降落（落点偏差 5cm）、
2D/3D 激光 SLAM、反应式避障、行为树任务、双机编队、三机集群交叉、VTOL 复合翼过渡……
全部有日志、轨迹、截图与**可执行的判分标准**。

我们甚至把**失败的实验**（VIO 视觉惯性里程计，6 轮尝试未收敛）完整写进第 8 章——
因为真实的工程研究里，失败记录比成功演示更有教学价值。

## 课程特色

| 特色 | 说明 |
|---|---|
| 🪜 阶梯式 | L0 起飞悬停 → L6 集群/VTOL，13 个场景按难度排课 |
| ✅ 可判分 | 每个实验有指标阈值，`tools/grade.py` 自动跑 N 次取通过率（≥2/3 才算 PASS） |
| 🔬 外部真值 | 精度判定一律用 Gazebo 世界系位姿，不被 PX4 局部坐标"欺骗" |
| 🧯 故障实验室 | 20+ 个真实踩坑（QoS 死锁、EKF 原点重置、坐标系失配…）做成教学素材 |
| 🔁 一键复现 | 新机器三条命令还原全部环境（锁版本 + 18 个补丁固化踩坑） |
| 📖 诚实清单 | 每个场景标注 verified / unverified / broken，不拿跑不通的东西当作业 |

## 学习路线

```
L0 首飞 ── L1 航点 ──┬── L2 YOLO ── L3 精准降落
                     ├── L4 激光SLAM ──┬── L5 避障规划 ── L6 集群
                     │   (VIO挑战)     ├── L5 行为树
                     └── 传感器桥接 ───┴── L5 安全围栏 ── L6 编队/VTOL
                                                          └── Sim2Real
```

## 快速开始（三条命令）

```bash
git clone https://github.com/howe12/Drone_ros2.git && cd Drone_ros2
bash scripts/setup_workspace.sh        # 克隆依赖+补丁+全量编译（需先装 ROS2 Humble）
bash scripts/run_simulation.sh xtd2    # 启动 Gazebo Harmonic + PX4 SITL
```

然后新开终端跑第一章的起飞实验。详见 [01 环境搭建与首飞](course/01-setup.md)。

## 章节总览

| 部分 | 章节 | 对应实验 | 难度 |
|---|---|---|---|
| 背景 | [00 认识无人机与仿真](course/00-basics.md)：机型分类、飞行原理、飞控职责、名字由来 | 无实验，阅读章 | 入门 |
| 基础 | 01 环境搭建与首飞 / 02 Offboard 与 PX4Interface / 03 传感器桥接 / 04 航点与场景 | E1 E2 E13 | L0–L1 |
| 视觉 | 05 YOLO 检测 / 06 ArUco 精准降落 | E8 | L2–L3 |
| 估计与建图 | 07 激光 SLAM / 08 VIO 失败案例 | E4 E5 E6 | L4 |
| 规划与任务 | 09 反应式避障 / 10 行为树 / 11 安全围栏 | E7 E10 E12 | L5 |
| 多机与机型 | 12 编队 / 13 集群交叉 / 14 VTOL | E3 E9 E11 | L6 |
| 工程化 | 15 Sim2Real / 16 自动判分 / 故障实验室 | E12 + 平台层 | — |

## 环境资产指南

要自己找场景、或者给某个实验配一个合适的世界，请先读 [虚拟环境资产指南](ENVIRONMENTS.md)：

- 四个可靠来源（Fuel / PX4 自带 / XTDrone2 / 本项目自建）以及各自的坑
- 自己造世界的 8 步 SOP 检查表（每步都来自实际踩坑）
- 三档策略：直接用 → 组合（推荐）→ 导入
- 按实验类型查表选环境 + 环境复杂度阶梯
- 把"造环境"布置成学生作业的 6 条验收标准
- 踩坑速查表（黑色贴图 / 空白相机 / 不能解锁 / 家具倒地 / 撞墙 …）

## 关于 AI 的使用

AI 在本课程里是**陪练，不是替写**：可以解释报错、解释概念、指出你代码哪里错、出题考你；
**不可以**让它写你的任务代码或直接给答案。

三条硬规矩：

1. **先写后问**——先有自己的代码/命令/判断，再让 AI 针对它挑错。
2. **无 AI 区**——每章的故障注入卡和最后一张卡，禁止使用 AI，必须独立完成。
3. **脱稿验收**——关掉 AI 能独立复现、能回答追问，才算这一章过了。

判分用 `grade.py`，AI 伪造不了结果；真正要守住的是你自己的能力。
完整规则（红绿灯、上下文模板、AI 常错的五处）见 **[AI 使用指南](AI_GUIDE.md)**。

## 教学设计

每章按固定**六段式**组织（驱动问题 → 上一章回顾 → 学习目标对比表 → 主体内容 → 六阶段实验 → 新问题），
章尾附**任务卡**（六段固定：功能 / 通信骨架 / 参考 prompt / 执行步骤 / 验收标准 / 原理扩学），
每张任务卡都埋一个**故障**让学生亲手修——踩一次比讲十次管用。
完整规范见[课程编写规范](AUTHORING.md)，可复制的空白模板见[章节模板](course/_template.md)。

[01 环境搭建与首飞](course/01-setup.md) 已按此规范完成改版，可作为样板。

## 适合谁 / 前置要求

- 会基本 Linux 命令行与 Python；了解 ROS2 话题/节点概念更佳（第 2 章会补）
- 一台 Ubuntu 22.04 机器（8 核 / 16G+ 内存 / 支持 OpenGL 的显卡；RTX 2070S 实测流畅）
- 不需要真机——全部实验在仿真中完成；第 15 章给出通向实机的路线

## 致谢

本课程站在开源社区肩上：[PX4](https://github.com/PX4/PX4-Autopilot)、
[XTDrone2](https://github.com/zuiyue52/XTDrone2)（北大团队）、
[px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard)、
[Point-LIO](https://github.com/thamyis100/point_lio_ros2)、slam_toolbox、py_trees 等。
课程形式致敬 [Datawhale hello-agents](https://datawhalechina.github.io/hello-agents/)。

> 仓库：[howe12/Drone_ros2](https://github.com/howe12/Drone_ros2) · 协议：BSD-3-Clause（自有代码）
> 问题反馈：[GitHub Issues](https://github.com/howe12/Drone_ros2/issues)
