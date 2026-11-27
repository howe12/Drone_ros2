# 16 自动判分与作业系统

> 平台 L2 层 · 本课程"可执行判定"的核心 · 面向学生与课程作者

## 学习目标

- 会用 grade.py 对自己的飞行自动判分
- 理解三条判分铁律（为什么"看起来正常"不算数）
- 课程作者：会读场景目录、加新场景

## 16.1 学生用法

```bash
python3 tools/grade.py --list                          # 看所有可判分场景
python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1   # 快速自测
python3 tools/grade.py --scenario ctrl.waypoint_square --runs 3 \
    --json out.json --md out.md                        # 正式判分(3次取通过率)
```

grade.py 会自动：起仿真栈 → 跑场景命令 → 采样 ROS 话题 → 对照阈值 → 出 JSON/MD 报告。
进程用**进程组**管理（`start_new_session=True`），结束按组终止——不用 pkill 模式匹配，
从根本上避免"pkill 匹配到自身"事故。

> 需要 GUI 的场景请在能访问 X 的终端运行（DISPLAY=:0）。

## 16.2 三条铁律（血泪教训的代码化）

1. **指标化**：不接受"看起来正常"。每个场景必须有 metrics（name/threshold/unit/how）
2. **N 次通过率**：单次成功不算 PASS。默认 N=3，要求 **≥2/3**。
   历史教训：E7 第 7 次才成功、E10 故障注入从未成功——按"最好一次"判定就是自欺
3. **外部真值优先**：位置/精度指标一律用 **gz 世界系位姿**（`/<ns>/pose`）。
   PX4 局部位置会重置原点（实测 xy_reset_counter=3），局部坐标会骗人

## 16.3 课程作者：场景目录契约

`catalog/scenarios.yaml` 是唯一事实源（**不要自己编命令**）：

```yaml
- id: ctrl.waypoint_square        # 课程引用键
  level: L1                       # L0–L6 阶梯排课
  model: gz_x500                  # 直接传给 run_simulation.sh
  world: aruco
  command: |                      # 可复制的完整命令
    ros2 launch drone_sim waypoint_mission.launch.py prefix:=/x500_0 mission:=square
  metrics:                        # 判定标准
    - {name: 到点误差, threshold: '< 0.5', unit: m, how: '各航点最小接近距离的最大值'}
  pitfalls: [EKF原点重置导致贴地爬行]   # 故障实验室素材
  status: verified                # verified/partial/unverified/broken
```

**约定**：`status: unverified`（当前仅 vio.vins_mono）**不得作为作业**，只能作研究挑战。

参数调参练习素材：`catalog/params.yaml`（14 节点 / 84 参数，`tools/gen_params.py` 自动生成，勿手改）。

## 16.4 新增一个实验的三步

1. 节点用 `PX4Interface`（第 2 章范式），`declare_parameter` 暴露旋钮
2. `setup.py` 注册 console_scripts → `python3 tools/gen_params.py` 更新注册表
3. `scenarios.yaml` 加场景（含 metrics 与 pitfalls）→ grade.py 跑 3 次验证 → 写章节

## 练习

1. 对第 1 章的首飞跑 `--runs 3` 正式判分，读 MD 报告。
2. 故意把起飞高度参数改成 20m（超时），观察判分如何 FAIL。
3. 课程作者题：为你第 4 章写的三角形任务加一个场景条目并判分。

[上一章](course/15-sim2real.md) · [故障实验室（FAQ）→](faq.md)
