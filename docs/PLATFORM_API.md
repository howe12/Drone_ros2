# 平台契约 (Platform API) v1

> 面向对象：**编写课程内容的 agent / 编写节点的学生**。
> 目标：不需要读 14 个节点的源码，也能正确地写实验、判分、加题目。

---

## 0. 平台分层（为什么这样组织）

```
L1 能跑起来    run_simulation.sh + setup_workspace.sh + patches/     (环境可复现)
L2 能自测      catalog/scenarios.yaml + tools/grade.py (待实现)      (判定可执行)
L3 能学        catalog/params.yaml + docs/ + 课程内容(由课程 agent 写) (可调可解释)
```

课程内容不属于本仓库职责；本仓库保证 **L1/L2 的契约稳定**。

---

## 1. 唯一控制入口：`PX4Interface`

**所有与 PX4 的交互必须走 `drone_sim/px4_interface.py`，不要自己建 publisher。**

原因：此前 8 个节点各写一份，导致过这些真实故障——
- `target_system` 默认值不一致 → 多机时 2 号机无法解锁
- QoS RELIABLE ↔ BEST_EFFORT 不匹配 → 与 Point-LIO 连不上、甚至 SHM 死锁
- 只有部分节点做 EKF 原点门控 → 遇 GPS 漂移"贴地爬行"
- 坐标系各写各的 → 飞反、打转

### 最小示例（推荐范式，约 30 行）

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
            ns='/x500_0',                                  # 单机可用 '' + launch remap
            target_system=self.get_parameter('target_system').value,
        )
        self.create_timer(0.1, self.loop)                  # 10Hz, 必须 > 2Hz

    def loop(self):
        self.px4.heartbeat_position(0.0, 0.0, -3.0, yaw=0.0)   # 心跳+设定点
        if not self.px4.ready:
            return
        if not (self.px4.armed and self.px4.offboard):
            if self.px4.ekf_stable:                        # EKF 稳定才解锁
                self.px4.request_offboard_and_arm()
            return
        if abs(self.px4.alt() - 3.0) < 0.3:
            self.px4.land()
```

### API 速查

| 成员 | 说明 |
|---|---|
| `ready` | 是否已收到 status + position |
| `armed` / `offboard` / `nav_state` | 飞行状态 |
| `ekf_stable` | EKF 重置计数器 5s 未变（**解锁前必须为真**）|
| `alt()` | 相对起飞点高度，向上为正 |
| `heading()` / `tilt_rad()` | 航向 / 倾角（倾角需 `subscribe_attitude=True`）|
| `distance_to(x, y, z_up)` | 到 NED 目标的三维距离 |
| `heartbeat_position(x, y, z_down, yaw)` | 位置模式心跳（10Hz 调用）|
| `heartbeat_velocity(vx, vy, vz_down, yaw)` | 速度模式心跳（自标定型控制用）|
| `heartbeat_hold()` | 原地悬停 |
| `request_offboard_and_arm(require_ekf_stable=True)` | 切 Offboard + 解锁 |
| `set_offboard_mode()` / `set_posctl_mode()` | 单独切模式 |
| `arm()` / `disarm()` / `takeoff(m)` / `land()` / `rtl()` | 常用动作 |
| `vtol_transition_to_fw()` | VTOL→固定翼（**内部先切 POSCTL**，踩坑固化）|
| `send_command(cmd, p1, p2, p3, p7)` | 底层 MAVLink 命令 |
| `enu_to_ned(x, y, z_up)` | ENU→NED 轴交换 |
| `body_to_world_vel(vx, vy, heading)` | 机体系速度→世界系速度 |
| `on_status` / `on_position` | 构造时传入回调，用于可视化/记录 |

---

## 2. 场景目录 `catalog/scenarios.yaml`

课程 agent 生成实验/作业时读这里，**不要自己编命令**。

关键字段：

| 字段 | 用法 |
|---|---|
| `id` | 课程引用键，如 `plan.obstacle_avoidance` |
| `level` | L0–L6 阶梯，用于排课顺序 |
| `model` / `world` | 直接传给 `run_simulation.sh` |
| `command` | 可直接复制的完整启动命令 |
| `metrics` | **判定标准**：`name` / `threshold` / `unit` / `how`（怎么取数）|
| `pitfalls` | 已知坑 → 课程里可作为"故障实验室"素材 |
| `status` | `verified` / `partial` / `unverified` / `broken` |

**重要约定**：`status: unverified` 的场景（当前仅 `vio.vins_mono`）**不得作为作业**，
只能作为"研究型挑战"或阅读材料——因为它当前跑不通。

`worlds` / `models` 段给出每个环境/机型的状态，`broken` 的（`indoor1`）不要用。

---

## 3. 参数注册表 `catalog/params.yaml`

**自动生成，勿手改**：

```bash
python3 tools/gen_params.py          # 重新生成
python3 tools/gen_params.py --check  # CI 校验是否与代码一致
```

内容：14 个节点 / 84 个参数，含 `type` 与 `default`。
课程 agent 用它生成"调参练习"（例如 `planner_reactive.safe_dist` 1.2→3.0 会怎样）。

> 待补充（v2）：每个参数加 `effect`（调大/调小的定性影响）与 `range`。
> 这需要人工判断，不能从代码自动生成——`docs/TUNING.md` 是它的自然归宿。

---

## 4. 判定标准的三条铁律

1. **指标化**：不接受"看起来正常"。每个场景必须有 `metrics`，且 `how` 写清取数方式。
2. **N 次通过率**：单次成功不算 PASS。默认 `N=3`，要求 **≥2/3** 通过。
   - 本仓库历史教训：E7 第 7 次才成功、E10 故障注入从未成功，若按"最好一次"判定就是自欺。
3. **外部真值优先**：位置/精度类指标一律用 **gz 世界系位姿**（`/<ns>/pose`）作为真值，
   不要用 PX4 局部位置——EKF 会重置原点（实测 `xy_reset_counter=3`），局部坐标会骗人。

---

## 5. 新增一个节点的三步

1. 继承 `Node`，用 `PX4Interface` 接管所有 PX4 交互（见第 1 节示例）
2. 在 `setup.py` 的 `console_scripts` 注册，`declare_parameter` 暴露旋钮
3. 跑 `python3 tools/gen_params.py` 更新注册表；在 `catalog/scenarios.yaml` 加场景（含 metrics）

---

## 6. 当前状态与后续（诚实清单）

### 已完成（本轮）
- ✅ `px4_interface.py`：唯一 PX4 接口
- ✅ 2 个节点完成迁移并**飞行验证**：`offboard_demo`、`waypoint_mission`
  （证据：`logs/smoke_iface.log`、`logs/smoke_wp2.log`）
- ✅ `tools/gen_params.py` + `catalog/params.yaml`（14 节点 / 84 参数）
- ✅ `catalog/scenarios.yaml`（13 场景，含实测指标与坑）

### 待办（按优先级）
| # | 事项 | 说明 |
|---|---|---|
| 1 | 迁移剩余 6 个节点到 `PX4Interface` | `formation_follower` / `aruco_precision_landing` / `bt_mission` / `swarm_agent` / `planner_reactive` / `safety_supervisor` |
| 2 | 实现 `tools/grade.py` + `tests/` | N 次运行、输出 JSON/MD 报告、CI 可跑 |
| 3 | `docs/TUNING.md` | 84 个参数的效果/范围（人工判断） |
| 4 | 处理 Gazebo Classic 死路径 | 包已从机器移除，但脚本/README 仍宣称可用 |
| 5 | `archive/`：ego_planner 集成、vio_evaluator | 非可用功能，但有教学价值 |
| 6 | 故障实验室（6 个真实坑） | 素材已在 `scenarios.yaml` 的 `pitfalls` 里 |
| 7 | 无 GPU 降级路径 | 学生显卡不足时的兜底 |
