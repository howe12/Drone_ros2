# 仿真到实机 (Sim2Real) 框架与流程

> E12 交付物。**当前无真机**，本文档给出仿真侧已实现的安全框架、参数整定流程、
> 日志回放对比方法，以及从仿真迁移到实机时需要逐项核对的清单。
> 实机飞行前请由具备资质的人员按本清单执行。

---

## 1. 坐标系与约定（最容易出错的地方）

我们在这个项目里踩过的坑，全部固化如下：

| 项目 | 仿真 (gz) | PX4 | 转换关系 |
|---|---|---|---|
| 世界系 | ENU (x=东, y=北, z=上) | NED (x=北, y=东, z=下) | `px4_x = gz_y`, `px4_y = gz_x`, `px4_z = -gz_z` |
| 机体系 | x=前, y=左, z=上 | x=前, y=右, z=下 | 相机光心系: x=右, y=下, z=前 |
| 高度 | `z` 向上为正 | `z` 向下为正 | 控制时取负 |

**实测证据**：
- E7 中把 ego-planner 的 ENU 位置直接当 NED 用 → 无人机飞到 58m 外；
  正确做法是轴交换（不是航向旋转）
- E8 中发现 EKF 会重置航向（heading_reset_counter）与原点（xy_reset_counter），
  固定映射会失效 → 控制环路必须用**世界系真值闭环**或监听 reset counter 重算

## 2. 安全框架（已在仿真验证）

`ros2_ws/src/drone_sim/drone_sim/safety_supervisor.py`：

| 监控项 | 默认阈值 | 触发动作 |
|---|---|---|
| 水平半径 | 15 m | RTL（返航） |
| 高度上限 | 25 m | LAND（降落） |
| 高度下限 | 0.15 m 且下坠 | LAND |
| 水平速度 | 8 m/s | RTL |
| 姿态倾角 | 45° | LAND |
| 外部急停 | `/safety/estop` (Bool) | LAND |

用法：
```bash
ros2 run drone_sim safety_supervisor --ros-args -p px4_ns:=/x500_0 -p max_radius:=5.0
# 急停
ros2 topic pub --once /safety/estop std_msgs/Bool "{data: true}"
```

**验证方法（仿真）**：把 `max_radius` 设小（如 2.5m），执行 3m 航点任务，
观察日志出现 `[安全事件] 超出水平围栏 ... -> 执行 RTL` 且飞行器返航。

**实机对应**（PX4 侧参数，需在 QGC 中设置）：
```
GF_ACTION       = 2        # 围栏动作: 2=返航 1=降落 3=终止
GF_MAX_HOR_DIST = 100      # 水平围栏 (m)
GF_MAX_VER_DIST = 120      # 垂直围栏 (m)
COM_RC_LOSS_T   = 5        # 遥控失联超时 (s)
NAV_RCL_ACT     = 2        # 失联动作: 2=返航
COM_DISARM_PRFLT= 10       # 地面未解锁自动上锁
CBRK_FLIGHTTERM = 0        # 终止开关可用
```

## 3. PX4 参数整定流程（迁移实机时）

1. **传感器标定**：加速度计（六面）、磁罗盘（远离干扰）、水平校准
2. **EKF2**：`EKF2_GPS_CHECK`、`EKF2_HGT_MODE`（GPS/气压/视觉）、`EKF2_EV_CTRL`（外部视觉）
3. **控制参数**：`MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP/DN`、`MPC_XY_P`、`MPC_ACC_HOR`
4. **地理围栏**：见上表
5. **失效保护**：`COM_RC_LOSS_T`、`NAV_RCL_ACT`、`COM_LOW_BAT_ACT`
6. **在仿真中先复现同一组参数**，用 `scripts/sim2real_check.sh` 导出对比

## 4. 日志回放与实机对比

```bash
# 仿真飞行时录制
ros2 bag record -o bags/sim_run /x500_0/fmu/out/vehicle_local_position \
    /x500_0/fmu/out/vehicle_attitude /x500_0/fmu/out/vehicle_status

# 回放（关闭仿真栈后）
ros2 bag play bags/sim_run --clock

# 轨迹对比：导出两份 CSV 后对比关键指标
python3 scripts/traj_compare.py bags/sim_run.csv bags/real_run.csv
```

对比指标：上升时间、超调、稳态误差、最大速度、姿态振荡幅度。

## 5. Sim2Real 差距清单

| 差距 | 仿真现状 | 实机影响 | 缓解 |
|---|---|---|---|
| 动力学 | gz 理想刚体 | 电机延迟、桨叶效率下降 | 加一阶惯性环节、注入延迟 |
| 噪声 | 高斯小噪声 | IMU/磁罗盘漂移大 | 用实机静止数据标定 `acc_n/gyr_n` |
| 时延 | 无 | 相机/图传 50-150ms | 控制器加延时补偿或降增益 |
| 风 | 无（windy 世界可开） | 阵风扰动 | 用 windy 世界做鲁棒性测试 |
| GPS | 理想 | 多路径、遮挡 | 室内改用视觉/动捕 |

## 6. 迁移检查清单（实机前逐项确认）

- [ ] 坐标系转换已在仿真中验证（ENU↔NED、机体系、相机光心系）
- [ ] 相机内参与 IMU 外参已标定，时间同步误差 < 1ms
- [ ] 安全框架阈值按实机场地重新设定，急停通道可用
- [ ] 地理围栏与失效保护参数已写入飞控并**断开遥控验证**过
- [ ] 电池低电量动作（`COM_LOW_BAT_ACT`）设定并测试
- [ ] 仿真日志与实机日志完成对比，关键指标偏差 < 20%
- [ ] 场地清空、人员撤离、有资质的飞手在场

---

**状态**：仿真侧框架（安全监控 + 参数清单 + 回放方法）已完成；
实机验证需硬件到位后执行，Issue #12 保持 open。
