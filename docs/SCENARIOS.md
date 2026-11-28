# 仿真场景画廊 (E13)

世界选择已参数化: `bash scripts/run_simulation.sh <平台> <机型> <世界>`

## Harmonic / XTDrone2 平台

| 世界 | 用途 | 来源 | 状态 | 启动命令 |
|---|---|---|---|---|
| aruco | ArUco 靶标场地, 精准降落 (E8) | XTDrone2 (贴图由我们替换为 DICT_4X4_50 ID42) | ✅ 实测 | `... xtd2 gz_x500 aruco` |
| walls | 障碍墙场地, 避障/2D 激光 SLAM | XTDrone2 (与 PX4 同名世界内容不同) | ✅ 实测 (激光 30Hz) | `... xtd2 gz_x500_lidar_2d walls` |
| **e7_obstacles** | 小块障碍群, 局部规划避障 (E7) | **自建** (补丁 0016) | ✅ 实测 | `... xtd2 gz_x500_depth e7_obstacles` |
| lawn | 草地户外, 通用飞行 | PX4 上游 (`Tools/simulation/gz/worlds/lawn.sdf` 逐字节相同) | ✅ 实测 | `... xtd2 gz_x500 lawn` |
| baylands | 湾区地形, 视觉/VIO | PX4 上游 (逐字节相同) | ✅ 实测 | `... xtd2 gz_x500 baylands` |
| windy | 风扰测试 | PX4 上游 (逐字节相同) | 可用 | `... xtd2 gz_x500 windy` |
| default | 极简默认 (空场) | XTDrone2 | ✅ 实测 | `... xtd2 gz_x500 default` |
| rover | 无人车场景 | XTDrone2 | 可用 (配 gz_r1_rover) | `... xtd2 gz_r1_rover rover` |
| indoor1 | 室内 cafe 场景 | XTDrone2 (引用 Gazebo **Classic** 旧资产) | ⚠️ 不可用 (gz abort) | 见下 |

### indoor1 已知问题
引用旧 Gazebo Fuel 模型 (cafe/cafe_table/cinder_block_wide/coke_can/grey_tote)。
模型已从 osrf/gazebo_models 稀疏克隆补齐 (third_party/gazebo_models, 复制进
xtd2_gz_sim/models 并重编), 但 Harmonic 加载时 gz sim 内部 abort (旧模型 SDF/网格
兼容性问题)。**待修**: 逐个替换问题模型或用 Harmonic 原生室内资产重建。
室内 SLAM 可先用 walls + 自建简单室内世界替代。

## 模型来源

XTDrone2 的模型目录 (`third_party/XTDrone2/xtd2_gz_sim/models/`) 共 34 个，来源分三类：

| 类别 | 模型 | 来源 |
|---|---|---|
| PX4 上游原样 | `x500`（与 PX4 逐字节相同）、`x500_base`、`x500_depth`、`x500_mono_cam`、`x500_vision`、`px4vision`、`standard_vtol`、`rc_cessna`、`advanced_plane`、`r1_rover`、`lawnmower`、`omnicopter`、`mono_cam`、`OakD-Lite` | PX4 `Tools/simulation/gz/models/` |
| XTDrone2 自有/改造 | `x500_lidar_2d`、`x500_lidar_down`、`x500_lidar_front`、`x500_gimbal`、`x500_mono_cam_down`、`lidar_2d_v2`、`gimbal`、`airspeed`、`quadtailsitter`、`tiltrotor`、`rover_ackermann` | XTDrone2 仓库 |
| **本项目新增** | `lidar_3d_v1`（16 线 3D 激光）、`x500_lidar_3d` | 补丁 0015 |
| 场景资产（另见） | `arucotag`、`cafe`、`cafe_table`、`cinder_block_wide`、`coke_can`、`grey_tote` | 上游/`osrf/gazebo_models` |

另有 **补丁 0012** 给 `x500_base/model.sdf` 增加 `OdometryPublisher` 插件（上游只有 `x500_vision` 有）。

**PX4 机架注册**（`ROMFS/.../init.d-posix/airframes/`）：14 个 `gz_*` 机架，
其中 `4012_gz_x500_lidar_2d`、`4013_gz_x500_mono_cam_down`、`4014_gz_x500_lidar_3d` 为本项目新增（补丁 0008/0013/0015）。

## Gazebo Classic 平台

> ⚠️ **当前不可用**: gazebo 包于 2026-11-26 被外部清理/安装移除 (仅存 libgazebo11)。
> 恢复: `sudo apt install gazebo` (需用户执行)。恢复后下表命令即可用。

| 世界 | 用途 | 启动命令 |
|---|---|---|
| empty (默认) | 通用飞行/演示 | `... classic iris` |
| warehouse | 仓库货架, 室内巡检 | `... classic iris warehouse` |
| ksql_airport | 机场, 固定翼 | `... classic plane ksql_airport` |
| mcmillan_airfield | 机场 | `... classic standard_vtol mcmillan_airfield` |
| sonoma_raceway | 赛道, 高速跟踪 | `... classic iris sonoma_raceway` |
| baylands | 湾区地形 | `... classic iris baylands` |
| yosemite | 山地地形 | `... classic iris yosemite` |
| ocean | 海面, 无人船 | `... classic boat ocean` |

Classic 世界列表: `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/`

## 场景截图

- walls 障碍场地: `screenshots/09_scene_walls.png`
- lawn 草地: `screenshots/11_scene_lawn.png`
- warehouse 仓库 (Classic): `screenshots/12_scene_warehouse.png`

## 参考仓库 (可扩展场景来源)

- [osrf/gazebo_models](https://github.com/osrf/gazebo_models) — 经典模型库 (已稀疏克隆 5 个)
- [XTDrone (Gitee)](https://gitee.com/robin_shaun/XTDrone) — ROS1 版更多世界 (森林/城市等, SDF 可移植)
- [aws-robotics worlds](https://github.com/aws-robotics) — small-house/hospital/bookstore (ROS2 友好)
- [Gazebo Fuel](https://app.gazebosim.org) — 在线模型库, Harmonic 原生资产

## 技术要点 (补丁 0009)

- 传感器 gz 话题含世界名 (`/world/<name>/model/...`), 桥接 yaml 已参数化 `${world}`,
  由 spawn launch 传入, 换世界无需改配置
- 新增世界资产流程: 模型放 `xtd2_gz_sim/models/` (实体目录, os.walk 不跟随符号链接)
  → `colcon build --packages-select xtd2_gz_sim` → 重启栈
