# 仿真场景画廊 (E13)

世界选择已参数化: `bash scripts/run_simulation.sh <平台> <机型> <世界>`

## Harmonic / XTDrone2 平台

| 世界 | 用途 | 状态 | 启动命令 |
|---|---|---|---|
| aruco | ArUco 靶标场地, 精准降落 (E8) | ✅ 实测 | `... xtd2 gz_x500 aruco` |
| walls | 障碍墙场地, 避障规划 (E7) | ✅ 实测 (激光 30Hz) | `... xtd2 gz_x500_lidar_2d walls` |
| lawn | 草地户外, 通用飞行 | ✅ 实测 | `... xtd2 gz_x500 lawn` |
| baylands | 湾区大地形, 长航程 | 可用 (上游自带) | `... xtd2 gz_x500 baylands` |
| default | 极简默认 | 可用 | `... xtd2 gz_x500 default` |
| windy | 风扰测试 | 可用 | `... xtd2 gz_x500 windy` |
| rover | 无人车场景 | 可用 (配 gz_r1_rover) | `... xtd2 gz_r1_rover rover` |
| indoor1 | 室内 cafe 场景, 室内 SLAM (E4) | ⚠️ 已知问题 | 见下 |

### indoor1 已知问题
引用旧 Gazebo Fuel 模型 (cafe/cafe_table/cinder_block_wide/coke_can/grey_tote)。
模型已从 osrf/gazebo_models 稀疏克隆补齐 (third_party/gazebo_models, 复制进
xtd2_gz_sim/models 并重编), 但 Harmonic 加载时 gz sim 内部 abort (旧模型 SDF/网格
兼容性问题)。**待修**: 逐个替换问题模型或用 Harmonic 原生室内资产重建。
室内 SLAM 可先用 walls + 自建简单室内世界替代。

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
