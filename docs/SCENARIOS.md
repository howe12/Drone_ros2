# 仿真场景画廊 (E13)

世界选择已参数化: `bash scripts/run_simulation.sh <平台> <机型> <世界>`

## Harmonic / XTDrone2 平台

| 世界 | 用途 | 来源 | 状态 | 启动命令 |
|---|---|---|---|---|
| aruco | ArUco 靶标场地, 精准降落 (E8) | XTDrone2 (贴图由我们替换为 DICT_4X4_50 ID42) | ✅ 实测 | `... xtd2 gz_x500 aruco` |
| walls | 障碍墙场地, 避障/2D 激光 SLAM | XTDrone2 (与 PX4 同名世界内容不同) | ✅ 实测 (激光 30Hz) | `... xtd2 gz_x500_lidar_2d walls` |
| **e7_obstacles** | 小块障碍群, 局部规划避障 (E7) | **自建** (补丁 0016) | ✅ 实测 | `... xtd2 gz_x500_depth e7_obstacles` |
| **yolo_targets** | 3 车 + 3 行人, 目标检测（含 COCO 类别）| **自建** (补丁 0019 + Fuel 模型) | ✅ 实测 (person 0.71~0.81) | `... xtd2 gz_x500_mono_cam yolo_targets` |
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

## 上游自带的仿真世界（XTDrone2）

XTDrone2 仓库（`third_party/XTDrone2/xtd2_gz_sim/worlds/`）自带 **8 个世界**。
下表逐一说明内容与"是否需要联网/外部资产"——这一列很关键，直接决定能不能离线跑：

| 世界 | 内容 | 自包含? | 说明 |
|---|---|---|---|
| `default` | 空场：地面 + 光照（1 个内联模型）| ✅ 全本地 | 控制调试首选 |
| `aruco` | 地面 + `model://arucotag` 靶标 | ✅ 本地模型 | 视觉降落（靶标贴图我们已换成 DICT_4X4_50 ID42）|
| `walls` | 5 个内联箱体组成的墙体 | ✅ 全本地 | 避障 / 2D 激光 SLAM |
| `lawn` | 草地地面（与 PX4 上游逐字节相同）| ✅ 全本地 | 通用飞行 |
| `windy` | 地面 + `<wind><linear_velocity>10 10 10</linear_velocity>`，且 `<enable_wind>true` | ✅ 全本地 | **真实的 10 m/s 三轴风扰**，适合做鲁棒性实验 |
| `rover` | 地面（配 `gz_r1_rover` 机型）| ✅ 全本地 | 地面车对照实验 |
| `baylands` | **运行时从 Gazebo Fuel 下载** `OpenRobotics/baylands`（约 393 MB）+ `Coast Water` | ❌ **需联网** | 大场景/视觉；首次启动要下模型 |
| `indoor1` | 25 个内联模型 + 引用 Gazebo **Classic** 资产（cafe / coke_can / grey_tote…）| ❌ 缺资产，gz 崩溃 | 室内场景，**当前不可用** |

> `baylands` 的坑：它的模型缓存到 `~/.gz/fuel/` 后，`model.sdf` 里的网格与贴图仍是
> **https 绝对地址**，所以每次加载都要联网；网络受限时贴图取不到，模型会渲染成**纯黑剪影**
> （本项目里"树是黑的"就是这个原因，不是我们的材质写错了）。
> 可执行 `bash scripts/fix_fuel_offline.sh` 把 URI 改写为本地相对路径，之后即可离线加载。
> 注：树即使改成本地路径仍可能是黑的——那是 DAE 内部材质问题，与 URI 无关。

### 我们新增的世界

| 世界 | 内容 | 补丁 |
|---|---|---|
| `e7_obstacles` | 两处小方块障碍群，用于局部规划避障 | 0016 |
| `yolo_targets` | 3 辆车 + 3 个行人（Gazebo Fuel 模型），用于目标检测 | 0019 |
| `indoor_hall` | **20×20 m 室内大厅**：地面 + 4 墙 + 天花板 + 5 盏吊灯 + **36 件家具**（货架/办公桌/椅子/托盘/货箱/冰箱），中央留 6 m 宽通道 | 0020 |

`indoor_hall` 实测状态（2026-11-28）：

| 项 | 结果 |
|---|---|
| 世界加载（36 件家具全部解析）| ✅ 无报错 |
| 机载相机画面 | ✅ 正常（能看到地板/墙/吊灯/桌椅/货箱，见 `screenshots/20_indoor_hall.png`）|
| 无人机解锁 | ✅ `armed=2`，进入 Offboard |
| **室内起飞爬升** | ⚠️ **未达标**：指令起飞至 2.0 m，实测悬停在 0.3 m 不动（机身可见于场景中）|

> 已知问题：室内起飞不爬升，原因待查（怀疑与 EKF 高度估计或室内首次定位有关，日志有
> `Preflight Fail: ekf2 missing data` 反复出现）。地面/室外世界起飞正常，所以不是节点问题。
> 需要先修这个问题，再谈"室内飞行"类课程章节。

### 为什么室内场景是自建而不是直接用一个 Fuel 模型？

试过 `OpenRobotics/Warehouse`（100×100 m，只有柱子）与 `OpenRobotics/Apartment`
（约 20×22 m，家具多）。两者都能加载，但存在同一个问题：**这类 Fuel 建筑的墙面是单面网格**，
无人机进入内部后朝外看会“穿透”墙体，只能看到背景色（实测机载相机输出为纯色空白帧）。
而且仓库模型自身定位不适合做飞行起点。

因此 `indoor_hall` 用**可完全控制的基本几何**（箱体墙/地面/天花板）+ Fuel 小家具搭建：
- 墙是双面箱体 → 内部看得到墙，画面正常
- 家具各带贴图，摆放位置可控 → 想加多少物体就加多少
- 20×20 m + 中央 6 m 通道 → 既“开阔”又“物体多”

家具模型由 `scripts/fetch_fuel_models.sh` 按需下载（约 5 MB，不入库）。

### XTDrone（ROS1 原版）有多少世界？

本仓库用的是 **XTDrone2**（ROS2 + Gazebo Ignition）。其前身 **XTDrone（ROS1）** 按官方 README
明确写有 **"两个户外场景 + 三个室内场景"**（见
[XTDrone README](https://raw.giteeusercontent.com/Doer/XTDrone/raw/master/README.md) "仿真配置 → Gazebo世界"）。
ROS1 版的世界是 SDF 文件，理论上可移植到 Harmonic，但需要逐个验证资产兼容性——这正是 `indoor1`
踩过的坑（它引用的就是 Classic 时代资产）。

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
