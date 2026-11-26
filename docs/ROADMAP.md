# 功能现状 × 课程对比 × 待开发工程清单

> 参照系: XTDrone 教程体系（北大团队配套课程/中国机器人大赛仿真组大纲）+ XTDrone2 功能大纲
> （单机控制/多机编队/避障导航/路径规划/任务规划/集群控制）。
> 如有具体课程大纲，可提供后做精确对齐。

## 一、现有能力盘点

| 能力 | 状态 | 载体 |
|---|---|---|
| 双仿真平台（Gazebo Classic / Harmonic+XTDrone2） | ✅ 实测飞行 | run_simulation.sh 双平台切换 |
| Offboard 位置控制（起飞/画圆/降落状态机） | ✅ 参数化 | drone_sim/offboard_demo.py |
| 键盘速度手飞 | ✅ 步进调参 | xtd2_control |
| ROS2↔PX4 DDS 链路 | ✅ | MicroXRCEAgent + px4_msgs |
| 传感器桥接 | ✅ 相机/IMU/深度/点云/激光全实测 (E1 完成) | ros_gz_bridge + 补丁0007/0008 |
| 多机命名空间机制 | ⚠️ 有基础无演示 | PX4_UXRCE_DDS_NS / x500_0 |
| 传感器机型模型库 | ✅ 模型齐全未启用 | x500_depth/mono_cam/lidar_2d/gimbal；iris_depth_camera/foggy_lidar 等 |
| 工程化（一键复现/补丁/日志安全/文档） | ✅ | setup_workspace.sh + patches 0001-0006 |

## 二、课程模块对比矩阵

| 课程模块 | 典型教学内容 | 我们的现状 | 差距 |
|---|---|---|---|
| 1 仿真与控制基础 | offboard、位置/速度/姿态控制、轨迹跟踪 | ✅ 位置控制+速度手飞 | 缺姿态控制、自定义轨迹、RViz 可视化 |
| 2 状态估计 | EKF、VIO(VINS-Fusion)、多传感器融合 | ⚠️ 仅用 PX4 内置 EKF2 | 缺外部视觉惯性里程计接入 |
| 3 SLAM | 视觉(ORB-SLAM3)、2D激光(Cartographer)、3D激光(LIO-SAM/FAST-LIO) | ❌ 无 | 传感器话题未桥接，算法包未集成 |
| 4 运动规划 | 2D(A*/RRT)、3D(Fast-Planner/ego-planner)、避障 | ❌ 无 | 缺深度/点云→局部地图→规划器链路 |
| 5 视觉应用 | YOLO 检测、目标跟踪、ArUco 精准降落 | ❌ 无（世界里有 arucotag 现成靶标） | 缺相机桥接+视觉节点 |
| 6 多机/集群 | 编队、分布式规划(ego_planner_swarm)、协同 | ⚠️ 命名空间机制就绪 | 缺多机 spawn/编队控制/机间通信 |
| 7 任务规划 | 任务状态机、行为树 | ⚠️ 硬编码状态机 | 缺可配置任务层(BT/航点文件) |
| 8 其他机型 | 固定翼/复合翼/车/船 | ⚠️ 模型可一键切换 | 缺对应控制逻辑(FW/VTOL 需专用接口) |
| 9 实机部署 | sim2real、参数整定、安全 | ❌ 纯仿真 | 远期 |

## 三、待开发工程清单（按优先级）

### P0 — 打通感知与任务基础（1–2 周量级）

**E1. 传感器话题桥接工程** `drone_sim` 扩展 ✅ **已完成 (2026-11-26, Issue #1)**
- 对应模块: 2/3/4/5 的前置条件
- 成果: 相机 23.6Hz(1280x960) / IMU 253Hz / 2D激光 30.4Hz(270°/30m) / 深度图 25.6Hz / 点云 28.1Hz
- 新增 PX4 机架 4012_gz_x500_lidar_2d；命名空间自动推导；补丁 0007/0008
- 启动: `bash scripts/run_simulation.sh xtd2 gz_x500_mono_cam|gz_x500_depth|gz_x500_lidar_2d`

**E2. 航点任务与可视化** `drone_sim/waypoint_mission` ✅ **已完成 (2026-11-26, Issue #2)**
- 成果: YAML 航点 + 4 种轨迹模式 (square/circle/spiral/figure8)，正方形/圆形全程实测，
  RViz Path/Marker 可视化，到点误差 ≤0.35m，自动降落
- 用法: `ros2 launch drone_sim waypoint_mission.launch.py prefix:=/x500_0 mission:=square`

**E13. 仿真场景扩展** ✅ **已完成 (2026-11-26, Issue #13)**
- 世界切换参数化 (补丁0009)，walls/lawn/aruco 实测，画廊 docs/SCENARIOS.md
- 已知问题: indoor1 资产兼容性 (gz abort)；Classic gazebo 包被外部移除待 sudo 恢复

**E3. 多机 spawn 与编队入门** `drone_sim/multi_vehicle + formation_follower` ✅ **已完成 (2026-11-26, Issue #3)**
- 成果: N 机 launch (num 参数)、leader-follower 编队实测 (双机同时起飞, 队形误差 0.14-0.29m)
- 关键修复: 补丁0010 每实例独立工作目录 (parameters.bson 冲突); 命令 target_system 必须匹配 MAV_SYS_ID (实例号+1)
- 用法: ros2 launch drone_sim multi_vehicle.launch.py num:=2 px4_dir:=<PX4路径>

### P1 — 核心算法集成（每项 1–3 周量级）

**E4. 2D 激光 SLAM** ✅ **已完成 (2026-11-26, Issue #4)**
- slam_toolbox (Humble 原生) + walls 世界建图飞行实测: 252×410 栅格 @0.05m, 631 障碍格
- 地图: maps/e4_walls_map.pgm+yaml (nav2 格式), 截图 14/15
- 关键修复: 补丁0011 /clock+协方差里程计桥接; 补丁0012 x500_base 加 OdometryPublisher 插件
  (上游仅 x500_vision 有); odom_to_tf 节点 + base_link→lidar 静态 TF; use_sim_time 全链路
- 用法: ros2 launch drone_sim slam_2d.launch.py (栈需 walls+lidar 机型)

**E5. 3D 激光 SLAM/里程计** ✅ **已完成 (2026-11-26, Issue #5)**
- Point-LIO (FAST-LIO2 后继算法, thamyis100/point_lio_ros2 @59d327bf) 编译集成
- 自建 16线3D激光模型 lidar_3d_v1 + x500_lidar_3d + 机架 4014 (补丁0008/0015)
- lidar3d_adapter: gz 点云补 ring/time 字段 + inf 过滤 (无回波点破坏 ikd-tree)
- livox_ros_driver2 消息桩包入库 (无需 Livox SDK)
- 实测: /odom 8Hz, 高度误差 9cm (LIO 4.87m vs 真值 4.96m), 3m 正方形轨迹完整复现,
  配准点云 43808 点覆盖 walls 场景 (maps/e5_registered_cloud.npy, 截图 19/20)
- 关键修复: EKF 原点重置门控 —— z 原点跳变使"地面=local z=-5", 任务误判已起飞致贴地爬行;
  waypoint_mission 现要求重置计数器稳定 5s 才解锁
- 用法: ros2 launch drone_sim point_lio.launch.py (栈需 walls + gz_x500_lidar_3d)

**E6. 视觉惯性里程计** — VINS-Fusion ROS2 移植 + x500_mono_cam(+IMU)
- 对应模块: 2；难度 ★★★★（编译/标定/时间同步坑多）

**E7. 局部规划与避障** ⚠️ **部分完成 (2026-11-26, Issue #7 保持 open)**
- ego-planner (Kaede-Rukawa/ego_planner_ros_humble @6aa3eb9d) 编译集成 (需链接器改 gold: 2072 库/目标)
- 深度相机(OakD-Lite 32FC1) → grid_map 局部占据栅格 → B样条规划 → traj_server → /position_cmd
- drone_sim/ego_bridge: position_cmd → PX4 Offboard (自动解锁/心跳/速度前馈限幅)
- 实测: 规划输出正确 (5.94,0,1.20 = 目标 6,0,1.2), 障碍最小距离 0.82m (不碰撞)
- **阻塞**: EKF 航向/原点重置 (heading_reset_counter/xy_reset_counter) 会使 PX4 局部系
  相对 gz 世界系旋转, 固定 ENU→NED 映射飞行中失配 → 目标收敛不稳定 (漂移)
- 下一步: 用 PX4 视觉/动捕里程计输入或在线估计两系变换; 或用 PX4 原生 EKF 输出给 ego

**E8. 视觉检测与精准降落** ✅ **已完成 (2026-11-26, Issue #8)**
- ArUco 精准降落世界系真值验证: **落点距靶标中心 0.052m** (0.5m 靶标)
- 关键发现: EKF 局部原点会重置 (xy_reset_counter=3), 落点精度必须用世界系位姿验证;
  伺服以图像光心为基准 (原点无关) + 偏航旋转 + 高度自适应增益 + 统计居中判定
- 新增机架 4013_gz_x500_mono_cam_down (补丁0008 含 4012/4013); 靶标纹理替换为标准 ArUco ID42
- YOLO: yolov8n 权重入包, 样本图实测 bus:0.87/person×3; ROS 节点需 conda python+LD_PRELOAD
  (scripts/run_yolo.sh 包装)
- 截图 16/17/18; 用法: ros2 run drone_sim aruco_precision_landing

### P2 — 进阶方向（按需）

**E9. 集群规划** — ego_planner_swarm（XTDrone2 路线图同款）；依赖 E3+E7
**E10. 任务层行为树** — BehaviorTree.CPP/py_trees 任务编排；依赖 E2
**E11. 固定翼/VTOL 控制** — px4-ros2-interface-lib（官方新接口，注册自定义模式）
**E12. sim2real** — 真机参数整定、安全框架、日志回放

## 四、建议路线

```
E1 传感器桥接 ──┬── E4 2D SLAM ──── E7 规划避障 ─── E9 集群
                ├── E5 3D SLAM ──┘        ↑
                ├── E6 VIO ───────────────┘
                └── E8 检测+精准降落
E2 航点任务 ──── E10 行为树
E3 多机编队 ─────────────────────────── E9 集群
```

里程碑建议:
- M1（P0 完成）: 相机/激光出图 + 航点任务 + 双机编队 → 覆盖课程模块 1/6/7 基础
- M2（P1 任选 2 项）: SLAM 建图 + 避障飞行 或 检测+精准降落 → 覆盖模块 3/4/5
- M3（P2）: 集群/任务层/实机 → 模块 6/7/9 进阶
