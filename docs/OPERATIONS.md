# 启动与控制手册

## 〇、环境准备（每个新终端）

```bash
source /opt/ros/humble/setup.bash
source /develop/IDEA/Drone_ros2/ros2_ws/install/setup.bash
export PATH=/usr/bin:$HOME/.local/bin:$PATH
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH   # 仅启动 Agent 的终端需要
cd /develop/IDEA/Drone_ros2
```

顺序不能反：先 humble 底座，再叠加工作空间 overlay。`cd` 到哪不重要，`source` 才重要
（ROS2 找包靠 `AMENT_PREFIX_PATH`）。验证: `ros2 pkg list | grep -E "drone_sim|px4_msgs|xtd2"`。

## 一、启动仿真

```bash
bash scripts/run_simulation.sh                      # Classic + iris + 自动演示
bash scripts/run_simulation.sh classic typhoon_h480 # 六旋翼
bash scripts/run_simulation.sh classic standard_vtol# 复合翼
bash scripts/run_simulation.sh xtd2                 # Harmonic + gz_x500（等待控制指令）
```

重复启动前先停旧栈（端口冲突）:

```bash
pkill -f "ros2 launch xtd2_launc[h]"; pkill -x MicroXRCEAgent
pkill -f "px4_sitl_default/bin/p[x]4"; pkill -f "gz si[m]"
pkill -f "multirotor_communicatio[n]"; pkill -f "parameter_bridg[e]"
```

⚠️ 不要直接关 Gazebo 窗口：xtd2 的 `on_exit_shutdown` 会联动停掉整栈，
且可能留下孤儿进程（Agent/PX4），需按上面命令清理后重启。

## 二、控制方式

### A. 自动演示（drone_sim: 起飞→画圆→降落）

```bash
# Classic:
ros2 launch drone_sim demo.launch.py
# 调参:
ros2 run drone_sim offboard_demo --ros-args -p takeoff_altitude:=8.0 -p circle_radius:=5.0 -p circle_duration:=40.0

# xtd2（重映射到 x500_0 命名空间）:
ros2 run drone_sim offboard_demo --ros-args \
  -r /fmu/in/offboard_control_mode:=/x500_0/fmu/in/offboard_control_mode \
  -r /fmu/in/trajectory_setpoint:=/x500_0/fmu/in/trajectory_setpoint \
  -r /fmu/in/vehicle_command:=/x500_0/fmu/in/vehicle_command \
  -r /fmu/out/vehicle_status:=/x500_0/fmu/out/vehicle_status \
  -r /fmu/out/vehicle_local_position:=/x500_0/fmu/out/vehicle_local_position
```

### B. 键盘手飞（xtd2 平台）

```bash
ros2 run xtd2_control multirotor_keyboard_control --model gz_x500 --id 0 --namespace x500_0 \
  --ros-args -p linear_velocity_step:=0.5 -p angular_velocity_step:=0.2 \
             -p max_linear_velocity:=5.0 -p max_angular_velocity:=1.5
```

**起飞三步曲: `b`(开 Offboard) → `t`(解锁) → `v`(起飞)**。
不按 `b` 时通信节点处于 DISABLED 状态，速度指令全部丢弃。

移动键（累加式速度，每按一次加一个步进）:
`w/x` 前后、`a/d` 左右、`i/,` 升降、`j/l` 偏航、`s` 悬停清零、
`n` 降落、`y` 上锁、`r` 返航、`Ctrl-C` 退出。

**默认步进仅 0.01 m/s（按 100 次才 1 m/s，几乎不动）——务必用上面的调参启动。**
解锁被拒（Preflight 警告）时等 10–20 s EKF 收敛后重试 `t`。

### C. 纯话题手控（两平台通用，offboard 三要素）

```bash
# 1. 心跳+setpoint 必须 >2Hz（NED: z 负值=向上）
while true; do
  ros2 topic pub -t 1 /x500_0/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
    '{position: true, timestamp: 0}' >/dev/null
  ros2 topic pub -t 1 /x500_0/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
    '{position: [0.0, 0.0, -3.0], yaw: 0.0}' >/dev/null
  sleep 0.1
done &
# 2. 切 offboard(176: param1=1,param2=6) + 解锁(400: param1=1)
ros2 topic pub -t 3 /x500_0/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  '{command: 176, param1: 1.0, param2: 6.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}'
ros2 topic pub -t 3 /x500_0/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  '{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}'
# 3. 降落: command 21。Classic 平台把 /x500_0/fmu/... 换成 /fmu/...
```

### D. 官方示例（Classic，带 RViz）

```bash
ros2 launch px4_offboard offboard_control.launch.py
ros2 launch px4_offboard visualize.launch.py
```

## 三、监控

```bash
ros2 topic echo /x500_0/fmu/out/vehicle_local_position --once  # 位置 (NED)
ros2 topic echo /x500_0/fmu/out/vehicle_status --once          # arming_state: 1=上锁 2=解锁
ros2 topic hz /x500_0/fmu/in/trajectory_setpoint               # setpoint 频率
tail -f logs/xtd2_sim.log                                      # 仿真日志（已限幅）
```

## 四、避坑速查

| 症状 | 处理 |
|---|---|
| 改了 drone_sim 代码不生效 | `colcon build --symlink-install --packages-select drone_sim`（复制非链接） |
| 新节点收不到话题/卡死 | 整栈重启；必要时停全部进程后清 `/dev/shm/fastrtps_*` |
| `gz: command not found` | `ln -sf /usr/bin/gz-classic ~/.local/bin/gz` |
| Agent 报找不到库 | `export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH` |
| 键盘控制没反应 | 先按 `b` 再 `t` 再 `v`；用调参命令启动（默认步进 0.01 太小） |
| 日志别写 /tmp | 一律走 `logs/`（脚本已内置限幅，防写爆根分区） |

## 双远端同步（GitHub + Gitee）

本仓库同时托管在两个平台，**一次 `git push` 即可同步两处**（`origin` 配置了多个 push URL）：

| 远端 | 地址 | 认证 |
|---|---|---|
| GitHub | `github.com/howe12/Drone_ros2` | HTTPS（网络不稳时代理/直连交替重试）|
| Gitee | `gitee.com/huo-haijie/Drone_ros2` | SSH（公钥已配置，免密）|

```bash
git push origin main          # 一条命令 -> 同时推 GitHub 和 Gitee
bash scripts/push_all.sh      # 备选: 容错版(逐个推, 一个失败不影响另一个, 带重试)
```

查看/修改推送目标：

```bash
git remote get-url --push --all origin    # 列出所有 push 目标
git remote set-url --add --push origin <新地址>   # 追加目标
```

### 注意

- 多 URL 推送时，**任一目标失败会让 `git push` 返回非零**（但其他目标通常已成功）。
  网络差时用 `bash scripts/push_all.sh`，它对每个远端单独重试并分别报告结果。
- Gitee 侧走 SSH，需要 `~/.ssh` 公钥已在 Gitee「设置 → SSH 公钥」注册；
  换机器或换用户时要重新配置。
- 校验两处是否一致：

```bash
git rev-parse --short HEAD
git ls-remote origin main | cut -c1-7
git ls-remote gitee main  | cut -c1-7
```
