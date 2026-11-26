#!/usr/bin/env bash
# E12 Sim2Real 前置检查 (仿真侧)
# 用途: 迁移实机前, 在仿真中确认环境/话题/坐标/安全链就绪, 并生成参数模板
set -u
cd "$(dirname "$0")/.." || exit 1
OK=0; FAIL=0
chk() { if eval "$2" >/dev/null 2>&1; then echo "  [OK]   $1"; OK=$((OK+1)); else echo "  [FAIL] $1"; FAIL=$((FAIL+1)); fi; }

echo "== E12 Sim2Real 前置检查 =="
echo "-- 1. 环境"
chk "ROS2 Humble 环境" "test -f /opt/ros/humble/setup.bash"
chk "PX4 源码目录" "test -d PX4-Autopilot"
chk "MicroXRCEAgent" "command -v MicroXRCEAgent || test -x $HOME/.local/bin/MicroXRCEAgent"
chk "px4_msgs 已编译" "test -d ros2_ws/install/px4_msgs"
chk "drone_sim 已编译" "test -d ros2_ws/install/drone_sim"

echo "-- 2. 代码与文档"
chk "安全监督节点" "test -f ros2_ws/src/drone_sim/drone_sim/safety_supervisor.py"
chk "Sim2Real 文档" "test -f docs/SIM2REAL.md"
chk "集群智能体" "test -f ros2_ws/src/drone_sim/drone_sim/swarm_agent.py"
chk "行为树任务" "test -f ros2_ws/src/drone_sim/drone_sim/bt_mission.py"

echo "-- 3. 生成 PX4 参数模板"
TPL=docs/px4_params_template.txt
cat > "$TPL" <<'TPL_EOF'
# PX4 安全/整定参数模板 (仿真对齐值 -> 实机按场地调整)
GF_ACTION        2      # 地理围栏动作 2=返航
GF_MAX_HOR_DIST  100    # 水平围栏 m
GF_MAX_VER_DIST  120    # 垂直围栏 m
COM_RC_LOSS_T    5      # 遥控失联超时 s
NAV_RCL_ACT      2      # 失联动作 2=返航
COM_LOW_BAT_ACT  2      # 低电量动作 2=返航
COM_DISARM_PRFLT 10     # 未解锁自动上锁 s
MPC_XY_VEL_MAX   8.0    # 最大水平速度 m/s
MPC_Z_VEL_MAX_UP 3.0
MPC_Z_VEL_MAX_DN 1.5
MPC_XY_P         0.95
MPC_ACC_HOR      5.0
EKF2_HGT_MODE    0      # 0=气压 1=GPS 3=视觉
EKF2_GPS_CHECK   21
EKF2_EV_CTRL     15     # 外部视觉融合 (0=禁用)
TPL_EOF
echo "  [写入] $TPL"
grep -c "^" "$TPL" | xargs -I{} echo "  [行数] {}"

echo
echo "== 结果: $OK 通过, $FAIL 失败 =="
[ "$FAIL" -eq 0 ] && echo "仿真侧已就绪; 实机飞行前请按 docs/SIM2REAL.md 第 6 节清单逐项确认。"
exit 0
