#!/usr/bin/env bash
# ============================================================
# 一键启动仿真 (双平台切换)
# 用法:
#   bash scripts/run_simulation.sh                        # classic 平台 + iris + drone_sim 演示
#   bash scripts/run_simulation.sh classic typhoon_h480   # classic 平台换机型 (六旋翼)
#   bash scripts/run_simulation.sh classic standard_vtol  # 复合翼; 机型列表见 README
#   bash scripts/run_simulation.sh xtd2                   # XTDrone2 平台 (gz_x500, 需 Gazebo Harmonic)
#   bash scripts/run_simulation.sh xtd2 gz_x500_depth     # XTDrone2 换模型
# 说明: 机型是运行时参数, 不需要 git 分支; 两个平台共用同一份 PX4 编译产物。
# ============================================================
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PLATFORM="${1:-classic}"
MODEL="${2:-}"
export PATH=/usr/bin:$HOME/.local/bin:$PATH
export LD_LIBRARY_PATH=$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

case "$PLATFORM" in
classic)
    MODEL="${MODEL:-iris}"
    # ★ 磁盘安全: 禁用 px4 交互控制台 (px4 -d)。无 TTY 时 pxh> 会重绘死循环,
    #   曾把根分区 /tmp 写满 62G。NO_PXH 由 sitl_run.sh 转为 -d 参数, GUI 不受影响。
    export NO_PXH=1
    # ★ 日志全部写 /develop 大盘的 logs/, 且带大小上限
    LOGDIR="$ROOT/logs"; mkdir -p "$LOGDIR"
    CAP="$ROOT/scripts/capped_log.sh"
    echo "[classic] 机型: $MODEL  日志(带上限): $LOGDIR/"
    echo "[1/3] 启动 MicroXRCEAgent (UDP 8888)..."
    MicroXRCEAgent udp4 -p 8888 2>&1 | bash "$CAP" "$LOGDIR/xrce_agent.log" 20 &
    sleep 1

    echo "[2/3] 启动 PX4 SITL + Gazebo Classic (首次会自动编译)..."
    cd "$ROOT/PX4-Autopilot"
    # 用 cmake 直调 ninja, 绕开 make jobserver 在部分环境的 fd 错误
    cmake --build build/px4_sitl_default --target "gazebo-classic_${MODEL}" 2>&1 | bash "$CAP" "$LOGDIR/px4_sitl.log" 50 &

    echo "等待 PX4 就绪 (30s)..."
    sleep 30

    echo "[3/3] 启动 drone_sim offboard 演示 (起飞->画圆->降落)..."
    ros2 launch drone_sim demo.launch.py 2>&1 | bash "$CAP" "$LOGDIR/drone_demo.log" 20 &

    cleanup() {
        pkill -x MicroXRCEAgent 2>/dev/null || true
        pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
        pkill -x gzserver 2>/dev/null || true
        pkill -x gzclient 2>/dev/null || true
        pkill -f "ros2 launch drone_sim" 2>/dev/null || true
    }
    trap cleanup EXIT
    wait
    ;;

xtd2)
    MODEL="${MODEL:-gz_x500}"
    # XTDrone2 需要 Gazebo Harmonic (gz sim 8.x); 本机 /usr/bin/gz 可能是 Classic 的包装器
    if ! gz sim --version 2>/dev/null | grep -q "Gazebo Sim"; then
        echo "错误: 未检测到 Gazebo Harmonic (gz sim)。"
        echo "请先执行: bash scripts/install_gz_harmonic.sh  (需 sudo)"
        echo "装好后重跑本命令。classic 平台不受影响: bash scripts/run_simulation.sh classic"
        exit 1
    fi
    echo "[xtd2] 模型: $MODEL  PX4: $ROOT/PX4-Autopilot"
    # 命名空间与 PX4 实际模型名一致: {model去掉gz_前缀}_{id} (如 x500_mono_cam_0)
    NS="${MODEL#gz_}_0"
    echo "[xtd2] 命名空间: $NS"
    echo "XTDrone2 的 launch 会自行启动 MicroXRCEAgent / gz sim / PX4 SITL / 通信节点"
    LOGDIR="$ROOT/logs"; mkdir -p "$LOGDIR"
    # 关闭 Gazebo 窗口会触发 on_exit_shutdown 整栈停机, 但上游 shell=True 启动的
    # 孙进程(Agent/PX4/通信节点)会变孤儿残留 —— 这里兜底清理
    cleanup_xtd2() {
        pkill -x MicroXRCEAgent 2>/dev/null || true
        pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
        pkill -f "multirotor_communication" 2>/dev/null || true
        pkill -f "parameter_bridge" 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        echo "[xtd2] 已清理全部仿真进程"
    }
    trap cleanup_xtd2 EXIT
    # 日志写 /develop 大盘并限幅 (磁盘安全, 见 README)
    ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py \
        px4_dir:="$ROOT/PX4-Autopilot" \
        model_name:="$MODEL" \
        namespace:="$NS" 2>&1 | bash "$ROOT/scripts/capped_log.sh" "$LOGDIR/xtd2_sim.log" 50
    ;;

*)
    echo "未知平台: $PLATFORM (可选: classic | xtd2)"
    exit 1
    ;;
esac
