#!/usr/bin/env bash
# ============================================================
# 一键复现 Drone_ros2 工作区 (Ubuntu 22.04 + ROS2 Humble)
# 用法: bash scripts/setup_workspace.sh
# 说明: 第三方仓库不入库, 由本脚本按锁定版本克隆;
#       本地适配修改以 patches/ 形式保存并自动应用。
# ============================================================
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
NPROC=$(nproc)

echo "[1/6] 克隆 PX4 v1.15.4 (浅克隆 + 子模块)..."
if [ ! -d PX4-Autopilot ]; then
    git clone -b v1.15.4 --depth 1 --recurse-submodules --shallow-submodules --jobs 8 \
        https://github.com/PX4/PX4-Autopilot.git
fi
# 提示: 若 GitHub 直连慢/断连, sitl_gazebo-classic 大仓库可走镜像 (git SHA 校验保证完整性):
#   cd PX4-Autopilot
#   git config submodule.Tools/simulation/gazebo-classic/sitl_gazebo-classic.url \
#       https://ghfast.top/https://github.com/PX4/PX4-SITL_gazebo-classic.git
#   git submodule update --depth 1 -- Tools/simulation/gazebo-classic/sitl_gazebo-classic
#   git -C Tools/simulation/gazebo-classic/sitl_gazebo-classic rev-parse HEAD
#   # 必须等于 da7206e057703cc645770f02437013358b71e1c0, 然后把 url 改回 github

echo "[2/6] 克隆 ROS2 依赖包 (与 PX4 1.15 配套分支)..."
mkdir -p ros2_ws/src
[ -d ros2_ws/src/px4_msgs ]     || git clone -b release/1.15 --depth 1 https://github.com/PX4/px4_msgs.git ros2_ws/src/px4_msgs
[ -d ros2_ws/src/px4_ros_com ]  || git clone -b release/1.15 --depth 1 https://github.com/PX4/px4_ros_com.git ros2_ws/src/px4_ros_com
[ -d ros2_ws/src/px4-offboard ] || git clone --depth 1 https://github.com/Jaeyoung-Lim/px4-offboard.git ros2_ws/src/px4-offboard
# 本地适配: 修复 px4-offboard 误注册 visualize.rviz 包标记 (会炸 ros_gz humble 的 gz_sim.launch)
git -C ros2_ws/src/px4-offboard apply "$ROOT/patches/0005-px4-offboard-fix-rviz-marker.patch" 2>/dev/null || echo "  补丁5已应用, 跳过"

echo "[2.5/6] 克隆 XTDrone2 (可选仿真平台, 锁定 commit 15b2ef6)..."
mkdir -p third_party
if [ ! -d third_party/XTDrone2 ]; then
    git clone --depth 1 https://github.com/zuiyue52/XTDrone2.git third_party/XTDrone2 \
    || git clone --depth 1 https://gitee.com/andy_zhuo/XTDrone2.git third_party/XTDrone2
    # 锁定到已验证的 commit (GitHub 允许按 SHA 浅 fetch)
    git -C third_party/XTDrone2 fetch --depth 1 origin 15b2ef6eb9fd8c2631f931a27064f323d26a9ed3 \
        && git -C third_party/XTDrone2 checkout 15b2ef6eb9fd8c2631f931a27064f323d26a9ed3
fi
# 防止其自带的 px4_msgs 子模块与 ros2_ws/src/px4_msgs 重复 (colcon 重复包报错)
mkdir -p third_party/XTDrone2/xtd2_third_party_pkgs/px4_msgs
touch third_party/XTDrone2/xtd2_third_party_pkgs/px4_msgs/COLCON_IGNORE
# 本地适配: 暴露 px4_dir / model_name launch 参数 (指向本工作区的 PX4)
git -C third_party/XTDrone2 apply "$ROOT/patches/0003-xtd2-px4-dir-and-model-args.patch" 2>/dev/null || echo "  补丁3已应用, 跳过"
# 本地适配: 移除未使用的 RosGzBridge 导入 (Humble 二进制包无该 Python 模块, 会炸 launch 加载)
git -C third_party/XTDrone2 apply "$ROOT/patches/0004-xtd2-remove-unused-rosgzbridge-import.patch" 2>/dev/null || echo "  补丁4已应用, 跳过"
# 本地适配: Humble 无 ros_gz_bridge.launch.py, 桥接改调 parameter_bridge
git -C third_party/XTDrone2 apply "$ROOT/patches/0006-xtd2-bridge-parameter-bridge.patch" 2>/dev/null || echo "  补丁6已应用, 跳过"
# E1: 传感器桥接 yaml 扩展 + 全部 gz 机架 autostart 映射
git -C third_party/XTDrone2 apply "$ROOT/patches/0007-xtd2-sensor-bridge-and-autostart.patch" 2>/dev/null || echo "  补丁7已应用, 跳过"
# E13: 桥接世界名参数化 (传感器路径 /world/<name>/)
git -C third_party/XTDrone2 apply "$ROOT/patches/0009-xtd2-world-parameterization.patch" 2>/dev/null || echo "  补丁9已应用, 跳过"
# E3: 多实例独立工作目录 (防 parameters.bson 冲突)
git -C third_party/XTDrone2 apply "$ROOT/patches/0010-xtd2-per-instance-workdir.patch" 2>/dev/null || echo "  补丁10已应用, 跳过"
# E4: /clock 桥接 (use_sim_time 节点必需)
git -C third_party/XTDrone2 apply "$ROOT/patches/0011-xtd2-clock-bridge.patch" 2>/dev/null || echo "  补丁11已应用, 跳过"
# E4: x500_base 里程计发布插件 (SLAM TF 链路必需)
git -C third_party/XTDrone2 apply "$ROOT/patches/0012-xtd2-x500-odometry-plugin.patch" 2>/dev/null || echo "  补丁12已应用, 跳过"
# E8: 下视相机机架 autostart + ArUco 标准靶标纹理
git -C third_party/XTDrone2 apply "$ROOT/patches/0013-xtd2-downcam-autostart.patch" 2>/dev/null || echo "  补丁13已应用, 跳过"

# E13: indoor1 场景所需旧 Fuel 模型 (稀疏克隆, 实体复制——os.walk 不跟随符号链接)
if [ ! -d third_party/gazebo_models ]; then
    git clone --filter=blob:none --sparse --depth 1 https://github.com/osrf/gazebo_models.git third_party/gazebo_models \
    || git clone --filter=blob:none --sparse --depth 1 https://ghfast.top/https://github.com/osrf/gazebo_models.git third_party/gazebo_models
    git -C third_party/gazebo_models sparse-checkout set cafe cafe_table cinder_block_wide coke_can grey_tote
fi
for m in cafe cafe_table cinder_block_wide coke_can grey_tote; do
    [ -d "third_party/XTDrone2/xtd2_gz_sim/models/$m" ] || cp -r "third_party/gazebo_models/$m" "third_party/XTDrone2/xtd2_gz_sim/models/$m"
done
# E8: 上游 arucotag 纹理为非标准图案(无法检测), 替换为标准 ArUco DICT_4X4_50 ID42
python3 - <<'PYEOF'
import cv2, numpy as np, os
p = 'third_party/XTDrone2/xtd2_gz_sim/models/arucotag/arucotag.png'
if os.path.exists(p) and not os.path.exists(p.replace('.png', '_original.png')):
    import shutil; shutil.copy(p, p.replace('.png', '_original.png'))
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
m = cv2.aruco.generateImageMarker(d, 42, 500, borderBits=1)
img = np.full((700, 700), 255, dtype=np.uint8); img[100:600, 100:600] = m
cv2.imwrite(p, img)
print('arucotag texture: DICT_4X4_50 ID42')
PYEOF

echo "[3/6] 应用本地适配补丁 (无 sudo 环境)..."
git -C PX4-Autopilot apply "$ROOT/patches/0001-px4-sitl-gazebo-classic-gstreamer-off.patch" 2>/dev/null || echo "  补丁1已应用, 跳过"
# E1: 新增 gz_x500_lidar_2d 机架 (4012, ROMFS 显式列表注册)
git -C PX4-Autopilot apply "$ROOT/patches/0008-px4-gz-lidar-airframe.patch" 2>/dev/null || echo "  补丁8已应用, 跳过"
SGC=PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
git -C "$SGC" apply "$ROOT/patches/0002-sitl-gazebo-classic-qt5-fix.patch" 2>/dev/null || echo "  补丁2已应用, 跳过"
# SITL 不需要 NuttX: 若其工作树为空但残留 .git gitlink, 移除以免版本脚本误报
NUTTX=PX4-Autopilot/platforms/nuttx/NuttX/nuttx
if [ -f "$NUTTX/.git" ] && [ -z "$(ls -A "$NUTTX" | grep -v '^\.git$')" ]; then
    rm "$NUTTX/.git"
fi

echo "[4/6] Python 编译依赖 (装到当前 python3 环境)..."
python3 -m pip install --quiet empy==3.3.4 jinja2 jsonschema kconfiglib numpy packaging \
    pyros-genmsg pyserial pyyaml toml future

echo "[5/6] 编译 Micro-XRCE-DDS-Agent (main 分支, 装到 ~/.local)..."
# 注意: v2.4.2 不可用 (其依赖的 FastDDS 2.12.x 分支已被上游删除)
[ -d Micro-XRCE-DDS-Agent ] || git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
# 用干净 PATH 避免 conda 头文件污染; fmt_DIR 指向系统 fmt (与系统 spdlog 配套)
CLEAN_PATH=/usr/bin:/bin:$HOME/.local/bin
( cd Micro-XRCE-DDS-Agent \
  && env PATH=$CLEAN_PATH cmake -B build -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_INSTALL_PREFIX=$HOME/.local -DUAGENT_BUILD_TESTS=OFF \
       -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
  && env PATH=$CLEAN_PATH cmake --build build -j"$NPROC" \
  && env PATH=$CLEAN_PATH cmake --install build )

echo "[6/6] 编译 PX4 SITL + Gazebo 插件 + ROS2 工作区..."
( cd PX4-Autopilot \
  && { DONT_RUN=1 make px4_sitl -j"$NPROC" || ninja -C build/px4_sitl_default -j"$NPROC"; } \
  && ninja -C build/px4_sitl_default sitl_gazebo-classic )
# 注: make 的 jobserver 在部分环境报 fd 错误, 上面已自动回退为 ninja 直调

echo "[6.5/6] ros_gz 源码 (仅 xtd2 平台需要; 二进制包只支持 Fortress, Harmonic 需源码编译)..."
if [ ! -d third_party/ros_gz ]; then
    git clone --depth 1 -b humble https://github.com/gazebosim/ros_gz.git third_party/ros_gz \
    || git clone --depth 1 -b humble https://ghfast.top/https://github.com/gazebosim/ros_gz.git third_party/ros_gz
fi
# 若已装 gz-harmonic (bash scripts/install_gz_harmonic.sh), 按 Harmonic 编译桥接:
if gz sim --version 2>/dev/null | grep -q "Gazebo Sim"; then
    ( source /opt/ros/humble/setup.bash && cd ros2_ws \
      && GZ_VERSION=harmonic colcon build --symlink-install \
           --base-paths src ../third_party/XTDrone2 ../third_party/ros_gz \
           --packages-up-to ros_gz_bridge ros_gz_sim ros_gz_image )
    # PX4 需重配置以编译 gz_bridge 模块 (依赖 gz-transport13)
    ( cd PX4-Autopilot && cmake -B build/px4_sitl_default && ninja -C build/px4_sitl_default -j"$NPROC" )
fi

( source /opt/ros/humble/setup.bash \
  && cd ros2_ws && colcon build --symlink-install --base-paths src ../third_party/XTDrone2 )

echo ""
echo "================ 复现完成 ================"
echo "启动仿真: bash scripts/run_simulation.sh"
echo "手动启动 Agent 需要: export LD_LIBRARY_PATH=\$HOME/.local/lib:\$LD_LIBRARY_PATH"
