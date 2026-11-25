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

echo "[3/6] 应用本地适配补丁 (无 sudo 环境)..."
git -C PX4-Autopilot apply "$ROOT/patches/0001-px4-sitl-gazebo-classic-gstreamer-off.patch" 2>/dev/null || echo "  补丁1已应用, 跳过"
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
( source /opt/ros/humble/setup.bash \
  && cd ros2_ws && colcon build --symlink-install )

echo ""
echo "================ 复现完成 ================"
echo "启动仿真: bash scripts/run_simulation.sh"
echo "手动启动 Agent 需要: export LD_LIBRARY_PATH=\$HOME/.local/lib:\$LD_LIBRARY_PATH"
