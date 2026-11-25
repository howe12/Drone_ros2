#!/usr/bin/env bash
# [可选升级] 安装 Gazebo Harmonic —— PX4 v1.15+ 官方推荐仿真器
# 本机已装 Ignition Fortress + Gazebo Classic 11, 当前演示用 Classic 即可运行。
# 若想用官方推荐的 `make px4_sitl gz_x500` (新 Gazebo), 需要 sudo 执行本脚本:
#   bash scripts/install_gz_harmonic.sh
set -e
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

# 防文件冲突: classic gazebo 与 gz-tools2(Harmonic) 都提供 /usr/bin/gz。
# 先把 classic 的 gz 包装器改名为 gz-classic (不影响 gzserver/gzclient/gazebo,
# 本项目 classic 仿真直接用 gzserver/gzclient, 不受影响)。
# 恢复方法: sudo dpkg-divert --remove --rename /usr/bin/gz
if dpkg -S /usr/bin/gz 2>/dev/null | grep -q "^gazebo:"; then
    echo "检测到 /usr/bin/gz 属于 classic gazebo, 先做 diversion 避免冲突..."
    sudo dpkg-divert --add --rename --divert /usr/bin/gz-classic /usr/bin/gz
fi

sudo apt-get install -y gz-harmonic

# 注意: diversion 对一切包生效, gz-tools2 的 /usr/bin/gz 也会被改道到 /usr/bin/gz-classic
# (即 Harmonic 的 gz)。建用户级符号链接恢复 `gz` 命令, 无需 sudo:
mkdir -p "$HOME/.local/bin"
ln -sf /usr/bin/gz-classic "$HOME/.local/bin/gz"
echo "完成! 验证: gz sim --version (应显示 Gazebo Sim 8.x)"
echo "启动 XTDrone2 平台: bash scripts/run_simulation.sh xtd2"
echo "或 PX4 官方 x500:   cd PX4-Autopilot && make px4_sitl gz_x500"
echo "[可选清理] 若想恢复系统级 /usr/bin/gz: sudo dpkg-divert --remove --rename /usr/bin/gz"
