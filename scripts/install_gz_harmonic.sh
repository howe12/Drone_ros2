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
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
echo "完成! 之后可用: cd PX4-Autopilot && make px4_sitl gz_x500"
