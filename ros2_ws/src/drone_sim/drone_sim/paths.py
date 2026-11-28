#!/usr/bin/env python3
"""
可移植路径工具
==============
**为什么需要**: 早期 6 个节点把日志路径硬编码成开发机的绝对路径
(`/develop/IDEA/Drone_ros2/logs/...`)。别人把仓库克隆到 `~/Drone_ros2` 后,
这些节点会尝试创建 `/develop/...` 而失败(权限/不存在) —— 属于真实的可移植性 bug。

本模块按以下优先级确定日志目录:
  1. 环境变量 `DRONE_SIM_LOG_DIR`（显式指定, 最高优先级）
  2. 从本文件位置向上找仓库根（同时含 `ros2_ws/` 与 `scripts/` 的目录）
     —— colcon 无论把模块放在 src/ 还是 build/ 下, 都仍在仓库内, 因此可靠
  3. 当前工作目录（若它看起来像仓库根）
  4. 兜底 `~/.drone_sim/logs`（绝不写到需要特权的绝对路径）

用法:
    from drone_sim.paths import log_path
    self.declare_parameter('log_file', log_path('e10_bt.log'))
"""
import os


def _looks_like_root(path):
    return (os.path.isdir(os.path.join(path, 'ros2_ws'))
            and os.path.isdir(os.path.join(path, 'scripts')))


def repo_root():
    """返回仓库根目录; 找不到时返回 None"""
    env = os.environ.get('DRONE_SIM_ROOT')
    if env and _looks_like_root(env):
        return os.path.abspath(env)

    here = os.path.dirname(os.path.abspath(__file__))
    cur = here
    for _ in range(8):                      # 最多向上 8 层, 防御性上限
        if _looks_like_root(cur):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent

    cwd = os.getcwd()
    if _looks_like_root(cwd):
        return cwd
    return None


def log_dir():
    """日志目录(自动创建)。优先 env, 其次仓库内 logs/, 最后用户目录兜底。"""
    env = os.environ.get('DRONE_SIM_LOG_DIR')
    if env:
        d = os.path.abspath(os.path.expanduser(env))
    else:
        root = repo_root()
        d = os.path.join(root, 'logs') if root else os.path.expanduser('~/.drone_sim/logs')
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        d = os.path.expanduser('~/.drone_sim/logs')
        os.makedirs(d, exist_ok=True)
    return d


def log_path(name):
    """日志文件完整路径。name 只应是文件名(不含目录)。"""
    return os.path.join(log_dir(), os.path.basename(name))


def px4_dir():
    """本仓库内的 PX4 目录(供 launch 默认值使用); 找不到则回退 ~/PX4-Autopilot"""
    env = os.environ.get('PX4_DIR')
    if env and os.path.isdir(env):
        return os.path.abspath(os.path.expanduser(env))
    root = repo_root()
    if root:
        cand = os.path.join(root, 'PX4-Autopilot')
        if os.path.isdir(cand):
            return cand
    return os.path.expanduser('~/PX4-Autopilot')
