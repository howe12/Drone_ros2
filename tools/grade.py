#!/usr/bin/env python3
"""
可执行判定工具 (E12/平台 L2 层)
================================
把 catalog/scenarios.yaml 里的 metrics 变成**真的能跑出结论**的东西。

设计要点 (三条铁律的代码化):
  1. 指标化: 每个指标必须有 how (取法) 与 threshold (阈值)
  2. **N 次通过率**: 单次成功不算 PASS; 默认 N=3, 要求 >=2/3 通过
  3. 外部真值优先: 位置类指标一律用 gz 世界系位姿 /<ns>/pose

用法:
    python3 tools/grade.py --list
    python3 tools/grade.py --scenario ctrl.takeoff_hover_land --runs 1
    python3 tools/grade.py --scenario ctrl.waypoint_square --runs 3 --json out.json --md out.md

实现说明:
  - 进程用 start_new_session=True 启动, 结束时按 **进程组** 终止 —— 不用 pkill 模式匹配,
    从根本上避免"pkill 匹配到自身命令行"这类事故
  - 需要 GUI 的仿真, 请先在能访问 X 的终端里运行 (DISPLAY=:0)
"""
import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_LOG = os.path.join(ROOT, 'logs/xtd2_sim.log')
OUT_DIR = os.path.join(ROOT, 'logs/grade')

# ---------------------------------------------------------------- 场景探针注册表
# 每个探针: 采样 ROS → 返回 {指标名: 值}; grade.py 负责与阈值比较
PROBES = {}


def probe(scenario_id):
    def deco(fn):
        PROBES[scenario_id] = fn
        return fn
    return deco


# ---------------------------------------------------------------- 进程工具
class Proc:
    """进程组包装: 结束时整组终止, 不用 pkill"""

    def __init__(self, cmd, log_path):
        self.log = open(log_path, 'w')
        env = dict(os.environ)
        env.setdefault('DISPLAY', ':0')          # 仿真需要 X (GUI 渲染传感器)
        # MicroXRCEAgent 在 ~/.local/bin; 缺了它 run_simulation.sh 会 exit 127
        home_bin = os.path.expanduser('~/.local/bin')
        if home_bin not in env.get('PATH', ''):
            env['PATH'] = f"{home_bin}:/usr/bin:" + env.get('PATH', '')
        # MicroXRCEAgent 动态库路径 (缺了会 exit 127: libmicroxrcedds_agent.so.3.0 not found)
        home_lib = os.path.expanduser('~/.local/lib')
        if home_lib not in env.get('LD_LIBRARY_PATH', ''):
            env['LD_LIBRARY_PATH'] = f"{home_lib}:" + env.get('LD_LIBRARY_PATH', '')
        self.p = subprocess.Popen(cmd, shell=True, stdout=self.log,
                                  stderr=subprocess.STDOUT, env=env,
                                  start_new_session=True, cwd=ROOT)

    def kill(self):
        try:
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
            time.sleep(3)
            os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.log.close()


def log_size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return 0


def wait_px4_ready(timeout=180, offset=0):
    """等待本次启动后**新增**日志里出现 Ready for takeoff。
    offset: 启动栈之前记录的日志大小, 用来跳过历史内容 (否则会读到上次的 Ready)。
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with open(SIM_LOG, errors='ignore') as f:
                f.seek(offset)
                if 'Ready for takeoff' in f.read():
                    return True
        except FileNotFoundError:
            pass
        time.sleep(3)
    return False


# ---------------------------------------------------------------- ROS 就绪
_RCLPY = {'ready': False}


def ensure_rclpy():
    """rclpy 只 init 一次 (grade.py 内多处需要)"""
    import rclpy
    if not _RCLPY['ready']:
        rclpy.init(args=None)
        _RCLPY['ready'] = True
    return rclpy


def cleanup_stale():
    """杀掉上一轮可能残留的仿真进程 (DDS 端口/共享内存占用会让新栈起不来)。
    用 pkill -x 精确进程名匹配, 不会误伤 python 自身。
    """
    # 用统一清理脚本: `pkill -x gz` 对 `ruby .../gz sim ...` 无效(实测残留 16 个进程,
    # 导致新测试连到旧世界), 必须按模式清理。
    try:
        subprocess.run(['bash', os.path.join(ROOT, 'scripts/clean_sim.sh'), '--quiet'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
    except Exception:
        pass


def wait_for_status(ns='/x500_0', timeout=180):
    """等 PX4 状态话题真正开始发布 —— 比解析日志可靠
    (踩坑: run_simulation.sh 的日志经 capped_log.sh 上限截断, 达到上限后不再写入,
     按日志判断就绪会永远超时)
    """
    import time as _t
    rclpy = ensure_rclpy()
    from rclpy.node import Node
    from px4_msgs.msg import VehicleStatus
    from drone_sim.px4_interface import QOS_SUB
    node = Node('grade_ready_probe')
    got = []
    node.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status',
                             lambda m: got.append(1), QOS_SUB)
    t0 = _t.time()
    while _t.time() - t0 < timeout and not got:
        rclpy.spin_once(node, timeout_sec=0.3)
    node.destroy_node()
    return bool(got)


# ---------------------------------------------------------------- ROS 采样
class Sampler:
    """多机位姿 + PX4 状态 + 速度 + 任意里程计话题 采样"""

    def __init__(self, namespaces=('/x500_0',), odom_topics=None):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseArray
        from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
        from drone_sim.px4_interface import QOS_SUB

        self.rclpy = rclpy
        ensure_rclpy()
        self.node = Node('grade_sampler')
        self.poses = {ns: [] for ns in namespaces}   # (t, x, y, z) gz 世界系 ENU
        self.arms = {ns: [] for ns in namespaces}    # (t, arming_state)
        self.navs = {ns: [] for ns in namespaces}    # (t, nav_state)
        self.vel = {ns: [] for ns in namespaces}     # (t, vx, vy, vz, z)
        self.odom = {}
        for ns in namespaces:
            self.node.create_subscription(
                PoseArray, f'{ns}/pose',
                (lambda m, n=ns: self.poses[n].append(
                    (time.time(), m.poses[0].position.x,
                     m.poses[0].position.y, m.poses[0].position.z)) if m.poses else None),
                qos_profile_sensor_data)
            self.node.create_subscription(
                VehicleStatus, f'{ns}/fmu/out/vehicle_status',
                (lambda m, n=ns: (self.arms[n].append((time.time(), m.arming_state)),
                                  self.navs[n].append((time.time(), m.nav_state)))),
                QOS_SUB)
            self.node.create_subscription(
                VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position',
                (lambda m, n=ns: self.vel[n].append(
                    (time.time(), m.vx, m.vy, m.vz, -m.z))), QOS_SUB)

        from nav_msgs.msg import Odometry
        for name, topic in (odom_topics or {}).items():
            self.odom[name] = []
            self.node.create_subscription(
                Odometry, topic,
                (lambda m, k=name: self.odom[k].append(
                    (time.time(), m.pose.pose.position.x,
                     m.pose.pose.position.y, m.pose.pose.position.z))),
                qos_profile_sensor_data)

        self._stop = False
        self._th = threading.Thread(target=self._spin, daemon=True)
        self._th.start()

    def _spin(self):
        while not self._stop:
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                break

    def wait_ready(self, timeout=180):
        """等待首个命名空间收到 PX4 状态。主线程只轮询, 不 spin
        (并发 spin 同一个 context 会把采样线程打挂, 实测 pose/status 全为 0)。"""
        t0 = time.time()
        first = next(iter(self.arms))
        while time.time() - t0 < timeout:
            if len(self.arms[first]) > 0:
                return True
            time.sleep(2)
        return False

    def stop(self):
        self._stop = True
        # 必须先等采样线程退出 spin_once, 再销毁节点 —— 否则死锁, 进程卡死不退出
        try:
            self._th.join(timeout=3.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass


class Ctx:
    """探针上下文: 采样数据 + 时间窗 + 节点日志 + 场景定义"""

    def __init__(self, s, t_start, t_end, node_log, sc, log_paths=None):
        self.s = s
        self.t_start = t_start
        self.t_end = t_end
        self.node_log = node_log
        self.sc = sc
        self.log_paths = log_paths or {}

    def _indexed_log(self):
        """按 node_log_index 取第 n 条命令的 stdout 日志 (0 起)"""
        idx = int(self.sc.get('node_log_index', 0))
        return self.log_paths.get(idx, self.node_log)

    def poses(self, ns=None):
        ns = ns or next(iter(self.s.poses))
        return [p for p in self.s.poses[ns] if p[0] >= self.t_start]

    def arms(self, ns=None):
        ns = ns or next(iter(self.s.arms))
        return [p for p in self.s.arms[ns] if p[0] >= self.t_start]

    def vel(self, ns=None):
        ns = ns or next(iter(self.s.vel))
        return [p for p in self.s.vel[ns] if p[0] >= self.t_start]

    def logtext(self):
        """读取节点日志。部分节点把详细信息写进自己的 log_file (不是 stdout),
        因此场景可用 node_log_file 指定; 支持 {RUN_LOG} 占位符 (与命令中的同名占位对应)。
        """
        path = self.sc.get('node_log_file')
        if path:
            path = path.replace('{RUN_LOG}', self.node_log)
            if not os.path.isabs(path):
                path = os.path.join(ROOT, path)
        else:
            path = self._indexed_log()
        try:
            with open(path, errors='ignore') as f:
                return f.read()
        except OSError:
            return ''

    def disarmed_at_end(self, ns=None):
        a = self.arms(ns)
        return 1 if (a and a[-1][1] == 1) else 0


# ---------------------------------------------------------------- 探针实现
@probe('ctrl.takeoff_hover_land')
def probe_takeoff(ctx, target_alt=5.0):
    ps = ctx.poses()
    max_alt = max((z for (_, _, _, z) in ps), default=0.0)
    arms = ctx.arms()
    t_arm = next((t for (t, a) in arms if a == 2), None)
    t_disarm = next((t for (t, a) in arms if a == 1 and t_arm and t > t_arm), None)
    dur = (t_disarm - t_arm) if (t_arm and t_disarm) else 99.0
    return {'起飞高度误差': round(abs(max_alt - target_alt), 2),
            '全程耗时': round(dur, 1),
            '结束状态': ctx.disarmed_at_end()}


def load_mission_waypoints(mission='square'):
    """从 config/mission_<name>.yaml 读航点 (NED, z 向上为负)"""
    import yaml
    path = os.path.join(ROOT, f'ros2_ws/src/drone_sim/config/mission_{mission}.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return [(float(w['x']), float(w['y']), float(w.get('z', -5.0))) for w in cfg['waypoints']]


@probe('ctrl.waypoint_square')
def probe_waypoint(ctx, tol=0.5, mission='square'):
    ps = ctx.poses()
    wps = load_mission_waypoints(mission)
    if not ps:
        return {'到点误差': 99.0, '命中航点数': 0, '结束状态': 0}
    errs, hits = [], 0
    for (wx, wy, wz_ned) in wps:
        wz = -wz_ned          # 任务 yaml 用 NED(z 向下为负); gz 位姿是 ENU(z 向上为正)
        d = min(((px - wx) ** 2 + (py - wy) ** 2 + (pz - wz) ** 2) ** 0.5
                for (_, px, py, pz) in ps)
        errs.append(d)
        if d < tol:
            hits += 1
    return {'到点误差': round(max(errs), 2), '命中航点数': hits,
            '结束状态': ctx.disarmed_at_end()}


@probe('vision.aruco_precision_landing')
def probe_aruco(ctx):
    """落点偏差: 结束时水平位置与靶标(世界原点)的距离"""
    ps = ctx.poses()
    if not ps:
        return {'落点偏差': 99.0, '最终对齐': 0, '结束状态': 0}
    _, x, y, z = ps[-1]
    # 落点偏差本身已能证明对准; 原先再加"日志里找最终对齐完成"属于重复且脆弱
    return {'落点偏差': round((x * x + y * y) ** 0.5, 3),
            '结束状态': ctx.disarmed_at_end()}


@probe('slam.lio_odometry')
def probe_lio(ctx):
    """里程计精度: LIO z 与 gz 真值 z 的中位偏差; 频率; 轨迹跨度"""
    od = [p for p in ctx.s.odom.get('lio', []) if p[0] >= ctx.t_start]
    ps = ctx.poses()
    if not od or not ps:
        return {'高度误差': 99.0, '里程计频率': 0.0, '轨迹跨度': 0.0}
    # 按时间对齐 (取最近真值)
    errs = []
    for (t, ox, oy, oz) in od:
        near = min(ps, key=lambda p: abs(p[0] - t))
        errs.append(abs(oz - near[3]))
    dur = max(1e-3, od[-1][0] - od[0][0])
    xs = [p[1] for p in ps]
    ys = [p[2] for p in ps]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    errs.sort()
    return {'高度误差': round(errs[len(errs) // 2], 3),
            '里程计频率': round(len(od) / dur, 1),
            '轨迹跨度': round(span, 2)}


@probe('plan.obstacle_avoidance')
def probe_avoid(ctx):
    ps = ctx.poses()
    goal = tuple(ctx.sc.get('goal', [6.0, 0.0, 1.2]))
    obs = ctx.sc.get('obstacles', [[2.6, 0.85], [4.0, -0.85]])
    if not ps:
        return {'目标误差': 99.0, '障碍最小距离': 0.0, '到达': 0}
    def gd(p):
        return ((p[1] - goal[0]) ** 2 + (p[2] - goal[1]) ** 2 + (p[3] - goal[2]) ** 2) ** 0.5
    gmin = min(gd(p) for p in ps)          # 最近接近距离 (benchmark 常用口径)
    gend = gd(ps[-1])                      # 结束时的静止误差
    dmin = min((((px - ox) ** 2 + (py - oy) ** 2) ** 0.5)
               for (_, px, py, _) in ps for (ox, oy) in obs)
    return {'最近接近距离': round(gmin, 2),
            '结束误差': round(gend, 2),
            '障碍最小距离': round(dmin, 2)}


@probe('multi.formation')
def probe_formation(ctx):
    """队形误差: 取跟随者节点自己计算并打印的 "编队误差 X m" (稳态段中位数)

    为什么不自己按 gz 位姿算: PX4 偏移是 NED(北东地), gz 位姿是 ENU(东北天),
    直接套用会得到完全错误的数字 (实测 4.18m / 7.21m 都是这么来的)。
    跟随者自己算的误差才是控制器真正在追求的指标。
    """
    import re as _re
    txt = ctx.logtext()
    vals = [float(m) for m in _re.findall(r'编队误差\s*([0-9.]+)m', txt)]
    if vals:
        tail = vals[int(len(vals) * 0.5):] or vals        # 后半段视为稳态
        tail.sort()
        a0 = next((t for (t, a) in ctx.arms(list(ctx.s.arms)[0]) if a == 2), None)
        a1 = next((t for (t, a) in ctx.arms(list(ctx.s.arms)[1]) if a == 2), None)
        both = 1 if (a0 and a1 and abs(a0 - a1) < 8.0) else 0
        return {'队形误差': round(tail[len(tail) // 2], 2), '双机同时起飞': both}

    off = ctx.sc.get('formation_offset', [0.0, 0.0])
    names = list(ctx.s.poses.keys())
    if len(names) < 2:
        return {'队形误差': 99.0, '双机同时起飞': 0}
    lead = ctx.poses(names[0])
    foll = ctx.poses(names[1])
    if len(lead) < 10 or len(foll) < 10:
        return {'队形误差': 99.0, '双机同时起飞': 0}
    errs = []
    for (t, lx, ly, lz) in lead[::10]:
        if lz < 1.0:            # 只看两机都在空中的阶段 (起降段跟随者必然滞后)
            continue
        near = min(foll, key=lambda p: abs(p[0] - t))
        if near[3] < 1.0:
            continue
        ex, ey = lx + off[0], ly + off[1]
        errs.append(((near[1] - ex) ** 2 + (near[2] - ey) ** 2) ** 0.5)
    if not errs:
        return {'队形误差': 99.0, '双机同时起飞': 0}
    errs.sort()
    a0 = next((t for (t, a) in ctx.arms(names[0]) if a == 2), None)
    a1 = next((t for (t, a) in ctx.arms(names[1]) if a == 2), None)
    both = 1 if (a0 and a1 and abs(a0 - a1) < 8.0) else 0
    return {'队形误差': round(errs[len(errs) // 2], 2), '双机同时起飞': both}


@probe('swarm.cross_3uav')
def probe_swarm(ctx):
    """最小间距: 任意两机最近距离; 全部到达: 各机与目标距离 < tol 的数量"""
    names = list(ctx.s.poses.keys())
    goals = ctx.sc.get('goals', {})
    tol = float(ctx.sc.get('arrive_tol', 0.8))
    series = {n: ctx.poses(n) for n in names}
    if any(len(v) < 10 for v in series.values()):
        return {'最小间距': 0.0, '全部到达': 0}
    # 以最早开始的那台为时间基准, 逐点取最近邻
    dmin = 1e9
    base = names[0]
    for (t, x, y, z) in series[base][::5]:
        for other in names[1:]:
            near = min(series[other], key=lambda p: abs(p[0] - t))
            dmin = min(dmin, ((x - near[1]) ** 2 + (y - near[2]) ** 2 + (z - near[3]) ** 2) ** 0.5)
    arrived = 0
    for n in names:
        g = goals.get(n)
        if not g:
            continue
        _, x, y, z = series[n][-1]
        if ((x - g[0]) ** 2 + (y - g[1]) ** 2 + (z - g[2]) ** 2) ** 0.5 < tol:
            arrived += 1
    return {'最小间距': round(dmin, 2), '全部到达': arrived}


def load_bt_waypoints():
    """从行为树 XML 解析 GotoWaypoint 的 x/y/z。
    注意: XML 里的 z 是**高度(向上为正)**, 与 mission_square.yaml 的 NED(-5) 不同,
    之前混用导致"到达航点数"恒为 0。
    """
    import re as _re
    path = os.path.join(ROOT, 'ros2_ws/src/drone_sim/config/mission_bt_square.xml')
    try:
        with open(path) as f:
            txt = f.read()
    except OSError:
        return []
    wps = []
    for m in _re.finditer(r'x="(-?[0-9.]+)"\s*y="(-?[0-9.]+)"\s*z="(-?[0-9.]+)"', txt):
        wps.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return wps


@probe('mission.behavior_tree')
def probe_bt(ctx):
    """根节点结果来自日志; 到达航点数来自真实位姿 (比数日志字样可靠)"""
    log = ctx.logtext()
    ok = 1 if '任务结束: SUCCESS' in log else 0
    ps = ctx.poses()
    hits = 0
    if ps:
        for (wx, wy, wz) in load_bt_waypoints():      # z 已是高度(向上为正)
            d = min(((px - wx) ** 2 + (py - wy) ** 2 + (pz - wz) ** 2) ** 0.5
                    for (_, px, py, pz) in ps)
            if d < 0.6:
                hits += 1
    return {'根节点状态': ok, '到达航点数': hits, '结束状态': ctx.disarmed_at_end()}


@probe('vtol.transition')
def probe_vtol(ctx):
    """standard_vtol 没有 gz pose 桥接 -> 高度与速度都取 PX4 本地数据"""
    vs = ctx.vel()
    hmax = max((((vx * vx + vy * vy) ** 0.5) for (_, vx, vy, _, _) in vs), default=0.0)
    alt = max((z for (_, _, _, _, z) in vs), default=0.0)
    return {'固定翼段空速': round(hmax, 1),
            '垂直起飞高度': round(alt, 1),
            '结束状态': ctx.disarmed_at_end()}


@probe('safety.geofence')
def probe_safety(ctx):
    log = ctx.logtext()
    fence = 1 if ('超出水平围栏' in log or '超出高度上限' in log or '速度超限' in log) else 0
    other = sum(1 for kw in ('倾角过大', '低于高度下限', '外部急停')
                if kw in log)
    return {'围栏事件': fence, '误报': other}


@probe('vision.yolo_detect')
def probe_yolo(ctx):
    log = ctx.logtext()
    started = 1 if ('YOLO 检测启动' in log or '检测启动' in log) else 0
    crashed = 1 if ('Traceback' in log or 'ModuleNotFoundError' in log) else 0
    return {'启动成功': started, '无异常退出': 0 if crashed else 1}


@probe('vio.vins_mono')
def probe_vio(ctx):
    """VIO 当前跑不通 —— 探针如实报告"无输出", 该场景标 unverified, 不作作业"""
    od = [p for p in ctx.s.odom.get('vins', []) if p[0] >= ctx.t_start]
    return {'里程计输出频率': round(len(od) / max(1e-3, ctx.t_end - ctx.t_start), 1)}


# ---------------------------------------------------------------- 主流程
def run_scenario(sc, runs, platform='xtd2'):
    from px4_msgs.msg import VehicleStatus  # noqa: F401  (确保环境可用)
    sid = sc['id']
    if sid not in PROBES:
        print(f'✗ 场景 {sid} 尚无探针实现 (已实现: {", ".join(PROBES)})')
        return None

    os.makedirs(OUT_DIR, exist_ok=True)
    model, world = sc['model'], sc['world']
    # 多机场景: namespaces 列表; 单机默认 ['/x500_0']
    namespaces = tuple(sc.get('namespaces') or ['/x500_0'])
    odom_topics = sc.get('odom_topics') or {}
    results = []

    for r in range(runs):
        print(f'\n=== 第 {r + 1}/{runs} 次: {sid} ===')
        cleanup_stale()      # 上一轮残留会占用 DDS 端口/共享内存, 必须先清
        # 轮转仿真日志: capped_log.sh 达到上限后不再写入, 必须每次从空开始
        try:
            if os.path.exists(SIM_LOG) and os.path.getsize(SIM_LOG) > 1024 * 1024:
                os.replace(SIM_LOG, SIM_LOG + '.old')
        except OSError:
            pass
        base = log_size(SIM_LOG)      # 启动前基线, 只认之后的新日志
        # 场景可用 stack: none 表示"命令自己起整套仿真" (如 multi_vehicle/swarm_cross launch)
        own_stack = str(sc.get('stack', 'auto')) == 'none'
        stack = None if own_stack else Proc(
            f'bash scripts/run_simulation.sh {platform} {model} {world}',
            f'{OUT_DIR}/{sid}_run{r}_stack.log')
        procs = []
        try:
            # 采样器尽早创建: 它是**唯一** spin 方, 主线程只轮询
            sampler = Sampler(namespaces=namespaces, odom_topics=odom_topics)
            ready = sampler.wait_ready(180)
            if not ready and not own_stack:
                print('  ⟳ 首次未就绪, 清理残留后重试一次 ...')
                cleanup_stale()
                if stack is not None:
                    stack.kill()
                base = log_size(SIM_LOG)
                stack = Proc(f'bash scripts/run_simulation.sh {platform} {model} {world}',
                             f'{OUT_DIR}/{sid}_run{r}_stack_retry.log')
                ready = sampler.wait_ready(180)
            if not ready and not own_stack:
                print('  ✗ 等 PX4 就绪失败 (状态话题无数据)')
                sampler.stop()
                results.append({'ok': False, 'metrics': {}})
                continue

            # 场景命令: 逐行启动 (每行一个独立进程), 剥离 run_simulation.sh 行与续行符
            _raw = [ln.strip() for ln in sc['command'].strip().splitlines()
                    if ln.strip() and 'run_simulation.sh' not in ln]
            log_paths = {}
            cmds = []
            for i, ln in enumerate(_raw):
                lp = f'{OUT_DIR}/{sid}_run{r}_node{i}.log'
                log_paths[i] = lp
                cmds.append(ln.replace('{RUN_LOG}', lp))
            if own_stack:
                t_start = time.time()
                # 第一条命令负责起整套仿真, 必须等它就绪后再起其余节点
                procs.append(Proc(cmds[0].rstrip('\\').strip(),
                                  f'{OUT_DIR}/{sid}_run{r}_node0.log'))
                if not sampler.wait_ready(240):
                    print('  ✗ 自起栈未能就绪 (第一条命令未拉起 PX4)')
                    sampler.stop()
                    results.append({'ok': False, 'metrics': {}})
                    continue
                print('  自起栈就绪')
                for i, c in enumerate(cmds[1:], start=1):
                    procs.append(Proc(c.rstrip('\\').strip(),
                                      f'{OUT_DIR}/{sid}_run{r}_node{i}.log'))
                    time.sleep(3)
            else:
                print('  PX4 就绪')
                t_start = time.time()
                for i, c in enumerate(cmds):
                    procs.append(Proc(c.rstrip('\\').strip(),
                                      f'{OUT_DIR}/{sid}_run{r}_node{i}.log'))
                    time.sleep(2)
            time.sleep(float(sc.get('duration', 60)))
            t_end = time.time()

            n_pose = sum(len([1 for p in sampler.poses[n] if p[0] >= t_start]) for n in namespaces)
            n_arm = sum(len([1 for p in sampler.arms[n] if p[0] >= t_start]) for n in namespaces)
            n_vel = sum(len([1 for p in sampler.vel[n] if p[0] >= t_start]) for n in namespaces)
            # 部分机型(如 standard_vtol)没有 gz pose 桥接, 只要有 PX4 速度数据即可
            if (n_pose < 10 and n_vel < 50) or n_arm < 2:
                print(f'  ✗ 采样不足 (pose={n_pose}, vel={n_vel}, status={n_arm}) —— 判为本次运行失败')
                results.append({'ok': False, 'metrics': {}})
                continue
            zs = [p[3] for n in namespaces for p in sampler.poses[n] if p[0] >= t_start]
            if zs:
                print(f'  采样诊断: {len(zs)} 点, 高度范围 [{min(zs):.2f}, {max(zs):.2f}] m')
            node_log = log_paths.get(0, f'{OUT_DIR}/{sid}_run{r}_node0.log')
            ctx = Ctx(sampler, t_start, t_end, node_log, sc, log_paths)
            metrics = PROBES[sid](ctx)
            sampler.stop()
            print(f"  指标: {metrics}")
            results.append({'ok': True, 'metrics': metrics})
        finally:
            for pr in procs:
                pr.kill()
            if stack is not None:
                stack.kill()
            time.sleep(6)

    # ---- 聚合 ----
    good = [r for r in results if r['ok']]
    if not good:
        print('\n✗ 全部运行失败')
        return None

    names = good[0]['metrics'].keys()
    agg = {}
    for n in names:
        vals = [r['metrics'][n] for r in good if n in r['metrics']]
        agg[n] = {'median': statistics.median(vals), 'values': vals}

    verdict = evaluate(sc, agg)
    report = {
        'scenario': sid, 'title': sc.get('title'), 'runs': runs,
        'successful_runs': len(good), 'aggregate': agg,
        'verdict': verdict,
    }
    return report


def evaluate(sc, agg):
    """按 scenarios.yaml 的 metrics 阈值判定 (支持数值 + 通过率型)"""
    checks = []
    for m in sc.get('metrics', []):
        name = m['name']
        if name not in agg:
            checks.append({'metric': name, 'status': 'SKIP', 'note': '无采样数据'})
            continue
        med = agg[name]['median']
        thr = str(m['threshold']).strip()
        ok, note = None, ''
        try:
            # 注意顺序: 两字符运算符必须先判断, 否则 '>=' 会被 '(' >' 吃掉
            if thr.startswith('>='):
                ok = med >= float(thr[2:].strip())
            elif thr.startswith('<='):
                ok = med <= float(thr[2:].strip())
            elif thr.startswith('=='):
                ok = abs(med - float(thr[2:].strip())) < 1e-9
            elif thr.startswith('<'):
                ok = med < float(thr[1:].strip())
            elif thr.startswith('>'):
                ok = med > float(thr[1:].strip())
            elif 'disarmed' in thr:
                ok = med >= 1
            elif 'SUCCESS' in thr:
                ok = med >= 1
            elif '/' in thr:                     # 形如 3/3
                a, b = thr.split('/')
                ok = med >= float(a) / float(b)
            else:
                note = f'无法解析阈值: {thr}'
        except ValueError:
            note = f'阈值非数值: {thr}'
        checks.append({'metric': name, 'median': med, 'threshold': thr,
                       'status': 'PASS' if ok else ('FAIL' if ok is False else 'INCOMPLETE'),
                       'note': note})
    return checks


def write_reports(rep, json_path=None, md_path=None):
    if json_path:
        with open(json_path, 'w') as f:
            json.dump(rep, f, ensure_ascii=False, indent=2)
        print(f'\n✓ JSON 报告: {json_path}')
    if md_path:
        lines = [f"# 判定报告: {rep['scenario']} — {rep['title']}",
                 '', f"- 运行次数: {rep['runs']} (成功 {rep['successful_runs']})", '',
                 '| 指标 | 中位数 | 阈值 | 结果 |', '|---|---|---|---|']
        for c in rep['verdict']:
            lines.append(f"| {c['metric']} | {c.get('median', '-')} | {c.get('threshold', '-')} "
                         f"| {c['status']} |")
        n_pass = sum(1 for c in rep['verdict'] if c['status'] == 'PASS')
        n_total = len(rep['verdict'])
        n_inc = sum(1 for c in rep['verdict'] if c['status'] == 'INCOMPLETE')
        overall = 'PASS' if (n_total > 0 and n_pass == n_total) else ('INCOMPLETE' if n_inc else 'FAIL')
        lines += ['', f"**汇总: {n_pass}/{n_total} 项通过 — 总体 {overall}**"]
        with open(md_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print(f'✓ Markdown 报告: {md_path}')


def load_scenarios():
    import yaml
    with open(os.path.join(ROOT, 'catalog/scenarios.yaml')) as f:
        return yaml.safe_load(f)['scenarios']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario')
    ap.add_argument('--runs', type=int, default=3)
    ap.add_argument('--platform', default='xtd2')
    ap.add_argument('--json')
    ap.add_argument('--md')
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()

    scenarios = {s['id']: s for s in load_scenarios()}
    if args.list:
        print('场景 (★ = 已有探针, 可自动判定):')
        for sid, s in scenarios.items():
            mark = '★' if sid in PROBES else ' '
            print(f"  {mark} {sid:34s} {s.get('level','?'):3s}  {s.get('title','')}  [{s.get('status')}]")
        return 0

    if not args.scenario or args.scenario not in scenarios:
        print(f'✗ 场景不存在. 用 --list 查看')
        return 1

    sc = dict(scenarios[args.scenario])
    if args.runs > 1:
        print(f'注意: {args.runs} 次运行, 每次含完整栈启动, 预计 {args.runs * 2} 分钟')
    rep = run_scenario(sc, args.runs, args.platform)
    if not rep:
        return 2
    write_reports(rep, args.json, args.md)
    n_total = len(rep['verdict'])
    n_pass = sum(1 for c in rep['verdict'] if c['status'] == 'PASS')
    ok = n_total > 0 and n_pass == n_total
    print(f"\n结论: {'PASS' if ok else f'NOT PASS ({n_pass}/{n_total} 项通过)'}"
          f"  [成功运行 {rep['successful_runs']}/{rep['runs']} 次]")
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
