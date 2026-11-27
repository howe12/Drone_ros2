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
        env.setdefault('PATH', '/usr/bin:' + env.get('PATH', ''))
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
    """后台线程采样 gz 真值位姿与 PX4 状态"""

    def __init__(self, ns='/x500_0'):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseArray
        from px4_msgs.msg import VehicleStatus

        self.rclpy = rclpy
        ensure_rclpy()
        self.node = Node('grade_sampler')
        self.poses = []          # (t, x, y, z)
        self.arm_states = []     # (t, arming_state)
        self.statuses = []       # (t, nav_state)
        self.node.create_subscription(
            PoseArray, f'{ns}/pose',
            lambda m: self.poses.append((time.time(), m.poses[0].position.x,
                                         m.poses[0].position.y, m.poses[0].position.z))
            if m.poses else None, qos_profile_sensor_data)
        from drone_sim.px4_interface import QOS_SUB
        self.node.create_subscription(
            VehicleStatus, f'{ns}/fmu/out/vehicle_status',
            lambda m: (self.arm_states.append((time.time(), m.arming_state)),
                       self.statuses.append((time.time(), m.nav_state))), QOS_SUB)
        self._stop = False
        self._th = threading.Thread(target=self._spin, daemon=True)
        self._th.start()

    def _spin(self):
        while not self._stop:
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                break

    def stop(self):
        self._stop = True
        time.sleep(0.3)
        try:
            self.node.destroy_node()
        except Exception:
            pass


# ---------------------------------------------------------------- 探针实现
@probe('ctrl.takeoff_hover_land')
def probe_takeoff(s, t_start, t_end, target_alt=5.0):
    """起飞-悬停-降落: 高度误差 / 是否上锁结束"""
    alts = [z for (t, x, y, z) in s.poses if t >= t_start]   # gz ENU: z 向上为正
    max_alt = max(alts) if alts else 0.0
    armed = [(t, a) for (t, a) in s.arm_states if t >= t_start]
    ended_disarmed = bool(armed) and armed[-1][1] == 1
    t_arm = next((t for (t, a) in armed if a == 2), None)
    t_disarm = next((t for (t, a) in armed if a == 1 and t_arm and t > t_arm), None)
    dur = (t_disarm - t_arm) if (t_arm and t_disarm) else 99.0
    return {
        '起飞高度误差': round(abs(max_alt - target_alt), 2),
        '全程耗时': round(dur, 1),
        '结束状态': 1 if ended_disarmed else 0,
    }


def load_mission_waypoints(mission='square'):
    """从 config/mission_<name>.yaml 读航点 (NED, z 向上为负)"""
    import yaml
    path = os.path.join(ROOT, f'ros2_ws/src/drone_sim/config/mission_{mission}.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return [(float(w['x']), float(w['y']), float(w.get('z', -5.0))) for w in cfg['waypoints']]


@probe('ctrl.waypoint_square')
def probe_waypoint(s, t_start, t_end, tol=0.5, mission='square'):
    """方形航点: 每个航点的最小接近距离 -> 到点误差; 命中数 -> 航点数"""
    pts = [(x, y, z) for (t, x, y, z) in s.poses if t >= t_start]
    wps = load_mission_waypoints(mission)
    if not pts:
        return {'到点误差': 99.0, '命中航点数': 0, '结束状态': 0}
    errs, hits = [], 0
    for (wx, wy, wz) in wps:
        d = min(((px - wx) ** 2 + (py - wy) ** 2 + (pz - wz) ** 2) ** 0.5 for (px, py, pz) in pts)
        errs.append(d)
        if d < tol:
            hits += 1
    armed = [(t, a) for (t, a) in s.arm_states if t >= t_start]
    ended_disarmed = bool(armed) and armed[-1][1] == 1
    return {
        '到点误差': round(max(errs), 2),
        '命中航点数': hits,
        '结束状态': 1 if ended_disarmed else 0,
    }


# ---------------------------------------------------------------- 主流程
def run_scenario(sc, runs, platform='xtd2'):
    from px4_msgs.msg import VehicleStatus  # noqa: F401  (确保环境可用)
    sid = sc['id']
    if sid not in PROBES:
        print(f'✗ 场景 {sid} 尚无探针实现 (已实现: {", ".join(PROBES)})')
        return None

    os.makedirs(OUT_DIR, exist_ok=True)
    model, world = sc['model'], sc['world']
    results = []

    for r in range(runs):
        print(f'\n=== 第 {r + 1}/{runs} 次: {sid} ===')
        # 轮转仿真日志: capped_log.sh 达到上限后不再写入, 必须每次从空开始
        try:
            if os.path.exists(SIM_LOG) and os.path.getsize(SIM_LOG) > 1024 * 1024:
                os.replace(SIM_LOG, SIM_LOG + '.old')
        except OSError:
            pass
        base = log_size(SIM_LOG)      # 启动前基线, 只认之后的新日志
        stack = Proc(f'bash scripts/run_simulation.sh {platform} {model} {world}',
                     f'{OUT_DIR}/{sid}_run{r}_stack.log')
        try:
            ready = wait_for_status('/x500_0', timeout=180) or wait_px4_ready(offset=base, timeout=30)
            if not ready:
                print('  ✗ 等 PX4 就绪失败 (状态话题无数据且日志无 Ready)')
                results.append({'ok': False, 'metrics': {}})
                continue
            print('  PX4 就绪')
            sampler = Sampler('/x500_0')
            time.sleep(2)

            # 场景命令 = yaml 里 command 的第二行 (第一行是栈启动)
            _lines = [ln.strip() for ln in sc['command'].strip().splitlines()
                      if ln.strip() and 'run_simulation.sh' not in ln]
            # 处理 yaml 块里的换行续行符, 否则 '\\' 会被当成参数传给 ros2
            node_cmd = ' '.join(l.rstrip('\\').strip() for l in _lines)
            t_start = time.time()
            node = Proc(node_cmd, f'{OUT_DIR}/{sid}_run{r}_node.log')
            time.sleep(float(sc.get('duration', 60)))
            t_end = time.time()

            n_pose = len([1 for (t, *_) in sampler.poses if t >= t_start])
            n_arm = len([1 for (t, _) in sampler.arm_states if t >= t_start])
            if n_pose < 10 or n_arm < 2:
                print(f'  ✗ 采样不足 (pose={n_pose}, status={n_arm}) —— 判为本次运行失败')
                sampler.stop(); node.kill()
                results.append({'ok': False, 'metrics': {}})
                continue
            zs = [z for (t, x, y, z) in sampler.poses if t >= t_start]
            if zs:
                print(f'  采样诊断: {len(zs)} 点, 高度范围 [{min(zs):.2f}, {max(zs):.2f}] m')
            metrics = PROBES[sid](sampler, t_start, t_end)
            sampler.stop()
            node.kill()
            print(f"  指标: {metrics}")
            results.append({'ok': True, 'metrics': metrics})
        finally:
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
            if thr.startswith('<'):
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
