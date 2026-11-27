#!/usr/bin/env python3
"""
从源码自动生成参数注册表 catalog/params.yaml
=============================================
课程作者需要知道"有哪些可调参数、默认值、属于哪个节点、怎么用"。
手写会失真, 所以从 declare_parameter 调用直接抽取。

用法:
    python3 tools/gen_params.py            # 生成 catalog/params.yaml
    python3 tools/gen_params.py --check    # 只校验是否与实际代码一致(CI 用)

输出格式:
    nodes:
      planner_reactive:
        params:
          max_speed: {type: float, default: 1.2, file: ...}
"""
import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, 'ros2_ws/src/drone_sim/drone_sim')
OUT = os.path.join(ROOT, 'catalog/params.yaml')

# declare_parameter('name', default) / declare_parameter('name', default)  (无类型声明)
PATTERN = re.compile(r"declare_parameter\(\s*['\"]([^'\"]+)['\"]\s*(?:,\s*([^)]+))?\)")


def infer_type(default_literal):
    if default_literal is None:
        return 'unknown'
    v = default_literal.strip()
    if v.startswith(('True', 'False')):
        return 'bool'
    if v.startswith(("'", '"')):
        return 'str'
    if re.fullmatch(r'-?\d+', v):
        return 'int'
    if re.fullmatch(r'-?\d*\.\d+(e-?\d+)?', v, re.I):
        return 'float'
    return 'expr'


def collect():
    result = {}
    for fn in sorted(os.listdir(PKG)):
        if not fn.endswith('.py') or fn in ('__init__.py', 'px4_interface.py'):
            continue
        path = os.path.join(PKG, fn)
        node = fn[:-3]
        with open(path) as f:
            src = f.read()
        params = {}
        for m in PATTERN.finditer(src):
            name, default = m.group(1), m.group(2)
            params[name] = {
                'type': infer_type(default),
                'default': default.strip() if default else None,
                'file': f'ros2_ws/src/drone_sim/drone_sim/{fn}',
            }
        if params:
            result[node] = {'params': params}
    return result


def to_yaml(data):
    lines = ['# 自动生成, 请勿手改 —— 运行 python3 tools/gen_params.py 重新生成',
             '# 用途: 课程作者据此生成"调参练习", 无需读源码',
             'nodes:']
    for node in sorted(data):
        lines.append(f'  {node}:')
        lines.append('    params:')
        for name in sorted(data[node]['params']):
            p = data[node]['params'][name]
            default = p['default'] if p['default'] is not None else 'null'
            lines.append(f"      {name}: {{type: {p['type']}, default: {default}}}")
    lines.append('')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='仅校验, 不写文件')
    args = ap.parse_args()

    data = collect()
    text = to_yaml(data)
    total = sum(len(v['params']) for v in data.values())

    if args.check:
        if not os.path.exists(OUT):
            print(f'✗ 缺少 {OUT}')
            return 1
        old = open(OUT).read()
        if old != text:
            print('✗ catalog/params.yaml 与代码不一致, 请重新生成')
            return 1
        print(f'✓ 一致 ({len(data)} 节点, {total} 参数)')
        return 0

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        f.write(text)
    print(f'✓ 写入 {OUT}: {len(data)} 节点, {total} 参数')
    for node in sorted(data):
        print(f"  - {node}: {', '.join(sorted(data[node]['params']))}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
