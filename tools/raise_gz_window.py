#!/usr/bin/env python3
"""
把 Gazebo 仿真窗口置顶并返回其 window id
==========================================
为什么需要: 用 ffmpeg 录全屏时, 若浏览器在其他窗口前面, 录到的是浏览器而不是仿真画面
(踩过: 生成的 GIF 里全是 GitHub 页面)。
X11 对"被遮挡窗口"做 XGetImage 会报 BadMatch, 所以必须先 raise 到最前。

用法:
    python3 tools/raise_gz_window.py            # 找 Gazebo 窗口并置顶, 打印 id
    python3 tools/raise_gz_window.py --wait 120 # 最多等 120s 直到窗口出现
"""
import argparse
import os
import sys
import time

os.environ.setdefault('DISPLAY', ':0')

try:
    from Xlib import X, display
    from Xlib.error import XError
except ImportError:
    print('缺少 python-xlib: /usr/bin/python3 -m pip install --user python-xlib', file=sys.stderr)
    sys.exit(2)

# Gazebo GUI 窗口标题包含这些关键词之一
KEYWORDS = ('Gazebo Sim', 'Gazebo', 'gz sim', 'sdformat')


def walk(win, depth=0):
    """遍历窗口树, 返回 (wid, name, geometry) 列表"""
    out = []
    try:
        name = win.get_wm_name() or ''
        if name:
            g = win.get_geometry()
            out.append((win.id, name, g.width, g.height))
        children = win.query_tree().children
        for c in children:
            out.extend(walk(c, depth + 1))
    except (XError, Exception):
        pass
    return out


def find_gz(d):
    for wid, name, w, h in walk(d.screen().root):
        if any(k.lower() in name.lower() for k in KEYWORDS):
            # 跳过太小的辅助窗口 (菜单/提示条)
            if w > 400 and h > 300:
                return wid, name, w, h
    return None, None, 0, 0


def raise_window(d, wid):
    """置顶 + 聚焦 + 去掉最小化"""
    try:
        win = d.create_resource_object('window', wid)
        win.configure(stack_mode=X.Above)
        win.map()
        win.set_input_focus(X.RevertToParent, X.CurrentTime)
        d.sync()
        return True
    except XError as e:
        print(f'置顶失败: {e}', file=sys.stderr)
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wait', type=int, default=0, help='等待窗口出现的秒数')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    d = display.Display()
    deadline = time.time() + args.wait
    while True:
        wid, name, w, h = find_gz(d)
        if wid:
            break
        if time.time() >= deadline:
            print('未找到 Gazebo 窗口', file=sys.stderr)
            return 1
        time.sleep(1)

    # 反复置顶几次: 仿真启动时会有新窗口弹出, 抢焦点
    for _ in range(3):
        raise_window(d, wid)
        time.sleep(0.3)
    wid, name, w, h = find_gz(d) or (wid, name, w, h)

    if not args.quiet:
        print(f'{wid} {w}x{h}')
        print(f'窗口标题: {name[:80]}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
