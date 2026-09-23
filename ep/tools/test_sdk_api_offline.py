#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线核对: rm.py 调的每个 SDK 接口, 在**仓库自带**的 robomaster 副本里真的存在。

为什么要有这个: 2026-09-13 拿到真机日志后才发现 rm.py 原先调的两处
  chassis.get_position() / arm.get_position()
在官方 0.1.1.68 里**根本不存在**(位置只能 sub_position 订阅)。本机没装 SDK、也连不上机器人,
这个错一路躲过了 selfcheck / env_check --offline / 冒烟测试 —— 因为它们都在查 config 和
我们自己的假 SDK, 没有一个去问真 SDK "你到底有什么"。现在把这句话变成一条离线断言。

不连机器人、不装任何东西(用 deploy/ep_sdk_offline/packages, 缺 netifaces 时用包里的纯 py 兜底):
    python3 ep/tools/test_sdk_api_offline.py [SDK 目录]
SDK 目录默认 deploy/ep_sdk_offline/packages; 也可以在 EP 现场指向真机上装的 robomaster 父目录。
"""
import ast
import inspect
import json
import os
import struct
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_SDK = os.path.join(REPO, 'deploy', 'ep_sdk_offline', 'packages')
sdk_dir = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SDK

if not os.path.isdir(os.path.join(sdk_dir, 'robomaster')):
    print('RESULT: FAIL —— 这里没有 robomaster/: {}'.format(sdk_dir))
    sys.exit(1)
sys.path.insert(0, sdk_dir)
# netaddr 是官方 SDK 的硬依赖(import robot 就要), 仓库离线包里带了纯 py 轮子, 直接挂到
# sys.path 上就能用 —— zipimport 认 wheel(它本身就是个 zip), 不用安装。
for whl in os.listdir(sdk_dir):
    if whl.startswith('netaddr') and whl.endswith('.whl'):
        sys.path.insert(0, os.path.join(sdk_dir, whl))

# netifaces 是官方 SDK 的硬依赖, 但仓库离线包里留了纯 py 兜底(只在真连机时才用得上)
try:
    import netifaces  # noqa: F401
except ImportError:
    # 兜底文件可能不在 REPO 下(比如只解压了替换包的那几个文件, 没带 deploy/),
    # 所以 SDK 目录本身和它的父目录也找一遍; 都找不到就给一句话, 别甩 traceback。
    import importlib.util
    stub = None
    for cand in (os.path.join(REPO, 'deploy', 'ep_sdk_offline', 'packages', 'netifaces_stub.py'),
                 os.path.join(sdk_dir, 'netifaces_stub.py'),
                 os.path.join(os.path.dirname(sdk_dir), 'netifaces_stub.py'),
                 os.path.join(sdk_dir, 'netifaces.py')):
        if os.path.isfile(cand):
            stub = cand
            break
    if stub is None:
        print('RESULT: FAIL —— 本机没有 netifaces, 也没找到兜底 netifaces_stub.py')
        print('  在**完整**的真机包里跑本测试(那里有 deploy/ep_sdk_offline/packages/),')
        print('  或先 `pip install netifaces`, 再重跑。')
        sys.exit(1)
    spec = importlib.util.spec_from_file_location('netifaces', stub)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sys.modules['netifaces'] = mod
    print('   注: 本机无 netifaces, 用纯 py 兜底代替: {}'.format(stub))

from robomaster import version                                    # noqa: E402
from robomaster.action import Action                              # noqa: E402
from robomaster.chassis import Chassis                            # noqa: E402
from robomaster import conn as conn_mod                           # noqa: E402
from robomaster.gripper import Gripper                            # noqa: E402
from robomaster.robotic_arm import ArmSubject, RoboticArm         # noqa: E402
from robomaster import robot as robot_mod                          # noqa: E402

sys.path.insert(0, REPO)
from ep.drive import rm as rm_mod                                 # noqa: E402

fails = []
print('SDK 目录: {}'.format(sdk_dir))
print('SDK 版本: {}'.format(version.__version__))
if version.__version__ != '0.1.1.68':
    print('   注: 不是我们核对过的 0.1.1.68 —— 下面的接口检查仍按同一套期望跑')

print('\n== 1 rm.py 直接调的接口是否存在 ==')
# (类, 名字, 种类) —— 与 ep/drive/rm.py 的调用一一对应。
# 注意 has_succeeded / state 是 **property**(取属性, 不是调方法), 所以判据要分开: 对 property
# 用 callable() 会得到 False, 把"接口在"误报成"接口缺"。
WANT = [
    (Chassis, 'move', 'call'), (Chassis, 'sub_position', 'call'),
    (Chassis, 'unsub_position', 'call'),
    (RoboticArm, 'moveto', 'call'), (RoboticArm, 'move', 'call'),
    (RoboticArm, 'recenter', 'call'), (RoboticArm, 'sub_position', 'call'),
    (RoboticArm, 'unsub_position', 'call'),
    (Gripper, 'open', 'call'), (Gripper, 'close', 'call'),
    (Action, 'wait_for_completed', 'call'),
    (Action, 'has_succeeded', 'attr'), (Action, 'state', 'attr'),
]
for cls, name, kind in WANT:
    got = getattr(cls, name, None)
    ok = callable(got) if kind == 'call' else hasattr(cls, name)
    print('   {} {}.{} ({})'.format('✓' if ok else '✗', cls.__name__, name,
                                    '方法' if kind == 'call' else '属性'))
    if not ok:
        fails.append('缺接口 {}.{}'.format(cls.__name__, name))

print('\n== 2 rm.py 传的关键字参数签名对得上 ==')
# ({类.方法: 我们传的 kwarg}, ...) —— 形参名对不上会 TypeError(只在真机上才炸)
SIGS = [
    (Chassis.move, ('x', 'y', 'z', 'xy_speed', 'z_speed')),
    (Chassis.sub_position, ('cs', 'freq', 'callback')),
    (RoboticArm.moveto, ('x', 'y')),
    (RoboticArm.sub_position, ('freq', 'callback')),
    (Gripper.open, ('power',)), (Gripper.close, ('power',)),
    (Action.wait_for_completed, ('timeout',)),
]
for fn, kwargs in SIGS:
    params = inspect.signature(fn).parameters
    missing = [k for k in kwargs if k not in params]
    print('   {} {}({})'.format('✓' if not missing else '✗', fn.__qualname__,
                                ', '.join(kwargs)))
    if missing:
        fails.append('{}.{} 不吃参数 {}'.format(fn.__qualname__, '', missing))

print('\n== 3 Robot 的属性名 ==')
# rm._resolve_modules 按 candidates 逐个 hasattr; env_check 首跑也会打印实际命中的那个
cands = ['robotic_arm', 'arm']
hit = [c for c in cands if isinstance(getattr(robot_mod.Robot, c, None), property)
       or hasattr(robot_mod.Robot, c)]
print('   {} 机械臂属性候选 {} -> 命中 {}'.format('✓' if hit else '✗', cands, hit or '无'))
if not hit:
    fails.append('Robot 上找不到机械臂属性(工程形态怎么暴露臂的?)')
for name in ('chassis', 'gripper'):
    ok = hasattr(robot_mod.Robot, name)
    print('   {} Robot.{}'.format('✓' if ok else '✗', name))
    if not ok:
        fails.append('Robot 缺属性 ' + name)

print('\n== 4 反证: 位置没有同步 getter(所以 rm.py 必须走订阅) ==')
for cls in (Chassis, RoboticArm):
    has = hasattr(cls, 'get_position')
    print('   {} {} 有 get_position -> {}'.format('·' if not has else '✗', cls.__name__, has))
    if has:
        fails.append('{} 竟然有 get_position —— 请重核 rm.py 的订阅设计'.format(cls.__name__))

print('\n== 5 臂坐标是无符号编码(<II), 负值必须补码还原 ==')
subject = ArmSubject()
subject.decode(b'\x00' + struct.pack('<II', 4294967290, 58))       # -6mm 在线上
raw = subject.arm_data()
print('   线上 4294967290,58 -> ArmSubject 原样给出 {}'.format(raw))
if list(raw) == [4294967290, 58]:
    print('   ✓ 确认无符号: 不做补码还原就会拿 4.29e9 当读数')
else:
    fails.append('ArmSubject 解码结果与预期不符: {}'.format(raw))

print('\n== 6 连接类型必须用 SDK 自己的常量(SDK 内部用 `is` 比, 不是 ==) ==')
# conn.request_connection 里是 `if conn_type is CONNECTION_WIFI_AP: / elif ... is ..._STA:`
# 三个分支全用 `is` 比。我们从 config_ep.json 读出来的字符串是 json.load 新建的对象,
# 与 SDK 模块里的字面量**不是同一个对象** → 三个分支一个都不命中 → proxy_addr 未赋值
# → 第一次连接就 UnboundLocalError。甲在现场能连上只因他脚本里写的是字面量(会被驻留)。
raw = json.loads('{"x": "ap"}')['x']            # 模拟 config_ep.json 的读法(json.load)
print('   json.load 读出的 "ap" is conn.CONNECTION_WIFI_AP -> {}'
      .format(raw is conn_mod.CONNECTION_WIFI_AP))
if raw is conn_mod.CONNECTION_WIFI_AP:
    print('   注: 本解释器恰好驻留了这个字符串 —— 换成别的写法就不成立, 下面照样按映射断言')

ALIAS = {'ap': 'CONNECTION_WIFI_AP', 'sta': 'CONNECTION_WIFI_STA',
         'rndis': 'CONNECTION_USB_RNDIS'}
for alias, const in ALIAS.items():
    got = rm_mod._sdk_conn_type(json.loads('"{}"'.format(alias)))   # 走 json 读出来的字符串
    want = getattr(conn_mod, const)
    ok = got is want
    print('   {} _sdk_conn_type({!r}) is conn.{}'.format('✓' if ok else '✗', alias, const))
    if not ok:
        fails.append('_sdk_conn_type({!r}) 没映射到 SDK 常量'.format(alias))
# 大小写/空格也收(config 里手写进来的)
for messy in (' AP ', 'Sta'):
    got, s = rm_mod._sdk_conn_type(messy), messy.strip().lower()
    if got is not getattr(conn_mod, ALIAS[s]):
        fails.append('_sdk_conn_type({!r}) 未归一化'.format(messy))

# 光有映射表不够: 得确认 connect() 真的用了它(映射写了却传旧值是本仓库刚踩过的坑)
src = open(os.path.join(REPO, 'ep', 'drive', 'rm.py'), encoding='utf-8').read()
tree = ast.parse(src)
wired = False
for node in ast.walk(tree):
    if isinstance(node, ast.FunctionDef) and node.name == 'connect':
        for sub in ast.walk(node):
            if isinstance(sub, ast.Call) and getattr(sub.func, 'id', '') == '_sdk_conn_type':
                wired = True
print('   {} RM.connect() 里调了 _sdk_conn_type()'.format('✓' if wired else '✗'))
if not wired:
    fails.append('rm.RM.connect() 没用 _sdk_conn_type() —— 映射表形同虚设')

print('\n' + ('RESULT: ALL_OK —— rm.py 的 SDK 用法在这份 SDK 上成立' if not fails
             else 'RESULT: FAIL —— ' + '; '.join(fails)))
sys.exit(1 if fails else 0)
