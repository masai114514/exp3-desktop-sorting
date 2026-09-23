#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 真机首跑自检: 先离线查 config, 再连真机探测模块/属性名, 确认 arm/gripper 能用。

用法(在 EP 持机人电脑上, 已装 robomaster SDK 并连上 EP 热点):
  python3 ep/drive/env_check.py --config ep/config_ep.json
可选项:
  --offline   只查 config, 不连机器人(本机也能跑)
  --odom      只打印底盘里程计(标 HOME/A/B 用: 把车摆到位, 抄下屏幕上的三个数)
  --jog       把臂移到 lift_high 再回 grab_low 并打印读回的 x/y(标定臂坐标用, 人盯)
  --grip      夹爪开合一次(空夹, 别放手指进去)

本自检会打印**实际导入的 robomaster 是哪一份**(路径 + 版本): 现场"到底跑的是哪个 SDK"
这个问题(官方源码装 / 仓库离线包 / 测试包内 vendor)一问路径就有答案, 省得靠猜。
"""
import argparse
import json
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ep.core.selfcheck import check_config, check_calibrated   # noqa: E402
from ep.drive.rm import RM, _SDK_OK                 # noqa: E402


def _ask(question):
    """y/n 确认; q 直接退出。"""
    while True:
        ans = input('{} (y/n/q) > '.format(question)).strip().lower()
        if ans in ('y', 'yes'):
            return True
        if ans in ('n', 'no'):
            return False
        if ans in ('q', 'quit', 'x'):
            sys.exit(2)


def _sdk_identity():
    """打印实际导入的 robomaster 是哪一份 —— 现场排查第一步。"""
    mod = sys.modules.get('robomaster')
    if mod is None:
        return '未导入(robomaster 装了吗?)'
    ver = getattr(sys.modules.get('robomaster.version'), '__version__', '?')
    return '{}  (版本 {})'.format(getattr(mod, '__file__', '?'), ver)


def print_odom(rm, period=0.5):
    """把里程计按秒刷出来, 供人工摆位读数(标 HOME/A/B)。Ctrl+C 结束。"""
    print('\n== 里程计读数(0.5s 刷一次, Ctrl+C 结束) ==')
    print('   把车摆到目标旁, 等数字稳定后抄下来, 填 config_ep.json 的 '
          'chassis.HOME_pose / A_pose / B_pose')
    print('   原点口径: cs={} —— 1 表示以**上电**位姿为原点(与标定/运行同口径), '
          '0 表示以订阅那一刻为原点'.format(rm.cfg.get('chassis', {}).get('odom_cs', 1)))
    try:
        while True:
            pose = rm.read_chassis_pose()
            print('   odom = [{:.3f} m, {:.3f} m, {:.1f} deg]'.format(*pose))
            time.sleep(period)
    except KeyboardInterrupt:
        print('\n(结束读里程计)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config_ep.json'))
    ap.add_argument('--offline', action='store_true', help='只查 config, 不连机器人')
    ap.add_argument('--odom', action='store_true', help='只刷底盘里程计(标 HOME/A/B 读数用)')
    ap.add_argument('--jog', action='store_true', help='臂到 lift_high/grab_low 各走一次并打印读数')
    ap.add_argument('--grip', action='store_true', help='夹爪空开/合一次')
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = json.load(f)

    # 1) 离线 config 校验
    errs = []
    check_config(cfg, errs)
    if errs:
        print('RESULT: NOT_OK —— config 有问题:')
        for e in errs:
            print('  - ' + e)
        sys.exit(1)
    print('RESULT: ALL_OK (config 离线校验通过)')
    print('A_pose={}  B_pose={}'.format(cfg['chassis']['A_pose'], cfg['chassis']['B_pose']))
    print('arm grab_low={} lift_high={} (mm)'.format(cfg['arm']['grab_low'], cfg['arm']['lift_high']))

    # 标定状态: 本自检**故意**不因未标定而退出(下面 --jog/--grip 就是标定工具),
    # 但要显眼提示: 未标定就别去跑 ep_cycle。
    cal_errs = []
    check_calibrated(cfg, cal_errs)
    if cal_errs:
        print('\n⚠ 未标定(本自检继续, 但 ep_cycle.py 会拒绝启动):')
        for e in cal_errs:
            print('  - ' + e)
        print('  → 现在能做的: --odom 读底盘里程计、--jog 标定臂两档高度、--grip 试夹爪'
              '(标法见 标定说明.md §1–§3)')

    if args.offline or not _SDK_OK:
        print('PASS(离线): 未连真机。装机后把本段以下步骤在 EP 现场跑。')
        sys.exit(0)

    # 2) 连真机 + 探测
    print('\n== SDK 身份(现场排查第一步: 你跑的到底是哪一份 robomaster) ==')
    print('  {}'.format(_sdk_identity()))
    rm = RM(cfg)
    try:
        rm.connect()
        mods = rm.probe()
        print('\n== 模块探测 ==')
        for name in ('chassis', 'arm', 'gripper'):
            methods = mods.get(name)
            print('  {}: {}'.format(name, methods if methods else 'MISSING'))
        if mods.get('arm') is None or mods.get('gripper') is None:
            print('FAIL: arm 或 gripper 缺失 —— 确认是**工程形态**(步兵形态无 arm/gripper)')
            sys.exit(1)
        print('  机械臂经属性: ep_robot.{}'.format(rm.arm_attr))
        # 位置只能订阅(get_position 在官方 0.1.1.68 里不存在) —— 探的是订阅接口
        for m in ('moveto', 'move', 'recenter', 'sub_position', 'unsub_position'):
            if m not in mods['arm']:
                print('  WARN: arm 没有 {}() —— 请核对 SDK 版本'.format(m))
        for m in ('sub_position', 'unsub_position', 'move'):
            if m not in (mods.get('chassis') or []):
                print('  WARN: chassis 没有 {}() —— 请核对 SDK 版本'.format(m))

        print('\n== 当前读数(来自订阅推送, 见 drive/rm.py 的 _PushCache) ==')
        print('  chassis odom: {}'.format([round(v, 3) for v in rm.read_chassis_pose()]))
        print('  arm x/y: {} mm'.format(rm.read_arm_xy()))

        # 3) 可选的标定动作(人盯)
        if args.odom:
            print_odom(rm)
            rm.disconnect()
            return

        if args.jog:
            print('\n--jog: 臂移到 lift_high  ->  停  ->  回 grab_low(现场确认位置/朝向)')
            if not _ask('准备好了? 臂会动'):
                sys.exit(2)
            for tag, pose in (('lift_high', cfg['arm']['lift_high']),
                              ('grab_low', cfg['arm']['grab_low'])):
                print('  移臂 -> {} {}mm'.format(tag, pose))
                pos = rm.arm_moveto(pose[0], pose[1], tag=tag)
                print('  读回 arm x/y: {} mm (与 config 差太多说明坐标换算出错)'.format(pos))
            rm.arm_moveto(cfg['arm']['lift_high'][0], cfg['arm']['lift_high'][1], tag='结束回lift')

        if args.grip:
            print('\n--grip: 夹爪空开/合一次(保持手离开)')
            if not _ask('准备好了? 夹爪会动'):
                sys.exit(2)
            rm.gripper_open()
            rm.gripper_close()
            rm.gripper_open()
            print('  grip open/close OK')
    except Exception as e:
        print('FAIL: {}'.format(e))
        sys.exit(1)
    finally:
        rm.disconnect()

    print('\nPASS: 真机探测通过。接下来可以跑 1 周期验收试跑:')
    print('  python3 ep/drive/ep_cycle.py --config {} --cycles 1'.format(args.config))
    print('  (config 里 pause_each_phase 保持 true, 首跑人盯全程)')


if __name__ == '__main__':
    main()
