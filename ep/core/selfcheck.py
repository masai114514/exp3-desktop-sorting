#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 真机包离线自检(纯 stdlib, 不连机器人也能跑; Mac/WSL2 通用)。

校验 config_ep.json 结构与数值合理性(机械臂/夹爪/底盘参数域), 输出 RESULT: ALL_OK。
不校验 SDK 是否安装——那是 env_check.py(连真机)的活。

用法: python3 ep/core/selfcheck.py [--config ep/config_ep.json]
"""
import argparse
import json
import os
import sys

# ---- 数值域(官方规格: 臂 2 轴, 水平 0.22m / 垂直 0.15m; 夹爪 power 1..100) ----
ARM_RANGE_MM = 500          # moveto x/y 绝对值不应超过(超出官方行程会被钳制, 先给宽松界)
GRIP_POWER = (1, 100)
CHASSIS_XY_SPEED = (0.05, 1.2)
# 上/下探两姿态必须不同, 否则没有"下探包块"这个动作
HINT = '用 drive 的 probe 在真机上试出臂姿态后回填, 勿凭空填'


def _fail(errs, msg):
    errs.append(msg)


def check_config(cfg, errs):
    top = ['profile', 'robot', 'chassis', 'arm', 'gripper', 'judge', 'cycle']
    for k in top:
        if k not in cfg:
            _fail(errs, '缺顶层键: {}'.format(k))
    if not cfg.get('description'):
        _fail(errs, 'description 为空(写明平台/口径)')

    # robot
    rt = cfg.get('robot') or {}
    if rt.get('conn_type') not in ('ap', 'sta'):
        _fail(errs, "robot.conn_type 应为 'ap' 或 'sta', 实际: %r" % rt.get('conn_type'))

    # chassis 里程计 waypoints
    #   null = 尚未现场标定(合法状态: env_check --jog 就是标定工具, 必须能在未标定时跑);
    #   是否"已标定到能上真机"由 check_calibrated 单独判定, 只有 ep_cycle 会调它。
    ch = cfg.get('chassis') or {}
    ch_poses = {}
    for k in ('HOME_pose', 'A_pose', 'B_pose'):
        if k not in ch:
            _fail(errs, 'chassis.{} 缺失(标定阶段现场填 odom 读数)'.format(k))
            continue
        p = ch[k]
        ch_poses[k] = p
        if p is None:
            continue
        if (not isinstance(p, list)) or len(p) != 3:
            _fail(errs, 'chassis.{} 应为 [x, y, z(deg)] 三元组或 null(未标定)'.format(k))
        else:
            for v in p:
                if not isinstance(v, (int, float)) or v != v:  # NaN 检查
                    _fail(errs, 'chassis.{} 含非数值: {}'.format(k, p))
                    break
    sp = ch.get('xy_speed', 0.5)
    if not (CHASSIS_XY_SPEED[0] <= sp <= CHASSIS_XY_SPEED[1]):
        _fail(errs, 'chassis.xy_speed={} 超出合理域 {}..{}'.format(sp, *CHASSIS_XY_SPEED))
    if all(isinstance(ch_poses.get(k), list) and len(ch_poses[k]) == 3
           for k in ('A_pose', 'B_pose')):
        a, b = ch_poses['A_pose'], ch_poses['B_pose']
        d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        if d < 0.1:
            _fail(errs, 'A_pose 与 B_pose 几乎重合(d={:.2f}m), 检查是否还没标定'.format(d))

    # arm
    am = cfg.get('arm') or {}
    if not isinstance(am.get('attr_candidates'), list) or not am['attr_candidates']:
        _fail(errs, 'arm.attr_candidates 应为非空列表(真机探测用候选属性名)')
    poses = {}
    for k in ('grab_low', 'lift_high'):
        p = am.get(k)
        poses[k] = p
        if (not isinstance(p, list)) or len(p) != 2:
            _fail(errs, 'arm.{} 应为 [x_mm, y_mm]; {}'.format(k, HINT))
        else:
            for v in p:
                if not isinstance(v, (int, float)):
                    _fail(errs, 'arm.{} 含非数值'.format(k))
    if poses.get('grab_low') and poses.get('lift_high') and \
            tuple(poses['grab_low']) == tuple(poses['lift_high']):
        _fail(errs, 'grab_low == lift_high: 没有下探行程, 请现场标定两档高度')
    for k, p in poses.items():
        if p and isinstance(p, list) and len(p) == 2:
            for v in p:
                if abs(v) > ARM_RANGE_MM:
                    _fail(errs, 'arm.{}({}mm) 超出宽松界 ±{}mm —— 超官方行程会钳制'.format(
                        k, v, ARM_RANGE_MM))

    # gripper
    gp = cfg.get('gripper') or {}
    for k in ('open_power', 'close_power'):
        v = gp.get(k)
        if not (isinstance(v, int) and GRIP_POWER[0] <= v <= GRIP_POWER[1]):
            _fail(errs, 'gripper.{}={!r} 应取整数 {}..{}'.format(k, v, *GRIP_POWER))

    # judge / cycle
    if (cfg.get('judge') or {}).get('held') != 'operator_confirm':
        _fail(errs, 'judge.held 应为 operator_confirm(EP 无物块感知, 暂只支持人工确认)')
    cyc = cfg.get('cycle') or {}
    if not (isinstance(cyc.get('cycles'), int) and cyc['cycles'] >= 1):
        _fail(errs, 'cycle.cycles 应为 >=1 整数')


def check_calibrated(cfg, errs):
    """真机驱动前的硬闸门: config 必须已是**现场标定过的真值**。

    故意与 check_config 分开:
      - env_check.py 只调 check_config —— 因为 --jog 本身就是标定臂坐标的工具, 必须能在
        未标定时跑;
      - ep_cycle.py 会驱动底盘与臂, 必须两个都调 —— 绝不允许拿示例占位值上真机。
    背景: 包内原先的占位值([0.35,0,0]/[0.35,0.6,0])恰好能通过 check_config 的全部检查
    (A/B 相距 0.6m > 0.1m 阈值), 即"是否已标定"这条检查从来没起过作用。
    """
    ch = cfg.get('chassis') or {}
    for k in ('HOME_pose', 'A_pose', 'B_pose'):
        if ch.get(k) is None:
            _fail(errs, 'chassis.{} 仍是 null(未标定) —— 按 标定说明.md §1 把 EP 开到标记点,'
                        '用 env_check.py --odom 读 (x,y,z_deg) 回填(SDK 无 get_position, '
                        '里程计靠 sub_position 订阅, cs 见 config.chassis.odom_cs)'.format(k))
    cal = cfg.get('calibration') or {}
    if cal.get('status') != 'calibrated':
        _fail(errs, 'calibration.status={!r} ≠ "calibrated" —— 标定没做完, 拒绝驱动真机'
                    '(标定说明.md §1 底盘 / §2 臂 / §3 夹爪)'.format(cal.get('status')))
    for k in ('by', 'date', 'venue'):
        if not cal.get(k):
            _fail(errs, 'calibration.{} 为空 —— 标定人/日期/场地要留档'
                        '(换场地或重新开机, 底盘里程计原点会变, 必须重标)'.format(k))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config_ep.json'))
    args = ap.parse_args()

    try:
        with open(args.config, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
    except Exception as e:
        print('FAIL: config 读取失败 {}: {}'.format(args.config, e))
        sys.exit(1)

    errs = []
    check_config(cfg, errs)
    if errs:
        print('RESULT: NOT_OK  ({} 项)'.format(len(errs)))
        for e in errs:
            print('  - ' + e)
        sys.exit(1)
    print('RESULT: ALL_OK (结构/数值域)')
    print('config: {}'.format(os.path.abspath(args.config)))
    print('profile: {}'.format(cfg.get('profile')))

    # 标定状态(不是错误, 但必须显眼 —— 未标定 = 不能跑 ep_cycle)
    cal_errs = []
    check_calibrated(cfg, cal_errs)
    if cal_errs:
        print('\n⚠ 未标定 —— 只能用 env_check --jog/--grip 做标定, ep_cycle.py 会被拒绝启动:')
        for e in cal_errs:
            print('  - ' + e)
    else:
        print('标定状态: calibrated (by={} date={} venue={})'.format(
            cfg['calibration']['by'], cfg['calibration']['date'], cfg['calibration']['venue']))
    print('  A_pose={}  B_pose={}  HOME_pose={}'.format(
        cfg['chassis']['A_pose'], cfg['chassis']['B_pose'], cfg['chassis']['HOME_pose']))
    print('  arm grab_low={}mm  lift_high={}mm'.format(
        cfg['arm']['grab_low'], cfg['arm']['lift_high']))
    print('  gripper open/close power = {}/{}'.format(
        cfg['gripper']['open_power'], cfg['gripper']['close_power']))


if __name__ == '__main__':
    main()
