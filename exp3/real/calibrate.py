#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验3 真机标定工具 —— 把「推着车到位 → 读数 → 填表」变成一条命令，**不用手抄数**。

    python3 real/calibrate.py                              # 进度：哪些位姿/档位还没标
    python3 real/calibrate.py --odom                       # 实时刷底盘 odom（推着车到位，Ctrl-C 停）
    python3 real/calibrate.py --record c3                  # 把当前位置记进 c3（车已在位）
    python3 real/calibrate.py --record-arm grab 74 120     # 臂试到 (74,120)，确认后记成 grab_low_mm
    python3 real/calibrate.py --grip close --power 60      # 按该档位合一次夹爪（试夹）
    python3 real/calibrate.py --record-grip close 60       # 记下试好的 power 档
    python3 real/calibrate.py --seal --by 马晨曦 --venue 实验室   # 封表 → status=calibrated
    python3 real/calibrate.py --invalidate                 # 换场地/重新上电后：底盘作废，重标

为什么不用 `ep/drive/env_check.py`：那是**实验二**那套，`--odom` 写死标 HOME/A/B 三个点，
`--jog` 读的是 `ep/config_ep.json` 的臂档位。实验三要 **6 格 + 2 料盒 + 归位点 = 9 个位姿**，
臂/夹爪档位在 `config_real/ep_waypoints.json` 里。用它标实验三，只能肉眼看屏幕抄 9×3 个数 ——
**抄错一个就是一个撞桌的点位**。这里直接把读数写进表，写完 `--progress` 立刻能复看。

★ 两条硬规则（都印在代码里，不靠记）：
  1. **odom 原点 = 上次上电时底盘所在的位置**。换场地 / 重新上电 ⇒ 9 个底盘位姿全部作废，
     必须重标并把 `status` 打回 `uncalibrated`（`--invalidate` 就是干这个的）。**臂和夹爪的档位
     不受影响**（它们在臂坐标系里，和车停在哪无关），不用重标。
  2. **越界的目标值会被 SD​K 安静地钳制**，臂"稳定地停在"一个并非目标的读数上。所以
     `--record-arm` 会核对读回值：差超过 `ARM_MISS_TOL_MM` 就**拒绝记录**（除非 --force）——
     把被钳制的值填进表，真机上就是一个永远够不着、还只打 WARN 的点。
"""
import argparse
import copy
import datetime
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXP3 = os.path.dirname(_HERE)
for p in (_EXP3, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import config_check                                        # noqa: E402
from real import ep_conn                                   # noqa: E402

_CELL_SLOTS = ('c1', 'c2', 'c3', 'c4', 'c5', 'c6')
_BIN_SLOTS = ('bin_cup', 'bin_mouse')
_HOME_SLOTS = ('home', 'home_pose')
_ARM_SLOTS = ('grab', 'lift', 'release')
_ARM_FIELD = {'grab': 'grab_low_mm', 'lift': 'lift_high_mm', 'release': 'release_low_mm'}
_GRIP_FIELD = {'open': 'open_power', 'close': 'close_power'}
_ALL_SLOTS = _CELL_SLOTS + _BIN_SLOTS + _HOME_SLOTS
# `--record` 接受 home 与 home_pose 两种写法（都指 chassis.home_pose）；列清单时只报一个。
_CANON_SLOTS = _CELL_SLOTS + _BIN_SLOTS + ('home_pose',)


# ---------------------------------------------------------------- JSON 读写
def waypoints_path(config_dir):
    return os.path.join(config_dir, 'ep_waypoints.json')


def load_ep(config_dir):
    p = waypoints_path(config_dir)
    if not os.path.isfile(p):
        raise SystemExit('读不到 %s\n（--config-dir 指到 config_real/ 了吗？）' % p)
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def save_ep(config_dir, ep):
    """写回。用 2 空格缩进、保留中文 —— 这个文件是给人看和手改的，不是给机器读的。"""
    p = waypoints_path(config_dir)
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(ep, f, ensure_ascii=False, indent=2)
        f.write('\n')
    return p


def slot_ref(ep, slot):
    """槽名 → (装它的字典, 键名)。未知槽名返回 (None, None)。"""
    if slot in _CELL_SLOTS:
        return ep.setdefault('cell_pose', {}), slot
    if slot in _BIN_SLOTS:
        return ep.setdefault('bin_pose', {}), slot
    if slot in _HOME_SLOTS:
        return ep.setdefault('chassis', {}), 'home_pose'
    return None, None


# ---------------------------------------------------------------- 人机交互
def _ask(question, assume_yes=False):
    if assume_yes:
        print('%s [--yes 自动答 y]' % question)
        return True
    while True:
        ans = input('%s (y/n) > ' % question).strip().lower()
        if ans in ('y', 'yes'):
            return True
        if ans in ('n', 'no'):
            return False


def _fmt_pose(p):
    return '[%s]' % ', '.join(('%.3f' % float(v)) for v in p)


# ---------------------------------------------------------------- 动作
def cmd_progress(config_dir):
    ep = load_ep(config_dir)
    cells, bins = ep.get('cell_pose') or {}, ep.get('bin_pose') or {}
    chassis, arm, grip = ep.get('chassis') or {}, ep.get('arm') or {}, ep.get('gripper') or {}
    cal = ep.get('calibration') or {}
    n_missing = 0

    def row(label, value, note='', optional=False):
        """optional=True 用于 **null 本身就合法**的项（release_low_mm）——
        它没值时算"默认"，不算"还差"。否则 --progress 数出来会比 --seal 的判据多一项，
        人就会去编一个 release 值填上：那正是本工具存在的理由（编的值能骗过全部检查）。"""
        nonlocal n_missing
        if value is None and not optional:
            n_missing += 1
            print('  %-16s %-28s %s' % (label, '★未标', note))
        elif value is None:
            print('  %-16s %-28s %s' % (label, '（未填=同 grab）', note))
        else:
            print('  %-16s %-28s %s' % (label, value, note))

    print('EP 位姿表：%s' % waypoints_path(config_dir))
    print('calibration.status = %s   by=%s  date=%s  venue=%s'
          % (cal.get('status'), cal.get('by'), cal.get('date'), cal.get('venue')))
    print('\n底盘位姿（odom 读数 [x_m, y_m, z_deg]）—— 推着车到位，一条 --record 记一个：')
    for s in _CELL_SLOTS:
        row(s, _fmt_pose(cells[s]) if isinstance(cells.get(s), list) else None, '取物网格')
    for s in _BIN_SLOTS:
        row(s, _fmt_pose(bins[s]) if isinstance(bins.get(s), list) else None, '分类料盒')
    row('home_pose', _fmt_pose(chassis['home_pose'])
        if isinstance(chassis.get('home_pose'), list) else None, '归位点')

    print('\n臂（mm，[x 前伸, y 高度]）—— 用 --record-arm 试一个记一个：')
    row(_ARM_FIELD['grab'], arm.get('grab_low_mm'), '爪指包住目标物')
    row(_ARM_FIELD['lift'], arm.get('lift_high_mm'), '爪底高于目标物顶面')
    row(_ARM_FIELD['release'], arm.get('release_low_mm'),
        'null 合法 = 与 grab_low_mm 同高（料盒有沿才单独标）', optional=True)

    print('\n夹爪（power 1..100）—— 用 --record-grip 记：')
    row('open_power', grip.get('open_power'))
    row('close_power', grip.get('close_power'))

    print('\n%s' % ('—' * 66))
    if n_missing:
        print('还差 %d 项。下一步：' % n_missing)
        print('  推着车到某个网格位 → python3 real/calibrate.py --record c1')
        print('  臂试位置           → python3 real/calibrate.py --record-arm grab 74 120')
        print('  试夹爪             → python3 real/calibrate.py --grip close --power 60')
    else:
        print('★ 本表已填全。下一步（封表 + 过全闸门）：')
        print('  python3 real/calibrate.py --seal --by <你的名字> --venue <场地>')
        print('  python3 config_check.py --config-dir config_real --require-calibrated')
    print('\n※ 这份只含 **EP 位姿表**。相机(=乙) / 网格 / 料盒 三份在 task.json、'
          'grid_cells.json、bins.json 里，闸门会分别点名。')
    return 0 if not n_missing else 1


def cmd_odom(config_dir, ep, period=0.5):
    """实时刷 odom（供人工摆位读数）。Ctrl-C 结束。"""
    import time
    rm = ep_conn.connect(ep, log=None)
    print('实时 odom（推着车/遥控到位 → 记下这三个数 → Ctrl-C 停）。'
          '原点 = 上次上电处。')
    try:
        while True:
            x, y, z = rm.read_chassis_pose()
            sys.stdout.write('\r  odom  x=%8.3f  y=%8.3f  z=%8.3f deg   ' % (x, y, z))
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print('\n停。把这个数用 --record <槽名> 记进表（别忘了它只对**本次上电**有效）。')
    finally:
        rm.disconnect()
    return 0


def cmd_record(config_dir, ep, slot, assume_yes=False, force=False):
    parent, key = slot_ref(ep, slot)
    if parent is None:
        raise SystemExit('未知槽名 %r。可用：%s' % (slot, ' / '.join(_ALL_SLOTS)))
    old = parent.get(key)
    if isinstance(old, list) and not force:
        raise SystemExit('%s 已经有值 %s。要覆盖请加 --force。' % (slot, _fmt_pose(old)))

    rm = ep_conn.connect(ep, log=None)
    try:
        pose = [float(v) for v in rm.read_chassis_pose()]
    finally:
        rm.disconnect()

    print('当前位置 odom = %s' % _fmt_pose(pose))
    if not _ask('车现在就在 %s 的位置吗？把上面这个读数记进 %s' % (slot, slot), assume_yes):
        print('没记。')
        return 1
    parent[key] = pose
    p = save_ep(config_dir, ep)
    print('已写入 %s 的 %s = %s' % (os.path.basename(p), slot, _fmt_pose(pose)))
    if isinstance(old, list):
        print('（旧值 %s 已被覆盖）' % _fmt_pose(old))
    print('※ 这个数只对**本次上电**有效 —— 重新上电后原点会变，必须重标（见 --invalidate）。')
    return 0


def cmd_record_arm(config_dir, ep, which, x_mm, y_mm, assume_yes=False, force=False):
    if which not in _ARM_SLOTS:
        raise SystemExit('未知臂档 %r。可用：%s' % (which, ' / '.join(_ARM_SLOTS)))
    field = _ARM_FIELD[which]
    old = (ep.get('arm') or {}).get(field)
    if old is not None and not force:
        raise SystemExit('%s 已经有值 %s。要覆盖请加 --force。' % (field, old))

    x_mm, y_mm = int(x_mm), int(y_mm)
    rm = ep_conn.connect(ep, log=None)
    try:
        rm.arm_moveto(x_mm, y_mm, tag=which)
        pos = [int(v) for v in rm.read_arm_xy()]
    finally:
        rm.disconnect()

    miss = max(abs(pos[0] - x_mm), abs(pos[1] - y_mm))
    tol = float(getattr(type(rm), 'ARM_MISS_TOL_MM', 3.0))
    print('要求 (%d, %d) → 读回 (%d, %d)，差 %.1f mm（容差 %.1f）'
          % (x_mm, y_mm, pos[0], pos[1], miss, tol))
    if miss > tol and not force:
        raise SystemExit(
            '★ 读回值和目标差 %.0f mm —— 多半是**超出行程被 SDK 钳制**了。\n'
            '  臂停在的并不是你要的位置。**别把这个数填进表**：真机上它会让这一档永远够不着，'
            '而且只打一条 WARN，现象看起来像"抓取不稳定"。\n'
            '  换个更靠内的目标再试（实验二实测 grab 的 x 在几十 mm 量级，x≈74）。\n'
            '  确实要强行记录就加 --force。' % miss)

    if not _ask('爪的位置对吗（这一档该在的地方）？把 [%d, %d] 记进 %s'
                % (x_mm, y_mm, field), assume_yes):
        print('没记。')
        return 1
    ep.setdefault('arm', {})[field] = [x_mm, y_mm]
    p = save_ep(config_dir, ep)
    print('已写入 %s 的 %s = [%d, %d]   （[x 前伸, y 高度] mm）'
          % (os.path.basename(p), field, x_mm, y_mm))
    return 0


def cmd_grip(config_dir, ep, which, power=None):
    if which not in _GRIP_FIELD:
        raise SystemExit('未知夹爪动作 %r。可用：open / close' % which)
    grip = ep.get('gripper') or {}
    if power is not None:
        _check_power(power)
        over = {_GRIP_FIELD[which]: int(power)}
    else:
        over = {}
    rm = ep_conn.connect(ep, log=None, gripper=over)
    try:
        (rm.gripper_open if which == 'open' else rm.gripper_close)()
    finally:
        rm.disconnect()
    eff = over.get(_GRIP_FIELD[which], grip.get(_GRIP_FIELD[which]))
    print('夹爪 %s 一次（power=%s）。看看：夹稳不掉 + 电机不啸叫。'
          % (which, eff if eff is not None else '（用 config 默认）'))
    if power is not None:
        print('这档合适就用 --record-grip %s %d 记进表。' % (which, int(power)))
    return 0


def _check_power(power):
    try:
        n = int(power)
    except (TypeError, ValueError):
        raise SystemExit('power 要是整数（官方范围 1..100）')
    if not 1 <= n <= 100:
        raise SystemExit('power=%d 越界，官方范围 1..100' % n)
    return n


def cmd_record_grip(config_dir, ep, which, power):
    if which not in _GRIP_FIELD:
        raise SystemExit('未知夹爪动作 %r。可用：open / close' % which)
    n = _check_power(power)
    field = _GRIP_FIELD[which]
    old = (ep.get('gripper') or {}).get(field)
    ep.setdefault('gripper', {})[field] = n
    p = save_ep(config_dir, ep)
    print('已写入 %s 的 %s = %d（旧值 %s）' % (os.path.basename(p), field, n, old))
    return 0


def cmd_seal(config_dir, ep, by, venue, date=None):
    """三处填全 → 写 by/date/venue + status=calibrated。"""
    missing = _ep_missing(ep)
    if missing:
        print('★ 还不能封表 —— 本表还有 %d 项没填：' % len(missing))
        for m in missing:
            print('  - %s' % m)
        print('先跑 python3 real/calibrate.py 看进度。')
        return 1
    if not by or not venue:
        raise SystemExit('--seal 要 --by <名字> --venue <场地>（留档：出问题时知道找谁、在哪儿标的）')
    date = date or datetime.date.today().isoformat()
    ep.setdefault('calibration', {}).update(
        {'status': 'calibrated', 'by': by, 'date': date, 'venue': venue})
    p = save_ep(config_dir, ep)
    print('已封表：%s\n  status=calibrated  by=%s  date=%s  venue=%s' % (p, by, date, venue))
    print('\n下一步（全闸门，四份记录一起查 —— 相机/网格/料盒也要填好）：')
    print('  python3 config_check.py --config-dir config_real --require-calibrated')
    print('  过闸门后再演练：python3 real/run_real.py --config-dir config_real --dry-run \\')
    print('                          --camera none --detector mock --assume-judge --no-pause')
    print('\n※ 换场地 / 重新上电后跑 --invalidate 把 status 打回去。')
    return 0


def _filled_poses(ep):
    """已填的底盘位姿槽名（去重：home / home_pose 是同一个槽的两个写法）。
    不读 ep 的返回顺序无关紧要，只用来数个数和列名字。"""
    seen, out = set(), []
    for s in _ALL_SLOTS:
        parent, key = slot_ref(ep, s)
        if parent is None or key in seen:
            continue
        seen.add(key)
        if isinstance(parent.get(key), list):
            out.append(key)
    return out


def cmd_invalidate(config_dir, ep):
    """换场地/重新上电 → 底盘位姿全部作废（臂/夹爪不受影响）。"""
    cal = ep.setdefault('calibration', {})
    was = cal.get('status')
    cal['status'] = 'uncalibrated'
    save_ep(config_dir, ep)
    n_pose = len(_filled_poses(ep))
    print('status: %s → uncalibrated（已写回）' % was)
    print('\n为什么：**odom 原点 = 上次上电时底盘所在的位置**。上电位置一换，'
          '表里 %d 个底盘位姿全部指向错误的地方。' % n_pose)
    print('要重标的：%s' % '、'.join(_CANON_SLOTS))
    print('**不用重标的**：臂两档（%s）与夹爪档位（%s）—— 它们在臂坐标系里，'
          '和车停在哪无关。' % ('、'.join(_ARM_FIELD.values()), '、'.join(_GRIP_FIELD.values())))
    print('\n重标：python3 real/calibrate.py --odom 然后逐个 --record；'
          '完了再 --seal。')
    return 0


def _ep_missing(ep):
    """本表（EP 那份）还没填的项，人类可读。"""
    out = []
    cells, bins = ep.get('cell_pose') or {}, ep.get('bin_pose') or {}
    chassis, arm, grip = ep.get('chassis') or {}, ep.get('arm') or {}, ep.get('gripper') or {}
    for s in _CELL_SLOTS:
        if not isinstance(cells.get(s), list):
            out.append('底盘位姿 %s' % s)
    for s in _BIN_SLOTS:
        if not isinstance(bins.get(s), list):
            out.append('底盘位姿 %s' % s)
    if not isinstance(chassis.get('home_pose'), list):
        out.append('底盘位姿 home_pose')
    if arm.get('grab_low_mm') is None:
        out.append('臂 grab_low_mm')
    if arm.get('lift_high_mm') is None:
        out.append('臂 lift_high_mm')
    if grip.get('open_power') is None:
        out.append('夹爪 open_power')
    if grip.get('close_power') is None:
        out.append('夹爪 close_power')
    return out


# ---------------------------------------------------------------- CLI
def main(argv=None, assume_yes=False):
    ap = argparse.ArgumentParser(
        description='实验3 真机标定工具（读数直接写进 config_real/ep_waypoints.json，不用手抄）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config-dir', default=os.path.join(_EXP3, 'config_real'))
    ap.add_argument('--progress', action='store_true', help='列出哪些位姿/档位还没标（默认动作）')
    ap.add_argument('--odom', action='store_true', help='实时刷底盘 odom 供摆位读数（Ctrl-C 停）')
    ap.add_argument('--record', metavar='SLOT',
                    help='把当前底盘位姿记进 SLOT（%s）' % ' / '.join(_ALL_SLOTS))
    ap.add_argument('--record-arm', nargs=3, metavar=('WHICH', 'X', 'Y'),
                    help='臂试到 (X,Y) 并记进 grab / lift / release')
    ap.add_argument('--grip', choices=['open', 'close'], help='按该档位张/合一次夹爪（试夹）')
    ap.add_argument('--power', type=int, help='配 --grip：用这个 power 试（1..100），不写表')
    ap.add_argument('--record-grip', nargs=2, metavar=('WHICH', 'POWER'),
                    help='记下试好的 open/close power')
    ap.add_argument('--seal', action='store_true', help='本表填全 → 写 by/date/venue + calibrated')
    ap.add_argument('--invalidate', action='store_true',
                    help='换场地/重新上电后：status 打回 uncalibrated（底盘作废，臂/夹爪不受影响）')
    ap.add_argument('--by', default='', help='配 --seal：标定人')
    ap.add_argument('--venue', default='', help='配 --seal：场地')
    ap.add_argument('--date', default='', help='配 --seal：日期（默认今天）')
    ap.add_argument('--force', action='store_true', help='覆盖已有值 / 强行记录被钳制的臂读数')
    ap.add_argument('--yes', action='store_true', help='不逐项确认（脚本化用；现场别开）')
    args = ap.parse_args(argv)
    assume_yes = assume_yes or args.yes

    if args.odom:
        return cmd_odom(args.config_dir, load_ep(args.config_dir))
    if args.invalidate:
        return cmd_invalidate(args.config_dir, load_ep(args.config_dir))
    if args.seal:
        return cmd_seal(args.config_dir, load_ep(args.config_dir),
                        args.by, args.venue, args.date or None)
    if args.record:
        return cmd_record(args.config_dir, load_ep(args.config_dir), args.record,
                          assume_yes, args.force)
    if args.record_arm:
        which, x, y = args.record_arm
        try:
            x, y = int(x), int(y)
        except ValueError:
            raise SystemExit('--record-arm 的 X Y 要是整数 mm')
        return cmd_record_arm(args.config_dir, load_ep(args.config_dir), which, x, y,
                              assume_yes, args.force)
    if args.grip:
        return cmd_grip(args.config_dir, load_ep(args.config_dir), args.grip, args.power)
    if args.record_grip:
        which, power = args.record_grip
        return cmd_record_grip(args.config_dir, load_ep(args.config_dir), which, power)
    return cmd_progress(args.config_dir)


if __name__ == '__main__':
    sys.exit(main())
