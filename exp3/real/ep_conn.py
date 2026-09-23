# -*- coding: utf-8 -*-
"""连 EP 的那一段单独放这里 —— 跑任务（run_real.py）和标定（calibrate.py）都要连机器人，
而"两份 config 怎么拼成 RM 能吃的参数"只该有一份：抄第二份一定会漂。

RM 本体是**实验二那套**（`mecharm-grasp-exp/ep/drive/rm.py`），**故意不在 exp3/ 里再抄一份**：
两份驱动一定会漂。代价是 exp3 的独立快照（`exp3_public/`）里没有 `ep/`，那边连不了真机。

连不上时抛 `EPConnectError`，消息里带**可执行的排查步骤**（现场不看 traceback）。调用方
（run_real.py）负责把它记成一次"连都没连上"的运行 —— 与"跑了失败"区分开，这是实验二
踩出来的口径（`real/标定说明.md` §6.5）。
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXP3 = os.path.dirname(_HERE)

# config_real/ep_waypoints.json 的字段名 → ep/config_ep.json 里 RM 认的字段名
_ARM_MAP = (('grab_low', 'grab_low_mm'), ('lift_high', 'lift_high_mm'))


class EPConnectError(Exception):
    """连不上 EP（驱动 import 失败 / SDK 没装 / 连不上 / 配置读不到）。

    消息自带排查步骤 —— 现场要的是"下一步做什么"，不是 traceback。
    """


def ep_config_path():
    """实验二的连接参数文件（robot/chassis/arm/gripper 的默认值都在里面）。"""
    return os.path.normpath(os.path.join(_EXP3, '..', 'ep', 'config_ep.json'))


def load_rm_class():
    """import 实验二的 RM 驱动。失败 → EPConnectError（说清它在哪、快照里没有）。"""
    repo = os.path.dirname(_EXP3)
    if repo not in sys.path:
        sys.path.insert(0, repo)
    try:
        from ep.drive.rm import RM
    except ImportError as e:
        raise EPConnectError(
            '真机驱动 import 失败（%s）\n'
            '  它在 mecharm-grasp-exp/ep/drive/rm.py，是实验二那套。\n'
            '  若你拿的是 exp3 的独立快照，目录里没有 ep/ —— 真机跑请在完整仓库里执行；'
            '只想演练就加 --dry-run。' % e)
    return RM


def build_rm_config(ep_cfg, arm=None, gripper=None, log=None):
    """`ep/config_ep.json`（连接参数）+ `config_real/ep_waypoints.json`（真机点位/姿态）→ RM 的 config。

    arm / gripper 是可选的**覆盖**，给标定用：标定时 config_real 里往往还是 null，
    得能拿一组候选值先试（`calibrate.py --record-arm grab 74 120`）。
    """
    import json
    p = ep_config_path()
    try:
        with open(p, encoding='utf-8') as f:
            base = json.load(f)
    except IOError as e:
        raise EPConnectError('读不到 EP 连接配置 %s: %s\n'
                            '（它在实验二仓库里，exp3 的独立快照没有）' % (p, e))

    base['chassis'] = dict(base.get('chassis') or {})
    base['chassis'].update({k: v for k, v in (ep_cfg.get('chassis') or {}).items()
                            if k != '_note'})

    # 臂：grab_low / lift_high 没标定（None）时**不写进去**，让 RM 用它自己的默认值 ——
    # 塞个 None 进去会让它在 SDK 层炸得莫名其妙。
    # release_low_override 例外：None 在那是**合法值**（= 与 grab_low 同高），必须照传。
    base['arm'] = dict(base.get('arm') or {})
    src_arm = dict(ep_cfg.get('arm') or {})
    src_arm.update(arm or {})
    for dest, field in _ARM_MAP:
        v = src_arm.get(field)
        if v is None:
            if log:
                log.text('臂 %s 未标定（config_real 里是 null）→ 本次用 %s 里的默认值'
                         % (field, os.path.basename(p)))
        else:
            base['arm'][dest] = v
    src_arm_release = (arm or {}).get('release_low_override',
                                      (ep_cfg.get('arm') or {}).get('release_low_mm'))
    base['arm']['release_low_override'] = src_arm_release
    base['arm']['lift_wait_s'] = (ep_cfg.get('arm') or {}).get('lift_wait_s', 1.0)
    base['arm']['drop_wait_s'] = (ep_cfg.get('arm') or {}).get('drop_wait_s', 1.0)

    src_grip = dict(ep_cfg.get('gripper') or {})
    src_grip.update(gripper or {})
    base['gripper'] = dict(base.get('gripper') or {})
    for k, v in src_grip.items():
        if k.startswith('_') or v is None:
            continue
        base['gripper'][k] = v
    return base


def connect(ep_cfg, log=None, arm=None, gripper=None):
    """建 RM 并连上。返回连好的 RM；连不上抛 EPConnectError。"""
    RM = load_rm_class()
    base = build_rm_config(ep_cfg, arm=arm, gripper=gripper, log=log)
    if log:
        log.text('EP 连接参数: conn_type=%s  arm 经 %s'
                 % ((base.get('robot') or {}).get('conn_type'),
                    (base.get('arm') or {}).get('attr_candidates')))
    rm = RM(base, log)
    try:
        rm.connect()
    except Exception as e:
        raise EPConnectError(
            '连不上 EP —— %s\n'
            '  排查: ① robomaster SDK 装了吗（python3 -c "import robomaster"）\n'
            '        ② 电脑连上 EP 热点了? 还是该把 ep/config_ep.json 的 conn_type 改 sta?\n'
            '        ③ EP 是**工程形态**吗（步兵形态没有 arm/gripper）\n'
            '  连通性试探: python3 real/calibrate.py --odom（刷出 odom 数就是连上了，Ctrl-C 停）\n'
            '  排查步骤: real/标定说明.md §0' % e)
    return rm
