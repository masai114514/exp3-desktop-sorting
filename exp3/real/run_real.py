#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验3 真机入口：相机 → 检测 → TaskController → EP 后端（一条命令起整链）。

    python3 real/run_real.py --config-dir config_real            # 真机
    python3 real/run_real.py --config-dir config_real --dry-run  # 演练（不动真机）

启动顺序**不可颠倒**：
  1. 读 config_real/（task + grid_cells + bins + ep_waypoints）
  2. **闸门**：结构自检 + check_calibration —— 没过就退出，**连机器人都不连**
  3. 建相机 + 检测器 → scan
  4. 连 EP（--dry-run 用假 EP）→ pick_place
  5. TaskController 跑任务，逐轮落 exp3_logs_real/run_*/

为什么闸门必须在连机器人**之前**：实验二的教训是 config 里编的占位值照样通过全部结构检查
（A/B 相距 0.6m > 0.1m 阈值，恰好过），于是"是否已标定"这条检查从来没起过作用。真机一旦
拿没标定的点位跑，轻则空抓重则撞桌。所以这里把它前置成硬门，并且不在同一个进程里给"跳过
闸门"的选项 —— 要跳过只能去改 config 里的标定记录，那是留痕的。

与仿真线的分工：sort_core / 契约 / 日志 schema 两边**完全共用**；差别只有 config 目录
（config/ 是甲的场景坐标，config_real/ 是现场实测值）和这两个缝隙的实现。
"""
import argparse
import json
import os
import sys
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXP3 = os.path.dirname(_HERE)
for p in (_EXP3, _HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import config_check                                        # noqa: E402  (exp3/config_check.py)
from sort_core.logging_util import Exp3RunLog, make_run_dir  # noqa: E402
from sort_core.task import OK, TaskController               # noqa: E402
from sort_core.taxonomy import REASON_ABORTED, REASON_SAFETY_STOP  # noqa: E402
from real.camera import (MockDetector, YoloDetector, load_detector_file,  # noqa: E402
                         load_scene, make_scan, open_camera)
from real.dry_run import DryRunRM                           # noqa: E402
from real.ep_backend import EPPickExecutor, EPPickPlace      # noqa: E402


def _load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _ask_stdin(question):
    """HELD/PLACED 人工确认。q = 中止整轮（由外层收尾）。"""
    while True:
        ans = input('%s (y/n/q) > ' % question).strip().lower()
        if ans in ('y', 'yes'):
            return True
        if ans in ('n', 'no'):
            return False
        if ans in ('q', 'quit', 'x'):
            raise KeyboardInterrupt('操作员中止')


def load_real_configs(config_dir):
    need = ('task.json', 'grid_cells.json', 'bins.json', 'ep_waypoints.json')
    out = {}
    for name in need:
        p = os.path.join(config_dir, name)
        if not os.path.isfile(p):
            raise SystemExit('真机线缺文件: %s\n（--config-dir 指到 config_real/ 了吗？'
                             '仿真线的 config/ 里没有 ep_waypoints.json）' % p)
        out[name.split('.')[0]] = _load(p)
    return out


def gate(cfg, config_dir):
    """未过闸门 → 打印原因并退出（**不连机器人**）。返回标定记录留档用。"""
    task, grid, bins, ep = cfg['task'], cfg['grid_cells'], cfg['bins'], cfg['ep_waypoints']
    issues = config_check.check_all_with_ep(task, grid, bins, ep)
    cal = config_check.check_calibration(task, grid, bins, ep)
    n_err = sum(1 for it in issues if it['severity'] == config_check.ERROR)
    if issues:
        print('config 结构自检：%d 项' % len(issues))
        for it in issues:
            print('  [%s] %-6s %s' % (it['severity'], it['who'], it['msg']))
    if n_err == 0 and not issues:
        print('config 结构自检：通过（%s）' % config_dir)
    if cal:
        print('\nFAIL: 未通过真机标定闸门（%d 项）—— 拒绝启动，不会连机器人：' % len(cal))
        for it in cal:
            print('  [%s] %-6s %s' % (it['severity'], it['who'], it['msg']))
        print('\n自检命令: python3 config_check.py --config-dir %s --require-calibrated' % config_dir)
        print('标定步骤: ep/标定说明.md §1(底盘 odom) §2(臂 jog) §3(夹爪试夹)；'
              '相机标定见 config_real/task.json 的 camera.calibration 说明')
        sys.exit(1)
    if n_err:
        print('\nFAIL: config 结构有 %d 项 ERROR —— 拒绝启动' % n_err)
        sys.exit(1)
    return dict(camera=(task.get('camera') or {}).get('calibration'),
                grid=grid.get('calibration'), bins=bins.get('calibration'),
                ep=ep.get('calibration'))


def build_detector(args, grid, task, log):
    """--detector mock|yolo|file → 检测器。mock 另外返回它的场景状态（真机不用）。"""
    cam_calib = task['camera']['calib']
    if args.detector == 'mock':
        scene = load_scene(args.scene)
        det = MockDetector(scene.get('objects'), grid['cells'], cam_calib,
                           extra_dets=scene.get('extra_dets'), log=log)
        log.text('检测器: mock（%d 个物体 + %d 条注入检测）—— ★不是真模型，只演练链路'
                 % (len(det.objects), len(det.extra)))
        return det
    if args.detector == 'yolo':
        if not args.model:
            raise SystemExit('--detector yolo 需要 --model 指向 .pt')
        # 模型的 conf 门槛故意压低（0.25）：低置信检测要**发得出来**，由决策层按
        # task.conf_min 判成 unrecognized_object。若在这里就按 conf_min 卡掉，
        # "置信度不足"这类异常在日志里永远看不到。
        return YoloDetector(args.model, conf_min=0.25, log=log)
    if args.detector == 'file':
        if not args.detector_file:
            raise SystemExit('--detector file 需要 --detector-file 指向检测器 .py')
        return load_detector_file(args.detector_file, log)
    raise SystemExit('未知 --detector: %r（mock | yolo | file）' % args.detector)


def main(argv=None):
    here = _EXP3
    ap = argparse.ArgumentParser(description='实验3 真机入口（相机→检测→任务控制→EP）')
    ap.add_argument('--config-dir', default=os.path.join(here, 'config_real'))
    ap.add_argument('--dry-run', action='store_true',
                    help='不连 EP，用假机械臂把动作链打出来（演练；结果不能当验收凭据）')
    ap.add_argument('--camera', default='cv2:0', help='none | cv2:0 | image:/path')
    ap.add_argument('--detector', default='mock', choices=['mock', 'yolo', 'file'])
    ap.add_argument('--scene', default=os.path.join(_HERE, 'scenes', 'demo_6obj.json'),
                    help='mock 检测器的物体表')
    ap.add_argument('--model', default='', help='yolo: .pt 路径')
    ap.add_argument('--detector-file', default='', help='file: 定义 detect(frame) 的 .py')
    ap.add_argument('--log-dir', default=None, help='覆盖 config 里的 log_dir')
    ap.add_argument('--assume-judge', action='store_true',
                    help='★把人工确认换成"一律算成功" —— 只给无人值守演练，真机验收别开')
    ap.add_argument('--no-pause', action='store_true', help='开局不等回车')
    args = ap.parse_args(argv)

    cfg = load_real_configs(args.config_dir)
    task, grid, bins, ep_cfg = cfg['task'], cfg['grid_cells'], cfg['bins'], cfg['ep_waypoints']
    calib_rec = gate(cfg, args.config_dir)

    if args.assume_judge:
        ep_cfg['judge']['held'] = ep_cfg['judge']['placed'] = 'assume_ok'

    run_dir = make_run_dir(args.log_dir or task.get('log_dir', 'exp3_logs_real'))
    runlog = Exp3RunLog(run_dir)
    runlog.text('=== 实验3 真机运行 ===')
    runlog.text('config-dir=%s  run_dir=%s' % (os.path.abspath(args.config_dir), run_dir))
    runlog.text('camera=%s  detector=%s%s  dry_run=%s  assume_judge=%s'
                % (args.camera, args.detector,
                   ('(' + (args.model or args.detector_file) + ')')
                   if args.detector != 'mock' else '',
                   args.dry_run, args.assume_judge))
    if args.dry_run:
        runlog.text('*' * 68)
        runlog.text('*** 演练模式(dry-run)：不会连 EP、不会动真机。')
        runlog.text('*** 本次 result.json 只证明链路通，**不能**作为真机验收凭据。')
        runlog.text('*' * 68)
    if args.assume_judge:
        runlog.text('*** 警告: HELD/PLACED 已改成 assume_ok —— 没有人确认夹起/放正。'
                    '真机验收不能这样跑。')

    camera = open_camera(args.camera, (task['camera']['image_width_px'],
                                       task['camera']['image_height_px']), runlog)
    detector = build_detector(args, grid, task, runlog)
    scan = make_scan(camera, detector, runlog)

    rm = DryRunRM(runlog) if args.dry_run else _connect_real_ep(ep_cfg, runlog)
    ex = EPPickExecutor(rm, ep_cfg, log=runlog, ask=_ask_stdin)
    pick_place = EPPickPlace(ex, grid, bins, log=runlog)
    if hasattr(detector, 'mark_picked'):
        pick_place = _with_mark_picked(pick_place, detector, runlog)

    def safe_stop(reason):
        runlog.text('=== 安全停止(%s)：收尾（开爪/抬到高/回 HOME）===' % reason)
        ex.safe_reset(reason)

    if not args.no_pause and not args.dry_run:
        print('\n摆物：6 个物体放到网格上（%s 的 (row,col) 见 config_real/grid_cells.json），'
              '%d 类各自入盒。' % (args.config_dir, len(task['classes'])))
        print('过程中**不要**人工判类别、不要人工触发动作 —— 只有夹起/放正两处会问你 y/n。')
        input('摆好后回车开跑 > ')

    ctl = TaskController(task, bins, grid['cells'], scan, pick_place,
                         safe_stop=safe_stop, log=runlog)
    exit_status, summary = None, None
    try:
        summary = ctl.run()
        exit_status = summary['exit_status']
    except KeyboardInterrupt:
        runlog.text('== 操作员中止（q / Ctrl-C）==')
        exit_status = REASON_ABORTED
        safe_stop(REASON_ABORTED)
    except Exception as e:
        runlog.text('== 运行异常，中止并收尾: %s: %s ==' % (type(e).__name__, e))
        runlog.text(traceback.format_exc())      # 现场排障全靠这条，别只留一行
        exit_status = REASON_SAFETY_STOP
        safe_stop(REASON_SAFETY_STOP)
    finally:
        try:
            rm.disconnect()
        finally:
            camera.close()

    if summary is None:                       # 异常/中止：没跑完，据实记 0
        summary = dict(placed_ok=0, objects_seen=0, exit_status=exit_status, reasons={})

    runlog.write_result(summary['placed_ok'], summary['objects_seen'],
                        task['expected_total'], task['pass_line'],
                        exit_status, summary['reasons'])
    _write_real_context(run_dir, args, calib_rec, summary, exit_status, ex, rm, runlog)

    with open(os.path.join(run_dir, 'result.json'), encoding='utf-8') as f:
        res = _load(os.path.join(run_dir, 'result.json'))
    print('\n结果: %s  placed_ok=%d/%d  exit_status=%s'
          % (res['verdict'], res['placed_ok'], res['required_total'], exit_status))
    print('run_dir: %s' % run_dir)
    return 0 if res['verdict'] == 'PASS' else 1


def _with_mark_picked(pick_place, detector, log):
    """给 mock 检测器接上"搬运成功 → 该格物体消失"。

    只在 mock 上做：真检测器的"桌面状态"是它**自己看出来的**，任务层不许去改它 ——
    去改就等于拿自己的判断喂自己，把"物体真的被搬走了吗"变成自我实现，异常也就永远
    暴露不出来了（物体掉了/放歪了，下一轮照样当它还在桌上才对）。
    """
    def wrapped(cell_id, cls):
        res, note = pick_place(cell_id, cls)
        if res == OK and detector.mark_picked(cell_id):
            log.text('mock 桌面: %s 的物体已移走' % cell_id)
        return res, note
    return wrapped


def _connect_real_ep(ep_cfg, log):
    """连 EP。SDK 没装/连不上时给可执行的排查步骤，而不是一段 traceback。

    RM 是实验二那套（mecharm-grasp-exp/ep/drive/rm.py），**故意不在 exp3/ 里再抄一份**：
    两份驱动一定会漂。代价是 exp3 的公开快照(exp3_public/)里没有 ep/，那边连不了真机、
    只能 --dry-run —— 这没问题，公开仓本来也没有机器人。
    """
    repo = os.path.dirname(_EXP3)
    if repo not in sys.path:
        sys.path.insert(0, repo)
    try:
        from ep.drive.rm import RM
    except ImportError as e:
        raise SystemExit('真机驱动 import 失败（%s）—— 它在 mecharm-grasp-exp/ep/drive/rm.py，'
                         '是实验二那套。若你拿的是 exp3 的独立快照，目录里没有 ep/，'
                         '真机跑请在完整仓库里执行；只想演练就加 --dry-run' % e)
    # RM 要的是 ep/config_ep.json 那套结构（robot/chassis/arm/gripper）；
    # 真机点位/姿态从 config_real/ep_waypoints.json 取，两者合起来才够驱动。
    base_path = os.path.normpath(os.path.join(_EXP3, '..', 'ep', 'config_ep.json'))
    try:
        base = _load(base_path)
    except IOError as e:
        raise SystemExit('读不到 EP 连接配置 %s: %s' % (base_path, e))
    base['chassis'] = dict(base.get('chassis') or {})
    base['chassis'].update({k: v for k, v in ep_cfg.get('chassis', {}).items()
                            if k != '_note'})
    base['arm'] = dict(base.get('arm') or {})
    base['arm']['grab_low'] = ep_cfg['arm']['grab_low_mm']
    base['arm']['lift_high'] = ep_cfg['arm']['lift_high_mm']
    base['arm']['release_low_override'] = ep_cfg['arm'].get('release_low_mm')
    base['arm']['lift_wait_s'] = ep_cfg['arm'].get('lift_wait_s', 1.0)
    base['arm']['drop_wait_s'] = ep_cfg['arm'].get('drop_wait_s', 1.0)
    base['gripper'] = dict(base.get('gripper') or {})
    base['gripper'].update({k: v for k, v in (ep_cfg.get('gripper') or {}).items()
                            if k != '_note'})
    log.text('EP 连接参数: conn_type=%s  arm 经 %s'
             % ((base.get('robot') or {}).get('conn_type'),
                (base.get('arm') or {}).get('attr_candidates')))
    rm = RM(base, log)
    try:
        rm.connect()
    except Exception as e:
        print('\nFAIL: 连不上 EP —— %s' % e)
        print('排查: ① robomaster SDK 装了吗（python3 -c "import robomaster"）')
        print('      ② 电脑连上 EP 热点了? 还是该把 ep/config_ep.json 的 conn_type 改 sta?')
        print('      ③ EP 是**工程形态**吗（步兵形态没有 arm/gripper）')
        print('先跑: python3 ep/drive/env_check.py --config ep/config_ep.json')
        sys.exit(1)
    return rm


def _write_real_context(run_dir, args, calib_rec, summary, exit_status, ex, rm, runlog):
    """把"这次是怎么跑出来的"单独留档：result.json 保持与仿真线同一 schema，
    真机特有的上下文(演练/标定记录/人工介入次数)写这里，验收时才查得清。"""
    data = dict(
        dry_run=bool(args.dry_run),
        config_dir=os.path.abspath(args.config_dir),
        camera=args.camera, detector=args.detector,
        detector_source=(args.model or args.detector_file or 'builtin-mock'),
        assume_judge=bool(args.assume_judge),
        judge={'held': ex.judge.get('held'), 'placed': ex.judge.get('placed')},
        calibration=calib_rec,
        operator_confirms=ex.n_asks,
        exit_status=exit_status,
        summary=summary,
        run_dir=run_dir,
    )
    if isinstance(rm, DryRunRM):
        data['dry_run_primitive_calls'] = len(rm.calls)
    with open(os.path.join(run_dir, 'real_run.json'), 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    runlog.text('real_run.json 已写（dry_run=%s，人工确认 %d 次）'
                % (args.dry_run, ex.n_asks))
    if isinstance(rm, DryRunRM):
        runlog.text(rm.dump())


if __name__ == '__main__':
    sys.exit(main())
