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

**标定**用 `real/calibrate.py`（读数直接写进 ep_waypoints.json，不用手抄）；步骤见
`real/标定说明.md`。

**连不上 EP 不算"跑了失败"**：那种情况也留一份 `result.json`（`placed_ok=0`、
`exit_status=no_exec`），另外在 `real_run.json` 里记 `outcome=connect_failed` 与错误原文 ——
否则目录里只有 run.log、没有 result.json，"没连上"和"跑挂了"从文件上分不出来（实验二
实验二 `ep/标定说明.md` §4.5 的教训，实验三抄在 `real/标定说明.md` §6.5）。

**三个开关会让本次运行不算验收凭据**（`--dry-run` / `--assume-judge` / `--confirm-each`）：
它们分别意味着"没碰真机"、"没人确认夹住"、"人按回车才动作"，都够不上 docx §六 的
"正常任务过程中不进行人工类别判断和动作触发"。开了就会在 `real_run.json` 里记成
`evidence_grade=rehearsal`，验收时一查就知道这一份能不能用。

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
from sort_core.taxonomy import (REASON_ABORTED, REASON_NO_EXEC,  # noqa: E402
                                REASON_SAFETY_STOP)
from real import ep_conn                                    # noqa: E402
from real.camera import (MockDetector, YoloDetector, load_detector_file,  # noqa: E402
                         load_scene, make_scan, open_camera)
from real.dry_run import DryRunRM                           # noqa: E402
from real.ep_backend import EPPickExecutor, EPPickPlace      # noqa: E402

_CALIB_GUIDE = 'real/标定说明.md'


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
        print('标定步骤: %s；工具: python3 real/calibrate.py（本表读数直接写回，不用手抄）'
              % _CALIB_GUIDE)
        print('          相机标定见 config_real/task.json 的 camera.calibration 说明（乙）')
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
    ap.add_argument('--confirm-each', action='store_true',
                    help='★每个物体动手前等一次回车 —— 真机**首跑**人盯用；'
                         '它是"人工触发动作"，验收跑不能开')
    args = ap.parse_args(argv)

    # 三个开关都会让这次运行够不上验收凭据（docx §六：不进行人工类别判断和动作触发）：
    #   --dry-run      = 没碰真机；--assume-judge = 没人确认夹住；
    #   --confirm-each = 人按回车才动作。
    # 与其靠人记，不如算出来写进 real_run.json —— 验收时一查就知道能不能用。
    rehearsal = [n for n in ('dry_run', 'assume_judge', 'confirm_each')
                 if getattr(args, n.replace('-', '_'))]

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
    if args.confirm_each:
        runlog.text('*** 警告: --confirm-each 开着 —— 每个物体由人按回车才动手。'
                    '这是"人工触发动作"，**验收跑不能开**；只给真机首跑人盯用。')
    runlog.text('证据等级: ' + ('acceptance（可作验收凭据）' if not rehearsal else
                                'rehearsal（**不能**作验收凭据）—— 开了 %s'
                                % '+'.join(rehearsal)))

    camera = open_camera(args.camera, (task['camera']['image_width_px'],
                                       task['camera']['image_height_px']), runlog)
    detector = build_detector(args, grid, task, runlog)
    scan = make_scan(camera, detector, runlog)

    try:
        rm = DryRunRM(runlog) if args.dry_run else ep_conn.connect(ep_cfg, runlog)
    except ep_conn.EPConnectError as e:
        # "连都没连上"要能和"跑了失败"从文件上分开 —— 照实记一次没跑成的运行。
        runlog.text('== 连不上 EP，本次没有执行任何动作 ==')
        runlog.text(str(e))
        runlog.write_result(0, 0, task['expected_total'], task['pass_line'],
                            REASON_NO_EXEC, {})
        _write_real_context(run_dir, args, calib_rec,
                            dict(placed_ok=0, objects_seen=0, exit_status=REASON_NO_EXEC,
                                 reasons={}),
                            REASON_NO_EXEC, None, None, runlog, rehearsal,
                            outcome='connect_failed', connect_error=str(e))
        print('\nFAIL: 连不上 EP —— 本次没有执行任何动作（不是"跑了失败"）。')
        for line in str(e).splitlines():
            print('  %s' % line)
        print('run_dir: %s（result.json 里 placed_ok=0 / exit_status=%s；'
              'real_run.json 的 outcome=connect_failed）' % (run_dir, REASON_NO_EXEC))
        return 1

    ex = EPPickExecutor(rm, ep_cfg, log=runlog, ask=_ask_stdin)
    pick_place = EPPickPlace(ex, grid, bins, log=runlog)
    if hasattr(detector, 'mark_picked'):
        pick_place = _with_mark_picked(pick_place, detector, runlog)
    if args.confirm_each:
        pick_place = _with_confirm_each(pick_place, runlog)

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
    _write_real_context(run_dir, args, calib_rec, summary, exit_status, ex, rm, runlog,
                        rehearsal)

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


def _with_confirm_each(pick_place, log):
    """--confirm-each：每个物体动手**之前**等一次回车。

    这是把实验二的首跑纪律（`ep/标定说明.md` §4：`pause_each_phase=true`，每段回车、全程人盯）
    搬到实验三（`real/标定说明.md` §6 第 2 步）。它**只给真机首跑**用 —— 人按回车才动作，本身就是 docx §六 所说的
    "人工触发动作"，所以开了它这一轮就记成 rehearsal，不能当验收凭据。

    留在这里而不是塞进 sort_core：那是两条线共用的核心，不该为了调试多一个开关。
    """
    def wrapped(cell_id, cls):
        log.text('等待操作员确认后执行: %s / %s' % (cell_id, cls))
        print('\n下一个: %s 的 %s —— 看一眼桌面和车的位置（有没有跑偏/挡住/掉了东西）。'
              % (cell_id, cls))
        ans = input('回车执行，输入 q 中止 > ').strip().lower()
        if ans in ('q', 'quit', 'x'):
            raise KeyboardInterrupt('操作员中止')
        return pick_place(cell_id, cls)
    return wrapped


def _write_real_context(run_dir, args, calib_rec, summary, exit_status, ex, rm, runlog,
                        rehearsal, outcome='ran', connect_error=None):
    """把"这次是怎么跑出来的"单独留档：result.json 保持与仿真线同一 schema，
    真机特有的上下文(演练/标定记录/人工介入次数/证据等级)写这里，验收时才查得清。

    ex / rm 允许为 None —— 连不上 EP 时走的就是那条路（那时没有执行者，也没连上机器人）。
    """
    data = dict(
        outcome=outcome,                      # ran | connect_failed
        evidence_grade='rehearsal' if rehearsal else 'acceptance',
        rehearsal_reasons=list(rehearsal),
        dry_run=bool(args.dry_run),
        config_dir=os.path.abspath(args.config_dir),
        camera=args.camera, detector=args.detector,
        detector_source=(args.model or args.detector_file or 'builtin-mock'),
        assume_judge=bool(args.assume_judge),
        confirm_each=bool(args.confirm_each),
        judge=({'held': ex.judge.get('held'), 'placed': ex.judge.get('placed')}
               if ex else None),
        calibration=calib_rec,
        operator_confirms=(ex.n_asks if ex else None),
        exit_status=exit_status,
        summary=summary,
        run_dir=run_dir,
    )
    if connect_error:
        data['connect_error'] = connect_error
    if isinstance(rm, DryRunRM):
        data['dry_run_primitive_calls'] = len(rm.calls)
    with open(os.path.join(run_dir, 'real_run.json'), 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    runlog.text('real_run.json 已写（outcome=%s  evidence_grade=%s  人工确认 %s 次）'
                % (outcome, data['evidence_grade'],
                   ex.n_asks if ex else '-'))
    if isinstance(rm, DryRunRM):
        runlog.text(rm.dump())


if __name__ == '__main__':
    sys.exit(main())
