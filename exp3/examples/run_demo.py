#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验3 任务控制线 —— 离线端到端 demo（真实 config，产出 exp3_logs）。

对每轮从 config/task.json 读的『任务』，用 mock 桌面/后端把 TaskController 完整跑一遍，
结果按 c4_2 口径落 exp3_logs/run_<时间戳>/（records.jsonl + run.log + result.json）。

场景（--mode）：
  happy        6 物全成功分类放置            → PASS (exit=no_target)
  recovery    每物先 1 次抓取失败再成功       → PASS，reasons 里有 grasp_failed（展示容错）
  safety_stop 同一物连续抓取失败到 fail_limit → safety_stop 安全停止
  unrecognized 桌面全是未知类                 → no_exec（跳过项按 (cell,cls) 只记一次）
  out_of_grid  检测中心都在网格外             → no_exec

用法（在 exp3/ 目录下）：
  python3 examples/run_demo.py --mode happy [--log-dir 覆盖默认 exp3_logs]
  python3 -m unittest discover -s tests -t . -v     # 全部离线测试

跑通后，甲在 Gazebo 里把 scan/pick_place 换成真的（mock 识别节点 / PickPlace action client），
TaskController 本体不用改 —— 这就是本 demo 想证明的缝隙。
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))          # exp3/（让 sort_core 可 import）

from sort_core.config import load_all                # noqa: E402
from sort_core.task import TaskController            # noqa: E402
from sort_core.logging_util import Exp3RunLog, make_run_dir  # noqa: E402
from examples.mock_backend import OfflineTable       # noqa: E402

SIX = [('c1', 'cup'), ('c2', 'mouse'), ('c3', 'cup'),
       ('c4', 'mouse'), ('c5', 'cup'), ('c6', 'mouse')]

# out_of_grid：把物体挪到网格外但仍在画面内（如 x=0.30m 处，超出 c1..c6 的 x<=0.19）
_FAR_PX = [((320 + 1000 * 0.30), 240), ((320 + 1000 * 0.30), 260), ((200, 240))]


def build_scenario(mode):
    """→ dict(objects, policy, fail_first, extra_dets)。objects/extra 只喂 mock，控制器无感。"""
    if mode == 'happy':
        return dict(objects=SIX, policy='ok', fail_first=0, extra_dets=None)
    if mode == 'recovery':
        return dict(objects=SIX, policy='recovery', fail_first=1, extra_dets=None)
    if mode == 'safety_stop':
        return dict(objects=[('c1', 'cup')], policy='fail', fail_first=0, extra_dets=None)
    if mode == 'unrecognized':
        objs = [(c, 'stapler') for c in ('c1', 'c2', 'c3', 'c4', 'c5', 'c6')]
        return dict(objects=objs, policy='ok', fail_first=0, extra_dets=None)
    if mode == 'out_of_grid':
        extras = [{'cls': 'mouse', 'conf': 0.9,
                   'bbox': [x - 6, y - 6, x + 6, y + 6]} for (x, y) in _FAR_PX]
        return dict(objects=[], policy='ok', fail_first=0, extra_dets=extras)
    raise SystemExit('未知 mode: %r（happy|recovery|safety_stop|unrecognized|out_of_grid）' % mode)


def run_once(mode, log_dir=''):
    """跑一遍真实 config 场景。返回 (summary, verdict)。verdict 见 taxonomy.VERDICT_*。"""
    cfg_all = load_all()
    task, bins, grid = cfg_all['task'], cfg_all['bins'], cfg_all['grid']
    cells = grid['cells']
    if task['camera']['calib'].get('mode', 'rectilinear') != 'rectilinear':
        raise SystemExit('离线 demo 只支持 rectilinear 标定（sim 正俯视）；'
                         'homography 由甲场景真节点出')

    run_dir = make_run_dir(log_dir or task.get('log_dir', ''))
    runlog = Exp3RunLog(run_dir)
    runlog.text('mode=%s run_dir=%s' % (mode, run_dir))

    sc = build_scenario(mode)
    table = OfflineTable(cells, task['camera']['calib'], sc['objects'],
                         policy=sc['policy'], fail_first=sc['fail_first'],
                         extra_dets=sc['extra_dets'])
    ctl = TaskController(task, bins, cells, table.scan, table.pick_place, log=runlog)
    summary = ctl.run()

    verdict = runlog.write_result(summary['placed_ok'], summary['objects_seen'],
                                  task['expected_total'], task['pass_line'],
                                  summary['exit_status'], summary['reasons'])
    runlog.text('summary=%s' % summary)
    return summary, verdict


def main():
    ap = argparse.ArgumentParser(description='实验3 任务控制离线 demo（真实 config）')
    ap.add_argument('--mode', default='happy',
                    choices=['happy', 'recovery', 'safety_stop', 'unrecognized', 'out_of_grid'])
    ap.add_argument('--log-dir', default='', help='覆盖默认日志目录')
    args = ap.parse_args()

    summary, verdict = run_once(args.mode, args.log_dir)
    print('verdict = %s' % verdict)
    print('placed_ok=%d  objects_seen=%d  exit_status=%s  reasons=%s'
          % (summary['placed_ok'], summary['objects_seen'],
             summary['exit_status'], summary['reasons']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
