#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 真机日志(与 c4_2 验收口径一致): run.log + events.jsonl + trajectory.csv + result.json。

result.json schema 与 c4_2/drive/logging_util.py 完全同构(ok_count/total/pass_line=4/
cycles/final), reason 枚举沿用:
  grasp_failed / dropped_or_missed_place / exec_error / aborted
→ 验收记录表可直接按同一列填。轨迹行与 EP 运动对应(底盘 odom + 臂 x/y + 夹爪状态)。
默认日志目录 ./ep_logs(避免与 c4_2_logs 混)。
"""
import json
import os
import time
from datetime import datetime

REASONS = ('grasp_failed', 'dropped_or_missed_place', 'exec_error', 'aborted')


def default_run_dir(base='ep_logs'):
    d = os.path.abspath(base)
    os.makedirs(d, exist_ok=True)
    run_dir = os.path.join(d, 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


class EPLog:
    def __init__(self, run_dir):
        self.dir = run_dir
        self.text_path = os.path.join(run_dir, 'run.log')
        self.events_path = os.path.join(run_dir, 'events.jsonl')
        self.traj_path = os.path.join(run_dir, 'trajectory.csv')
        self.result_path = os.path.join(run_dir, 'result.json')
        self.err_path = os.path.join(run_dir, 'error.txt')
        self._tf = open(self.traj_path, 'w', encoding='utf-8')
        self._tf.write('t_motion,phase,chassis_x,chassis_y,chassis_z_deg,'
                       'arm_x_mm,arm_y_mm,gripper_state\n')
        self._tf.flush()
        self.results = []
        self._t0 = time.monotonic()

    def text(self, msg):
        line = '[{:7.2f}] {}'.format(time.monotonic() - self._t0, msg)
        print(line, flush=True)
        with open(self.text_path, 'a', encoding='utf-8') as f:
            f.write(line + '\n')

    def event(self, kind, **kw):
        rec = dict(kind=kind, t_motion=round(time.monotonic() - self._t0, 3))
        rec.update(kw)
        with open(self.events_path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    def sample(self, phase, chassis_pose, arm_xy, gripper_state):
        row = '{:.3f},{},{},{},{},{},{},{}\n'.format(
            time.monotonic() - self._t0, phase,
            *['%.4f' % v for v in chassis_pose],
            int(round(arm_xy[0])), int(round(arm_xy[1])), gripper_state)
        self._tf.write(row)
        self._tf.flush()

    def write_exception(self, exc):
        import traceback
        with open(self.err_path, 'w', encoding='utf-8') as f:
            f.write(traceback.format_exc())
        self.text('异常已写 {}'.format(self.err_path))

    def add_cycle_result(self, index, ok, reason, detail, t_cycle):
        assert reason in REASONS, reason
        self.results.append(dict(cycle=index, ok=bool(ok), reason=reason,
                                 detail=detail, t_cycle_s=round(t_cycle, 2)))
        self.write_result()

    def write_result(self, final=False):
        data = dict(ok_count=sum(1 for r in self.results if r['ok']),
                    total=len(self.results),
                    pass_line=4,
                    cycles=self.results,
                    final=final)
        with open(self.result_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def close(self):
        self._tf.close()
