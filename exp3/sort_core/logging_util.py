# -*- coding: utf-8 -*-
"""exp3 运行日志：每物体 records.jsonl + run.log + 汇总 result.json。

verdict 口径复刻 c4_2：只有「>=required_total 个物体走完的完整验收轮」才下 PASS/FAIL，
不足(或试跑)标 TRIAL，进行中标 RUNNING。落地目录默认 ./exp3_logs/run_<时间戳>/。
"""
import json
import os
import time
from datetime import datetime

from .taxonomy import (VERDICT_PASS, VERDICT_FAIL, VERDICT_TRIAL, VERDICT_RUNNING)


def make_run_dir(log_dir):
    base = os.path.abspath(log_dir) if log_dir else os.path.join(os.getcwd(), 'exp3_logs')
    os.makedirs(base, exist_ok=True)
    run_dir = os.path.join(base, 'run_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


def verdict_for(placed_ok, objects_seen, required_total, pass_line, final):
    """运行级结论：final=False → RUNNING；final 且走完 → PASS/FAIL；final 但物体数不足 → TRIAL。"""
    if not final:
        return VERDICT_RUNNING
    if objects_seen >= required_total:
        return VERDICT_PASS if placed_ok >= pass_line else VERDICT_FAIL
    return VERDICT_TRIAL


class Exp3RunLog:
    """run.log(事件流) + records.jsonl(每物体 1 条) + result.json(汇总)。"""

    def __init__(self, run_dir):
        self.dir = run_dir
        self.text_path = os.path.join(run_dir, 'run.log')
        self.records_path = os.path.join(run_dir, 'records.jsonl')
        self.result_path = os.path.join(run_dir, 'result.json')
        self.records = []
        self._t0 = time.monotonic()

    def text(self, msg):
        line = '[{:7.2f}] {}'.format(time.monotonic() - self._t0, msg)
        print(line, flush=True)
        with open(self.text_path, 'a', encoding='utf-8') as f:
            f.write(line + '\n')

    def record(self, rec):
        """rec: dict，见 config/log_schema.md。写 records.jsonl 并缓存用于汇总。"""
        rec = dict(rec)
        rec.setdefault('ts', datetime.now().isoformat(timespec='seconds'))
        self.records.append(rec)
        with open(self.records_path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    def write_result(self, placed_ok, objects_seen, required_total, pass_line,
                     exit_status, reasons, final=True):
        verdict = verdict_for(placed_ok, objects_seen, required_total, pass_line, final)
        data = dict(placed_ok=placed_ok,
                    objects_seen=objects_seen,
                    required_total=required_total,
                    pass_line=pass_line,
                    verdict=verdict,
                    exit_status=exit_status,
                    reasons=reasons,
                    run_dir=self.dir)
        with open(self.result_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return verdict
