# -*- coding: utf-8 -*-
"""任务状态机（实验3 任务控制，纯逻辑可离线测）。

循环规则（对齐 docx §三.4/5/6）：
  扫描俯视识别 → 逐轮选 1 个可执行目标 → PickPlace → 直到：
    - 视野空(全部处理完)              → exit_status=no_target（正常结束）
    - 只剩不可执行目标(空转超过上限)   → exit_status=no_exec（异常已记录）
    - 连续失败超过 fail_limit          → exit_status=safety_stop（安全停止，reason 落盘）

识别侧语义：scan() 返回原始检测(cls/conf/bbox)；定位/分类在本控制器内完成，
因此单元测试可喂合成检测；ROS 侧只需把 Detection2DArray 转成这个 dict 列表。
pick_place 语义：返回 ('ok', note) 表示该物体已成功放入料盒并从桌面消失；
返回任一 ACTION_REASON 表示失败（桌面物体仍在，后续轮会重扫/重试/止损）。
"""
from collections import Counter

from .geometry import locate
from .decision import classify, sort_by_cell
from .taxonomy import (ACTION_REASONS, REASON_NO_TARGET, REASON_NO_EXEC,
                       REASON_SAFETY_STOP, REASON_UNRECOGNIZED, REASON_OUT_OF_GRID)

OK = 'ok'


class TaskController:
    def __init__(self, task_cfg, bins_cfg, cells, scan, pick_place,
                 safe_stop=None, log=None):
        self.cfg = task_cfg
        self.bins = bins_cfg
        self.cells = cells
        self.scan = scan                # () -> [ {cls, conf, bbox:[x1,y1,x2,y2]} ]
        self.pick_place = pick_place    # (cell_id, cls) -> ('ok', note) | (reason, note)
        self.safe_stop = safe_stop      # (reason) -> None，可选
        self.log = log
        cam = task_cfg['camera']
        self.image_size = (cam['image_width_px'], cam['image_height_px'])
        self.calib = cam['calib']

    # ---- 内部工具 -------------------------------------------------
    def _classify_det(self, det):
        lr = locate(det['bbox'], self.calib, self.cells, self.image_size)
        return classify(lr, det.get('cls'), det.get('conf', 0.0),
                        self.cfg['conf_min'], self.bins)

    def _record(self, **kw):
        if self.log:
            self.log.record(kw)

    # ---- 主循环 ---------------------------------------------------
    def run(self):
        cfg = self.cfg
        reasons = Counter()
        placed_ok = 0
        done_cells = set()          # 已成功处理完的网格，不再二次取放
        logged_skip = set()         # 已记录过的跳过项：(cell_id, cls)
        fail_streak = 0
        no_exec_streak = 0
        exit_status = None
        rounds = 0

        while rounds < cfg['max_rounds']:
            rounds += 1
            dets = self.scan() or []

            if not dets:            # 视野已空 → 全部处理完
                exit_status = REASON_NO_TARGET
                break

            executable = []
            for det in dets:
                dec = self._classify_det(det)
                cell = dec['cell']
                key = (cell['id'] if cell else None, dec['cls'])
                if not dec['executable']:
                    if key not in logged_skip:
                        logged_skip.add(key)
                        reasons[dec['reason']] += 1
                        self._record(decision='skip', cell_id=(cell['id'] if cell else None),
                                     cls=dec['cls'], conf=dec['conf'], skip_reason=dec['reason'])
                    continue
                if cell['id'] in done_cells:
                    continue
                executable.append(dec)

            if executable:
                target = sort_by_cell(executable)[0]
                cell_id = target['cell']['id']
                res, note = self.pick_place(cell_id, target['cls'])
                if res == OK:
                    placed_ok += 1
                    done_cells.add(cell_id)
                    fail_streak = 0
                    no_exec_streak = 0
                    self._record(decision='exec', cell_id=cell_id, cls=target['cls'],
                                 conf=target['conf'], grasp_ok=True, place_ok=True,
                                 reason=None, detail=note)
                else:
                    assert res in ACTION_REASONS, 'pick_place 返回非法 reason: %r' % res
                    fail_streak += 1
                    reasons[res] += 1
                    self._record(decision='exec', cell_id=cell_id, cls=target['cls'],
                                 conf=target['conf'], grasp_ok=False, place_ok=False,
                                 reason=res, detail=note)
                    if fail_streak >= cfg['fail_limit']:
                        exit_status = REASON_SAFETY_STOP
                        reasons[REASON_SAFETY_STOP] += 1
                        if self.safe_stop:
                            self.safe_stop(REASON_SAFETY_STOP)
                        break
            else:
                # 有检测但没有可执行的（全是跳过项或已在 done_cells）→ 空转
                no_exec_streak += 1
                if no_exec_streak >= cfg['no_exec_limit']:
                    exit_status = REASON_NO_EXEC
                    reasons[REASON_NO_EXEC] += 1
                    break

        if exit_status is None:         # max_rounds 兜底
            exit_status = REASON_NO_EXEC
            reasons[REASON_NO_EXEC] += 1

        # 结束时再扫一次，估计仍留在桌上的物体数（用于 objects_seen 口径）
        residual = len(self.scan() or [])

        summary = {
            'placed_ok': placed_ok,
            'objects_seen': placed_ok + residual,
            'exit_status': exit_status,
            'reasons': dict(reasons),
        }
        if self.log:
            self.log.text('exit_status=%s placed_ok=%d residual=%d reasons=%s'
                          % (exit_status, placed_ok, residual, dict(reasons)))
        return summary
