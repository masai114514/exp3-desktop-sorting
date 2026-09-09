# -*- coding: utf-8 -*-
"""离线 mock 后端：把 TaskController 的 scan / pick_place 缝隙在『真实 config』上跑通。

不连 ROS、不连 Gazebo。用途：
  * 任务控制线的离线验收 —— 控制循环 + 异常处理 + 日志 verdict 一把跑通；
  * 给甲仿真期 mock 识别节点、给 ROS2 PickPlace client 指一条「同口径」的参考实现：
      - scan()        ：物体所在网格中心 按 camera.calib 反算像素 → 合成 det（同 mock 节点做法）
      - pick_place()  ：把 (cell_id, cls) 兑现成『桌面物体移走』，可按 policy 注入失败
  * 只支持 rectilinear 标定（仿真正俯视）；homography 由甲场景真节点出，离线不做。

离线用法见 run_demo.py；单测见 tests/test_run_demo.py。
"""
from collections import Counter

from sort_core.geometry import cell_center

# 失败注入 policy：'ok' 全成功；'recovery' 每个 (cell,cls) 先失败 fail_first 次再成功；
# 'fail' 永远 grasp_failed。失败 reason 必须是 ACTION_REASONS 里的值（taxonomy）。
FAIL_REASON = 'grasp_failed'


def rectilinear_px(x_m, y_m, calib):
    """桌面(米) → 像素(rectilinear 正俯视反向标定)。"""
    cx, cy = calib['offset_px']
    sx, sy = calib['scale_px_per_m']
    return cx + sx * x_m, cy + sy * y_m


class OfflineTable:
    """一张放在网格上的『物体表』。pick 成功即把物体从桌面移除。

    objects : [(cell_id, cls), ...]  一张物体各占一格（可少于格数）
    extra_dets : [det, ...]          每轮额外注入的检测（造 unrecognized / out_of_grid）
    bbox_hw   : 合成 bbox 的半宽(px)，只影响观感；判定只用 bbox 中心
    """

    def __init__(self, cells, calib, objects, policy='ok', fail_first=1,
                 extra_dets=None, bbox_hw=6):
        self.cells = {c['id']: c for c in cells}
        self.calib = calib
        self.objects = list(objects)
        self.policy = policy
        self.fail_first = fail_first
        self.extra_dets = list(extra_dets or [])
        self.bbox_hw = bbox_hw
        self.attempts = Counter()   # (cell_id, cls) -> 尝试次数
        self.picked = []            # 已成功处理的 (cell_id, cls)

    # ---------- TaskController 缝隙：scan ----------
    def scan(self):
        dets = []
        for cid, cls in self.objects:
            xm, ym = cell_center(self.cells[cid])
            px, py = rectilinear_px(xm, ym, self.calib)
            hw = self.bbox_hw
            dets.append({'cls': cls, 'conf': 0.93,
                         'bbox': [px - hw, py - hw, px + hw, py + hw]})
        return dets + self.extra_dets

    # ---------- TaskController 缝隙：pick_place ----------
    def pick_place(self, cell_id, cls):
        key = (cell_id, cls)
        self.attempts[key] += 1
        n = self.attempts[key]

        if self.policy == 'fail':
            return FAIL_REASON, 'mock: always fail'
        if self.policy == 'recovery' and n <= self.fail_first:
            return FAIL_REASON, 'mock: attempt %d/%d fail' % (n, self.fail_first + 1)

        if key in self.objects:
            self.objects.remove(key)
            self.picked.append(key)
            return 'ok', 'mock: removed %s from %s' % (cls, cell_id)
        return 'exec_error', 'mock: %s not on table (double exec?)' % (cell_id,)
