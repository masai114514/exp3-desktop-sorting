# -*- coding: utf-8 -*-
"""类别决策：class → 料盒；单条检测 → 可执行目标 or 跳过(带 reason)。纯函数，离线可测。"""
from .taxonomy import REASON_UNRECOGNIZED, REASON_OUT_OF_GRID


def bin_for_cls(cls_name, bins_cfg):
    """类别 → 料盒 id；未知类别返回 None。bins_cfg = config/bins.json 的 class_to_bin。"""
    return bins_cfg.get('class_to_bin', {}).get(cls_name)


def classify(locate_result, cls_name, conf, conf_min, bins_cfg):
    """把『单条检测 + 定位结果』分类为可执行目标或跳过项。

    locate_result: geometry.locate 的返回。规则：
      - 类别不在映射表 / 置信度低于 conf_min → unrecognized_object（跳过；cell 保留作记录）
      - 定位未命中任何网格(含出画面)         → out_of_grid（跳过；cell=None）
      - 否则可执行，cell 保留几何供取放。

    返回 dict：{executable, cell, cls, conf, reason}，reason 仅跳过/失败时非空。
    """
    cell = locate_result['cell']
    if cls_name not in bins_cfg.get('class_to_bin', {}):
        return {'executable': False, 'cell': cell,
                'reason': REASON_UNRECOGNIZED, 'cls': cls_name, 'conf': conf}
    if conf < conf_min:
        return {'executable': False, 'cell': cell,
                'reason': REASON_UNRECOGNIZED, 'cls': cls_name, 'conf': conf}
    if cell is None:
        return {'executable': False, 'cell': None,
                'reason': REASON_OUT_OF_GRID, 'cls': cls_name, 'conf': conf}
    return {'executable': True, 'cell': cell,
            'reason': None, 'cls': cls_name, 'conf': conf}


def sort_by_cell(candidates):
    """可执行目标按网格 (row, col) 排序，保证处理顺序确定。"""
    return sorted(candidates, key=lambda c: (c['cell']['row'], c['cell']['col']))
