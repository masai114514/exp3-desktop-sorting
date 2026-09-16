# -*- coding: utf-8 -*-
"""PickPlace 执行契约（**ROS-free**）：PickExecutor ABC + goal→A/B 解算。

放成非 ROS 模块，是为了让两条线都能用：仿真执行者 C4PickExecutor、**真机执行者
EPPickExecutor**（`real/ep_backend.py`，EP 不走 ROS2）和离线单测，都从**同一处**取契约，
保证不会分叉 —— 所以它在 `ros2/` 目录下，却不 import 任何 ROS。

ctx 里的 A/B 是**桌面系**坐标（相机标定那条线的坐标系）。真机执行者只取 ctx 的
`cell_id`/`bin_id`，拿 id 去 `config_real/ep_waypoints.json` 查底盘里程计位姿 ——
两个坐标系之间没有已知关系也不需要建立，见 `real/ep_backend.py` 的设计决定 1。

方法签名统一是 (self, ctx)，ctx = resolve_goal 的返回：
    {cell, cell_id, cls, bin_id,
     A: {x, y, z_pick},          # 格中心 + 取物安全高(grid_cells.json pick_z_m)
     B: {x, y}}                  # 料盒中心(bins.json)
ctx 里不含关节级参数 —— 那些是执行者/运动配置(motion)的事，见 C4PickExecutor。
"""
from sort_core.geometry import cell_center
from sort_core.decision import bin_for_cls


class PickExecutor:
    """一次 PickPlace 的机械执行者接口；每个方法返回 (ok: bool, note: str)。

    阶段 → 方法：
      STAGE_APPROACH  → descend(ctx)   到格上方 + 下探到抓取高
      STAGE_GRASP     → grasp(ctx)     压紧夹爪
      STAGE_LIFT      → lift(ctx)      抬离并判 held（没夹起 → 回 False → grasp_failed）
      STAGE_MOVE_TO_BIN→ carry(ctx)    搬到料盒放置位
      STAGE_PLACE     → release(ctx)   张开放置并判 placed（掉/放偏 → 回 False）
      STAGE_RETRACT   → retract(ctx)   退回安全位/回零
    方法内 throw 异常 = 硬错（server 兜底成 exec_error）。调用顺序由 server 保证。
    """

    def descend(self, ctx):
        raise NotImplementedError

    def grasp(self, ctx):
        raise NotImplementedError

    def lift(self, ctx):
        raise NotImplementedError

    def carry(self, ctx):
        raise NotImplementedError

    def release(self, ctx):
        raise NotImplementedError

    def retract(self, ctx):
        raise NotImplementedError


def resolve_goal(grid, bins, cell_id, cls):
    """校验 goal 并解算 A/B 目标 → (ctx, None) 或 (None, err_msg)。

    语义（对齐 contract/PickPlace.action）：goal 只给 cell_id+cls，不写坐标；
    取放点从 grid_cells.json、料盒从 bins.json 现查 —— 换格/换料盒不改接口。
    """
    cells = {c['id']: c for c in grid['cells']}
    if cell_id not in cells:
        return None, '未知 cell_id=%r' % cell_id
    bin_id = bin_for_cls(cls, bins)
    if not bin_id or bin_id not in bins.get('bins', {}):
        return None, '未知类别/无对应料盒: cls=%r' % cls
    cell = cells[cell_id]
    xm, ym = cell_center(cell)
    b = bins['bins'][bin_id]
    ctx = dict(cell=cell, cell_id=cell_id, cls=cls, bin_id=bin_id,
               A=dict(x=xm, y=ym, z_pick=grid.get('pick_z_m', 0.0)),
               B=dict(x=b.get('x_m', 0.0), y=b.get('y_m', 0.0)))
    return ctx, None
