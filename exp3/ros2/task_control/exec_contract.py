# -*- coding: utf-8 -*-
"""PickPlace 执行契约（**ROS-free**）：PickExecutor ABC + goal→A/B 解算。

放成非 ROS 模块，是为了让两条线都能用：仿真执行者 C4PickExecutor、**真机执行者
EPPickExecutor**（`real/ep_backend.py`，EP 不走 ROS2）和离线单测，都从**同一处**取契约，
保证不会分叉 —— 所以它在 `ros2/` 目录下，却不 import 任何 ROS。

ctx 里的 A/B 是**桌面系**坐标（相机标定那条线的坐标系）。真机执行者只取 ctx 的
`cell_id`/`bin_id`，拿 id 去 `config_real/ep_waypoints.json` 查底盘里程计位姿 ——
两个坐标系之间没有已知关系也不需要建立，见 `real/ep_backend.py` 的设计决定 1。

方法签名统一是 (self, ctx)，ctx = resolve_goal 的返回：
    {cell, cell_id, cls, bin_id, slot, slot_index, bin_yaw_rad,
     A: {x, y, z_pick},          # 格中心 + 取物安全高(grid_cells.json pick_z_m)
     B: {x, y, z_drop}}          # 料盒**该落料点**的坐标(bins.json x_m/y_m + slots 偏移)
                                 # z_drop = 分层高度，执行者把它加到落料高上（叠放）
    bin_yaw_rad                  # 料盒侧动作的工具偏航（bins.json yaw_deg 的弧度值）。
                                 # 顶抓绕竖直轴自转不改变夹持，但能把腕关节从
                                 # "够不着 +x 半边"的构型里解放出来（bin_reach_levers
                                 # --lever B 实测）。取物侧不用它；真机线忽略它。
ctx 里不含关节级参数 —— 那些是执行者/运动配置(motion)的事，见 C4PickExecutor。
"""
import math

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


def slot_offset(bin_cfg, slot):
    """取该料盒第 slot 个落料点的偏移量 {dx,dy,dz}；无 slots 配置时返回零偏移。

    slot 超出已有落料点数量时**钳到最后一个**而不是环绕：环绕会让第 5 个物块又落回
    第 1 个的位置上（重叠 → LCP 爆解），钳住至少是"叠在最后一摞上"。
    """
    slots = bin_cfg.get('slots') or []
    if not slots:
        return {}
    i = min(max(int(slot), 0), len(slots) - 1)
    return slots[i]


def resolve_goal(grid, bins, cell_id, cls, slot=0):
    """校验 goal 并解算 A/B 目标 → (ctx, None) 或 (None, err_msg)。

    语义（对齐 contract/PickPlace.action）：goal 只给 cell_id+cls，不写坐标；
    取放点从 grid_cells.json、料盒从 bins.json 现查 —— 换格/换料盒不改接口。

    slot：**同一料盒内第几个落料点**，由调用方按"该料盒已成功放入几个"给出。
    为什么需要它：6 个物块只分 2 类 ⇒ 必然有一个料盒要放 3~4 个；若都瞄准料盒中心，
    第 2 个物块**下降途中**就会穿进第 1 个体内，ODE 的 LCP 爆解并把已放好的那个
    炸飞（实测 run_20260924_005503 c1 被抛到 (-75.2, 47.7) m）。所以落料点必须错开。
    越界处理：slot 超出 slots 数量时钳到最后一个（不退化成"重叠落料"）。
    bin_id 未配 slots 时 slot 无效（向后兼容：真机线与旧配置行为不变）。
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
    s = slot_offset(b, slot)
    ctx = dict(cell=cell, cell_id=cell_id, cls=cls, bin_id=bin_id,
               slot=slot,
               slot_index=min(max(int(slot), 0), len(b.get('slots') or [{}]) - 1),
               # 料盒侧动作的工具偏航（绕竖直轴自转，不改变顶抓夹持）。见模块 docstring。
               bin_yaw_rad=math.radians(float(b.get('yaw_deg', 0.0) or 0.0)),
               A=dict(x=xm, y=ym, z_pick=grid.get('pick_z_m', 0.0)),
               # B.z_drop = 该落料点相对料盒底的分层高度（叠放用；第 1 层为 0）
               B=dict(x=b.get('x_m', 0.0) + s.get('dx', 0.0),
                      y=b.get('y_m', 0.0) + s.get('dy', 0.0),
                      z_drop=s.get('dz', 0.0)))
    return ctx, None
