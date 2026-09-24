# -*- coding: utf-8 -*-
"""仿真场景的物块摆放表 —— **无 ROS import**，因此可离线单测。

为什么不放在 pick_place_server.py 里：那个模块 import rclpy，离线套件跑不了。
本项目的纪律是「先让逻辑离线绿，再接线」，所以凡是能被离线验证的**判据**（而不是
调用），都要能在没有 ROS 的机器上跑。摆放表 + 异常轮的空格剔除就是这样一个判据。

坐标口径（与 c4_2/config_sim.json、geometry.py 的标定一致）：
  - 桌面顶面 world z = 0.80，即 base_z；物块中心 = base_z + object_z。
  - cup  圆柱 r=0.020 l=0.035 ⇒ 半高 0.0175（中心相对桌面 0.0175，顶面 0.8345）
  - mouse 盒 0.040x0.025x0.020 ⇒ 半高 0.0100（中心相对桌面 0.0100）

★ 这两组数字与 pick_place_server._object_sdf() 里的几何**必须同步**：
object_z 就是半高，改了几何没改这里会让物块悬空或陷进桌面。
"""

CELL_IDS = ('c1', 'c2', 'c3', 'c4', 'c5', 'c6')

# (model_name, cls, x, y, object_z)  —— object_z 是物块中心相对桌面顶面的高度
SCENE_OBJECTS = (
    ('block_c1', 'cup', 0.09, -0.08, 0.0175),
    ('block_c2', 'mouse', 0.09, 0.00, 0.0100),
    ('block_c3', 'cup', 0.09, 0.08, 0.0175),
    ('block_c4', 'cup', 0.14, -0.08, 0.0175),
    ('block_c5', 'mouse', 0.14, 0.00, 0.0100),
    ('block_c6', 'cup', 0.14, 0.08, 0.0175),
)

# 各格标称类别（供离线用例核对"摆放表 vs 期望类别表"是否一致）
NOMINAL_CLS = {name.split('_')[1]: cls for name, cls, _x, _y, _z in SCENE_OBJECTS}

# ---------------------------------------------------------------------------
# 料盒上方"横移高度" carry_z 的实测边界与安全取值
#
# carry_z 绝对高度 = grasp_z + lift_dz * fraction。约束来自**三条腿**，取最小值：
#   下界：物块底面必须越过沿途其它物块的顶面 —— 最高的是 cup（顶面 0.8345）。
#   上界：go_cartesian 走 plan_segment，全程锁 cartesian_rpy=[0,0,0]，
#        z 越高 IK 越是解到 LIMITS[4]=2.0071 之外。
#
# ★ 三条腿的上界（carry_z_probe.py + retract_probe.py 离线细扫，step=0.006）：
#
#     格           目标料盒      搬运腿    竖起腿    回程腿 RISE_AWAY    综合
#     c1 c3 c4 c6  bin_cup       0.096     0.096     0.096              0.096
#     c2           bin_mouse     0.112     0.096     0.096              0.096
#     c5           bin_mouse     0.112     0.112     0.096              0.096
#
#   ⇒ **综合上限 0.096，6/6 格一致，且由【回程腿】决定**，不是由"能抬多高"决定。
#     mouse 那两格搬运腿能到 0.112，但抬上去就回不来。
#
# ⚠ 两次被实测否决的改法，别再写回去：
#   ① 倍率整体抬到 (0.87, 0.74) ⇒ 绝对 (0.125, 0.112)：0.125 对 6/6 格不可达。
#   ② 保留 0.112 一档（哪怕是作为最高候选）：第 6 轮云机实跑 run_20260924_004129
#      证明这是**脆的** —— 瞄准 IK 边界值，臂的跟踪过冲（实测到位 0.113 而非 0.112）
#      会把 q0 推到可行域之外，于是【下一步规划从非法起点出发而必然失败】：
#        round2 c2 放置成功 → 回程 [RISE_AWAY] 规划失败 LIMIT q5=2.176
#        round3 c2 抬到 0.113 → [CARRY_TO_B] 规划失败 LIMIT q5=2.369
#      即：**在边界上取点，成功是偶然，失败是必然。**
#
# ⇒ 因此目标值必须离边界留出余量。CARRY_Z_IK_CEILING 是实测边界，
#   CARRY_Z_SAFETY_MARGIN 是余量，候选值由「倍率」与「边界−余量」取小。
#   0.096 也在边界上，一并弃用；0.088 是第 4 轮云机实跑验证过的取值。
CARRY_Z_IK_CEILING = {'bin_cup': 0.096, 'bin_mouse': 0.112}
CARRY_Z_SAFETY_MARGIN = 0.008
CARRY_Z_FRACTIONS = (0.50, 0.35)          # → 0.088 / 0.073
CARRY_Z_DEFAULT = 0.088                    # 第一档（= 0.50 倍率），也是 0.096−0.008


def carry_z_candidates(grasp_z, lift_dz, bin_id=None):
    """由 grasp_z / lift_dz 算出横移高度候选（**从高到低**，第一个可达的胜出）。

    bin_id 给出时按该料盒的实测边界减去安全余量封顶；不给则用默认第一档。
    """
    ceiling = CARRY_Z_IK_CEILING.get(bin_id) if bin_id else None
    hi = (ceiling - CARRY_Z_SAFETY_MARGIN) if ceiling is not None else None
    out = [grasp_z + lift_dz * f for f in CARRY_Z_FRACTIONS]
    if hi is not None:
        out = [min(z, hi) for z in out]
    return out


def scene_objects(empty_cell=''):
    """返回本轮要摆放的物块列表（元组的列表）；empty_cell 指定的格被剔除。

    ★ 异常轮为什么必须【真的空一格】，而不是改配置假装：
      任务书 §六 要求"空网格应被跳过"，这一条要求的**证据形式与其它条不同** ——
      不是某一行写了什么，而是那一格【根本没有产生任何记录】
      （`records.jsonl` 里没有该 cell_id 的行，`result.json` 的 `reasons` 里也没有
      对应键）。一个"把空格也当目标去抓"的系统才会在这里空抓撞桌。
      所以空格必须是场景事实，不能是配置断言。

    ★ 异常轮的 verdict 必然是 TRIAL，这是正确语义不是失败：
      `sort_core/logging_util.verdict_for` 只给 `objects_seen >= required_total`
      的完整轮下 PASS/FAIL；场上 5 个物块 ⇒ `seen_cells=5 < 6` ⇒ TRIAL。
      因此异常轮**必须与记分轮分开跑、分开记**，不能合并成一轮。

    参数：
      empty_cell: '' 或 None = 正常 6 物记分轮；'c1'..'c6' = 该格故意不放。
    """
    skip = str(empty_cell or '').strip()
    if skip and skip not in CELL_IDS:
        raise ValueError('empty_cell must be one of %s, got %r'
                         % (', '.join(CELL_IDS), empty_cell))
    return [item for item in SCENE_OBJECTS if item[0].split('_')[1] != skip]
