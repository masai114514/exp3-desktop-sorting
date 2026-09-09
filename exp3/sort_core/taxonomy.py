# -*- coding: utf-8 -*-
"""reason / verdict 常量。reason 复刻 c4_2(grasp_failed 等) 并加实验3 任务级原因。"""
# 动作级失败原因（对齐 c4_2 枚举；动作=一次 PickPlace）
REASON_GRASP_FAILED = 'grasp_failed'                # 没夹起
REASON_DROPPED = 'dropped_or_missed_place'          # 中途掉/放偏
REASON_UNREACHABLE = 'unreachable_target'           # 不可达/逆解失败/路径规划失败
REASON_EXEC_ERROR = 'exec_error'                    # ROS/规划/驱动硬错
REASON_ABORTED = 'aborted'                          # 被中断

# 跳过/终止类原因（任务级）
REASON_OUT_OF_GRID = 'out_of_grid'                  # 检测框中心不在任何网格
REASON_UNRECOGNIZED = 'unrecognized_object'         # 未知类别 或 置信度 < conf_min
REASON_NO_TARGET = 'no_target'                      # 桌面已无目标(全部处理完)，正常结束
REASON_NO_EXEC = 'no_exec'                          # 只剩不可执行目标(空转后结束)
REASON_SAFETY_STOP = 'safety_stop'                  # 连续失败/保护触发，安全停止

ACTION_REASONS = frozenset({
    REASON_GRASP_FAILED, REASON_DROPPED, REASON_UNREACHABLE,
    REASON_EXEC_ERROR, REASON_ABORTED,
})
SKIP_REASONS = frozenset({REASON_OUT_OF_GRID, REASON_UNRECOGNIZED})
EXIT_REASONS = frozenset({REASON_NO_TARGET, REASON_NO_EXEC, REASON_SAFETY_STOP})

# 运行级结论（复刻 c4_2：只有完整验收轮才下 PASS/FAIL，试跑=TRIAL，进行中=RUNNING）
VERDICT_PASS = 'PASS'
VERDICT_FAIL = 'FAIL'
VERDICT_TRIAL = 'TRIAL'
VERDICT_RUNNING = 'RUNNING'

# PickPlace feedback 阶段号（.action 不支持常量；对齐 exp3/ros2/sort_msgs/README.md 阶段表。
# 权威注释见 exp3/contract/PickPlace.action —— 服务端每进入一段就发一次 feedback）
STAGE_APPROACH = 1      # 运动到 cell 上方/下探
STAGE_GRASP = 2         # 下降 + 夹取(压紧)
STAGE_LIFT = 3          # 抬起(+判 held)
STAGE_MOVE_TO_BIN = 4   # 移到料盒上方(搬运/斜线落位)
STAGE_PLACE = 5         # 下降 + 松爪 + 判放置
STAGE_RETRACT = 6       # 退回安全位/回零
STAGE_FAILING = -1      # 失败正在收尾(随后回 result)

STAGE_NAMES = {
    STAGE_APPROACH: 'approach', STAGE_GRASP: 'grasp', STAGE_LIFT: 'lift',
    STAGE_MOVE_TO_BIN: 'move_to_bin', STAGE_PLACE: 'place',
    STAGE_RETRACT: 'retract', STAGE_FAILING: 'failing',
}
