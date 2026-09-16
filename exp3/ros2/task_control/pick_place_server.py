#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PickPlace action server —— 实验3 抓取接口的『机器人无关』时序骨架。

契约见 contract/PickPlace.action 与 sort_msgs（goal=cell_id+cls；feedback=stage 1..6；
result=success+reason+detail；reason 对齐 sort_core/taxonomy.py 动作级枚举）。

设计：把『一次取放』拆成固定 6 个阶段，每进入一段先发 feedback 再交给 **PickExecutor**
（exec_contract.py，ROS-free）。A/B 已由 resolve_goal 按 config 解算好放 ctx。
这是**仿真线**的 server（真机线是 EP，不走 ROS2，见 real/ep_backend.py）。
仿真执行者实现见 c4_executor.py（用 c4_2 直驱原语）——本文件不含任何关节/规划代码，
可先审时序与 reason 映射；--executor stub 做离线冒烟。

跑法（在 mecharm-grasp-exp/exp3 下，先 source 工作区使 sort_msgs 可用）：
  python3 ros2/task_control/pick_place_server.py --executor stub
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sort_msgs.action import PickPlace

from sort_core.taxonomy import (STAGE_APPROACH, STAGE_GRASP, STAGE_LIFT,
                                STAGE_MOVE_TO_BIN, STAGE_PLACE, STAGE_RETRACT,
                                STAGE_FAILING, STAGE_NAMES,
                                REASON_GRASP_FAILED, REASON_DROPPED,
                                REASON_UNREACHABLE, REASON_EXEC_ERROR,
                                REASON_ABORTED)
from ros2.task_control.exec_contract import PickExecutor, resolve_goal

# 每个动作段的失败 reason：唯一例外是『规划/驱动硬错』(异常) → exec_error / unreachable
_FAIL_IF_STAGE_FAILS = {
    STAGE_APPROACH: REASON_UNREACHABLE,   # 到格/下探失败 → 不可达
    STAGE_GRASP: REASON_EXEC_ERROR,       # 夹爪闭合动作本身失败
    STAGE_LIFT: REASON_GRASP_FAILED,      # 抬离后判 held 不过 → 没夹起
    STAGE_MOVE_TO_BIN: REASON_UNREACHABLE,
    STAGE_PLACE: REASON_DROPPED,          # 张开放置后判 placed 不过 → 掉了/放偏
    STAGE_RETRACT: REASON_UNREACHABLE,
}


class PickPlaceServer(Node):
    def __init__(self, grid, bins, executor, action_name='/pick_place'):
        super().__init__('pick_place_server')
        self.grid = grid
        self.bins = bins
        self.exec = executor
        self._as = ActionServer(self, PickPlace, action_name,
                                execute_callback=self._on_goal,
                                cancel_callback=self._on_cancel)

    def _fb(self, gh, stage, note):
        gh.publish_feedback(PickPlace.Feedback(
            stage=stage,
            note='%s | %s' % (STAGE_NAMES.get(stage, '?'), note)))

    def _on_cancel(self, gh):
        self.get_logger().warn('收到取消请求：让当前执行段自然停，回 aborted')
        return True  # 接受取消；当前段返回后按 aborted 收尾

    def _run_stage(self, gh, stage, fn):
        self._fb(gh, stage, '...')
        self.get_logger().info('stage=%s' % STAGE_NAMES.get(stage))
        ok, note = fn()                     # 执行者内部异常向外抛 → _on_goal 兜底
        if not ok:
            self._fb(gh, STAGE_FAILING, note)
        return ok, note

    def _on_goal(self, gh):
        g = gh.request
        t0 = time.monotonic()
        result = PickPlace.Result(success=False, reason=REASON_EXEC_ERROR, detail='')
        ctx, err = resolve_goal(self.grid, self.bins, g.cell_id, g.cls)
        if err:
            result.reason, result.detail = REASON_EXEC_ERROR, err
            gh.abort()
            return result

        try:
            chain = (
                (STAGE_APPROACH, lambda: self.exec.descend(ctx)),
                (STAGE_GRASP, lambda: self.exec.grasp(ctx)),
                (STAGE_LIFT, lambda: self.exec.lift(ctx)),
                (STAGE_MOVE_TO_BIN, lambda: self.exec.carry(ctx)),
                (STAGE_PLACE, lambda: self.exec.release(ctx)),
                (STAGE_RETRACT, lambda: self.exec.retract(ctx)),
            )
            for stage, fn in chain:
                if gh.is_cancel_requested:
                    self._fb(gh, STAGE_FAILING, 'cancelled')
                    result.reason, result.detail = REASON_ABORTED, 'cancelled'
                    gh.canceled()
                    return result
                ok, note = self._run_stage(gh, stage, fn)
                if not ok:
                    reason = _FAIL_IF_STAGE_FAILS.get(stage) or REASON_EXEC_ERROR
                    result.reason, result.detail = reason, note or ''
                    gh.abort()
                    self.get_logger().warn('goal %s/%s → %s' % (g.cell_id, g.cls, reason))
                    return result
        except Exception as e:               # 含 RuntimeError：动作硬错
            self.get_logger().error('executor 异常: %r' % (e,))
            result.reason, result.detail = REASON_EXEC_ERROR, repr(e)
            gh.abort()
            return result

        result.success = True
        result.reason = 'ok'
        result.detail = '%s→%s ok in %.1fs' % (ctx['cell_id'], ctx['bin_id'],
                                               time.monotonic() - t0)
        gh.succeed()
        return result


class _StubExecutor(PickExecutor):
    """离线冒烟用：假成功推进各段（仿真上图换 c4_executor.C4PickExecutor）。"""
    def descend(self, ctx): return True, 'stub descend A=%s' % ctx['A']
    def grasp(self, ctx): return True, 'stub grasp'
    def lift(self, ctx): return True, 'stub lift(held assumed)'
    def carry(self, ctx): return True, 'stub carry B=%s' % ctx['B']
    def release(self, ctx): return True, 'stub release(placed assumed)'
    def retract(self, ctx): return True, 'stub retract'


def main(args=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--executor', choices=['stub'], default='stub')
    a = ap.parse_args(args)

    rclpy.init()
    from sort_core.config import load_grid, load_bins
    node = PickPlaceServer(load_grid(), load_bins(),
                           {'stub': _StubExecutor}[a.executor]())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
