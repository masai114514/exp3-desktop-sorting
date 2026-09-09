#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PickPlace action client —— 把 TaskController 的 pick_place(cell_id, cls) 缝隙接到 ROS2 Action。

TaskController 缝隙签名（sort_core/task.py 的注入点）：
    pick_place(cell_id, cls) -> ('ok', note) | (ACTION_REASON, note)
本类用 sort_msgs/action/PickPlace 兑现该缝隙 —— 与 examples/mock_backend.py 是同一个缝，
只是从『mock 移走物体』换成『发 ROS2 Action 给执行端』。跑法见 task_control/README.md。

调用约束（同 c4_2/drive/c4_cycle.py）：TaskController.run() 应跑在主线程（spin 放另一线程），
否则 send_goal 后的 spin_until_future_complete 会套进 executor 回调里阻塞。典型编排：
    先 rclpy 起一个线程 spin executor，主线程里 TaskController(...).run() 同步调 client。
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sort_msgs.action import PickPlace

from sort_core.taxonomy import REASON_EXEC_ERROR, REASON_ABORTED, ACTION_REASONS


def _spin(node, fut, timeout):
    """给一次 send/result 的同步等待；超时返回 False。"""
    if not rclpy.ok():
        return False
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.done() and not fut.cancelled()


class PickPlaceActionClient(Node):
    def __init__(self, action_name='/pick_place', wait_timeout=10.0):
        super().__init__('pick_place_client')
        self._cli = ActionClient(self, PickPlace, action_name)
        if not self._cli.wait_for_server(timeout_sec=wait_timeout):
            self.get_logger().error('PickPlace server 不可达: %s', action_name)

    def ready(self):
        return self._cli.server_is_ready()

    def pick_place(self, cell_id, cls, timeout=120.0):
        if not self.ready():
            return REASON_EXEC_ERROR, 'PickPlace server 不可达'
        goal = PickPlace.Goal(cell_id=cell_id, cls=cls)

        send = self._cli.send_goal_async(goal)
        if not _spin(self, send, timeout):
            return REASON_ABORTED, 'send_goal 超时/中断'
        handle = send.result()
        if handle is None or not handle.accepted:
            return REASON_EXEC_ERROR, 'goal 被拒'

        got = handle.get_result_async()
        if not _spin(self, got, timeout):
            return REASON_ABORTED, '等 result 超时'
        if got.result() is None:
            return REASON_ABORTED, 'result 缺失（可能被取消）'

        res = got.result().result          # sort_msgs/action/PickPlace.Result
        if res.success:
            return 'ok', res.detail
        # 下游把 reason 落 records.jsonl：必须是 taxonomy 动作级枚举，别名也收下
        reason = res.reason if res.reason in ACTION_REASONS else REASON_EXEC_ERROR
        return reason, res.detail or res.reason
