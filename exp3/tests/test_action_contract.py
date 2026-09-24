# -*- coding: utf-8 -*-
"""PickPlace.action 段落顺序防回归测试（不需要 ROS 2）。

ROS 2 的 .action 段落顺序是**固定的 Goal / Result / Feedback**：第二段永远是
Result、第三段永远是 Feedback，与"哪个语义更重要"无关。写反了 rosidl 照样能生成，
但字段会串进错误的类里，直到运行时才以

    AttributeError: 'PickPlace_Result' object has no attribute 'success'

的形式炸出来（2026-09-23 仿真线就是这么炸的：Feedback 被写在第二段，
result 里只剩 stage/note，feedback 里反而多出 success/reason/detail）。

本测试只解析 .action 文本，因此在没有 ROS 2 的机器上也能挡住这个错误。
"""
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# 权威版 + rosidl 实际编译的精简拷贝，两者都要检查
ACTION_FILES = [
    os.path.join(ROOT, 'contract', 'PickPlace.action'),
    os.path.join(ROOT, 'ros2', 'sort_msgs', 'action', 'PickPlace.action'),
]


def parse_action_segments(path):
    """按 ROS 2 约定返回 [goal, result, feedback]，每段是 [(类型, 字段名), ...]。"""
    with open(path, encoding='utf-8') as fh:
        lines = fh.read().splitlines()
    segments = [[]]
    for raw in lines:
        line = raw.split('#')[0].strip()
        if line == '---':
            segments.append([])
            continue
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            segments[-1].append((parts[0], parts[1]))
    return segments


class PickPlaceActionLayoutTest(unittest.TestCase):
    def test_files_exist_and_have_three_segments(self):
        for path in ACTION_FILES:
            with self.subTest(path=path):
                self.assertTrue(os.path.isfile(path), '缺少 %s' % path)
                self.assertEqual(len(parse_action_segments(path)), 3)

    def test_segments_are_goal_result_feedback(self):
        for path in ACTION_FILES:
            goal, result, feedback = parse_action_segments(path)
            with self.subTest(path=path, seg='goal'):
                self.assertEqual([n for _, n in goal], ['cell_id', 'cls'])
            with self.subTest(path=path, seg='result'):
                self.assertEqual([n for _, n in result],
                                 ['success', 'reason', 'detail'])
            with self.subTest(path=path, seg='feedback'):
                self.assertEqual([n for _, n in feedback], ['stage', 'note'])

    def test_field_types(self):
        for path in ACTION_FILES:
            goal, result, feedback = parse_action_segments(path)
            with self.subTest(path=path):
                self.assertEqual(goal, [('string', 'cell_id'), ('string', 'cls')])
                self.assertEqual(result, [('bool', 'success'),
                                          ('string', 'reason'),
                                          ('string', 'detail')])
                self.assertEqual(feedback, [('int32', 'stage'),
                                            ('string', 'note')])


if __name__ == '__main__':
    unittest.main()
