#!/usr/bin/env python3
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor


def _dets_brief(dets, cells, calib, image_size):
    """把一帧检测结果缩成一行：每个 blob 的 类别@(像素x,像素y)→格子或 OUT。

    ★ 为什么必须打这个：run_20260924_105338 里第 4 轮起 dets 数正常（6 个），
    却一个都不可执行（全是 out_of_grid 或落在已完成的 c1/c2/c3 上），连续 3 轮
    ⇒ no_exec。只看 dets **数量**分不出"没看到"和"看到了但映射失败"，
    必须看到像素坐标与映射结果才能定案。
    """
    if not cells or not calib or not image_size:
        return ''
    from sort_core.geometry import bbox_center, locate
    parts = []
    for d in dets:
        cx, cy = bbox_center(d['bbox'])
        loc = locate(d['bbox'], calib, cells, image_size)
        cid = loc['cell']['id'] if loc['cell'] else ('OUT' if loc['in_view'] else 'OFF')
        parts.append('%s@(%d,%d)->%s' % (d['cls'], cx, cy, cid))
    return ' '.join(parts)


def make_fresh_scan(scan, timeout=3.0, min_interval=8.0, cells=None,
                    calib=None, image_size=None):
    """把 scan 包一层"等新帧 + 最小间隔"，并把帧戳打进日志。

    ★ 为什么要等新帧：实测 run_20260924_005503，物块互穿引爆 ODE 之后**相机仍在
    以 ~48 fps 发帧，但画面内容冻结** —— 后续每轮扫描因此读到同一份陈旧检测结果，
    控制器误判"桌上只剩料盒里那个杯"，连续 3 轮无可做目标 ⇒ `no_exec` 提前收尾。
    只比对 dets 内容分不出"真没看到"和"画面卡住"，所以必须比对 header.stamp。

    ★ 为什么要有最小间隔（本轮真正修的那个 bug，run_20260924_104207 实测）：
    新一轮的 goal 会在**上一轮 retract 走完之前**就发出（日志里 c3 的 retract 期间
    出现了下一轮的 stage=grasp），于是本轮扫描时机械臂还在网格上方横穿
    （trace_blocks 实测刀尖 tip=(0.101, 0.001, 0.122)，正从料盒回格子上方），
    **整条臂的投影盖住 x=0.14 那一列** ⇒ 桌上剩余物块全部映射失败 ⇒ out_of_grid
    ⇒ 连续 3 轮无可做目标 ⇒ no_exec。臂回到停靠位后视野就是干净的（round 1 实测
    6/6 全见）。所以扫描之间必须留出臂回位的时间。

    正常时零影响：一轮执行要 45~60 s，远大于 min_interval；只有连续空转时才触发等待。
    """
    state = {'last': None, 'last_t': 0.0}

    def fresh():
        # ① 距上次扫描至少 min_interval —— 给机械臂留出回停靠位的时间
        gap = min_interval - (time.monotonic() - state['last_t'])
        if gap > 0:
            time.sleep(gap)
        # ② 等到一帧比上次更新过的检测结果
        t0 = time.monotonic()
        while True:
            f = scan.frame()
            if f.stamp_ns is None:
                return None                      # 还没首帧
            if f.stamp_ns != state['last']:
                state['last'] = f.stamp_ns
                state['last_t'] = time.monotonic()
                print('SCAN frame stamp=%d dets=%d waited=%.2fs %s'
                      % (f.stamp_ns, len(f.dets), time.monotonic() - t0,
                         _dets_brief(f.dets, cells, calib, image_size)), flush=True)
                return f.dets
            if time.monotonic() - t0 >= timeout:
                print('SCAN STALE stamp=%d dets=%d (%.1fs 未更新 —— 画面可能冻结)'
                      % (f.stamp_ns, len(f.dets), timeout), flush=True)
                state['last_t'] = time.monotonic()
                return f.dets                    # 如实接受，但留下痕迹
            time.sleep(0.05)
    return fresh


def main(args=None):
    exp3_root = os.environ.get('EXP3_ROOT')
    if not exp3_root:
        raise RuntimeError('EXP3_ROOT must be set by exp3_sim.launch.py')
    sys.path.insert(0, exp3_root)
    from ros2.task_control.pick_place_client import PickPlaceActionClient
    from ros2.task_control.scan_from_detections import ScanFromDetections
    from sort_core.config import load_all
    from sort_core.logging_util import Exp3RunLog, make_run_dir
    from sort_core.task import TaskController

    rclpy.init(args=args)
    scan = None
    client = None
    executor = None
    thread = None
    try:
        cfg = load_all()
        task, bins, grid = cfg['task'], cfg['bins'], cfg['grid']
        cam = task['camera']
        scan = ScanFromDetections('/detections_d2a',
                                  cam['image_width_px'], cam['image_height_px'])
        client = PickPlaceActionClient('/pick_place', wait_timeout=300.0)
        if not client.ready():
            raise RuntimeError('PickPlace action server is not ready')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(scan)
        executor.add_node(client)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        deadline = time.monotonic() + 45.0
        while scan.latest() is None and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.1)
        if scan.latest() is None:
            raise RuntimeError('timed out waiting for the first camera detection message')

        log_dir = os.path.join(exp3_root, task.get('log_dir', 'exp3_logs'))
        run_dir = make_run_dir(log_dir)
        runlog = Exp3RunLog(run_dir)
        runlog.text('source=camera HSV detector; run_dir=%s' % run_dir)
        controller = TaskController(
            task, bins, grid['cells'],
            make_fresh_scan(scan, cells=grid['cells'],
                            calib=cam['calib'],
                            image_size=(cam['image_width_px'], cam['image_height_px'])),
            client.pick_place, log=runlog)
        summary = controller.run()
        verdict = runlog.write_result(summary['placed_ok'], summary['objects_seen'],
                                      task['expected_total'], task['pass_line'],
                                      summary['exit_status'], summary['reasons'])
        runlog.text('verdict=%s summary=%s' % (verdict, summary))
        print('EXP3_RESULT verdict=%s placed_ok=%d objects_seen=%d exit_status=%s run_dir=%s'
              % (verdict, summary['placed_ok'], summary['objects_seen'],
                 summary['exit_status'], run_dir), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if scan is not None:
            scan.destroy_node()
        if client is not None:
            client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
