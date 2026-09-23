#!/usr/bin/env python3
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor


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
        controller = TaskController(task, bins, grid['cells'], scan.latest,
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
