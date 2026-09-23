#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EP 真机 A->B 定点取放主循环(验收口径同 c4_2: 5 次连续 >=4 成功)。

每周期:
  [臂回 lift_high 高悬停] -> [夹爪张开] -> [底盘 goto A_pose]
  -> [臂下探 grab_low 包住目标物] -> [夹爪闭紧] -> [臂抬回 lift_high 提走]
  -> 人工确认 HELD(y/n) -> [底盘 goto B_pose]
  -> [臂下放到位] -> [夹爪张开释放] -> [臂抬回退开]
  -> 人工确认 PLACED(y/n) -> [底盘 goto HOME_pose] -> 记周期结果

日志/结果文件同 c4_2 schema(ep_logs/run_*/result.json)。reason 枚举一致。
EP 无物块感知 + 底盘开环定位有漂移 → 夹起/放正两处都靠操作员确认(pause 首跑必开)。

用法: python3 ep/drive/ep_cycle.py --config ep/config_ep.json [--cycles 5] [--log-dir ...]
"""
import argparse
import json
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from ep.core.selfcheck import check_config, check_calibrated   # noqa: E402
from ep.drive.rm import RM                          # noqa: E402
from ep.drive.ep_log import EPLog, default_run_dir  # noqa: E402


def _confirm(log, question):
    while True:
        ans = input('{} (y/n/q) > '.format(question)).strip().lower()
        if ans in ('y', 'yes'):
            return True
        if ans in ('n', 'no'):
            return False
        if ans in ('q', 'quit', 'x'):
            raise KeyboardInterrupt('操作员中止')


class Runner:
    def __init__(self, cfg, log):
        self.cfg = cfg
        self.log = log
        self.pause = cfg['cycle'].get('pause_each_phase', False)

    def _pause(self, tag):
        if self.pause:
            self.log.text('[PAUSE] {} 回车继续'.format(tag))
            input()
            self.log.text('[继续]')

    def run_cycle(self, i, rm):
        """执行第 i 个周期, 返回 (ok, reason, detail)。异常向上抛由外层记 exec_error。"""
        ch = self.cfg['chassis']
        am = self.cfg['arm']
        retry_left = int(self.cfg['cycle'].get('retry_grab', 1))
        held, placed = False, False
        try:
            # 1) 高悬停 + 开爪(兜底: 上周期可能停在低处/闭爪)
            rm.arm_moveto(*am['lift_high'], tag='cycle_start_high')
            self._sample('PREP', rm)
            rm.gripper_open()
            self._pause('PREP: 臂已回高, 爪已开, 把目标物(真机=矿泉水瓶)放到 A 标记上')

            # 2) 底盘到 A, 下探, 夹紧, 抬走
            rm.goto(*ch['A_pose'])
            self._sample('TO_A', rm)
            rm.arm_moveto(*am['grab_low'], tag='descend_A')
            self._sample('GRAB_PREP', rm)
            rm.gripper_close()
            rm.arm_moveto(*am['lift_high'], tag='lift_A')
            self._sample('HELD', rm)

            # 3) HELD 人工确认(可重抓 retry 次)
            while not held:
                held = _confirm(self.log, 'HELD 确认: 目标物被夹起且没滑?')
                if held:
                    break
                if retry_left > 0:
                    retry_left -= 1
                    self.log.text('HELD=n, 重抓(剩 {} 次): 开爪->下探->闭紧->提起'.format(retry_left))
                    rm.gripper_open()
                    rm.arm_moveto(*am['grab_low'], tag='retry_descend')
                    rm.gripper_close()
                    rm.arm_moveto(*am['lift_high'], tag='retry_lift')
                else:
                    return False, 'grasp_failed', 'HELD 连续确认失败, 开爪回 HOME'
            self.log.event('held_ok', cycle=i)

            # 4) 携带到 B, 下放, 释放, 退开
            rm.goto(*ch['B_pose'])
            self._sample('CARRY_TO_B', rm)
            release = am.get('release_low_override') or am['grab_low']
            rm.arm_moveto(*release, tag='descend_B')
            time.sleep(am.get('drop_wait_s', 1.0))
            self._sample('PLACE_PREP', rm)
            rm.gripper_open()
            time.sleep(0.3)
            rm.arm_moveto(*am['lift_high'], tag='retreat_B')
            self._sample('RELEASED', rm)
            placed = _confirm(self.log, 'PLACED 确认: 目标物落在 B 圈内、没翻?')
            if not placed:
                return False, 'dropped_or_missed_place', 'B 落放未被确认(目标物偏/翻)'

            # 5) 回 HOME
            rm.goto(*ch['HOME_pose'])
            self._sample('HOME', rm)
            return True, '', 'A->B 完整(held/placed 均确认)'
        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.log.text('周期 {} 执行异常: {}'.format(i, e))
            raise

    def _sample(self, phase, rm):
        try:
            self.log.sample(phase, rm.read_chassis_pose(), rm.read_arm_xy(),
                            getattr(rm, 'grip_state', '?'))
        except Exception:
            pass

    def finish_cleanup(self, rm):
        """非成功路径的收尾: 开爪/回高, 尽可能回到 HOME。"""
        try:
            rm.gripper_open()
        except Exception:
            pass
        try:
            rm.arm_moveto(*self.cfg['arm']['lift_high'], tag='cleanup_high')
        except Exception:
            pass
        try:
            rm.goto(*self.cfg['chassis']['HOME_pose'])
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config_ep.json'))
    ap.add_argument('--cycles', type=int, default=None)
    ap.add_argument('--log-dir', default=None)
    args = ap.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        cfg = json.load(f)
    if args.cycles is not None:
        cfg['cycle']['cycles'] = args.cycles

    errs = []
    check_config(cfg, errs)
    check_calibrated(cfg, errs)          # ← 硬闸门: 未标定不许驱动真机
    if errs:
        print('FAIL: config 未通过检查, 拒绝启动(不会连机器人):')
        for e in errs:
            print('  - ' + e)
        print('\n先做标定: python3 ep/drive/env_check.py --config {} --jog'.format(args.config))
        print('标定步骤见 ep/标定说明.md §1–§3; 做完把 config 里 calibration.status 改成 calibrated')
        sys.exit(1)

    run_dir = default_run_dir(args.log_dir or cfg.get('log_dir') or 'ep_logs')
    log = EPLog(run_dir)
    log.text('config: {}'.format(os.path.abspath(args.config)))
    log.text('cycles={}  pause_each_phase={}  A_pose={}  B_pose={}'.format(
        cfg['cycle']['cycles'], cfg['cycle'].get('pause_each_phase'),
        cfg['chassis']['A_pose'], cfg['chassis']['B_pose']))

    rm = RM(cfg, log)
    try:
        try:
            rm.connect()
        except Exception as e:
            # 真机上第一次跑最常卡在这: SDK 没装 / 热点没连 / 不是工程形态。
            # 别让操作员看 traceback —— 直接把原因和下一步打出来。
            print('\nFAIL: 连不上 EP —— {}'.format(e))
            print('排查: ① robomaster SDK 装了吗(python -c "import robomaster")')
            print('      ② 电脑连上 EP 热点了? 还是该把 conn_type 改 sta?')
            print('      ③ EP 是**工程形态**吗(步兵形态没有 arm/gripper)')
            print('先跑: python3 ep/drive/env_check.py --config {}'.format(args.config))
            sys.exit(1)
        total = int(cfg['cycle']['cycles'])
        runner = Runner(cfg, log)
        for i in range(1, total + 1):
            t0 = time.monotonic()
            log.text('===== 周期 {}/{} 开始 (先把目标物放回 A 标记) ====='.format(i, total))
            try:
                ok, reason, detail = runner.run_cycle(i, rm)
                log.add_cycle_result(i, ok, reason, detail, time.monotonic() - t0)
                log.text('周期 {} 结果: ok={} reason={}'.format(i, ok, reason or '-'))
            except KeyboardInterrupt:
                log.text('== 操作员中止 ==')
                log.add_cycle_result(i, False, 'aborted', '操作员 q/ctrl-c', time.monotonic() - t0)
                break
            except Exception as e:
                log.write_exception(e)
                log.add_cycle_result(i, False, 'exec_error', str(e), time.monotonic() - t0)
                log.text('== exec_error, 中止剩余周期 ==')
                runner.finish_cleanup(rm)
                break
    finally:
        rm.disconnect()
        log.write_result(final=True)
        log.close()

    with open(log.result_path, 'r', encoding='utf-8') as f:
        res = json.load(f)
    print('\n结果: {}/{} 成功(达标线 4/5): {}'.format(
        res['ok_count'], res['total'], 'PASS' if res['ok_count'] >= 4 else '未达标'))
    print('result.json: {}'.format(log.result_path))


if __name__ == '__main__':
    main()
