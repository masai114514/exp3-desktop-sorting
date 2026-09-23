#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验3 真机探针 —— 测「夹爪状态回传」能不能用来自动判"夹住没有"。

     python3 real/gripper_probe.py --label empty  --reps 3    # 爪间什么都不放
     python3 real/gripper_probe.py --label bottle --reps 3    # 放候选实物（名字随便起）
     python3 real/gripper_probe.py --report                   # 两种都测完 → 给结论

来历：别组给的提示 —— 「夹爪从张开到收紧有个响应时间，**夹到物体跟夹空的时间阈值不一样**」。
这台 EP 的官方 SDK 里确实有一条**我们从来没用过**的状态订阅
（`robomaster/gripper.py` 的 `GripperSubject` → `opened` / `closed` / `normal`，
`sub_status(freq)` 最高 50 Hz），走的是和 `arm.sub_position`（实验二已验证可用）同一套 DDS 机制。

**但这个脚本存在的意义是"我们不知道固件的 status 是从什么推出来的"**，有三种可能：

  (a) 行程/限位 —— 爪被挡住 ⇒ 永远停在 normal           ⇒ **状态本身就是判据**（最好）
  (b) 堵转/电流 —— 挡住也报 closed，只是用时不同        ⇒ **只能用时间阈值**
  (c) 命令回执 —— 发令就变，和物理无关                  ⇒ **这条路作废**

三种在纸面上分不出来，**只有实测能分**。所以本脚本不预设任何结论：它把
「(时间, 状态) 轨迹」原样记下来，最后用 `classify()` 判是哪种，并把该用的阈值算给你。

★ 三条纪律：
  1. **在最终确定的 `close_power` 下测** —— 换 power = 换一套物理行为，别处测的阈值不能用。
  2. **每个标签至少 3 次** —— 单次测不出稳定性，而稳定性正是这条判据的全部价值。
  3. **最小/最软的那个候选实物必须单独测一遍** —— 物体比夹爪闭合间隙还小、或一夹就变形时，
     空合与夹物可能给出同一个状态 ⇒ 那它就判不了，**这会反过来否决这个实物**。

本脚本**不写任何配置、不改任何代码**，只产出轨迹与结论；改不改判据是之后的事。
"""
import argparse
import datetime
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXP3 = os.path.dirname(_HERE)
for _p in (_EXP3, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import ep_conn                                              # noqa: E402

# 状态字符串（SDK 原样给的就是这三个）
OPENED, CLOSED, NORMAL = 'opened', 'closed', 'normal'
KNOWN = (OPENED, CLOSED, NORMAL)

DEFAULT_OUT = os.path.join(_EXP3, 'exp3_logs_gripper')


# ---------------------------------------------------------------- 纯逻辑（好测）

def new_trace():
    return {'samples': []}          # [(t_rel_s, status)]


def add_sample(trace, t_rel, status):
    trace['samples'].append([round(float(t_rel), 4), status])


def final_status(trace):
    """最后一个样本的状态；没有样本 → None。"""
    s = trace['samples']
    return s[-1][1] if s else None


def settle_time(trace, target=None):
    """首次到达 target（默认=终态）的相对时间；从未到达 → None。

    只在**终态等于 target** 时才有意义 —— 一个从没到过 closed 的轨迹，"到达时间"是 None，
    这正是判据 (a) 与 (b) 的分界，所以这里**不做兜底**，让 None 如实传出去。
    """
    s = trace['samples']
    if not s:
        return None
    if target is None:
        target = s[-1][1]
    for t, st in s:
        if st == target:
            return t
    return None


def summarize(trace):
    """一次开合的结论：(终态, 到终态用时 | None, 样本数)。"""
    return final_status(trace), settle_time(trace), len(trace['samples'])


def classify(empty_traces, obj_traces, margin=1.5):
    """从「空合」与「夹物」两组轨迹判断是哪种固件行为。

    返回 dict：verdict ∈ {'state','time','unusable','need_more'}，外加 reason / threshold_s。

      state    ：空合全部到 closed、夹物全部停 normal ⇒ 按状态判，不需要阈值（最好）
      time     ：终态一样，但夹物用时明显更长 ⇒ 按时间判，给阈值
      unusable ：终态一样且时间也分不开 ⇒ 这条判据不成立
    """
    es = [summarize(t) for t in empty_traces]
    os_ = [summarize(t) for t in obj_traces]
    if not es or not os_:
        return {'verdict': 'need_more',
                'reason': '两组都要至少 1 次（空的 %d 次、物的 %d 次）' % (len(es), len(os_)),
                'threshold_s': None}

    ef = {r[0] for r in es}
    of = {r[0] for r in os_}
    if None in ef or None in of:
        return {'verdict': 'unusable', 'threshold_s': None,
                'reason': '有轨迹一个样本都没收到 —— 订阅没生效（先确认 sub_status 返回 True、'
                          '以及 EP 是工程形态）。'}

    # (a) 状态可分：空合到 closed，夹物到不了 closed
    if ef == {CLOSED} and CLOSED not in of:
        return {'verdict': 'state', 'threshold_s': None,
                'reason': '空合稳定到 closed、夹物停在 %s' % '/'.join(sorted(of))}

    # (b) 终态一样 → 看时间
    if ef == of:
        et = [r[1] for r in es]
        ot = [r[1] for r in os_]
        if None in et or None in ot:
            return {'verdict': 'unusable', 'threshold_s': None,
                    'reason': '终态相同但有用时缺失，判不了'}
        e_max, o_min = max(et), min(ot)
        if o_min > e_max * margin:
            return {'verdict': 'time', 'threshold_s': round((e_max + o_min) / 2.0, 3),
                    'reason': '终态都是 %s；空合最慢 %.3fs、夹物最快 %.3fs（差 %.2f 倍）'
                              % ('/'.join(sorted(ef)), e_max, o_min, o_min / max(e_max, 1e-9))}
        return {'verdict': 'unusable', 'threshold_s': None,
                'reason': '终态都是 %s，且用时分不开（空合最慢 %.3fs vs 夹物最快 %.3fs）'
                          % ('/'.join(sorted(ef)), e_max, o_min)}

    # 终态不一致但方向反了（空合 normal / 夹物 closed）—— 说不通，照实报
    return {'verdict': 'unusable', 'threshold_s': None,
            'reason': '空合终态 %s、夹物终态 %s —— 方向不对，先确认标签有没有贴反'
                      % ('/'.join(sorted(ef)), '/'.join(sorted(of)))}


# ---------------------------------------------------------------- 采集

class GripRecorder(object):
    """接一条 (t, status) 轨迹。订阅回调只做一件事：append。

    回调在 SDK 的线程池里跑（`dds.py` 的 dispatcher），所以这里**只记不判** ——
    在回调里做判断/IO 会拖慢派发，把后面的样本挤掉（臂位置的订阅也是这个纪律）。
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._t0 = None
        self.trace = new_trace()
        self.n_total = 0

    def arm(self):
        """开始记一条新轨迹（在**发 close 之前**调，免得把上一次的尾巴记进来）。"""
        self._t0 = self._clock()
        self.trace = new_trace()

    def __call__(self, status):
        self.n_total += 1
        if self._t0 is None:
            return
        if status not in KNOWN:
            status = 'unknown:%r' % (status,)
        add_sample(self.trace, self._clock() - self._t0, status)


def wait_settle(rec, settle_s, quiet_s=0.5):
    """等到轨迹「安静」下来（quiet_s 内没有新样本）或超时。返回实际等待秒数。

    不做"等到某个状态"——那等于把假设写进采集里。采集只管**等它不动了**。
    """
    t_start = time.monotonic()
    last_n, last_change = -1, time.monotonic()
    while time.monotonic() - t_start < settle_s:
        n = len(rec.trace['samples'])
        if n != last_n:
            last_n, last_change = n, time.monotonic()
        elif time.monotonic() - last_change >= quiet_s:
            break
        time.sleep(0.02)
    return time.monotonic() - t_start


def one_rep(rm, rec, power, settle_s, log=None):
    """一次开合：先开到底 → 记起点 → 合 → 等安静。返回该次的结论。"""
    rm.gripper_open()
    wait_settle(rec, settle_s)
    rec.arm()
    if log:
        log('  close(power=%d) 发令，开始记轨迹…' % power)
    rm.gripper_close()
    waited = wait_settle(rec, settle_s)
    fin, t_set, n = summarize(rec.trace)
    if log:
        log('    终态=%-8s 到终态=%-8s 样本=%d 等待=%.2fs'
            % (fin, ('%.3fs' % t_set) if t_set is not None else '—', n, waited))
    return rec.trace, fin, t_set


# ---------------------------------------------------------------- 落盘

def run_dir(out_root):
    d = os.path.join(out_root, datetime.datetime.now().strftime('probe_%Y%m%d_%H%M%S'))
    return d


def save_run(d, label, power, freq, settle_s, results, note=''):
    if not os.path.isdir(d):
        os.makedirs(d)
    p = os.path.join(d, 'label_%s.json' % label)
    with open(p, 'w', encoding='utf-8') as f:
        json.dump({'label': label, 'power': power, 'freq': freq, 'settle_s': settle_s,
                   'note': note, 'when': datetime.datetime.now().isoformat(timespec='seconds'),
                   'reps': [{'trace': t, 'final': fin, 'settle_s': ts}
                            for t, fin, ts in results]},
                  f, ensure_ascii=False, indent=2)
    return p


def load_labels(d):
    """读回同一目录下所有 label_*.json → {label: [trace, ...]}。"""
    out = {}
    if not os.path.isdir(d):
        return out
    for fn in sorted(os.listdir(d)):
        if not (fn.startswith('label_') and fn.endswith('.json')):
            continue
        with open(os.path.join(d, fn), encoding='utf-8') as f:
            d0 = json.load(f)
        out[d0['label']] = [r['trace'] for r in d0.get('reps', [])]
    return out


def latest_dir(out_root):
    if not os.path.isdir(out_root):
        return None
    ds = sorted(x for x in os.listdir(out_root) if x.startswith('probe_'))
    return os.path.join(out_root, ds[-1]) if ds else None


def resolve_dir(args):
    """--report / 续测 该读哪个目录。按"越明确越优先"：--out-dir → 本身就是记录目录 →
    里面最新的 probe_*。宽容一点是有意的：现场人是`--out-root`指哪儿就指望哪儿能用。"""
    if args.out_dir:
        return args.out_dir
    root = args.out_root
    if os.path.isdir(root) and any(f.startswith('label_') and f.endswith('.json')
                                   for f in os.listdir(root)):
        return root
    return latest_dir(root)


# ---------------------------------------------------------------- dry-run 假爪

class _FakeGripper(object):
    """--dry-run 用：不连机器人也能把整条链路跑通、看到输出长什么样。

    ⚠ 这里的时间**是编的**，只为了让流程能走。真结论只能现场实测得到。
    """

    def __init__(self, label, freq=20):
        self._label = label
        self._cb = None
        self._t = None

    def sub_status(self, freq=20, callback=None, *a, **kw):
        self._cb = callback
        return True

    def unsub_status(self):
        self._cb = None
        return True

    def open(self, power=50):
        self._emit(OPENED, 0.35)
        return True

    def close(self, power=50):
        if self._label == 'empty':
            self._emit(NORMAL, 0.03)
            self._emit(CLOSED, 0.42)
        else:
            self._emit(NORMAL, 0.03)
            self._emit(NORMAL, 0.30)
        return True

    def _emit(self, status, delay):
        import threading

        def fire():
            time.sleep(delay)
            if self._cb:
                self._cb(status)
        threading.Thread(target=fire, daemon=True).start()


class _FakeRM(object):
    def __init__(self, label):
        self.gripper = _FakeGripper(label)
        self._label = label

    def gripper_open(self):
        self.gripper.open(power=60)

    def gripper_close(self):
        self.gripper.close(power=60)

    def disconnect(self):
        self.gripper.unsub_status()


# ---------------------------------------------------------------- 命令

def cmd_probe(args):
    ep = load_ep(args.config_dir)
    power = args.power if args.power is not None else (ep.get('gripper') or {}).get('close_power', 60)
    d = args.out_dir or run_dir(args.out_root)

    print('★ 探针：测夹爪状态回传能不能判"夹住没有"')
    print('  标签(爪间的东西) = %s   次数 = %d   close_power = %s   push = %d Hz'
          % (args.label, args.reps, power, args.freq))
    print('  过程：张开 → 记起点 → 合 → 记录 (时间, 状态) 轨迹 → 等安静')
    if args.dry_run:
        print('  ⚠ --dry-run：**没有连机器人**，下面的时间是编的，只演示输出格式。')
    else:
        print('  ⚠ 确认爪间放的就是「%s」，而且放好之后手离开机器人。' % args.label)
    print('')

    rm = _FakeRM(args.label) if args.dry_run else ep_conn.connect(ep, log=None)
    rec = GripRecorder()
    if not rm.gripper.sub_status(freq=args.freq, callback=rec):
        print('夹爪状态订阅**没被接受**（sub_status 返回 False）。先别继续 —— '
              '这本身就是一个结论：这条路在这台机器上走不通。')
        return 2

    results = []
    try:
        for i in range(args.reps):
            print('第 %d/%d 次：' % (i + 1, args.reps))
            results.append(one_rep(rm, rec, power, args.settle_s, log=print))
    except KeyboardInterrupt:
        print('\n打断。已采集的照实存下来（不足 %d 次，结论会更弱）。' % args.reps)
    finally:
        try:
            rm.gripper.unsub_status()
        except Exception:
            pass
        try:
            rm.disconnect()
        except Exception:
            pass

    if not results:
        print('一次都没采到，没东西可存。')
        return 1
    p = save_run(d, args.label, power, args.freq, args.settle_s, results, note=args.note)
    print('\n已存 %s（总样本 %d 个）' % (p, rec.n_total))

    labels = load_labels(d)
    others = [k for k in labels if k != args.label]
    if others:
        print('\n目录里已有标签 %s —— 直接给结论：' % '、'.join(sorted(others)))
        return _report_from(labels, args.label, others[0])
    nxt = 'empty' if args.label != 'empty' else '<实物名>'
    print('\n再测另一组（同一个目录，别换）：')
    print('  python3 real/gripper_probe.py --label %s --reps %d --out-dir %s'
          % (nxt, args.reps, d))
    return 0


def _report_from(labels, obj_label, empty_label):
    empty = labels.get('empty') or labels.get(empty_label, [])
    obj = labels.get(obj_label, [])
    r = classify(empty, obj)
    print('\n' + '=' * 66)
    print('结论：%s' % r['verdict'].upper())
    print('理由：%s' % r['reason'])
    if r['verdict'] == 'state':
        print('\n⇒ **状态本身就是判据**，不需要时间阈值：')
        print('   夹完读一次状态：closed ⇒ 夹空（没夹住）；其余 ⇒ 夹到东西了。')
        print('   接线方式见 实验3_夹爪判据_状态回传方案.md §3（judge.held 加第三个值）。')
    elif r['verdict'] == 'time':
        print('\n⇒ **只能按时间判**，建议阈值 %.3fs（空合/夹物的中间值）：'
              % r['threshold_s'])
        print('   close 发令后等这个时间再读状态/看是否已落定，两侧都要留现场复测的余量。')
    elif r['verdict'] == 'unusable':
        print('\n⇒ **这条判据在这台机器上不成立**（至少对这个实物不成立）。')
        print('   两个去处：① 换一个更大/更硬的实物再测；'
              '② 改用相机重扫（网格/料盒状态）那条线。')
    else:
        print('\n⇒ 样本还不够，两组都至少测 3 次再来。')
    print('=' * 66)
    return 0 if r['verdict'] in ('state', 'time') else 1


def cmd_report(args):
    d = resolve_dir(args)
    if not d:
        raise SystemExit('没找到任何探针记录（%s）。先跑 --label empty 与 --label <实物>。'
                         % args.out_root)
    labels = load_labels(d)
    if len(labels) < 2:
        raise SystemExit('%s 里只有 %s —— 两类（empty + 实物）都测完才能判。'
                         % (d, '、'.join(labels) or '空'))
    empty = labels.get('empty', [])
    objs = [k for k in labels if k != 'empty']
    if not empty:
        raise SystemExit('缺 empty 那组（爪间什么都不放），没有基准判不了。')
    rc = 0
    for k in sorted(objs):
        print('\n### 实物标签：%s' % k)
        rc = _report_from(labels, k, 'empty')
    print('\n记录目录：%s' % d)
    return rc


def load_ep(config_dir):
    """读 `config_real/ep_waypoints.json`。**直接读**，不 import calibrate ——
    免得本探针被另一个模块的内部实现绑住（探针要能在标定工具改版后照旧跑）。"""
    p = os.path.join(config_dir, 'ep_waypoints.json')
    if not os.path.isfile(p):
        raise SystemExit('读不到 %s\n（--config-dir 指到 config_real/ 了吗？）' % p)
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='测夹爪状态回传能不能自动判"夹住没有"（不写配置、不改代码）')
    ap.add_argument('--label', help='爪间放的是什么：空合用 empty，实物随便起名')
    ap.add_argument('--reps', type=int, default=3, help='重复几次（默认 3；单次测不出稳定性）')
    ap.add_argument('--power', type=int, help='close_power 覆盖（默认取 config_real）')
    ap.add_argument('--freq', type=int, default=20, help='状态推送频率 Hz（默认 20，最大 50）')
    ap.add_argument('--settle-s', type=float, default=4.0, help='每次等状态安静的上限秒数')
    ap.add_argument('--note', default='', help='这次的情况（实物名/尺寸/夹住了没有，人眼看）')
    ap.add_argument('--config-dir', default=os.path.join(_EXP3, 'config_real'))
    ap.add_argument('--out-root', default=DEFAULT_OUT, help='记录根目录')
    ap.add_argument('--out-dir', help='直接指定某一次记录的目录（配 --report 或续测）')
    ap.add_argument('--report', action='store_true', help='不连机器人，读已有记录给结论')
    ap.add_argument('--dry-run', action='store_true',
                    help='不连机器人，用假爪演示流程（时间是编的）')
    args = ap.parse_args(argv)

    if args.report:
        return cmd_report(args)
    if not args.label:
        ap.error('要么给 --label（开始测），要么给 --report（读已有记录）')
    return cmd_probe(args)


if __name__ == '__main__':
    sys.exit(main())
