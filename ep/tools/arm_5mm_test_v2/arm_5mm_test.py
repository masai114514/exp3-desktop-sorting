#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One manually requested +5 mm arm movement; default mode is offline.

This is a first-step diagnostic for the user's observed post-calibration pose, not a
calibration controller. No automatic retry, return, recenter, chassis motion,
gripper motion, or private action-abort call is implemented. A wait timeout
does not cancel a robot action. Ctrl+C and disconnect are not emergency stops.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
import ipaddress
import json
import logging
import math
from pathlib import Path
import sys
import time
import traceback

from check_ep import Telemetry, load_sdk

ROOT = Path(__file__).resolve().parent
VERSION = "2"
REFERENCE_MM = (74, 57)
REFERENCE_TOL_MM = 10  # Pose-change check, NOT a reach/collision limit.
MIN_BATTERY = 30      # Chosen for this diagnostic, NOT a manufacturer limit.
ACTION_TIMEOUT_S = 10
STABLE_SAMPLES = 3
STABLE_SPREAD_MM = 1
DELTA_TOL_MM = 2      # Coarse observation screen, NOT calibrated accuracy.
POST_ACTION_OBSERVE_S = 2.0


class TraceTelemetry(Telemetry):
    """Keep continuous arm callbacks, including invalid data, for diagnosis."""
    def __init__(self):
        super().__init__()
        self.arm_trace = deque(maxlen=10000)
        self.arm_trace_total = 0

    def callback(self, name):
        receive = super().callback(name)
        if name != "arm_position":
            return receive

        def receive_and_record(value, *args, **kwargs):
            receive(value, *args, **kwargs)
            with self.lock:
                current = self.data.get(name)
                event = {"utc_time": datetime.now(timezone.utc).isoformat(),
                         "monotonic_s": time.monotonic(),
                         "valid": current is not None}
                if current is not None:
                    event.update(xy_mm=list(current["value"]),
                                 raw_xy=list(current["raw_value"]),
                                 received_at_monotonic=current["received_at"])
                else:
                    event.update(raw_repr=repr(value), error=self.invalid.get(name, "invalid"))
                self.arm_trace.append(event)
                self.arm_trace_total += 1
        return receive_and_record

    def trace_snapshot(self):
        with self.lock:
            return {"total_samples": self.arm_trace_total,
                    "retained_samples": len(self.arm_trace),
                    "truncated": self.arm_trace_total > len(self.arm_trace),
                    "samples": list(self.arm_trace)}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--offline", action="store_true", help="只做导入检查（默认），不连接")
    mode.add_argument("--connect", action="store_true", help="连接并等人工输入 u 后向上移动一次 5 mm")
    parser.add_argument("--wait", type=float, default=15, help="启动等待读数的秒数，1..60")
    parser.add_argument("--local-ip", default=None, help="可选：笔记本 EP Wi-Fi 网卡的 IPv4")
    args = parser.parse_args(argv)
    if not math.isfinite(args.wait) or not 1 <= args.wait <= 60:
        parser.error("--wait 必须在 1..60 秒之间")
    if args.local_ip:
        try:
            addr = ipaddress.IPv4Address(args.local_ip)
            if addr not in ipaddress.IPv4Network("192.168.2.0/24") or int(addr) & 255 in (0, 1, 255):
                raise ValueError("只接受笔记本的 192.168.2.2–254 地址")
        except ValueError as exc:
            parser.error(str(exc))
    return args


def stable_reading(telemetry, after, timeout=4.0):
    """Require distinct, fresh, stable arm samples received after a boundary."""
    deadline = time.monotonic() + timeout
    samples = []
    last_received = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        with telemetry.lock:
            arm = dict(telemetry.data.get("arm_position", {}))
            battery = dict(telemetry.data.get("battery", {}))
        valid = all(item and item.get("count", 0) >= 2
                    and 0 <= now - item["received_at"] <= 1.0
                    for item in (arm, battery))
        if not valid:
            samples.clear()
        elif arm["received_at"] > after and arm["received_at"] != last_received:
            last_received = arm["received_at"]
            samples.append(list(arm["value"]))
            samples = samples[-STABLE_SAMPLES:]
            if len(samples) == STABLE_SAMPLES and all(
                    max(p[axis] for p in samples) - min(p[axis] for p in samples)
                    <= STABLE_SPREAD_MM for axis in (0, 1)):
                return {"xy_mm": samples[-1], "samples_mm": list(samples),
                        "raw_xy_mm": arm["raw_value"], "battery_percent": battery["value"],
                        "received_at_monotonic": arm["received_at"]}
        time.sleep(0.05)
    raise RuntimeError("未收到连续、稳定且新鲜的机械臂/电量读数；本轮不再发动作。")


def check_start(reading):
    if reading["battery_percent"] < MIN_BATTERY:
        raise RuntimeError("本测试要求电量至少 {}%；请先充电。".format(MIN_BATTERY))
    if any(abs(v - ref) > REFERENCE_TOL_MM
           for v, ref in zip(reading["xy_mm"], REFERENCE_MM)):
        raise RuntimeError("当前位置 {} 与校准后已确认姿态 {} 相差较大；请保留读数和照片，不执行点动。"
                           .format(reading["xy_mm"], list(REFERENCE_MM)))


def ask_observation(read, emit):
    while True:
        answer = read("是否亲眼看到机械臂向上微动并停住？y=是 / n=没有或异常 / u=不确定 > ").strip().lower()
        if answer in ("y", "n", "u", "q"):
            return {"y": True, "n": False, "u": None, "q": None}[answer]
        emit("这里填写 y、n 或 u；不会继续发送动作。")


def console(arm, telemetry, result, emit, read=input):
    initial = stable_reading(telemetry, after=time.monotonic())
    result["initial"] = initial
    emit("BATTERY: {}%   ARM_POSITION: {} mm".format(initial["battery_percent"], initial["xy_mm"]))
    check_start(initial)
    emit("请确认：机器人在平整空地，夹爪空载，臂上方和周围没有显示器、电线、手或其他障碍。")
    emit("输入 u 代表已确认现场条件，执行一次相对向上 5 mm；q 退出。")
    emit("5 mm 是指令距离；本接口不提供速度参数，实际位移还要读数和观察核对。")
    emit("Ctrl+C/超时/断开连接不等于机械臂已停止。如异常运动或持续顶住，请关闭机器人电源。")
    while True:
        command = read("arm> ").strip().lower()
        if command == "q":
            result["status"] = "NO_MOTION_REQUESTED"
            return
        if command == "u":
            break
        emit("请输入 u 或 q。")

    # Input may have been left open for minutes: reacquire after confirmation.
    before = stable_reading(telemetry, after=time.monotonic())
    result["before"] = before
    check_start(before)
    if any(abs(v - prev) > 2 for v, prev in zip(before["xy_mm"], initial["xy_mm"])):
        raise RuntimeError("等待输入期间机械臂姿态发生变化；本轮不发动作，请保留读数。")
    result["requested_delta_mm"] = [0, 5]
    result["expected_after_mm"] = [before["xy_mm"][0], before["xy_mm"][1] + 5]
    emit("ARM_BEFORE: {} mm".format(before["xy_mm"]))
    emit("EXPECTED_AFTER: {} mm（坐标预期，不是已测结果）".format(result["expected_after_mm"]))
    # Exactly one movement attempt, including if the SDK raises after sending.
    result["command_attempted"] = True
    result["command_started_monotonic_s"] = time.monotonic()
    result["command_started_utc"] = datetime.now(timezone.utc).isoformat()
    action = arm.move(x=0, y=5)
    if action is None or not callable(getattr(action, "wait_for_completed", None)):
        raise RuntimeError("动作已尝试发送，但 SDK 没返回可核对的 Action；不重试。")
    result["action_wait_returned"] = action.wait_for_completed(timeout=ACTION_TIMEOUT_S) is True
    result["action_succeeded"] = getattr(action, "has_succeeded", False) is True
    result["action_state"] = str(getattr(action, "state", "UNKNOWN"))
    result["action_wait_finished_monotonic_s"] = time.monotonic()
    result["action_elapsed_s"] = round(result["action_wait_finished_monotonic_s"] - result["command_started_monotonic_s"], 4)
    # The SDK object replaces its private x/y fields with decoded push data.
    # Keep its representation as diagnostic evidence only, never as proof of
    # the final measured position and never as a new movement target.
    result["sdk_action_snapshot"] = repr(action)
    emit("ACTION_COMPLETED: {}   ACTION_SUCCEEDED: {}".format(
        result["action_wait_returned"], result["action_succeeded"]))
    emit("ACTION_ELAPSED_S: {}".format(result["action_elapsed_s"]))
    emit("SDK_ACTION_SNAPSHOT: " + result["sdk_action_snapshot"])
    if not result["action_wait_returned"] or not result["action_succeeded"]:
        raise RuntimeError("SDK 未确认动作成功（{}）；超时不是停止确认，不继续发指令。".format(result["action_state"]))

    emit("继续观察 2 秒，再读取新的稳定位置；这段时间不会发送新动作。")
    result["post_action_observe_s"] = POST_ACTION_OBSERVE_S
    after = stable_reading(telemetry, after=time.monotonic() + POST_ACTION_OBSERVE_S,
                           timeout=POST_ACTION_OBSERVE_S + 4.0)
    result["after"] = after
    delta = [new - old for new, old in zip(after["xy_mm"], before["xy_mm"])]
    result["measured_delta_mm"] = delta
    result["position_check"] = abs(delta[0]) <= DELTA_TOL_MM and abs(delta[1] - 5) <= DELTA_TOL_MM
    emit("ARM_AFTER: {} mm".format(after["xy_mm"]))
    emit("ARM_DELTA: {} mm".format(delta))
    emit("POSITION_CHECK: {}（本次粗略位移核对）".format(result["position_check"]))
    result["operator_observed_up_and_stopped"] = ask_observation(read, emit)
    emit("OBSERVED_UP_AND_STOPPED: {}".format(result["operator_observed_up_and_stopped"]))
    result["status"] = ("ARM_STEP_OBSERVATIONS_RECORDED"
                        if result["position_check"] and result["operator_observed_up_and_stopped"] is True
                        else "NEEDS_REVIEW")
    emit("本轮已结束，不会自动回落或重复点动。请保留输出，不要反复启动累加位移。")


def run_connected(robot_module, sdk_config, sdk_conn, args, result, emit, read=input):
    robot = None
    subscriptions = []
    telemetry = TraceTelemetry()
    try:
        sdk_config.LOCAL_IP_STR = args.local_ip
        emit("正在连接并读取机械臂/电量；启动时不发送移动命令。")
        robot = robot_module.Robot()
        if robot.initialize(conn_type=sdk_conn.CONNECTION_WIFI_AP) is not True:
            raise RuntimeError("EP 初始化失败，请核对热点连接。")
        for module, subscribe, unsubscribe, name in (
            (robot.battery, "sub_battery_info", "unsub_battery_info", "battery"),
            (robot.robotic_arm, "sub_position", "unsub_position", "arm_position"),
        ):
            if getattr(module, subscribe)(freq=5, callback=telemetry.callback(name)) is not True:
                raise RuntimeError("订阅未确认：" + name)
            subscriptions.append((module, unsubscribe))
        # Allow AP telemetry discovery before asking for a movement.
        initial = stable_reading(telemetry, after=time.monotonic(), timeout=args.wait)
        result["connected_reading"] = initial
        console(robot.robotic_arm, telemetry, result, emit, read)
    except BaseException as exc:
        result["error"] = str(exc) or type(exc).__name__
        if result.get("command_attempted"):
            result["status"] = "NEEDS_REVIEW" if result.get("action_succeeded") else "MOTION_UNCONFIRMED"
            emit("本轮动作/观察未完成核对。若机械臂仍动、持续顶住或异常，请先关闭机器人电源。")
        else:
            result["status"] = "CANCELLED" if isinstance(exc, (KeyboardInterrupt, EOFError)) else "NOT_READY"
            emit("未发送机械臂移动命令。")
        raise
    finally:
        result["telemetry_at_exit"] = telemetry.snapshot()
        for module, unsubscribe in reversed(subscriptions):
            try:
                if getattr(module, unsubscribe)() is not True:
                    result.setdefault("cleanup_warnings", []).append(unsubscribe + " not acknowledged")
            except Exception as exc:
                result.setdefault("cleanup_warnings", []).append(str(exc))
        if robot is not None:
            try:
                robot.close()
                result["sdk_close_returned"] = True
                emit("DISCONNECTED")
            except Exception as exc:
                result["sdk_close_returned"] = False
                result.setdefault("cleanup_warnings", []).append(str(exc))
        for warning in result.get("cleanup_warnings", []):
            emit("退出提示：" + warning)
        result["arm_telemetry_trace"] = telemetry.trace_snapshot()


def main(argv=None):
    args = parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    run_dir = ROOT / "logs" / ("arm_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True)
    result = {"status": "FAILED", "arm_test_version": VERSION, "command_attempted": False,
              "utc_time": datetime.now(timezone.utc).isoformat(), "python": sys.version,
              "executable": sys.executable, "mode": "connect" if args.connect else "offline"}
    with (run_dir / "arm.log").open("w", encoding="utf-8") as log:
        def emit(message):
            print(message, flush=True)
            log.write(message + "\n")
            log.flush()
        emit("Arm 5mm test version: " + VERSION)
        emit("Log folder: " + str(run_dir))
        handler = None
        sdk_logger = None
        try:
            robot_module, sdk_config, sdk_conn = load_sdk()
            from robomaster.robotic_arm import RoboticArm
            from robomaster.action import Action
            if not callable(getattr(RoboticArm, "move", None)) or not hasattr(Action, "has_succeeded"):
                raise RuntimeError("SDK 缺少相对移动或动作结果接口。")
            if args.connect:
                sdk_logger = logging.getLogger("sdk")
                for existing in sdk_logger.handlers:
                    existing.setLevel(logging.WARNING)
                handler = logging.FileHandler(run_dir / "sdk.log", encoding="utf-8")
                handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
                sdk_logger.addHandler(handler)
                sdk_logger.setLevel(logging.INFO)
                run_connected(robot_module, sdk_config, sdk_conn, args, result, emit)
            else:
                result["status"] = "OFFLINE_OK"
                emit("SDK/API 导入检查通过；没有连接机器人。")
        except (KeyboardInterrupt, EOFError):
            if result["status"] == "FAILED":
                result["status"] = "CANCELLED"
            emit("已中断程序；这不构成物理停止确认。")
        except Exception as exc:
            result["error"] = str(exc)
            emit("ERROR: " + str(exc))
            log.write(traceback.format_exc())
        finally:
            if handler is not None:
                sdk_logger.removeHandler(handler)
                handler.close()
            (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        emit("RESULT: " + result["status"])
        emit("结果仅记录本次向上小步与观察，不代表完成抓取标定或验收。")
        emit("Result file: " + str(run_dir / "result.json"))
    return 0 if result["status"] in ("OFFLINE_OK", "NO_MOTION_REQUESTED", "ARM_STEP_OBSERVATIONS_RECORDED") else 1


if __name__ == "__main__":
    sys.exit(main())
