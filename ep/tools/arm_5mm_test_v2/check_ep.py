#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone RoboMaster EP AP connection/telemetry checker.

Default: offline SDK import and API inspection. Explicit --connect is required
for robot communication. This program has no chassis/arm/gripper motion path.
Uses only the local vendor directory; installs nothing into Python environments.
"""
import argparse
from datetime import datetime, timezone
import ipaddress
import json
import logging
import math
from pathlib import Path
import platform
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parent
CHECKER_VERSION = "2"
REQUIRED = ("battery", "chassis_position", "chassis_attitude", "arm_position")
# Gross data-integrity screen only, NOT a calibrated travel/reach/safety limit.
ARM_DIAGNOSTIC_ABS_MM = 1000


def decode_arm_position(value):
    """Interpret SDK uint32 coordinates as signed int32, retaining raw values.

    Bundled ArmSubject.decode uses struct.unpack('<II'). Negative millimetre
    coordinates consequently appear near 2**32. Accept signed callbacks too;
    apply the two's-complement interpretation only to the uint32 upper half.
    """
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError("arm position must contain two coordinates")
    raw, signed = [], []
    for coordinate in value:
        if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)):
            raise ValueError("arm coordinates must be numeric integer millimetres")
        if not math.isfinite(coordinate) or int(coordinate) != coordinate:
            raise ValueError("arm coordinates must be finite integer millimetres")
        original = int(coordinate)
        if not -(2**31) <= original <= 2**32 - 1:
            raise ValueError("arm coordinate is outside int32/uint32 encoding")
        decoded = original - 2**32 if original >= 2**31 else original
        if abs(decoded) > ARM_DIAGNOSTIC_ABS_MM:
            raise ValueError(
                "implausible arm position: raw={} signed={} mm; diagnostic screen +/-{} mm"
                .format(original, decoded, ARM_DIAGNOSTIC_ABS_MM))
        raw.append(original)
        signed.append(decoded)
    return raw, signed


def load_sdk():
    if not (3, 9) <= sys.version_info[:2] <= (3, 12):
        raise RuntimeError("Use Python 3.9-3.12; your existing Python 3.11 is suitable.")
    vendor = ROOT / "vendor"
    wheel = vendor / "netaddr-1.3.0-py3-none-any.whl"
    if not wheel.is_file() or not (vendor / "robomaster" / "robot.py").is_file():
        raise RuntimeError("Missing vendor files. Extract the entire ZIP before running.")
    sys.path[:0] = [str(vendor), str(wheel)]
    from robomaster import robot, chassis, robotic_arm, gripper, battery, config, conn
    expected = vendor / "robomaster" / "robot.py"
    if Path(robot.__file__).resolve() != expected.resolve():
        raise RuntimeError("Unexpected SDK import location; run this in a fresh Python process.")
    for cls, methods in (
        (chassis.Chassis, ("sub_position", "unsub_position", "sub_attitude", "unsub_attitude")),
        (robotic_arm.RoboticArm, ("sub_position", "unsub_position")),
        (gripper.Gripper, ("sub_status", "unsub_status")),
        (battery.Battery, ("sub_battery_info", "unsub_battery_info")),
    ):
        for name in methods:
            if not callable(getattr(cls, name, None)):
                raise RuntimeError("Missing SDK interface: {}.{}".format(cls.__name__, name))
    # Import inspection deliberately does not instantiate Robot or open sockets.
    return robot, config, conn


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {}
        self.invalid = {}

    def callback(self, name):
        def receive(value, *args, **kwargs):
            try:
                extra = {}
                if name == "gripper_status":
                    if value not in ("opened", "closed", "normal"):
                        raise ValueError("unknown gripper state")
                elif name == "battery":
                    value = float(value)
                    if not math.isfinite(value) or not 0 <= value <= 100:
                        raise ValueError("battery must be 0..100")
                elif name == "arm_position":
                    raw, value = decode_arm_position(value)
                    extra = {"raw_value": raw, "encoding": "signed_int32_mm",
                             "unsigned_wrap_corrected": raw != value}
                else:
                    value = [float(v) for v in value]
                    if len(value) != 3 or not all(math.isfinite(v) for v in value):
                        raise ValueError("invalid position/attitude tuple")
                with self.lock:
                    previous = self.data.get(name, {})
                    self.data[name] = {
                        "value": value,
                        "received_at": time.monotonic(),
                        "count": previous.get("count", 0) + 1,
                        **extra,
                    }
                    self.invalid.pop(name, None)
            except (TypeError, ValueError, OverflowError) as exc:
                with self.lock:
                    self.invalid[name] = str(exc)
                    # A corrupt latest sample must not reuse a previous good one.
                    self.data.pop(name, None)
        return receive

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            return {
                name: {**{key: value for key, value in item.items() if key != "received_at"},
                       "age_seconds": round(max(0, now - item["received_at"]), 3)}
                for name, item in self.data.items()
            }

    def missing(self):
        data = self.snapshot()
        return [name for name in REQUIRED if name not in data
                or data[name]["count"] < 2 or data[name]["age_seconds"] > 2.0]


def connect_and_read(robot_module, sdk_config, sdk_conn, args, result, emit):
    telemetry = Telemetry()
    subscriptions = []
    robot = None
    try:
        sdk_config.LOCAL_IP_STR = args.local_ip
        emit("Connecting to EP in AP mode. No movement or gripper open/close commands are sent.")
        robot = robot_module.Robot()
        if robot.initialize(conn_type=sdk_conn.CONNECTION_WIFI_AP) is not True:
            raise RuntimeError("SDK initialization failed. Check EP Wi-Fi and AP mode.")
        result["sdk_initialized"] = True
        emit("CONNECTED (initialization returned; waiting for actual telemetry)")
        requests = (
            ("battery", robot.battery, "sub_battery_info", "unsub_battery_info", {}),
            ("chassis_position", robot.chassis, "sub_position", "unsub_position", {"cs": 1}),
            ("chassis_attitude", robot.chassis, "sub_attitude", "unsub_attitude", {}),
            ("arm_position", robot.robotic_arm, "sub_position", "unsub_position", {}),
            ("gripper_status", robot.gripper, "sub_status", "unsub_status", {}),
        )
        for name, module, subscribe, unsubscribe, extras in requests:
            try:
                accepted = getattr(module, subscribe)(
                    freq=5, callback=telemetry.callback(name), **extras)
                if accepted is not True:
                    raise RuntimeError("subscription not acknowledged")
                subscriptions.append((module, unsubscribe))
                emit("SUBSCRIBED: " + name)
            except Exception as exc:
                result.setdefault("subscription_errors", {})[name] = str(exc)
                emit("SUBSCRIPTION_FAILED: {}: {}".format(name, exc))

        deadline = time.monotonic() + args.wait
        last_message = 0.0
        while telemetry.missing() and time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_message >= 3:
                emit("Waiting for: " + ", ".join(telemetry.missing()))
                last_message = now
            time.sleep(0.1)

        snapshot = telemetry.snapshot()
        result["telemetry"] = snapshot
        with telemetry.lock:
            result["invalid_telemetry"] = dict(telemetry.invalid)
        for name in (*REQUIRED, "gripper_status"):
            item = snapshot.get(name)
            if name == "arm_position" and item:
                emit("ARM_POSITION_RAW: {}".format(item["raw_value"]))
            emit("{}: {}".format(name.upper(), item["value"] if item else "NOT_RECEIVED"))
            if name == "arm_position" and item:
                emit("ARM_POSITION_UNIT: mm (signed int32; not a calibrated target)")
                if item["unsigned_wrap_corrected"]:
                    emit("ARM_DECODE: uint32 wrap converted to signed int32; raw values retained")
        for name, error in result["invalid_telemetry"].items():
            emit("INVALID_{}: {}".format(name.upper(), error))
        missing = telemetry.missing()
        if missing:
            raise RuntimeError("Missing or stale telemetry: " + ", ".join(missing))
        if "gripper_status" not in snapshot:
            emit("NOTE: No gripper status received. Its operation remains unverified.")
        result["status"] = "CONNECT_OK"
        emit("RESULT: CONNECT_OK")
        emit("Connection and required telemetry only. Calibration/grasping have NOT been tested.")
    finally:
        result.setdefault("telemetry", telemetry.snapshot())
        for module, unsubscribe in reversed(subscriptions):
            try:
                getattr(module, unsubscribe)()
            except Exception as exc:
                emit("Cleanup warning: {}: {}".format(unsubscribe, exc))
        if robot is not None:
            try:
                robot.close()
                emit("DISCONNECTED")
            except Exception as exc:
                emit("Disconnect warning: " + str(exc))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--offline", action="store_true", help="check imports only (default)")
    mode.add_argument("--connect", action="store_true", help="connect to EP AP and read telemetry")
    parser.add_argument("--wait", type=float, default=15, help="telemetry timeout, 1..60 seconds")
    parser.add_argument("--local-ip", default=None, help="optional laptop Wi-Fi IPv4, 192.168.2.x")
    args = parser.parse_args(argv)
    if not 1 <= args.wait <= 60 or not math.isfinite(args.wait):
        parser.error("--wait must be between 1 and 60 seconds")
    if args.local_ip:
        try:
            addr = ipaddress.IPv4Address(args.local_ip)
            net = ipaddress.IPv4Network("192.168.2.0/24")
            if addr not in net or str(addr) in ("192.168.2.0", "192.168.2.1", "192.168.2.255"):
                raise ValueError("expected laptop address in 192.168.2.2..254")
        except ValueError as exc:
            parser.error("--local-ip: " + str(exc))
    return args


def main(argv=None):
    args = parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    run_dir = ROOT / "logs" / ("check_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True)
    result = {
        "status": "FAILED", "mode": "connect" if args.connect else "offline",
        "checker_version": CHECKER_VERSION,
        "utc_time": datetime.now(timezone.utc).isoformat(),
        "python": sys.version, "executable": sys.executable, "platform": platform.platform(),
        "motion_commands_sent_by_checker": False,
    }
    exit_code = 1
    with (run_dir / "check.log").open("w", encoding="utf-8") as log:
        def emit(message):
            print(message, flush=True)
            log.write(message + "\n")
            log.flush()
        emit("Checker version: " + CHECKER_VERSION + " (signed arm telemetry)")
        emit("Python: " + sys.version.split()[0])
        emit("Interpreter: " + sys.executable)
        emit("Log folder: " + str(run_dir))
        handler = None
        sdk_logger = None
        try:
            robot_module, sdk_config, sdk_conn = load_sdk()
            emit("SDK_OK: " + robot_module.__file__)
            emit("API_OK: required subscription methods are present")
            if not args.connect:
                result["status"] = "OFFLINE_OK"
                emit("RESULT: OFFLINE_OK (no robot connection was attempted)")
            else:
                sdk_logger = logging.getLogger("sdk")
                for existing in sdk_logger.handlers:
                    existing.setLevel(logging.WARNING)
                handler = logging.FileHandler(run_dir / "sdk.log", encoding="utf-8")
                handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
                sdk_logger.addHandler(handler)
                sdk_logger.setLevel(logging.INFO)
                connect_and_read(robot_module, sdk_config, sdk_conn, args, result, emit)
            exit_code = 0
        except KeyboardInterrupt:
            result["status"] = "CANCELLED"
            emit("RESULT: CANCELLED")
            exit_code = 130
        except Exception as exc:
            result["status"] = "FAILED"
            result["error"] = str(exc)
            emit("RESULT: FAILED: " + str(exc))
            log.write(traceback.format_exc())
        finally:
            if handler is not None:
                sdk_logger.removeHandler(handler)
                handler.close()
            (run_dir / "result.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        emit("Result file: " + str(run_dir / "result.json"))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
