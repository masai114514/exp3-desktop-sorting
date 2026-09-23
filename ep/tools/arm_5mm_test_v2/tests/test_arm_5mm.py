"""All robot interfaces are fakes; these tests send no network commands."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_5mm_test as program
from check_ep import Telemetry


def reading(x=74, y=57, battery=53):
    return {"xy_mm": [x, y], "raw_xy_mm": [x, y], "samples_mm": [[x, y]] * 3,
            "battery_percent": battery, "received_at_monotonic": 100}


class FakeArm:
    def __init__(self, waited=True, succeeded=True, error=None):
        self.calls = []
        self.waits = []
        self.waited, self.succeeded, self.error = waited, succeeded, error

    def move(self, *, x, y):
        self.calls.append((x, y))
        if self.error:
            raise self.error
        def wait_for_completed(timeout):
            self.waits.append(timeout)
            return self.waited
        return SimpleNamespace(wait_for_completed=wait_for_completed,
                               has_succeeded=self.succeeded,
                               state="action_succeeded" if self.succeeded else "action_failed")


class WorkflowTests(unittest.TestCase):
    def run_console(self, answers, readings, arm=None):
        arm = arm or FakeArm()
        result = {}
        replies = iter(answers)
        with mock.patch.object(program, "stable_reading", side_effect=readings):
            program.console(arm, Telemetry(), result, lambda s: None, read=lambda s: next(replies))
        return arm, result

    def test_default_offline(self):
        self.assertFalse(program.parse_args([]).connect)

    def test_quit_and_unknown_input_never_move(self):
        arm, result = self.run_console(["", "o", "uu", "q"], [reading()])
        self.assertEqual(arm.calls, [])
        self.assertEqual(result["status"], "NO_MOTION_REQUESTED")

    def test_one_relative_step_then_observation_no_auto_return(self):
        arm, result = self.run_console(["u", "y"], [reading(), reading(), reading(y=62)])
        self.assertEqual(arm.calls, [(0, 5)])
        self.assertEqual(arm.waits, [10])
        self.assertEqual(result["expected_after_mm"], [74, 62])
        self.assertEqual(result["measured_delta_mm"], [0, 5])
        self.assertEqual(result["status"], "ARM_STEP_OBSERVATIONS_RECORDED")

    def test_action_wait_true_but_failed_is_not_success(self):
        arm = FakeArm(waited=True, succeeded=False)
        with self.assertRaisesRegex(RuntimeError, "未确认动作成功"):
            self.run_console(["u"], [reading(), reading()], arm)
        self.assertEqual(arm.calls, [(0, 5)])

    def test_timeout_does_not_retry_or_send_home(self):
        arm = FakeArm(waited=False, succeeded=False)
        with self.assertRaisesRegex(RuntimeError, "超时不是停止"):
            self.run_console(["u"], [reading(), reading()], arm)
        self.assertEqual(arm.calls, [(0, 5)])

    def test_no_readback_change_is_needs_review(self):
        arm, result = self.run_console(["u", "y"], [reading(), reading(), reading()])
        self.assertEqual(arm.calls, [(0, 5)])
        self.assertFalse(result["position_check"])
        self.assertEqual(result["status"], "NEEDS_REVIEW")

    def test_wrong_direction_or_large_displacement_never_passes(self):
        for after in (reading(y=52), reading(y=82), reading(x=80, y=62)):
            with self.subTest(after=after):
                arm, result = self.run_console(["u", "y"], [reading(), reading(), after])
                self.assertEqual(arm.calls, [(0, 5)])
                self.assertEqual(result["status"], "NEEDS_REVIEW")

    def test_operator_no_or_uncertain_never_passes(self):
        for answer in ("n", "u"):
            with self.subTest(answer=answer):
                arm, result = self.run_console(["u", answer], [reading(), reading(), reading(y=62)])
                self.assertEqual(arm.calls, [(0, 5)])
                self.assertEqual(result["status"], "NEEDS_REVIEW")

    def test_initial_bad_pose_or_battery_blocks_motion(self):
        for start in (reading(x=-74, y=-57), reading(x=74, y=70), reading(battery=29)):
            with self.subTest(start=start):
                arm = FakeArm()
                with self.assertRaises(RuntimeError):
                    self.run_console(["u"], [start], arm)
                self.assertEqual(arm.calls, [])

    def test_input_wait_rechecks_staleness_battery_and_pose(self):
        for new in (reading(battery=29), reading(y=70), RuntimeError("stale")):
            with self.subTest(new=new):
                arm = FakeArm()
                with self.assertRaises(RuntimeError):
                    self.run_console(["u"], [reading(), new], arm)
                self.assertEqual(arm.calls, [])

    def test_missing_post_action_samples_ends_without_retry(self):
        arm = FakeArm()
        with self.assertRaises(RuntimeError):
            self.run_console(["u"], [reading(), reading(), RuntimeError("missing telemetry")], arm)
        self.assertEqual(arm.calls, [(0, 5)])

    def test_send_exception_or_interrupt_does_not_retry(self):
        for error in (RuntimeError("send lost"), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                arm = FakeArm(error=error)
                with self.assertRaises(type(error)):
                    self.run_console(["u"], [reading(), reading()], arm)
                self.assertEqual(arm.calls, [(0, 5)])


class FakeClock:
    def __init__(self, callback=None):
        self.value = 100.0
        self.callback = callback
        self.ticks = 0

    def now(self):
        return self.value

    def sleep(self, duration):
        self.value += duration
        self.ticks += 1
        if self.callback:
            self.callback(self.ticks)


class FreshnessTests(unittest.TestCase):
    def evaluate(self, telemetry, clock, after=100):
        with mock.patch.object(program.time, "monotonic", clock.now), mock.patch.object(program.time, "sleep", clock.sleep):
            return program.stable_reading(telemetry, after=after, timeout=0.6)

    def test_unchanged_cached_frame_cannot_count_as_multiple_new_samples(self):
        telemetry, clock = Telemetry(), FakeClock()
        with mock.patch.object(program.time, "monotonic", clock.now):
            for _ in range(3):
                telemetry.callback("battery")(53)
                telemetry.callback("arm_position")([74, 57])
        with self.assertRaises(RuntimeError):
            self.evaluate(telemetry, clock, after=99)

    def test_pre_action_samples_cannot_supply_post_action_reading(self):
        telemetry, clock = Telemetry(), FakeClock()
        with mock.patch.object(program.time, "monotonic", clock.now):
            for _ in range(3):
                telemetry.callback("battery")(53)
                telemetry.callback("arm_position")([74, 57])
        with self.assertRaises(RuntimeError):
            self.evaluate(telemetry, clock, after=100)

    def test_new_stable_samples_are_collected(self):
        telemetry = Telemetry()
        def feed(tick):
            telemetry.callback("battery")(53)
            telemetry.callback("arm_position")([74, 62])
        actual = self.evaluate(telemetry, FakeClock(feed))
        self.assertEqual(actual["xy_mm"], [74, 62])
        self.assertEqual(len(actual["samples_mm"]), 3)

    def test_unstable_or_invalid_samples_are_rejected(self):
        for invalid in (False, True):
            with self.subTest(invalid=invalid):
                telemetry = Telemetry()
                def feed(tick):
                    telemetry.callback("battery")(53)
                    telemetry.callback("arm_position")([74, float("nan") if invalid else 57 + (tick % 2) * 5])
                with self.assertRaises(RuntimeError):
                    self.evaluate(telemetry, FakeClock(feed))


class SessionTests(unittest.TestCase):
    def make_robot(self, arm):
        calls = []
        arm.sub_position = lambda **kwargs: True
        arm.unsub_position = lambda: calls.append("unsub_arm") or True
        class Robot:
            battery = SimpleNamespace(sub_battery_info=lambda **kwargs: True,
                                      unsub_battery_info=lambda: calls.append("unsub_battery") or True)
            robotic_arm = arm
            def initialize(self, **kwargs):
                calls.append("initialize")
                return True
            def close(self):
                calls.append("disconnect")
            def __getattr__(self, name):
                raise AssertionError("Unexpected module access: " + name)
        return Robot(), calls

    def run_session(self, robot, result, readings, answers):
        answers = iter(answers)
        with mock.patch.object(program, "stable_reading", side_effect=readings):
            program.run_connected(SimpleNamespace(Robot=lambda: robot), SimpleNamespace(),
                                  SimpleNamespace(CONNECTION_WIFI_AP="ap"),
                                  SimpleNamespace(local_ip=None, wait=15), result, lambda s: None,
                                  read=lambda s: next(answers))

    def test_success_session_uses_arm_only_and_closes(self):
        arm, result = FakeArm(), {}
        robot, calls = self.make_robot(arm)
        self.run_session(robot, result, [reading(), reading(), reading(), reading(y=62)], ["u", "y"])
        self.assertEqual(arm.calls, [(0, 5)])
        self.assertEqual(calls[-3:], ["unsub_arm", "unsub_battery", "disconnect"])
        self.assertEqual(result["status"], "ARM_STEP_OBSERVATIONS_RECORDED")

    def test_timeout_and_interrupt_stay_unconfirmed_and_close(self):
        for arm in (FakeArm(waited=False, succeeded=False), FakeArm(error=KeyboardInterrupt())):
            with self.subTest(arm=arm):
                result = {}
                robot, calls = self.make_robot(arm)
                with self.assertRaises((RuntimeError, KeyboardInterrupt)):
                    self.run_session(robot, result, [reading(), reading(), reading()], ["u"])
                self.assertEqual(result["status"], "MOTION_UNCONFIRMED")
                self.assertEqual(arm.calls, [(0, 5)])
                self.assertEqual(calls[-1], "disconnect")

    def test_connection_readiness_failure_closes_without_moving(self):
        arm, result = FakeArm(), {}
        robot, calls = self.make_robot(arm)
        with self.assertRaises(RuntimeError):
            self.run_session(robot, result, [RuntimeError("no arm data")], [])
        self.assertEqual(result["status"], "NOT_READY")
        self.assertEqual(arm.calls, [])
        self.assertEqual(calls[-1], "disconnect")


if __name__ == "__main__":
    unittest.main()
