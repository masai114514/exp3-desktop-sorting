#!/usr/bin/env python3
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor


def _source_roots():
    exp3_root = os.environ.get('EXP3_ROOT')
    c4_root = os.environ.get('C4_ROOT')
    if not exp3_root or not c4_root:
        raise RuntimeError('EXP3_ROOT and C4_ROOT must be set by exp3_sim.launch.py')
    sys.path.insert(0, exp3_root)
    sys.path.insert(0, os.path.join(c4_root, 'drive'))
    sys.path.insert(0, os.path.join(c4_root, 'core'))
    return exp3_root, c4_root


class _DriverLog:
    def __init__(self, node):
        self.node = node

    def text(self, message):
        self.node.get_logger().info(str(message))

    def sample(self, *args):
        return None


def _load_json(path):
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def _rotation_to_quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        return ((matrix[2, 1] - matrix[1, 2]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[1, 0] - matrix[0, 1]) / s,
                0.25 * s)
    index = int(np.argmax(np.diag(matrix)))
    if index == 0:
        s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return (0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
                (matrix[2, 1] - matrix[1, 2]) / s)
    if index == 1:
        s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return ((matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s,
                (matrix[1, 2] + matrix[2, 1]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s)
    s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return ((matrix[0, 2] + matrix[2, 0]) / s,
            (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s,
            (matrix[1, 0] - matrix[0, 1]) / s)


def _object_sdf(name, cls_name):
    if cls_name == 'cup':
        geometry = '<cylinder><radius>0.020</radius><length>0.035</length></cylinder>'
        mass = '0.04'
        color = '0.95 0.08 0.035 1'
    else:
        geometry = '<box><size>0.040 0.025 0.020</size></box>'
        mass = '0.035'
        color = '0.035 0.17 0.95 1'
    return ('<sdf version="1.6"><model name="{name}"><link name="body">'
            '<inertial><mass>{mass}</mass><inertia><ixx>0.00001</ixx>'
            '<iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>'
            '<collision name="collision"><geometry>{geometry}</geometry></collision>'
            '<visual name="visual"><geometry>{geometry}</geometry><material>'
            '<ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'
            '</link></model></sdf>').format(name=name, mass=mass,
                                             geometry=geometry, color=color)


def _spawn_scene(driver, base_z):
    client = driver.create_client(SpawnEntity, '/spawn_entity')
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('/spawn_entity service is unavailable')
    objects = [
        ('block_c1', 'cup', 0.09, -0.08, 0.0175),
        ('block_c2', 'mouse', 0.09, 0.00, 0.0100),
        ('block_c3', 'cup', 0.09, 0.08, 0.0175),
        ('block_c4', 'cup', 0.14, -0.08, 0.0175),
        ('block_c5', 'mouse', 0.14, 0.00, 0.0100),
        ('block_c6', 'cup', 0.14, 0.08, 0.0175),
    ]
    for name, cls_name, x, y, object_z in objects:
        request = SpawnEntity.Request()
        request.name = name
        request.xml = _object_sdf(name, cls_name)
        request.reference_frame = 'world'
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = base_z + object_z
        pose.orientation.w = 1.0
        request.initial_pose = pose
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(20.0):
            raise RuntimeError('spawn timed out for ' + name)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('spawn failed for %s: %s' %
                               (name, getattr(response, 'status_message', 'no response')))
        driver.get_logger().info('spawned %s (%s) after safe-arm positioning' %
                                 (name, cls_name))


class GazeboFollowExecutor:
    """C4 executor with explicit SetModelState following; this is not ODE grasping."""

    def __init__(self, driver, motion, base_z=0.8):
        from ros2.task_control.c4_executor import C4PickExecutor
        self._base = C4PickExecutor
        self._delegate = C4PickExecutor(driver, motion)
        self.r = driver
        self.base_z = float(base_z)
        self._lock = threading.RLock()
        self._states = {}
        self._attached = None
        self._offset = None
        self._initial_z = None
        self._follow_stop = threading.Event()
        self._follow_thread = None
        self._model_sub = driver.create_subscription(
            ModelStates, '/gazebo/model_states', self._on_model_states, 10)
        self._state_client = driver.create_client(SetEntityState, '/gazebo/set_entity_state')
        if not self._state_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/gazebo/set_entity_state service is unavailable')

    def __getattr__(self, name):
        return getattr(self._delegate, name)

    def _on_model_states(self, msg):
        with self._lock:
            self._states = {
                name: (pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w)
                for name, pose in zip(msg.name, msg.pose)
            }

    def _state(self, name):
        with self._lock:
            return self._states.get(name)

    def _wait_state(self, name, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self._state(name)
            if state is not None:
                return state
            time.sleep(0.05)
        return None

    def _set_model(self, name, xyz, quaternion):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x, state.pose.position.y, state.pose.position.z = xyz
        (state.pose.orientation.x, state.pose.orientation.y,
         state.pose.orientation.z, state.pose.orientation.w) = quaternion
        req.state = state
        future = self._state_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(1.0):
            return False
        try:
            response = future.result()
        except Exception:
            return False
        return bool(response and response.success)

    def _attach(self, ctx):
        from fk import midpose
        name = 'block_' + ctx['cell_id']
        state = self._wait_state(name)
        if state is None:
            return False, 'model_state_missing:' + name
        q = self.r.arm_q()
        if len(q) != 6 or not np.all(np.isfinite(q)):
            return False, 'joint_state_invalid'
        tool = midpose(q)
        object_local = np.array([state[0], state[1], state[2] - self.base_z])
        offset = tool[:3, :3].T @ (object_local - tool[:3, 3])
        with self._lock:
            self._attached = name
            self._offset = offset
            self._initial_z = state[2]
            self._object_quaternion = state[3:7]
        self._follow_stop.clear()
        self._follow_thread = threading.Thread(target=self._follow, daemon=True)
        self._follow_thread.start()
        return True, 'script_follow_enabled_not_ODE_grasp'

    def _follow(self):
        from fk import midpose
        while not self._follow_stop.wait(0.05):
            with self._lock:
                name, offset = self._attached, self._offset
                quaternion = getattr(self, '_object_quaternion', (0, 0, 0, 1))
            if not name or offset is None:
                continue
            q = self.r.arm_q()
            if len(q) != 6 or not np.all(np.isfinite(q)):
                continue
            tool = midpose(q)
            pos = tool[:3, 3] + tool[:3, :3] @ offset
            xyz = (float(pos[0]), float(pos[1]), float(pos[2] + self.base_z))
            self._set_model(name, xyz, quaternion)

    def _stop_follow(self):
        self._follow_stop.set()
        if self._follow_thread is not None:
            self._follow_thread.join(timeout=2.0)
        with self._lock:
            self._attached = None
            self._offset = None

    def descend(self, ctx):
        return self._delegate.descend(ctx)

    def grasp(self, ctx):
        ok, note = self._delegate.grasp(ctx)
        if not ok:
            return ok, note
        attached, attach_note = self._attach(ctx)
        return attached, attach_note

    def lift(self, ctx):
        _, _, lift_target, _ = self._delegate._targets(ctx)
        attempts = 1 + self._delegate._lift_retry
        for attempt in range(attempts):
            ok, _ = self.r.go_cartesian(lift_target, 'LIFT_A')
            if not ok:
                return False, 'lift_planning_or_execution_failed'
            held, note = self._judge_hold(ctx)
            if held:
                return True, note
            if attempt < attempts - 1 and self._delegate._close is not None:
                self.r.grip_set(self._delegate._close, 're-PRESS', settle=0.5)
        return False, 'not_held:' + note

    def _judge_hold(self, ctx):
        state = self._state('block_' + ctx['cell_id'])
        if state is None or self._initial_z is None:
            return False, 'model_state_missing'
        delta = state[2] - self._initial_z
        return delta >= 0.035, 'script_follow_dz=%.3f' % delta

    def carry(self, ctx):
        return self._delegate.carry(ctx)

    def release(self, ctx):
        opened = self._delegate.r.grip_set(self._delegate._open, 'RELEASE',
                                            settle=self._delegate._release_settle)
        if not opened[0]:
            return False, 'gripper_release_failed'
        self._stop_follow()
        time.sleep(0.8)
        return self._judge_place(ctx)

    def _judge_place(self, ctx):
        state = self._wait_state('block_' + ctx['cell_id'], timeout=3.0)
        if state is None:
            return False, 'model_state_missing_after_release'
        dx = abs(state[0] - ctx['B']['x'])
        dy = abs(state[1] - ctx['B']['y'])
        ok = dx <= 0.043 and dy <= 0.031 and state[2] <= self.base_z + 0.10
        return ok, 'script_place_error=(%.3f,%.3f,%.3f)' % (dx, dy, state[2] - self.base_z)

    def retract(self, ctx):
        return self._delegate.retract(ctx)

    def close(self):
        self._stop_follow()


def main(args=None):
    exp3_root, c4_root = _source_roots()
    from c4_cycle import C4Driver
    from rclpy.node import Node
    from ros2.task_control.pick_place_server import PickPlaceServer
    from sort_core.config import load_bins, load_grid

    rclpy.init(args=args)
    init_node = Node('exp3_sim_server_startup')
    init_node.declare_parameter('base_z', 0.8)
    base_z = float(init_node.get_parameter('base_z').value)
    cfg = _load_json(os.path.join(c4_root, 'config_sim.json'))
    cfg['backend'] = {'type': 'manual'}
    cfg['timing'].update({'cart_pt_t': 0.15, 'descend_pt_t': 0.6,
                          'gripper_t': 0.5, 'press_settle': 0.5,
                          'release_settle': 0.5})
    cfg['planner'].update({'step': 0.006, 'cartesian_rpy': [0.0, 0.0, 0.0]})

    driver = C4Driver(cfg, _DriverLog(init_node))
    driver_executor = MultiThreadedExecutor(num_threads=4)
    driver_executor.add_node(driver)
    driver_thread = threading.Thread(target=driver_executor.spin, daemon=True)
    driver_thread.start()
    follow = None
    server = None
    try:
        deadline = time.monotonic() + 300.0
        next_notice = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            arm_ready = driver.arm.wait_for_server(timeout_sec=1.0)
            grip_ready = driver.grp.wait_for_server(timeout_sec=1.0)
            if arm_ready and grip_ready:
                break
            if time.monotonic() >= next_notice:
                init_node.get_logger().info('Waiting for Gazebo arm/gripper controllers...')
                next_notice = time.monotonic() + 15.0
        else:
            raise RuntimeError('timed out waiting for arm/gripper action servers')
        driver._wait_joint_state(timeout=30.0)
        opened, status = driver.grip_set(cfg['gripper']['open'], 'EXP3_START_OPEN', settle=0.2)
        if not opened:
            raise RuntimeError('could not open gripper at startup: %s' % status)
        safe_pose = cfg['descend_a_joints'][-1]
        ok, status, _ = driver.go_chain([safe_pose], 'EXP3_SAFE_APPROACH')
        if not ok:
            raise RuntimeError('safe approach failed: %s' % status)
        ok, _ = driver.go_cartesian((0.09, -0.08, 0.138), 'EXP3_CLEAR_ABOVE_GRID')
        if not ok:
            raise RuntimeError('could not clear the grid before spawning objects')
        motion = {
            'home': None,
            'gripper_open': cfg['gripper']['open'],
            'gripper_close': cfg['gripper']['close'],
            'grasp_z': cfg['task']['grasp_mid_z'],
            'lift_dz': cfg['task']['lift_dz'],
            'place_z': cfg['task']['grasp_mid_z'],
            'lift_retry': 0,
            'press_settle': 0.5,
            'release_settle': 0.5,
            'hold_assume': False,
            'place_assume': False,
        }
        follow = GazeboFollowExecutor(driver, motion, base_z=base_z)
        _spawn_scene(driver, base_z)
        time.sleep(1.0)
        server = PickPlaceServer(load_grid(), load_bins(), follow)
        init_node.get_logger().info('PickPlace ready; script-follow attachment is enabled and disclosed')
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        if follow is not None:
            follow.close()
        if server is not None:
            server.destroy_node()
        driver_executor.shutdown(timeout_sec=3.0)
        driver.destroy_node()
        init_node.destroy_node()
        rclpy.shutdown()
