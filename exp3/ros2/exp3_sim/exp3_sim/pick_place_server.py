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

# 摆放表 + 异常轮空格剔除 + carry_z 阶梯（无 ROS import，可离线单测）
from exp3_sim.scene import carry_z_candidates, scene_objects


def _source_roots():
    exp3_root = os.environ.get('EXP3_ROOT')
    c4_root = os.environ.get('C4_ROOT')
    if not exp3_root or not c4_root:
        raise RuntimeError('EXP3_ROOT and C4_ROOT must be set by exp3_sim.launch.py')
    sys.path.insert(0, exp3_root)
    sys.path.insert(0, os.path.join(c4_root, 'drive'))
    sys.path.insert(0, os.path.join(c4_root, 'core'))
    return exp3_root, c4_root


# 扫描停靠位：把底座关节 j1 旋到 base 反面，使整条臂的**投影**离开桌面网格，
# 这样 6 个格子才全部可见。依据 dock_scan.py 的离线评估（同一份 fk.py + 相机标定）：
#   - 原 safe_pose（cfg['descend_a_joints'][-1]，j1=0）时，7 条臂段里 **5 条**穿进
#     "网格 + 物块半径 + 40px 余量"的投影矩形，刀尖 (0.150,0,0.038) 正落在网格内；
#   - j1=-2.6（仍在 LIMITS[0]=±2.79 内）时 **0/7** 命中，刀尖投影 (18,146) 完全出网格。
# 实测对照（scan_probe.py）：原停靠位下 /detections_d2a 只有 5 条 —— c4 完全漏检、
# c1 被压成 27x52(期望 47.2x46.5) 且中心右偏 4.5px，而 c2/c3/c5/c6 尺寸全部正常，
# 标定投影 6/6 落在正确格子 ⇒ 漏检的唯一变量就是机械臂自身的视觉几何。
DOCK_J1 = -2.6


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
    """物块 SDF。

    ★ collision 里必须给 ODE 接触参数，否则 release 后物块落到料盒底那一下的
    高速冲击会把 LCP 求解器打爆（实测 `ODE Message 3: LCP internal error, s <= 0`
    冲到 s=-4.8e+07），物块被弹飞到空中 —— `_judge_place` 于是读到
    script_place_error=(0.000,0.000,0.138)：x/y 精确落在料盒中心、z 却高 0.138。
    max_vel 限制接触点速度、min_depth 允许 2 mm 穿透，两者合起来即可抑制该发散。
    """
    if cls_name == 'cup':
        geometry = '<cylinder><radius>0.020</radius><length>0.035</length></cylinder>'
        mass = '0.04'
        color = '0.95 0.08 0.035 1'
    else:
        geometry = '<box><size>0.040 0.025 0.020</size></box>'
        mass = '0.035'
        color = '0.035 0.17 0.95 1'
    contact = ('<surface><contact><ode><max_vel>0.05</max_vel>'
               '<min_depth>0.002</min_depth></ode></contact></surface>')
    return ('<sdf version="1.6"><model name="{name}"><link name="body">'
            '<inertial><mass>{mass}</mass><inertia><ixx>0.00001</ixx>'
            '<iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>'
            '<collision name="collision"><geometry>{geometry}</geometry>'
            '{contact}</collision>'
            '<visual name="visual"><geometry>{geometry}</geometry><material>'
            '<ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'
            '</link></model></sdf>').format(name=name, mass=mass,
                                            geometry=geometry, color=color,
                                            contact=contact)


def _spawn_scene(driver, base_z, empty_cell=''):
    """摆放物块；empty_cell 指定某一格【故意不放】（异常轮）。

    ★ 摆放表与空格的剔除逻辑在 `exp3_sim/scene.py`（无 ROS import，可离线单测）：
      本文件 import rclpy，进不了离线套件；而"空格必须真的空"是个判据，值得被
      离线用例锁住。这里只负责调 spawn service。

    ★ 异常轮 verdict 必然是 TRIAL（场上 5 个 < 验收线 6 个），是"样本不足不能判定"
      的正确语义，必须与记分轮分开跑 —— 详见 scene.scene_objects 的 docstring。
    """
    client = driver.create_client(SpawnEntity, '/spawn_entity')
    if not client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('/spawn_entity service is unavailable')
    objects = scene_objects(empty_cell)
    skip = str(empty_cell or '').strip()
    if skip:
        driver.get_logger().info(
            'ANOMALY ROUND: cell %s intentionally left empty, %d objects on table '
            '(expect verdict=TRIAL and zero records for %s)'
            % (skip, len(objects), skip))
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

    def __init__(self, driver, motion, base_z=0.8, approach=None, dock=None):
        from ros2.task_control.c4_executor import C4PickExecutor
        self._base = C4PickExecutor
        self._delegate = C4PickExecutor(driver, motion)
        self.r = driver
        self.base_z = float(base_z)
        self._approach = approach      # 离开停靠位的中转关节列（None=不中转）
        # ★ 每轮结束后**强制**回到的停靠位（j1=DOCK_J1，整条臂在 base 反面）。
        #   为什么必须在这里兜底而不是依赖 c4_executor.retract 的 motion['home']：
        #   实测 run_20260924_104207，retract 之后日志里只出现过一个 JTC goal，
        #   臂停在 A_above（**网格正上方**）就以 success 返回了 —— 于是下一轮扫描
        #   时整条臂横在网格上方，桌上剩余物块全部映射失败（out_of_grid），
        #   连续 3 轮无可做目标 ⇒ no_exec，placed_ok 卡在 3。
        #   这里显式发一次 go_chain 并**把失败如实报出去**（不再静默成功），
        #   既保证视野干净，也让"回不去"这件事在 records 里可见。
        self._dock = dock
        # 料盒上方的安全横移高度候选（**从高到低**依次尝试，第一个可达的胜出）。
        #
        # ① 下界（为什么不能低）：物块底面必须高于沿途其它物块的顶面。最高的是 cup
        #    （桌面 0.80 + 半高 0.0175 ⇒ 顶面 0.8345，比料盒壁顶 0.830 还高）。
        #    实测 trace_blocks.log：carry_z=0.088 时，c2(携带的 mouse) 越过 c3(cup) 的
        #    间隙从名义 +0.0375 掉到 −0.0015 ⇒ c3 被推走。
        # ② 上界（为什么不能高）：go_cartesian 走 plan_segment，要求整段姿态恒为
        #    planner.cartesian_rpy=[0,0,0]；z 越高越是解到 LIMITS[4]=2.0071 之外。
        #    ★ 上界要按【三条腿】取最小值，而且由**回程腿**决定 ——
        #    retract_probe.py 离线细扫（step=0.006）实测：
        #        格          目标料盒    搬运腿  竖起腿  回程腿  综合
        #        c1c3c4c6    bin_cup     0.096  0.096  0.096  0.096
        #        c2          bin_mouse   0.112  0.096  0.096  0.096
        #        c5          bin_mouse   0.112  0.112  0.096  0.096
        #    即 mouse 那两格**抬得上去、回不来**。第 6 轮云机实跑 run_20260924_004129
        #    正是栽在这里：c2 放置成功后被 [RISE_AWAY] 挡住（q5=2.176），
        #    重试时更从 q5 已越界的 q0 起步，[CARRY_TO_B] 直接 q5=2.369。
        # ③ **边界值本身是脆的**：跟踪过冲实测约 1 mm（瞄准 0.112、到位 0.113），
        #    足以把 q0 顶出可行域，于是"下一步规划从非法起点出发而必然失败"。
        #    所以在边界上取点，成功是偶然、失败是必然 —— 目标值必须离边界留余量。
        #    ⇒ 候选值由「倍率」与「实测边界−余量」取小（0.096−0.008 ⇒ 0.088，
        #      正好是第 4 轮云机实跑验证过的取值）。
        # 退让式的乘数表 + 实测边界 + 被否决过的改法都在 exp3_sim/scene.py
        # （无 ROS import，可离线单测，且有回归护栏）。
        self._carry_z_default = carry_z_candidates(self._delegate._zg,
                                                   self._delegate._lift_dz)
        # ---- 料盒侧工具偏航（yaw）------------------------------------------------
        # c4_cycle.go_cartesian 每次都**现读** driver.cfg['planner']['cartesian_rpy']，
        # 所以按料盒改写它即可注入 yaw，不用动 c4_2。为什么需要 yaw：料盒 +x 半边
        # 的 IK 在 θ=0 不可达（q5 超 LIMITS[4] 约 3%~6%）；工具绕竖直轴自转不改变
        # 顶抓的夹持，却能把腕关节解放出来（bin_reach_levers.py --lever B 实测
        # θ=−45° 时 bin_cup 两槽全通，末点 q5≈1.72）。
        # 取物侧恒为 θ=0；姿态转移腿（θ0→θ）= 杠杆 B 测过的第 1 腿；回程后下一轮
        # descend 开头的 go_chain([approach]) 是关节空间移动，天然吸收任意 yaw。
        # 整链 6/6 已由 round8_probe.py 离线验证（含回停靠位插值净空）。
        planner_cfg = driver.cfg.get('planner', {}) if isinstance(
            getattr(driver, 'cfg', None), dict) else {}
        self._rpy_base = list(planner_cfg.get('cartesian_rpy', [0.0, 0.0, 0.0]))
        self._lock = threading.RLock()
        self._states = {}
        self._states_seq = 0       # model_states 帧序号：用于"等一帧新数据"的判据
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

    def _set_tool_yaw(self, rad):
        """在默认 rpy 基础上叠加工具偏航 rad（0 = 恢复默认）。

        只改 planner.cartesian_rpy 这一个键；go_chain（关节空间）与夹爪不受影响。
        ★ 必须在 descend 开头恢复 0：若上一轮在料盒侧失败退出，yaw 会残留，
        下一次取物的笛卡尔腿就会带着料盒偏航去够格子上方（未验证、也不该发生）。
        """
        b = self._rpy_base
        self.r.cfg.setdefault('planner', {})['cartesian_rpy'] = [
            float(b[0]), float(b[1]), float(b[2]) + float(rad)]

    def _on_model_states(self, msg):
        with self._lock:
            self._states = {
                name: (pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w)
                for name, pose in zip(msg.name, msg.pose)
            }
            self._states_seq += 1

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

    def _seq(self):
        with self._lock:
            return self._states_seq

    def _wait_fresh_state(self, name, after_seq, timeout=2.0):
        """等到一帧【比 after_seq 更新】的 model_states 再取值；超时则退回当前缓存。

        为什么需要它：原 _wait_state 只判"缓存里有值"，不判"值够不够新"，于是
        release() 之后可能立刻读到**释放前**的旧位姿。实测症状正是
        script_place_error=(0.000,0.000,0.192) —— x/y 精确落在料盒中心、z 却高
        0.192，即"物块还挂在跟随线程上"那一刻的快照，被当成放置结果判了失败。
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._states_seq > after_seq:
                    return self._states.get(name)
            time.sleep(0.01)
        return self._state(name)

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
        # 从扫描停靠位回网格：**先走关节空间回中转位**，再交给 delegate 做笛卡尔下探。
        # 不能省这一步：dock 位 j1=-2.6（整条臂在 base 反面），而格子在前方 x≈0.09~0.14。
        # 直接 go_cartesian(A_above) 会试图沿直线切过去，路径穿过近奇异区、IK 无解，
        # 实测 records 里全是 unreachable_target（'到格 c1 上方失败'）。
        self._set_tool_yaw(0.0)     # 取物侧恒为 θ=0（防上一轮料盒侧失败后残留）
        if self._approach is not None:
            ok, status, _ = self.r.go_chain([self._approach], 'LEAVE_DOCK')
            if not ok:
                return False, 'leave_dock_failed:%s' % status
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
            seq = self._seq()
            ok, _ = self.r.go_cartesian(lift_target, 'LIFT_A')
            if not ok:
                return False, 'lift_planning_or_execution_failed'
            held, note = self._judge_hold(ctx, seq)
            if held:
                return True, note
            if attempt < attempts - 1 and self._delegate._close is not None:
                self.r.grip_set(self._delegate._close, 're-PRESS', settle=0.5)
        return False, 'not_held:' + note

    def _judge_hold(self, ctx, after_seq=0, timeout=1.2):
        """判据口径不变（delta >= 0.035），但改成在 timeout 内轮询【最新帧】。

        原实现只取一次快照。跟随线程 20 Hz，go_cartesian 刚返回时它常落后 1~2 个
        周期，实测出现过 script_follow_dz=0.034（差 1 mm 就过线）；而 _attach 之后
        物块是被"相对刀尖固定偏移"刚性 set 的，只要给跟随线程一点时间，delta 必然
        收敛到 lift_dz≈0.1。所以这里是"等系统到稳态"，不是放宽判据。
        """
        name = 'block_' + ctx['cell_id']
        if self._initial_z is None:
            return False, 'model_state_missing'
        deadline = time.monotonic() + timeout
        delta = None
        while time.monotonic() < deadline:
            state = self._wait_fresh_state(name, after_seq, timeout=0.5)
            if state is None:
                return False, 'model_state_missing'
            delta = state[2] - self._initial_z
            if delta >= 0.035:
                return True, 'script_follow_dz=%.3f' % delta
            after_seq = self._seq()
            time.sleep(0.05)
        return False, 'script_follow_dz=%.3f' % (delta if delta is not None else 0.0)

    def _go_above_bin(self, B_place, label, bin_id=None):
        """斜移到料盒正上方（带下降），按候选高度从高到低退让。返回 (ok, last_target)。

        候选高度按**该料盒的实测 IK 边界**封顶（见 scene.carry_z_candidates）：
        起点越接近边界，跟踪过冲把 q0 顶出可行域、下一步规划必然失败的风险越大。
        """
        last = None
        for z in carry_z_candidates(self._delegate._zg, self._delegate._lift_dz,
                                    bin_id) or self._carry_z_default:
            target = (B_place[0], B_place[1], z)
            ok, _ = self.r.go_cartesian(target, label)
            if ok:
                return True, target
            last = target
        return False, last

    def carry(self, ctx):
        """★ 两段式搬运：先斜移到料盒上方（物块底面高于料盒壁顶），再垂直下降投放。

        不能直接 go_cartesian(B_place)：那条斜线会让物块在 z≈0.8175（正好落在
        料盒壁高 0.802~0.830 的区间内）**从侧面穿过料盒壁**，瞬间互穿把 ODE 的 LCP
        求解器打爆（实测 `ODE Message 3: LCP internal error, s <= 0` 冲到 s=-4.8e+07），
        物块被炸飞 —— 表现为 script_place_error=(0.426,0.476,-0.694) 这类
        xy 乱飞 + z 掉到桌面下 0.78 m，且搬运距离越远越容易发生
        （c3 距离 0.24 m 最远，连败 3 次）。
        """
        _, _, _, B_place = self._delegate._targets(ctx)
        # 料盒侧整段（斜移/下探/抬起/回程）用**该料盒的偏航**：bin_cup θ=−45° 才够得着
        # +x 半边的槽位（bin_reach_levers --lever B 实测）；mouse 盒全在 −x，θ=0 即可。
        # yaw 保持到 retract 结束（回程腿也在同一偏航下离袋）；随后下一轮 descend 恢复 0。
        self._set_tool_yaw(float(ctx.get('bin_yaw_rad', 0.0) or 0.0))
        ok, last = self._go_above_bin(B_place, 'CARRY_ABOVE_BIN', ctx.get('bin_id'))
        if not ok:
            return False, '抬到料盒上方失败 (last=%s)' % (last,)
        return self._delegate.carry(ctx)

    def release(self, ctx):
        opened = self._delegate.r.grip_set(self._delegate._open, 'RELEASE',
                                            settle=self._delegate._release_settle)
        if not opened[0]:
            return False, 'gripper_release_failed'
        self._stop_follow()
        time.sleep(0.8)
        # seq 必须在 _stop_follow() 之后取：它标定"物块已脱离跟随"的时间点，
        # 之后到的第一帧才是"自由落体后的位置"。
        seq = self._seq()
        return self._judge_place(ctx, seq)

    def _judge_place(self, ctx, after_seq=0):
        state = self._wait_fresh_state('block_' + ctx['cell_id'], after_seq, timeout=3.0)
        if state is None:
            return False, 'model_state_missing_after_release'
        dx = abs(state[0] - ctx['B']['x'])
        dy = abs(state[1] - ctx['B']['y'])
        ok = dx <= 0.043 and dy <= 0.031 and state[2] <= self.base_z + 0.10
        return ok, 'script_place_error=(%.3f,%.3f,%.3f)' % (dx, dy, state[2] - self.base_z)

    def retract(self, ctx):
        # 同理：自料盒底直接斜拉到格子上方会再次擦过料盒壁，先抬到料盒正上方再走。
        _, _, _, B_place = self._delegate._targets(ctx)
        ok, last = self._go_above_bin(B_place, 'RISE_ABOVE_BIN', ctx.get('bin_id'))
        if not ok:
            return False, '自料盒抬起失败 (last=%s)' % (last,)
        ok, note = self._delegate.retract(ctx)
        # 无论 retract 内部有没有真的回零，这里都**显式**再回一次停靠位。
        # 目的不是冗余，而是让"扫描时臂必须在停靠位"成为一条硬约束：
        # 只要回不去就报失败（会在 records 里留痕），而不是留一条臂在网格上方
        # 让下一轮扫描静默失效。
        if self._dock is not None:
            dok, dstatus, _ = self.r.go_chain([self._dock], 'RETURN_DOCK')
            if not dok:
                return False, 'return_dock_failed:%s' % dstatus
        return ok, note

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
    # empty_cell: 异常轮用。留空=正常 6 物记分轮；填 c1..c6 之一=该格故意不放物体。
    init_node.declare_parameter('empty_cell', '')
    base_z = float(init_node.get_parameter('base_z').value)
    empty_cell = str(init_node.get_parameter('empty_cell').value or '').strip()
    cfg = _load_json(os.path.join(c4_root, 'config_sim.json'))
    cfg['backend'] = {'type': 'manual'}
    # ★ 减速是**唯一**能对抗"搬运途中物块下坠"的杠杆 —— 抬高那条路被 IK 堵死了。
    # 实测：carry_z=0.088 时携带的 mouse 越过 c3(cup) 时，间隙从名义 +0.0375 掉到
    # −0.0015，即物块实际比名义位姿**低了约 39 mm**（无论这 39 mm 来自
    # /gazebo/set_entity_state 的服务延迟，还是 JTC 的轨迹跟踪欠调，两者都随速度线性缩放，
    # 所以减速对两种成因同时有效）。
    # 而 carry_z 最多只能抬到 0.096（cup 格的 IK 上限，见 _carry_z_candidates 的实测），
    # 即最多多买 8 mm —— 远不够。故：抬到上限 + 速度减半，两者相乘才有富余。
    # 预期：名义间隙 0.0375+0.008=0.0455，下坠折半 ≈0.0195 ⇒ 余量 ≈26 mm。
    # cart_pt_t: 每 0.006 m 路径点耗时。0.15→0.30 把搬运动速从 ~0.04 m/s 降到 ~0.02 m/s。
    cfg['timing'].update({'cart_pt_t': 0.30, 'descend_pt_t': 0.6,
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
        safe_pose = list(cfg['descend_a_joints'][-1])
        # 停靠位：safe_pose 只把 j1 旋到 base 反面，其余关节不动。
        dock_pose = list(safe_pose)
        dock_pose[0] = DOCK_J1
        ok, status, _ = driver.go_chain([dock_pose], 'EXP3_DOCK_BEFORE_SCAN')
        if not ok:
            raise RuntimeError('dock before scan failed: %s' % status)
        motion = {
            # ★ 必须是"关节列组成的链"（[[q...]]）而不是裸的 6 元关节列：
            # c4_executor.retract 直接把它交给 robot.go_chain(home)，而 go_chain 的
            # chain 语义是 list-of-poses（main 里 go_chain([safe_pose]) 同理）。
            # 传裸关节列会让 go_chain 把每个 float 当成一个 pose 去迭代 ⇒
            # TypeError("'float' object is not iterable")，实测报 exec_error。
            'home': [dock_pose],    # retract 后回停靠位：保证下一轮扫描时整条臂不在网格上方
            'gripper_open': cfg['gripper']['open'],
            'gripper_close': cfg['gripper']['close'],
            'grasp_z': cfg['task']['grasp_mid_z'],
            'lift_dz': cfg['task']['lift_dz'],
            'place_z': cfg['task']['grasp_mid_z'],
            'lift_retry': 1,        # 抬离判定失败时允许"就地重压再抬"一次（原为 0=不重试）
            'press_settle': 0.5,
            'release_settle': 0.5,
            'hold_assume': False,
            'place_assume': False,
        }
        # 中转位必须用 safe_pose（descend_a_joints[-1]）而不是 [0]：
        # go_cartesian 是"锁当前姿态"的直线规划，[0] 的姿态（q5=1.6722）下解
        # (0.09,-0.08,0.138) 会得到 q5=2.122 > LIMITS[4]=2.0071，实测报
        # "[TO_ABOVE_A] 规划失败: LIMIT q5=2.122"。safe_pose 是原代码里已验证能
        # 到达该点的构型，且与 dock_pose 只差 j1，LEAVE_DOCK 就是一次干净的 j1 回转。
        follow = GazeboFollowExecutor(driver, motion, base_z=base_z,
                                      approach=list(cfg['descend_a_joints'][-1]),
                                      dock=dock_pose)
        _spawn_scene(driver, base_z, empty_cell)
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
