# task_control —— 仿真线的任务控制 ROS2 缝隙

> 这是**仿真线**（mechArm 270 / Gazebo）。真机线是 EP，**不走 ROS2**，见 [`../../real/README.md`](../../real/README.md)。
> 两条线共用 `sort_core/`，只换下面这两个缝隙。

把 `exp3/sort_core` 的 TaskController 接进 ROS2。TaskController 只有两个注入缝隙，
离线等价实现已在 `examples/`（mock 桌面/后端，可先跑 `python3 examples/run_demo.py --mode happy` 理解缝隙）：

| TaskController 缝隙 | 离线(examples) | 仿真线 ROS2(本目录) | 真机线(`real/`) |
|---|---|---|---|
| `scan()` → 每轮识别 det 列表 | `OfflineTable.scan` | `ScanFromDetections.latest`（订阅 Detection2DArray） | `real/camera.py` 的 `make_scan` |
| `pick_place(cell_id, cls)` | `OfflineTable.pick_place` | `PickPlaceActionClient.pick_place`（发 Action） | `real/ep_backend.py` 的 `EPPickPlace` |

- Action server 骨架：`pick_place_server.py`（时序 + A/B 解算 + reason 收尾，机器人无关）
- 检测订阅：`scan_from_detections.py`

> 本目录文件需要 ROS2 + `sort_msgs` 编译产物 + vision_msgs，**只能在装了 ROS2 的机器上跑**
> （Mac 离线只做语法检查）。跑法继承 c4_2 风格：直接 `python3 <file>`，不是 colcon 包。
>
> **例外**：`exec_contract.py` 是**故意 ROS-free** 的 —— 真机执行者 `EPPickExecutor` 也从这里
> 取 `PickExecutor` ABC 与 `resolve_goal`，保证两条线的执行契约是同一份、不会分叉。

## 三件套怎么拼（仿真机，main 线程编排同 c4_2/drive/c4_cycle.py）

```bash
# 0) 环境：source ROS2 + 工作区(已 colcon build sort_msgs)，在 mecharm-grasp-exp/exp3 下
source ~/ws_mecharm/install/setup.bash
```

1. **起 PickPlace server**（终端 A，先冒烟时序）：
   ```bash
   python3 ros2/task_control/pick_place_server.py --executor stub
   ```
   server CLI 目前只带 `stub`（假成功，验时序/reason 用）。

### 仿真执行者已经写好：`c4_executor.C4PickExecutor`

它把上面 6 段动作**实现完了**（组合 C4Driver 当 robot、只调运动原语；A/B 由 server 按 goal 解算），
把真 C4Driver 实例传进去即可，不用手写 6 段：

```python
# C4Driver 实例就绪后（构造方式见 c4_2/drive/c4_cycle.py main()）
from ros2.task_control.c4_executor import C4PickExecutor

motion = dict(
    home=cfg['home'],                          # c4_2 config_sim.json 现成
    gripper_open=cfg['gripper']['open'],
    gripper_close=cfg['gripper']['close'],
    grasp_z=cfg['task'].get('grasp_mid_z', 0.038),   # 爪中高度（按甲标定覆盖）
    lift_dz=cfg['task'].get('lift_dz', 0.1),
    place_z=cfg['task'].get('grasp_mid_z', 0.038),
    hold_assume=True, place_assume=True,       # 彩排口径；真判据 override 下面两个方法
)
executor = C4PickExecutor(driver, motion)      # driver = 你的 C4Driver 实例
node = PickPlaceServer(load_grid(), load_bins(), executor)
```

**与 c4_2 的三点不同**（见 c4_executor 模块注释）：不 reset_block（物体由场景摆好，选哪格夹哪格）、
多 A/B 由 server 解算、判 held/placed 默认 assume。真判据在联调时对 `C4PickExecutor`
override `_judge_hold(ctx)` / `_judge_place(ctx)`（读物体位姿、或人工确认，对齐 c4_2 judge=manual）。
`motion` 数值联调时按甲标定覆盖 —— 这部分只能在 Gazebo 上验证，Mac 只做了时序/失败传播的
离线单测（tests/test_c4_executor.py，假 robot）。

> **真机线的对应物**是 [`real/ep_backend.py`](../../real/ep_backend.py) 的 `EPPickExecutor`：
> 同一个 `PickExecutor` ABC、同样 6 段、同样的 reason 映射，但走 EP Python SDK 而不是 C4Driver。
> 那边没有 ROS2，所以 `c4_executor.py` 与它**互不相干**，改一个不用动另一个。

2. **起识别/仿真侧**：乙的 `detection_d2a_node` 或 甲的 mock 识别节点发布 `Detection2DArray`
   （默认话题 `detections_d2a`，可在代码/launch 改）。

3. **跑任务控制**（终端 B）：
   ```python
   # 拼装示意（落到 launch/脚本里）
   from sort_core.task import TaskController
   from sort_core.config import load_all
   from sort_core.logging_util import Exp3RunLog, make_run_dir
   from ros2.task_control.scan_from_detections import ScanFromDetections, wait_first_frame
   from ros2.task_control.pick_place_client import PickPlaceActionClient
   ...
   scan = ScanFromDetections(image_topic='/detections_d2a',
                             image_width_px=cfg['task']['camera']['image_width_px'], ...)
   wait_first_frame(scan_node, scan.latest)          # 收到首帧再开跑
   ctl = TaskController(task, bins, cells, scan.latest, client.pick_place, log=runlog)
   summary = ctl.run()                               # 与离线 demo 同一条路径
   verdict = runlog.write_result(...)
   ```

## 阶段号 / reason

- stage 1..6 见 `sort_core/taxonomy.py`（STAGE_APPROACH..RETRACT），服务端按段发 feedback。
  真机线用**同一套**阶段号（`real/ep_backend.py` 的 `STAGE_ORDER`），只是不经过 Action 消息。
- result.reason 只出动作级枚举（taxonomy）：`ok/grasp_failed/dropped_or_missed_place/
  unreachable_target/exec_error/aborted`；客户端把非动作级值兜底成 `exec_error`。
- 「连续失败→safety_stop / 空转→no_exec / 桌面空→no_target」由 TaskController 判定（离线已测）。

## 与乙/甲的交接点

- bbox 是归一化 `[0,1]` 原点左上（乙骨架同款），本目录按 config 图像尺寸还原像素；若口径改像素直出，
  改 `scan_from_detections.dets_from_msg` 里 `_norm_to_px`（或加参数关掉）。真机线走同一个归一化口径。
- class/conf 读取兼容 vision_msgs 新旧字段；乙若用新版 `class_name` 直发则走第一分支，无需对照表。
