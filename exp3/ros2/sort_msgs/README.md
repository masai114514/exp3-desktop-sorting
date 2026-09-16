# sort_msgs —— 实验3 接口包（PickPlace action）

只定义消息，不含任何业务逻辑。供甲（**仿真**执行侧）和乙（视觉侧）共同 `find_package` / `import`，
保证两边拿到的 `PickPlace` goal/feedback/result 字段一致。

> **真机线（EP）不用这个包**：EP 走 Python SDK 直连，没有 ROS2，因此 `PickPlace` 的
> goal/阶段/reason 语义以**代码契约**的形式复用（`ros2/task_control/exec_contract.py` 的
> `PickExecutor` ABC，故意写成 ROS-free），而不是以 ROS 消息的形式。
> 语义仍然是同一份，见 [`contract/PickPlace.action`](../../contract/PickPlace.action)。

权威语义注释见 [`contract/PickPlace.action`](../../contract/PickPlace.action)，本目录 `.action` 为同内容的精简拷贝。

## 构建（甲，装了 ROS2 的仿真机）

```bash
cd ~/ws_mecharm/src        # 或用你建仿真包的那个 workspace
cp -r <本目录 sort_msgs> ./sort_msgs
cd ~/ws_mecharm && colcon build --packages-select sort_msgs --symlink-install
source install/setup.bash

# 验证消息生成成功
ros2 interface show sort_msgs/action/PickPlace
ros2 pkg prefix sort_msgs    # 应打印 workspace 安装路径
```

## 阶段编号约定

`.action` 不支持常量，阶段号在业务代码里定义。甲的实现（服务端）与乙的演示（客户端 feedback 打印）
统一用下表，**不要自创数字**：

| 常量名 | 值 | 含义 |
|---|---|---|
| `STAGE_APPROACH` | 1 | 运动到 cell 上方 |
| `STAGE_GRASP` | 2 | 下降 + 夹取 |
| `STAGE_LIFT` | 3 | 抬起 |
| `STAGE_MOVE_TO_BIN` | 4 | 移到料盒上方 |
| `STAGE_PLACE` | 5 | 下降 + 松爪 + 放入 |
| `STAGE_RETRACT` | 6 | 退回安全位 |
| `STAGE_FAILING` | -1 | 失败正在收尾（随后回 result） |

## 目标点/料盒怎么来（不写死在 action 里）

- 取/放点坐标：`config/grid_cells.json` 的 `cells[].id` 对应格子的中心 `x,y` + `pick_z_m`/`object_z_m`。
- 料盒：`config/bins.json` 的 `class_to_bin[cls]`（如 `cup → bin_cup`）。
- goal 只带 `cell_id + cls`，其余运行时查配置。

## result.reason 取值

与 [`sort_core/taxonomy.py`](../../sort_core/taxonomy.py) 的 `REASON_*` 对齐：
`ok / grasp_failed / dropped_or_missed_place / unreachable_target / exec_error / aborted`。
客户端看到 `success=false` 时按 reason 决定是否记录并继续下一个 cell。
