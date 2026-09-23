# exp3 — 实验3《桌面物体自动分类整理》接口契约 + 工程

> 位置：`mecharm-grasp-exp/exp3/`（本目录 = **三人共同的接口规格** + 任务控制核心的可离线测试雏形）
> 定版：2026-09-08 ｜ 2026-09-15 补真机线（EP）｜ 分工总览见仓库外 `机器人实验/实验3_分工.md`

本工程是**一套任务控制核心、两条执行线**：

|  | 仿真线 | 真机线 |
|---|---|---|
| 平台 | mechArm 270（Gazebo） | **RoboMaster EP 工程形态** |
| 中间件 | ROS 2 Humble | **无 ROS**（EP Python SDK 直连） |
| 执行者 | `ros2/task_control/c4_executor.py` | `real/ep_backend.py`（`EPPickExecutor`） |
| 配置 | `config/` | `config_real/` |
| 入口 | `examples/run_demo.py`；ROS2 三件套见 `ros2/task_control/README.md` | `real/run_real.py`（见 `real/README.md`） |
| 日志 | `exp3_logs/` | `exp3_logs_real/`（**故意分开**，别混） |

两条线**只换两个缝隙**，任务逻辑、`PickPlace` 契约、reason 枚举、日志 schema 完全共用：

```
        仿真线                                        真机线
  scan       ── ScanFromDetections（订阅 D2A）   real/camera.py        ── scan()
  pick_place ── PickPlaceActionClient（发 Action） real/ep_backend.py    ── pick_place()
                    └───────────────┬────────────────────────┘
                      sort_core/（ROS-free，两边共用，一行不改）
```

这不是为了好看：`TaskController` 只有这两个注入缝隙，所以**换平台不动核心** ——
真机从 mechArm+Jetson 换成 EP 之后，契约一行没改，就是这个原因。

本目录分三层：

- **`sort_core/` + `tests/`**：纯 Python、无 ROS 依赖的任务控制核心，Mac/Windows/任意机器
  `python3` 都能离线跑单测 —— 是“先让逻辑绿、再接线”的锚点。
- **`contract/` + `ros2/`**：仿真线的 ROS2 接口规格与缝隙实现（PickPlace action 定义、sort_msgs
  接口包）—— 需要 ROS2 环境才能 build/验证，**离线跑不了**，但内容即“共同口径”，照此接线。
- **`real/` + `config_real/`**：真机线（EP）。不依赖 ROS2；没 EP 也能 `--dry-run` 把动作链跑通，
  但**没标定就拒绝启动**（见 `real/README.md`）。

---

## 0. 运行（离线部分，任意机器）

```bash
cd mecharm-grasp-exp/exp3
python3 -m unittest discover -s tests -t . -v   # 无需 ROS，205 个用例
python3 examples/run_demo.py --mode happy       # 仿真线端到端：mock 桌面跑完整控制循环 → exp3_logs
python3 config_check.py --strict                # 回填自检：甲/乙改 config 后先跑，ERROR/WARN 拦错填

# 真机线（没标定会被闸门拦下，这是设计如此）
python3 config_check.py --config-dir config_real                        # 看还差哪些标定
python3 real/run_real.py --config-dir config_real --dry-run \
        --camera none --detector mock --assume-judge --no-pause          # 演练：不连 EP
```

## 1. 系统接口契约（三人共同遵守，改动需三方同步）

| 接口 | 定义 | 谁实现/消费 |
|---|---|---|
| 图像 | `sensor_msgs/Image`（仿真）／USB 相机取帧（真机） | 甲出相机 → 乙订阅；真机由 `real/camera.py` 取帧 |
| 识别输出 | `vision_msgs/Detection2DArray`，每条含 class_name + bbox + confidence | 乙改 `detection_node` 发布；骨架见 `ros2/reference/detection_d2a_node.py`。真机线同样口径，交付成 `detect(frame)` 一个函数（见 `real/README.md`） |
| 抓取 | ROS2 Action **`PickPlace`**：goal=`cell_id`+期望 `cls`；feedback 每阶段发 `int32 stage`(+note)；result=`success`+`reason`+`detail` | 甲/你封 Action server，甲在仿真侧接线。定义见 `contract/PickPlace.action`。**真机线不经过 ROS**，但 goal/阶段/reason 语义逐字相同 |
| 任务控制 | 状态机循环：扫描→选目标→抓取→分类放置→直到无目标 | 你（`sort_core/task.py`，离线可测） |
| 日志 | 每物体 1 条记录 + run 级 `result.json` | 你（`sort_core/logging_util.py`），落地 `exp3_logs/run_<ts>/` |

reason 枚举（`sort_core/taxonomy.py`，与 c4_2 对齐）：
- **动作级**（一次 PickPlace 的 result.reason）：`grasp_failed` / `dropped_or_missed_place` /
  `unreachable_target` / `exec_error` / `aborted`；成功=`ok`。
- **任务级**（控制器跳过/退出用）：`no_target`(桌面清空，正常结束) / `no_exec`(只剩不可执行) /
  `out_of_grid`(框中心不在网格) / `unrecognized_object`(低置信/未知类) / `safety_stop`(连续失败或保护触发)。

> **真机侧的位姿从哪来**：goal 只带 `cell_id`+`cls`，**不带坐标**。仿真线按该 id 查
> `config/grid_cells.json` 的格中心（机器人系）；真机线按该 id 查 `config_real/ep_waypoints.json`
> 的底盘里程计位姿。两个坐标系之间**不需要建立换算关系** —— 这也是契约能一行不改的原因。

## 2. 目录与谁填什么

```
exp3/
├── README.md                 ★ 本文件（契约；改动需同步三方）
├── contract/
│   └── PickPlace.action      ★ PickPlace 权威语义注释（阶段号、reason 取值口径）
├── examples/                 仿真线离线 demo（真实 config，不连 ROS）
│   ├── mock_backend.py       mock 桌面/后端：scan/pick_place 缝隙的离线等价实现
│   └── run_demo.py           CLI：happy/recovery/safety_stop/unrecognized/out_of_grid → exp3_logs
├── ros2/                      ← 仿真线，需 ROS2 环境才能 build；内容是可照抄的规格
│   ├── sort_msgs/            接口包：action/PickPlace.action + CMake/package.xml（只装消息）
│   ├── reference/
│   │   └── detection_d2a_node.py  乙改造检测节点的参考骨架（Image→Detection2DArray）
│   └── task_control/         仿真线 ROS2 缝隙（见其 README）
│       ├── exec_contract.py          执行契约：PickExecutor ABC + goal→A/B 解算（ROS-free，离线可测）
│       ├── c4_executor.py            仿真执行者：用 c4_2 直驱原语实现 6 段动作
│       ├── pick_place_server.py      Action server 时序骨架（契约 + reason 收尾）
│       ├── pick_place_client.py      seam：pick_place(cell_id,cls) 发 PickPlace Action
│       └── scan_from_detections.py   seam：订阅 Detection2DArray → TaskController.scan()
├── real/                      ← 真机线（EP），不依赖 ROS；见 real/README.md
│   ├── run_real.py           入口：闸门 → 相机 → 检测 → TaskController → EP
│   ├── ep_backend.py         真机执行者 EPPickExecutor（同 PickExecutor ABC）
│   ├── camera.py             相机 + 检测器（mock / YOLO / 外部 detect(frame)）
│   ├── dry_run.py            假 EP：不连硬件演练动作链
│   └── scenes/               mock 检测器的物体表
├── config/                    ★ 仿真线配置（甲填）
│   ├── grid_cells.json       网格在机器人系的矩形+取放中心
│   ├── bins.json             cup/mouse 对应料盒坐标；乙核对 class 名
│   ├── task.json             任务参数：conf_min、连续失败上限、expected_total、pass_line；camera.calib
│   └── log_schema.md         日志字段说明（对齐 c4_2 result.json 口径）
├── config_real/               ★ 真机线配置（组长/乙填，见 real/README.md）
│   ├── task.json / grid_cells.json / bins.json   同 schema，数值是现场实测
│   └── ep_waypoints.json     EP 执行侧：底盘位姿 + 臂两档 + 夹爪 + 判据
├── config_check.py           ★ 回填自检器：结构一致性 + **真机标定闸门**（`--require-calibrated`）
├── docs/                     提交材料：状态机规格（docx §七.3）、异常测试记录（§三/四）、依赖说明（§七.1）
│   └── state_machine.md      状态机规格：状态/转移/阈值/去重口径；参数文件即 config*/task.json
├── sort_core/                ★ 任务控制纯 Python 核心（离线可测）
│   ├── taxonomy.py           reason / verdict 常量
│   ├── geometry.py           bbox 中心 → 桌面点 → 网格 cell（rectilinear / homography）
│   ├── decision.py           class→料盒、可执行性判定
│   ├── task.py               TaskController 状态机（注入 scan/exec/safe_stop）
│   ├── logging_util.py       记录构造 + result.json 汇总
│   └── config.py             读 config/*.json 成 dict
└── tests/                    unittest：sort_core + 两条线的缝隙（离线绿）
```

## 3. 两条线分别怎么接

### 3.1 仿真线（ROS2，详见各子目录 README）

- **sort_msgs**（`ros2/sort_msgs/README.md`）：拷贝进 workspace → `colcon build --packages-select sort_msgs`
  → `ros2 interface show sort_msgs/action/PickPlace` 验证。
- **检测节点**（`ros2/reference/detection_d2a_node.py`）：乙按其改旧节点。注意 vision_msgs 版本差异
  （老版 `ObjectHypothesis{class_id, score}` vs 新版 `class_probabilities`），代码已用 `hasattr` 两头兼容；
  改前先 `ros2 interface show vision_msgs/msg/ObjectHypothesis` 看本机字段。
- **任务控制缝隙**（`ros2/task_control/README.md`）：你的线 —— 先跑 `examples/run_demo.py` 弄清
  scan/pick_place 两个缝隙，再把 TaskController 接到 ScanFromDetections + PickPlace client，
  server 用 `c4_executor.C4PickExecutor` 作执行者（c4_2 C4Driver 直驱，6 个动作段）。

### 3.2 真机线（EP，**不走 ROS**，详见 `real/README.md`）

`real/run_real.py` 一条命令起整链：**闸门 → 相机 → 乙的检测 → TaskController → EP 后端**。

```bash
cd exp3
python3 real/run_real.py --config-dir config_real \
        --camera cv2:0 --detector file --detector-file /path/to/detector.py
```

- 闸门在**连机器人之前**：四份标定记录（相机=乙 / 网格 / 料盒 / EP=组长）任一没填就退出。
- EP 的 `moveto(x,y)` 只有**一个水平自由度** ⇒ 桌面横向覆盖靠底盘平移/转向，不是靠臂。
- EP **没有物块感知** ⇒ “夹住没有/放正没有”由人工确认（`judge=operator_confirm`，对齐实验二）；
  类别判断与动作触发全程由系统做，人不参与（docx §六）。
- `--dry-run` 只证明链路通，**不能**当验收凭据。

## 4. 三人下一步

- **你（任务控制）**：控制器核心 + 日志已在 `sort_core/` 且离线绿；仿真线 e2e 见 `examples/run_demo.py`，
  真机线 e2e 见 `real/run_real.py`。两条线的缝隙都已实现，**你这边不缺代码，缺的是现场标定**。
- **乙（识别）**：按 `ros2/reference/` 骨架改 `detection_node` 发布 `Detection2DArray`（仿真线）；
  真机线交付一个 `detect(frame)` 函数（口径见 `real/README.md`），并回填
  `config_real/task.json` 的 `camera.calibration`（相机标定）。用真模型出识别短片。
- **甲（仿真）**：起场景后把网格/料盒几何回填 `config/*.json`（**仿真尺寸按真机实物**，
  D4 从软约束变硬门）；相机非正俯视就把 `task.json` 的 `camera.calib` 换成 homography，
  `geometry.px_to_table` 两模式都支持。
- **全组**：跑 `python3 -m unittest discover -s tests -t . -v` 确认绿，再各接各线。
