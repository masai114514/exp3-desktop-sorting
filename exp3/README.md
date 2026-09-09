# exp3 — 实验3《桌面物体自动分类整理》接口契约 + 起步包

> 位置：`mecharm-grasp-exp/exp3/`（本目录 = **三人共同的接口规格** + 任务控制核心的可离线测试雏形）
> 定版：2026-09-08 ｜ 分工总览见仓库外 `机器人实验/实验3_分工.md`

本目录分成两层：

- **`sort_core/` + `tests/`**：纯 Python、无 ROS 依赖的任务控制核心，Mac/Windows/Jetson 任意一处
  `python3` 都能离线跑单测 —— 是“先让逻辑绿、再接线”的锚点。
- **`contract/` + `ros2/`**：跨 ROS2 的接口规格（PickPlace action 定义、Detection2DArray 参考骨架、
  sort_msgs 接口包）—— 需要 ROS2 环境才能 build/验证，**离线跑不了**，但内容即“共同口径”，照此接线。

---

## 0. 运行（离线部分）

```bash
cd mecharm-grasp-exp/exp3
python3 -m unittest discover -s tests -t . -v   # 无需 ROS，sort_core 逻辑绿即可
python3 examples/run_demo.py --mode happy       # 真实 config 端到端：mock 桌面跑完整控制循环 → exp3_logs
python3 config_check.py --strict                # 回填自检：甲/乙改 config 后先跑，ERROR/WARN 拦错填
```

## 1. 系统接口契约（三人共同遵守，改动需三方同步）

| 接口 | 定义 | 谁实现/消费 |
|---|---|---|
| 图像 | `sensor_msgs/Image`（俯视相机） | 甲出相机 → 乙订阅 |
| 识别输出 | `vision_msgs/Detection2DArray`，每条含 class_name + bbox + confidence | 乙改 `detection_node` 发布；骨架见 `ros2/reference/detection_d2a_node.py` |
| 抓取 | ROS2 Action **`PickPlace`**：goal=`cell_id`+期望 `cls`；feedback 每阶段发 `int32 stage`(+note)；result=`success`+`reason`+`detail` | 甲/你封 Action server，甲在仿真侧接线；定义见 `contract/PickPlace.action` |
| 任务控制 | 状态机循环：扫描→选目标→抓取→分类放置→直到无目标 | 你（`sort_core/task.py`，离线可测） |
| 日志 | 每物体 1 条记录 + run 级 `result.json` | 你（`sort_core/logging_util.py`），落地 `exp3_logs/run_<ts>/` |

reason 枚举（`sort_core/taxonomy.py`，与 c4_2 对齐）：
- **动作级**（一次 PickPlace 的 result.reason）：`grasp_failed` / `dropped_or_missed_place` /
  `unreachable_target` / `exec_error` / `aborted`；成功=`ok`。
- **任务级**（控制器跳过/退出用）：`no_target`(桌面清空，正常结束) / `no_exec`(只剩不可执行) /
  `out_of_grid`(框中心不在网格) / `unrecognized_object`(低置信/未知类) / `safety_stop`(连续失败或保护触发)。

## 2. 目录与谁填什么

```
exp3/
├── README.md                 ★ 本文件（契约；改动需同步三方）
├── contract/
│   └── PickPlace.action      ★ PickPlace 权威语义注释（阶段号、reason 取值口径）
├── examples/                 任务控制线离线 demo（真实 config，不连 ROS）
│   ├── mock_backend.py       mock 桌面/后端：scan/pick_place 缝隙的离线等价实现
│   └── run_demo.py           CLI：happy/recovery/safety_stop/unrecognized/out_of_grid → exp3_logs
├── ros2/                      ← 需 ROS2 环境才能 build；内容是可照抄的规格
│   ├── sort_msgs/            接口包：action/PickPlace.action + CMake/package.xml（只装消息）
│   ├── reference/
│   │   └── detection_d2a_node.py  乙改造检测节点的参考骨架（Image→Detection2DArray）
│   └── task_control/         任务控制线 ROS2 缝隙（Jetson 跑，见其 README）
│       ├── exec_contract.py          执行契约：PickExecutor ABC + goal→A/B 解算（ROS-free，离线可测）
│       ├── c4_executor.py            C4PickExecutor：用 c4_2 直驱原语实现 6 段动作（真执行者）
│       ├── pick_place_server.py      Action server 时序骨架（契约 + reason 收尾）
│       ├── pick_place_client.py      seam：pick_place(cell_id,cls) 发 PickPlace Action
│       └── scan_from_detections.py   seam：订阅 Detection2DArray → TaskController.scan()
├── config/
│   ├── grid_cells.json       ★ 甲回填：网格在机器人系的矩形+取放中心（M1 前）
│   ├── bins.json             ★ 甲回填：cup/mouse 对应料盒坐标；乙核对 class 名(cup/mouse)
│   ├── task.json             任务参数：conf_min、连续失败上限、expected_total、pass_line；camera.calib
│   └── log_schema.md         日志字段说明（对齐 c4_2 result.json 口径）
├── config_check.py           ★ 回填自检器：grid/bins/camera/cross 契约一致性，改 config 后先跑
├── sort_core/                ★ 任务控制纯 Python 核心（离线可测）
│   ├── taxonomy.py           reason / verdict 常量
│   ├── geometry.py           bbox 中心 → 桌面点 → 网格 cell（rectilinear / homography）
│   ├── decision.py           class→料盒、可执行性判定
│   ├── task.py               TaskController 状态机（注入 scan/exec/safe_stop）
│   ├── logging_util.py       记录构造 + result.json 汇总
│   └── config.py             读 config/*.json 成 dict
└── tests/                    unittest：test_geometry / test_decision / test_task（离线绿）
```

## 3. ROS2 侧怎么接（详见各子目录 README）

- **sort_msgs**（`ros2/sort_msgs/README.md`）：拷贝进 workspace → `colcon build --packages-select sort_msgs`
  → `ros2 interface show sort_msgs/action/PickPlace` 验证。
- **检测节点**（`ros2/reference/detection_d2a_node.py`）：乙按其改旧节点。注意 vision_msgs 版本差异
  （老版 `ObjectHypothesis{class_id, score}` vs 新版 `class_probabilities`），代码已用 `hasattr` 两头兼容；
  改前先 `ros2 interface show vision_msgs/msg/ObjectHypothesis` 看本机字段。
- **任务控制缝隙**（`ros2/task_control/README.md`）：你的线 —— 先跑 `examples/run_demo.py` 弄清
  scan/pick_place 两个缝隙，再在 Jetson 把 TaskController 接到 ScanFromDetections + PickPlace client，
  server 换真执行者（c4_2 C4Driver 派生实现 6 个动作段）。

## 4. 三人下一步

- **你（任务控制）**：控制器核心 + 日志已在 `sort_core/` 且离线绿；离线 e2e 见 `examples/run_demo.py`。
  Jetson 上把两个缝隙接 ROS2：`scan`→订阅 `Detection2DArray`、`exec_pick`→`PickPlace` action client
  （骨架 `ros2/task_control/`）；Action server 用 c4_2 直驱原语补真执行者（多格多料盒，非 c4_2 单 A/B）。
- **乙（识别）**：按 `ros2/reference/` 骨架改 `detection_node` 发布 `Detection2DArray`；用真模型出识别短片。
  仿真期如接 mock：mock 节点发布与真节点同 topic 同 msg，控制器不感知差异。
- **甲（仿真）**：Jetson 起场景后把网格/料盒几何回填 `config/*.json`；相机非正俯视就把 `task.json`
  的 `camera.calib` 换成 homography，`geometry.px_to_table` 两模式都支持。
- **全组**：跑 `python3 -m unittest discover -s tests -t . -v` 确认绿，再各接各线。
