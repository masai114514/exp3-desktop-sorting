# exp3-desktop-sorting — 实验3《桌面物体自动分类整理》（公开参考）

三人机器人集成小组实验3 的**共享接口规格 + 任务控制核心**公开仓库。内容在 [`exp3/`](exp3/)，
clone 下来即可离线单测（不需 ROS、不需真机）；甲（仿真/Jetson）与乙（识别）照 [`exp3/README.md`](exp3/README.md)
的契约接线。

> 与三个实验仓库的区分，clone 前先读：
>
> | 实验 | 仓库 | 可见性 | 内容 |
> |---|---|---|---|
> | 实验1（检测模型训练） | `masai114514/object_detection_lab` | public | YOLOv8 cup/mouse 训练/推理（乙复用） |
> | 实验2（定点抓取单 A/B） | `masai114514/mecharm-grasp-exp` | private | 仿真 + 真机定点抓取基建（c4_2 直驱） |
> | **实验3（本仓库）** | **`exp3-desktop-sorting`** | **public** | **多格多料盒自动分类整理：接口契约 + 可离线测的任务控制核心** |
>
> 实验3 **不**依赖实验2 的代码逻辑，本仓库自包含（多格网格→选物→夹取→按类放料盒 全链路）。
> 唯一例外：Jetson 真执行件 `C4PickExecutor`（`exp3/ros2/task_control/c4_executor.py`）只组合
> 实验2 私有仓库里的 `C4Driver` 运动原语 —— 需要跑真机/仿真侧时找组长要 `mecharm-grasp-exp`
> 访问权；只想读契约与离线部分则本仓库足够。

## 目录

```
exp3/                     工程根（接口契约 + 纯 Python 任务控制核心 + ROS2 缝隙）
├── README.md             契约与目录导览（权威文档，改动三方同步）
├── contract/PickPlace.action   抓取 Action 权威语义（阶段号 / reason 口径）
├── sort_core/            任务控制纯 Python 核心（无 ROS import）
├── examples/             离线端到端 demo（mock 桌面，真实 config 跑控制循环）
├── config/               grid_cells / bins / task 参数（当前为示例占位，★场景落地后回填）
├── config_check.py       回填自检器：改 config 后先跑，ERROR/WARN 拦错填
├── ros2/                 需 ROS2 环境才能 build/跑（内容是照抄规格）
│   ├── sort_msgs/        PickPlace 接口包（colcon ament_cmake）
│   ├── reference/        乙改 detection_node 的参考骨架（→ Detection2DArray）
│   └── task_control/     任务控制 ROS2 缝隙：exec_contract / c4_executor / server / client / scan
└── tests/                unittest（离线绿：geometry/decision/task/run_demo/config_check/c4_executor）
```

## 离线先跑（无需 ROS / 真机）

```bash
cd exp3
python3 -m unittest discover -s tests -t . -v   # 任务控制核心逻辑全绿
python3 examples/run_demo.py --mode happy       # mock 桌面端到端：完整控制循环 → exp3_logs
python3 config_check.py --strict                # 甲/乙回填 config 后先自检
```

## 这套东西解决的问题

桌面摆 4–6 个取物格、cup/mouse 两类 ≥6 物，俯视相机识别后按类放进对应料盒：
识别（乙）— 任务状态机（本轮仓库核心）— 抓取执行（甲/组长）三段通过两个**缝隙**接起来，
两个缝隙都有离线等价实现，所以三人能各自在本机把逻辑跑绿再上 Jetson 联调。

详见 [`exp3/README.md`](exp3/README.md)（契约、目录职责、三人下一步）。
