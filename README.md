# exp3-desktop-sorting — 实验3《桌面物体自动分类整理》（公开参考）

三人机器人集成小组实验3 的**共享接口规格 + 任务控制核心**公开仓库。内容在 [`exp3/`](exp3/)，
clone 下来即可离线单测（不需 ROS、不需真机、不需相机）。**工程分两条线**：仿真线（mechArm +
Gazebo + ROS 2，组员甲）与真机线（**RoboMaster EP 工程形态，不走 ROS 2**，组长）。

> 与三个实验仓库的区分，clone 前先读：
>
> | 实验 | 仓库 | 可见性 | 内容 |
> |---|---|---|---|
> | 实验1（检测模型训练） | `masai114514/object_detection_lab` | public | YOLOv8 cup/mouse 训练/推理（乙复用） |
> | 实验2（定点抓取） | `masai114514/mecharm-grasp-exp` | private | 仿真 + **EP 真机**定点抓取基建（`ep/drive/rm.py` 驱动） |
> | **实验3（本仓库）** | **`exp3-desktop-sorting`** | **public** | **多格多料盒自动分类整理：接口契约 + 任务控制核心 + 两条线的缝隙实现** |
>
> **本仓库自包含的部分**：任务控制核心（`sort_core/`）、接口契约、配置自检、日志 schema，
> 以及仿真线与真机线两个缝隙的实现，全部可离线跑通、可离线单测。
>
> **两个例外，需要实验2 的私有仓库**（找组长要 `mecharm-grasp-exp` 访问权）：
> ① 仿真执行件 `ros2/task_control/c4_executor.py` 组合实验2 的 `C4Driver` 运动原语；
> ② 真机执行件 `real/ep_backend.py` 组合实验2 的 **`ep/drive/rm.py`**（EP SDK 驱动）。
> 驱动**故意不在本仓库再抄一份**（两份驱动一定会漂），所以**本仓库里连不了真机——
> 这是设计，不是缺文件**。

## 目录

```
exp3/                     工程根（接口契约 + 纯 Python 任务控制核心 + 两条线的缝隙）
├── README.md             契约与目录导览（权威文档，改动三方同步）
├── contract/PickPlace.action   抓取接口权威语义（阶段号 / reason 口径）
├── sort_core/            任务控制纯 Python 核心（无 ROS import，两条线共用）
├── examples/             离线端到端 demo（mock 桌面，真实 config 跑控制循环）
├── config/               仿真线参数（甲的场景坐标；当前为示例占位）
├── config_real/          真机线参数（现场实测；**出厂全是 null 占位**，见下）
├── config_check.py       回填自检器：改 config 后先跑；--require-calibrated 查标定闸门
├── ros2/                 仿真线（mechArm + Gazebo + ROS 2）
│   ├── sort_msgs/        PickPlace 接口包（colcon ament_cmake）
│   ├── reference/        乙改 detection_node 的参考骨架（→ Detection2DArray）
│   └── task_control/     仿真线缝隙：exec_contract / c4_executor / server / client / scan
├── real/                 真机线（RoboMaster EP，**无 ROS 2**）
│   ├── run_real.py       单命令入口：相机 → 检测 → 任务控制 → EP
│   ├── ep_backend.py     EPPickExecutor：执行契约六段 → EP 原语
│   ├── camera.py         USB 相机取帧 + 检测器接口（含 mock）
│   └── dry_run.py        假 EP：不碰硬件把整条动作链打出来
└── tests/                unittest（离线全绿：geometry/decision/task/run_demo/config_check/
                          c4_executor/ep_backend/run_real）
```

## 离线先跑（无需 ROS / 真机 / 相机）

```bash
cd exp3
python3 -m unittest discover -s tests -t . -v   # 205 个用例，全绿
python3 examples/run_demo.py --mode happy       # mock 桌面端到端：完整控制循环 → exp3_logs/
python3 config_check.py --strict                # 甲/乙回填 config 后先自检
```

### ⚠ 真机入口在本仓库里会被**拒绝启动** —— 这是设计，不是故障

`python3 real/run_real.py --config-dir config_real` 会打印一长串「未标定」并 `exit 1`，
**连机器人都不会连**。原因：`config_real/` 出厂的标定项一律是 `null`（不是编得像真的占位值），
而真机入口的第一件事就是**标定闸门**——没量过的坐标不许驱动机械臂。设计动机见
[`exp3/README.md`](exp3/README.md) 的「未标定拒跑闸门」一节。

想看**真机链路的完整动作链**（六段 → EP 原语的调用序列、失败注入、人工确认凭据），
跑这几个即可，它们用**临时目录造一份标定**，不碰硬件：

```bash
python3 -m unittest tests.test_run_real.TestDryRunHappyPath -v   # 六个物体走完整链
python3 -m unittest tests.test_ep_backend -v                     # 逐段失败注入 → reason 映射
```

## 这套东西解决的问题

桌面摆 4–6 个取物格、cup/mouse 两类 ≥6 物，相机识别后按类放进对应料盒：
识别（乙）— 任务状态机（本轮仓库核心）— 抓取执行（甲 = 仿真 / 组长 = 真机）三段通过两个
**缝隙**接起来，两个缝隙都有离线等价实现，所以三人能各自在本机把逻辑跑绿再上硬件联调。
**换平台因此不改接口**：仿真换真机只换了这两个缝隙的实现，契约与日志 schema 一行未改。

详见 [`exp3/README.md`](exp3/README.md)（契约、目录职责、两条线的分工与下一步）。
