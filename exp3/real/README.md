# 实验3 真机线（RoboMaster EP）

真机不是另写一套系统，而是把仿真线的**两个缝隙**换掉。任务逻辑、契约、reason 枚举、
日志 schema 两边完全共用：

```
        仿真线                                  真机线（本目录）
  ┌──────────────────┐                    ┌──────────────────┐
  │ scan             │  ← 缝隙 1 ────→    │ camera.py        │  相机取帧 + 检测器
  │ pick_place       │  ← 缝隙 2 ────→    │ ep_backend.py    │  EP 六段执行
  └────────┬─────────┘                    └────────┬─────────┘
           └────────────┬──────────────────────── ──┘
                sort_core/（ROS-free，两边共用，一行不改）
```

配置也分目录：`config/` 是甲的仿真场景坐标，`config_real/` 是现场实测值。两套坐标系的
数不可能一样，但 schema 一样 —— `sort_core` 只吃内容，不关心文件在哪个目录。

## 文件

| 文件 | 作用 |
|---|---|
| `run_real.py` | 入口：相机 → 检测 → `TaskController` → EP。**闸门在这里** |
| `ep_backend.py` | `EPPickExecutor`（六段 → EP 原语）+ `EPPickPlace`（reason 映射） |
| `camera.py` | 相机（USB / 图片 / 无）+ 检测器（mock / YOLO / 外部文件） |
| `dry_run.py` | 假 EP：不连硬件，把动作链打出来（演练用） |
| `scenes/` | mock 检测器的物体表（真机用真检测器时不需要） |
| `../config_real/` | 真机配置：`task`/`grid_cells`/`bins` + `ep_waypoints`（EP 位姿表） |

## 跑之前：四份标定记录

`run_real.py` 启动第一件事是过闸门，四份标定记录**任一没填**就退出，**连机器人都不连**。

| 记录在哪 | 谁做 | 填什么 |
|---|---|---|
| `config_real/task.json` → `camera.calibration` | 乙 | 相机标定（`calib` 的 mode/H + 反投影误差） |
| `config_real/grid_cells.json` → `calibration` | 组长/甲 | 卷尺量每格两角坐标，删掉各 cell 的 `_note` |
| `config_real/bins.json` → `calibration` | 组长 | 卷尺量料盒中心，删掉各 bin 的 `_note` |
| `config_real/ep_waypoints.json` → `calibration` | 组长 | 8 个底盘位姿 + 臂两档 + 夹爪档位（见 `ep/标定说明.md`） |

随时可查进度：

```bash
cd exp3
python3 config_check.py --config-dir config_real                        # 看还差什么
python3 config_check.py --config-dir config_real --require-calibrated   # 闸门口径（真机开工前必须 0）
```

**为什么要做成硬门**：实验二栽过一次 —— config 里编的占位值照样通过全部结构检查
（A/B 恰好满足"相距 > 0.1m"），于是"是否已标定"这条检查从来没起过作用。真机拿没标定的
点位跑，轻则空抓、重则撞桌。所以这里 (a) 未标定项一律 `null`（编不出像样的默认值）、
(b) 占位项留 `_note` 标记、(c) 闸门在**连机器人之前**。

## 命令

```bash
cd exp3

# ① 演练：不连 EP、不动真机，只把动作链打出来。标定填完先跑这个
python3 real/run_real.py --config-dir config_real --dry-run \
        --camera none --detector mock --assume-judge --no-pause

# ② 真机：USB 相机 + 乙的模型。HELD/PLACED 会问你 y/n
python3 real/run_real.py --config-dir config_real \
        --camera cv2:0 --detector file --detector-file /path/to/detector.py

# ③ 真机 + YOLO（.pt）
python3 real/run_real.py --config-dir config_real --camera cv2:0 \
        --detector yolo --model /path/to/best.pt
```

常用开关：`--camera none|cv2:N|image:/path`、`--detector mock|yolo|file`、
`--log-dir`、`--no-pause`、`--assume-judge`（★跳过人工确认，只给无人值守演练）。

检测器交给乙的接口就一个函数（`--detector file`）：文件里定义

```python
def detect(frame) -> [ {'cls': 'cup', 'conf': 0.87, 'bbox': [x1, y1, x2, y2]} ]   # 像素 xyxy
```

## 真机运行纪律

1. **先 `--dry-run` 一遍**，看动作链是不是你想的那样（会打印 66 步原语调用）。
2. **第一轮盯全程**。`--assume-judge` 不要开 —— 开着就等于没人看夹住没有。
3. 人工只在两个地方被问：**HELD**（夹住了吗）和 **PLACED**（放正了吗）。按 `q` 或
   Ctrl-C 中止整轮，程序会收尾（开爪 → 抬到高 → 回 HOME）。
4. 类别判断和动作触发**全程由系统做**，人不参与（docx §六）。跑了多少次人工确认会写进
   `real_run.json` 的 `operator_confirms`，正常应等于 `2 × 物体数`。
5. 换场地 / 重新上电 → **底盘必须重标**（odom 原点跟着上电位姿走），
   把 `ep_waypoints.json` 的 `calibration.status` 打回 `uncalibrated`。

## 异常测试怎么做（docx §六 要 ≥2 类）

不用改代码、也不用改 scene —— 直接在桌上摆出来：

| 异常 | 怎么造 | 日志里看 |
|---|---|---|
| 未识别物体 | 桌上放一个没训练过的物体（杯子/鼠标之外） | `unrecognized_object`，该物跳过，其余照常分类 |
| 置信度不足 | 同上（模型给低分）或遮挡目标物 | 同上（`conf < conf_min`） |
| 空网格 | 6 个格子里只摆 5 个 | 不报错，扫到的照样分类 |
| 抓取失败 | 目标物太滑 / 故意少夹一点 | `grasp_failed`，自动重抓一次，连续 3 次 → `safety_stop` |

## 日志

落 `exp3_logs_real/run_<时间戳>/`（仿真线是 `exp3_logs/`，两边的日志**故意分开**）：

- `run.log` —— 事件流：每轮识别结果、每段动作、失败在哪一段
- `records.jsonl` —— 每物体一条：`reason` + `detail`（含 `held=`/`placed=`/`retract=` 的来源）
- `result.json` —— 汇总与 verdict，**schema 与仿真线完全相同**（可直接比对两线结果）
- `real_run.json` —— 真机特有的上下文：是否演练、标定记录、人工确认次数、动作链步数

`detail` 里 `held=已抬起（assume_ok）` 和 `held=已抬起（操作员确认夹住）` 是能区分的 ——
验收时要能证明"夹起/放正是人看的"，这一条就是凭据。

## 已知限制

- **EP 没有物块感知**。夹住没有、放正没有靠人工确认（对齐实验二 `judge=operator_confirm`）。
  `assume_ok` 是演练口径，不是验收口径。
- **底盘是开环定位**。odom 有漂移，每轮从 HOME 出发能抵消一部分；`goto` 每段读 odom 做误差
  纠正，但纠正不了累积误差。点位标定时把格间距留够余量。
- **`--dry-run` 不是验收凭据**。它只证明链路通，不证明能抓起来。
- 相机支架是斜俯视（垫高放桌上）⇒ `camera.calib` 要用 `homography`，不是 `rectilinear`。
- `real/run_real.py` 连真机时 import 的是 `../ep/drive/rm.py`（实验二那套，故意不抄第二份）。
  exp3 的独立快照里没有 `ep/`，那边只能 `--dry-run` —— 公开仓本来也没有机器人。

## 离线测试

```bash
cd exp3 && python3 -m unittest discover -s tests -t . -v
```

`tests/test_ep_backend.py` 逐段注入失败，验"段号 → reason"的对应与失败即收尾；
`tests/test_run_real.py` 验闸门拦得住、dry-run 全链路通、四类异常各自的表现。
