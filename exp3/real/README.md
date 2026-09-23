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
| `calibrate.py` | 标定工具：读数**直接写进表**，不用手抄（步骤见 [标定说明.md](标定说明.md)） |
| `ep_conn.py` | 连 EP 的那一段（`run_real` 与 `calibrate` 共用，免得不一致） |
| `ep_backend.py` | `EPPickExecutor`（六段 → EP 原语）+ `EPPickPlace`（reason 映射） |
| `camera.py` | 相机（USB / 图片 / 无）+ 检测器（mock / YOLO / 外部文件） |
| `dry_run.py` | 假 EP：不连硬件，把动作链打出来（演练用） |
| `scenes/` | mock 检测器的物体表（真机用真检测器时不需要） |
| `../config_real/` | 真机配置：`task`/`grid_cells`/`bins` + `ep_waypoints`（EP 位姿表） |

`calibrate.py` 为什么不复用实验二的 `ep/drive/env_check.py`：那个 `--odom` 写死标 HOME/A/B
三个点、`--jog` 读的是 `ep/config_ep.json` 的臂档位。实验三要 **9 个底盘位姿 + 臂两档 +
夹爪两档**，拿它标只能肉眼看屏幕抄数 —— **抄错一个就是一个撞桌的点位**。

## 跑之前：四份标定记录

`run_real.py` 启动第一件事是过闸门，四份标定记录**任一没填**就退出，**连机器人都不连**。

| 记录在哪 | 谁做 | 填什么 |
|---|---|---|
| `config_real/task.json` → `camera.calibration` | 乙 | 相机标定（`calib` 的 mode/H + 反投影误差） |
| `config_real/grid_cells.json` → `calibration` | 组长/甲 | 卷尺量每格两角坐标，删掉各 cell 的 `_note` |
| `config_real/bins.json` → `calibration` | 组长 | 卷尺量料盒中心，删掉各 bin 的 `_note` |
| `config_real/ep_waypoints.json` → `calibration` | 组长 | 9 个底盘位姿 + 臂两档 + 夹爪档位（见 [标定说明.md](标定说明.md)） |

随时可查进度：

```bash
cd exp3
python3 real/calibrate.py --config-dir config_real                      # EP 那份：还差哪几项
python3 config_check.py --config-dir config_real                        # 四份一起看
python3 config_check.py --config-dir config_real --require-calibrated   # 闸门口径（真机开工前必须 0）
```

标定一条命令记一项，不用手抄：

```bash
python3 real/calibrate.py --odom                    # 推着车到位，实时看 odom
python3 real/calibrate.py --record c1               # 车已在位 → 记进 c1（c1..c6 / bin_* / home_pose）
python3 real/calibrate.py --record-arm grab 74 120  # 臂试到 (74,120)，确认后写 grab_low_mm
python3 real/calibrate.py --grip close --power 60   # 试夹一次（不写表）
python3 real/calibrate.py --record-grip close 60    # 试好了再记
python3 real/calibrate.py --seal --by <名字> --venue <场地>   # 填全才许封表
python3 real/calibrate.py --invalidate              # 换场地/重新上电：底盘作废，臂/夹爪不受影响
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
`--log-dir`、`--no-pause`、`--assume-judge`（★跳过人工确认，只给无人值守演练）、
`--confirm-each`（★每个物体动手**前**等一次回车，真机首跑人盯用）。

**三个开关会让本次运行不算验收凭据**（docx §六 要求"不进行人工类别判断和动作触发"）：

| 开关 | 意味着 | 本次运行 |
|---|---|---|
| `--dry-run` | 没碰真机 | rehearsal |
| `--assume-judge` | 没人确认夹住/放正 | rehearsal |
| `--confirm-each` | 人按回车才动作（人工触发动作） | rehearsal |

等级**由代码算出来**写进 `real_run.json` 的 `evidence_grade`（`acceptance` / `rehearsal`），
外加 `rehearsal_reasons` 列出是哪几个开关 —— 与其靠人记，不如落盘，验收时一查就知道
这一份能不能用。**正常的验收跑三个都不开。**

> HELD/PLACED **两处人工 y/n 不算**违背 §六：EP 没有物块位姿感知，判"夹住没有/放正没有"
> 只能人来；§六 禁的是**判类别**（类别由识别给）和**触发动作**（动作由任务控制给）。

检测器交给乙的接口就一个函数（`--detector file`）：文件里定义

```python
def detect(frame) -> [ {'cls': 'cup', 'conf': 0.87, 'bbox': [x1, y1, x2, y2]} ]   # 像素 xyxy
```

## 真机运行纪律

1. **先 `--dry-run` 一遍**，看动作链是不是你想的那样（会打印原语调用序列）。
2. **第一轮开 `--confirm-each` 盯全程**：每个物体动手前它会停下来等你回车，
   开着的那一轮记成 rehearsal（不能当验收凭据，但它本来就是给你看第一眼的）。
   `--assume-judge` 不要开 —— 开着就等于没人看夹住没有。
3. 人工只在两个地方被问：**HELD**（夹住了吗）和 **PLACED**（放正了吗）。按 `q` 或
   Ctrl-C 中止整轮，程序会收尾（开爪 → 抬到高 → 回 HOME）。
4. 类别判断和动作触发**全程由系统做**，人不参与（docx §六）。跑了多少次人工确认会写进
   `real_run.json` 的 `operator_confirms`，正常应等于 `2 × 物体数`。
5. 换场地 / 重新上电 → **底盘必须重标**（odom 原点跟着上电位姿走）：
   `python3 real/calibrate.py --invalidate` 再逐个 `--record`。
   **臂两档和夹爪档位不用重标**（它们在臂坐标系里，和车停在哪无关）。
6. 连不上 EP **不是"跑了失败"**：那种情况照样留一份 `result.json`（`placed_ok=0`、
   `exit_status=no_exec`），并在 `real_run.json` 里记 `outcome=connect_failed` 和错误原文 ——
   否则目录里只有 `run.log`、没有 `result.json`，两种事从文件上分不出来。

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
- `real_run.json` —— 真机特有的上下文：`evidence_grade`（能不能当验收凭据）、是否演练、
  标定记录、人工确认次数、动作链步数；连不上 EP 时这里是 `outcome=connect_failed`

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
`tests/test_run_real.py` 验闸门拦得住、dry-run 全链路通、证据等级算得对、连不上 EP 时
留下可分辨的记录、四类异常各自的表现；
`tests/test_calibrate.py` 验标定工具的防错（**钳制读数拒绝入表**、不误覆盖、臂写成 2 元列表、
封表条件、`--invalidate` 只废底盘）。
