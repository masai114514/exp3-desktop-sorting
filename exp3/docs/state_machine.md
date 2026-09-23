# 任务状态机规格 — exp3《桌面物体自动分类整理》

> **这是 docx §七.3 要求的提交物**（原文：「BehaviorTree XML **或**状态机配置文件」）。
> 我们没有用 BehaviorTree，用的是**显式状态机**，所以交两样东西：
>
> | 落地物 | 是什么 | 路径 |
> |---|---|---|
> | **本文件** | 状态机的**规格**（状态、转移、判定规则、终态语义） | `exp3/docs/state_machine.md` |
> | **配置文件** | 状态机的**参数/阈值**（`conf_min` / `fail_limit` / `no_exec_limit` / `max_rounds` 等） | `exp3/config/task.json`（仿真线）<br>`exp3/config_real/task.json`（真机线） |
> | 实现 | 执行上述规格的代码 | `exp3/sort_core/task.py` ─ `TaskController` |
>
> **本文件与代码是同一件事的两种写法**：改了任一侧必须同步另一侧。
> 校验方式是 `exp3/tests/test_task.py`（离线可跑，不需要 ROS、不需要机器人）。

## 0. 一句话

**一条循环走完一个物体**：扫描 → 逐条判定可执行性 → 取网格号最小的那个 → 调一次 `PickPlace` →
成功就把该网格记进 `done_cells`，失败就累加失败计数；**直到视野空、或只剩不可执行目标、或连续失败触顶**。

「谁来抓、怎么抓」全部由两个注入缝隙负责，状态机自己**只做决策与记账**：

```python
TaskController(task_cfg, bins_cfg, cells, scan, pick_place, safe_stop=None, log=None)
#                                   ^^^^  ^^^^^^^^^^  ^^^^^^^^^
#                                   两个缝隙：仿真线走 ROS2 Action，真机线走 EP SDK
```

---

## 1. 状态表

实现上是一个 `while rounds < max_rounds` 的单循环 + 三个计数器，**不是**逐个显式的状态对象
（代码里没有 `class SCAN` 这种东西）。下表是它与 `TaskController.run()` **逐段等价**的状态机写法：
「本状态做什么」一列就是对应代码块在做的事，按顺序读下来即 `run()` 的全文。

| 状态 | 代号 | 进入条件 | 本状态做什么 | 出口 |
|---|---|---|---|---|
| 初始化 | `INIT` | `run()` 被调用 | `placed_ok=0`；`done_cells`/`seen_cells`/`logged_skip` 置空；`fail_streak=0`；`no_exec_streak=0`；`rounds=0` | `SCAN` |
| 扫描 | `SCAN` | 每轮开头 | `rounds += 1`；`dets = scan()` | `dets` 为空 → `DONE`；否则 `DECIDE` |
| 判定 | `DECIDE` | 拿到本轮检测 | 对**每一条**检测跑 `locate()` + `classify()`，拆成「可执行」与「跳过」两堆 | 可执行堆非空 → `SELECT`；空 → `NOOP` |
| 选目标 | `SELECT` | 可执行堆非空 | `sort_by_cell(...)[0]` ─ 按 `(row, col)` 升序取第一个 | `EXEC` |
| 执行 | `EXEC` | 已选定 `(cell_id, cls)` | 调 `pick_place(cell_id, cls)`，阻塞等待结果 | 返回 `ok` → `SUCCESS`；返回任一 `ACTION_REASONS` → `FAIL` |
| 成功 | `SUCCESS` | `pick_place` 返回 `ok` | `placed_ok += 1`；`done_cells.add(cell_id)`；**两个 streak 清零** | `SCAN` |
| 失败 | `FAIL` | `pick_place` 返回失败 reason | `fail_streak += 1`；`reasons[reason] += 1` | `fail_streak >= fail_limit` → `SAFETY_STOP`；否则 `SCAN` |
| 空转 | `NOOP` | 本轮有检测，但**一条可执行的都没有** | `no_exec_streak += 1` | `>= no_exec_limit` → `NO_EXEC`；否则 `SCAN` |
| 结束·正常 | `DONE` / `NO_TARGET` | `scan()` 返回空 | 置 `exit_status = no_target` | **终态** |
| 结束·无目标可做 | `NO_EXEC` | 空转触顶，或 `max_rounds` 兜底 | 置 `exit_status = no_exec` | **终态** |
| 结束·安全停止 | `SAFETY_STOP` | 连续失败触顶 | `exit_status = safety_stop`；调 `safe_stop(reason)` 回调 | **终态** |

> **`SUCCESS` 是唯一会清零 `fail_streak` / `no_exec_streak` 的状态。**
> 也就是说「连续失败」数的是**不间断**的失败次数，中间插一轮跳过**不算**中断 —— 这是有意的：
> 一台开始连续抓不住的机器，不该因为某一轮恰好没得抓就被重新计数。

## 2. 转移表

| 当前状态 | 条件（按优先级自上而下） | 下一状态 | 副作用（同时落盘） |
|---|---|---|---|
| `SCAN` | `len(dets) == 0` | `NO_TARGET` | 无记录（这是正常结束） |
| `SCAN` | `len(dets) > 0` | `DECIDE` | 无 |
| `DECIDE` | 该检测**不可执行**，且 `(cell_id, cls)` 不在 `logged_skip` | 留在 `DECIDE` | `reasons[skip_reason] += 1`；`records.jsonl` 写一条 `decision=skip`；`logged_skip` 记住该键 |
| `DECIDE` | 该检测**不可执行**，但 `(cell_id, cls)` 已在 `logged_skip` | 留在 `DECIDE` | **不写记录、不计数**（同一目标的持续跳过只记一次） |
| `DECIDE` | 该检测**可执行** | 留在 `DECIDE` | `seen_cells.add(cell['id'])` |
| `DECIDE` | 可执行的网格已在 `done_cells` | 留在 `DECIDE` | 不写记录（换目标） |
| `DECIDE` | 判定完毕后可执行堆非空 | `SELECT` | 无 |
| `DECIDE` | 判定完毕后可执行堆为空 | `NOOP` | 无 |
| `EXEC` | 结果 `ok` | `SUCCESS` | `records.jsonl` 写一条 `decision=exec, grasp_ok=true, place_ok=true, reason=null` |
| `EXEC` | 结果是 `ACTION_REASONS` 之一 | `FAIL` | `records.jsonl` 写一条 `decision=exec, grasp_ok=false, place_ok=false, reason=<reason>`；`reasons[reason] += 1` |
| `EXEC` | 返回**非法** reason | ─ | 代码里是 `assert res in ACTION_REASONS`：直接抛断言错，属于**接线错误**，不算运行期异常 |
| `FAIL` | `fail_streak >= fail_limit` | `SAFETY_STOP` | `reasons[safety_stop] += 1`；调 `safe_stop('safety_stop')` |
| `NOOP` | `no_exec_streak >= no_exec_limit` | `NO_EXEC` | `reasons[no_exec] += 1` |
| 任一 `SCAN`/`EXEC` | `rounds` 已达 `max_rounds` | `NO_EXEC` | 循环退出后若 `exit_status is None`，兜底置 `no_exec` 并计数 |

### 主循环流程图

```
                 ┌──────────────────────────────┐
                 │ INIT                         │
                 │ placed_ok=0  streaks=0       │
                 └───────────────┬──────────────┘
                                 v
        ┌─────────────>  SCAN  rounds+1, dets=scan()
        │                        │
        │              ┌─────────┴─────────┐
        │         dets 空              dets 非空
        │              v                   v
        │        NO_TARGET ──►终态      DECIDE  逐条 classify
        │                                  │
        │                    ┌─────────────┴─────────────┐
        │              可执行堆非空                 可执行堆为空
        │                    v                           v
        │                 SELECT 取 (row,col) 最小      NOOP  streak+1
        │                    v                           │
        │                 EXEC  pick_place()        ┌───┴────┐
        │                    │                  <limit   >=limit
        │          ┌─────────┴─────────┐            │        v
        │        ok                reason          └──►  NO_EXEC ──►终态
        │          v                   v
        │      SUCCESS  streak清零   FAIL  fail_streak+1
        │          │                   │
        │          │             ┌─────┴─────┐
        │          │         <fail_limit  >=fail_limit
        └──────────┴─────────────┘            v
        （回到 SCAN）                    SAFETY_STOP ──►终态
                                        safe_stop() 回调
```

## 3. 参数与阈值（＝状态机配置文件）

`config/task.json`（仿真线）与 `config_real/task.json`（真机线）**schema 完全一致**，
状态机读到的键一字不差 —— 这就是「换目录即换场景、代码一行不改」的含义。

| 键 | 仿真线 | 真机线 | 含义 | 改它会影响什么 |
|---|---|---|---|---|
| `classes` | `["cup","mouse"]` | 同 | 本实验的类别集合 | 须与检测模型类别表一致；不一致 → 全部落 `unrecognized_object` |
| `expected_total` | `6` | `6` | docx 要求的物体总数 | **只进 `result.json`，不进状态机循环**；决定 verdict 是 PASS/FAIL 还是 TRIAL |
| `pass_line` | `5` | `5` | docx 的及格线（6 个里 ≥5） | 同上 |
| `conf_min` | `0.5` | `0.5` | 置信度下限 | 调高 → 更多 `unrecognized_object` 跳过 |
| `fail_limit` | `3` | `3` | 连续失败上限 → `safety_stop` | 调小 → 更早保护停止，但容错变差 |
| `no_exec_limit` | `3` | `3` | 连续空转上限 → `no_exec` | 调大 → 面对不可执行目标多耗几轮才退出 |
| `max_rounds` | `60` | `60` | 总轮数硬上限（兜底） | 撞上也会 `no_exec` 退出，防死循环 |
| `log_dir` | `exp3_logs` | **`exp3_logs_real`** | 日志落地目录 | 两条线**故意分开**，别混 |

> `camera.calib` 与 `image_width_px/height_px` 不在状态机的转移条件里直接出现（它们是被
> `DECIDE` 状态里的 `locate()` 消费的），但同属这份配置文件，故一并列出。真机线的 `camera.calibration.status`
> 出厂是 `uncalibrated` —— 那是 `config_check.py --require-calibrated` 的**闸门**，
> 在状态机启动之前就把人拦下。

## 4. 判定规则：什么算「可执行目标」

`DECIDE` 状态对每条检测调用 `sort_core/decision.py::classify()`，**按顺序**过三关，
第一关不过就返回，不再往下走：

| 顺序 | 条件 | 结论 | reason |
|---|---|---|---|
| 1 | 类别不在 `bins.json:class_to_bin` 里 | 不可执行 | `unrecognized_object` |
| 2 | `conf < conf_min` | 不可执行 | `unrecognized_object` |
| 3 | 检测框中心 `locate()` 后**不落在任何网格内** | 不可执行 | `out_of_grid` |
| ─ | 以上皆过 | **可执行**，带上 `cell` 几何供取放 | `null` |

> 第 3 关的 `locate()` 支持两种标定模式：`rectilinear`（光轴垂直向下）与
> `homography`（斜俯视，3×3 H）。**真机侧支架是斜俯视，必须用 `homography`** ——
> 用错模式的后果不是报错，而是**框心算出来的桌面坐标整体偏**，表现为「明明在格子里却判 `out_of_grid`」。

**同一优先级的多个可执行目标怎么选？** `sort_by_cell()` 按 `(row, col)` 升序 —— 从左到右、
从前到后，**顺序确定且可复现**，不依赖检测框的返回顺序。

## 5. 三个集合：去重口径

状态机只维护三个 `set`，全部语义都靠它们：

| 集合 | 里面是什么 | 何时写入 | 作用 |
|---|---|---|---|
| `done_cells` | **已成功处理完**的网格 id | 仅在 `SUCCESS` 状态 | 该网格不再二次取放（防止一个格子里还剩别的东西时反复抓同一个格） |
| `seen_cells` | **曾检测到可执行目标**的唯一网格 id | 在 `DECIDE` 里、**在 `done_cells` 检查之前** | 就是 `result.json` 的 **`objects_seen`** |
| `logged_skip` | `(cell_id, cls)` 二元组 | 每次写 `decision=skip` 记录时 | 同一个跳过目标只记一次，不让 `records.jsonl` 被刷屏 |

> **`objects_seen` 为什么是「唯一网格数」而不是「放好的 + 结束时再扫一遍的残量」？**
> 后者会把**已经放回料盒的物体**再数一遍（料盒若在视野内），导致 `objects_seen` 虚高 ——
> 空网格轮的 `verdict` 可能被误判成 `PASS`。改成「唯一网格数」之后，
> 空一格摆 5 个物 → `objects_seen=5 < expected_total=6` → `verdict=TRIAL`。
> 这条口径的定义写在**仓库外**的 `机器人实验/实验3_现场作业包_20260916/4_异常注入方案.md`
> （该文件第一张表就是「两类异常的 verdict 不一样」）；本口径的落地改动在 commit `89d69d8`
> （`sort_core/task.py` 的 `objects_seen` 从 `placed_ok + residual` 改为 `len(seen_cells)`）。

## 6. 终态与 `exit_status`

`run()` 返回的 `summary` 里只有 4 个键：`placed_ok` / `objects_seen` / `exit_status` / `reasons`。
其中 `exit_status` 只有三种取值，**没有第 4 种**：

| `exit_status` | 含义 | 是失败吗 |
|---|---|---|
| `no_target` | `scan()` 返回空 —— 桌上已经没有目标了 | **不是**，这是正常结束 |
| `no_exec` | 连续 `no_exec_limit` 轮「有检测但一条可执行都没有」；或 `max_rounds` 兜底 | **不是失败**，是「剩下的都做不了，停在安全状态」 |
| `safety_stop` | 连续 `fail_limit` 次动作失败 | **是**保护触发，`safe_stop()` 回调已执行 |

> ★ **一条容易误读的现场现象**：末次扫描若仍能看到**料盒里的物体**（框落在网格外 → 判
> `out_of_grid` 跳过），那一轮的 `exit_status` 会是 **`no_exec` 而不是 `no_target`**。
> **这不是失败。** 验收判据看的是 `verdict` + `placed_ok`，不是 `exit_status` 是不是 `no_target`。

## 7. 运行级结论 `verdict`（与状态机的分工）

状态机**不产生** `verdict`；它把 `placed_ok` / `objects_seen` / `exit_status` / `reasons` 交给
`sort_core/logging_util.py::verdict_for()`，由后者按 c4_2 的既有习惯下结论：

| `verdict` | 条件 | 用途 |
|---|---|---|
| `RUNNING` | 还没跑完（`final=False`） | 中断/进行中 |
| `PASS` | 跑完 **且** `objects_seen >= expected_total` **且** `placed_ok >= pass_line` | 验收轮达标 |
| `FAIL` | 跑完 **且** `objects_seen >= expected_total` **但** `placed_ok < pass_line` | 验收轮不达标 |
| `TRIAL` | 跑完 **但** `objects_seen < expected_total` | **试跑 / 异常轮**（如空网格轮只有 5 个物） |

> 这条分级是**故意的**：只有「>= `expected_total` 个物体走完的完整轮」才配下 `PASS`/`FAIL`，
> 目的就是**不让试跑被误读成验收失败**，也**不让不完整的轮被误读成达标**。
> 现场的空网格异常轮必然是 `TRIAL` —— 这不是没做到，这是口径。

## 8. 一次 `EXEC` 内部的 6 段（动作级子状态机）

上面第 1 节的状态机把一次 `PickPlace` 当**黑盒**：它只关心返回 `ok` 还是某个 reason。
盒子**里面**还有一层子状态机，由 Action server / `EPPickExecutor` 执行，通过
`feedback.stage` 每进入一段发一次消息（定义见 `contract/PickPlace.action`，
阶段名常量见 `sort_core/taxonomy.py`）：

| stage | 名称 | 干什么 |
|---|---|---|
| `1` | `approach` | 运动到该网格上方 / 开始下探 |
| `2` | `grasp` | 下降 + 夹取（压紧） |
| `3` | `lift` | 抬起，这里要判 **HELD**（夹住了没有） |
| `4` | `move_to_bin` | 搬运到料盒上方 |
| `5` | `place` | 下降 + 松爪，这里要判 **PLACED**（放进去了没有） |
| `6` | `retract` | 退回安全位 |
| `-1` | `failing` | 失败正在收尾，随后回 `result` |

**内外两层的分工是刻意的**：内层（执行）可能因为「夹空了」失败 → 返回 `grasp_failed`；
外层（状态机）据此累加 `fail_streak`，**连续 3 次**才拉起 `safety_stop`。
单次失败不终止任务 —— 后续轮会重扫、重试，这正是 docx §三.6 要求的「抓取失败」处理。

## 9. 状态 → 日志字段对应

落地目录 `exp3_logs/run_<时间戳>/`（真机线是 `exp3_logs_real/`）。

**注意分工**：状态机自己**只写两样**（`records.jsonl` 与 `run.log` 的最后一行汇总），
`run.log` 的其余内容与 `result.json` 由**入口脚本**和**两个缝隙**写：

| 文件 | 谁写 | 内容 |
|---|---|---|
| `records.jsonl` | 状态机：`DECIDE`（每条 skip 一条）、`SUCCESS` / `FAIL`（每次 exec 一条） | **每物体 1 条**，字段见 `config/log_schema.md` |
| `run.log` | 状态机只写**最后一行汇总**（`exit_status=... placed_ok=...`）；其余由缝隙与入口写（真机线 `ep_backend.py` 每进入一个 stage 写一行、`camera.py` 写取帧信息、`run_real.py`/`run_demo.py` 写表头与证据等级） | 人类可读事件流，带相对时间戳 |
| `result.json` | **入口脚本**调 `logging_util.write_result()`（`examples/run_demo.py`、`real/run_real.py`、`exp3_sim/sim_task.py`）；状态机只负责把 `summary` 交出去 | `placed_ok` / `objects_seen` / `verdict` / `exit_status` / `reasons` / `run_dir` |
| `events.jsonl` | ★ **`sort_core` 侧目前不产出这个文件**。`config/log_schema.md` 把它列为 run 目录的组成之一，但实际只有真机 EP 线自己的日志器 `ep/drive/ep_log.py` 在写。这是**文档与实现的一处不一致**，已记入「待对齐」清单；在补齐之前，验收证据请以 `records.jsonl` + `result.json` 为准 | — |

`records.jsonl` 一行和一个状态的对应关系：

| 触发状态 | `decision` | `grasp_ok` | `place_ok` | `reason` | `skip_reason` |
|---|---|---|---|---|---|
| `DECIDE`（跳过） | `skip` | `null` | `null` | `null` | `out_of_grid` / `unrecognized_object` |
| `SUCCESS` | `exec` | `true` | `true` | `null` | `null` |
| `FAIL` | `exec` | `false` | `false` | 5 种 `ACTION_REASONS` 之一 | `null` |

> `reasons` 直方图会**同时**装跳过类与动作类 reason（键名不重叠）。
> `safety_stop` 计数是在 `FAIL → SAFETY_STOP` 转移时**额外加一次**的，
> 所以它出现 ≥1 次就意味着这一轮触顶了。

## 10. 复现与验证（离线，不需要 ROS、不需要机器人）

```bash
cd exp3
python3 -m unittest discover -s tests -t .     # 全套 205 项（其中 25 项需真机/ROS 自动跳过）
python3 -m unittest tests.test_task -v          # 只验状态机：转移、三个集合、degradation 口径
python3 examples/run_demo.py --mode happy       # 端到端：mock 桌面跑完整循环 → exp3_logs/
python3 examples/run_demo.py --mode safety_stop # 连失败 → 观察 SAFETY_STOP 转移
python3 examples/run_demo.py --mode unrecognized
python3 examples/run_demo.py --mode out_of_grid
python3 config_check.py --strict                # 配置文件结构自检
```

`examples/run_demo.py` 的每种 mode 就是**状态机某条路径的活体演示**，
跑完直接看 `exp3_logs/run_<ts>/`：`records.jsonl` 里能看到 `decision=skip` 的行，
`result.json` 里能看到对应的 `exit_status` 与 `reasons` 直方图。

## 11. 与 docx 条款的对照

| docx 出处 | 要求 | 本状态机怎么满足 |
|---|---|---|
| §二.4 | 抓取接口用 Action 或等价的可反馈长时间任务接口 | `EXEC` 状态调的就是 `PickPlace` Action；6 段 feedback 见第 8 节 |
| §二.5 | 任务控制用 BehaviorTree **或状态机** | **本文件 + `config/task.json`** |
| §三.4 | 调用抓取动作，完成取物、分类放置和返回 | `EXEC`＝取物+放置；`retract` 段（stage 6）＝返回安全位 |
| §三.5 | 循环处理，直至全部完成或没有可执行目标 | `while rounds < max_rounds`；出口 `no_target` / `no_exec` |
| §三.6 | 处理空网格、未识别、不可达、抓取失败 | 空网格→框不落格→`out_of_grid`；未识别→`unrecognized_object`；不可达→`unreachable_target`；抓取失败→`grasp_failed`；连续 3 次→`safety_stop` |
| §四V3 | 不得人工指定类别、网格、料盒 | 状态机**没有任何人工入口**：类别来自检测。网格来自 `locate()`，料盒来自 `class_to_bin` 查表 |
| §四V5 / §五R6 | 保存识别、抓取、任务状态日志；每物 4 字段 | 第 9 节的三个写入点，字段见 `config/log_schema.md` |
| §六W5 | 异常时能安全停止 | `SAFETY_STOP` 终态 + `safe_stop()` 回调 |

---

## 附：改动本状态机时的检查清单

1. 改 `sort_core/task.py` → 同步改本文件的状态表/转移表 → 跑 `tests/test_task.py`。
2. 改阈值 → 只动 `config/task.json` 与 `config_real/task.json`（**两份都要改**，
   除非有意让两条线不同）→ 跑 `config_check.py --strict`。
3. 新增 reason → 必须进 `taxonomy.py` 的对应集合（`SKIP_REASONS` / `ACTION_REASONS` /
   `EXIT_REASONS`），否则 `EXEC` 状态的 `assert` 会拦住你；同步改 `config/log_schema.md` 与本文件第 9 节。
4. 改 `PickPlace` 阶段号 → 同步改 `contract/PickPlace.action`、`taxonomy.py` 的 `STAGE_*`、
   `ros2/sort_msgs/README.md` 的阶段表（三处）。
