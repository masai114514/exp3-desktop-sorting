# exp3 日志 schema（v0）— 与 c4_2 result.json 口径对齐

落地目录：`exp3_logs/run_<时间戳>/`
- `run.log`       人类可读事件流（对齐 c4_2）
- `events.jsonl`  结构化事件（对齐 c4_2）
- `records.jsonl` **每物体 1 条**（识别/抓取/放置/异常）
- `result.json`   运行级汇总（verdict 口径同 c4_2：PASS/FAIL/TRIAL/RUNNING）

## 每物体记录（records.jsonl，一行一条）

| 字段 | 含义 |
|---|---|
| ts | 记录时刻 |
| cell_id | 命中网格（c1..c6）或 null |
| cls | 识别类别（cup/mouse）或 null |
| conf | 置信度（0..1）或 null |
| decision | `exec`(可执行) / `skip` |
| skip_reason | 跳过原因：`out_of_grid` / `unrecognized_object`，可执行则为 null |
| grasp_ok | bool 或 null（未执行） |
| place_ok | bool 或 null |
| reason | 动作失败原因：`grasp_failed`/`dropped_or_missed_place`/`unreachable_target`/`exec_error`/`safety_stop`；成功为 null |
| detail | 自由文本 |

## result.json（运行级）

| 字段 | 含义 |
|---|---|
| objects_seen | 本轮上桌看到的物体数 |
| placed_ok | 正确分类放置数 |
| pass_line | docx 线（默认 5） |
| required_total | docx 物体数（默认 6） |
| verdict | placed_ok>=pass_line 且 objects_seen>=required_total 时 PASS，否则按 c4_2 习惯标 FAIL/TRIAL |
| exit_status | `no_target`(处理完) / `no_exec`(剩不可执行目标) / `safety_stop`(保护停止) |
| reasons | reason → 次数 直方图 |
| run_dir | 本 run 目录 |

> verdict 规则复刻 c4_2：只有「完整的验收轮（>=required_total 物体走完）」才下 PASS/FAIL，
> 中途/试跑标 TRIAL，运行中标 RUNNING，避免把试跑误读成验收失败。
