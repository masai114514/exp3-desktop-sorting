# ep/ —— RoboMaster EP 真机定点取放(2026-09-08 起, 真机验收平台)

> 老师把**真机验收平台**从 mechArm 270 换成 **DJI RoboMaster EP(工程形态, 夹爪为主, 云台不参与)**;
> **任务与验收口径不变**: A→B 定点取放, 5 次连续 ≥4 成功, 全程留日志。
> **真机实际目标物 = 矿泉水瓶(约 550mL 塑料瓶), 放在地面**; 仿真侧(mechArm 270)用的才是
> 5cm 方块/桌面。两侧目标物不同是**真实差异不是笔误** —— 见
> `docs/真机侧证据_EP_20260914/README.md` 与"目标物口径"一节。
> **仿真实验仍用大象 mechArm 270 那套**(仓库 c4_2/ 与 mecharm_grasp/, 不涉及 EP)。
>
> 本目录 = EP 真机侧的取放 driver。`result.json` 的 schema 与 reason 枚举**和 c4_2 完全同构**
> → 验收记录表按同一列填即可。

## 背景一句话

EP 是**移动底盘**, 定位靠里程计(有漂移), **没有官方 ROS2/ros2_control** —— 官方只有
RoboMaster Python SDK(`robot.chassis / robotic_arm / gripper`)。所以这个 driver 直接走 SDK,
几何全靠现场标定, 代码在 Mac 上离线写、**未连真机跑过**; 首跑必须先过 `env_check.py`(探测
模块/属性名), 再 1 周期盯全程, 再 5 周期验收。

## 需要 Jetson 吗? —— 不需要(高频误解, 先看这里)

**误解来源**: 《机械臂定点抓取》指导书 §二 实验环境写的是"真机环境: **Jetson Orin**、mechArm 270、
夹爪或吸盘…"。那是 **mechArm 时代的条款**——旧真机方案 = mechArm 270 + Jetson Orin + ROS 2,
**已停用**, 只在 `c4_2/` 留档。指导书 docx 未随平台变更同步更新。

**当前口径(2026-09-08 起, 老师口头拍板, 验收即按此执行)**: 真机验收平台 = **RoboMaster EP**,
仿真实验平台仍 = mechArm 270(不必重跑)。指导书其余条款(动作流程、5 次 ≥4 成功、安全停止、
留日志)一律照旧适用。

EP 侧**不走 ROS2**, 所以:

- **不需要 Jetson**, 也不需要 Ubuntu 22.04 / ROS 2 Humble / ros2_control / Gazebo。
- 只要一台能跑 **Python 3.9~3.12** 的机器(笔记本 / ARM 板 / 树莓派均可)、能连上 EP 的 Wi-Fi,
  装上官方 RoboMaster Python SDK 就行; 无外网时用仓库 `deploy/ep_sdk_offline/`(见其 README)。
- **先查 Python 版本**: `python3 -V`。若手头真是 Jetson, JetPack 5 自带 **py3.8 低于要求**
  (需 JetPack 6 或另装 py3.10+); 我们只验证过该离线包在 py3.9~3.12 下的导入。
- Jetson 只在**仿真**侧才谈得上(它是"任一台 Ubuntu 22.04 + ROS2"的特例), 而仿真也不强绑它。

> 一句话: 真机走 `ep/`, **不需要 Jetson**; 在仓库里看到 Jetson 字样, 一律是 mechArm 历史留档。

## 目录

```
ep/
├── config_ep.json        # 全部几何/参数(A/B/HOME 里程计位姿、臂坐标、夹爪 power)——现场标定回填
├── core/selfcheck.py     # 离线自检(纯 stdlib, 不连机器也能跑): RESULT: ALL_OK
├── drive/rm.py           # RoboMaster SDK 薄封装(延迟导入、属性探测、goto 里程计补正)
├── drive/env_check.py    # 真机首跑自检: 连机 + 探测模块/属性名 + (可选 --jog 标臂 / --grip)
├── drive/ep_cycle.py     # 主循环(N 周期取放状态机), 日志/结果同 c4_2 口径
├── drive/ep_log.py       # 日志: run.log / events.jsonl / trajectory.csv / result.json
└── 标定说明.md            # 持机人必读: 三组底盘坐标 + 臂坐标 + 夹爪的标定方法
```

## 运行顺序(持机人)

```bash
# 0) 离线自检(任何机器, 不连 EP)
python3 ep/core/selfcheck.py --config ep/config_ep.json          # 期望 RESULT: ALL_OK

# 1) 装 SDK + 连机 + 真机探测(EP 现场, 已装 robomaster SDK)
#    conn_type: config 默认 'ap'(连 EP 自带热点); 多机/信号差用 'sta'
python3 ep/drive/env_check.py --config ep/config_ep.json
#    先标臂坐标: python3 ep/drive/env_check.py --config ep/config_ep.json --jog
#    再标夹爪:   python3 ep/drive/env_check.py --config ep/config_ep.json --grip
#    三组底盘坐标标定见 标定说明.md

# 2) 首跑 1 周期盯全程(pause_each_phase=true 必开, 人工确认 HELD/PLACED)
bash ep/run_ep.sh ep/config_ep.json 1

# 3) C5 验收 5 周期(>=4 成功, 结果在 ep_logs/run_*/result.json)
bash ep/run_ep.sh ep/config_ep.json 5
```

## 每周期动作

```
[臂回 lift_high 高悬停] → [夹爪张开] → [底盘 goto A_pose]
→ [臂下探 grab_low 包住目标物] → [夹爪闭紧] → [臂抬回 lift_high 提走]
→ 人工确认 HELD → [底盘 goto B_pose]
→ [臂下放到位] → [夹爪张开释放] → [臂抬回退开]
→ 人工确认 PLACED → [底盘 goto HOME_pose] → 记周期结果
```

## 与 c4_2 的对照(口径不变)

| 项 | c4_2(mechArm 仿真/旧真机) | ep/(EP 真机) |
|---|---|---|
| 执行对象 | 6-DOF 臂 + JTC(ros2_control) | EP 底盘 + 2 轴臂 + 夹爪(RoboMaster SDK) |
| 平台差异 | 全在 config json | 全在 config_ep.json(标定回填) |
| 判据 | sim=物块位姿; 真机=人工 | 人工确认 HELD/PLACED(EP 无物块感知) |
| 结果文件 | c4_2_logs/run_*/result.json | ep_logs/run_*/result.json(**同 schema**) |
| reason | grasp_failed / dropped_or_missed_place / exec_error / aborted | 同左 |

## 已知风险(真机验收成败关键)

1. **底盘里程计漂移**: B 落点容差靠"每轮从 HOME 出发 + goto 里程计补正"压住, 但长距离多次
   往返仍可能累积; 若连续掉点, 加"到 B 前的对位微调"或贴 B 靶纸人工判圈。
2. **臂 y 方向符号/行程**: 臂垂直行程仅 0.15m, 目标物要放在**爪能下探够到的高度**。
   2026-09-14 的演示视频里 EP 是**在地面上**夹放**矿泉水瓶**成功的 —— 瓶子有高度、抓握面高,
   所以地面可行; 换回 5cm 方块放地面则大概率够不到(块顶只 5cm, 臂下探行程不够),
   那时要垫高/放桌面。**量与形状变了就重标 `grab_low`/`lift_high`, 别沿用旧值。**
   **y 增大 = 向上**已由 2026-09-13 的 5mm 实测坐实
   (指令 +5mm → 读回 74,57 → 75,62 且人眼确认向上; 证据见 `docs/真机侧证据_EP_20260913/`),
   所以 `lift_high` 的 y 比 `grab_low` 大。另: 位置读回有 ±1mm 抖动 + x 固定 +1mm 偏移,
   判到位按 ±2mm 容差, 别用等号(`rm.py` 的 `_settle_arm` 已按此实现)。
3. **SDK API 版本差异**: arm 属性名(`robotic_arm` vs `arm`)、move 返回值是否 Action, 各版不一;
   `env_check` 首跑会逐个探测并打印, 按它提示核对即可。**已被真机日志/源码核对纠正过的三条**
   (官方 0.1.1.68): ①位置**没有同步 getter**, `chassis.get_position()` / `arm.get_position()`
   都不存在, 只能 `sub_position` 订阅 —— 驱动已改为订阅 + 新鲜度闸门(`_PushCache`);
   ②臂坐标是无符号 `<II>` 编码, 负 mm 会以接近 2**32 出现, 驱动按 `signed_int32_mm` 还原;
   ③`conn_type` **不能用 config 里读出的字符串**直接传 —— `conn.request_connection` 三个分支
   全用 `is` 比对象身份, `json.load` 出来的是新对象、比不过 SDK 模块里的字面量, 三个分支都不
   命中会让 `proxy_addr` 未赋值、**连接第一步就崩**; 驱动现在用 `_sdk_conn_type()` 映射回
   SDK 常量(拿不到 `conn` 时退化 `sys.intern`)。三条都做成离线断言:
   `python3 ep/tools/test_sdk_api_offline.py`。
4. **夹爪 hold**: 运送途中要保持闭紧; 首跑 1 周期时盯一下"闭爪后开去 B 是否松手", 松就在
   config 加 grip 等待或换更高 close_power。

## 本包边界

- 未做: EP 视觉/红外感知自动判据、底盘自动避障、多机。需要再议。
- sim 模块(mechArm 仿真实验)不受影响, 见仓库根 README / c4_2/README。
