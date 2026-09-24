# 实验三 提交材料（2026-09-24）

本目录是**交付物快照**，不是源码。源码仍在本仓库 `exp3/` 下，两边按下面的对应表查。

| 任务书条目 | 本目录里的文件 |
| --- | --- |
| §七.1 完整 ROS 2 工程 / Launch / 依赖 | 源码见 `exp3/ros2/exp3_sim/launch/exp3_sim.launch.py`、`exp3/docs/依赖说明.md`；实跑留证见 `仿真运行证据/` |
| §七.2 检测模型 / 类别 / 网格配置 | 源码见 `exp3/config/task.json`、`exp3/config/grid_cells.json` |
| §七.3 状态机配置 | `exp3/docs/state_machine.md` |
| §七.4 仿真与真机演示视频 | `仿真演示视频/exp3_sim.mp4`（**真机待现场**） |
| §七.5 分类结果 / 异常测试记录 / 任务日志 / 报告 | `仿真运行证据/`（三轮 result.json + records.jsonl）、`实验3_提交材料对照表_20260923.docx`、`个人实验报告_实验三_仿真_桌面物体自动分类整理.pdf` |

**关于个人实验报告的范围**：本人分工是**仿真线 + 任务控制核心**，故报告主体为阶段一仿真。
阶段二真机（RoboMaster EP）的执行侧由组内其他同学承担，报告只在开头作范围声明，不写其实现细节。

---

## 一、仿真三轮实跑（2026-09-24，云机 AutoDL / Gazebo + ROS 2 Humble）

| 轮次 | run 目录 | verdict | placed_ok | objects_seen | 说明 |
| --- | --- | --- | --- | --- | --- |
| 记分轮 | `run_20260924_112831` | **PASS** | 5 | 6 | 及格线 5，规划失败 0；c6 为唯一失败项 |
| 异常轮 | `run_20260924_114058` | **TRIAL** | 5 | 5 | `empty_cell:=c3`；records 里**无 c3 行** |
| 录屏轮 | `run_20260924_114931` | FAIL | 3 | 6 | 该轮并发录屏，供视频取证；**不是**记分凭据 |

### 必须分开陈述的口径

云机容器内无 GPU，Gazebo 走软件渲染。用 HSV `color_detector`（实测 479% CPU）时，
跑到第 3~4 轮会出现**画面停滞**：帧戳照常推进而画面内容一字不变，依赖图像的检测随之失效。

为此记分轮改用**真值投影**生成检测框（`exp3/ros2/exp3_sim/exp3_sim/truth_detector.py`，
实测 4.5% CPU）：物块世界坐标经**同一份已标定的相机参数**投影为像素，
再走完全相同的「像素 → 桌面 → 网格」映射。**相机标定与映射链路未作任何改动。**

因此：

- ✅ **能证明**：相机标定正确、投影与网格映射正确、规划—取放—落料—回航这条执行链可行（5 次放置成功、规划失败 0）。
- ❌ **不能声称**：HSV 检测算法在整轮任务中稳定有效。渲染健康的第 1 轮 HSV 实测 6/6 正确映射，这份证据单独保留。
- ℹ️ 换成真值检测器后，录屏轮抽 7 帧 md5 两两全不同 ⇒ 渲染**不再停帧**。这反向印证了停帧根因是 CPU 争抢，不是模型或场景问题。

### 遗留

- 叠第 2 层时物块互穿，LCP 迭代数冲到 122847 并把已放好的物块弹飞；
  第 2 层落差 `dz=0.036` 仅比杯高 `0.035` 多 1 mm。方向：加大落差或层间 xy 错开。
- c6（第 4 个 cup）是记分轮唯一失败项，`script_place_error=(0.052, 0.087, 0.017)`。
- **真机线全部数值仍为占位，须现场实测回填，本目录不含任何估计值。**（真机线非本人分工）

## 二、演示视频

`仿真演示视频/exp3_sim.mp4` — 393.7 s / 640×480 / 10 fps / **3937 帧、0 丢帧**。
画面左下角烧入仿真时间戳，可与 `records.jsonl` 每一行对齐。

画面里机械臂是白色方块：为避开软渲染卡死，机械臂的 mesh 被刻意换成 box 图元
（脚本 `实验3_仿真运行证据_20260923/make_box_visual_urdf.py`），这是取舍不是模型缺失。

## 三、复现

```bash
# 同步权威源码 + 重建（URDF 图元化 + colcon build）
cd 实验3_仿真运行证据_20260923 && bash resume.sh
# 记分轮（真值检测器）
ssh autodl-exp3 'cd /root/autodl-tmp/ws_mecharm && DET=truth bash runf2.sh'
# 异常轮
ssh autodl-exp3 'cd /root/autodl-tmp/ws_mecharm && DET=truth LAUNCH_EXTRA="empty_cell:=c3" bash runf2.sh'
```

离线用例（不需要 ROS / Gazebo）：

```bash
cd exp3_public/exp3 && <venv>/bin/python -m unittest discover -s tests -t .
```
