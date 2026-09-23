# 归档：EP 机械臂 5mm 单步测试工具 v2（组员甲）

> **归档日期** 2026-09-13（本次现场跑完之后补档；此前"脚本本体不在仓库"是本目录唯一遗留项）
> **谁写的** 组员甲（ParisAn）；不是本仓库作者写的，**归档不改内容**
> **来源包** `EP_Arm_5mm_Test_v2 (1).zip`
> sha256 `e521fa527f72ac75eef17fb3799d110aab8fee4d52f2cbc7b1d05da4646d5f51`
> **现场日志** `arm_20260913_222009_108465.zip`
> sha256 `bc0a2b85660b1a0230b2ecc08288c498ec73525d374355857d85a992e36d17a3`
> → 日志与结果已转写进 `docs/真机侧证据_EP_20260913/arm_20260913_222009_108465/`

## 这是什么

一个**只做一次 +5mm 相对移动**的诊断工具：连机 → 读姿态/电量 → 人工输入 `u` 确认现场安全
→ `robotic_arm.move(x=0, y=5)` → 等动作 → 再等 2 秒观察窗口 → 读新的稳定位置 → 人工回答
"是否亲眼看到臂向上并停住" → 落 `logs/arm_<时间戳>/{arm.log, sdk.log, result.json}` → 退出。
没有自动回落、自动重试、底盘/夹爪运动，也**不调用**私有的 `_abort()`。

它证明了什么、没证明什么，见 `docs/真机侧证据_EP_20260913/README.md`：**证明了 SDK 通路与
mm 级位移可用；不是标定、不是验收**（脚本自己最后一行也这么写）。

## 目录

```
arm_5mm_test_v2/
├─ arm_5mm_test.py        # 主程序（默认离线；--connect 才连机）
├─ check_ep.py            # 连接/遥测框架：load_sdk() + Telemetry + stable_reading()
├─ tests/test_arm_5mm.py  # 19 项离线行为测试（替身对象，不发真机指令）
├─ 04_arm_5mm_test.cmd    # Windows 启动器（挑 3.9–3.12 的解释器）
├─ 先读我.md / 验证记录.txt / THIRD_PARTY_NOTICES.md
└─ README_归档说明.md      # 本文件
```

**没有归档 `vendor/`**（2.6 MB）。理由：它与本仓库 `deploy/ep_sdk_offline/packages/robomaster`
**只差 4 行**（见下），整包入库等于复制一份 SDK。要跑就用本仓库那份拼一个 `vendor/`（下面有命令）。

## vendor 与本仓库离线包的关系（已逐文件比对）

- `vendor/robomaster/` == `deploy/ep_sdk_offline/packages/robomaster/`，**仅 `robot.py` 差 4 行**：
  甲那份把 camera 的**实例化**也一并去掉了（我们只注释了 `import`），所以 `ep.camera` 返回的是
  未注册状态而不是构造失败：

  | 行 | 本仓库离线包 | 甲的 vendor |
  |---|---|---|
  | 407 / 1247 | `_camera = camera.EPCamera(self)` | `# EP_Windows_Check: camera is excluded from this local bundle.` |
  | 420 / 1266 | `self._modules[_camera.__class__.__name__] = _camera` | `# EP_Windows_Check: no camera module is registered.` |

  → 这一步比我们的离线包更彻底（我们那份一旦有人访问 `.camera` 就会去构造 cv2 那一路）。
  值不值得并回 `deploy/ep_sdk_offline/` 另议（那要动已交付的真机包，不在本次范围）。
- 另有 `vendor/LICENSE.txt`、`vendor/netaddr-1.3.0-py3-none-any.whl`（与我们的同版本）、
  以及甲自己写的 `vendor/netifaces.py`：**与我们的 `netifaces_stub.py` 不同** —— 他那份在
  不支持的网络发现场景**直接抛错**，我们那份返回空。AP 直连用不到这两个函数。
- 结论：现场跑的是**官方 0.1.1.68 的纯 py 代码 + 我们的 camera 裁剪 + 甲再裁一刀**，
  既不是 pip 装的官方包，也不是你我某一边单独那份。**这一点回答了"60 条协议警告是不是
  我们裁剪造成的"——不是**，理由见证据 README。

## 怎么跑（本仓库内，已实测）

```bash
cd ep/tools/arm_5mm_test_v2
mkdir -p vendor
cp -R ../../../deploy/ep_sdk_offline/packages/robomaster vendor/
cp ../../../deploy/ep_sdk_offline/packages/netaddr-*.whl vendor/
cp ../../../deploy/ep_sdk_offline/packages/netifaces_stub.py vendor/netifaces.py

python3 arm_5mm_test.py --offline        # 期望 RESULT: OFFLINE_OK（只导入，不连机）
python3 -m unittest discover -s tests    # 期望 Ran 19 tests OK
```

已实测（2026-09-13，本机）：
- `python3.11 arm_5mm_test.py --offline` → `RESULT: OFFLINE_OK` ✔（上面这条 vendor 拼装命令）
- `python3 -m unittest discover -s tests` → `Ran 19 tests ... OK` ✔

注意 **`check_ep.load_sdk()` 自己带 Python 版本闸门（只收 3.9–3.12）**：本机默认的 3.14 会被它
拒绝并打印 `Use Python 3.9-3.12; your existing Python 3.11 is suitable.` —— 那不是坏，是它的设计；
用 `python3.11` 跑即可。

## 对主线代码的影响（为什么它值得留档）

1. 它的 `result.json`（100 条 5Hz 采样）是 `ep/drive/rm.py` 里 `_settle_arm` 的建模依据：
   "动作完成早于位置推送 ~0.5s"整条结论都出自这份数据。
2. 它的 `stable_reading()` 与我们的 `_settle_arm` **独立收敛到同一条规矩**：都要求
   "读数已偏离动作前的值" **且** "新样本"（甲的单测里那一条就叫
   *"缓存的一个样本不当成多个新样本"*）—— 说明这不是我们的臆断，是这类订阅式 API 的共性。
3. 它的 `check_ep.py` 里 `decode_arm_position()` 处理的无符号问题，正是我们 `rm.py`
   `_signed_mm()` 的依据（`<II` 编码，负 mm 会以接近 2\*\*32 出现）。
4. **它没有发现**、由它顺带暴露出来的是我们自己的两个 bug：`chassis.get_position()` /
   `arm.get_position()` 在官方 0.1.1.68 里**根本不存在**（它从头到尾用的是订阅）。已修，
   并加了离线断言 `ep/tools/test_sdk_api_offline.py`。
