# -*- coding: utf-8 -*-
"""真机视觉入口：相机取帧 + 检测器 → TaskController 的 scan 缝隙。

缝隙契约（与 ROS 侧 scan_from_detections.py **完全一致**）：
    scan() -> [ {cls, conf, bbox:[x1,y1,x2,y2]} ]，bbox 是**像素** xyxy。
    上游 Detection2DArray 那边的归一化 [0,1] 在 ROS 侧就还原成像素了，所以本层与
    sort_core 之间只走像素 —— 两条线（仿真/真机）喂给 TaskController 的东西因此是同一种。

一条与 ROS 侧**故意不同**的约定：**取不到帧要抛，不要返回 None / []**。
ROS 侧 latest() 返回 None 表示"还没收到首帧"（甲那边有 wait_first_frame 兜），但 mid-run
的 None 到了 TaskController 里会走 `dets = self.scan() or []` → 当场判成"桌面已空、正常结束"
—— 一次掉帧就变成一次"跑完了"。真机这边让 CameraError 一路抛到 run_real，判成 exec_error
停线：宁可停，也不能把掉线当成功。
"""
import json
import os

from sort_core.geometry import cell_center, table_to_px


class CameraError(RuntimeError):
    pass


# ---------- 相机 ----------
class Cv2Camera:
    """USB 摄像头（与实验一同款）。cv2 延迟导入 —— 没有 cv2 的环境也能 import 本模块。"""

    def __init__(self, index=0, size=(640, 480), warmup=10, log=None):
        self.log = log
        self.size = (int(size[0]), int(size[1]))
        try:
            import cv2
        except ImportError as e:
            raise CameraError('没装 opencv —— pip3 install opencv-python（或先用 '
                              '--camera none 演练）: %s' % e)
        self._cv2 = cv2
        self.cap = cv2.VideoCapture(int(index))
        if not self.cap.isOpened():
            raise CameraError('打不开摄像头 index=%d —— 被别的程序占用? 线松了? 换个 index 试?'
                              % index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        # 前几帧是黑的/花的（曝光/白平衡还在收敛），丢掉 —— 不丢的话第一次 scan 必空
        for _ in range(int(warmup)):
            self.cap.read()
        if log:
            log.text('相机就绪: index=%d 请求 %dx%d（实际 %dx%d）'
                     % (index, self.size[0], self.size[1],
                        int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    def grab(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise CameraError('读帧失败 —— 摄像头被抢占/掉线了（这一轮作废，不当作桌面已空）')
        h, w = frame.shape[:2]
        if (w, h) != self.size:
            raise CameraError('帧尺寸 %dx%d ≠ config 里的 %dx%d —— 标定是按后者做的，'
                              '尺寸不符会让定位整体错位' % (w, h, self.size[0], self.size[1]))
        return frame

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


class ImageCamera:
    """固定图片（回放/单测）：每次 grab 都返回同一张。"""

    def __init__(self, path, log=None):
        try:
            import cv2
        except ImportError as e:
            raise CameraError('没装 opencv，读不了图片: %s' % e)
        self.frame = cv2.imread(path)
        if self.frame is None:
            raise CameraError('读不了图片: %s' % path)
        if log:
            log.text('回放图: %s %s' % (path, self.frame.shape))

    def grab(self):
        return self.frame

    def close(self):
        pass


class NullCamera:
    """不接相机（--camera none）：只在配 mock 检测器时用 —— 那条路不需要像素。"""

    def __init__(self, size=(640, 480), log=None):
        self.size = size
        if log:
            log.text('相机: none（不取帧；只有 mock 检测器能配它）')

    def grab(self):
        return None

    def close(self):
        pass


def open_camera(spec, size=(640, 480), log=None):
    """spec: 'none' | 'cv2:0' | 'image:/path/to.jpg'（默认 cv2:0）。"""
    spec = (spec or 'cv2:0').strip()
    if spec == 'none':
        return NullCamera(size, log)
    kind, _, arg = spec.partition(':')
    if kind == 'cv2':
        return Cv2Camera(int(arg or 0), size, log=log)
    if kind == 'image':
        return ImageCamera(arg, log)
    raise SystemExit('未知 --camera: %r（none | cv2:0 | image:/path）' % spec)


# ---------- 检测器 ----------
class MockDetector:
    """合成检测：把"桌上摆了哪些物体"按 camera.calib **反投影**成 bbox，当作识别结果。

    为什么需要它 —— 乙的真检测器还没到，但真机链路（取帧→检测→定位→任务控制→EP）现在就
    要能跑通、能演练、能离线回归。它和将被换掉的那一环是**同一个接口**：换真检测器时只改
    run_real 的 --detector，别的一行不动。

    ★ 它证明的是"接口与链路对"，**不证明分类精度** —— 验收必须用真模型（docx §五）。

    objects    : [{"cell": "c1", "cls": "cup"}, ...]，被 pick 成功后自动消失
    extra_dets : 每轮额外注入的检测（造 unrecognized / out_of_grid 用，见 scene 文件）
    """

    def __init__(self, objects, cells, calib, extra_dets=None, bbox_hw=6, log=None):
        self.cells = {c['id']: c for c in cells}
        self.calib = calib
        self.objects = [(o['cell'], o['cls']) for o in (objects or [])]
        self.extra = list(extra_dets or [])
        self.bbox_hw = int(bbox_hw)
        self.log = log

    def mark_picked(self, cell_id):
        """把该格的物体移出桌面（真机由机械臂真的搬走了，mock 这边同步状态）。"""
        before = len(self.objects)
        self.objects = [(c, k) for (c, k) in self.objects if c != cell_id]
        return before != len(self.objects)

    def detect(self, frame=None):
        out = []
        for cell_id, cls in self.objects:
            cell = self.cells.get(cell_id)
            if cell is None:
                raise CameraError('scene 里的 cell_id=%r 不在 grid_cells.json 里' % cell_id)
            xm, ym = cell_center(cell)
            px = table_to_px(xm, ym, self.calib)
            if px is None:
                raise CameraError('calib 无法把 %s 反投影(标定退化?)' % cell_id)
            hw = self.bbox_hw
            out.append({'cls': cls, 'conf': 0.93,
                        'bbox': [px[0] - hw, px[1] - hw, px[0] + hw, px[1] + hw]})
        return out + list(self.extra)


class YoloDetector:
    """乙的检测模型（Ultralytics YOLO 口径）。

    model 是 .pt 路径；classes 是模型输出名 → 本工程类别名的对照（不填则原样透传）。
    只做**一次**前向，不加跟踪/平滑 —— 平滑要做也应该做在任务层的"多帧一致"上。
    """

    def __init__(self, model_path, conf_min=0.25, classes=None, log=None):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise CameraError('没装 ultralytics —— pip3 install ultralytics（或先用 '
                              '--detector mock 演练）: %s' % e)
        self.model = YOLO(model_path)
        self.names = self.model.names if isinstance(self.model.names, dict) \
            else dict(enumerate(self.model.names))
        self.alias = dict(classes or {})
        self.conf_min = float(conf_min)
        if log:
            log.text('检测模型: %s  类别表=%s' % (model_path, self.names))

    def detect(self, frame):
        if frame is None:
            raise CameraError('YoloDetector 需要真实帧，但相机是 none')
        res = self.model.predict(frame, verbose=False, conf=self.conf_min)[0]
        out = []
        for b in res.boxes:
            name = self.names.get(int(b.cls), str(int(b.cls)))
            out.append({'cls': self.alias.get(name, name),
                        'conf': float(b.conf),
                        'bbox': [float(v) for v in b.xyxy[0]]})
        return out


def load_detector_file(path, log=None):
    """加载外部检测器文件（交给乙的接口：文件里定义 detect(frame) -> dets 即可）。

    这样乙换模型/换后处理都不用动本工程 —— 给他一个文件名就说清了交付物。
    """
    import importlib.util
    if not os.path.isfile(path):
        raise SystemExit('--detector-file 不存在: %s' % path)
    spec = importlib.util.spec_from_file_location('exp3_detector', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, 'detect'):
        raise SystemExit('%s 里没有 detect(frame) —— 检测器文件的唯一要求就是它' % path)
    if log:
        log.text('检测器: 外部文件 %s' % path)
    return mod


def load_scene(path):
    """读 mock 场景文件：{"objects": [{"cell","cls"}...], "extra_dets": [...]}。"""
    with open(path, encoding='utf-8') as f:
        sc = json.load(f)
    if not isinstance(sc, dict):
        raise SystemExit('scene 文件应是 JSON 对象: {"objects": [...], "extra_dets": [...]}')
    return sc


def make_scan(camera, detector, log=None):
    """把 (相机, 检测器) 合成 TaskController 要的 scan。取帧失败会抛，不返回空表。"""
    def scan():
        frame = camera.grab()
        dets = detector.detect(frame)
        dets = list(dets or [])
        if log:
            log.text('识别: %d 个检测 %s' % (
                len(dets), [(d.get('cls'), round(float(d.get('conf', 0)), 2)) for d in dets]))
        return dets
    return scan
