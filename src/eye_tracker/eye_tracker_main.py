"""眼球追踪模块主封装。

职责分层：
  1) 数据模型: CameraConfig, AppConfig
  2) 配置持久化: ConfigPersistence
  3) 调试 UI: CropDebugWindow, ControlPanel, DebugMainWindow
  4) 批处理管线: Normalizer, GazeConsumer
  5) 核心编排: EyeTrackingModule
  6) 独立入口: __main__

对外接口：
  module = EyeTrackingModule()
  module.start()
  state = module.get_normalized_eye_state()  # {"left": {"eye_x":..., "eye_y":...}, "right":...}
  module.stop()

  调试画面控制（隐藏/显示子进程 OpenCV 窗口）：
    module.enter_headless_mode()   # 隐藏 OpenCV 调试窗口
    module.exit_headless_mode()    # 显示 OpenCV 调试窗口
    module.headless_runtime        # bool，查询当前状态

  ⚠️ 归一化依赖 extreme_vectors.yaml 标定文件（通过控制面板录制极值向量）。
     无此文件时 get_normalized_eye_state() 返回的 eye_x / eye_y 均为 None，
     下游调用者须自行处理降级（保持上一帧有效值 / 使用 get_raw_gaze_vector() / 输出 0.0）。
"""

__all__ = [
    "EyeTrackingModule",
    "detect_cameras",
    "AppConfig",
    "CameraConfig",
]

import tkinter as tk
import cv2
from tkinter import ttk
from PIL import Image, ImageTk
import yaml
import multiprocessing
import threading
import queue
import os
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple

# 内部依赖
from gaze_vector_tracker import GazeVectorTracker

logger = logging.getLogger(__name__)


# ============================================================
# 0. 工具函数
# ============================================================

def _setup_camera(index: int) -> Optional[cv2.VideoCapture]:
    """打开指定索引的相机并设置基本参数。"""
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        cap.release()
        return None
    return cap


# ============================================================
# 1. 数据模型
# ============================================================

@dataclass
class CameraConfig:
    """单个相机的配置。crop 格式: [x1, y1, x2, y2]。"""
    index: int = 0
    crop: List[int] = field(default_factory=lambda: [0, 0, 640, 480])
    flip: bool = False


@dataclass
class AppConfig:
    """双眼相机的应用配置。"""
    left: CameraConfig = field(default_factory=CameraConfig)
    right: CameraConfig = field(default_factory=lambda: CameraConfig(index=0))


# ============================================================
# 2. 配置持久化（与数据模型解耦）
# ============================================================

class ConfigPersistence:
    """负责 AppConfig 的 YAML 文件读写。"""

    def __init__(self, filepath: str = "config.yaml"):
        self._filepath = filepath

    def load(self) -> AppConfig:
        """从文件加载配置，文件不存在时返回默认配置并保存。"""
        try:
            with open(self._filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            logger.info("无配置文件，使用默认配置")
            config = AppConfig()
            self.save(config)
            return config
        except Exception as e:
            logger.error(f"读取配置文件失败: {e}")
            return AppConfig()

        if data is None:
            return AppConfig()

        try:
            left_data = data.get('left', {})
            right_data = data.get('right', {})
            config = AppConfig(
                left=CameraConfig(
                    index=left_data.get('camera_index', 0),
                    crop=left_data.get('crop', [0, 0, 640, 480]),
                    flip=left_data.get('flip', False),
                ),
                right=CameraConfig(
                    index=right_data.get('camera_index', 0),
                    crop=right_data.get('crop', [0, 0, 640, 480]),
                    flip=right_data.get('flip', False),
                ),
            )
        except Exception as e:
            logger.warning(f"配置数据格式有误，部分使用默认值: {e}")
            return AppConfig()

        return config

    def save(self, config: AppConfig) -> None:
        """保存配置到文件。"""
        data = {
            'left': {
                'camera_index': config.left.index,
                'crop': config.left.crop,
                'flip': config.left.flip,
            },
            'right': {
                'camera_index': config.right.index,
                'crop': config.right.crop,
                'flip': config.right.flip,
            },
        }
        try:
            with open(self._filepath, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=None)
            logger.info("配置已保存")
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")


# ============================================================
# 3. 调试预览窗口
# ============================================================

class CropDebugWindow:
    """Toplevel 调试窗口：显示相机画面，支持垂直翻转与固定比例剪裁。"""

    CROP_TARGET_RATIO: float = 4.0 / 3.0

    def __init__(
        self,
        master: tk.Tk,
        cam_index: int,
        side: str,
        cam_config: CameraConfig,
        on_config_changed: "callable" = None,
    ):
        self._master = master
        self._side = side
        self._cam_config = cam_config
        self._on_config_changed = on_config_changed

        self._cap = _setup_camera(cam_index)
        if self._cap is None:
            raise RuntimeError(f"无法打开相机 {cam_index}")

        self._window = tk.Toplevel(master)
        self._window.title(
            f"调试 - {'左眼' if side == 'left' else '右眼'}相机 (索引 {cam_index})"
        )

        self._crop_mode: bool = False
        self._crop_rect: Optional[Tuple[int, int, int, int]] = None
        self._drawing: bool = False
        self._start_x: int = 0
        self._start_y: int = 0
        self._running: bool = True

        self._build_ui()
        self._bind_mouse_events()
        self._window.protocol("WM_DELETE_WINDOW", self._on_close)
        self._show_frame_loop()

    def _build_ui(self) -> None:
        btn_frame = tk.Frame(self._window)
        btn_frame.pack(pady=(5, 0))

        self._flip_var = tk.BooleanVar(value=self._cam_config.flip)
        tk.Checkbutton(btn_frame, text="垂直翻转", variable=self._flip_var).pack(
            side=tk.LEFT, padx=5
        )

        self._btn_crop = tk.Button(btn_frame, text="剪裁", command=self._toggle_crop)
        self._btn_crop.pack(side=tk.LEFT, padx=5)

        tk.Button(btn_frame, text="保存剪裁配置", command=self._save_crop_config).pack(
            side=tk.LEFT, padx=5
        )

        self._video_label = tk.Label(self._window)
        self._video_label.pack()

    def _bind_mouse_events(self) -> None:
        self._video_label.bind("<ButtonPress-1>", self._on_press)
        self._video_label.bind("<B1-Motion>", self._on_drag)
        self._video_label.bind("<ButtonRelease-1>", self._on_release)

    def _toggle_crop(self) -> None:
        self._crop_mode = not self._crop_mode
        if self._crop_mode:
            self._crop_rect = None
            self._btn_crop.config(relief=tk.SUNKEN)
        else:
            self._btn_crop.config(relief=tk.RAISED)

    def _save_crop_config(self) -> None:
        if self._crop_rect is not None:
            x1, y1, x2, y2 = self._crop_rect
            logger.info(f"剪裁区域: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
            self._cam_config.crop = [x1, y1, x2, y2]
            self._cam_config.flip = self._flip_var.get()
            if self._on_config_changed:
                self._on_config_changed()
            logger.info(f"{self._side}眼相机配置已保存")
        else:
            logger.warning("未选择剪裁区域")

    def _on_press(self, event: tk.Event) -> None:
        if not self._crop_mode:
            return
        self._drawing = True
        self._start_x = event.x
        self._start_y = event.y
        self._crop_rect = None

    def _on_drag(self, event: tk.Event) -> None:
        if not self._crop_mode or not self._drawing:
            return
        self._crop_rect = self._constrain_rect(
            self._start_x, self._start_y, event.x, event.y
        )

    def _on_release(self, event: tk.Event) -> None:
        if not self._crop_mode:
            return
        self._drawing = False
        self._crop_rect = self._constrain_rect(
            self._start_x, self._start_y, event.x, event.y
        )

    @classmethod
    def _constrain_rect(
        cls, x1: int, y1: int, x2: int, y2: int
    ) -> Optional[Tuple[int, int, int, int]]:
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        if w == 0 and h == 0:
            return None
        ratio = cls.CROP_TARGET_RATIO
        if w / max(h, 1) > ratio:
            new_w = w
            new_h = max(1, int(w / ratio))
        else:
            new_h = h
            new_w = max(1, int(h * ratio))
        end_x = x1 + new_w if x2 >= x1 else x1 - new_w
        end_y = y1 + new_h if y2 >= y1 else y1 - new_h
        result_x1, result_x2 = min(x1, end_x), max(x1, end_x)
        result_y1, result_y2 = min(y1, end_y), max(y1, end_y)
        return [result_x1, result_y1, result_x2, result_y2]

    def _show_frame_loop(self) -> None:
        if not self._running:
            return
        ret, frame = self._cap.read()
        if self._flip_var.get():
            frame = cv2.flip(frame, 0)
        if ret:
            if self._crop_mode and self._crop_rect is not None:
                x1, y1, x2, y2 = self._crop_rect
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self._video_label.imgtk = imgtk
            self._video_label.configure(image=imgtk)
            self._video_label.after(30, self._show_frame_loop)
        else:
            logger.warning("无法读取帧")
            self._video_label.after(100, self._show_frame_loop)

    def _on_close(self) -> None:
        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._window.destroy()

    def close(self) -> None:
        self._on_close()

    def get_window(self) -> tk.Toplevel:
        return self._window


# ============================================================
# 4. 控制面板
# ============================================================

class ControlPanel:
    """Toplevel 控制面板：锁定/解锁眼球参数 + 保存极限注视向量。"""

    def __init__(
        self,
        master: tk.Tk,
        cmd_queue_left: Optional[multiprocessing.Queue],
        cmd_queue_right: Optional[multiprocessing.Queue],
        gaze_reader: "callable",
        normalizer: "Normalizer",
    ):
        self._master = master
        self._cmd_queue_left = cmd_queue_left
        self._cmd_queue_right = cmd_queue_right
        self._gaze_reader = gaze_reader
        self._normalizer = normalizer
        self._locks: Dict[str, bool] = {
            "left_radius": False, "left_center": False,
            "right_radius": False, "right_center": False,
        }
        self._window: Optional[tk.Toplevel] = None
        self._open()

    def _open(self) -> None:
        self._window = tk.Toplevel(self._master)
        self._window.title("控制面板")
        self._window.resizable(False, False)

        tk.Label(self._window, text="眼球追踪控制面板", font=("", 12, "bold")).pack(
            pady=(10, 5)
        )

        tk.Label(self._window, text="左眼").pack(anchor="w", padx=20, pady=(5, 0))
        self._make_button(self._cmd_queue_left, "左眼半径", "left_radius").pack(pady=2)
        self._make_button(self._cmd_queue_left, "左眼中心", "left_center").pack(pady=2)

        tk.Label(self._window, text="右眼").pack(anchor="w", padx=20, pady=(10, 0))
        self._make_button(self._cmd_queue_right, "右眼半径", "right_radius").pack(pady=2)
        self._make_button(self._cmd_queue_right, "右眼中心", "right_center").pack(pady=2)

        tk.Label(self._window, text="极限注视向量", font=("", 10, "bold")).pack(pady=(10, 5))
        directions = [("仰视", "up"), ("俯视", "down"), ("内眼角", "inner"), ("外眼角", "outer")]
        for eye_side in ("left", "right"):
            eye_label = "左眼" if eye_side == "left" else "右眼"
            tk.Label(self._window, text=eye_label).pack(anchor="w", padx=20, pady=(5, 0))
            for label, dir_key in directions:
                tk.Button(
                    self._window,
                    text=f"保存{eye_label}{label}向量",
                    command=lambda s=eye_side, d=dir_key: self._save_extreme_vector(s, d),
                    width=22,
                ).pack(pady=1)

        def _on_close():
            self._window.destroy()
            self._window = None
        self._window.protocol("WM_DELETE_WINDOW", _on_close)

    def _make_button(self, queue, label_prefix, lock_key):
        btn_text = tk.StringVar()

        def update_text(*args):
            btn_text.set(f"解锁{label_prefix}" if self._locks[lock_key] else f"锁定{label_prefix}")

        def toggle():
            if queue is None:
                logger.warning(f"[{label_prefix}] 追踪尚未启动")
                return
            locked = self._locks[lock_key]
            if locked:
                queue.put_nowait("unlock_radius" if "半径" in label_prefix else "unlock_center")
            else:
                queue.put_nowait("lock_radius" if "半径" in label_prefix else "lock_center")
            self._locks[lock_key] = not locked
            update_text()

        update_text()
        btn = tk.Button(self._window, textvariable=btn_text, command=toggle, width=18)
        if queue is None:
            btn.config(state=tk.DISABLED)
        return btn

    def _save_extreme_vector(self, side: str, direction: str) -> None:
        vector = self._gaze_reader(side)
        if vector is None:
            logger.warning(f"未能获取 {side} 眼当前向量，跳过保存")
            return
        self._normalizer.set_extreme(side, direction, vector)
        logger.info(f"已保存 {side}_{direction}: {vector}")

    def destroy(self) -> None:
        if self._window is not None:
            self._window.destroy()
            self._window = None

    def is_open(self) -> bool:
        return self._window is not None


# ============================================================
# 5. 归一化器
# ============================================================

class Normalizer:
    def __init__(self, extreme_file: str = "extreme_vectors.yaml"):
        self._extreme_file = os.path.join(os.path.dirname(__file__), extreme_file)
        self._extremes: Dict[str, List[float]] = {}
        self._load_extremes()

    def _load_extremes(self) -> None:
        try:
            with open(self._extreme_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self._extremes = {k: list(v) for k, v in data.items()}
        except FileNotFoundError:
            self._extremes = {}
            logger.info("极值向量文件不存在，归一化将返回 None")

    def set_extreme(self, side: str, direction: str, vector: List[float]) -> None:
        key = f"{side}_{direction}"
        self._extremes[key] = vector
        try:
            with open(self._extreme_file, 'w', encoding='utf-8') as f:
                yaml.dump(self._extremes, f, allow_unicode=True)
        except Exception as e:
            logger.error(f"保存极值向量失败: {e}")

    def normalize(self, side: str, gaze_rotated: List[float]) -> Dict[str, Optional[float]]:
        result = {"eye_x": None, "eye_y": None}
        if not gaze_rotated or len(gaze_rotated) != 3:
            return result
        inner_key, outer_key = f"{side}_inner", f"{side}_outer"
        up_key, down_key = f"{side}_up", f"{side}_down"
        if not all(k in self._extremes for k in (inner_key, outer_key, up_key, down_key)):
            return result
        eye_x, eye_y = self._orthogonal_project(
            gaze_rotated,
            self._extremes[inner_key], self._extremes[outer_key],
            self._extremes[up_key], self._extremes[down_key],
        )
        result["eye_x"] = eye_x
        result["eye_y"] = eye_y
        return result

    @staticmethod
    def _orthogonal_project(current, inner, outer, up, down):
        import math
        cx = inner[0] + outer[0] + up[0] + down[0]
        cy = inner[1] + outer[1] + up[1] + down[1]
        cz = inner[2] + outer[2] + up[2] + down[2]
        c_len = math.sqrt(cx * cx + cy * cy + cz * cz)
        if c_len < 1e-12:
            return None, None
        cx /= c_len; cy /= c_len; cz /= c_len

        raw_ex = [inner[0] - outer[0], inner[1] - outer[1], inner[2] - outer[2]]
        d_ex = raw_ex[0] * cx + raw_ex[1] * cy + raw_ex[2] * cz
        ex = [raw_ex[0] - d_ex * cx, raw_ex[1] - d_ex * cy, raw_ex[2] - d_ex * cz]
        ex_len = math.sqrt(ex[0]**2 + ex[1]**2 + ex[2]**2)
        if ex_len < 1e-12:
            return None, None
        ex = [ex[0] / ex_len, ex[1] / ex_len, ex[2] / ex_len]

        ey = [cy * ex[2] - cz * ex[1], cz * ex[0] - cx * ex[2], cx * ex[1] - cy * ex[0]]
        ey_len = math.sqrt(ey[0]**2 + ey[1]**2 + ey[2]**2)
        if ey_len < 1e-12:
            return None, None
        ey = [ey[0] / ey_len, ey[1] / ey_len, ey[2] / ey_len]

        def dot(a, b):
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        outer_x, inner_x = dot(outer, ex), dot(inner, ex)
        down_y, up_y = dot(down, ey), dot(up, ey)
        cur_x, cur_y = dot(current, ex), dot(current, ey)

        def clamp_map(val, lo, hi):
            if hi - lo < 1e-12:
                return 0.0
            val = max(lo, min(val, hi))
            return 2.0 * (val - lo) / (hi - lo) - 1.0

        return clamp_map(cur_x, outer_x, inner_x), clamp_map(cur_y, down_y, up_y)


# ============================================================
# 6. 注视向量消费者线程
# ============================================================

class GazeConsumer:
    def __init__(self, normalizer: Normalizer):
        self._normalizer = normalizer
        self._queue: Optional[multiprocessing.Queue] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._raw_gaze: Dict = {"left": None, "right": None}
        self._normalized: Dict = {
            "left": {"eye_x": None, "eye_y": None, "confidence": None},
            "right": {"eye_x": None, "eye_y": None, "confidence": None},
        }
        self._last_update: float = 0.0

    def start(self, result_queue: multiprocessing.Queue) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._queue = result_queue
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("注视向量消费者线程已启动")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                logger.warning("消费者线程未能及时终止")
            self._thread = None
        self._queue = None
        with self._lock:
            self._raw_gaze = {"left": None, "right": None}
            self._normalized = {
                "left": {"eye_x": None, "eye_y": None, "confidence": None},
                "right": {"eye_x": None, "eye_y": None, "confidence": None},
            }

    def _loop(self) -> None:
        import numpy as np
        while not self._stop_event.is_set():
            try:
                data = self._queue.get(timeout=0.001)
                self._process(data)
            except queue.Empty:
                pass
            except Exception:
                pass

    def _process(self, data: dict) -> None:
        side = data.get("side")
        gaze = data.get("gaze_rotated")
        if side is None or gaze is None:
            return
        with self._lock:
            self._raw_gaze[side] = gaze
            self._normalized[side] = self._normalizer.normalize(side, gaze)
            self._normalized[side]["confidence"] = data.get("confidence", None)
            self._last_update = time.time()

    def get_raw_gaze(self, side: str) -> Optional[List[float]]:
        with self._lock:
            return self._raw_gaze.get(side)

    def get_normalized_state(self) -> dict:
        with self._lock:
            return {
                "left": dict(self._normalized["left"]),
                "right": dict(self._normalized["right"]),
                "timestamp": self._last_update,
            }


# ============================================================
# 7. 核心编排：EyeTrackingModule
# ============================================================

class EyeTrackingModule:
    """眼球追踪核心模块 —— 对外唯一入口。

    使用方式：
        # 头模式（作为大软件的子模块）
        mod = EyeTrackingModule(headless=True)
        mod.start()
        state = mod.get_normalized_eye_state()
        mod.stop()

        # 调试模式（独立运行 GUI）
        mod = EyeTrackingModule(headless=False, master=root)
        mod.start()
        mod.open_control_panel()
        ...
        mod.stop()

    调试画面控制（仅隐藏/显示子进程 OpenCV 窗口，tkinter 窗口不受影响）：
        mod.enter_headless_mode()   # 隐藏 OpenCV 调试窗口
        mod.exit_headless_mode()    # 显示 OpenCV 调试窗口
        mod.headless_runtime        # 查询当前状态

    Parameters
    ----------
    headless : bool
        True 时禁止创建任何 GUI 窗口。
    master : tk.Tk | None
        tkinter 父窗口实例。headless=False 时若为 None 会自动创建一个隐藏窗口。
    config_path : str
        相机配置文件的路径。
    extreme_file : str
        极值向量文件的路径（文件名部分）。
    """

    def __init__(
        self,
        headless: bool = False,
        master: Optional[tk.Tk] = None,
        config_path: str = "config.yaml",
        extreme_file: str = "extreme_vectors.yaml",
    ):
        self._headless = headless

        if headless:
            self._master: Optional[tk.Tk] = None
        elif master is not None:
            self._master = master
        else:
            self._master = tk.Tk()
            self._master.withdraw()

        self._persistence = ConfigPersistence(config_path)
        self.config: AppConfig = self._persistence.load()
        self._normalizer = Normalizer(extreme_file)
        self._consumer = GazeConsumer(self._normalizer)

        self._process_left: Optional[multiprocessing.Process] = None
        self._process_right: Optional[multiprocessing.Process] = None
        self._cmd_queue_left: Optional[multiprocessing.Queue] = None
        self._cmd_queue_right: Optional[multiprocessing.Queue] = None
        self._result_queue: Optional[multiprocessing.Queue] = None

        self._crop_window_left: Optional[CropDebugWindow] = None
        self._crop_window_right: Optional[CropDebugWindow] = None
        self._control_panel: Optional[ControlPanel] = None

        # 运行时无头状态（仅控制子进程 OpenCV 窗口）
        self._headless_runtime: bool = False

    # ----------------------------------------------------------
    # 公共 API
    # ----------------------------------------------------------

    @property
    def headless_runtime(self) -> bool:
        return self._headless_runtime

    def enter_headless_mode(self) -> None:
        """隐藏子进程的 OpenCV 调试窗口。tkinter 窗口不受影响。"""
        if self._headless_runtime:
            return
        for q in [self._cmd_queue_left, self._cmd_queue_right]:
            if q is not None:
                try:
                    q.put_nowait("headless_on")
                except Exception:
                    pass
        self._headless_runtime = True
        logger.info("已隐藏 OpenCV 调试窗口")

    def exit_headless_mode(self) -> None:
        """恢复子进程的 OpenCV 调试窗口。"""
        if not self._headless_runtime:
            return
        for q in [self._cmd_queue_left, self._cmd_queue_right]:
            if q is not None:
                try:
                    q.put_nowait("headless_off")
                except Exception:
                    pass
        self._headless_runtime = False
        logger.info("已恢复 OpenCV 调试窗口")

    def start(self) -> None:
        self._stop_internal()

        self._cmd_queue_left = multiprocessing.Queue()
        self._cmd_queue_right = multiprocessing.Queue()
        self._result_queue = multiprocessing.Queue()

        self._process_left = multiprocessing.Process(
            target=_run_tracker_in_process,
            args=(self.config.left.index, self.config.left.flip, self.config.left.crop,
                  "left", self._cmd_queue_left, self._result_queue, self._headless),
            daemon=True,
        )
        self._process_left.start()

        self._process_right = multiprocessing.Process(
            target=_run_tracker_in_process,
            args=(self.config.right.index, self.config.right.flip, self.config.right.crop,
                  "right", self._cmd_queue_right, self._result_queue, self._headless),
            daemon=True,
        )
        self._process_right.start()

        time.sleep(0.5)
        left_alive = self._process_left.is_alive()
        right_alive = self._process_right.is_alive()

        if not left_alive or not right_alive:
            self._stop_internal()
            failed_side = []
            if not left_alive:
                failed_side.append("左眼")
            if not right_alive:
                failed_side.append("右眼")
            raise RuntimeError(f"{'、'.join(failed_side)}追踪子进程启动失败（相机不可用或索引错误）")

        self._consumer.start(self._result_queue)
        logger.info("双眼眼球追踪已启动")

    def stop(self) -> None:
        self._stop_internal()

    def _stop_internal(self) -> None:
        if self._control_panel is not None:
            self._control_panel.destroy()
            self._control_panel = None

        for proc in (self._process_left, self._process_right):
            if proc is not None and proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
                if proc.is_alive():
                    logger.warning(f"子进程 {proc.pid} 未及时终止")
        self._process_left = None
        self._process_right = None
        self._cmd_queue_left = None
        self._cmd_queue_right = None
        self._consumer.stop()
        self._result_queue = None
        logger.info("眼球追踪已停止")

    def get_normalized_eye_state(self) -> dict:
        return self._consumer.get_normalized_state()

    def get_raw_gaze_vector(self, side: str) -> Optional[List[float]]:
        return self._consumer.get_raw_gaze(side)

    def is_running(self) -> bool:
        return (
            self._process_left is not None and self._process_left.is_alive()
            and self._process_right is not None and self._process_right.is_alive()
        )

    # ----------------------------------------------------------
    # 调试 UI 方法
    # ----------------------------------------------------------

    def open_crop_window(self, side: str) -> None:
        if self._headless or self._master is None:
            logger.warning("headless 模式下无法打开调试窗口")
            return

        if side == "left":
            if self._crop_window_left is not None:
                self._crop_window_left.get_window().deiconify()
                return
            self._crop_window_left = CropDebugWindow(
                self._master, self.config.left.index, "left", self.config.left,
                on_config_changed=self.save_config,
            )
            self._crop_window_left.get_window().protocol(
                "WM_DELETE_WINDOW", self._on_left_crop_close
            )
        else:
            if self._crop_window_right is not None:
                self._crop_window_right.get_window().deiconify()
                return
            self._crop_window_right = CropDebugWindow(
                self._master, self.config.right.index, "right", self.config.right,
                on_config_changed=self.save_config,
            )
            self._crop_window_right.get_window().protocol(
                "WM_DELETE_WINDOW", self._on_right_crop_close
            )

    def _on_left_crop_close(self) -> None:
        if self._crop_window_left is not None:
            self._crop_window_left.close()
        self._crop_window_left = None

    def _on_right_crop_close(self) -> None:
        if self._crop_window_right is not None:
            self._crop_window_right.close()
        self._crop_window_right = None

    def open_control_panel(self) -> None:
        if self._headless or self._master is None:
            logger.warning("headless 模式下无法打开控制面板")
            return
        if self._control_panel is not None and self._control_panel.is_open():
            self._control_panel.destroy()
        self._control_panel = ControlPanel(
            self._master, self._cmd_queue_left, self._cmd_queue_right,
            self.get_raw_gaze_vector, self._normalizer,
        )

    def save_config(self) -> None:
        self._persistence.save(self.config)

    def set_left_camera(self, index: int) -> None:
        self.config.left.index = index

    def set_right_camera(self, index: int) -> None:
        self.config.right.index = index

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


# ============================================================
# 8. 子进程入口
# ============================================================

def _run_tracker_in_process(
    cam_index: int, flip: bool, crop: List[int], side: str,
    command_queue: multiprocessing.Queue,
    result_queue: multiprocessing.Queue, headless: bool,
) -> None:
    tracker = GazeVectorTracker(cam_index=cam_index, flip=flip, crop=crop, side=side)
    tracker.start_tracking(
        command_queue=command_queue, result_queue=result_queue, headless=headless,
    )


# ============================================================
# 9. 工具函数
# ============================================================

def detect_cameras(max_cams: int = 6) -> List[int]:
    available: List[int] = []
    for i in range(max_cams):
        cap = _setup_camera(i)
        if cap is not None:
            available.append(i)
            cap.release()
    return available


# ============================================================
# 10. 调试入口
# ============================================================

if __name__ == "__main__":
    multiprocessing.set_start_method("spawn", force=True)
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    cameras = detect_cameras()
    if not cameras:
        print("Error: 没有可用相机")
        exit(1)
    print(f"可用相机: {cameras}")

    root = tk.Tk()
    root.title("眼球追踪模块 — 调试面板")
    module = EyeTrackingModule(headless=False, master=root)

    frame_select = tk.Frame(root)
    frame_select.pack(pady=(10, 5))
    tk.Label(frame_select, text="左眼相机").pack(side=tk.LEFT, padx=(10, 5))
    cam_left = ttk.Combobox(frame_select, values=cameras, state="readonly", width=8)

    def _set_combo(combo, index):
        try:
            pos = combo['values'].index(str(index))
            combo.current(pos)
        except (ValueError, tk.TclError):
            if combo['values']:
                combo.current(0)

    _set_combo(cam_left, module.config.left.index)
    cam_left.pack(side=tk.LEFT, padx=(0, 20))

    tk.Label(frame_select, text="右眼相机").pack(side=tk.LEFT, padx=(10, 5))
    cam_right = ttk.Combobox(frame_select, values=cameras, state="readonly", width=8)
    _set_combo(cam_right, module.config.right.index)
    cam_right.pack(side=tk.LEFT, padx=(0, 10))

    frame_debug = tk.Frame(root)
    frame_debug.pack(pady=5)

    def _open_crop_left():
        module.set_left_camera(int(cam_left.get()))
        module.open_crop_window("left")

    def _open_crop_right():
        module.set_right_camera(int(cam_right.get()))
        module.open_crop_window("right")

    tk.Button(frame_debug, text="调试剪裁左眼相机", command=_open_crop_left).pack(side=tk.LEFT, padx=10)
    tk.Button(frame_debug, text="调试剪裁右眼相机", command=_open_crop_right).pack(side=tk.LEFT, padx=10)

    frame_action = tk.Frame(root)
    frame_action.pack(pady=(5, 10))

    def _start():
        module.set_left_camera(int(cam_left.get()))
        module.set_right_camera(int(cam_right.get()))
        module.start()
        module.open_control_panel()

    def _stop():
        module.stop()

    tk.Button(frame_action, text="开始眼球追踪", command=_start).pack(side=tk.LEFT, padx=10)
    tk.Button(frame_action, text="停止眼球追踪", command=_stop).pack(side=tk.LEFT, padx=10)
    tk.Button(
        frame_action, text="保存配置",
        command=lambda: (module.set_left_camera(int(cam_left.get())),
                         module.set_right_camera(int(cam_right.get())),
                         module.save_config())
    ).pack(side=tk.LEFT, padx=10)

    # ---- 调试画面隐藏/显示按钮 ----
    frame_display = tk.Frame(root)
    frame_display.pack(pady=(0, 10))

    display_btn_text = tk.StringVar(value="隐藏OpenCV（无头模式）")
    def _toggle_display():
        if module.headless_runtime:
            module.exit_headless_mode()
            display_btn_text.set("隐藏OpenCV（无头模式）")
        else:
            module.enter_headless_mode()
            display_btn_text.set("显示OpenCV（调试模式）")

    tk.Button(
        frame_display, textvariable=display_btn_text, command=_toggle_display, width=14
    ).pack(side=tk.LEFT, padx=5)

    def _on_close():
        module.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()