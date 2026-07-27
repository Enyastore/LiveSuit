"""眼球追踪模块主封装。

职责分层：
  1) 数据模型: CameraConfig, AppConfig
  2) 配置持久化: ConfigPersistence
  3) 调试 UI: CropDebugWindow, ControlPanel, DebugMainWindow
  4) 批处理管线: Normalizer, GazeConsumer
  5) 核心编排: EyeTrackingModule
  6) 独立入口: __main__

提供对外接口：
  module = EyeTrackingModule()
  module.start()
  state = module.get_normalized_eye_state()  # {"left": {"eye_x":..., "eye_y":...}, "right":...}
  module.stop()
"""

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
from pathlib import Path

# 内部依赖
from gaze_vector_tracker import GazeVectorTracker

logger = logging.getLogger(__name__)


# ============================================================
# 1. 数据模型
# ============================================================

@dataclass
class CameraConfig:
    """单个相机的配置。"""
    index: int = 0
    crop: List[int] = field(default_factory=lambda: [0, 640, 0, 480])
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

        config = AppConfig()
        try:
            if 'left_cam_i' in data:
                config.left.index = int(data['left_cam_i'])
            if 'right_cam_i' in data:
                config.right.index = int(data['right_cam_i'])
            if 'left_cam_crop' in data and isinstance(data['left_cam_crop'], list):
                config.left.crop = data['left_cam_crop']
            if 'right_cam_crop' in data and isinstance(data['right_cam_crop'], list):
                config.right.crop = data['right_cam_crop']
            if 'left_cam_flip' in data:
                config.left.flip = bool(data['left_cam_flip'])
            if 'right_cam_flip' in data:
                config.right.flip = bool(data['right_cam_flip'])
        except Exception as e:
            logger.warning(f"配置数据格式有误，部分使用默认值: {e}")
        return config

    def save(self, config: AppConfig) -> None:
        """保存配置到文件。"""
        data = {
            'left_cam_i': config.left.index,
            'right_cam_i': config.right.index,
            'left_cam_crop': config.left.crop,
            'right_cam_crop': config.right.crop,
            'left_cam_flip': config.left.flip,
            'right_cam_flip': config.right.flip,
        }
        try:
            with open(self._filepath, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True)
            logger.info("配置已保存")
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")


# ============================================================
# 3. 调试预览窗口（封装原 _setup_cam 逻辑）
# ============================================================

class CropDebugWindow:
    """Toplevel 调试窗口：显示相机画面，支持垂直翻转与固定比例剪裁。"""

    CROP_TARGET_RATIO: float = 4.0 / 3.0

    def __init__(
        self,
        parent: tk.Tk,
        cam_index: int,
        side: str,
        cam_config: CameraConfig,
        on_config_changed: "callable" = None,
    ):
        """
        Parameters
        ----------
        parent : tk.Tk
            父窗口。
        cam_index : int
            相机索引。
        side : str
            "left" 或 "right"。
        cam_config : CameraConfig
            当前相机配置（将被原地修改）。
        on_config_changed : callable | None
            配置发生变更时的回调。
        """
        self._parent = parent
        self._side = side
        self._cam_config = cam_config
        self._on_config_changed = on_config_changed

        # 视频捕获
        self._cap = cv2.VideoCapture(cam_index)
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开相机 {cam_index}")

        # 窗口
        self._window = tk.Toplevel(parent)
        self._window.title(
            f"调试 - {'左眼' if side == 'left' else '右眼'}相机 (索引 {cam_index})"
        )

        # 裁剪状态
        self._crop_mode: bool = False
        self._crop_rect: Optional[List[int]] = None  # [x1, y1, x2, y2]
        self._drawing: bool = False
        self._start_x: int = 0
        self._start_y: int = 0

        # 运行标志
        self._running: bool = True

        self._build_ui()
        self._bind_mouse_events()
        self._window.protocol("WM_DELETE_WINDOW", self._on_close)
        self._show_frame_loop()

    # ----------------------------------------------------------
    # UI 构建
    # ----------------------------------------------------------

    def _build_ui(self) -> None:
        """构建按钮栏与视频标签。"""
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
        """绑定鼠标事件用于绘制剪裁矩形。"""
        self._video_label.bind("<ButtonPress-1>", self._on_press)
        self._video_label.bind("<B1-Motion>", self._on_drag)
        self._video_label.bind("<ButtonRelease-1>", self._on_release)

    # ----------------------------------------------------------
    # 剪裁交互
    # ----------------------------------------------------------

    def _toggle_crop(self) -> None:
        """切换剪裁模式。"""
        self._crop_mode = not self._crop_mode
        if self._crop_mode:
            self._crop_rect = None
            self._btn_crop.config(relief=tk.SUNKEN)
        else:
            self._btn_crop.config(relief=tk.RAISED)

    def _save_crop_config(self) -> None:
        """保存当前剪裁区域到配置。"""
        if self._crop_rect is not None:
            x1, y1, x2, y2 = self._crop_rect
            logger.info(f"剪裁区域: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
            self._cam_config.crop = [x1, x2, y1, y2]
            self._cam_config.flip = self._flip_var.get()
            if self._on_config_changed:
                self._on_config_changed()
            logger.info(f"{self._side}眼相机配置已保存")
        else:
            logger.warning("未选择剪裁区域")

    # ----------------------------------------------------------
    # 鼠标事件
    # ----------------------------------------------------------

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
    ) -> Optional[List[int]]:
        """返回固定比例的矩形 [x1, y1, x2, y2]，或 None。"""
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return None

        ratio = cls.CROP_TARGET_RATIO
        if abs(dx) / max(abs(dy), 1) > ratio:
            new_w = abs(dx)
            new_h = int(new_w / ratio)
            y2 = y1 + new_h if dy >= 0 else y1 - new_h
            x2 = x1 + dx
        else:
            new_h = abs(dy)
            new_w = int(new_h * ratio)
            x2 = x1 + new_w if dx >= 0 else x1 - new_w
            y2 = y1 + dy

        x1, x2 = min(x1, x2), max(x1, x2)
        y1, y2 = min(y1, y2), max(y1, y2)
        return [x1, y1, x2, y2]

    # ----------------------------------------------------------
    # 视频循环
    # ----------------------------------------------------------

    def _show_frame_loop(self) -> None:
        """读取帧、绘制剪裁框、更新 Tk Label。"""
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
        """关闭窗口，释放相机。"""
        self._running = False
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._window.destroy()

    def get_window(self) -> tk.Toplevel:
        """返回托管的 Toplevel 窗口。"""
        return self._window


# ============================================================
# 4. 控制面板（封装原 _open_control_panel 逻辑）
# ============================================================

class ControlPanel:
    """Toplevel 控制面板：锁定 / 解锁眼球参数 + 保存极限注视向量。"""

    def __init__(
        self,
        parent: tk.Tk,
        cmd_queue_left: Optional[multiprocessing.Queue],
        cmd_queue_right: Optional[multiprocessing.Queue],
        gaze_reader: "callable",   # (side: str) -> Optional[List[float]]
        normalizer: "Normalizer",
    ):
        self._parent = parent
        self._cmd_queue_left = cmd_queue_left
        self._cmd_queue_right = cmd_queue_right
        self._gaze_reader = gaze_reader
        self._normalizer = normalizer

        # 锁定状态
        self._lock_left_radius = False
        self._lock_left_center = False
        self._lock_right_radius = False
        self._lock_right_center = False

        self._window: Optional[tk.Toplevel] = None
        self._open()

    def _open(self) -> None:
        self._window = tk.Toplevel(self._parent)
        self._window.title("控制面板")
        self._window.resizable(False, False)

        tk.Label(self._window, text="眼球追踪控制面板", font=("", 12, "bold")).pack(
            pady=(10, 5)
        )

        # 左眼
        tk.Label(self._window, text="左眼").pack(anchor="w", padx=20, pady=(5, 0))
        self._make_button(
            self._cmd_queue_left,
            "左眼半径",
            "_lock_left_radius",
        ).pack(pady=2)
        self._make_button(
            self._cmd_queue_left,
            "左眼中心",
            "_lock_left_center",
        ).pack(pady=2)

        # 右眼
        tk.Label(self._window, text="右眼").pack(anchor="w", padx=20, pady=(10, 0))
        self._make_button(
            self._cmd_queue_right,
            "右眼半径",
            "_lock_right_radius",
        ).pack(pady=2)
        self._make_button(
            self._cmd_queue_right,
            "右眼中心",
            "_lock_right_center",
        ).pack(pady=2)

        # 极限注视向量
        tk.Label(
            self._window, text="极限注视向量", font=("", 10, "bold")
        ).pack(pady=(10, 5))
        directions = [
            ("仰视", "up"),
            ("俯视", "down"),
            ("内眼角", "inner"),
            ("外眼角", "outer"),
        ]
        for eye_side in ("left", "right"):
            eye_label = "左眼" if eye_side == "left" else "右眼"
            tk.Label(self._window, text=eye_label).pack(
                anchor="w", padx=20, pady=(5, 0)
            )
            for label, dir_key in directions:
                btn = tk.Button(
                    self._window,
                    text=f"保存{eye_label}{label}向量",
                    command=lambda s=eye_side, d=dir_key: self._save_extreme_vector(s, d),
                    width=22,
                )
                btn.pack(pady=1)

        def _on_close() -> None:
            self._window.destroy()
            self._window = None

        self._window.protocol("WM_DELETE_WINDOW", _on_close)

    def _make_button(
        self,
        queue: Optional[multiprocessing.Queue],
        label_prefix: str,
        lock_attr: str,
    ) -> tk.Button:
        """创建一个带动态文字的锁定 / 解锁按钮。"""
        btn_text = tk.StringVar()

        def update_text(*args) -> None:
            locked = getattr(self, lock_attr)
            btn_text.set(
                f"解锁{label_prefix}" if locked else f"锁定{label_prefix}"
            )

        def toggle() -> None:
            if queue is None:
                logger.warning(f"[{label_prefix}] 追踪尚未启动")
                return
            locked = getattr(self, lock_attr)
            if locked:
                if "半径" in label_prefix:
                    queue.put_nowait("unlock_radius")
                else:
                    queue.put_nowait("unlock_center")
            else:
                if "半径" in label_prefix:
                    queue.put_nowait("lock_radius")
                else:
                    queue.put_nowait("lock_center")
            setattr(self, lock_attr, not locked)
            update_text()

        update_text()
        btn = tk.Button(
            self._window, textvariable=btn_text, command=toggle, width=18
        )
        if queue is None:
            btn.config(state=tk.DISABLED)
        return btn

    def _save_extreme_vector(self, side: str, direction: str) -> None:
        """从当前注视向量缓存中取出值并保存为极限向量。"""
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
    """将 gaze_rotated 矢量映射为归一化 eye_x / eye_y 参数。

    依赖极值向量文件 (extream_vectors.yaml) 进行插值。
    若某轴的极值不全则对应轴返回 None。
    """

    def __init__(self, extreme_file: str = "extream_vectors.yaml"):
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
        """记录一个极值向量并持久化。"""
        key = f"{side}_{direction}"
        self._extremes[key] = vector
        try:
            with open(self._extreme_file, 'w', encoding='utf-8') as f:
                yaml.dump(self._extremes, f, allow_unicode=True)
        except Exception as e:
            logger.error(f"保存极值向量失败: {e}")

    def normalize(self, side: str, gaze_rotated: List[float]) -> Dict[str, Optional[float]]:
        """计算归一化参数。

        Returns
        -------
        dict:
            {"eye_x": float|None, "eye_y": float|None}
            eye_x: -1=外眼角, +1=内眼角
            eye_y: +1=仰视, -1=俯视
        """
        result = {"eye_x": None, "eye_y": None}

        if not gaze_rotated or len(gaze_rotated) != 3:
            return result

        # eye_x: inner ↔ outer
        inner_key = f"{side}_inner"
        outer_key = f"{side}_outer"
        if inner_key in self._extremes and outer_key in self._extremes:
            result["eye_x"] = self._angular_interpolate(
                gaze_rotated,
                self._extremes[outer_key],   # outer → -1
                self._extremes[inner_key],   # inner → +1
            )

        # eye_y: up ↔ down
        up_key = f"{side}_up"
        down_key = f"{side}_down"
        if up_key in self._extremes and down_key in self._extremes:
            result["eye_y"] = self._angular_interpolate(
                gaze_rotated,
                self._extremes[down_key],    # down → -1
                self._extremes[up_key],      # up → +1
            )

        return result

    @staticmethod
    def _angular_interpolate(
        current: List[float],
        neg_ref: List[float],
        pos_ref: List[float],
    ) -> float:
        """基于向量夹角做线性插值，neg_ref → -1, pos_ref → +1。"""
        import numpy as np
        a = np.array(current)
        b = np.array(neg_ref)
        c = np.array(pos_ref)

        a_norm = a / np.linalg.norm(a)
        b_norm = b / np.linalg.norm(b)
        c_norm = c / np.linalg.norm(c)

        # total angle between neg and pos references
        total_cos = np.clip(np.dot(b_norm, c_norm), -1.0, 1.0)
        total_angle = np.arccos(total_cos)

        if total_angle < 1e-6:
            return 0.0

        # angle from neg_ref to current
        cos_angle = np.clip(np.dot(b_norm, a_norm), -1.0, 1.0)
        angle = np.arccos(cos_angle)

        # Clamp to [0, total_angle]
        angle = max(0.0, min(angle, total_angle))

        # Map: 0 → -1, total_angle → +1
        return float(2.0 * angle / total_angle - 1.0)


# ============================================================
# 6. 注视向量消费者线程
# ============================================================

class GazeConsumer:
    """守护线程：以高频轮询 result_queue，实时产出归一化眼动数据。

    设计要点：
    - 约 1ms 轮询一次队列，确保不落后于视频帧率
    - 结果写入线程安全的缓存供外部读取
    - 内嵌 Normalizer 管道
    """

    def __init__(self, normalizer: Normalizer):
        self._normalizer = normalizer
        self._queue: Optional[multiprocessing.Queue] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        # 缓存最新的 raw 与 normalized 数据
        self._raw_gaze: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._normalized: Dict[str, Dict[str, Optional[float]]] = {
            "left": {"eye_x": None, "eye_y": None, "confidence": None},
            "right": {"eye_x": None, "eye_y": None, "confidence": None},
        }
        self._last_update: float = 0.0

    def start(self, result_queue: multiprocessing.Queue) -> None:
        """启动消费者线程。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._queue = result_queue
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("注视向量消费者线程已启动")

    def stop(self) -> None:
        """停止消费者线程并清空缓存。"""
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
        """主循环：消费队列 → 归一化 → 写入缓存。"""
        import numpy as np  # local import for subprocess compatibility

        while not self._stop_event.is_set():
            try:
                while True:
                    data = self._queue.get_nowait()
                    self._process(data)
            except queue.Empty:
                pass
            except Exception:
                pass
            self._stop_event.wait(0.001)  # 约 1ms 轮询

    def _process(self, data: dict) -> None:
        """处理单条队列消息。"""
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
        """读取某侧最新原始注视向量（线程安全）。"""
        with self._lock:
            return self._raw_gaze.get(side)

    def get_normalized_state(self) -> dict:
        """读取最新归一化状态（线程安全）。"""
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
        mod = EyeTrackingModule(headless=False)
        mod.start()
        mod.open_control_panel()
        ...
        mod.stop()
    """

    def __init__(
        self,
        headless: bool = False,
        config_path: str = "config.yaml",
        extreme_file: str = "extream_vectors.yaml",
    ):
        self._headless = headless

        # 配置与持久化
        self._persistence = ConfigPersistence(config_path)
        self.config: AppConfig = self._persistence.load()

        # 归一化器
        self._normalizer = Normalizer(extreme_file)

        # 消费者
        self._consumer = GazeConsumer(self._normalizer)

        # 进程管理
        self._process_left: Optional[multiprocessing.Process] = None
        self._process_right: Optional[multiprocessing.Process] = None
        self._cmd_queue_left: Optional[multiprocessing.Queue] = None
        self._cmd_queue_right: Optional[multiprocessing.Queue] = None
        self._result_queue: Optional[multiprocessing.Queue] = None

        # 调试 UI 引用
        self._crop_window_left: Optional[CropDebugWindow] = None
        self._crop_window_right: Optional[CropDebugWindow] = None
        self._control_panel: Optional[ControlPanel] = None

    # ----------------------------------------------------------
    # 公共 API
    # ----------------------------------------------------------

    def start(self) -> None:
        """启动双眼追踪（子进程 + 消费者线程）。"""
        self._stop_internal()

        self._cmd_queue_left = multiprocessing.Queue()
        self._cmd_queue_right = multiprocessing.Queue()
        self._result_queue = multiprocessing.Queue()

        # 启动左眼子进程
        self._process_left = multiprocessing.Process(
            target=_run_tracker_in_process,
            args=(
                self.config.left.index,
                self.config.left.flip,
                self.config.left.crop,
                "left",
                self._cmd_queue_left,
                self._result_queue,
                self._headless,
            ),
            daemon=True,
        )
        self._process_left.start()

        # 启动右眼子进程
        self._process_right = multiprocessing.Process(
            target=_run_tracker_in_process,
            args=(
                self.config.right.index,
                self.config.right.flip,
                self.config.right.crop,
                "right",
                self._cmd_queue_right,
                self._result_queue,
                self._headless,
            ),
            daemon=True,
        )
        self._process_right.start()

        # 启动消费者线程
        self._consumer.start(self._result_queue)

        logger.info("双眼眼球追踪已启动")

    def stop(self) -> None:
        """停止追踪，回收资源。"""
        self._stop_internal()

    def _stop_internal(self) -> None:
        """内部停止逻辑。"""
        # 关闭控制面板
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
        """获取最新的归一化眼动参数。

        Returns
        -------
        dict:
            {
                "left":  {"eye_x": float|None, "eye_y": float|None, "confidence": float|None},
                "right": {"eye_x": float|None, "eye_y": float|None, "confidence": float|None},
                "timestamp": float
            }
        """
        return self._consumer.get_normalized_state()

    def get_raw_gaze_vector(self, side: str) -> Optional[List[float]]:
        """获取某侧最新的原始 gaze_rotated 向量。"""
        return self._consumer.get_raw_gaze(side)

    def is_running(self) -> bool:
        """追踪是否正在运行。"""
        return (
            self._process_left is not None
            and self._process_left.is_alive()
            and self._process_right is not None
            and self._process_right.is_alive()
        )

    # ----------------------------------------------------------
    # 调试 UI 方法（仅非 headless 模式下使用）
    # ----------------------------------------------------------

    def open_crop_window(self, side: str) -> None:
        """打开指定侧的剪裁调试窗口。

        Parameters
        ----------
        side : str
            "left" / "right"
        """
        if self._headless:
            logger.warning("headless 模式下无法打开调试窗口")
            return

        import tkinter as tk
        # 使用一个隐藏的 root 作为父窗口
        parent = tk._default_root
        if parent is None:
            parent = tk.Tk()
            parent.withdraw()

        if side == "left":
            if self._crop_window_left is not None:
                self._crop_window_left.get_window().deiconify()
                return
            self._crop_window_left = CropDebugWindow(
                parent,
                self.config.left.index,
                "left",
                self.config.left,
                on_config_changed=lambda: None,
            )
            def _on_left_close():
                if self._crop_window_left is not None:
                    self._crop_window_left._on_close()
                self._crop_window_left = None
            self._crop_window_left.get_window().protocol(
                "WM_DELETE_WINDOW", _on_left_close
            )
        else:
            if self._crop_window_right is not None:
                self._crop_window_right.get_window().deiconify()
                return
            self._crop_window_right = CropDebugWindow(
                parent,
                self.config.right.index,
                "right",
                self.config.right,
                on_config_changed=lambda: None,
            )
            def _on_right_close():
                if self._crop_window_right is not None:
                    self._crop_window_right._on_close()
                self._crop_window_right = None
            self._crop_window_right.get_window().protocol(
                "WM_DELETE_WINDOW", _on_right_close
            )

    def open_control_panel(self) -> None:
        """打开控制面板（需要先 start）。"""
        if self._headless:
            logger.warning("headless 模式下无法打开控制面板")
            return

        import tkinter as tk
        parent = tk._default_root
        if parent is None:
            parent = tk.Tk()
            parent.withdraw()

        if self._control_panel is not None and self._control_panel.is_open():
            self._control_panel.destroy()

        self._control_panel = ControlPanel(
            parent,
            self._cmd_queue_left,
            self._cmd_queue_right,
            self.get_raw_gaze_vector,
            self._normalizer,
        )

    def save_config(self) -> None:
        """持久化当前配置。"""
        self._persistence.save(self.config)

    def set_left_camera(self, index: int) -> None:
        self.config.left.index = index

    def set_right_camera(self, index: int) -> None:
        self.config.right.index = index

    # ----------------------------------------------------------
    # 生命周期
    # ----------------------------------------------------------

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


# ============================================================
# 8. 子进程入口
# ============================================================

def _run_tracker_in_process(
    cam_index: int,
    flip: bool,
    crop: List[int],
    side: str,
    command_queue: multiprocessing.Queue,
    result_queue: multiprocessing.Queue,
    headless: bool,
) -> None:
    """在独立子进程中创建并运行 GazeVectorTracker。"""
    tracker = GazeVectorTracker(
        cam_index=cam_index, flip=flip, crop=crop, side=side
    )
    tracker.start_tracking(
        command_queue=command_queue,
        result_queue=result_queue,
        headless=headless,
    )


# ============================================================
# 9. 工具函数
# ============================================================

def detect_cameras(max_cams: int = 6) -> List[int]:
    """检测可用的相机索引。"""
    available: List[int] = []
    for i in range(max_cams):
        cap = cv2.VideoCapture(i)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            available.append(i)
            cap.release()
    return available


# ============================================================
# 10. 调试入口（独立运行）
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

    # 创建核心模块（调试模式）
    module = EyeTrackingModule(headless=False)

    # ========================
    # 简单调试 UI
    # ========================
    root = tk.Tk()
    root.title("眼球追踪模块 — 调试面板")

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

    # 调试按钮
    frame_debug = tk.Frame(root)
    frame_debug.pack(pady=5)

    def _open_crop_left():
        module.set_left_camera(int(cam_left.get()))
        module.open_crop_window("left")

    def _open_crop_right():
        module.set_right_camera(int(cam_right.get()))
        module.open_crop_window("right")

    tk.Button(frame_debug, text="调试剪裁左眼相机", command=_open_crop_left).pack(
        side=tk.LEFT, padx=10
    )
    tk.Button(frame_debug, text="调试剪裁右眼相机", command=_open_crop_right).pack(
        side=tk.LEFT, padx=10
    )

    # 功能按钮
    frame_action = tk.Frame(root)
    frame_action.pack(pady=(5, 10))

    def _start():
        module.set_left_camera(int(cam_left.get()))
        module.set_right_camera(int(cam_right.get()))
        module.start()
        module.open_control_panel()

    def _stop():
        module.stop()

    tk.Button(frame_action, text="开始眼球追踪", command=_start).pack(
        side=tk.LEFT, padx=10
    )
    tk.Button(frame_action, text="停止眼球追踪", command=_stop).pack(
        side=tk.LEFT, padx=10
    )
    tk.Button(
        frame_action, text="保存配置",
        command=lambda: (module.set_left_camera(int(cam_left.get())),
                         module.set_right_camera(int(cam_right.get())),
                         module.save_config())
    ).pack(side=tk.LEFT, padx=10)

    def _on_close():
        module.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()