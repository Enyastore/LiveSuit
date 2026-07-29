"""眼球追踪模块主入口。

职责分层：
  1) 数据模型: CameraConfig, AppConfig
  2) 配置持久化: ConfigPersistence
  3) 调试 UI: CropDebugWindow, ControlPanel
  4) 核心编排: EyeTrackingModule
  5) 独立入口: __main__

对外接口：
  module = EyeTrackingModule()
  module.start()
  state = module.get_normalized_eye_state()
  module.stop()

调试画面控制（隐藏/显示子进程 OpenCV 窗口）：
  module.enter_headless_mode()
  module.exit_headless_mode()
  module.headless_runtime
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
import subprocess
import time
import logging
import re
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple

# 内部依赖
from eye_tracker_core import GazeVectorTracker, Normalizer

logger = logging.getLogger(__name__)


# ============================================================
# 工具函数
# ============================================================

# V4L2 四字符码整数值（比 cv2.VideoWriter_fourcc 更可靠，V4L2 原生）
_V4L2_FOURCC_MAP = {
    "YUYV": 0x56595559,
    "MJPG": 0x47504A4D,
    "NV12": 0x3231564E,
    "H264": 0x34363248,
    "BGR3": 0x33524742,
    "RGB3": 0x33424752,
}

def _setup_camera(index: int, fps: int = 30, width: int = 640, height: int = 480,
                  fourcc_str: str = "") -> Optional[cv2.VideoCapture]:
    """打开指定索引的相机并设置基本参数。使用 CAP_V4L2 后端确保 FOURCC 兼容。"""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if fourcc_str and fourcc_str in _V4L2_FOURCC_MAP:
        cap.set(cv2.CAP_PROP_FOURCC, _V4L2_FOURCC_MAP[fourcc_str])
    if not cap.isOpened():
        cap.release()
        return None
    return cap


def probe_camera_modes(cam_index: int) -> List[Dict]:
    """通过 v4l2-ctl 探测相机支持的 (width, height, fps, format) 模式列表。

    返回的每个字典包含: width, height, fps, format (如 'YUYV', 'MJPG', '')。
    """
    modes: List[Dict] = []
    try:
        proc = subprocess.run(
            ["v4l2-ctl", "-d", f"/dev/video{cam_index}", "--list-formats-ext"],
            capture_output=True, text=True, timeout=3,
        )
        if proc.returncode != 0:
            logger.warning(f"v4l2-ctl 查询相机 {cam_index} 失败: {proc.stderr.strip()}")
            return modes

        current_fmt = ""
        current_w = current_h = None
        for line in proc.stdout.splitlines():
            # 解析像素格式行: [0]: 'YUYV' (YUYV 4:2:2)
            fmt_m = re.search(r"'(\w+)'\s+\(", line)
            if fmt_m:
                current_fmt = fmt_m.group(1)
                current_w = current_h = None
                continue
            m = re.search(r"Size:\s+Discrete\s+(\d+)x(\d+)", line)
            if m:
                current_w, current_h = int(m.group(1)), int(m.group(2))
                continue
            m = re.search(r"Interval:\s+Discrete\s+[\d.]+\w*\s*\(\s*([\d.]+)\s+fps\)", line)
            if m and current_w is not None and current_h is not None:
                fps = round(float(m.group(1)))
                modes.append({
                    "width": current_w, "height": current_h,
                    "fps": fps, "format": current_fmt,
                })

        # 去重（含格式去重）
        seen = set()
        unique_modes = []
        for m in modes:
            key = (m["width"], m["height"], m["fps"], m["format"])
            if key not in seen:
                seen.add(key)
                unique_modes.append(m)
        return unique_modes
    except FileNotFoundError:
        logger.warning("v4l2-ctl 未安装，无法探测相机模式")
        return modes
    except subprocess.TimeoutExpired:
        logger.warning(f"v4l2-ctl 查询相机 {cam_index} 超时")
        return modes
    except Exception as e:
        logger.error(f"探测相机模式失败: {e}")
        return modes


def mode_to_label(mode: Dict) -> str:
    """将模式字典转换为显示文本，如 '640x480 @30fps (YUYV)'。"""
    fmt = mode.get("format", "")
    label = f"{mode['width']}x{mode['height']} @{mode['fps']}fps"
    if fmt:
        label += f" ({fmt})"
    return label


def match_current_mode(modes: List[Dict], w: int, h: int, fps: int, fmt: str = "") -> Optional[int]:
    """在模式列表中查找匹配当前配置的索引。"""
    for i, m in enumerate(modes):
        if m["width"] == w and m["height"] == h and m["fps"] == fps:
            # 优先完全匹配格式，如果 fmt 为空则仅匹配宽高帧率
            mode_fmt = m.get("format", "")
            if not fmt or not mode_fmt or mode_fmt == fmt:
                return i
    return None


# ============================================================
# 数据模型
# ============================================================

@dataclass
class CameraConfig:
    """单个相机的配置。crop 格式: [x1, y1, x2, y2]。"""
    index: int = 0
    crop: List[int] = field(default_factory=lambda: [0, 0, 640, 480])
    flip: bool = False
    frame_width: int = 640
    frame_height: int = 480
    frame_rate: int = 30
    fourcc: str = ""  # 像素格式，如 'YUYV', 'MJPG'
    use_recommended_resolution: bool = True
    dark_search_roi_scale: float = 0.70
    brightness: float = 0.0    # 明度偏移 [-100, 100]，0 = 不变
    contrast: float = 1.0       # 对比度系数 [0.0, 3.0]，1.0 = 不变
    # 眼睛开闭检测双阈值（0-255）
    openness_threshold_low: int = 0
    openness_threshold_high: int = 80
    # 瞳孔检测双阈值偏移（相对于最暗像素）
    pupil_threshold_low: int = 5
    pupil_threshold_high: int = 25


@dataclass
class AppConfig:
    """双眼相机的应用配置。"""
    left: CameraConfig = field(default_factory=CameraConfig)
    right: CameraConfig = field(default_factory=lambda: CameraConfig(index=0))


# ============================================================
# 配置持久化（与数据模型解耦）
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
                    frame_width=left_data.get('frame_width', 640),
                    frame_height=left_data.get('frame_height', 480),
                    frame_rate=left_data.get('frame_rate', 30),
                    fourcc=left_data.get('fourcc', ''),
                    use_recommended_resolution=left_data.get('use_recommended_resolution', True),
                    dark_search_roi_scale=left_data.get('dark_search_roi_scale', 0.70),
                    brightness=left_data.get('brightness', 0.0),
                    contrast=left_data.get('contrast', 1.0),
                    openness_threshold_low=left_data.get('openness_threshold_low', 0),
                    openness_threshold_high=left_data.get('openness_threshold_high', 80),
                    pupil_threshold_low=left_data.get('pupil_threshold_low', 5),
                    pupil_threshold_high=left_data.get('pupil_threshold_high', 25),
                ),
                right=CameraConfig(
                    index=right_data.get('camera_index', 0),
                    crop=right_data.get('crop', [0, 0, 640, 480]),
                    flip=right_data.get('flip', False),
                    frame_width=right_data.get('frame_width', 640),
                    frame_height=right_data.get('frame_height', 480),
                    frame_rate=right_data.get('frame_rate', 30),
                    fourcc=right_data.get('fourcc', ''),
                    use_recommended_resolution=right_data.get('use_recommended_resolution', True),
                    dark_search_roi_scale=right_data.get('dark_search_roi_scale', 0.70),
                    brightness=right_data.get('brightness', 0.0),
                    contrast=right_data.get('contrast', 1.0),
                    openness_threshold_low=right_data.get('openness_threshold_low', 0),
                    openness_threshold_high=right_data.get('openness_threshold_high', 80),
                    pupil_threshold_low=right_data.get('pupil_threshold_low', 5),
                    pupil_threshold_high=right_data.get('pupil_threshold_high', 25),
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
                'frame_width': config.left.frame_width,
                'frame_height': config.left.frame_height,
                'frame_rate': config.left.frame_rate,
                'fourcc': config.left.fourcc,
                'use_recommended_resolution': config.left.use_recommended_resolution,
                'dark_search_roi_scale': config.left.dark_search_roi_scale,
                'brightness': config.left.brightness,
                'contrast': config.left.contrast,
                'openness_threshold_low': config.left.openness_threshold_low,
                'openness_threshold_high': config.left.openness_threshold_high,
                'pupil_threshold_low': config.left.pupil_threshold_low,
                'pupil_threshold_high': config.left.pupil_threshold_high,
            },
            'right': {
                'camera_index': config.right.index,
                'crop': config.right.crop,
                'flip': config.right.flip,
                'frame_width': config.right.frame_width,
                'frame_height': config.right.frame_height,
                'frame_rate': config.right.frame_rate,
                'fourcc': config.right.fourcc,
                'use_recommended_resolution': config.right.use_recommended_resolution,
                'dark_search_roi_scale': config.right.dark_search_roi_scale,
                'brightness': config.right.brightness,
                'contrast': config.right.contrast,
                'openness_threshold_low': config.right.openness_threshold_low,
                'openness_threshold_high': config.right.openness_threshold_high,
                'pupil_threshold_low': config.right.pupil_threshold_low,
                'pupil_threshold_high': config.right.pupil_threshold_high,
            },
        }
        try:
            with open(self._filepath, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=None)
            logger.info("配置已保存")
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")


# ============================================================
# 调试预览窗口
# ============================================================

class CropDebugWindow:
    """Toplevel 调试窗口：显示相机画面，支持垂直翻转与固定比例剪裁。

    右半部分包含：
      - 二值化调试视口（实时显示模糊+二值化+椭圆掩膜结果及水平参考线）
      - 开度检测参数滑块（阈值、模糊核、聚合方式）
    """

    CROP_TARGET_RATIO: float = 4.0 / 3.0

    # 二值化调试视口大小（缩放后显示）
    BINARY_VIEW_WIDTH: int = 320
    BINARY_VIEW_HEIGHT: int = 240

    def __init__(
        self,
        master: tk.Tk,
        cam_index: int,
        side: str,
        cam_config: CameraConfig,
        on_config_changed: "callable" = None,
        cmd_queue: Optional[multiprocessing.Queue] = None,
    ):
        self._master = master
        self._side = side
        self._cam_config = cam_config
        self._on_config_changed = on_config_changed
        self._cmd_queue = cmd_queue

        self._cap = _setup_camera(
            cam_index,
            fps=cam_config.frame_rate,
            width=cam_config.frame_width,
            height=cam_config.frame_height,
            fourcc_str=cam_config.fourcc,
        )
        if self._cap is None:
            raise RuntimeError(f"无法打开相机 {cam_index}")

        self._window = tk.Toplevel(master)
        self._window.title(
            f"调试 - {'左眼' if side == 'left' else '右眼'}相机 (索引 {cam_index})"
        )

        saved_crop = cam_config.crop
        if saved_crop and len(saved_crop) == 4:
            self._crop_rect: Optional[Tuple[int, int, int, int]] = list(saved_crop)
        else:
            self._crop_rect: Optional[Tuple[int, int, int, int]] = (
                [0, 0, cam_config.frame_width, cam_config.frame_height]
            )
        self._drawing: bool = False
        self._start_x: int = 0
        self._start_y: int = 0
        self._running: bool = True

        # 开度调试参数（双阈值）
        self._openness_low = cam_config.openness_threshold_low
        self._openness_high = cam_config.openness_threshold_high
        self._openness_blur = 3
        self._openness_aggregation = "median"

        # 瞳孔调试参数（双阈值偏移）
        self._pupil_low = cam_config.pupil_threshold_low
        self._pupil_high = cam_config.pupil_threshold_high

        self._build_ui()
        self._bind_mouse_events()
        self._window.protocol("WM_DELETE_WINDOW", self._on_close)
        self._show_frame_loop()

    def _build_ui(self) -> None:
        # === 主水平容器：左(主画面+控制) | 右(二值化调试) ===
        main_horizontal = tk.Frame(self._window)
        main_horizontal.pack(fill=tk.BOTH, expand=True)

        # ---- 左侧：主画面 + 控制条 ----
        left_panel = tk.Frame(main_horizontal)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 左控制条 1
        btn_frame = tk.Frame(left_panel)
        btn_frame.pack(pady=(5, 0))
        self._flip_var = tk.BooleanVar(value=self._cam_config.flip)
        tk.Checkbutton(btn_frame, text="垂直翻转", variable=self._flip_var).pack(
            side=tk.LEFT, padx=5
        )
        self._btn_crop = tk.Button(btn_frame, text="重置剪裁", command=self._reset_crop)
        self._btn_crop.pack(side=tk.LEFT, padx=5)
        self._recommended_var = tk.BooleanVar(value=self._cam_config.use_recommended_resolution)
        tk.Checkbutton(btn_frame, text="使用推荐宽高比(4:3)", variable=self._recommended_var).pack(
            side=tk.LEFT, padx=5
        )
        tk.Button(btn_frame, text="保存相机配置", command=self._save_crop_config).pack(
            side=tk.LEFT, padx=5
        )

        # 左控制条 2：搜索区域 + 帧率/分辨率
        roi_frame = tk.Frame(left_panel)
        roi_frame.pack(fill=tk.X, padx=10, pady=(3, 0))
        tk.Label(roi_frame, text="搜索区域:").pack(side=tk.LEFT)
        self._roi_scale_var = tk.DoubleVar(value=self._cam_config.dark_search_roi_scale)
        self._roi_scale = tk.Scale(
            roi_frame, from_=0.1, to=1.0, resolution=0.05,
            orient=tk.HORIZONTAL, variable=self._roi_scale_var,
            length=150, showvalue=True,
            command=self._on_roi_scale_changed,
        )
        self._roi_scale.pack(side=tk.LEFT, padx=(5, 0))

        res_frame = tk.Frame(left_panel)
        res_frame.pack(fill=tk.X, padx=10, pady=(3, 5))
        tk.Label(res_frame, text="帧率/分辨率:").pack(side=tk.LEFT)
        self._modes = probe_camera_modes(self._cam_config.index)
        mode_labels = [mode_to_label(m) for m in self._modes]
        self._mode_var = tk.StringVar()
        self._mode_combo = ttk.Combobox(
            res_frame, values=mode_labels, state="readonly", width=30,
            textvariable=self._mode_var,
        )
        self._mode_combo.pack(side=tk.LEFT, padx=(5, 0))
        idx = match_current_mode(
            self._modes,
            self._cam_config.frame_width,
            self._cam_config.frame_height,
            self._cam_config.frame_rate,
            self._cam_config.fourcc,
        )
        if idx is not None:
            self._mode_combo.current(idx)
        elif self._modes:
            self._mode_combo.current(0)
            m = self._modes[0]
            self._cam_config.frame_width = m["width"]
            self._cam_config.frame_height = m["height"]
            self._cam_config.frame_rate = m["fps"]
            self._cam_config.fourcc = m.get("format", "")
        self._mode_combo.bind("<<ComboboxSelected>>", self._on_resolution_changed)

        # 左控制条 3：明度/对比度
        pp_frame = tk.Frame(left_panel)
        pp_frame.pack(fill=tk.X, padx=10, pady=(3, 0))
        tk.Label(pp_frame, text="明度:").pack(side=tk.LEFT)
        self._brightness_var = tk.DoubleVar(value=self._cam_config.brightness)
        tk.Scale(
            pp_frame, from_=-100, to=100, resolution=1,
            orient=tk.HORIZONTAL, variable=self._brightness_var,
            length=120, showvalue=True,
            command=self._on_postprocess_changed,
        ).pack(side=tk.LEFT, padx=(5, 10))
        tk.Label(pp_frame, text="对比度:").pack(side=tk.LEFT)
        self._contrast_var = tk.DoubleVar(value=self._cam_config.contrast)
        tk.Scale(
            pp_frame, from_=0.0, to=5.0, resolution=0.1,
            orient=tk.HORIZONTAL, variable=self._contrast_var,
            length=120, showvalue=True,
            command=self._on_postprocess_changed,
        ).pack(side=tk.LEFT, padx=(5, 0))

        # 主画面标签
        self._video_label = tk.Label(left_panel)
        self._video_label.pack()

        # ---- 右侧：上(开闭二值化) | 下(瞳孔二值化) ----
        right_panel = tk.Frame(main_horizontal, padx=10, pady=5)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y)

        # --- 上半：开闭二值化 ---
        openness_frame = tk.LabelFrame(right_panel, text="开闭二值化调试", padx=5, pady=3)
        openness_frame.pack(fill=tk.X, pady=(0, 8))

        self._openness_binary_label = tk.Label(openness_frame, bg="black")
        self._openness_binary_label.pack()

        sliders_frame = tk.Frame(openness_frame)
        sliders_frame.pack(fill=tk.X, pady=(3, 0))

        tk.Label(sliders_frame, text="low:", font=("", 7)).pack(side=tk.LEFT)
        self._open_low_var = tk.IntVar(value=self._openness_low)
        tk.Scale(
            sliders_frame, from_=0, to=255, resolution=1,
            orient=tk.HORIZONTAL, variable=self._open_low_var,
            length=140, showvalue=True,
            command=self._on_open_low_changed,
        ).pack(side=tk.LEFT)

        tk.Label(sliders_frame, text="high:", font=("", 7)).pack(side=tk.LEFT, padx=(8, 0))
        self._open_high_var = tk.IntVar(value=self._openness_high)
        tk.Scale(
            sliders_frame, from_=0, to=255, resolution=1,
            orient=tk.HORIZONTAL, variable=self._open_high_var,
            length=140, showvalue=True,
            command=self._on_open_high_changed,
        ).pack(side=tk.LEFT)

        params_frame = tk.Frame(openness_frame)
        params_frame.pack(fill=tk.X, pady=(2, 0))
        tk.Label(params_frame, text="模糊核:").pack(side=tk.LEFT)
        self._open_blur_var = tk.IntVar(value=self._openness_blur)
        tk.Scale(
            params_frame, from_=1, to=15, resolution=2,
            orient=tk.HORIZONTAL, variable=self._open_blur_var,
            length=100, showvalue=True,
            command=self._on_openness_blur_changed,
        ).pack(side=tk.LEFT, padx=(2, 10))
        tk.Label(params_frame, text="聚合:").pack(side=tk.LEFT)
        self._open_agg_var = tk.StringVar(value=self._openness_aggregation)
        agg_combo = ttk.Combobox(
            params_frame, values=["median", "average"], state="readonly",
            textvariable=self._open_agg_var, width=8,
        )
        agg_combo.pack(side=tk.LEFT, padx=(2, 0))
        agg_combo.bind("<<ComboboxSelected>>", self._on_openness_aggregation_changed)

        self._openness_info_label = tk.Label(
            openness_frame, text="", font=("", 7), justify=tk.LEFT
        )
        self._openness_info_label.pack(anchor="w")

        # --- 下半：瞳孔二值化 ---
        pupil_frame = tk.LabelFrame(right_panel, text="瞳孔二值化调试", padx=5, pady=3)
        pupil_frame.pack(fill=tk.X)

        self._pupil_binary_label = tk.Label(pupil_frame, bg="black")
        self._pupil_binary_label.pack()

        pupil_sliders = tk.Frame(pupil_frame)
        pupil_sliders.pack(fill=tk.X, pady=(3, 0))

        tk.Label(pupil_sliders, text="low offset:", font=("", 7)).pack(side=tk.LEFT)
        self._pupil_low_var = tk.IntVar(value=self._pupil_low)
        tk.Scale(
            pupil_sliders, from_=0, to=50, resolution=1,
            orient=tk.HORIZONTAL, variable=self._pupil_low_var,
            length=120, showvalue=True,
            command=self._on_pupil_low_changed,
        ).pack(side=tk.LEFT)

        tk.Label(pupil_sliders, text="high offset:", font=("", 7)).pack(side=tk.LEFT, padx=(8, 0))
        self._pupil_high_var = tk.IntVar(value=self._pupil_high)
        tk.Scale(
            pupil_sliders, from_=1, to=100, resolution=1,
            orient=tk.HORIZONTAL, variable=self._pupil_high_var,
            length=120, showvalue=True,
            command=self._on_pupil_high_changed,
        ).pack(side=tk.LEFT)

        self._pupil_info_label = tk.Label(
            pupil_frame, text="", font=("", 7), justify=tk.LEFT
        )
        self._pupil_info_label.pack(anchor="w")

    def _bind_mouse_events(self) -> None:
        self._video_label.bind("<ButtonPress-1>", self._on_press)
        self._video_label.bind("<B1-Motion>", self._on_drag)
        self._video_label.bind("<ButtonRelease-1>", self._on_release)

    def _reset_crop(self) -> None:
        """重置剪裁黄框为整个画面范围。"""
        self._crop_rect = [0, 0, self._cam_config.frame_width, self._cam_config.frame_height]

    def _save_crop_config(self) -> None:
        # 保存当前所有参数到 cam_config
        if self._crop_rect is None:
            self._crop_rect = [0, 0, self._cam_config.frame_width, self._cam_config.frame_height]
        x1, y1, x2, y2 = self._crop_rect
        logger.info(f"剪裁区域: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
        self._cam_config.crop = [x1, y1, x2, y2]
        self._cam_config.flip = self._flip_var.get()
        self._cam_config.use_recommended_resolution = self._recommended_var.get()
        idx = self._mode_combo.current()
        if 0 <= idx < len(self._modes):
            m = self._modes[idx]
            self._cam_config.frame_width = m["width"]
            self._cam_config.frame_height = m["height"]
            self._cam_config.frame_rate = m["fps"]
            self._cam_config.fourcc = m.get("format", "")
        # 保存阈值参数
        self._cam_config.openness_threshold_low = self._open_low_var.get()
        self._cam_config.openness_threshold_high = self._open_high_var.get()
        self._cam_config.pupil_threshold_low = self._pupil_low_var.get()
        self._cam_config.pupil_threshold_high = self._pupil_high_var.get()
        if self._on_config_changed:
            self._on_config_changed()
        logger.info(f"{self._side}眼相机配置已保存")

    def _on_press(self, event: tk.Event) -> None:
        self._drawing = True
        self._start_x = event.x
        self._start_y = event.y
        self._crop_rect = None

    def _on_drag(self, event: tk.Event) -> None:
        if not self._drawing:
            return
        enforce = self._recommended_var.get()
        self._crop_rect = self._constrain_rect(
            self._start_x, self._start_y, event.x, event.y,
            enforce_ratio=enforce,
        )

    def _on_release(self, event: tk.Event) -> None:
        self._drawing = False
        enforce = self._recommended_var.get()
        self._crop_rect = self._constrain_rect(
            self._start_x, self._start_y, event.x, event.y,
            enforce_ratio=enforce,
        )

    @classmethod
    def _constrain_rect(
        cls, x1: int, y1: int, x2: int, y2: int,
        enforce_ratio: bool = True,
    ) -> Optional[Tuple[int, int, int, int]]:
        if not enforce_ratio:
            return [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]
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

    def _on_resolution_changed(self, event: tk.Event = None) -> None:
        """下拉框选择分辨率/帧率时回调：更新配置并重启相机预览，同时通知子进程。"""
        idx = self._mode_combo.current()
        if idx < 0 or idx >= len(self._modes):
            return
        m = self._modes[idx]
        new_fmt = m.get("format", "")

        # 重启预览相机
        if self._cap is not None:
            self._cap.release()
        new_cap = _setup_camera(
            self._cam_config.index,
            fps=m["fps"],
            width=m["width"],
            height=m["height"],
            fourcc_str=new_fmt,
        )
        if new_cap is None:
            # 切换失败，回退到原模式
            logger.warning(f"[{self._side}] 切换分辨率 {mode_to_label(m)} 失败，已还原")
            self._cap = _setup_camera(
                self._cam_config.index,
                fps=self._cam_config.frame_rate,
                width=self._cam_config.frame_width,
                height=self._cam_config.frame_height,
                fourcc_str=self._cam_config.fourcc,
            )
            # 回退下拉框选中项
            fallback_idx = match_current_mode(
                self._modes,
                self._cam_config.frame_width,
                self._cam_config.frame_height,
                self._cam_config.frame_rate,
                self._cam_config.fourcc,
            )
            if fallback_idx is not None:
                self._mode_combo.current(fallback_idx)
            return

        self._cap = new_cap
        self._cam_config.frame_width = m["width"]
        self._cam_config.frame_height = m["height"]
        self._cam_config.frame_rate = m["fps"]
        self._cam_config.fourcc = new_fmt
        logger.info(f"[{self._side}] 切换分辨率: {mode_to_label(m)}")

        # 通知子进程动态切换相机参数
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait((
                    "restart_capture",
                    m["width"], m["height"], m["fps"], new_fmt,
                ))
                logger.info(f"[{self._side}] 已通知子进程切换分辨率")
            except Exception as e:
                logger.error(f"[{self._side}] 通知子进程切换失败: {e}")

    def _on_roi_scale_changed(self, val: str) -> None:
        """搜索区域滑块回调：更新配置，实时发送到子进程。"""
        scale = float(val)
        self._cam_config.dark_search_roi_scale = scale
        logger.info(f"[{self._side}] 搜索区域比例: {scale:.2f}")

    def _on_postprocess_changed(self, val: str = None) -> None:
        """明度/对比度滑块回调：更新配置，实时发送到子进程。"""
        brightness = self._brightness_var.get()
        contrast = self._contrast_var.get()
        self._cam_config.brightness = brightness
        self._cam_config.contrast = contrast
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_postprocess", brightness, contrast))
            except Exception as e:
                logger.error(f"[{self._side}] 发送后处理参数失败: {e}")
        logger.info(f"[{self._side}] 明度: {brightness:.0f}, 对比度: {contrast:.1f}")

    def _draw_search_ellipse_on_frame(self, frame, crop_w, crop_h, crop_rect=None):
        """在帧上绘制绿色椭圆标记搜索区域。

        若有 crop_rect（黄框），椭圆中心为黄框中心，轴比例匹配黄框宽高比；
        否则椭圆在整帧中心。
        """
        scale = self._cam_config.dark_search_roi_scale
        if crop_rect is not None:
            x1, y1, x2, y2 = crop_rect
            cw = x2 - x1
            ch = y2 - y1
            cx_e, cy_e = (x1 + x2) // 2, (y1 + y2) // 2
            rx_e = int((cw / 2) * scale)
            ry_e = int((ch / 2) * scale)
        else:
            cx_e, cy_e = crop_w // 2, crop_h // 2
            rx_e = int((crop_w / 2) * scale)
            ry_e = int((crop_h / 2) * scale)
        cv2.ellipse(
            frame,
            (cx_e, cy_e),
            (max(rx_e, 1), max(ry_e, 1)),
            0, 0, 360,
            (0, 220, 0), 2,
        )

    def _compute_binary_debug(self, frame, crop_rect=None):
        """对一帧执行与 C++ compute_eye_openness 相同的流程，返回二值图+参考线。

        椭圆中心与 _draw_search_ellipse_on_frame 一致（跟随 crop_rect）。

        Parameters
        ----------
        frame : np.ndarray (BGR)
        crop_rect : None 或 [x1, y1, x2, y2]，用于定位椭圆中心

        Returns
        -------
        binary_bgr : np.ndarray
            带参考线的彩色二值图，用于显示
        top_agg, bottom_agg : float
            聚合后的高点/低点 y 坐标（原始帧坐标系）
        raw_distance : float
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        scale = self._cam_config.dark_search_roi_scale

        # 椭圆中心与半径必须与 _draw_search_ellipse_on_frame 完全一致
        if crop_rect is not None:
            x1, y1, x2, y2 = crop_rect
            cx_roi = (x1 + x2) // 2
            cy_roi = (y1 + y2) // 2
            cw = x2 - x1
            ch = y2 - y1
            rx = int((cw / 2.0) * scale)
            ry = int((ch / 2.0) * scale)
        else:
            cx_roi, cy_roi = w // 2, h // 2
            rx = int((w / 2.0) * scale)
            ry = int((h / 2.0) * scale)

        # 模糊
        blur = self._open_blur_var.get()
        if blur % 2 == 0:
            blur += 1
        blurred = gray
        if blur > 1:
            blurred = cv2.GaussianBlur(gray, (blur, blur), 0)

        # 二值化
        threshold = self._open_threshold_var.get()
        _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY_INV)

        # 椭圆掩膜
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.ellipse(mask, (cx_roi, cy_roi), (max(rx, 1), max(ry, 1)), 0, 0, 360, 255, -1)
        masked = cv2.bitwise_and(binary, mask)

        # 逐列扫描
        min_x = max(0, cx_roi - rx)
        max_x = min(w, cx_roi + rx)
        min_y = max(0, cy_roi - ry)
        max_y = min(h, cy_roi + ry)

        top_pts, bottom_pts = [], []
        for col in range(min_x, max_x):
            col_slice = masked[min_y:max_y, col]
            fg = np.where(col_slice > 0)[0]
            if len(fg) >= 2:
                top_pts.append(float(min_y + fg[0]))
                bottom_pts.append(float(min_y + fg[-1]))

        top_agg, bottom_agg = 0.0, 0.0
        raw_distance = 0.0
        if top_pts and bottom_pts:
            agg = self._open_agg_var.get()
            if agg == "average":
                top_agg = np.mean(top_pts)
                bottom_agg = np.mean(bottom_pts)
            else:  # median
                top_agg = float(np.median(top_pts))
                bottom_agg = float(np.median(bottom_pts))
            raw_distance = max(0.0, bottom_agg - top_agg)

        # 构建彩色二值图
        binary_bgr = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
        # 画椭圆边框
        cv2.ellipse(binary_bgr, (cx_roi, cy_roi), (max(rx, 1), max(ry, 1)), 0, 0, 360, (0, 200, 0), 1)
        # 画聚合水平线
        if top_agg > 0 and bottom_agg > 0:
            cv2.line(binary_bgr, (min_x, int(top_agg)), (max_x, int(top_agg)), (0, 255, 0), 2)  # 绿-高点
            cv2.line(binary_bgr, (min_x, int(bottom_agg)), (max_x, int(bottom_agg)), (0, 0, 255), 2)  # 红-低点
            # 两点连线
            mid_x = (min_x + max_x) // 2
            cv2.line(binary_bgr, (mid_x, int(top_agg)), (mid_x, int(bottom_agg)), (255, 255, 0), 1)

        return binary_bgr, top_agg, bottom_agg, raw_distance

    def _compute_openness_binary_debug(self, frame, crop_rect=None):
        """模拟 C++ compute_eye_openness 的双阈值 inRange 流程。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        scale = self._cam_config.dark_search_roi_scale

        if crop_rect is not None:
            x1, y1, x2, y2 = crop_rect
            cx_roi = (x1 + x2) // 2
            cy_roi = (y1 + y2) // 2
            cw = x2 - x1
            ch = y2 - y1
            rx = int((cw / 2.0) * scale)
            ry = int((ch / 2.0) * scale)
        else:
            cx_roi, cy_roi = w // 2, h // 2
            rx = int((w / 2.0) * scale)
            ry = int((h / 2.0) * scale)

        blur = self._open_blur_var.get()
        if blur % 2 == 0:
            blur += 1
        blurred = gray
        if blur > 1:
            blurred = cv2.GaussianBlur(gray, (blur, blur), 0)

        # 双阈值 inRange
        low_val = self._open_low_var.get()
        high_val = self._open_high_var.get()
        if low_val >= high_val:
            low_val, high_val = 0, 1
        binary = cv2.inRange(blurred, low_val, high_val)

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.ellipse(mask, (cx_roi, cy_roi), (max(rx, 1), max(ry, 1)), 0, 0, 360, 255, -1)
        masked = cv2.bitwise_and(binary, mask)

        min_x = max(0, cx_roi - rx)
        max_x = min(w, cx_roi + rx)
        min_y = max(0, cy_roi - ry)
        max_y = min(h, cy_roi + ry)

        top_pts, bottom_pts = [], []
        for col in range(min_x, max_x):
            col_slice = masked[min_y:max_y, col]
            fg = np.where(col_slice > 0)[0]
            if len(fg) >= 2:
                top_pts.append(float(min_y + fg[0]))
                bottom_pts.append(float(min_y + fg[-1]))

        top_agg, bottom_agg = 0.0, 0.0
        raw_distance = 0.0
        if top_pts and bottom_pts:
            agg = self._open_agg_var.get()
            if agg == "average":
                top_agg = np.mean(top_pts)
                bottom_agg = np.mean(bottom_pts)
            else:
                top_agg = float(np.median(top_pts))
                bottom_agg = float(np.median(bottom_pts))
            raw_distance = max(0.0, bottom_agg - top_agg)

        binary_bgr = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
        cv2.ellipse(binary_bgr, (cx_roi, cy_roi), (max(rx, 1), max(ry, 1)), 0, 0, 360, (0, 200, 0), 1)
        if top_agg > 0 and bottom_agg > 0:
            cv2.line(binary_bgr, (min_x, int(top_agg)), (max_x, int(top_agg)), (0, 255, 0), 2)
            cv2.line(binary_bgr, (min_x, int(bottom_agg)), (max_x, int(bottom_agg)), (0, 0, 255), 2)
            mid_x = (min_x + max_x) // 2
            cv2.line(binary_bgr, (mid_x, int(top_agg)), (mid_x, int(bottom_agg)), (255, 255, 0), 1)
        return binary_bgr, top_agg, bottom_agg, raw_distance

    def _compute_pupil_binary_debug(self, frame, crop_rect=None):
        """模拟 C++ 瞳孔双阈值二值化，并在二值图上拟合椭圆、计算 goodness。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        scale = self._cam_config.dark_search_roi_scale

        if crop_rect is not None:
            x1, y1, x2, y2 = crop_rect
            cx_roi = (x1 + x2) // 2
            cy_roi = (y1 + y2) // 2
            cw = x2 - x1
            ch = y2 - y1
            rx = int((cw / 2.0) * scale)
            ry = int((ch / 2.0) * scale)
        else:
            cx_roi, cy_roi = w // 2, h // 2
            rx = int((w / 2.0) * scale)
            ry = int((h / 2.0) * scale)

        # 找最暗点
        blurred_gray = cv2.GaussianBlur(gray, (3, 3), 0)
        min_val = 255
        best_pt = (cx_roi, cy_roi)
        for gy in range(10, h - 10, 5):
            for gx in range(10, w - 10, 5):
                v = int(blurred_gray[gy, gx])
                if v < min_val:
                    min_val = v
                    best_pt = (gx, gy)

        low_offset = self._pupil_low_var.get()
        high_offset = self._pupil_high_var.get()
        low_val = max(0, min(255, min_val + low_offset))
        high_val = max(low_val + 1, min(255, min_val + high_offset))

        pupil_binary = cv2.inRange(gray, low_val, high_val)

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.ellipse(mask, (cx_roi, cy_roi), (max(rx, 1), max(ry, 1)), 0, 0, 360, 255, -1)
        masked = cv2.bitwise_and(pupil_binary, mask)

        # ---- 椭圆拟合与 goodness 计算（参考原始 Python 算法） ----
        ellipse_info = None  # ((cx, cy), (w, h), angle) or None
        goodness_cover = 0.0  # 覆盖百分比
        goodness_aspect = 0.0  # 椭圆长短轴比例接近圆程度
        goodness_total = 0.0  # 综合分数

        # 膨胀以连接轮廓
        kernel = np.ones((5, 5), np.uint8)
        dilated = cv2.dilate(masked, kernel, iterations=2)

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 筛选面积 >= 200、长宽比 <= 4 的最大轮廓
        best_contour = None
        best_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= 200:
                x, y, cw_c, ch_c = cv2.boundingRect(cnt)
                ratio = max(cw_c, ch_c) / max(min(cw_c, ch_c), 1)
                if ratio <= 4.0 and area > best_area:
                    best_area = area
                    best_contour = cnt

        if best_contour is not None and len(best_contour) >= 5:
            ellipse = cv2.fitEllipse(best_contour)
            ellipse_info = ellipse
            (el_cx, el_cy), (el_w, el_h), el_angle = ellipse

            # ---- goodness 计算 ----
            # 1) 椭圆内覆盖比例
            el_mask = np.zeros_like(masked)
            cv2.ellipse(el_mask, ellipse, 255, -1)
            covered = np.sum((masked == 255) & (el_mask == 255))
            el_area = np.sum(el_mask == 255)
            goodness_cover = covered / max(el_area, 1)

            # 2) 长短轴接近圆程度（0~1，1=正圆）
            goodness_aspect = min(el_w, el_h) / max(el_w, el_h)

            # 3) 综合（参考原始算法：cover * 面积因子近似）
            goodness_total = goodness_cover * goodness_aspect * 100.0

        # ---- 构建显示图像 ----
        binary_bgr = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)

        # 画搜索区域椭圆（绿色）
        cv2.ellipse(binary_bgr, (cx_roi, cy_roi), (max(rx, 1), max(ry, 1)), 0, 0, 360, (0, 200, 0), 1)

        # 画最暗点（红色）
        cv2.circle(binary_bgr, best_pt, 4, (0, 0, 255), -1)

        # 画拟合椭圆（青色），仅当拟合成功
        if ellipse_info is not None:
            cv2.ellipse(binary_bgr, ellipse_info, (255, 255, 0), 2)  # 青色
            # 标记椭圆中心
            cv2.circle(binary_bgr, (int(ellipse_info[0][0]), int(ellipse_info[0][1])), 3, (255, 255, 0), -1)

        return binary_bgr, best_pt, min_val, ellipse_info, goodness_cover, goodness_aspect, goodness_total

    def _resize_binary_view(self, binary_bgr):
        debug_h, debug_w = binary_bgr.shape[:2]
        scale_x = self.BINARY_VIEW_WIDTH / debug_w
        scale_y = self.BINARY_VIEW_HEIGHT / debug_h
        scale = min(scale_x, scale_y)
        new_w = int(debug_w * scale)
        new_h = int(debug_h * scale)
        resized = cv2.resize(binary_bgr, (new_w, new_h))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        return ImageTk.PhotoImage(image=img)

    def _show_frame_loop(self) -> None:
        if not self._running:
            return
        if self._cap is None:
            self._video_label.after(100, self._show_frame_loop)
            return
        ret, frame = self._cap.read()
        if not ret:
            logger.warning("无法读取帧")
            self._video_label.after(100, self._show_frame_loop)
            return

        if self._flip_var.get():
            frame = cv2.flip(frame, 0)

        alpha = self._cam_config.contrast
        beta = self._cam_config.brightness
        if alpha != 1.0 or beta != 0.0:
            frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        h, w = frame.shape[:2]

        self._draw_search_ellipse_on_frame(frame, w, h, self._crop_rect)
        if self._crop_rect is not None:
            x1, y1, x2, y2 = self._crop_rect
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # 主画面
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        self._video_label.imgtk = imgtk
        self._video_label.configure(image=imgtk)

        # 开闭二值化视口
        try:
            bf = frame.copy()
            open_bin, top_y, bottom_y, raw_dist = self._compute_openness_binary_debug(
                bf, crop_rect=self._crop_rect)
            open_imgtk = self._resize_binary_view(open_bin)
            self._openness_binary_label.imgtk = open_imgtk
            self._openness_binary_label.configure(image=open_imgtk)
            info = f"raw={raw_dist:.1f}px  top={top_y:.0f}  bot={bottom_y:.0f}"
            info += f"\nagg={self._open_agg_var.get()}  blur={self._open_blur_var.get()}"
            self._openness_info_label.config(text=info)
        except Exception:
            pass

        # 瞳孔二值化视口
        try:
            bf2 = frame.copy()
            pupil_bin, best_pt, darkest_val, ellipse_info, g_cover, g_aspect, g_total = self._compute_pupil_binary_debug(
                bf2, crop_rect=self._crop_rect)
            pupil_imgtk = self._resize_binary_view(pupil_bin)
            self._pupil_binary_label.imgtk = pupil_imgtk
            self._pupil_binary_label.configure(image=pupil_imgtk)
            pinfo = f"darkest={darkest_val}  lo={self._pupil_low_var.get()}  hi={self._pupil_high_var.get()}"
            if ellipse_info is not None:
                pinfo += f"\ngoodness: cover={g_cover:.2f}  aspect={g_aspect:.2f}  total={g_total:.1f}"
            else:
                pinfo += "\nno ellipse fitted"
            self._pupil_info_label.config(text=pinfo)
        except Exception:
            pass

        self._video_label.after(30, self._show_frame_loop)

    # ---- 开闭双阈值回调 ----
    def _on_open_low_changed(self, val: str) -> None:
        self._openness_low = int(val)
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_openness_threshold_low", self._openness_low))
            except Exception as e:
                logger.error(f"[{self._side}] 发送开闭 low 阈值失败: {e}")

    def _on_open_high_changed(self, val: str) -> None:
        self._openness_high = int(val)
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_openness_threshold_high", self._openness_high))
            except Exception as e:
                logger.error(f"[{self._side}] 发送开闭 high 阈值失败: {e}")

    def _on_openness_blur_changed(self, val: str) -> None:
        blur = int(val)
        if blur % 2 == 0:
            blur += 1
        self._openness_blur = blur
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_openness_blur", blur))
            except Exception as e:
                logger.error(f"[{self._side}] 发送开度模糊核失败: {e}")

    def _on_openness_aggregation_changed(self, event: tk.Event = None) -> None:
        agg = self._open_agg_var.get()
        self._openness_aggregation = agg
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_openness_aggregation", agg))
            except Exception as e:
                logger.error(f"[{self._side}] 发送开度聚合方式失败: {e}")

    # ---- 瞳孔双阈值回调 ----
    def _on_pupil_low_changed(self, val: str) -> None:
        self._pupil_low = int(val)
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_pupil_threshold_low", self._pupil_low))
            except Exception as e:
                logger.error(f"[{self._side}] 发送瞳孔 low 阈值失败: {e}")

    def _on_pupil_high_changed(self, val: str) -> None:
        self._pupil_high = int(val)
        if self._cmd_queue is not None:
            try:
                self._cmd_queue.put_nowait(("set_pupil_threshold_high", self._pupil_high))
            except Exception as e:
                logger.error(f"[{self._side}] 发送瞳孔 high 阈值失败: {e}")

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
# 控制面板
# ============================================================

class ControlPanel:
    """Toplevel 控制面板：锁定/解锁眼球参数 + 保存极限注视向量 + 眼睛开度标定。

    极值向量通过命令队列发送到子进程，由子进程内部的 Normalizer 处理。
    """

    def __init__(
        self,
        master: tk.Tk,
        cmd_queue_left: Optional[multiprocessing.Queue],
        cmd_queue_right: Optional[multiprocessing.Queue],
        gaze_reader: "callable",
        side: str = "left",
        openness_normalizer: "Normalizer" = None,
        module: "EyeTrackingModule" = None,
    ):
        self._master = master
        self._cmd_queue_left = cmd_queue_left
        self._cmd_queue_right = cmd_queue_right
        self._gaze_reader = gaze_reader
        self._openness_normalizer = openness_normalizer
        self._module = module
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
            row = tk.Frame(self._window)
            row.pack(pady=(2, 0))
            tk.Label(row, text=eye_label, width=4).pack(side=tk.LEFT)
            for label, dir_key in directions:
                tk.Button(
                    row,
                    text=label,
                    command=lambda s=eye_side, d=dir_key: self._save_extreme_vector(s, d),
                    width=10,
                ).pack(side=tk.LEFT, padx=2)

        # 清除极值向量按钮
        tk.Label(self._window, text="清除极值向量", font=("", 10, "bold")).pack(pady=(10, 5))
        btn_clear_left = tk.Button(
            self._window,
            text="清除左眼极值向量",
            command=lambda: self._clear_extremes("left"),
            width=22,
        )
        btn_clear_left.pack(pady=1)
        btn_clear_right = tk.Button(
            self._window,
            text="清除右眼极值向量",
            command=lambda: self._clear_extremes("right"),
            width=22,
        )
        btn_clear_right.pack(pady=1)

        # ---- 眼睛开度跳过阈值 ----
        tk.Label(self._window, text="低开度跳过阈值", font=("", 10, "bold")).pack(pady=(10, 5))
        tk.Label(self._window, text="开度低于此值时跳过眼追省电 (0=无)", font=("", 8)).pack()

        skip_frame = tk.Frame(self._window)
        skip_frame.pack(pady=(5, 0))

        tk.Label(skip_frame, text="左眼").pack(side=tk.LEFT, padx=(10, 2))
        self._skip_left_var = tk.DoubleVar(value=0.0)
        tk.Scale(
            skip_frame, from_=0, to=200, resolution=1,
            orient=tk.HORIZONTAL, variable=self._skip_left_var,
            length=200, showvalue=True,
            command=self._on_skip_left_changed,
        ).pack(side=tk.LEFT, padx=(0, 15))

        tk.Label(skip_frame, text="右眼").pack(side=tk.LEFT, padx=(10, 2))
        self._skip_right_var = tk.DoubleVar(value=0.0)
        tk.Scale(
            skip_frame, from_=0, to=200, resolution=1,
            orient=tk.HORIZONTAL, variable=self._skip_right_var,
            length=200, showvalue=True,
            command=self._on_skip_right_changed,
        ).pack(side=tk.LEFT, padx=(0, 10))

        # ---- 保存开度参考值 ----
        tk.Label(self._window, text="眼睛开度标定", font=("", 10, "bold")).pack(pady=(10, 5))

        # 当前 raw 值显示
        self._raw_label = tk.Label(self._window, text="", font=("", 8))
        self._raw_label.pack()

        for eye_side in ("left", "right"):
            eye_label = "左眼" if eye_side == "left" else "右眼"
            tk.Label(self._window, text=eye_label).pack(anchor="w", padx=20, pady=(3, 0))
            btn_frame = tk.Frame(self._window)
            btn_frame.pack(pady=(0, 2))
            tk.Button(
                btn_frame,
                text="  保存全睁距离  ",
                command=lambda s=eye_side: self._save_openness_ref(s, "open"),
            ).pack(side=tk.LEFT, padx=5)
            tk.Button(
                btn_frame,
                text="  保存全闭距离  ",
                command=lambda s=eye_side: self._save_openness_ref(s, "close"),
            ).pack(side=tk.LEFT, padx=5)
            tk.Button(
                btn_frame,
                text="  清除开闭距离  ",
                command=lambda s=eye_side: self._clear_openness_refs(s),
            ).pack(side=tk.LEFT, padx=5)

        # ---- 最终输出值（eye_x, eye_y, eye_o）----
        tk.Label(self._window, text="最终输出2", font=("", 10, "bold")).pack(pady=(10, 5))
        self._final_output_label = tk.Label(self._window, text="", font=("", 9), justify=tk.LEFT)
        self._final_output_label.pack()
        self._update_raw_display()

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
        # 通过命令队列发送到子进程，由子进程的 Normalizer 处理
        q = self._cmd_queue_left if side == "left" else self._cmd_queue_right
        if q is not None:
            try:
                q.put_nowait(("save_extreme", direction, vector))
                logger.info(f"已发送 {side}_{direction} 到子进程保存")
            except Exception as e:
                logger.error(f"发送极值向量到子进程失败: {e}")
        else:
            logger.warning(f"[{side}] 追踪尚未启动，无法保存")

    def _clear_extremes(self, side: str) -> None:
        """清除指定眼（left/right）的所有极值向量。"""
        q = self._cmd_queue_left if side == "left" else self._cmd_queue_right
        if q is not None:
            try:
                q.put_nowait("clear_extremes")
                logger.info(f"已发送清除 {side} 眼极值向量命令到子进程")
            except Exception as e:
                logger.error(f"发送清除极值向量命令到子进程失败: {e}")
        else:
            logger.warning(f"[{side}] 追踪尚未启动，无法清除极值向量")

    def _on_skip_left_changed(self, val: str) -> None:
        threshold = float(val)
        if self._module is not None:
            self._module._openness_skip_threshold_left = threshold  # store for UI
        if self._cmd_queue_left is not None:
            try:
                self._cmd_queue_left.put_nowait(("set_openness_skip_threshold", threshold))
                logger.info(f"左眼跳过阈值: {threshold:.0f}")
            except Exception as e:
                logger.error(f"发送左眼跳过阈值失败: {e}")

    def _on_skip_right_changed(self, val: str) -> None:
        threshold = float(val)
        if self._module is not None:
            self._module._openness_skip_threshold_right = threshold
        if self._cmd_queue_right is not None:
            try:
                self._cmd_queue_right.put_nowait(("set_openness_skip_threshold", threshold))
                logger.info(f"右眼跳过阈值: {threshold:.0f}")
            except Exception as e:
                logger.error(f"发送右眼跳过阈值失败: {e}")

    def _update_raw_display(self) -> None:
        """定时更新当前 raw 开度显示，以及最终输出 eye_x/eye_y/eye_o。"""
        if self._window is None or not self._window.winfo_exists():
            return
        if self._module is not None:
            state = self._module.get_normalized_eye_state()

            # --- raw 显示（仅开度，2 位小数） ---
            left_raw = state["left"].get("raw_eye_openness", "N/A")
            right_raw = state["right"].get("raw_eye_openness", "N/A")
            text = f"左: raw={left_raw:.2f}" if isinstance(left_raw, (int, float)) else f"左: raw={left_raw}"
            text += f"  右: raw={right_raw:.2f}" if isinstance(right_raw, (int, float)) else f"  右: raw={right_raw}"
            self._raw_label.config(text=text)

            # --- 最终输出显示（eye_x, eye_y, eye_o，各 2 位小数） ---
            if hasattr(self, '_final_output_label'):
                final_lines = []
                for side_key, label in [("left", "左眼"), ("right", "右眼")]:
                    x = state[side_key].get("eye_x", None)
                    y = state[side_key].get("eye_y", None)
                    o = state[side_key].get("eye_o", None)
                    parts = []
                    parts.append(f"x={x:+.2f}" if isinstance(x, (int, float)) else "x=N/A")
                    parts.append(f"y={y:+.2f}" if isinstance(y, (int, float)) else "y=N/A")
                    parts.append(f"o={o:.2f}" if isinstance(o, (int, float)) else "o=N/A")
                    final_lines.append(f"{label}: {'  '.join(parts)}")
                self._final_output_label.config(text="\n".join(final_lines))

        self._window.after(500, self._update_raw_display)

    def _save_openness_ref(self, side: str, ref_type: str) -> None:
        """保存开度参考值（全睁/全闭）到 Normalizer。"""
        if self._module is not None:
            state = self._module.get_normalized_eye_state()
            raw = state[side].get("raw_eye_openness", None)
            if raw is not None and raw > 0:
                self._openness_normalizer.set_openness_ref(side, ref_type, raw)
                logger.info(f"已保存 {side}_{ref_type} = {raw:.1f}")
            else:
                logger.warning(f"未能获取 {side} 当前开度值，跳过保存")

    def _clear_openness_refs(self, side: str) -> None:
        """清除指定眼睛（left/right）的开闭参考距离（open/close）并持久化。"""
        if self._openness_normalizer is not None:
            self._openness_normalizer.clear_openness_ref(side)
            logger.info(f"已清除 {side} 眼开闭参考距离")
        else:
            logger.warning(f"[{side}] Normalizer 未初始化，无法清除开闭参考距离")

    def destroy(self) -> None:
        if self._window is not None:
            self._window.destroy()
            self._window = None

    def is_open(self) -> bool:
        return self._window is not None


# ============================================================
# 核心编排：EyeTrackingModule
# ============================================================

class EyeTrackingModule:
    """眼球追踪核心模块 —— 对外唯一入口。

    使用方式：
        mod = EyeTrackingModule(headless=True)
        mod.start()
        state = mod.get_normalized_eye_state()
        mod.stop()

    调试模式：
        mod = EyeTrackingModule(headless=False, master=root)
        mod.start()
        mod.open_control_panel()

    调试画面控制（仅隐藏/显示子进程 OpenCV 窗口，tkinter 窗口不受影响）：
        mod.enter_headless_mode()
        mod.exit_headless_mode()
        mod.headless_runtime

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
        self._extreme_file = extreme_file

        if headless:
            self._master: Optional[tk.Tk] = None
        elif master is not None:
            self._master = master
        else:
            self._master = tk.Tk()
            self._master.withdraw()

        self._persistence = ConfigPersistence(config_path)
        self.config: AppConfig = self._persistence.load()

        self._process_left: Optional[multiprocessing.Process] = None
        self._process_right: Optional[multiprocessing.Process] = None
        self._cmd_queue_left: Optional[multiprocessing.Queue] = None
        self._cmd_queue_right: Optional[multiprocessing.Queue] = None
        self._result_queue: Optional[multiprocessing.Queue] = None

        # 结果收集
        self._latest_state: dict = {
            "left": {"eye_x": None, "eye_y": None, "confidence": None,
                     "raw_eye_openness": None, "eye_o": None},
            "right": {"eye_x": None, "eye_y": None, "confidence": None,
                      "raw_eye_openness": None, "eye_o": None},
            "timestamp": 0.0,
        }
        self._raw_gaze: Dict[str, Optional[List[float]]] = {"left": None, "right": None}
        self._result_thread: Optional[threading.Thread] = None
        self._result_stop = threading.Event()

        self._crop_window_left: Optional[CropDebugWindow] = None
        self._crop_window_right: Optional[CropDebugWindow] = None
        self._control_panel: Optional[ControlPanel] = None

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
            args=(
                self.config.left.index, self.config.left.flip, self.config.left.crop,
                "left", self._cmd_queue_left, self._result_queue, self._headless,
                self.config.left.frame_width, self.config.left.frame_height,
                self.config.left.frame_rate,
                self._extreme_file, self.config.left.use_recommended_resolution,
                self.config.left.dark_search_roi_scale,
                self.config.left.fourcc,
                self.config.left.brightness,
                self.config.left.contrast,
                self.config.left.openness_threshold_low,
                self.config.left.openness_threshold_high,
                self.config.left.pupil_threshold_low,
                self.config.left.pupil_threshold_high,
            ),
            daemon=True,
        )
        self._process_left.start()

        self._process_right = multiprocessing.Process(
            target=_run_tracker_in_process,
            args=(
                self.config.right.index, self.config.right.flip, self.config.right.crop,
                "right", self._cmd_queue_right, self._result_queue, self._headless,
                self.config.right.frame_width, self.config.right.frame_height,
                self.config.right.frame_rate,
                self._extreme_file, self.config.right.use_recommended_resolution,
                self.config.right.dark_search_roi_scale,
                self.config.right.fourcc,
                self.config.right.brightness,
                self.config.right.contrast,
                self.config.right.openness_threshold_low,
                self.config.right.openness_threshold_high,
                self.config.right.pupil_threshold_low,
                self.config.right.pupil_threshold_high,
            ),
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

        # 启动结果收集线程
        self._start_result_collector()
        logger.info("双眼眼球追踪已启动")

    def stop(self) -> None:
        self._stop_internal()

    def _start_result_collector(self) -> None:
        """启动轻量线程，从 result_queue 读取数据并更新 latest_state。"""
        self._result_stop.clear()
        self._result_thread = threading.Thread(target=self._collect_results, daemon=True)
        self._result_thread.start()

    def _collect_results(self) -> None:
        # 创建 Normalizer 实例用于读取开度参考值
        openness_normalizer = Normalizer(self._extreme_file)

        while not self._result_stop.is_set():
            try:
                data = self._result_queue.get(timeout=0.01)
            except queue.Empty:
                continue
            except Exception:
                continue

            side = data.get("side")
            if side is None:
                continue
            self._raw_gaze[side] = data.get("gaze_rotated")

            raw_openness = data.get("raw_eye_openness", None)

            # 计算归一化 eye_o
            eye_o = None
            if raw_openness is not None and isinstance(raw_openness, (int, float)) and raw_openness > 0:
                open_ref = openness_normalizer.get_openness_ref(side, "open")
                close_ref = openness_normalizer.get_openness_ref(side, "close")
                if open_ref is not None and close_ref is not None and open_ref > close_ref:
                    eye_o = (raw_openness - close_ref) / (open_ref - close_ref)
                    eye_o = max(0.0, min(1.0, eye_o))

            self._latest_state[side] = {
                "eye_x": data.get("eye_x"),
                "eye_y": data.get("eye_y"),
                "confidence": data.get("confidence"),
                "raw_eye_openness": raw_openness,
                "eye_o": eye_o,
            }
            self._latest_state["timestamp"] = time.time()

    def _stop_internal(self) -> None:
        if self._control_panel is not None:
            self._control_panel.destroy()
            self._control_panel = None

        # 停止结果收集线程
        self._result_stop.set()
        if self._result_thread is not None:
            self._result_thread.join(timeout=2)
            self._result_thread = None

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
        self._result_queue = None
        self._raw_gaze = {"left": None, "right": None}
        self._latest_state = {
            "left": {"eye_x": None, "eye_y": None, "confidence": None},
            "right": {"eye_x": None, "eye_y": None, "confidence": None},
            "timestamp": 0.0,
        }
        logger.info("眼球追踪已停止")

    def get_normalized_eye_state(self) -> dict:
        return dict(self._latest_state)

    def get_raw_gaze_vector(self, side: str) -> Optional[List[float]]:
        return self._raw_gaze.get(side)

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
                cmd_queue=self._cmd_queue_left,
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
                cmd_queue=self._cmd_queue_right,
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
        openness_normalizer = Normalizer(self._extreme_file)
        self._control_panel = ControlPanel(
            self._master, self._cmd_queue_left, self._cmd_queue_right,
            self.get_raw_gaze_vector,
            openness_normalizer=openness_normalizer,
            module=self,
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
# 子进程入口
# ============================================================

def _run_tracker_in_process(
    cam_index: int, flip: bool, crop: List[int], side: str,
    command_queue: multiprocessing.Queue,
    result_queue: multiprocessing.Queue, headless: bool,
    frame_width: int, frame_height: int, frame_rate: int,
    extreme_file: str,
    use_recommended_resolution: bool = True,
    dark_search_roi_scale: float = 0.70,
    fourcc_str: str = "",
    brightness: float = 0.0,
    contrast: float = 1.0,
    openness_threshold_low: int = 0,
    openness_threshold_high: int = 80,
    pupil_threshold_low: int = 5,
    pupil_threshold_high: int = 25,
) -> None:
    tracker = GazeVectorTracker(
        cam_index=cam_index, flip=flip, crop=crop, side=side,
        frame_width=frame_width, frame_height=frame_height,
        frame_rate=frame_rate, extreme_file=extreme_file,
        use_recommended_resolution=use_recommended_resolution,
        dark_search_roi_scale=dark_search_roi_scale,
        fourcc_str=fourcc_str,
        brightness=brightness,
        contrast=contrast,
        openness_threshold_low=openness_threshold_low,
        openness_threshold_high=openness_threshold_high,
        pupil_threshold_low=pupil_threshold_low,
        pupil_threshold_high=pupil_threshold_high,
    )
    tracker.start_tracking(
        command_queue=command_queue, result_queue=result_queue, headless=headless,
    )


# ============================================================
# 工具函数
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
# 调试入口
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

    btn_debug_left = tk.Button(frame_debug, text="调试剪裁左眼相机", command=lambda: _open_crop_left())
    btn_debug_left.pack(side=tk.LEFT, padx=10)
    btn_debug_right = tk.Button(frame_debug, text="调试剪裁右眼相机", command=lambda: _open_crop_right())
    btn_debug_right.pack(side=tk.LEFT, padx=10)

    def _open_crop_left():
        module.set_left_camera(int(cam_left.get()))
        module.open_crop_window("left")

    def _open_crop_right():
        module.set_right_camera(int(cam_right.get()))
        module.open_crop_window("right")

    def _set_debug_enabled(enabled: bool) -> None:
        """启用/禁用调试相关控件。"""
        state = tk.NORMAL if enabled else tk.DISABLED
        cam_left.config(state=state)
        cam_right.config(state=state)
        btn_debug_left.config(state=state)
        btn_debug_right.config(state=state)

    frame_action = tk.Frame(root)
    frame_action.pack(pady=(5, 10))

    def _start():
        module.set_left_camera(int(cam_left.get()))
        module.set_right_camera(int(cam_right.get()))
        module.start()
        module.open_control_panel()
        _set_debug_enabled(False)

    def _stop():
        module.stop()
        _set_debug_enabled(True)

    btn_text = tk.StringVar(value="开始眼球追踪")

    def _toggle():
        if module.is_running():
            _stop()
            btn_text.set("开始眼球追踪")
        else:
            _start()
            btn_text.set("停止眼球追踪")

    tk.Button(frame_action, textvariable=btn_text, command=_toggle, width=14).pack(side=tk.LEFT, padx=10)

    frame_display = tk.Frame(root)
    frame_display.pack(pady=(0, 10))

    display_btn_text = tk.StringVar(value="隐藏OpenCV")
    def _toggle_display():
        if module.headless_runtime:
            module.exit_headless_mode()
            display_btn_text.set("隐藏OpenCV")
        else:
            module.enter_headless_mode()
            display_btn_text.set("显示OpenCV")

    tk.Button(
        frame_display, textvariable=display_btn_text, command=_toggle_display, width=14
    ).pack(side=tk.LEFT, padx=5)

    def _on_close():
        module.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    root.mainloop()