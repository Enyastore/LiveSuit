"""眼球追踪主窗口模块。

提供相机检测、配置管理、调试预览和眼球追踪控制的 GUI 界面。
可作为独立应用运行，也可被其他模块导入使用。
"""

import tkinter as tk
import cv2
from tkinter import ttk
from PIL import Image, ImageTk
import yaml
import multiprocessing
from dataclasses import dataclass, field
from typing import Optional, List
import logging

from eye_tracker import EyeTracker

logger = logging.getLogger(__name__)


# ============================================================
# 数据模型
# ============================================================

@dataclass
class CameraConfig:
    """单个相机的配置。

    Attributes
    ----------
    index : int
        相机索引。
    crop : List[int]
        画面剪裁区域，格式为 [x1, x2, y1, y2]。
    flip : bool
        是否垂直翻转画面。
    """
    index: int = 0
    crop: List[int] = field(default_factory=lambda: [0, 640, 0, 480])
    flip: bool = False


@dataclass
class AppConfig:
    """双眼相机的应用配置，负责持久化到 YAML 文件。

    Attributes
    ----------
    left : CameraConfig
        左眼相机配置。
    right : CameraConfig
        右眼相机配置。
    """
    left: CameraConfig = field(default_factory=CameraConfig)
    right: CameraConfig = field(default_factory=lambda: CameraConfig(index=0))

    CONFIG_FILE: str = "config.yaml"

    @classmethod
    def load(cls) -> "AppConfig":
        """从 YAML 文件加载配置；若文件不存在则创建默认配置并保存。"""
        try:
            with open(cls.CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            logger.info("无配置文件，使用默认配置")
            config = cls()
            config.save()
            return config
        except Exception as e:
            logger.error(f"读取配置文件失败: {e}")
            return cls()

        if data is None:
            return cls()

        config = cls()
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

    def save(self) -> None:
        """保存当前配置到 YAML 文件。"""
        data = {
            'left_cam_i': self.left.index,
            'right_cam_i': self.right.index,
            'left_cam_crop': self.left.crop,
            'right_cam_crop': self.right.crop,
            'left_cam_flip': self.left.flip,
            'right_cam_flip': self.right.flip,
        }
        try:
            with open(self.CONFIG_FILE, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True)
            logger.info("配置已保存")
        except Exception as e:
            logger.error(f"保存配置文件失败: {e}")


# ============================================================
# 工具函数
# ============================================================

def detect_cameras(max_cams: int = 6) -> List[int]:
    """检测可用的相机索引。

    Parameters
    ----------
    max_cams : int
        最多检测的相机数量。

    Returns
    -------
    List[int]
        可用相机的索引列表。
    """
    available: List[int] = []
    for i in range(max_cams):
        cap = cv2.VideoCapture(i)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            available.append(i)
            cap.release()
    return available


def _run_tracker(
    cam_index: int,
    flip: bool,
    crop: List[int],
    side: str,
    command_queue: multiprocessing.Queue
) -> None:
    """在独立进程中运行 EyeTracker（解决 OpenCV GUI 线程冲突）。

    Parameters
    ----------
    command_queue : multiprocessing.Queue
        接收来自主进程的锁定/解锁命令。
    """
    tracker = EyeTracker(cam_index=cam_index, flip=flip, crop=crop, side=side)
    tracker.start_tracking(command_queue=command_queue)


# ============================================================
# 主窗口
# ============================================================

class MainWindow:
    """眼球追踪主窗口。

    负责相机选择、调试剪裁、启动/停止追踪以及锁定控制。

    Parameters
    ----------
    available_cameras : Optional[List[int]]
        可用相机索引列表。若为 None，则自动检测。
    """

    CROP_TARGET_RATIO: float = 4.0 / 3.0

    def __init__(self, available_cameras: Optional[List[int]] = None):
        self.config: AppConfig = AppConfig.load()
        self.available_cameras: List[int] = available_cameras or detect_cameras()

        # 校验配置中的相机索引
        self._validate_config_indices()

        # 进程管理
        self._process_left: Optional[multiprocessing.Process] = None
        self._process_right: Optional[multiprocessing.Process] = None
        self.cmd_queue_left: Optional[multiprocessing.Queue] = None
        self.cmd_queue_right: Optional[multiprocessing.Queue] = None

        # 锁定状态（与子进程实际状态保持同步）
        self._lock_left_radius: bool = False
        self._lock_left_center: bool = False
        self._lock_right_radius: bool = False
        self._lock_right_center: bool = False

        # 构建界面
        self.window = tk.Tk()
        self.window.title("主窗口")
        self._build_ui()

    # ----------------------------------------------------------
    # 内部辅助
    # ----------------------------------------------------------

    def _validate_config_indices(self) -> None:
        """确保配置中的相机索引在可用列表中，否则回退到第一个可用相机。"""
        if not self.available_cameras:
            return
        if self.config.left.index not in self.available_cameras:
            logger.warning(
                f"配置的左眼相机索引 {self.config.left.index} 不可用，"
                f"回退到 {self.available_cameras[0]}"
            )
            self.config.left.index = self.available_cameras[0]
        if self.config.right.index not in self.available_cameras:
            logger.warning(
                f"配置的右眼相机索引 {self.config.right.index} 不可用，"
                f"回退到 {self.available_cameras[0]}"
            )
            self.config.right.index = self.available_cameras[0]

    def _get_side_config(self, side: str) -> CameraConfig:
        """获取指定侧的相机配置。"""
        if side == 'left':
            return self.config.left
        return self.config.right

    def _set_side_config(self, side: str, cam_config: CameraConfig) -> None:
        """更新指定侧的相机配置。"""
        if side == 'left':
            self.config.left = cam_config
        else:
            self.config.right = cam_config

    def _sync_config_from_ui(self) -> None:
        """将 UI 中的相机选择同步到配置对象。"""
        try:
            self.config.left.index = int(self.camera_left.get())
            self.config.right.index = int(self.camera_right.get())
        except (ValueError, tk.TclError):
            pass

    # ----------------------------------------------------------
    # UI 构建
    # ----------------------------------------------------------

    def _build_ui(self) -> None:
        """构建主窗口界面。"""
        cameras = self.available_cameras

        # --- 第一行：下拉选择框 ---
        frame_select = tk.Frame(self.window)
        frame_select.pack(pady=(10, 5))

        tk.Label(frame_select, text="左眼相机").pack(side=tk.LEFT, padx=(10, 5))
        self.camera_left = ttk.Combobox(
            frame_select, values=cameras, state="readonly", width=8
        )
        self._set_combobox_to_index(self.camera_left, self.config.left.index)
        self.camera_left.pack(side=tk.LEFT, padx=(0, 20))

        tk.Label(frame_select, text="右眼相机").pack(side=tk.LEFT, padx=(10, 5))
        self.camera_right = ttk.Combobox(
            frame_select, values=cameras, state="readonly", width=8
        )
        self._set_combobox_to_index(self.camera_right, self.config.right.index)
        self.camera_right.pack(side=tk.LEFT, padx=(0, 10))

        # --- 第二行：调试按钮 ---
        frame_debug = tk.Frame(self.window)
        frame_debug.pack(pady=5)

        tk.Button(
            frame_debug, text="调试剪裁左眼相机",
            command=lambda: self._setup_cam("left", int(self.camera_left.get()))
        ).pack(side=tk.LEFT, padx=10)
        tk.Button(
            frame_debug, text="调试剪裁右眼相机",
            command=lambda: self._setup_cam("right", int(self.camera_right.get()))
        ).pack(side=tk.LEFT, padx=10)

        # --- 第三行：功能按钮 ---
        frame_action = tk.Frame(self.window)
        frame_action.pack(pady=(5, 10))

        tk.Button(frame_action, text="开始眼球追踪", command=self.start_eye_tracking).pack(
            side=tk.LEFT, padx=10
        )
        tk.Button(frame_action, text="停止眼球追踪", command=self.stop_eye_tracking).pack(
            side=tk.LEFT, padx=10
        )
        tk.Button(
            frame_action, text="锁定控制面板", command=self.open_lock_control_panel
        ).pack(side=tk.LEFT, padx=10)
        tk.Button(frame_action, text="保存配置", command=self._save_config).pack(
            side=tk.LEFT, padx=10
        )

    @staticmethod
    def _set_combobox_to_index(combo: ttk.Combobox, index: int) -> None:
        """安全地将 Combobox 设置到指定索引，索引不存在则设为 0。"""
        values = combo['values']
        try:
            pos = values.index(index)
            combo.current(pos)
        except (ValueError, tk.TclError):
            if values:
                combo.current(0)

    # ----------------------------------------------------------
    # 配置持久化
    # ----------------------------------------------------------

    def _save_config(self) -> None:
        """将当前 UI 选择保存到配置文件。"""
        self._sync_config_from_ui()
        self.config.save()

    # ----------------------------------------------------------
    # 调试窗口
    # ----------------------------------------------------------

    def _setup_cam(self, side: str, index: int) -> None:
        """打开 Toplevel 窗口，使用 OpenCV 显示相机视频流并进行剪裁调试。

        Parameters
        ----------
        side : str
            "left" 或 "right"，表示左眼或右眼。
        index : int
            要打开的相机索引。
        """
        cap = cv2.VideoCapture(index)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            logger.error(f"无法打开相机 {index}")
            return

        cam_config = self._get_side_config(side)

        # 创建子窗口
        cam_window = tk.Toplevel(self.window)
        cam_window.title(
            f"调试 - {'左眼' if side == 'left' else '右眼'}相机 (索引 {index})"
        )

        # --- 顶部按钮栏 ---
        btn_frame = tk.Frame(cam_window)
        btn_frame.pack(pady=(5, 0))

        flip_var = tk.BooleanVar(value=cam_config.flip)
        tk.Checkbutton(btn_frame, text="垂直翻转", variable=flip_var).pack(
            side=tk.LEFT, padx=5
        )

        # 裁剪相关状态（使用 nonlocal 替代列表包装）
        crop_mode: bool = False
        crop_rect: Optional[List[int]] = None  # [x1, y1, x2, y2]
        drawing: bool = False
        start_x: int = 0
        start_y: int = 0

        def toggle_crop() -> None:
            nonlocal crop_mode, crop_rect
            crop_mode = not crop_mode
            if crop_mode:
                crop_rect = None
                btn_crop.config(relief=tk.SUNKEN)
            else:
                btn_crop.config(relief=tk.RAISED)

        def save_crop_config() -> None:
            nonlocal crop_rect
            if crop_rect is not None:
                x1, y1, x2, y2 = crop_rect
                logger.info(f"剪裁区域: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                # 转换为 [x1, x2, y1, y2] 格式
                cam_config.crop = [x1, x2, y1, y2]
                cam_config.index = index
                cam_config.flip = flip_var.get()
                self._set_side_config(side, cam_config)
                logger.info(f"{side}眼相机配置已保存")
            else:
                logger.warning("未选择剪裁区域")

        btn_crop = tk.Button(btn_frame, text="剪裁", command=toggle_crop)
        btn_crop.pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="保存剪裁配置", command=save_crop_config).pack(
            side=tk.LEFT, padx=5
        )

        # 视频显示标签
        video_label = tk.Label(cam_window)
        video_label.pack()

        # --- 鼠标事件（绘制剪裁矩形） ---
        def on_press(event: tk.Event) -> None:
            nonlocal drawing, start_x, start_y, crop_rect
            if not crop_mode:
                return
            drawing = True
            start_x = event.x
            start_y = event.y
            crop_rect = None

        def _constrain_rect(
            x1: int, y1: int, x2: int, y2: int
        ) -> Optional[List[int]]:
            """根据起止点计算固定比例的矩形，返回 [x1, y1, x2, y2] 或 None。"""
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0 and dy == 0:
                return None

            ratio = self.CROP_TARGET_RATIO
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

        def on_drag(event: tk.Event) -> None:
            nonlocal crop_rect
            if not crop_mode or not drawing:
                return
            crop_rect = _constrain_rect(start_x, start_y, event.x, event.y)

        def on_release(event: tk.Event) -> None:
            nonlocal drawing, crop_rect
            if not crop_mode:
                return
            drawing = False
            crop_rect = _constrain_rect(start_x, start_y, event.x, event.y)

        video_label.bind("<ButtonPress-1>", on_press)
        video_label.bind("<B1-Motion>", on_drag)
        video_label.bind("<ButtonRelease-1>", on_release)

        # --- 视频循环 ---
        running: bool = True

        def show_frame() -> None:
            nonlocal running
            if not running:
                return
            ret, frame = cap.read()
            if flip_var.get():
                frame = cv2.flip(frame, 0)
            if ret:
                if crop_mode and crop_rect is not None:
                    x1, y1, x2, y2 = crop_rect
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.configure(image=imgtk)
                video_label.after(30, show_frame)
            else:
                logger.warning("无法读取帧")

        def on_close() -> None:
            nonlocal running
            running = False
            cap.release()
            cam_window.destroy()

        cam_window.protocol("WM_DELETE_WINDOW", on_close)
        show_frame()

    # ----------------------------------------------------------
    # 追踪控制
    # ----------------------------------------------------------

    def start_eye_tracking(self) -> None:
        """开始眼球追踪 - 在新进程中启动双眼 EyeTracker。

        若已有追踪在运行，会先将其停止再启动新的。
        """
        self.stop_eye_tracking()

        # 从 UI 同步配置
        self._sync_config_from_ui()

        # 创建命令队列
        self.cmd_queue_left = multiprocessing.Queue()
        self.cmd_queue_right = multiprocessing.Queue()

        # 重置锁定状态
        self._lock_left_radius = False
        self._lock_left_center = False
        self._lock_right_radius = False
        self._lock_right_center = False

        # 启动左眼追踪进程
        self._process_left = multiprocessing.Process(
            target=_run_tracker,
            args=(
                self.config.left.index,
                self.config.left.flip,
                self.config.left.crop,
                "left",
                self.cmd_queue_left,
            ),
            daemon=True,
        )
        self._process_left.start()

        # 启动右眼追踪进程
        self._process_right = multiprocessing.Process(
            target=_run_tracker,
            args=(
                self.config.right.index,
                self.config.right.flip,
                self.config.right.crop,
                "right",
                self.cmd_queue_right,
            ),
            daemon=True,
        )
        self._process_right.start()

        logger.info("双眼眼球追踪已启动")

    def stop_eye_tracking(self) -> None:
        """停止眼球追踪，终止子进程并清理队列。"""
        for proc in (self._process_left, self._process_right):
            if proc is not None and proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
                if proc.is_alive():
                    logger.warning(f"进程 {proc.pid} 未能及时终止")
        self._process_left = None
        self._process_right = None
        self.cmd_queue_left = None
        self.cmd_queue_right = None
        logger.info("眼球追踪已停止")

    # ----------------------------------------------------------
    # 锁定控制面板
    # ----------------------------------------------------------

    def open_lock_control_panel(self) -> None:
        """打开锁定控制面板，可独立锁定/解锁双眼的半径和中心参数。"""
        panel = tk.Toplevel(self.window)
        panel.title("锁定控制面板")
        panel.resizable(False, False)

        # 状态变量（初始值从实例跟踪状态读取）
        var_left_radius = tk.BooleanVar(value=self._lock_left_radius)
        var_right_radius = tk.BooleanVar(value=self._lock_right_radius)
        var_left_center = tk.BooleanVar(value=self._lock_left_center)
        var_right_center = tk.BooleanVar(value=self._lock_right_center)

        def _make_toggle(
            queue: Optional[multiprocessing.Queue],
            state_var: tk.BooleanVar,
            label_prefix: str,
            lock_attr: str,
        ):
            """工厂函数：返回一个 toggle 回调。"""
            def toggle() -> None:
                if queue is None:
                    logger.warning(f"[{label_prefix}] 追踪尚未启动，无法发送命令")
                    return
                if state_var.get():
                    # 当前已锁定 → 发送解锁
                    if "半径" in label_prefix:
                        queue.put_nowait("unlock_radius")
                    else:
                        queue.put_nowait("unlock_center")
                    state_var.set(False)
                    setattr(self, lock_attr, False)
                else:
                    # 当前已解锁 → 发送锁定
                    if "半径" in label_prefix:
                        queue.put_nowait("lock_radius")
                    else:
                        queue.put_nowait("lock_center")
                    state_var.set(True)
                    setattr(self, lock_attr, True)
            return toggle

        def _make_button(
            parent: tk.Widget,
            queue: Optional[multiprocessing.Queue],
            state_var: tk.BooleanVar,
            label_prefix: str,
            lock_attr: str,
        ) -> tk.Button:
            """创建一个带动态文字的锁定/解锁按钮。"""
            btn_text = tk.StringVar()

            def update_text(*args) -> None:
                locked = state_var.get()
                btn_text.set(
                    f"解锁{label_prefix}" if locked else f"锁定{label_prefix}"
                )

            state_var.trace_add("write", update_text)
            update_text()

            cmd = _make_toggle(queue, state_var, label_prefix, lock_attr)
            btn = tk.Button(parent, textvariable=btn_text, command=cmd, width=18)
            if queue is None:
                btn.config(state=tk.DISABLED)
            return btn

        # 布局
        tk.Label(panel, text="眼球追踪锁定控制", font=("", 12, "bold")).pack(
            pady=(10, 5)
        )

        tk.Label(panel, text="左眼").pack(anchor="w", padx=20, pady=(5, 0))
        _make_button(
            panel, self.cmd_queue_left, var_left_radius, "左眼半径", "_lock_left_radius"
        ).pack(pady=2)
        _make_button(
            panel, self.cmd_queue_left, var_left_center, "左眼中心", "_lock_left_center"
        ).pack(pady=2)

        tk.Label(panel, text="右眼").pack(anchor="w", padx=20, pady=(10, 0))
        _make_button(
            panel, self.cmd_queue_right, var_right_radius, "右眼半径", "_lock_right_radius"
        ).pack(pady=2)
        _make_button(
            panel, self.cmd_queue_right, var_right_center, "右眼中心", "_lock_right_center"
        ).pack(pady=2)

        tk.Button(panel, text="关闭面板", command=panel.destroy).pack(pady=(10, 10))

    # ----------------------------------------------------------
    # 生命周期
    # ----------------------------------------------------------

    def run(self) -> None:
        """运行主循环。"""
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)
        self.window.mainloop()

    def _on_close(self) -> None:
        """窗口关闭时的清理回调。"""
        self.stop_eye_tracking()
        self.window.destroy()


# ============================================================
# 入口
# ============================================================

if __name__ == "__main__":
    multiprocessing.set_start_method("spawn", force=True)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    cameras = detect_cameras()
    if not cameras:
        print("Error: 没有可用相机")
        exit(1)
    print(f"可用相机: {cameras}")
    app = MainWindow(available_cameras=cameras)
    app.run()
