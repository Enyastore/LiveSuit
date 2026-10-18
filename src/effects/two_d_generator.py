"""2D 生成器效果器：0 输入 -> 2 输出。

端口语义：outputs[0] = 手柄 x 坐标，outputs[1] = 手柄 y 坐标（均为归一化值）。
自带面板：xOy 矩形网格，坐标范围均为 0~1，原点位于 (0.5, 0.5)；
左键点击或长按拖动手柄，右键把坐标重置回原点。
"""

import threading

from core.effectors import Effector

CENTER = 0.5
DEFAULT_POSITION = (CENTER, CENTER)
GRID_STEP = 0.1

# 面板配色（与 gui 层保持一致）
_CANVAS_BG = "#ffffff"
_GRID_COLOR = "#e0e0e0"
_AXIS_COLOR = "#555555"
_BORDER_COLOR = "#90a4ae"
_HANDLE_COLOR = "#c62828"
_TEXT_COLOR = "#263238"


class TwoDGeneratorEffector(Effector):
    """用手柄生成二维坐标的效果器（无输入；坐标由面板在主线程设置）。"""

    TYPE_NAME = "2D-Generator"

    def __init__(self, x=DEFAULT_POSITION[0], y=DEFAULT_POSITION[1]):
        self._lock = threading.RLock()
        self._x = self._clamp(x)
        self._y = self._clamp(y)
        self._panel = None

    # ---- Effector 接口 ----
    def get_input_count(self):
        return 0

    def get_output_count(self):
        return 2

    def process(self, inputs):
        with self._lock:
            return [self._x, self._y]

    def reset(self):
        with self._lock:
            self._x, self._y = DEFAULT_POSITION

    def get_params(self):
        with self._lock:
            return {"x": self._x, "y": self._y}

    def set_params(self, params):
        if not params:
            return
        with self._lock:
            if params.get("x") is not None:
                self._x = self._clamp(params["x"])
            if params.get("y") is not None:
                self._y = self._clamp(params["y"])

    def snapshot(self):
        """返回 (x, y)，加锁供面板读取。"""
        with self._lock:
            return (self._x, self._y)

    # ---- 参数 ----
    def set_position(self, x, y):
        with self._lock:
            self._x = self._clamp(x)
            self._y = self._clamp(y)

    @staticmethod
    def _clamp(value):
        return max(0.0, min(1.0, float(value)))

    # ---- 自带面板 ----
    def show_panel(self, parent=None):
        """显示/置顶 2D 生成器面板（xOy 网格 + 可拖动手柄）。"""
        if self._panel is not None:
            try:
                if self._panel.winfo_exists():
                    self._panel.deiconify()
                    self._panel.lift()
                    return self._panel
            except Exception:  # noqa: BLE001  面板已销毁
                self._panel = None
        self._panel = _TwoDGeneratorPanel(parent, self)
        return self._panel

    def close_panel(self):
        if self._panel is not None:
            try:
                self._panel.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._panel = None


class _TwoDGeneratorPanel:
    """2D 生成器自带面板（tkinter Toplevel）。"""

    def __init__(self, parent, effector, width=320, height=300):
        import tkinter as tk

        self.effector = effector
        self._width = width
        self._height = height

        self.top = tk.Toplevel(parent) if parent is not None else tk.Tk()
        self.top.title(effector.label or "2D 生成器")
        self.top.resizable(True, True)

        self._dragging = False
        self.canvas = tk.Canvas(self.top, width=width, height=height,
                                bg=_CANVAS_BG, highlightthickness=1,
                                highlightbackground="#ccc")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 4))
        self.canvas.bind("<Button-1>", self._on_left_press)
        self.canvas.bind("<B1-Motion>", self._on_left_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_release)
        self.canvas.bind("<Button-3>", self._on_right_click)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        self._coord_label = tk.Label(self.top, text="", anchor="w")
        self._coord_label.pack(fill=tk.X, padx=8, pady=(0, 8))

        self.redraw()

    # ---- 面板动作 ----
    def _on_left_press(self, event):
        if self._set_from_event(event):
            self._dragging = True

    def _on_left_motion(self, event):
        if self._dragging:
            # 拖出网格后仍按边界 clamp，便于沿着四条边继续拖动手柄。
            self._set_from_event(event, clamp=True)

    def _on_left_release(self, _event):
        self._dragging = False

    def _on_right_click(self, _event):
        self.effector.set_position(*DEFAULT_POSITION)
        self.redraw()

    def _on_canvas_configure(self, _event):
        """窗口缩放时立即按新尺寸重绘，而不是等下一次手柄交互。"""
        self.redraw()

    def _set_from_event(self, event, clamp=False):
        """把鼠标位置换算为归一化坐标。

        初始按下落在绘图区外则忽略；拖动时 clamp=True，越界鼠标会被
        钳到最近的边界值，从而实现沿边界拖动。
        """
        rect = self._plot_rect()
        left, top, pw, ph = rect
        inside = (left <= event.x <= left + pw
                  and top <= event.y <= top + ph)
        if not inside and not clamp:
            return False
        x = (event.x - left) / pw
        y = 1.0 - (event.y - top) / ph
        self.effector.set_position(x, y)
        self.redraw()
        return True

    def _canvas_size(self):
        """返回画布实际尺寸；窗口尚未映射时回退到构造尺寸。"""
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w <= 1:
            w = self._width
        if h <= 1:
            h = self._height
        return w, h

    def _plot_rect(self):
        pad_l, pad_r, pad_t, pad_b = 30, 16, 28, 30
        w, h = self._canvas_size()
        return (pad_l, pad_t, max(1.0, w - pad_l - pad_r),
                max(1.0, h - pad_t - pad_b))

    def redraw(self):
        x, y = self.effector.snapshot()
        c = self.canvas
        c.delete("all")
        left, top, pw, ph = self._plot_rect()

        # 0.1 步长的矩形网格
        steps = int(round(1.0 / GRID_STEP))
        for i in range(steps + 1):
            frac = i / steps
            gx = left + frac * pw
            gy = top + frac * ph
            c.create_line(gx, top, gx, top + ph, fill=_GRID_COLOR)
            c.create_line(left, gy, left + pw, gy, fill=_GRID_COLOR)

        # 外框与过原点的 x/y 轴
        c.create_rectangle(left, top, left + pw, top + ph,
                           outline=_BORDER_COLOR)
        ox = left + CENTER * pw
        oy = top + CENTER * ph
        c.create_line(left, oy, left + pw, oy, fill=_AXIS_COLOR)
        c.create_line(ox, top, ox, top + ph, fill=_AXIS_COLOR)
        c.create_text(left + pw + 6, oy, text="x", anchor="w",
                      fill=_AXIS_COLOR)
        c.create_text(ox, top - 8, text="y", anchor="s",
                      fill=_AXIS_COLOR)
        c.create_text(ox + 8, oy + 8, text="O", anchor="nw",
                      fill=_AXIS_COLOR)

        # 刻度标签
        c.create_text(left, top + ph + 8, text="0", anchor="n",
                      fill=_AXIS_COLOR)
        c.create_text(left + pw, top + ph + 8, text="1", anchor="n",
                      fill=_AXIS_COLOR)
        c.create_text(left - 8, top + ph, text="0", anchor="e",
                      fill=_AXIS_COLOR)
        c.create_text(left - 8, top, text="1", anchor="e",
                      fill=_AXIS_COLOR)

        # 手柄
        hx = left + x * pw
        hy = top + (1.0 - y) * ph
        c.create_oval(hx - 7, hy - 7, hx + 7, hy + 7,
                      fill=_HANDLE_COLOR, outline="")
        c.create_text(hx, hy - 12, text=f"({x:.2f}, {y:.2f})", anchor="s",
                      fill=_TEXT_COLOR)

        self._coord_label.config(text=f"x: {x:.3f}    y: {y:.3f}")

    def destroy(self):
        try:
            self.top.destroy()
        except Exception:  # noqa: BLE001
            pass
