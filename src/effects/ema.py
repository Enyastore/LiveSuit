"""EMA（指数移动平均）效果器范例：1 输入 -> 1 输出。

端口语义：inputs[0] = 原始输入（归一化），outputs[0] = 平滑输出（归一化）。
自带面板：alpha 滑条 + 纵向波形（淡色原始、实色滤波、最新值标注）。
"""

import threading
from collections import deque

import core  # noqa: F401  导入 core 以触发 servo_control 路径注入
from after_process import Filter

from core.effectors import Effector

DEFAULT_ALPHA = 0.85

# 面板配色（与 gui 层保持一致）
_CANVAS_BG = "#ffffff"
_GRID_COLOR = "#e0e0e0"
_AXIS_COLOR = "#555555"
_RAW_COLOR = "#78909c"
_FILTER_COLOR = "#1565c0"
_ARROW_COLOR = "#c62828"


class EMAEffector(Effector):
    """EMA 平滑效果器（有状态；遥测 history 私有，仅面板读取）。"""

    TYPE_NAME = "ema"

    def __init__(self, alpha=DEFAULT_ALPHA, max_history=320):
        self._lock = threading.RLock()
        self._alpha = max(0.0, min(1.0, float(alpha)))
        self._filter = Filter(alpha=self._alpha)
        self._history = deque(maxlen=int(max_history))   # (raw_in, filtered_out)
        self._latest = 0.0
        self._panel = None

    # ---- Effector 接口 ----
    def get_input_count(self):
        return 1

    def get_output_count(self):
        return 1

    def process(self, inputs):
        raw = inputs[0] if inputs else None
        with self._lock:
            _, out = self._filter.update(raw)
            self._history.append((raw, out))
            self._latest = out
        return [out]

    def reset(self):
        with self._lock:
            self._filter = Filter(alpha=self._alpha)
            self._history.clear()
            self._latest = 0.0

    def get_params(self):
        with self._lock:
            return {"alpha": self._alpha}

    def set_params(self, params):
        if params and params.get("alpha") is not None:
            self.set_alpha(params["alpha"])

    def snapshot(self):
        """返回 (history 副本, alpha, latest)，加锁拷贝供面板绘制。"""
        with self._lock:
            return (list(self._history), self._alpha, self._latest)

    # ---- 参数 ----
    def set_alpha(self, alpha):
        """更新 EMA 系数，并用当前历史重放，保证波形即时响应。"""
        alpha = max(0.0, min(1.0, float(alpha)))
        with self._lock:
            self._alpha = alpha
            self._filter = Filter(alpha=alpha)
            seed = None
            for raw, _ in self._history:
                if raw is not None:
                    seed = raw
                    break
            if seed is not None:
                self._filter.set_state(seed)
            filtered = seed if seed is not None else 0.0
            new_history = []
            for raw, _ in self._history:
                _, filtered = self._filter.update(raw)
                new_history.append((raw, filtered))
            self._history = deque(new_history, maxlen=self._history.maxlen)
            self._latest = filtered

    # ---- 自带面板 ----
    def show_panel(self, parent=None):
        """显示/置顶 EMA 面板（alpha 滑条 + 波形）。"""
        if self._panel is not None:
            try:
                if self._panel.winfo_exists():
                    self._panel.deiconify()
                    self._panel.lift()
                    return self._panel
            except Exception:  # noqa: BLE001  面板已销毁
                self._panel = None
        self._panel = _EMAPanel(parent, self)
        return self._panel

    def close_panel(self):
        if self._panel is not None:
            try:
                self._panel.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._panel = None


class _EMAPanel:
    """EMA 效果器自带面板（tkinter Toplevel）。"""

    def __init__(self, parent, effector, width=260, height=140):
        import tkinter as tk
        from tkinter import font as tkfont

        self.effector = effector
        self._width = width
        self._height = height
        self._font = tkfont.Font(size=8)

        self.top = tk.Toplevel(parent) if parent is not None else tk.Tk()
        self.top.title("EMA 效果器")
        self.top.resizable(True, True)

        row = tk.Frame(self.top)
        row.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(row, text="Alpha:").pack(side=tk.LEFT)
        self._alpha_var = tk.DoubleVar(value=effector.get_params()["alpha"])
        tk.Scale(row, from_=0.0, to=1.0, resolution=0.01,
                 orient=tk.HORIZONTAL, variable=self._alpha_var,
                 showvalue=False, command=self._on_alpha,
                 length=140).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self._alpha_label = tk.Label(row, text=f"{self._alpha_var.get():.2f} ")
        self._alpha_label.pack(side=tk.LEFT)

        self.canvas = tk.Canvas(self.top, width=width, height=height,
                                bg=_CANVAS_BG, highlightthickness=1,
                                highlightbackground="#ccc")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 8))
        self._redraw_job = None
        self._schedule_redraw()

    # ---- 面板动作 ----
    def _on_alpha(self, _value):
        a = float(self._alpha_var.get())
        self.effector.set_alpha(a)
        self._alpha_label.config(text=f"{a:.2f} ")
        self.redraw()

    def _schedule_redraw(self):
        try:
            self._redraw_job = self.top.after(50, self._tick_redraw)
        except Exception:  # noqa: BLE001  窗口已销毁
            self._redraw_job = None

    def _tick_redraw(self):
        try:
            self.redraw()
        except Exception:  # noqa: BLE001  关闭竞态
            return
        self._schedule_redraw()

    def redraw(self):
        history, _alpha, latest = self.effector.snapshot()
        c = self.canvas
        c.delete("all")
        w = c.winfo_width() or self._width
        h = c.winfo_height() or self._height
        pad_l, pad_r, pad_t, pad_b = 24, 10, 8, 16
        pw, ph = w - pad_l - pad_r, h - pad_t - pad_b
        if pw <= 0 or ph <= 0:
            return

        # 静态网格（数值 0 / 0.5 / 1）
        for xv in (0.0, 0.5, 1.0):
            x = pad_l + xv * pw
            c.create_line(x, pad_t, x, pad_t + ph, fill=_GRID_COLOR)
            c.create_text(x, pad_t + ph + 4, text=f"{xv:.1f}", anchor="n",
                          fill=_AXIS_COLOR, font=self._font)

        total = max(2, self.effector._history.maxlen or 2)
        n = len(history)
        segments = {"raw": [], "filtered": []}
        for idx, (raw, filt) in enumerate(history):
            frac = (n - 1 - idx) / (total - 1)
            y = pad_t + frac * ph
            if raw is not None:
                segments["raw"].append((pad_l + max(0.0, min(1.0, raw)) * pw, y))
            if filt is not None:
                segments["filtered"].append(
                    (pad_l + max(0.0, min(1.0, filt)) * pw, y))
        for key, color, width in (("raw", _RAW_COLOR, 1),
                                  ("filtered", _FILTER_COLOR, 2)):
            pts = segments[key]
            if len(pts) >= 2:
                c.create_line(*[v for p in pts for v in p],
                              fill=color, width=width)

        if latest is not None:
            x = pad_l + max(0.0, min(1.0, latest)) * pw
            c.create_oval(x - 4, pad_t - 4, x + 4, pad_t + 4,
                          fill=_ARROW_COLOR, outline="")
            c.create_text(x + 10, pad_t + 6, text=f"{latest:.3f}", anchor="w",
                          fill=_ARROW_COLOR, font=self._font)

    def destroy(self):
        if self._redraw_job is not None:
            try:
                self.top.after_cancel(self._redraw_job)
            except Exception:  # noqa: BLE001
                pass
            self._redraw_job = None
        try:
            self.top.destroy()
        except Exception:  # noqa: BLE001
            pass
