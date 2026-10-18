"""1D 生成器效果器：0 输入 -> 1 输出。

端口语义：outputs[0] = 面板滑条当前值（归一化）。
自带面板：一个 0~1 的横向滑条，步长 0.1，直接把滑条值作为输出。
"""

import threading

from core.effectors import Effector

DEFAULT_VALUE = 0.0
SLIDER_RESOLUTION = 0.01


class OneDGeneratorEffector(Effector):
    """手动生成常量值的效果器（无输入；滑条值由面板在主线程设置）。"""

    TYPE_NAME = "1D-Generator"

    def __init__(self, value=DEFAULT_VALUE):
        self._lock = threading.RLock()
        self._value = self._clamp(value)
        self._panel = None

    # ---- Effector 接口 ----
    def get_input_count(self):
        return 0

    def get_output_count(self):
        return 1

    def process(self, inputs):
        with self._lock:
            return [self._value]

    def reset(self):
        with self._lock:
            self._value = DEFAULT_VALUE

    def get_params(self):
        with self._lock:
            return {"value": self._value}

    def set_params(self, params):
        if params and params.get("value") is not None:
            self.set_value(params["value"])

    def snapshot(self):
        """返回当前滑条值，加锁供面板读取。"""
        with self._lock:
            return self._value

    # ---- 参数 ----
    def set_value(self, value):
        with self._lock:
            self._value = self._clamp(value)

    @staticmethod
    def _clamp(value):
        return max(0.0, min(1.0, float(value)))

    # ---- 自带面板 ----
    def show_panel(self, parent=None):
        """显示/置顶 1D 生成器面板（0~1 滑条）。"""
        if self._panel is not None:
            try:
                if self._panel.winfo_exists():
                    self._panel.deiconify()
                    self._panel.lift()
                    return self._panel
            except Exception:  # noqa: BLE001  面板已销毁
                self._panel = None
        self._panel = _OneDGeneratorPanel(parent, self)
        return self._panel

    def close_panel(self):
        if self._panel is not None:
            try:
                self._panel.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._panel = None


class _OneDGeneratorPanel:
    """1D 生成器效果器自带面板（tkinter Toplevel）。"""

    def __init__(self, parent, effector, width=260):
        import tkinter as tk

        self.effector = effector

        self.top = tk.Toplevel(parent) if parent is not None else tk.Tk()
        self.top.title(effector.label or "1D 生成器")
        self.top.resizable(False, False)

        row = tk.Frame(self.top)
        row.pack(fill=tk.X, padx=8, pady=8)
        tk.Label(row, text="值:").pack(side=tk.LEFT)
        self._value_var = tk.DoubleVar(value=effector.snapshot())
        tk.Scale(row, from_=0.0, to=1.0, resolution=SLIDER_RESOLUTION,
                 orient=tk.HORIZONTAL, variable=self._value_var,
                 showvalue=False, command=self._on_value,
                 length=width).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self._value_label = tk.Label(row, text=f"{self._value_var.get():.2f} ")
        self._value_label.pack(side=tk.LEFT)

    # ---- 面板动作 ----
    def _on_value(self, _value):
        v = float(self._value_var.get())
        self.effector.set_value(v)
        self._value_label.config(text=f"{v:.2f} ")

    def destroy(self):
        try:
            self.top.destroy()
        except Exception:  # noqa: BLE001
            pass
