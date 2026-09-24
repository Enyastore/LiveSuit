"""单个舵机通道窗口：样条编辑 + 舵机绑定。

与旧 SlotControllerWindow 的区别：EMA 相关的 alpha 滑条与波形已迁到
效果器自带面板（EMAEffector.show_panel）；本窗口只负责样条与绑定。
"""

import tkinter as tk
from tkinter import ttk

from gui.spline_canvas import SplineCurveCanvas

TITLE_FONT = ("TkDefaultFont", 11, "bold")
TINY_FONT = ("TkDefaultFont", 8)
SMALL_FONT = ("TkDefaultFont", 9)
VALUE_FONT = ("TkDefaultFont", 10, "bold")


class ChannelWindow(tk.Toplevel):
    """舵机通道控制器：样条曲线编辑 + 绑定状态/下拉。"""

    def __init__(self, master, manager, channel, on_close=None):
        super().__init__(master)
        self.manager = manager
        self.channel = channel
        self._on_close = on_close
        self.title(f"LiveSuit · {channel.name}")
        self.minsize(220, 280)

        tk.Label(self, text=channel.name, font=TITLE_FONT).pack(pady=(6, 2))
        self.spline_title = tk.Label(self, text="样条曲线（归一化 0~1）",
                                     font=TINY_FONT, fg="#333")
        self.spline_title.pack(anchor="w", padx=8)
        tk.Label(self, text="（左键单击曲线/双击空白处新建点；双击点编辑；右键点删除）",
                 font=TINY_FONT, fg="#333").pack(anchor="w", padx=8)

        self.spline = SplineCurveCanvas(self, channel,
                                        on_changed=self._on_spline_changed)
        self.spline.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 4))

        row = tk.Frame(self)
        row.pack(fill=tk.X, padx=8, pady=(0, 4))
        tk.Label(row, text="绑定舵机:").pack(side=tk.LEFT)
        self._binding_var = tk.StringVar()
        self._combo = ttk.Combobox(row, textvariable=self._binding_var,
                                   state="readonly", width=12)
        self._combo.pack(side=tk.LEFT, padx=6)
        self._combo.bind("<<ComboboxSelected>>", self._on_bind)

        self.binding_label = tk.Label(self, text="", font=SMALL_FONT, fg="#666")
        self.binding_label.pack(pady=(0, 8))

        self._last_out_range = channel.out_range
        self._last_bound = None
        self.protocol("WM_DELETE_WINDOW", self._handle_close)

        self._rebuild_options()
        self.refresh()
        self.spline.draw()

    # ---- 生命周期 ----
    def _handle_close(self):
        if self._on_close is not None:
            self._on_close(self)
        self.destroy()

    def _on_spline_changed(self):
        self.spline.redraw_indicator()

    # ---- 绑定 ----
    def _rebuild_options(self):
        """重建下拉选项：已注册舵机，已被其它通道占用的除外（自身保留）。"""
        used = set()
        for ch in self.manager.get_channels():
            idx = self.manager.get_binding(ch.name)
            if idx is not None:
                used.add(idx)
        own = self.manager.get_binding(self.channel.name)
        registered = self.manager.registered_servo_indices()
        opts = ["无"] + [f"servo_{i}" for i in sorted(registered)
                         if i not in used or i == own]
        self._combo["values"] = opts

    def _on_bind(self, _event=None):
        value = self._binding_var.get()
        idx = None if value == "无" else int(value.split("_")[1])
        ok, msg = self.manager.bind(self.channel.name, idx)
        if not ok:
            from tkinter import messagebox
            messagebox.showwarning("绑定失败", msg, parent=self)
        self.refresh()

    # ---- 刷新 ----
    def refresh(self):
        """同步样条标题、绑定显示与指示器（可从任意线程后的主线程调用）。"""
        if self.channel.out_range != self._last_out_range:
            self._last_out_range = self.channel.out_range
            self.spline.sync_from_channel()

        idx = self.manager.get_binding(self.channel.name)
        bound = idx is not None
        if bound != self._last_bound:
            self._last_bound = bound
            self.spline_title.config(
                text=("样条曲线（脉宽 µs）" if bound
                      else "样条曲线（归一化 0~1）"))
        if bound and self._combo.get() != f"servo_{idx}":
            self._rebuild_options()
            self._binding_var.set(f"servo_{idx}")
        elif not bound and self._combo.get() != "无":
            self._rebuild_options()
            self._binding_var.set("无")

        limits = self.manager.servo_limits.get(idx) if bound else None
        text = (f"已绑定舵机: servo_{idx}（{limits.min_pulse:.0f}~"
                f"{limits.max_pulse:.0f}µs）" if limits is not None
                else "未绑定舵机（输出归一化 0~1）")
        if self.binding_label.cget("text") != text:
            self.binding_label.config(text=text)

        snap = self.manager.snapshot()
        norm = snap["channel_inputs"].get(self.channel.name)
        self.spline.set_indicator(norm)
        self.spline.redraw_indicator()

    def redraw(self):
        """每帧轻量刷新（由主应用重绘循环调用）。"""
        self.refresh()
