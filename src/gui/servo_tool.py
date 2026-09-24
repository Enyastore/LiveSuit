"""舵机调试工具窗口（脉宽标定）。

从 gui.py 迁出：对外仅暴露 ServoToolWindow。依赖 app.servo_debug 与
app.servo，所需字体/配色常量在本模块内自带，避免依赖 gui.py。
"""

import tkinter as tk
from tkinter import ttk

DANGER_COLOR = "#b71c1c"      # 深红：调试脉宽超出配置范围的警示色
SMALL_FONT = ("Arial", 9)
TITLE_FONT = ("Arial", 12, "bold")


class ServoToolWindow(tk.Toplevel):
    """舵机调试工具窗口（脉宽标定）。

    用于启动 LiveSuit 之前手动调试单个舵机通道：
      - 标题栏：舵机工具
      - 第一行：舵机输出 标签 + 下拉框（仅列 servo_configs.yaml 注册舵机）
      - 第二行：[-] 按钮 + 脉宽数字输入框（µs）+ [+] 按钮
      - 第三行：状态提示（配置范围 / 安全范围 / 越界警示）

    行为约定：
      - 下拉选择通道：仅切换并显示该通道当前脉宽，不下发（防误触发）；
      - 点击 +/- 或回车输入数值：立即下发到当前选中通道；
      - 允许设置 servo_configs.yaml 推荐范围之外的脉宽（便于实测找最佳值
        写回配置文件），但钳制到该通道的安全提示范围；
      - 当数值超出该通道配置范围时，输入框与提示变为深红色并显示舵机
        损坏风险警示；每次 +/- 步长 1µs。
    """

    def __init__(self, master, app, on_close=None):
        super().__init__(master)
        self.app = app
        self._on_close = on_close
        self.debugger = app.servo_debug
        self.title("舵机工具")
        self.resizable(False, False)

        title = tk.Label(self, text="舵机工具", font=TITLE_FONT)
        title.pack(pady=(10, 6))

        # 第一行：舵机输出 标签 + 通道下拉框（仅注册舵机）
        row1 = tk.Frame(self)
        row1.pack(padx=12, pady=(2, 4))
        tk.Label(row1, text="舵机输出", font=SMALL_FONT).pack(side=tk.LEFT,
                                                              padx=(0, 8))
        self._channels = [f"servo_{i}" for i in self.debugger.registered_indices()]
        self._channel_var = tk.StringVar(value=self._channels[0])
        self._combo = ttk.Combobox(row1, textvariable=self._channel_var,
                                   values=self._channels, state="readonly",
                                   width=14)
        self._combo.pack(side=tk.LEFT)
        self._combo.bind("<<ComboboxSelected>>", self._on_channel_select)

        # 第二行：[-] 按钮 + 脉宽输入框（µs）+ [+] 按钮
        row2 = tk.Frame(self)
        row2.pack(padx=12, pady=(4, 10))
        tk.Button(row2, text="-", width=4, command=lambda: self._adjust(-1))\
            .pack(side=tk.LEFT, padx=6)
        self._pulse_var = tk.StringVar(value=self._fmt(self.debugger.get_pulse()))
        self._pulse_entry = tk.Entry(row2, textvariable=self._pulse_var,
                                     width=12, justify=tk.CENTER)
        self._pulse_entry.pack(side=tk.LEFT, padx=6)
        self._pulse_entry.bind("<Return>", self._on_entry_commit)
        tk.Label(row2, text="µs", font=SMALL_FONT).pack(side=tk.LEFT, padx=(0, 6))
        tk.Button(row2, text="+", width=4, command=lambda: self._adjust(1))\
            .pack(side=tk.LEFT, padx=6)

        # 第三行：状态提示（配置范围 / 安全范围 / 越界警示）
        self._status_label = tk.Label(self, text="", font=SMALL_FONT,
                                      fg="#666", wraplength=280, justify=tk.CENTER)
        self._status_label.pack(padx=12, pady=(0, 10))

        self.protocol("WM_DELETE_WINDOW", self._handle_close)
        self._update_status()

    @staticmethod
    def _fmt(pulse):
        """脉宽显示格式：整数不带小数（µs，如 1500 / 1450.5）。"""
        return f"{pulse:g}"

    def _update_status(self):
        """根据当前脉宽相对配置范围的状态刷新输入框颜色与警示标签。

        在推荐范围内：正常黑色 + 提示「配置范围 / 安全范围」；
        超出推荐范围：输入框与提示变深红 + 舵机损坏风险警示。
        """
        out = self.debugger.is_out_of_range()
        lo, hi = self.debugger.config_range()
        safe_lo, safe_hi = self.debugger.safe_range()
        if out:
            self._pulse_entry.config(fg=DANGER_COLOR)
            self._status_label.config(
                fg=DANGER_COLOR,
                text=(f"⚠ 超出配置范围 {lo:.0f}~{hi:.0f}µs，"
                      f"可能有舵机损坏风险（安全提示范围 {safe_lo:.0f}~"
                      f"{safe_hi:.0f}µs）"))
        else:
            self._pulse_entry.config(fg="#000000")
            self._status_label.config(
                fg="#666",
                text=(f"配置范围: {lo:.0f}~{hi:.0f}µs · "
                      f"安全提示范围: {safe_lo:.0f}~{safe_hi:.0f}µs"))

    def _on_channel_select(self, _event=None):
        """下拉切换通道：仅切换，不下发，数字框显示该通道当前脉宽。"""
        idx = int(self._channel_var.get().split("_")[1])
        pulse = self.debugger.select_channel(idx)
        self._pulse_var.set(self._fmt(pulse))
        self._update_status()

    def _adjust(self, delta):
        """点击 +/-：以 1µs 步长增减当前选中通道脉宽并立即下发。"""
        pulse = self.debugger.adjust(delta)
        self._pulse_var.set(self._fmt(pulse))
        self._output()
        self._update_status()

    @staticmethod
    def _parse_pulse(text):
        """解析数字框文本，允许 µs / us / 微秒 后缀；非法输入返回 None。"""
        text = text.strip()
        for suffix in ("µs", "us", "微秒", "μs"):
            if text.endswith(suffix):
                text = text[:-len(suffix)].strip()
                break
        try:
            return float(text)
        except ValueError:
            return None

    def _on_entry_commit(self, _event=None):
        """回车提交输入框数值：立即下发；非法输入恢复显示、不下发。"""
        value = self._parse_pulse(self._pulse_var.get())
        if value is None:
            self._pulse_var.set(self._fmt(self.debugger.get_pulse()))
            self._update_status()
            return
        pulse = self.debugger.set_pulse(value)
        self._pulse_var.set(self._fmt(pulse))
        self._output()
        self._update_status()

    def _output(self):
        """把当前调试状态（16 路完整脉宽向量）立即下发到舵机硬件。"""
        if self.app.servo is None:
            return
        try:
            self.app.servo.set_pulse(self.debugger.get_vector())
        except Exception as exc:  # noqa: BLE001  硬件下发异常不阻塞 UI
            print(f"[gui] 舵机调试下发失败（{exc}）")

    def _handle_close(self):
        """用户关闭本窗口：通知 LiveSuitApp 置空引用后销毁。"""
        if self._on_close is not None:
            self._on_close(self)
        self.destroy()
