"""舵机总线配置面板。

从 gui.py 迁出：对外仅暴露 ServoBusPanel。所需的字体常量在本模块内
自带，避免依赖 gui.py（gui.py 将被重排）。
"""

import tkinter as tk
from tkinter import ttk

SMALL_FONT = ("Arial", 9)
TITLE_FONT = ("Arial", 12, "bold")


class ServoBusPanel(tk.Toplevel):
    """舵机总线配置面板。

    - 输入槽位列：自动检测当前系统所有可用输入槽位（来自 manager 的通道
      注册表），以 Label 文本呈现；
    - 舵机输出列：每个槽位一个下拉框，选项为 servo_configs.yaml 中注册过的
      舵机与「无」；
    - 去重校验：同一舵机通道只能绑定一个槽位；已选舵机自动从其他
      下拉框中移除，取消选择后重新出现（无需弹窗）；
    - 实时绑定：选定后立即建立 slot -> servo 的映射，且槽位输出脉宽范围
      切换为该舵机注册的 [min,max]（µs）。
    """

    def __init__(self, master, manager, on_close=None):
        super().__init__(master)
        self.manager = manager
        self._on_close = on_close
        self.title("舵机总线配置面板")
        self.resizable(False, False)
        self._combos = {}
        self._prev = {}

        title = tk.Label(self, text="舵机总线配置面板", font=TITLE_FONT)
        title.grid(row=0, column=0, columnspan=2, pady=(10, 6))

        tk.Label(self, text="输入槽位", font=SMALL_FONT, fg="#333")\
            .grid(row=1, column=0, padx=14, sticky="w")
        tk.Label(self, text="舵机输出", font=SMALL_FONT, fg="#333")\
            .grid(row=1, column=1, padx=14, sticky="w")

        ttk.Separator(self, orient=tk.HORIZONTAL)\
            .grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=2)

        # 只列出 servo_configs.yaml 中注册过的舵机（只有注册过的才可用）
        options = ["无"] + [f"servo_{i}"
                           for i in self.manager.registered_servo_indices()]
        row = 3
        for channel in self.manager.get_channels():
            name = channel.name
            tk.Label(self, text=name, font=("Courier", 10, "bold"))\
                .grid(row=row, column=0, sticky="w", padx=16, pady=2)
            combo = ttk.Combobox(self, values=options, state="readonly", width=12)
            cur = self.manager.get_binding(name)
            combo.set(f"servo_{cur}" if cur is not None else "无")
            combo.grid(row=row, column=1, padx=14, pady=2, sticky="w")
            combo.bind("<<ComboboxSelected>>",
                       lambda _e, n=name: self._on_select(n))
            self._combos[name] = combo
            self._prev[name] = combo.get()
            row += 1
        self._rebuild_options()   # 按已有绑定过滤各下拉框选项

        self.protocol("WM_DELETE_WINDOW", self._handle_close)

    def _handle_close(self):
        """用户关闭本面板：通知 LiveSuitApp 置空引用后销毁。"""
        if self._on_close is not None:
            self._on_close(self)
        self.destroy()

    def _on_select(self, name):
        combo = self._combos[name]
        val = combo.get()
        idx = None if val == "无" else int(val.split("_")[1])
        ok, _msg = self.manager.bind(name, idx)
        if not ok:
            # 防御：正常不会发生（可用选项已排除已选舵机），失败则回退
            combo.set(self._prev[name])
            return
        self._prev[name] = val

    def refresh_bindings(self):
        """外部绑定变化时同步下拉框显示与可用选项（由 Store 订阅触发）。"""
        self._rebuild_options()
        for name, combo in self._combos.items():
            cur = self.manager.get_binding(name)
            combo.set(f"servo_{cur}" if cur is not None else "无")
            self._prev[name] = combo.get()

    def _rebuild_options(self):
        """按当前绑定关系重建所有下拉框可用选项。

        已选中的舵机从其他槽位的下拉框中移除（用户改绑后自动释放），
        自身绑定的舵机保留在当前槽位的下拉框内，保证显示值始终有效。
        """
        used = set()
        for name in self._combos:
            cur = self.manager.get_binding(name)
            if cur is not None:
                used.add(cur)
        registered = set(self.manager.registered_servo_indices())
        for name, combo in self._combos.items():
            own = self.manager.get_binding(name)
            opts = ["无"] + [f"servo_{i}" for i in sorted(registered)
                             if i not in used or i == own]
            combo["values"] = opts
