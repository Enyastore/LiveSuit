"""欢迎页。

从 gui.py 迁出：对外暴露 WelcomePage 及其 Logo 辅助函数 _load_logo。
所需的字体常量与 Logo 路径在本模块内自带，避免依赖 gui.py。
"""

import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path

BODY_FONT = ("Arial", 10)
SMALL_FONT = ("Arial", 9)

# 仓库根目录下的 Logo（本文件位于 src/gui/，故向上三层到仓库根）
_LOGO_PATH = Path(__file__).resolve().parent.parent.parent / "LiveSuitLogo.png"


def _load_logo(max_width=None):
    """加载 LiveSuitLogo.png，返回 PhotoImage；缺图时返回 None。"""
    try:
        from PIL import Image, ImageTk
        img = Image.open(str(_LOGO_PATH))
        if max_width is not None and img.width > max_width:
            ratio = max_width / img.width
            img = img.resize((max_width, int(img.height * ratio)), Image.LANCZOS)
        return ImageTk.PhotoImage(img)
    except Exception:  # noqa: BLE001  缺图 / 缺 PIL 时降级为文字
        return None


class WelcomePage(tk.Frame):
    """启动后的欢迎页面。

    - 项目 Logo 与标题、版本、简介
    - 【启动LiveSuit】按钮：点击后变为【停止】，用于关闭一切进程与窗口
    - 【切换窗口渲染】按钮：启动前禁用，启动后在 debug / headless 间切换
    """

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app

        self.logo_img = _load_logo(max_width=240)
        if self.logo_img is not None:
            tk.Label(self, image=self.logo_img).pack(pady=(24, 8))

        tk.Label(self, text="版本: beta1.0", font=BODY_FONT, fg="#444").pack()
        intro_label = tk.Label(
            self, text="简介: LiveSuit是一个个人可动兽装项目。采用MIT许可证。",
            font=BODY_FONT, fg="#444")
        intro_label.pack(pady=(4, 18))

        # 数据提供者信息（来源 slots.PROVIDER_INFO，纯文本常量，无硬件依赖——
        # slots 已将 cv2 / eye_tracker 依赖栈延迟到提供者实例化时才导入）。
        # wraplength 取简介标签文本的像素宽度，多行换行后总宽度不超过简介。
        try:
            from slots import PROVIDER_INFO  # noqa: PLC0415  轻量导入，仅读取纯文本常量
        except Exception:  # noqa: BLE001  极端环境兜底，不阻塞欢迎页
            PROVIDER_INFO = ""
        wrap_width = tkfont.Font(font=BODY_FONT).measure(intro_label.cget("text"))
        tk.Label(self, text=PROVIDER_INFO, font=SMALL_FONT, fg="#444",
                 wraplength=wrap_width, justify=tk.CENTER).pack(pady=(0, 18))

        btn_row = tk.Frame(self)
        btn_row.pack(pady=(0, 24))
        # 左列「启动LiveSuit」按钮纵向占两行（与右侧两按钮对齐）
        self.start_btn = tk.Button(btn_row, text="启动LiveSuit", width=16,
                                   command=self.app.start)
        self.start_btn.grid(row=0, column=0, rowspan=2, sticky="ns", padx=10)
        # 右列上方「切换窗口渲染」、下方「打开舵机工具」
        self.toggle_btn = tk.Button(btn_row, text="切换窗口渲染", width=16,
                                    command=self.app.toggle_render,
                                    state=tk.DISABLED)   # 启动前禁用
        self.toggle_btn.grid(row=0, column=1, padx=10, pady=(0, 3))
        self.servo_tool_btn = tk.Button(btn_row, text="打开舵机工具", width=16,
                                        command=self.app.open_servo_tool)
        self.servo_tool_btn.grid(row=1, column=1, padx=10, pady=(3, 0))
        # 右列再一组：「效果器编排」（启动后可用）
        self.node_graph_btn = tk.Button(btn_row, text="效果器编排", width=16,
                                        command=self.app.open_node_graph,
                                        state=tk.DISABLED)
        self.node_graph_btn.grid(row=0, column=2, rowspan=2, sticky="ns",
                                 padx=10)

        self.render_label = tk.Label(self, text="", font=SMALL_FONT, fg="#666")
        self.render_label.pack(pady=(0, 12))
        self.update_render_label()

    def on_started(self):
        """启动后：启动按钮变为停止按钮，启用渲染切换与效果器编排，停用舵机工具。"""
        self.start_btn.config(text="停止", command=self.app.stop)
        self.toggle_btn.config(state=tk.NORMAL)
        self.node_graph_btn.config(state=tk.NORMAL)
        self.servo_tool_btn.config(state=tk.DISABLED)   # 启动后禁止再打开舵机工具

    def update_render_label(self, mode=None):
        # 启动前 manager 为 None，render_mode 只能从 Store 读取（Store 恒存在）
        mode = mode if mode is not None else self.app.store.get("render_mode")
        text = ("当前渲染: 调试模式（显示所有子面板）"
                if mode == "debug" else
                "当前渲染: 无头模式（隐藏并停止渲染所有子面板）")
        self.render_label.config(text=text)
