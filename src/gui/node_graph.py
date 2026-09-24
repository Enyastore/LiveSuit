"""效果器编排节点图窗口（tkinter）。

以三列节点展示 参数 -> 效果器 -> 舵机通道 的信息流：
  - 左列参数节点只有输出口；中列效果器有若干输入/输出口；右列通道只有输入口。
  - 点击输出口选中（高亮），再点击输入口接线；右键输入口断开。
  - 双击效果器节点打开其自带面板；右键效果器节点可删除。

编排状态一律以 PipelineManager 为准，本窗口只负责展示与交互，不缓存业务状态；
定时轻量重绘以反映外部（如载入配置）造成的接线变化。
"""

import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox

from core.graph import GraphError
from effects import EFFECTOR_TYPES

# 固定三列布局：参数 -> 效果器 -> 通道（简单纵向下标排布，不做拖拽）
_COL_X = (20, 230, 440)
_NODE_W = 170
_HEADER_H = 36
_PORT_SPACING = 22
_NODE_PAD = 12
_NODE_GAP = 24
_TOP_MARGIN = 20
_PORT_RADIUS = 4
_HIT_RADIUS = 8

_CANVAS_BG = "#f5f5f5"
_PARAM_BG = "#fff8e1"
_EFFECTOR_BG = "#e3f2fd"
_CHANNEL_BG = "#e8f5e9"
_NODE_BORDER = "#90a4ae"
_TEXT_COLOR = "#263238"
_SUBTLE_TEXT = "#607d8b"
_WIRE_COLOR = "#546e7a"
_SELECTED_COLOR = "#e53935"
_PORT_IN_COLOR = "#1565c0"
_PORT_OUT_COLOR = "#2e7d32"


class NodeGraphWindow(tk.Toplevel):
    """效果器编排节点图窗口。"""

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.title("效果器编排")
        self.geometry("680x560")

        # 交互状态：当前选中的输出口 source_ref（None 表示未选中）
        self._selected_source = None
        # 命中检测数据：端口圆圈坐标 + 归属，节点包围盒
        self._ports = []
        self._nodes = []
        self._redraw_job = None

        self._title_font = tkfont.Font(size=9, weight="bold")
        self._sub_font = tkfont.Font(size=8)

        self._build_toolbar()
        self._build_canvas()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.redraw()
        self._schedule_redraw()

    # --------------------------------------------------------
    # 便捷访问
    # --------------------------------------------------------
    @property
    def manager(self):
        """实时从 app 取编排层，避免持有过期引用。"""
        app = getattr(self, "app", None)
        return getattr(app, "manager", None)

    # --------------------------------------------------------
    # 界面搭建
    # --------------------------------------------------------
    def _build_toolbar(self):
        bar = tk.Frame(self)
        bar.pack(side=tk.TOP, fill=tk.X)
        self._add_button = tk.Button(bar, text="新增效果器",
                                     command=self._show_add_menu)
        self._add_button.pack(side=tk.LEFT, padx=6, pady=4)
        tk.Label(bar, text="点输出口再点输入口接线；右键输入口断开，"
                           "右键效果器节点删除").pack(side=tk.LEFT, padx=8)

    def _build_canvas(self):
        container = tk.Frame(self)
        container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(container, bg=_CANVAS_BG,
                                highlightthickness=0)
        vbar = tk.Scrollbar(container, orient=tk.VERTICAL,
                            command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=vbar.set)
        vbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self._on_left_click)
        self.canvas.bind("<Double-Button-1>", self._on_double_click)
        self.canvas.bind("<Button-3>", self._on_right_click)

    # --------------------------------------------------------
    # 重绘
    # --------------------------------------------------------
    def redraw(self):
        """按当前编排全量重绘节点图（主线程调用）。"""
        manager = self.manager
        if manager is None:
            return
        try:
            self.canvas.delete("all")
        except tk.TclError:
            return
        self._ports = []
        self._nodes = []

        try:
            params = manager.get_params()
            effectors = manager.get_effectors()
            channels = manager.get_channels()
            connections = manager.get_connections()
        except Exception as exc:  # noqa: BLE001  外部状态异常时不清空后崩溃
            print(f"[node_graph] 读取编排状态失败（{exc}）")
            return

        out_pos = {}
        in_pos = {}
        max_bottom = _TOP_MARGIN
        col_bottom = [_TOP_MARGIN, _TOP_MARGIN, _TOP_MARGIN]

        # 左列：参数节点（1 个输出口）
        y = _TOP_MARGIN
        for param in params:
            h = self._node_height(0, 1)
            self._draw_node(_COL_X[0], y, h, _PARAM_BG,
                            [param.label, param.name], "param", param)
            px, py = _COL_X[0] + _NODE_W, self._port_y(y, 0)
            self._draw_port(px, py, "output", ("param", param.name), 0)
            out_pos[("param", param.name)] = (px, py)
            y += h + _NODE_GAP
        col_bottom[0] = y

        # 中列：效果器节点（若干输入/输出口）
        y = _TOP_MARGIN
        for ref in effectors:
            name = manager.effector_name(ref)
            in_count = ref.get_input_count()
            out_count = ref.get_output_count()
            h = self._node_height(in_count, out_count)
            type_name = getattr(ref, "TYPE_NAME", None) or "?"
            self._draw_node(_COL_X[1], y, h, _EFFECTOR_BG,
                            [str(name), type_name], "effector", ref)
            for port in range(in_count):
                px, py = _COL_X[1], self._port_y(y, port)
                self._draw_port(px, py, "input",
                                ("effector", name, port), port, target=ref)
                in_pos[("effector", name, port)] = (px, py)
            for port in range(out_count):
                px, py = _COL_X[1] + _NODE_W, self._port_y(y, port)
                self._draw_port(px, py, "output",
                                ("effector", name, port), port)
                out_pos[("effector", name, port)] = (px, py)
            y += h + _NODE_GAP
        col_bottom[1] = y

        # 右列：通道节点（1 个输入口）
        y = _TOP_MARGIN
        for channel in channels:
            binding = manager.get_binding(channel.name)
            info = f"servo_{binding}" if binding is not None else "未绑定"
            h = self._node_height(1, 0)
            self._draw_node(_COL_X[2], y, h, _CHANNEL_BG,
                            [channel.name, info], "channel", channel)
            px, py = _COL_X[2], self._port_y(y, 0)
            self._draw_port(px, py, "input",
                            ("channel", channel.name, 0), 0, target=channel)
            in_pos[("channel", channel.name, 0)] = (px, py)
            y += h + _NODE_GAP
        col_bottom[2] = y

        # 连线（源 -> 目标）
        for src_ref, dst_ref in connections:
            p0 = out_pos.get(tuple(src_ref))
            p1 = in_pos.get(tuple(dst_ref))
            if p0 is not None and p1 is not None:
                self._draw_wire(p0, p1)

        max_bottom = max(col_bottom)
        width = _COL_X[2] + _NODE_W + 20
        height = max_bottom + 20
        self.canvas.configure(scrollregion=(0, 0, width, height))

    def _node_height(self, in_count, out_count):
        rows = max(in_count, out_count, 1)
        return _HEADER_H + rows * _PORT_SPACING + _NODE_PAD

    @staticmethod
    def _port_y(node_y, index):
        return node_y + _HEADER_H + _PORT_SPACING * (index + 0.5)

    def _draw_node(self, x, y, h, bg, lines, kind, ref):
        c = self.canvas
        c.create_rectangle(x, y, x + _NODE_W, y + h,
                           fill=bg, outline=_NODE_BORDER, width=1)
        c.create_text(x + 8, y + 6, text=lines[0], anchor="nw",
                      fill=_TEXT_COLOR, font=self._title_font)
        if len(lines) > 1:
            c.create_text(x + 8, y + 21, text=lines[1], anchor="nw",
                          fill=_SUBTLE_TEXT, font=self._sub_font)
        self._nodes.append({"box": (x, y, x + _NODE_W, y + h),
                            "kind": kind, "ref": ref})

    def _draw_port(self, x, y, direction, ref, port, target=None):
        c = self.canvas
        selected = direction == "output" and self._selected_source == tuple(ref)
        color = _SELECTED_COLOR if selected else (
            _PORT_OUT_COLOR if direction == "output" else _PORT_IN_COLOR)
        r = _PORT_RADIUS + (2 if selected else 0)
        c.create_oval(x - r, y - r, x + r, y + r,
                      fill=color, outline="#ffffff", width=1)
        self._ports.append({"x": x, "y": y, "direction": direction,
                            "ref": tuple(ref), "port": port, "target": target})

    def _draw_wire(self, p0, p1):
        x0, y0 = p0
        x1, y1 = p1
        dx = max(30.0, abs(x1 - x0) * 0.4)
        self.canvas.create_line(x0, y0, x0 + dx, y0, x1 - dx, y1, x1, y1,
                                smooth=True, splinesteps=24,
                                fill=_WIRE_COLOR, width=2)

    # --------------------------------------------------------
    # 事件处理
    # --------------------------------------------------------
    def _canvas_pos(self, event):
        return (self.canvas.canvasx(event.x), self.canvas.canvasy(event.y))

    def _hit_port(self, x, y):
        best = None
        best_dist = _HIT_RADIUS
        for port in self._ports:
            dist = ((port["x"] - x) ** 2 + (port["y"] - y) ** 2) ** 0.5
            if dist <= best_dist:
                best_dist = dist
                best = port
        return best

    def _hit_node(self, x, y):
        for node in self._nodes:
            x0, y0, x1, y1 = node["box"]
            if x0 <= x <= x1 and y0 <= y <= y1:
                return node
        return None

    def _on_left_click(self, event):
        x, y = self._canvas_pos(event)
        port = self._hit_port(x, y)
        if port is None:
            return
        if port["direction"] == "output":
            self._selected_source = port["ref"]
            self.redraw()
            return
        if self._selected_source is None:
            return
        self._connect(self._selected_source, port)
        self._selected_source = None
        self.redraw()

    def _on_double_click(self, event):
        x, y = self._canvas_pos(event)
        node = self._hit_node(x, y)
        if node is None or node["kind"] != "effector":
            return
        try:
            node["ref"].show_panel(parent=self)
        except Exception as exc:  # noqa: BLE001  面板实现失败不应崩溃
            messagebox.showerror("打开面板失败", str(exc), parent=self)

    def _on_right_click(self, event):
        x, y = self._canvas_pos(event)
        port = self._hit_port(x, y)
        if port is not None and port["direction"] == "input":
            self._disconnect(port)
            return
        node = self._hit_node(x, y)
        if node is not None and node["kind"] == "effector":
            self._show_node_menu(event, node)

    # --------------------------------------------------------
    # 编排操作
    # --------------------------------------------------------
    def _show_add_menu(self):
        menu = tk.Menu(self, tearoff=0)
        types = list(EFFECTOR_TYPES.keys())
        if not types:
            menu.add_command(label="（无可用效果器）", state=tk.DISABLED)
        for type_name in types:
            menu.add_command(
                label=type_name,
                command=lambda t=type_name: self._on_add_effector(t))
        try:
            menu.tk_popup(self._add_button.winfo_rootx(),
                          self._add_button.winfo_rooty()
                          + self._add_button.winfo_height())
        finally:
            menu.grab_release()

    def _on_add_effector(self, type_name):
        try:
            self.manager.create_effector(type_name)
        except Exception as exc:  # noqa: BLE001  类型非法/实例化失败时提示
            messagebox.showerror("新增效果器失败", str(exc), parent=self)
            return
        self.redraw()

    def _connect(self, src_ref, dst_port):
        """把选中的输出口接到目标输入口；形成环等错误弹框提示。"""
        try:
            self.manager.connect(tuple(src_ref), 0,
                                 dst_port["target"], dst_port["port"])
        except GraphError as exc:
            messagebox.showerror("接线失败", str(exc), parent=self)
        except Exception as exc:  # noqa: BLE001  端口非法等
            messagebox.showerror("接线失败", str(exc), parent=self)

    def _disconnect(self, port):
        try:
            self.manager.disconnect(port["target"], port["port"])
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("断开失败", str(exc), parent=self)
        self.redraw()

    def _show_node_menu(self, event, node):
        menu = tk.Menu(self, tearoff=0)
        menu.add_command(
            label="删除效果器",
            command=lambda: self._on_remove_effector(node["ref"]))
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()

    def _on_remove_effector(self, ref):
        self.manager.remove_effector(ref)
        self._selected_source = None
        self.redraw()

    # --------------------------------------------------------
    # 定时轻量重绘 / 关闭
    # --------------------------------------------------------
    def _schedule_redraw(self):
        try:
            self._redraw_job = self.after(200, self._tick_redraw)
        except tk.TclError:
            self._redraw_job = None

    def _tick_redraw(self):
        self._redraw_job = None
        try:
            if not self.winfo_exists():
                return
            self.redraw()
        except tk.TclError:
            return
        self._schedule_redraw()

    def _on_close(self):
        if self._redraw_job is not None:
            try:
                self.after_cancel(self._redraw_job)
            except tk.TclError:
                pass
            self._redraw_job = None
        self.destroy()
