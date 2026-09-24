"""样条编辑画布 + 样本点编辑弹窗。

从 gui.py 迁移而来，绑定对象由旧的 Slot 换成 core.channels.ServoChannel：
  - SplineCurveCanvas   三次样条插值调节画布（单击新增 / 拖拽 / 双击编辑 / 右键删除）
  - EditPointDialog     双击样本点的 X/Y 编辑模态框

本模块自带绘制所需的配色与字体常量，不依赖 gui.py。
"""

import math
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox

# 简单 tkinter 风格下的图表配色（默认浅底），与 gui.py 保持一致
CANVAS_BG = "#ffffff"
GRID_COLOR = "#e0e0e0"
AXIS_COLOR = "#555555"
SPLINE_COLOR = "#1a73e8"      # 三次样条曲线（蓝）
POINT_COLOR = "#e65100"       # 样本点（橙）
ARROW_COLOR = "#c62828"       # 指示箭头（红）

# 字体：沿用 gui.py 的取值，保证绘制观感一致
TINY_FONT = ("Arial", 8)
BODY_FONT = ("Arial", 10)
VALUE_FONT = ("Courier", 11, "bold")


class SplineCurveCanvas(tk.Canvas):
    """三次样条插值调节画布（任务管理器风格曲线图）。

    交互:
      - 左键单击空白区域 -> 新增样本点
      - 左键拖拽样本点    -> 移动样本点并实时刷新样条曲线
      - 左键双击样本点    -> 弹出 X(0~1) / Y(MIN~MAX) 编辑弹窗
      - 右键单击样本点    -> 删除样本点（x=0 与 x=1 边界点不可删除）

    X 轴为归一化输入 0~1；Y 轴为通道当前输出范围：未绑定舵机时为归一化
    0~1，绑定某路舵机后为该舵机在 servo_configs.yaml 中注册的真实脉宽范围
    （µs）。顶部 X 轴上绘制当前滤波值的映射指示（与下方 EMA 图箭头对齐）。
    """

    PAD_L = 30
    PAD_R = 10
    PAD_T = 10
    PAD_B = 16
    HIT_R = 7     # 命中半径（px）

    def __init__(self, master, channel, width=232, height=132, on_changed=None):
        super().__init__(master, width=width, height=height,
                         bg=CANVAS_BG, highlightthickness=1,
                         highlightbackground="#ccc")
        self.channel = channel
        self.on_changed = on_changed
        self._cw = width
        self._ch = height
        self.points = [list(p) for p in channel.curve_points()]  # 本地可编辑样本点（µs）
        self.indicator_x = None                        # 当前滤波值在 X 轴上的位置
        self._drag_index = None
        self._pending_add = None
        # 增量渲染状态：静态层 / 曲线 / 样本点只在变化时重建，
        # 每帧仅移动指示器（redraw_indicator），达成 30Hz+ 刷新。
        self._static_drawn = False
        self._point_items = []
        self._indicator_items = None

        self.bind("<Button-1>", self._on_press)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Double-Button-1>", self._on_double)
        self.bind("<Button-3>", self._on_right)
        self.bind("<Configure>", self._on_configure)

    # ---- 坐标换算（Y 轴为当前通道输出脉宽范围，µs）----
    def _plot_w(self):
        return self._cw - self.PAD_R - self.PAD_L

    def _plot_h(self):
        return self._ch - self.PAD_B - self.PAD_T

    def _px(self, xv):
        return self.PAD_L + xv * self._plot_w()

    def _y_range(self):
        """当前生效的输出范围：未绑定为 (0,1) 归一化，绑定后为脉宽 (MIN, MAX)。"""
        return tuple(self.channel.out_range)

    def _is_normalized(self):
        """当前通道是否为未绑定的归一化输出（Y 轴 0~1）。"""
        lo, hi = self._y_range()
        return lo == 0.0 and hi == 1.0

    def _py(self, yv):
        lo, hi = self._y_range()
        span = (hi - lo) if hi != lo else 1.0
        return self.PAD_T + (hi - yv) / span * self._plot_h()

    def _ix(self, px):
        return (px - self.PAD_L) / self._plot_w()

    def _iy(self, py):
        lo, hi = self._y_range()
        span = (hi - lo) if hi != lo else 1.0
        return hi - (py - self.PAD_T) / self._plot_h() * span

    # ---- 命中检测 ----
    def _find_point(self, px, py):
        best, best_d = None, self.HIT_R
        for i, (x, y) in enumerate(self.points):
            d = math.hypot(self._px(x) - px, self._py(y) - py)
            if d <= best_d:
                best_d, best = d, i
        return best

    def _is_boundary(self, i):
        return i == 0 or i == len(self.points) - 1

    def _x_bounds(self, i):
        # 边界点（x=0 / x=1）锁定 X 坐标，仅允许调整 Y
        if self._is_boundary(i):
            x = 0.0 if i == 0 else 1.0
            return (x, x)
        # 余量取 0.5e-3：与 _reorder_points / _add_point 的 1e-3 最小间距
        # 一致，保证 lo <= hi 恒成立（不会拖出重复 x 触发送样条重建异常）。
        lo = self.points[i - 1][0] + 0.5e-3
        hi = self.points[i + 1][0] - 0.5e-3
        return (lo, hi)

    # ---- 交互事件 ----
    def _on_press(self, e):
        i = self._find_point(e.x, e.y)
        if i is not None:
            # 命中已有点：取消可能排队的空白区新增，避免 260ms 后误加一个点
            if self._pending_add is not None:
                self.after_cancel(self._pending_add)
                self._pending_add = None
            self._drag_index = i
        else:
            # 空白区单击：延迟执行新增，避免与双击冲突
            if self._pending_add is not None:
                self.after_cancel(self._pending_add)
            self._pending_add = self.after(260, lambda: self._add_point(e.x, e.y))

    def _on_drag(self, e):
        if self._drag_index is None:
            # 空白区按下后发生拖拽：取消延迟新增，避免被误判为单击添加
            if self._pending_add is not None:
                self.after_cancel(self._pending_add)
                self._pending_add = None
            return
        i = self._drag_index
        lo, hi = self._x_bounds(i)
        x = max(lo, min(hi, self._ix(e.x)))
        ylo, yhi = self._y_range()
        y = max(ylo, min(yhi, self._iy(e.y)))
        self.points[i] = [x, y]
        self._commit()

    def _on_release(self, _e):
        self._drag_index = None

    def _on_double(self, e):
        if self._pending_add is not None:
            self.after_cancel(self._pending_add)
            self._pending_add = None
        i = self._find_point(e.x, e.y)
        if i is not None:
            self._edit_point(i)
        else:
            self._add_point(e.x, e.y)

    def _on_right(self, e):
        i = self._find_point(e.x, e.y)
        if i is None:
            return
        if self._is_boundary(i):
            return  # 边界节点不可删除
        del self.points[i]
        self._commit()


    def _add_point(self, px, py):
        x = max(0.001, min(0.999, self._ix(px)))
        ylo, yhi = self._y_range()
        y = max(ylo, min(yhi, self._iy(py)))
        for ex, _ey in self.points:
            if abs(ex - x) < 1e-3:
                return  # 与现有样本点 X 过近则忽略
        self.points.append([x, y])
        self.points.sort(key=lambda p: p[0])
        self._commit()

    def _edit_point(self, i):
        x, y = self.points[i]
        dlg = EditPointDialog(self, x, y, lock_x=self._is_boundary(i),
                              y_range=self._y_range())
        if dlg.result is None:
            return
        nx, ny = dlg.result
        ylo, yhi = self._y_range()
        ny = max(ylo, min(yhi, ny))
        if self._is_boundary(i):
            # 边界点（x=0 / x=1）X 锁定，仅允许调整 Y
            nx = x
        else:
            # 非边界点：允许输入任意 x（(0,1) 开区间），输入后整体重新排序，
            # 而不是 clamp 到原邻居区间（避免"改大 x 被钳到邻居处"）。
            nx = max(1e-4, min(1.0 - 1e-4, nx))
        self.points[i] = [nx, ny]
        if not self._is_boundary(i):
            self._reorder_points()
        self._commit()

    def _reorder_points(self):
        """按 x 升序重排，并对重复/过近的 x 做最小间距推挤。

        双击编辑可能把非边界点的 x 改成大于/小于其他点的值，排序后
        需保证 x 严格递增（Mapper 要求）。采用「正向推挤 + 反向收拢」：
          1. 正向：每个内部点不小于前一点 + eps（消除过近/重合）；
          2. 反向：最右内部点不越过 x=1-eps，其余内部点不越过
             后一点 - eps，避免正向推挤把点挤过头后与后一点/边界碰撞
             （旧实现只在最后硬压 points[-2]，可能与前一点重复/逆序，
             使 Mapper 抛 ValueError）。
        两端边界点（x=0 / x=1）X 锁定不移动。
        """
        eps = 1e-3
        self.points.sort(key=lambda p: p[0])
        n = len(self.points)
        for j in range(1, n - 1):   # 跳过两端边界点：正向推挤
            need = self.points[j - 1][0] + eps
            if self.points[j][0] < need:
                self.points[j][0] = need
        if n > 2:
            # 反向收拢：先压最右内部点到 1-eps，再从右向左逐点收拢。
            # 由正向推挤保证的「右点 >= 左点 + eps」可推出单遍反向
            # 后仍满足「左点 + eps <= 右点」，不会压出新碰撞。
            self.points[-2][0] = min(self.points[-2][0], 1.0 - eps)
            for j in range(n - 3, 0, -1):
                cap = self.points[j + 1][0] - eps
                if self.points[j][0] > cap:
                    self.points[j][0] = cap

    def _commit(self):
        self.channel.set_curve_points(self.points)
        if self.on_changed is not None:
            self.on_changed()
        self.draw()

    def sync_from_channel(self):
        """通道输出范围变化（换绑舵机 / 解绑）后，从通道重新取样本点并完整重绘。

        channel.set_out_range() 会按归一化比例重缩放样本点（形状保持），
        本画布持有的本地副本需同步，且静态层（Y 轴刻度）也要按新的
        MIN~MAX 重建。
        """
        self.points = [list(p) for p in self.channel.curve_points()]
        self.delete("static")
        self._static_drawn = False
        self.draw()

    # ---- 绘制 ----
    def set_indicator(self, norm_x):
        self.indicator_x = norm_x

    def _on_configure(self, e):
        """画布尺寸变化（窗口缩放）时更新内部尺寸并完整重绘。"""
        if e.width != self._cw or e.height != self._ch:
            self._cw, self._ch = e.width, e.height
            self.delete("static")          # 旧网格/刻度按旧尺寸绘制，需清除
            self._static_drawn = False
            self.draw()

    # ---- 增量渲染：静态层（网格/刻度/说明，只画一次）----
    def _draw_static(self):
        if self._static_drawn:
            return
        self._static_drawn = True
        w, h = self._cw, self._ch
        px0, px1 = self.PAD_L, w - self.PAD_R
        py0, py1 = self.PAD_T, h - self.PAD_B

        # 网格 + Y 轴刻度（未绑定：归一化 0~1；绑定：脉宽范围 MIN~MAX，µs，5 等分）
        lo, hi = self._y_range()
        normalized = self._is_normalized()
        for k in range(5):
            v = lo + (hi - lo) * k / 4.0
            y = self._py(v)
            self.create_line(px0, y, px1, y, fill=GRID_COLOR, tag="static")
            text = f"{v:.2f}" if normalized else f"{v:.0f}"
            self.create_text(px0 - 6, y, text=text, anchor="e",
                             fill=AXIS_COLOR, font=TINY_FONT, tag="static")

        # 网格 + X 轴刻度（归一化 0~1）
        for xv in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = self._px(xv)
            self.create_line(x, py0, x, py1, fill=GRID_COLOR, tag="static")
            self.create_text(x, py1 + 4, text=f"{xv:.2f}", anchor="n",
                             fill=AXIS_COLOR, font=TINY_FONT, tag="static")


    # ---- 增量渲染：样条曲线（样本点变化时重建；超限段画红色警示）----
    def _draw_curve(self):
        self.delete("curve")
        # 采样：y 钳制到 [MIN,MAX]（µs）防止飞出画布，同时记录是否超限
        lo, hi = self._y_range()
        samples = []
        for s in range(121):
            xv = s / 120.0
            yv = self.channel.map_value(xv)
            over = yv > hi or yv < lo
            yv = max(lo, min(hi, yv))
            samples.append((self._px(xv), self._py(yv), over))
        # 按超限标志分组：同组连续点连成一条线，超限段用红色凸显
        for is_over, group in self._split_by_flag(samples):
            if len(group) < 2:
                continue
            coords = [c for pt in group for c in pt]
            self.create_line(*coords,
                             fill=ARROW_COLOR if is_over else SPLINE_COLOR,
                             width=2, tag="curve")

    @staticmethod
    def _split_by_flag(samples):
        """把采样点按 over 标志切成连续段，返回 [(flag, [(x, y), ...]), ...]。"""
        segments = []
        cur, cur_flag = [], None
        for x, y, over in samples:
            if cur_flag is None:
                cur_flag = over
            if over != cur_flag:
                if cur:
                    segments.append((cur_flag, cur))
                cur, cur_flag = [(x, y)], over
            else:
                cur.append((x, y))
        if cur:
            segments.append((cur_flag, cur))
        return segments

    # ---- 增量渲染：样本点（仅编辑时重建，频率极低）----
    def _draw_points(self):
        for items in self._point_items:
            for it in items:
                self.delete(it)
        self._point_items = []
        for x, y in self.points:
            px, py = self._px(x), self._py(y)
            oval = self.create_oval(px - 4, py - 4, px + 4, py + 4,
                                    fill=POINT_COLOR, outline="#000000", width=1)
            hline = self.create_line(px - 6, py, px + 6, py, fill=POINT_COLOR)
            vline = self.create_line(px, py - 6, px, py + 6, fill=POINT_COLOR)
            self._point_items.append((oval, hline, vline))

    # ---- 增量渲染：指示器（每帧仅移动坐标，绝不重建）----
    def redraw_indicator(self):
        if self.indicator_x is None:
            if self._indicator_items is not None:
                for it in self._indicator_items:
                    self.delete(it)
                self._indicator_items = None
            return
        ix = self._px(max(0.0, min(1.0, self.indicator_x)))
        py0, py1 = self.PAD_T, self._ch - self.PAD_B
        # 交点脉宽同样钳制到 [MIN,MAX]，防止样条过冲时交点圆飞出画布
        lo, hi = self._y_range()
        iy = self._py(max(lo, min(hi,
                                  self.channel.map_value(self.indicator_x))))
        if self._indicator_items is None:
            line = self.create_line(ix, py0, ix, py1, fill=ARROW_COLOR,
                                    dash=(2, 3), width=1)
            poly = self.create_polygon(ix, py1 - 1, ix - 5, py1 + 9,
                                       ix + 5, py1 + 9,
                                       fill=ARROW_COLOR, outline=ARROW_COLOR)
            oval = self.create_oval(ix - 4, iy - 4, ix + 4, iy + 4,
                                    fill=ARROW_COLOR, outline="")
            self._indicator_items = (line, poly, oval)
        else:
            line, poly, oval = self._indicator_items
            self.coords(line, ix, py0, ix, py1)
            self.coords(poly, ix, py1 - 1, ix - 5, py1 + 9, ix + 5, py1 + 9)
            self.coords(oval, ix - 4, iy - 4, ix + 4, iy + 4)

    def draw(self):
        """完整重绘：静态层 + 样条曲线 + 样本点 + 指示器。

        仅当画布初始化或样本点被编辑时调用；逐帧刷新请用 redraw_indicator()。
        """
        self._draw_static()
        self._draw_curve()
        self._draw_points()
        self.redraw_indicator()


class EditPointDialog(tk.Toplevel):
    """左键双击样本点弹出的 X/Y 数值编辑模态框。

    - X 输入范围 0~1（归一化）
    - Y 输入范围：未绑定通道为 0~1（归一化），绑定后为该通道脉宽范围（µs）
    - 边界点（x=0 / x=1）锁定 X，仅允许修改 Y
    """

    def __init__(self, master, x, y, lock_x=False, y_range=(0.0, 1.0)):
        super().__init__(master.winfo_toplevel())
        self.result = None
        self._y_range = (float(y_range[0]), float(y_range[1]))
        self._normalized = (self._y_range == (0.0, 1.0))
        self.title("编辑样本点")
        self.resizable(False, False)
        self.transient(master.winfo_toplevel())

        body = tk.Frame(self, padx=12, pady=10)
        body.pack()
        tk.Label(body, text="X (0~1):").grid(row=0, column=0, sticky="e")
        self._x_var = tk.StringVar(value=f"{x:.4f}")
        x_entry = tk.Entry(body, textvariable=self._x_var, width=12)
        if lock_x:
            x_entry.config(state="disabled")
        x_entry.grid(row=0, column=1, padx=6, pady=4)
        lo, hi = self._y_range
        y_label = "Y (0~1):" if self._normalized else f"Y ({lo:.0f}~{hi:.0f}µs):"
        tk.Label(body, text=y_label).grid(row=1, column=0, sticky="e")
        self._y_var = tk.StringVar(value=f"{y:.2f}")
        tk.Entry(body, textvariable=self._y_var, width=12)\
            .grid(row=1, column=1, padx=6, pady=4)

        btns = tk.Frame(self)
        btns.pack(pady=(0, 10))
        tk.Button(btns, text="确定", width=8, command=self._on_ok)\
            .pack(side=tk.LEFT, padx=6)
        tk.Button(btns, text="取消", width=8, command=self._on_cancel)\
            .pack(side=tk.LEFT, padx=6)

        self.bind("<Return>", lambda _e: self._on_ok())
        self.bind("<Escape>", lambda _e: self._on_cancel())
        self.protocol("WM_DELETE_WINDOW", self._on_cancel)

        # 居中于父窗口后进入模态阻塞：
        # grab_set() 必须在窗口映射之后调用——直接在 __init__ 同步调用会抛
        # "grab failed: window not viewable"（TclError）导致对话框空白。
        # 用 after_idle 推迟到事件循环空闲（窗口已可见）时再 grab。
        self.update_idletasks()
        self._center(master.winfo_toplevel())
        self.after_idle(self._set_grab)
        self.wait_window()

    def _set_grab(self):
        """窗口可见后置为模态 grab；极端环境仍失败则降级为非 grab 模态。"""
        try:
            self.grab_set()
        except tk.TclError:   # 窗口仍未映射（如特殊 WM/无头环境）时降级
            pass

    def _center(self, parent):
        w, h = self.winfo_width(), self.winfo_height()
        x = parent.winfo_rootx() + (parent.winfo_width() - w) // 2
        y = parent.winfo_rooty() + (parent.winfo_height() - h) // 2
        self.geometry(f"+{x}+{y}")

    def _on_ok(self):
        try:
            x = float(self._x_var.get())
            y = float(self._y_var.get())
        except ValueError:
            messagebox.showerror("输入错误", "请输入有效的数字", parent=self)
            return
        lo, hi = self._y_range
        if not (0.0 <= x <= 1.0) or not (lo <= y <= hi):
            y_hint = "0~1" if self._normalized else f"{lo:.0f}~{hi:.0f}µs"
            messagebox.showerror(
                "输入错误",
                f"X 需在 0~1 之间，Y 需在 {y_hint} 之间",
                parent=self)
            return
        self.result = (x, y)
        self.destroy()

    def _on_cancel(self):
        self.result = None
        self.destroy()
