"""LiveSuit 前端 GUI —— 基于 tkinter 的个人可动兽装控制系统界面。

包含:
  - WelcomePage          欢迎页（Logo / 标题 / 版本 / 简介 / 启动与渲染切换按钮）
  - SplineCurveCanvas    三次样条插值调节画布（单击新增 / 拖拽 / 双击编辑 / 右键删除）
  - EmaFilterCanvas      EMA 平滑滤波画布（时间轴纵向，淡色原始 + 实色滤波 + 向上指示箭头）
  - EditPointDialog      双击样本点的 X/Y 编辑模态框
  - SlotControllerWindow 单个槽位的控制器窗口（输出参数名 + 样条图 + EMA 图 + Alpha 滑条）
  - ServoBusPanel        舵机总线配置面板（角度输入自动检测 + 舵机下拉绑定 + 去重校验）
  - LiveSuitApp          主应用：欢迎页 + 面板管理 + 逐帧渲染循环

渲染模式（由 Store 管理）:
  - debug    调试模式：渲染并显示所有子面板
  - headless 无头模式：隐藏并停止渲染所有子面板（最大化节约系统资源）

界面采用与 eye_tracker_main.py 一致的简单 tkinter 风格（默认主题）。
"""

import argparse
import math
import sys
import time
import tkinter as tk
import tkinter.font as tkfont
from pathlib import Path
from tkinter import messagebox, ttk

_HERE = Path(__file__).resolve().parent            # src/
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
_SERVO_CONTROL = _HERE / "servo_control"           # src/servo_control/
if str(_SERVO_CONTROL) not in sys.path:
    sys.path.insert(0, str(_SERVO_CONTROL))

# 输出槽位注册表与真实数据源统一来自 src/servo_control/slots.py（延迟导入，
# 见 _load_real_runtime）。gui.py 顶层不 import slots，且真正实例化 Slots
# （会唤起眼追调试面板）延迟到点击「启动LiveSuit」之后才发生，
# 避免把 cv2 / eye_tracker 等硬件依赖栈强耦合进纯 UI 或过早弹出眼追面板。

from pipeline_manager import (  # noqa: E402
    DEFAULT_MAX_PULSE,
    DEFAULT_MIN_PULSE,
    PULSE_SAFE_MAX,
    PULSE_SAFE_MIN,
    CallableSource,
    PipelineManager,
    SERVO_CONFIGS_PATH,
    SERVO_CONFIG_STATUS_REPLACED,
    ServoDebugger,
    read_servo_configs,
)

_LOGO_PATH = _HERE.parent / "LiveSuitLogo.png"   # 仓库根目录下的 Logo

def _load_real_runtime(root):
    """加载真实运行环境：真实槽位注册表 + 真实数据源（眼球追踪 Slots）。

    仅在点击「启动LiveSuit」后由 start() 调用：此刻才延迟导入 slots
    （含 eye_tracker / cv2 依赖栈）并实例化 Slots，唤起眼追调试面板。
    返回 (specs, source, runtime)：runtime 为真实 Slots 提供者，供停止时
    释放追踪进程。缺依赖 / 无摄像头等异常直接冒泡，由 start() 弹窗提示。

    真实数据源为 CallableSource(slots.get_all_output)，直接对接
    src/servo_control/slots.py 的 Slots 提供者。
    """
    from slots import Slots, get_slot_specs
    slots = Slots(root)                        # 启动眼球追踪提供者
    return (get_slot_specs(), CallableSource(slots.get_all_output), slots)


def _load_servo_controller(pulse_configs):
    """惰性初始化舵机控制器（依赖 Adafruit CircuitPython 库与 I2C 总线）。

    pulse_configs 为 {通道索引: (min_pulse, max_pulse)} 舵机脉宽注册表
    （来自 servo_configs.yaml）；仅注册过的通道可下发。

    无 PCA9685 硬件 / 缺依赖时打印警告并返回 None（跳过舵机下发），
    不影响 GUI 与管线运行；这是硬件缺失的真实场景，非异常兜底。
    """
    try:
        from servo_controller import ServoController
        return ServoController(pulse_configs=pulse_configs)
    except Exception as exc:  # noqa: BLE001  缺库 / 无 I2C 设备 / 总线探测失败
        print(f"[gui] 无法初始化舵机控制器（{exc}），将跳过舵机下发。")
        return None

# ---------------------------------------------------------------
# 常量：简单 tkinter 风格下的图表配色（默认浅底）
# ---------------------------------------------------------------
CANVAS_BG = "#ffffff"
GRID_COLOR = "#e0e0e0"
AXIS_COLOR = "#555555"
SPLINE_COLOR = "#1a73e8"      # 三次样条曲线（蓝）
POINT_COLOR = "#e65100"       # 样本点（橙）
RAW_SERIES_COLOR = "#78909c"  # EMA 原始序列（蓝灰，中等深度、清晰可见）
FIL_SERIES_COLOR = "#1565c0"  # EMA 滤波序列（实色）
ARROW_COLOR = "#c62828"       # 指示箭头（红）
DANGER_COLOR = "#b71c1c"      # 深红：调试脉宽超出配置范围的警示色

POLL_MS = 30                  # 默认渲染/处理轮询间隔（约 33Hz，≥30Hz 目标留余量）

SERVO_FAIL_STREAK_LIMIT = 30   # 舵机下发连续失败约 1s 后暂停逐帧下发
SERVO_RETRY_INTERVAL = 600     # 降级后约 20s 周期性重试一次

TINY_FONT = ("Arial", 8)
SMALL_FONT = ("Arial", 9)
BODY_FONT = ("Arial", 10)
TITLE_FONT = ("Arial", 12, "bold")
BIG_TITLE_FONT = ("Arial", 22, "bold")
VALUE_FONT = ("Courier", 11, "bold")


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


class SplineCurveCanvas(tk.Canvas):
    """三次样条插值调节画布（任务管理器风格曲线图）。

    交互:
      - 左键单击空白区域 -> 新增样本点
      - 左键拖拽样本点    -> 移动样本点并实时刷新样条曲线
      - 左键双击样本点    -> 弹出 X(0~1) / Y(MIN~MAX) 编辑弹窗
      - 右键单击样本点    -> 删除样本点（x=0 与 x=1 边界点不可删除）

    X 轴为归一化输入 0~1，Y 轴为该槽位当前输出脉宽范围 MIN~MAX（µs）。
    MIN/MAX 来自 slot.out_range：未绑定舵机时为默认 500~2500，绑定某路
    舵机后为该舵机在 servo_configs.yaml 中注册的真实脉宽范围。
    顶部 X 轴上绘制当前滤波值的映射指示（与下方 EMA 图箭头对齐）。
    """

    PAD_L = 30
    PAD_R = 10
    PAD_T = 10
    PAD_B = 16
    HIT_R = 7     # 命中半径（px）

    def __init__(self, master, slot, width=232, height=132, on_changed=None):
        super().__init__(master, width=width, height=height,
                         bg=CANVAS_BG, highlightthickness=1,
                         highlightbackground="#ccc")
        self.slot = slot
        self.on_changed = on_changed
        self._cw = width
        self._ch = height
        self.points = [list(p) for p in slot.points]   # 本画布维护的可编辑样本点（µs）
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

    # ---- 坐标换算（Y 轴为当前槽位输出脉宽范围，µs）----
    def _plot_w(self):
        return self._cw - self.PAD_R - self.PAD_L

    def _plot_h(self):
        return self._ch - self.PAD_B - self.PAD_T

    def _px(self, xv):
        return self.PAD_L + xv * self._plot_w()

    def _y_range(self):
        """当前生效的输出脉宽范围 (MIN, MAX)。"""
        return tuple(self.slot.out_range)

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
        self.slot.set_points(self.points)
        if self.on_changed is not None:
            self.on_changed()
        self.draw()

    def sync_from_slot(self):
        """槽位输出范围变化（换绑舵机 / 解绑）后，从槽位重新取样本点并完整重绘。

        slot.set_out_range() 会按归一化比例重缩放样本点（形状保持），
        本画布持有的本地副本需同步，且静态层（Y 轴刻度）也要按新的
        MIN~MAX 重建。
        """
        self.points = [list(p) for p in self.slot.points]
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

        # 网格 + Y 轴刻度（当前脉宽范围 MIN~MAX，µs，5 等分）
        lo, hi = self._y_range()
        for k in range(5):
            v = lo + (hi - lo) * k / 4.0
            y = self._py(v)
            self.create_line(px0, y, px1, y, fill=GRID_COLOR, tag="static")
            self.create_text(px0 - 6, y, text=f"{v:.0f}", anchor="e",
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
            yv = self.slot.mapper.get_result(xv)
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
                                  self.slot.mapper.get_result(self.indicator_x))))
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


class EmaFilterCanvas(tk.Canvas):
    """EMA 平滑滤波画布（时间轴纵向，最新采样在顶部）。

    - 时间轴从下向上推进，最新采样位于顶部（靠近上方插值图 X 轴）；
    - 原始数据用淡色线条，滤波后数据用实色线条，沿垂直方向叠加；
    - 最新滤波值以红色圆点 + 数值标签标注在顶部。
    """

    PAD_L = 30
    PAD_R = 24
    PAD_T = 8
    PAD_B = 14

    def __init__(self, master, slot, width=232, height=96):
        super().__init__(master, width=width, height=height,
                         bg=CANVAS_BG, highlightthickness=1,
                         highlightbackground="#ccc")
        self.slot = slot
        self._cw = width
        self._ch = height
        # 增量渲染状态：静态层只画一次，折线/箭头每帧增量更新
        self._static_drawn = False
        self._hint_tag = "hint"
        self._raw_tag = "raw"
        self._filter_line_item = None   # 滤波序列折线（复用，每帧 coords 更新）
        self._arrow_items = None        # (引导线, 圆点, 数值标签)
        self.bind("<Configure>", self._on_configure)

    # ---- 坐标换算（与 SplineCurveCanvas 共享同一 X 轴）----
    def _plot_w(self):
        return self._cw - self.PAD_R - self.PAD_L

    def _plot_h(self):
        return self._ch - self.PAD_B - self.PAD_T

    def _px(self, xv):
        return self.PAD_L + xv * self._plot_w()

    def _series_segments(self, hist, total, key_index):
        """把历史序列映射为画布折线段；None 值处断开（形成缺口）。

        frac = (n - 1 - idx) / (total - 1)：最新样本固定顶部(0)，
        旧样本向下推进，历史填满后最旧样本位于底部(1)。
        """
        n = len(hist)
        segments = []
        current = []
        for idx, item in enumerate(hist):
            v = item[key_index]
            if v is None:
                if current:
                    segments.append(current)
                    current = []
                continue
            frac = (n - 1 - idx) / (total - 1)   # 0=顶(新) 1=底(旧)
            x = self._px(max(0.0, min(1.0, v)))
            y = self.PAD_T + frac * self._plot_h()
            current.append((x, y))
        if current:
            segments.append(current)
        return segments

    def _on_configure(self, e):
        """画布尺寸变化（窗口缩放）时更新内部尺寸并完整重绘。"""
        if e.width != self._cw or e.height != self._ch:
            self._cw, self._ch = e.width, e.height
            self.delete("static")
            self._static_drawn = False
            self.draw()

    def _draw_static(self):
        """静态层（网格/时间标签，只画一次）。"""
        if self._static_drawn:
            return
        self._static_drawn = True
        px0, px1 = self.PAD_L, self._cw - self.PAD_R
        py0, py1 = self.PAD_T, self._ch - self.PAD_B

        # 数值方向网格（0 / 0.5 / 1）+ 底部刻度
        for xv in (0.0, 0.5, 1.0):
            x = self._px(xv)
            self.create_line(x, py0, x, py1, fill=GRID_COLOR, tag="static")
            self.create_text(x, py1 + 4, text=f"{xv:.2f}", anchor="n",
                             fill=AXIS_COLOR, font=TINY_FONT, tag="static")

        # 时间方向标签（右侧）：时间从下向上推进，最新样本在顶部
        self.create_text(px1 + 4, py0 + 6, text="时间", anchor="w",
                         fill=AXIS_COLOR, font=TINY_FONT, tag="static")
        self.create_text(px1 + 4, py1 - 4, text="↑", anchor="w",
                         fill=AXIS_COLOR, font=TINY_FONT, tag="static")

    def redraw(self):
        """每帧增量重绘：静态层只画一次，折线/箭头仅更新坐标。"""
        self._draw_static()
        self.delete(self._hint_tag)
        self.delete(self._raw_tag)
        px0, px1 = self.PAD_L, self._cw - self.PAD_R
        py0, py1 = self.PAD_T, self._ch - self.PAD_B
        hist = list(self.slot.history)
        total = max(2, self.slot.max_history)

        if not hist:
            self.create_text((px0 + px1) / 2, (py0 + py1) / 2,
                             text="等待数据…", fill=AXIS_COLOR,
                             font=BODY_FONT, tag=self._hint_tag)
            self.create_text((px0 + px1) / 2, py1 + 4,
                             text="数值 (归一化 0~1)", fill=AXIS_COLOR,
                             font=TINY_FONT, tag=self._hint_tag)
            self._clear_series()
            return

        # 原始序列（淡色细线；None 处按段重建——段数少且变化不频繁）
        for seg in self._series_segments(hist, total, 0):
            if len(seg) < 2:
                continue   # 单点段无法连成线，跳过
            coords = [c for pt in seg for c in pt]
            self.create_line(*coords, fill=RAW_SERIES_COLOR, width=1,
                             tag=self._raw_tag)

        # 滤波序列（实色粗线；复用单个 line item，整段 coords 替换）
        segs = self._series_segments(hist, total, 1)
        seg = max(segs, key=len) if segs else None
        if seg is not None and len(seg) >= 2:
            coords = [c for pt in seg for c in pt]
            if self._filter_line_item is None:
                self._filter_line_item = self.create_line(
                    *coords, fill=FIL_SERIES_COLOR, width=2)
            else:
                self.coords(self._filter_line_item, *coords)
        elif self._filter_line_item is not None:
            self.delete(self._filter_line_item)
            self._filter_line_item = None

        # 最新滤波值：指示箭头（向上指向插值图 X 轴）+ 数值标签
        latest = None
        for item in hist:
            if item[1] is not None:
                latest = item
        self._update_arrow(latest)

    def _clear_series(self):
        """清空折线/箭头（历史为空时调用）。"""
        if self._filter_line_item is not None:
            self.delete(self._filter_line_item)
            self._filter_line_item = None
        self._update_arrow(None)

    def _update_arrow(self, latest):
        """增量更新最新滤波值指示（圆点 + 数值标签），最新样本固定在顶部。

        最新点位于画布顶部，与上方插值图 X 轴水平对齐，无需向上引导线。
        """
        if latest is None:
            if self._arrow_items is not None:
                for it in self._arrow_items:
                    self.delete(it)
                self._arrow_items = None
            return
        x = self._px(max(0.0, min(1.0, latest[1])))
        y_latest = self.PAD_T                         # 最新样本固定在顶部
        if self._arrow_items is None:
            oval = self.create_oval(x - 4, y_latest - 4, x + 4, y_latest + 4,
                                    fill=ARROW_COLOR, outline="")
            text = self.create_text(x + 12, y_latest + 8,
                                    text=f"{latest[1]:.3f}", anchor="w",
                                    fill=ARROW_COLOR, font=VALUE_FONT)
            self._arrow_items = (oval, text)
        else:
            oval, text = self._arrow_items
            self.coords(oval, x - 4, y_latest - 4, x + 4, y_latest + 4)
            self.coords(text, x + 12, y_latest + 8)
            self.itemconfig(text, text=f"{latest[1]:.3f}")

    def draw(self):
        """完整重绘（等价于 redraw；首次调用时含静态层）。"""
        self.redraw()


class EditPointDialog(tk.Toplevel):
    """左键双击样本点弹出的 X/Y 数值编辑模态框。

    - X 输入范围 0~1（归一化）
    - Y 输入范围 MIN~MAX（该槽位当前脉宽范围，µs）
    - 边界点（x=0 / x=1）锁定 X，仅允许修改 Y
    """

    def __init__(self, master, x, y, lock_x=False, y_range=(500.0, 2500.0)):
        super().__init__(master.winfo_toplevel())
        self.result = None
        self._y_range = (float(y_range[0]), float(y_range[1]))
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
        tk.Label(body, text=f"Y ({lo:.0f}~{hi:.0f}µs):")\
            .grid(row=1, column=0, sticky="e")
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
            messagebox.showerror(
                "输入错误",
                f"X 需在 0~1 之间，Y 需在 {lo:.0f}~{hi:.0f}µs 之间",
                parent=self)
            return
        self.result = (x, y)
        self.destroy()

    def _on_cancel(self):
        self.result = None
        self.destroy()


class SlotControllerWindow(tk.Toplevel):
    """单个槽位的控制器 UI 窗口。

    布局（自顶向下）：
      输出参数名
      # 三次样条插值调节窗口（曲线图，支持点击 / 拖拽 / 双击 / 右键）
      # EMA平滑滤波调节窗口（时间轴纵向）
      Alpha 值调节滑条（实时更新波形与指示箭头映射位置）
      舵机绑定状态
      保存配置 / 重置配置 按钮
    """

    def __init__(self, master, slot, manager=None, on_close=None):
        super().__init__(master)
        self.slot = slot
        self.manager = manager
        self._on_close = on_close
        self.title(f"LiveSuit · {slot.name}")
        self.resizable(True, True)      # 支持动态缩放（画布随窗口伸缩）
        self.minsize(200, 300)

        header = tk.Label(self, text=f"{slot.name}",
                          font=TITLE_FONT)
        header.pack(pady=(6, 2))

        tk.Label(self,
                 text="样条曲线（脉宽 µs）",
                 font=TINY_FONT, fg="#333").pack(anchor="w", padx=8)

        tk.Label(self,
                 text="（左键单击曲线/双击空白处新建点；双击点编辑；右键点删除）",
                 font=TINY_FONT, fg="#333").pack(anchor="w", padx=8)


        self.spline = SplineCurveCanvas(self, slot,
                                        on_changed=self._on_spline_changed)
        self.spline.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 4))

        tk.Label(self, text="EMA滤波",
                 font=TINY_FONT, fg="#333").pack(anchor="w", padx=8)
        self.ema = EmaFilterCanvas(self, slot)
        self.ema.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 4))

        # alpha 行：滑条随窗口伸缩（布局自然，窄窗口不溢出裁剪）。
        # resize 后由 _on_window_resize 强制刷新该行，规避 tkinter/X11
        # 快速伸缩时相邻 Label 漏重绘出现的白块残影。
        self.alpha_row = tk.Frame(self)
        self.alpha_row.pack(fill=tk.X, padx=8, pady=(0, 4))
        tk.Label(self.alpha_row, text="Alpha:").pack(side=tk.LEFT)
        self._alpha_var = tk.DoubleVar(value=slot.alpha)
        self.alpha_scale = tk.Scale(self.alpha_row, from_=0.0, to=1.0,
                                    resolution=0.01, orient=tk.HORIZONTAL,
                                    variable=self._alpha_var, showvalue=False,
                                    command=self._on_alpha, length=150)
        self.alpha_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.alpha_val = tk.Label(self.alpha_row, text=f"{slot.alpha:.2f} ",
                                  font=VALUE_FONT, width=10, anchor="e")
        self.alpha_val.pack(side=tk.LEFT)

        self.binding_label = tk.Label(self, text="", font=SMALL_FONT, fg="#666")
        self.binding_label.pack(pady=(0, 6))

        # 配置操作行：保存配置 / 重置配置（同一行，位于窗口最底部）
        self.config_btn_row = tk.Frame(self)
        self.config_btn_row.pack(pady=(0, 8))
        tk.Button(self.config_btn_row, text="保存配置", width=10,
                  command=self._on_save_config).pack(side=tk.LEFT, padx=6)
        tk.Button(self.config_btn_row, text="重置配置", width=10,
                  command=self._on_reset_config).pack(side=tk.LEFT, padx=6)

        self._resize_refresh_pending = False
        self._last_out_range = slot.out_range   # 绑定变化时用于触发样条重绘
        self.bind("<Configure>", self._on_window_resize)
        self.protocol("WM_DELETE_WINDOW", self._handle_close)
        self.refresh_binding()
        # 初始化必须完整绘制样条图（静态层 + 曲线 + 样本点 + 指示器）；
        # redraw() 仅做每帧轻量刷新，不会绘制样条曲线与样本点。
        self.spline.set_indicator(self.slot.latest_filtered)
        self.spline.draw()
        self.ema.redraw()

    def _handle_close(self):
        """用户关闭本窗口：先通知 LiveSuitApp 摘除引用，再销毁。

        否则 _tick 会对已销毁的画布继续 redraw，抛出 TclError 并中断主循环。
        """
        if self._on_close is not None:
            self._on_close(self)
        self.destroy()

    def _on_window_resize(self, _e):
        """窗口 resize 后强制刷新 alpha 行。

        tkinter/X11 下窗口快速伸缩时，部分 Label（"Alpha:" / 数值）可能
        漏重绘显示为纯色块。after_idle 合并到事件流结束后一次性刷新，
        避免高频 resize 期间反复 update 造成卡顿。
        """
        if self._resize_refresh_pending:
            return
        self._resize_refresh_pending = True
        try:
            self.alpha_row.after_idle(self._refresh_alpha_row)
        except tk.TclError:   # 窗口已销毁
            self._resize_refresh_pending = False

    def _refresh_alpha_row(self):
        self._resize_refresh_pending = False
        try:
            self.alpha_row.update()
        except tk.TclError:   # 窗口已销毁
            pass

    def _on_alpha(self, _val):
        a = float(self._alpha_var.get())
        self.slot.set_alpha(a)
        self.alpha_val.config(text=f"{a:.2f} ")
        self.redraw()

    def _on_spline_changed(self):
        # 样条映射变化后立即刷新（上方曲线与下方箭头同步）
        self.redraw()

    # ---- 配置保存 / 重置 ----
    def _on_save_config(self):
        """保存配置：把当前槽位的 alpha + 样本点写入 slot_configs.yaml。"""
        if self.manager is None:
            return
        try:
            self.manager.save_slot_config(self.slot.name)
        except Exception as exc:  # noqa: BLE001  磁盘/权限等写入异常需明确提示
            messagebox.showerror("保存配置失败",
                                 f"保存 {self.slot.name} 配置时出错：\n{exc}",
                                 parent=self)
            return
        messagebox.showinfo("保存配置", f"{self.slot.name} 配置已保存。", parent=self)

    def _on_reset_config(self):
        """重置配置：当前槽位恢复默认 alpha + 样本点，并写回配置文件。"""
        if self.manager is None:
            return
        if not messagebox.askokcancel("重置配置", "将重置滤波参数和曲线！", parent=self):
            return
        self.manager.reset_slot_config(self.slot.name)
        self._sync_from_slot()
        messagebox.showinfo("重置配置",
                            f"{self.slot.name} 已重置为默认配置。", parent=self)

    def _sync_from_slot(self):
        """把槽位当前状态（alpha / 样本点）同步回 UI 控件并重绘。"""
        self._alpha_var.set(self.slot.alpha)
        self.alpha_val.config(text=f"{self.slot.alpha:.2f} ")
        self.spline.points = [list(p) for p in self.slot.points]
        self.spline.draw()
        self.redraw()

    def refresh_binding(self):
        if self.manager is None:
            return
        # 换绑舵机 / 解绑会改变槽位输出脉宽范围（MIN~MAX）：
        # 检测到变化时同步画布样本点并重绘静态层（Y 轴刻度）。
        if self.slot.out_range != self._last_out_range:
            self._last_out_range = self.slot.out_range
            self.spline.sync_from_slot()
            self.ema.redraw()
        idx = self.manager.get_binding(self.slot.name)
        lo, hi = self.slot.out_range
        if idx is not None:
            text = (f"已绑定舵机: servo_{idx}（{lo:.0f}~{hi:.0f}µs）")
        else:
            text = (f"未绑定舵机（默认 {DEFAULT_MIN_PULSE:.0f}~"
                    f"{DEFAULT_MAX_PULSE:.0f}µs）")
        if self.binding_label.cget("text") != text:   # 脏检查，避免每帧重建
            self.binding_label.config(text=text)

    def redraw(self):
        """每帧轻量刷新：样条图仅移动指示器，EMA 图增量更新折线/箭头。"""
        norm = self.slot.latest_filtered
        self.spline.set_indicator(norm)
        self.spline.redraw_indicator()
        self.ema.redraw()
        self.refresh_binding()


class ServoBusPanel(tk.Toplevel):
    """舵机总线配置面板。

    - 输入槽位列：自动检测当前系统所有可用输入槽位（来自 slots.py 注册表），
      以 Label 文本呈现；
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
        for name in self.manager.slot_names():
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
        写回配置文件），但硬钳制到全局安全范围 [400, 2700]µs；
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
        if out:
            self._pulse_entry.config(fg=DANGER_COLOR)
            self._status_label.config(
                fg=DANGER_COLOR,
                text=(f"⚠ 超出配置范围 {lo:.0f}~{hi:.0f}µs，"
                      f"可能有舵机损坏风险（安全范围 {PULSE_SAFE_MIN:.0f}~"
                      f"{PULSE_SAFE_MAX:.0f}µs）"))
        else:
            self._pulse_entry.config(fg="#000000")
            self._status_label.config(
                fg="#666",
                text=(f"配置范围: {lo:.0f}~{hi:.0f}µs · "
                      f"安全范围: {PULSE_SAFE_MIN:.0f}~{PULSE_SAFE_MAX:.0f}µs"))

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

        self.render_label = tk.Label(self, text="", font=SMALL_FONT, fg="#666")
        self.render_label.pack(pady=(0, 12))
        self.update_render_label()

    def on_started(self):
        """启动后：启动按钮变为停止按钮，启用渲染切换，停用舵机工具。"""
        self.start_btn.config(text="停止", command=self.app.stop)
        self.toggle_btn.config(state=tk.NORMAL)
        self.servo_tool_btn.config(state=tk.DISABLED)   # 启动后禁止再打开舵机工具

    def update_render_label(self, mode=None):
        mode = mode if mode is not None else self.app.manager.render_mode
        text = ("当前渲染: 调试模式（显示所有子面板）"
                if mode == "debug" else
                "当前渲染: 无头模式（隐藏并停止渲染所有子面板）")
        self.render_label.config(text=text)


class LiveSuitApp:
    """主应用：欢迎页 + 子面板管理 + 逐帧渲染循环。

    - 启动后按当前渲染模式显示 / 隐藏所有子面板
    - 「切换窗口渲染」在 debug 与 headless 间切换（Store 订阅驱动）
    - 无头模式下管线仍继续运行，但跳过所有子面板渲染（节约资源）
    """

    def __init__(self, root, poll_ms=POLL_MS, initial_headless=False):
        self.root = root
        self.poll_ms = poll_ms
        self._runtime = None
        # 每次运行都读取 servo_configs.yaml（缺失自动生成默认 16 路，非法则
        # 用默认覆盖并打印/弹窗提示）；只有注册过的舵机在后续可用。
        servo_configs, servo_cfg_status = read_servo_configs(SERVO_CONFIGS_PATH)
        self.servo_pulse_configs = servo_configs
        # 注意：启动阶段不实例化 Slots 提供者（否则会立即唤起眼追调试面板）。
        # 先用空注册表 + 占位数据源构建占位管理器；真实的槽位注册表与
        # 眼球追踪数据源在点击「启动LiveSuit」后由 start() 中的
        # _load_real_runtime() 加载，并重建管理器（沿用同一 Store，保留状态）。
        self.manager = PipelineManager(slots_spec={},
                                       source=CallableSource(lambda: {}),
                                       servo_pulse_configs=servo_configs)
        if initial_headless:
            self.manager.set_render_mode("headless")
        self.store = self.manager.store

        self._slot_windows = []
        self.servo_panel = None
        self.servo = None              # ServoController，start() 时惰性初始化
        self.servo_debug = ServoDebugger(pulse_configs=servo_configs)
        self.servo_tool = None         # 舵机调试工具窗口引用
        self._tick_job = None
        self._tick_count = 0              # 逐帧计数（降级后用于周期性重试舵机）
        self._servo_fail_streak = 0       # 舵机下发连续失败帧数
        self._servo_degraded = False      # 连续失败后暂停逐帧下发
        self._last_tick_err = 0.0         # 单帧异常日志限频时间戳
        self._started = False

        # servo_configs.yaml 非法被默认覆盖时给出明确提示（read 已打印详情）
        if servo_cfg_status == SERVO_CONFIG_STATUS_REPLACED:
            messagebox.showwarning(
                "舵机配置提示",
                "servo_configs.yaml 不合法，已用默认配置（servo_0~15，"
                "500~2500µs）覆盖该文件。\n请按实际舵机型号编辑该文件。")

        root.title("LiveSuit")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.welcome = WelcomePage(root, self)
        self.welcome.pack(fill=tk.BOTH, expand=True)

        # 订阅 Store：渲染模式 / 绑定变化时自动同步 UI
        self.store.subscribe(self._on_state)

    # ---- Store 订阅 ----
    def _on_state(self, _state, patch):
        if "render_mode" in patch:
            self._apply_render_mode(patch["render_mode"])
            self.welcome.update_render_label(patch["render_mode"])
        if "bindings" in patch:
            self._prune_slot_windows()
            if self.servo_panel is not None and not self.servo_panel.winfo_exists():
                self.servo_panel = None
            if self.servo_panel is not None:
                self.servo_panel.refresh_bindings()
            for w in self._slot_windows:
                w.refresh_binding()

    # ---- 启动 / 渲染切换 ----
    def start(self):
        if self._started:
            return
        self._started = True
        # 点击「启动LiveSuit」后才实例化 Slots 提供者（此刻才唤起眼追调试面板）。
        # 缺依赖 / 无相机等异常在此捕获并弹窗提示，可重试，不影响欢迎页。
        try:
            specs, source, self._runtime = _load_real_runtime(self.root)
        except Exception as exc:  # noqa: BLE001  缺依赖 / 无相机 / 摄像头初始化失败
            import traceback
            traceback.print_exc()
            self._started = False
            messagebox.showerror(
                "启动失败",
                f"无法启动眼球追踪：{exc}\n请检查依赖与摄像头后重新点击启动。")
            return
        # 用真实槽位注册表与数据源重建管理器（沿用原 Store，保留渲染模式等状态）
        self.manager = PipelineManager(slots_spec=specs, source=source,
                                       store=self.store,
                                       servo_pulse_configs=self.servo_pulse_configs)
        self.manager.start()
        # 启动时读取槽位配置（alpha + 样本点）；无配置文件则按默认配置新建
        self.manager.load_slot_configs()
        # 启动后管线接管舵机逐帧下发：关闭启动前的舵机调试工具窗口，
        # 并停用「打开舵机工具」按钮（由 welcome.on_started() 完成）。
        self._close_servo_tool()
        # 舵机控制器可能已在打开调试工具时惰性初始化，避免重复创建
        if self.servo is None:
            self.servo = _load_servo_controller(self.servo_pulse_configs)
        self._build_panels()
        self._apply_render_mode(self.manager.render_mode)
        self._schedule_tick()
        self.welcome.on_started()   # 启动按钮 -> 停止按钮，启用渲染切换

    def toggle_render(self):
        self.manager.toggle_render_mode()  # 触发 Store -> _on_state -> _apply_render_mode

    # ---- 舵机调试工具 ----
    def open_servo_tool(self):
        """打开舵机调试工具（仅启动前可用）。

        - 启动后（管线运行中）直接返回，防止与逐帧下发冲突；
        - 舵机控制器未初始化时惰性初始化（无硬件时为 None，仅跳过下发）；
        - 已打开窗口时置顶复用，避免重复创建。
        """
        if self._started:
            return
        if not self.servo_debug.registered_indices():
            # 无任何注册舵机时无法调试：提示后直接返回，
            # 避免 ServoToolWindow 对空通道列表取 [0] 崩溃。
            messagebox.showwarning(
                "舵机工具",
                "servo_configs.yaml 中没有注册任何舵机，无法调试。\n"
                "请先在配置文件中添加 min_pulse / max_pulse 条目。")
            return
        if self.servo is None:
            self.servo = _load_servo_controller(self.servo_pulse_configs)
        if self.servo_tool is not None and self.servo_tool.winfo_exists():
            self.servo_tool.deiconify()
            self.servo_tool.lift()
            return
        self.servo_tool = ServoToolWindow(self.root, self,
                                          on_close=self._on_servo_tool_closed)

    def _on_servo_tool_closed(self, tool):
        """ServoToolWindow 关闭时置空引用。"""
        if self.servo_tool is tool:
            self.servo_tool = None

    def _close_servo_tool(self):
        """直接销毁已打开的舵机调试工具窗口（启动时调用）。

        直接 destroy() 不会触发 WM_DELETE_WINDOW 回调，需手动置空引用。
        """
        if self.servo_tool is not None:
            try:
                self.servo_tool.destroy()
            except tk.TclError:   # 窗口可能已销毁
                pass
            self.servo_tool = None

    def _build_panels(self):
        for name in self.manager.slot_names():
            w = SlotControllerWindow(self.root, self.manager.get_slot(name),
                                     manager=self.manager,
                                     on_close=self._remove_slot_window)
            self._slot_windows.append(w)
        self.servo_panel = ServoBusPanel(self.root, self.manager,
                                         on_close=self._on_servo_panel_closed)

    # ---- 子窗口生命周期管理 ----
    def _remove_slot_window(self, w):
        """SlotControllerWindow 关闭时从列表中摘除。"""
        if w in self._slot_windows:
            self._slot_windows.remove(w)

    def _on_servo_panel_closed(self, panel):
        """ServoBusPanel 关闭时置空引用。"""
        if self.servo_panel is panel:
            self.servo_panel = None

    def _prune_slot_windows(self):
        """防御性清理：移除已被销毁（用户直接关闭）的子窗口引用。

        winfo_exists() 对已销毁 widget 返回 0 而不抛异常，
        与 WM_DELETE_WINDOW 回调构成双保险。
        """
        self._slot_windows = [w for w in self._slot_windows if w.winfo_exists()]

    def _apply_render_mode(self, mode):
        self._prune_slot_windows()
        if self.servo_panel is not None and not self.servo_panel.winfo_exists():
            self.servo_panel = None
        if mode == "debug":
            for w in self._slot_windows:
                w.deiconify()
            if self.servo_panel is not None:
                self.servo_panel.deiconify()
        else:
            for w in self._slot_windows:
                w.withdraw()
            if self.servo_panel is not None:
                self.servo_panel.withdraw()

    # ---- 逐帧循环 ----
    def _schedule_tick(self):
        self._tick_job = self.root.after(self.poll_ms, self._tick)

    def _tick(self):
        try:
            self.manager.tick()
            # 舵机下发：绑定通道取槽位最新脉宽，注册未绑定通道保持中点脉宽，
            # 未注册通道不下发（None）
            if self.servo is not None:
                degraded = self._servo_degraded
                # 降级期间不再逐帧尝试，仅周期性重试以观察是否恢复
                if not degraded or self._tick_count % SERVO_RETRY_INTERVAL == 0:
                    try:
                        self.servo.set_pulse(self.manager.get_servo_vector())
                        if degraded:
                            self._servo_degraded = False
                            print("[gui] 舵机下发已恢复")
                        self._servo_fail_streak = 0
                    except Exception as exc:  # noqa: BLE001  硬件故障不中断渲染
                        self._servo_fail_streak += 1
                        if self._servo_fail_streak >= SERVO_FAIL_STREAK_LIMIT:
                            self._servo_degraded = True
                        # 限频提示：首次与每 300 帧打印一次，避免 30ms 刷屏
                        if (self._servo_fail_streak == 1
                                or self._servo_fail_streak % 300 == 0):
                            print(f"[gui] 舵机下发失败（{exc}），"
                                  f"连续 {self._servo_fail_streak} 帧"
                                  + ("，已暂停逐帧下发" if self._servo_degraded else ""))
            # 无头模式下跳过子面板渲染，最大化节约系统资源
            if self.manager.render_mode == "debug":
                self._prune_slot_windows()
                for w in self._slot_windows:
                    w.redraw()
        except Exception:  # noqa: BLE001  单帧数据管道异常不中断渲染循环
            # 限频打印一次 traceback，避免 Tk 每帧隐式刷屏
            now = time.time()
            if now - self._last_tick_err > 2.0:
                import traceback
                traceback.print_exc()
                self._last_tick_err = now
        finally:
            self._tick_count += 1
            # 兜底：单帧异常（如子窗口竞态）不中断渲染循环，保证续排
            self._tick_job = self.root.after(self.poll_ms, self._tick)

    def stop(self):
        """停止按钮：关闭一切相关进程与窗口。"""
        self._shutdown()
        self.root.destroy()

    def on_close(self):
        self._shutdown()
        self.root.destroy()

    def _shutdown(self):
        """停止追踪进程与逐帧渲染循环。"""
        if self._tick_job is not None:
            self.root.after_cancel(self._tick_job)
            self._tick_job = None
        if self._runtime is not None:
            try:
                self._runtime.eye_tracker.stop()   # 停止眼球追踪子进程
            except Exception:  # noqa: BLE001  追踪进程可能已停止
                pass
        if self.servo is not None:
            try:
                self.servo.deinit()                # 释放 PCA9685 PWM 输出
            except Exception:  # noqa: BLE001  硬件可能已断开
                pass
            self.servo = None


def main():
    parser = argparse.ArgumentParser(
        description="LiveSuit 个人可动兽装控制系统前端（tkinter）")
    parser.add_argument("--headless", action="store_true",
                        help="以无头模式启动（隐藏并停止渲染所有子面板）")
    parser.add_argument("--poll-ms", type=int, default=POLL_MS,
                        help=f"渲染/处理轮询间隔（毫秒），默认 {POLL_MS}（≈33Hz）")
    args = parser.parse_args()

    root = tk.Tk()
    try:
        LiveSuitApp(root, poll_ms=args.poll_ms, initial_headless=args.headless)
    except Exception as exc:  # noqa: BLE001  构造期异常（如缺 tkinter 依赖）
        # 注意：缺依赖 / 无相机等眼球追踪运行时错误已不在构造期发生——
        # 真实运行时在点击「启动LiveSuit」后才加载，失败由 start() 弹窗提示并可重试
        print(f"[gui] 启动失败：{exc}")
        root.destroy()
        sys.exit(1)
    root.mainloop()


if __name__ == "__main__":
    main()
