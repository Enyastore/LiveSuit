"""LiveSuit 管线管理器 —— 槽位可拓展的并行管道架构。

数据流拓扑（每个槽位一条并行管道，最终汇总到舵机控制总线）::

    原始数据(Slot i) -> 归一化 -> EMA滤波 -> 三次样条插值 -> 舵机控制总线

本模块是纯逻辑层（不依赖 tkinter），负责:
  1. Store            —— Zustand/Redux 风格的单数据源状态管理
                        （渲染模式 render_mode、启动态 started、舵机绑定 bindings）
  2. Normalizer       —— 原始值 -> [0, 1] 归一化
  3. Slot             —— 单个并行管道槽位（归一化 + EMA + 三次样条）
  4. PipelineManager  —— 槽位构建、逐帧处理、舵机总线映射与去重校验
  5. DataSource       —— 数据源抽象（CallableSource 对接真实提供者）
  6. 槽位配置持久化   —— alpha 与三次样条样本点读写 slot_configs.yaml

槽位注册表（有哪些输出槽位、各自的取值范围）统一由
src/servo_control/slots.py 的 get_slot_specs() 提供，本模块不硬编码任何槽位，
保证可拓展性。

底层算法复用 src/servo_control/after_process.py 中的 Filter(EMA) 与
Mapper(自然三次样条插值)。
"""

import math
import sys
from collections import OrderedDict, deque
from pathlib import Path

import yaml

# ---------------------------------------------------------------
# 路径解析：兼容本仓库布局（after_process.py 位于 src/servo_control/ 下）
# ---------------------------------------------------------------
_HERE = Path(__file__).resolve().parent            # src/
_SERVO_CONTROL = _HERE / "servo_control"           # src/servo_control/
if str(_SERVO_CONTROL) not in sys.path:
    sys.path.insert(0, str(_SERVO_CONTROL))

from after_process import Filter, Mapper  # noqa: E402  EMA / 三次样条

# ---------------------------------------------------------------
# 常量
# ---------------------------------------------------------------
SERVO_CHANNELS = 16             # 舵机总线通道数（servo_0 ~ servo_15）
DEFAULT_ALPHA = 0.85            # 默认 EMA 平滑系数
DEFAULT_NEUTRAL_ANGLE = 90.0    # 未绑定通道的中位角度

# 默认三次样条样本点：仅左右边界，即 [0,1] -> [0,180] 的线性映射
DEFAULT_POINTS = [(0.0, 0.0), (1.0, 180.0)]

# 槽位配置持久化文件（alpha + 三次样条样本点）
SLOT_CONFIGS_PATH = _HERE / "slot_configs.yaml"


class _FlowList(list):
    """YAML 行内流式列表标记：slot_configs.yaml 的 point_set 按 [[x,y],...] 写出。"""


def _flow_style_list(dumper, data):
    """把 _FlowList 序列化为行内流式风格（point_set: [[0,0],[1,180]]）。"""
    return dumper.represent_sequence("tag:yaml.org,2002:seq",
                                     list(data), flow_style=True)


yaml.SafeDumper.add_representer(_FlowList, _flow_style_list)


def read_slot_configs(path):
    """读取槽位配置文件，返回 {slot_name: {alpha, point_set}}。

    - 文件不存在 / 解析失败：返回 None（调用方按默认配置新建）；
    - 文件存在但内容为空 / 顶层不是字典：返回 {}（各槽位回退默认）。
    """
    path = Path(path)
    if not path.exists():
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as exc:  # noqa: BLE001  文件损坏 / 权限等异常按缺失处理
        print(f"[pipeline_manager] 读取槽位配置失败（{exc}），将按默认配置新建。")
        return None
    return data if isinstance(data, dict) else {}


def write_slot_configs(path, data):
    """把 {slot_name: {alpha, point_set}} 写回槽位配置文件。"""
    payload = {}
    for name, cfg in data.items():
        payload[name] = {
            "alpha": float(cfg["alpha"]),
            "point_set": _FlowList([list(map(float, p)) for p in cfg["point_set"]]),
        }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)


class Store:
    """Zustand / Redux 风格的单数据源状态容器。

    所有跨组件共享的状态集中于此:
      - render_mode : "debug"（渲染子面板） | "headless"（无头模式，隐藏并停止渲染）
      - started     : 是否已启动
      - bindings    : {slot_name: servo_index | None} 舵机绑定关系

    通过 subscribe() 订阅变更，通过 set() 浅合并更新并通知订阅者。
    """

    def __init__(self, initial=None):
        self._state = dict(initial or {})
        self._listeners = []

    def get(self, key=None, default=None):
        """读取状态：无 key 时返回整个状态快照（浅拷贝）。"""
        if key is None:
            return dict(self._state)
        return self._state.get(key, default)

    def set(self, patch):
        """浅合并更新状态并通知所有订阅者。"""
        if not patch:
            return
        self._state.update(patch)
        self._notify(dict(self._state), dict(patch))

    def subscribe(self, listener):
        """订阅状态变更，返回取消订阅函数。"""
        self._listeners.append(listener)

        def unsubscribe():
            if listener in self._listeners:
                self._listeners.remove(listener)
        return unsubscribe

    def _notify(self, state, patch):
        for fn in list(self._listeners):
            try:
                fn(state, patch)
            except Exception:  # noqa: BLE001  订阅者异常不应中断状态分发
                import traceback
                traceback.print_exc()


class Normalizer:
    """将原始值线性归一化到 [0, 1]。

    输入 in_range=(lo, hi)：raw 先钳制到 [lo, hi]，再线性映射到 [0, 1]。
    None 或非有限数值输入返回 None（表示无效，由下游 hold-last 兜底）。
    """

    def __init__(self, in_range=(-1.0, 1.0)):
        lo, hi = float(in_range[0]), float(in_range[1])
        self.lo, self.hi = lo, hi
        self._span = (hi - lo) if hi != lo else 1.0

    def map(self, raw):
        if raw is None:
            return None
        try:
            v = float(raw)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(v):
            return None
        v = min(max(v, self.lo), self.hi)
        return (v - self.lo) / self._span


class Slot:
    """单个并行管道槽位：归一化 -> EMA滤波 -> 三次样条插值 -> 角度。

    维护一个滚动历史缓冲 history（供 GUI 的 EMA 纵向时间轴图表使用）:
      每条记录为 (raw_norm, filtered)，raw_norm 可能为 None（无效采样）。
    """

    def __init__(self, name, spec=None, alpha=DEFAULT_ALPHA,
                 points=DEFAULT_POINTS, out_range=(0.0, 180.0),
                 max_history=320):
        spec = spec or {}
        self.name = name
        self.label = spec.get("label", name)
        self.normalizer = Normalizer(spec.get("in_range", (-1.0, 1.0)))
        self.out_range = (float(spec.get("out_range", out_range)[0]),
                          float(spec.get("out_range", out_range)[1]))

        self.alpha = float(alpha)
        self.filter = Filter(alpha=self.alpha)

        # 样本点以角度（0~180）存储，映射到 Mapper 时换算成归一化 y
        self._points = [list(p) for p in points]
        self.mapper = Mapper(points=None, range=self.out_range)
        self._rebuild_mapper()

        self.max_history = int(max_history)
        self.history = deque(maxlen=self.max_history)

        self.latest_raw = None        # 最新原始归一化值（可能为 None）
        self.latest_filtered = 0.0    # 最新滤波值（归一化 0~1）
        self.latest_angle = None      # 最新映射角度

    # ---- 样本点管理 ----
    @property
    def points(self):
        """返回样本点副本列表 [(x, y), ...]，y 为角度 0~180。"""
        return [tuple(p) for p in self._points]

    def set_points(self, points):
        """整体替换样本点并重建样条。"""
        self._points = [list(p) for p in points]
        self._rebuild_mapper()

    def _rebuild_mapper(self):
        """把角度样本点换算为 Mapper 的归一化 y 并重建样条。

        样本点由调用方保证合法（≥2 个且 x 严格递增，Mapper 要求）；
        非法输入直接抛 ValueError 暴露问题，不再静默回退为直线。
        """
        lo, hi = self.out_range
        span = (hi - lo) if hi != lo else 1.0
        pts = [(p[0], (p[1] - lo) / span) for p in self._points]
        self.mapper.set_points(pts)

    # ---- 滤波参数 ----
    def set_alpha(self, alpha):
        """实时更新 EMA 系数，并用当前历史重新计算滤波序列。

        这样滑动 Alpha 滑条时，纵向波形形状会立即响应（而非仅影响后续采样）。
        """
        alpha = max(0.0, min(1.0, float(alpha)))
        self.alpha = alpha
        self.filter = Filter(alpha=alpha)

        # 以首个有效原始值作为初值，避免重新滤波时出现 0 起始的过渡抖动
        seed = None
        for raw_norm, _ in self.history:
            if raw_norm is not None:
                seed = raw_norm
                break
        if seed is not None:
            self.filter.set_state(seed)

        filtered = seed if seed is not None else 0.0
        new_history = []
        for raw_norm, _ in self.history:
            if raw_norm is None:
                _, filtered = self.filter.update(None)
            else:
                _, filtered = self.filter.update(raw_norm)
            new_history.append((raw_norm, filtered))
        self.history = deque(new_history, maxlen=self.max_history)
        self.latest_filtered = filtered
        lo, hi = self.out_range
        self.latest_angle = max(lo, min(hi, self.mapper.get_result(filtered)))

    # ---- 逐帧处理 ----
    def process(self, raw):
        """处理一帧原始数据，返回最终角度。

        输出角度钳制到 out_range：三次样条在自然边界下可能过冲
        （归一化 y 越界导致角度超出 0~180），必须钳制，防止舵机
        收到超范围角度而异常运动。
        """
        norm = self.normalizer.map(raw)
        if norm is None:
            _, filtered = self.filter.update(None)
        else:
            _, filtered = self.filter.update(norm)
        angle = self.mapper.get_result(filtered)
        lo, hi = self.out_range
        angle = max(lo, min(hi, angle))

        self.latest_raw = norm
        self.latest_filtered = filtered
        self.latest_angle = angle
        self.history.append((norm, filtered))
        return angle


class DataSource:
    """数据源抽象：read() 返回 {slot_name: raw_value}。"""

    def read(self):
        raise NotImplementedError


class CallableSource(DataSource):
    """包装任意「get_all_output()」风格的可调用对象作为数据源。

    用于对接真实的 Slots 提供者（src/servo_control/slots.py）::

        from servo_control.slots import Slots, get_slot_specs
        slots = Slots(root)                          # 需传入 Tk 根窗口
        manager = PipelineManager(slots_spec=get_slot_specs(),
                                  source=CallableSource(slots.get_all_output))
    """

    def __init__(self, getter):
        self._getter = getter

    def read(self):
        data = self._getter()
        return dict(data or {})


class PipelineManager:
    """槽位可拓展的并行管道 + 舵机控制总线。

    职责:
      1. 依据槽位注册表构建各槽位（Slot）的并行管道
      2. 逐帧 tick：从数据源取原始数据 -> 各槽位并行处理 -> 汇总角度
      3. 舵机绑定：slot_name -> servo_index，去重校验（一舵机只能绑一个槽位）
      4. 通过 Store 持久化 render_mode / started / bindings

    槽位注册表由调用者传入；未传入时尝试从 servo_control/slots.py 读取默认注册。
    本类不硬编码任何具体槽位，保证可拓展性。
    """

    def __init__(self, store=None, source=None, slots_spec=None, config_path=None):
        self.store = store or Store(initial={
            "render_mode": "debug",
            "started": False,
            "bindings": {},
        })
        self.config_path = Path(config_path) if config_path else SLOT_CONFIGS_PATH
        if slots_spec is None:
            slots_spec = _load_default_slot_specs()
        self.slots_spec = OrderedDict(slots_spec) if slots_spec else OrderedDict()
        self.slots = OrderedDict()
        for name, spec in self.slots_spec.items():
            self.slots[name] = Slot(name, spec)
        if source is None:
            raise ValueError("PipelineManager 必须提供数据源 source（如 CallableSource）")
        self.source = source
        self._reverse = {}   # servo_index -> slot_name（去重快速查找）
        self._sync_reverse()

    # ---- 槽位配置持久化（slot_configs.yaml）----
    def default_slot_config(self):
        """返回全部槽位的默认配置 {name: {"alpha", "point_set"}}。"""
        return {
            name: {
                "alpha": DEFAULT_ALPHA,
                "point_set": [list(p) for p in DEFAULT_POINTS],
            }
            for name in self.slots
        }

    @staticmethod
    def _sanitize_slot_config(name, cfg, default):
        """校验 / 清洗单个槽位配置；非法字段回退默认，保证加载不抛异常。

        - alpha     ：[0,1] 内的有限浮点数；
        - point_set ：≥2 个 [x,y]，x∈[0,1] 且严格递增，y∈[0,180]。
        """
        if not isinstance(cfg, dict):
            return dict(default)
        out = dict(default)
        try:
            alpha = float(cfg.get("alpha"))
            if math.isfinite(alpha) and 0.0 <= alpha <= 1.0:
                out["alpha"] = alpha
        except (TypeError, ValueError):
            pass
        try:
            pts = [[float(a), float(b)] for a, b in (cfg.get("point_set") or [])]
        except (TypeError, ValueError):
            pts = []
        if (len(pts) >= 2
                and all(0.0 <= x <= 1.0 and 0.0 <= y <= 180.0 for x, y in pts)
                and all(pts[i + 1][0] > pts[i][0] for i in range(len(pts) - 1))):
            out["point_set"] = pts
        return out

    def _slot_to_config(self, name):
        """导出单个槽位当前配置 {alpha, point_set}（y 为角度 0~180）。"""
        slot = self.slots[name]
        return {"alpha": slot.alpha,
                "point_set": [list(p) for p in slot.points]}

    def load_slot_configs(self):
        """启动时读取 slot_configs.yaml 并应用到各槽位。

        无配置文件 / 读取失败时，按默认配置生成并新建该文件；
        文件已存在但缺失或含非法字段的槽位项，回退默认配置。
        """
        data = read_slot_configs(self.config_path)
        if data is None:
            data = self.default_slot_config()
            write_slot_configs(self.config_path, data)
        defaults = self.default_slot_config()
        for name, slot in self.slots.items():
            cfg = self._sanitize_slot_config(name, data.get(name), defaults[name])
            slot.set_points(cfg["point_set"])
            slot.set_alpha(cfg["alpha"])
        return data

    def save_slot_config(self, name=None):
        """保存槽位配置到 slot_configs.yaml。

        name 为 None 时保存全部槽位；否则仅更新该槽位，
        文件内其它槽位已保存的内容保留（合并写入）。
        """
        data = read_slot_configs(self.config_path)
        if data is None:
            data = self.default_slot_config()   # 无文件：先补全默认再覆盖目标槽位
        else:
            data = dict(data)
        if name is None:
            for slot_name in self.slots:
                data[slot_name] = self._slot_to_config(slot_name)
        else:
            data[name] = self._slot_to_config(name)
        write_slot_configs(self.config_path, data)

    def reset_slot_config(self, name):
        """把指定槽位重置为默认配置（alpha + 样本点），并同步写回配置文件。

        先 set_points 再 set_alpha：set_alpha 会以新样条重算 latest_angle，
        保证重置后槽位状态与 UI 指示完全一致。
        """
        if name not in self.slots:
            return
        default = self.default_slot_config()[name]
        slot = self.slots[name]
        slot.set_points(default["point_set"])
        slot.set_alpha(default["alpha"])
        self.save_slot_config(name)

    # ---- 状态 ----
    def _sync_reverse(self):
        self._reverse.clear()
        for name, idx in self.store.get("bindings", {}).items():
            if idx is not None and 0 <= idx < SERVO_CHANNELS:
                self._reverse[idx] = name

    @property
    def render_mode(self):
        return self.store.get("render_mode")

    @property
    def started(self):
        return self.store.get("started")

    def start(self):
        self.store.set({"started": True})

    def set_render_mode(self, mode):
        self.store.set({"render_mode": mode})

    def toggle_render_mode(self):
        mode = "headless" if self.render_mode == "debug" else "debug"
        self.store.set({"render_mode": mode})
        return mode

    # ---- 槽位 ----
    def slot_names(self):
        return list(self.slots.keys())

    def get_slot(self, name):
        return self.slots.get(name)

    def add_slot(self, name, spec=None, **slot_kwargs):
        """动态注册一个新槽位（新增一条并行管道）。"""
        self.slots[name] = Slot(name, spec or {}, **slot_kwargs)
        self.slots_spec[name] = spec or {}   # 注册表只保存 spec，勿存 Slot 实例
        return self.slots[name]

    # ---- 逐帧处理 ----
    def tick(self):
        """处理一帧：所有槽位并行走完各自管道，返回 {slot_name: angle}。"""
        raw = self.source.read()
        out = {}
        for name, slot in self.slots.items():
            out[name] = slot.process(raw.get(name))
        return out

    def get_angle_map(self):
        """返回总线输入汇总：{slot_name: latest_angle}。"""
        return {name: slot.latest_angle for name, slot in self.slots.items()}

    # ---- 舵机总线 ----
    def get_binding(self, name):
        """返回槽位绑定的舵机通道索引，未绑定返回 None。"""
        return self.store.get("bindings", {}).get(name)

    def bind(self, name, servo_index):
        """绑定 slot_name -> servo_index（servo_index 为 None 表示解绑）。

        去重校验：同一舵机通道只能绑定一个槽位，冲突时拒绝并返回提示。
        返回 (ok: bool, message: str)。
        """
        if name not in self.slots:
            return (False, f"未知槽位: {name}")

        bindings = dict(self.store.get("bindings", {}))

        if servo_index is None:
            old = bindings.get(name)
            if old is not None and self._reverse.get(old) == name:
                del self._reverse[old]
            bindings[name] = None
            self.store.set({"bindings": bindings})
            return (True, f"{name} 已解除绑定")

        if not (0 <= servo_index < SERVO_CHANNELS):
            return (False, f"非法舵机通道: {servo_index}")

        existing = self._reverse.get(servo_index)
        if existing is not None and existing != name:
            return (False, f"servo_{servo_index} 已被 {existing} 绑定，禁止重复绑定")

        # 解除该槽位旧绑定（如从 servo_1 改到 servo_0）
        old = bindings.get(name)
        if old is not None and old != servo_index and self._reverse.get(old) == name:
            del self._reverse[old]

        self._reverse[servo_index] = name
        bindings[name] = servo_index
        self.store.set({"bindings": bindings})
        return (True, f"{name} -> servo_{servo_index}")

    def get_servo_vector(self, neutral=DEFAULT_NEUTRAL_ANGLE):
        """构造 16 路舵机输出向量。

        绑定槽位的通道取该槽位最新角度，未绑定通道取 neutral（默认 90°）。
        该结果可直接交给 ServoController.set_angle() 下发。
        """
        vec = [float(neutral)] * SERVO_CHANNELS
        for name, idx in self.store.get("bindings", {}).items():
            if idx is None or not (0 <= idx < SERVO_CHANNELS):
                continue
            slot = self.slots.get(name)
            if slot is not None and slot.latest_angle is not None:
                vec[idx] = float(slot.latest_angle)
        return vec


class ServoDebugger:
    """舵机调试工具（纯逻辑层，不依赖 tkinter 与具体舵机硬件）。

    用于在启动 LiveSuit 之前手动调试单个舵机通道：
      - 16 个通道各自维护一个当前角度（默认 90° 中位）；
      - select_channel() 仅切换当前通道，不产生输出指令，并返回该通道
        当前角度（供 UI 刷新数字框，防止切换通道时误触发下发）；
      - set_angle() / adjust() 校验并钳制到 [0,180]，返回钳制后的角度；
      - get_vector() 返回 16 路完整角度向量，可直接交给
        ServoController.set_angle() 下发。

    该工具仅在启动 LiveSuit 之前可用（启动后按钮停用且已打开的窗口被关闭），
    因此其手动角度不会与管线逐帧下发发生冲突。
    """

    def __init__(self, channels=SERVO_CHANNELS, neutral=DEFAULT_NEUTRAL_ANGLE):
        self.channels = int(channels)
        self._angles = [float(neutral)] * self.channels
        self._selected = 0

    @property
    def selected(self):
        """当前选中的舵机通道索引。"""
        return self._selected

    def select_channel(self, index):
        """切换当前选中通道。

        仅切换选中通道，不产生输出指令；返回该通道当前角度，供调用方
        （gui）刷新数字框显示。
        """
        index = int(index)
        if not (0 <= index < self.channels):
            raise ValueError(f"非法舵机通道: {index}")
        self._selected = index
        return self._angles[index]

    def get_angle(self, index=None):
        """读取指定通道（缺省为当前选中通道）的角度。"""
        if index is None:
            index = self._selected
        return self._angles[int(index)]

    def set_angle(self, value, index=None):
        """设置指定通道（缺省为当前选中通道）的角度，钳制到 [0,180]。

        返回钳制后的角度。
        """
        if index is None:
            index = self._selected
        index = int(index)
        if not (0 <= index < self.channels):
            raise ValueError(f"非法舵机通道: {index}")
        angle = max(0.0, min(180.0, float(value)))
        self._angles[index] = angle
        return angle

    def adjust(self, delta, index=None):
        """在当前角度基础上增减 delta（步长为 1°），并返回调整后角度。"""
        if index is None:
            index = self._selected
        return self.set_angle(self.get_angle(index) + delta, index)

    def get_vector(self):
        """返回 16 路完整角度向量（可直接交给 ServoController.set_angle）。"""
        return list(self._angles)


def _load_default_slot_specs():
    """尝试从 servo_control/slots.py 读取默认槽位注册表。"""
    try:
        import slots  # noqa: PLC0415  servo_control/slots.py（_SERVO_CONTROL 已在 sys.path）
        return slots.get_slot_specs()
    except Exception as exc:  # noqa: BLE001  无硬件/缺依赖环境下回退为空注册表
        print(f"[pipeline_manager] 无法读取默认槽位注册表: {exc}")
        return OrderedDict()


__all__ = [
    "Store",
    "Normalizer",
    "Slot",
    "PipelineManager",
    "ServoDebugger",
    "DataSource",
    "CallableSource",
    "SERVO_CHANNELS",
    "DEFAULT_ALPHA",
    "DEFAULT_NEUTRAL_ANGLE",
    "DEFAULT_POINTS",
    "SLOT_CONFIGS_PATH",
    "read_slot_configs",
    "write_slot_configs",
]
