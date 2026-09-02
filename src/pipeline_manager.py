"""LiveSuit 管线管理器 —— 槽位可拓展的并行管道架构。

数据流拓扑（每个槽位一条并行管道，最终汇总到舵机控制总线）::

    原始数据(Slot i) -> 归一化 -> EMA滤波 -> 三次样条插值 -> 舵机控制总线（脉宽 µs）

本模块是纯逻辑层（不依赖 tkinter），负责:
  1. Store            —— Zustand/Redux 风格的单数据源状态管理
                        （渲染模式 render_mode、启动态 started、舵机绑定 bindings）
  2. Normalizer       —— 原始值 -> [0, 1] 归一化
  3. Slot             —— 单个并行管道槽位（归一化 + EMA + 三次样条）
  4. PipelineManager  —— 槽位构建、逐帧处理、舵机总线映射与去重校验
  5. DataSource       —— 数据源抽象（CallableSource 对接真实提供者）
  6. 槽位配置持久化   —— alpha 与三次样条样本点读写 slot_configs.yaml
  7. 舵机脉宽注册表   —— servo_configs.yaml（每路舵机独立的 min_pulse/max_pulse）

管线输出为「脉宽（µs）」而非角度：样条曲线以归一化形状存储，输出范围由
该槽位绑定的舵机决定（servo_configs.yaml 中注册的 min_pulse ~ max_pulse），
因此换用不同脉宽范围的舵机型号时无需重新标定曲线形状。

槽位注册表（有哪些输出槽位）统一由 src/servo_control/slots.py 的
get_slot_specs() 提供，本模块不硬编码任何槽位，保证可拓展性。

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
SERVO_CHANNELS = 16             # 舵机总线物理通道数上限（servo_0 ~ servo_15）
DEFAULT_ALPHA = 0.85            # 默认 EMA 平滑系数
DEFAULT_MIN_PULSE = 500.0       # 默认脉宽下限（µs）
DEFAULT_MAX_PULSE = 2500.0      # 默认脉宽上限（µs）
# 槽位未绑定舵机时的默认输出范围（µs）；绑定后切换为对应舵机的实际范围
DEFAULT_PULSE_RANGE = (DEFAULT_MIN_PULSE, DEFAULT_MAX_PULSE)

# 调试工具允许的绝对安全脉宽范围（µs）：运行/调试时任何通道都不允许越过
# 该硬边界（对应 test_servo.py 实测的 400~2700µs 无堵转区间）。
PULSE_SAFE_MIN = 400.0
PULSE_SAFE_MAX = 2700.0

# 默认三次样条样本点：仅左右边界，即 [0,1] -> [min_pulse,max_pulse] 的线性映射
DEFAULT_POINTS = [(0.0, DEFAULT_MIN_PULSE), (1.0, DEFAULT_MAX_PULSE)]

# 槽位配置持久化文件（alpha + 三次样条样本点；point_set 的 y 以归一化 [0,1] 存储）
SLOT_CONFIGS_PATH = _HERE / "slot_configs.yaml"

# 舵机脉宽注册表文件（每路舵机独立的 min_pulse/max_pulse，µs）
SERVO_CONFIGS_PATH = _HERE / "servo_configs.yaml"

# read_servo_configs() 返回的状态：ok=正常读取 / created=缺失新建 / replaced=非法被默认覆盖
SERVO_CONFIG_STATUS_OK = "ok"
SERVO_CONFIG_STATUS_CREATED = "created"
SERVO_CONFIG_STATUS_REPLACED = "replaced"


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


# ---------------------------------------------------------------
# 舵机脉宽注册表（servo_configs.yaml）
# ---------------------------------------------------------------

def default_servo_pulse_configs(channels=SERVO_CHANNELS):
    """返回默认舵机脉宽注册表 {索引: (min_pulse, max_pulse)}。

    默认生成 servo_0 ~ servo_{channels-1}，全部为 500~2500 µs。
    """
    return OrderedDict(
        (i, (float(DEFAULT_MIN_PULSE), float(DEFAULT_MAX_PULSE)))
        for i in range(int(channels))
    )


def write_servo_configs(path, configs):
    """把 {索引: (min_pulse, max_pulse)} 写为 servo_configs.yaml。

    文件格式：:

        servo_0:
          min_pulse: 450
          max_pulse: 2650
        servo_1:
          ...
    """
    payload = {
        f"servo_{int(idx)}": {
            "min_pulse": int(round(float(lo))),
            "max_pulse": int(round(float(hi))),
        }
        for idx, (lo, hi) in (configs or {}).items()
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)


def _parse_servo_key(key):
    """解析舵机条目键 'servo_N' -> 通道索引 N；非法返回 None。"""
    if not isinstance(key, str):
        return None
    if not key.startswith("servo_"):
        return None
    tail = key[len("servo_"):]
    if not tail.isdigit():
        return None
    return int(tail)


def read_servo_configs(path):
    """加载舵机脉宽注册表，返回 (configs, status)。

    configs 为 {通道索引: (min_pulse, max_pulse)}（仅包含合法注册项），
    status 取值见 SERVO_CONFIG_STATUS_* 常量。

    规则：
      - 文件不存在            -> 新建默认 16 路（500/2500）并写盘，status=created；
      - YAML 解析失败 / 顶层   -> 打印错误，用默认参数覆盖错误文件，status=replaced；
        非字典 / 无任何合法条目
      - 单条非法（键非 servo_N / 越界 / min>=max / 非正数）-> 丢弃该条并打印提示，
        其余合法条目照常生效（只有写入且合法的舵机才可用）。
    """
    path = Path(path)
    defaults = default_servo_pulse_configs()

    if not path.exists():
        print(f"[pipeline_manager] 未找到 {path.name}，已生成默认 16 路舵机配置。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_CREATED

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as exc:  # noqa: BLE001  文件损坏 / 权限等异常
        print(f"[pipeline_manager] 读取 {path.name} 失败（{exc}），"
              f"将用默认配置覆盖该文件。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_REPLACED

    if not isinstance(data, dict):
        print(f"[pipeline_manager] {path.name} 顶层结构非法（不是字典），"
              f"将用默认配置覆盖该文件。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_REPLACED

    configs = OrderedDict()
    for key, value in data.items():
        idx = _parse_servo_key(key)
        if idx is None:
            print(f"[pipeline_manager] 忽略非法舵机条目: {key!r}")
            continue
        if not (0 <= idx < SERVO_CHANNELS):
            print(f"[pipeline_manager] 忽略越界舵机条目: {key!r}"
                  f"（通道需在 0~{SERVO_CHANNELS - 1}）")
            continue
        if not isinstance(value, dict):
            print(f"[pipeline_manager] 忽略非法舵机条目: {key!r}（配置不是字典）")
            continue
        try:
            lo = float(value.get("min_pulse"))
            hi = float(value.get("max_pulse"))
        except (TypeError, ValueError):
            lo = hi = None
        if lo is None or not (math.isfinite(lo) and math.isfinite(hi)):
            print(f"[pipeline_manager] 忽略非法舵机条目: {key!r}（min_pulse/"
                  f"max_pulse 缺失或非数字）")
            continue
        if not (0.0 < lo < hi):
            print(f"[pipeline_manager] 忽略非法舵机条目: {key!r}"
                  f"（需要 0 < min_pulse < max_pulse，当前 {lo:g}/{hi:g}）")
            continue
        configs[idx] = (lo, hi)

    if not configs:
        print(f"[pipeline_manager] {path.name} 中没有任何有效舵机条目，"
              f"将用默认配置覆盖该文件。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_REPLACED

    return configs, SERVO_CONFIG_STATUS_OK


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
    """单个并行管道槽位：归一化 -> EMA滤波 -> 三次样条插值 -> 脉宽（µs）。

    输出范围（out_range）是当前生效的脉宽范围：未绑定舵机时为默认 500~2500，
    绑定某路舵机后切换为该舵机在 servo_configs.yaml 中注册的 [min,max]。
    切换范围时样本点按归一化比例重缩放（曲线形状保持，换舵机型号无需重调）。

    维护一个滚动历史缓冲 history（供 GUI 的 EMA 纵向时间轴图表使用）:
      每条记录为 (raw_norm, filtered)，raw_norm 可能为 None（无效采样）。
    """

    def __init__(self, name, spec=None, alpha=DEFAULT_ALPHA,
                 points=DEFAULT_POINTS, out_range=DEFAULT_PULSE_RANGE,
                 max_history=320):
        spec = spec or {}
        self.name = name
        self.label = spec.get("label", name)
        self.normalizer = Normalizer(spec.get("in_range", (-1.0, 1.0)))
        self.out_range = (float(spec.get("out_range", out_range)[0]),
                          float(spec.get("out_range", out_range)[1]))

        self.alpha = float(alpha)
        self.filter = Filter(alpha=self.alpha)

        # 样本点以当前输出范围（脉宽 µs）存储，映射到 Mapper 时换算成归一化 y
        self._points = [list(p) for p in points]
        self.mapper = Mapper(points=None, range=self.out_range)
        self._rebuild_mapper()

        self.max_history = int(max_history)
        self.history = deque(maxlen=self.max_history)

        self.latest_raw = None        # 最新原始归一化值（可能为 None）
        self.latest_filtered = 0.0    # 最新滤波值（归一化 0~1）
        self.latest_pulse = None      # 最新映射输出脉宽（µs）

    # ---- 输出范围 / 样本点管理 ----
    def set_out_range(self, out_range):
        """切换槽位输出脉宽范围并保持曲线形状（按归一化比例重缩放样本点）。

        绑定舵机 / 解绑时调用：新样本点 y' = new_lo + (y-old_lo)/span * new_span，
        保证曲线形状（归一化 y）不变，仅 Y 轴数值按新 MIN~MAX 重新标定。
        """
        lo, hi = self.out_range
        nlo, nhi = (float(out_range[0]), float(out_range[1]))
        if (lo, hi) == (nlo, nhi) or not (nlo < nhi):
            return
        span = (hi - lo) if hi != lo else 1.0
        nspan = (nhi - nlo) if nhi != nlo else 1.0
        new_points = [[x, nlo + (y - lo) / span * nspan] for x, y in self._points]
        self._points = new_points
        self.out_range = (nlo, nhi)
        self._rebuild_mapper()
        if self.latest_filtered is not None:
            self.latest_pulse = max(nlo, min(nhi,
                                             self.mapper.get_result(self.latest_filtered)))

    @property
    def points(self):
        """返回样本点副本列表 [(x, y), ...]，y 为当前输出范围的脉宽（µs）。"""
        return [tuple(p) for p in self._points]

    def set_points(self, points):
        """整体替换样本点并重建样条。"""
        self._points = [list(p) for p in points]
        self._rebuild_mapper()

    def _rebuild_mapper(self):
        """把脉宽样本点换算为 Mapper 的归一化 y 并重建样条。

        同步 Mapper 的 range 到当前 out_range；样本点由调用方保证合法
        （≥2 个且 x 严格递增，Mapper 要求）；非法输入直接抛 ValueError
        暴露问题，不再静默回退为直线。
        """
        lo, hi = self.out_range
        span = (hi - lo) if hi != lo else 1.0
        pts = [(p[0], (p[1] - lo) / span) for p in self._points]
        self.mapper.range = self.out_range
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
        self.latest_pulse = max(lo, min(hi, self.mapper.get_result(filtered)))

    # ---- 逐帧处理 ----
    def process(self, raw):
        """处理一帧原始数据，返回最终输出脉宽（µs）。

        输出脉宽钳制到 out_range：三次样条在自然边界下可能过冲
        （归一化 y 越界导致脉宽超出 MIN~MAX），必须钳制，防止舵机
        收到超范围脉宽而异常运动。
        """
        norm = self.normalizer.map(raw)
        if norm is None:
            _, filtered = self.filter.update(None)
        else:
            _, filtered = self.filter.update(norm)
        pulse = self.mapper.get_result(filtered)
        lo, hi = self.out_range
        pulse = max(lo, min(hi, pulse))

        self.latest_raw = norm
        self.latest_filtered = filtered
        self.latest_pulse = pulse
        self.history.append((norm, filtered))
        return pulse


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
    """槽位可拓展的并行管道 + 舵机控制总线（脉宽标定）。

    职责:
      1. 依据槽位注册表构建各槽位（Slot）的并行管道
      2. 逐帧 tick：从数据源取原始数据 -> 各槽位并行处理 -> 汇总脉宽（µs）
      3. 舵机绑定：slot_name -> servo_index，去重校验（一舵机只能绑一个槽位），
         且只能绑定 servo_configs.yaml 中注册过的舵机；绑定后槽位输出范围
         切换为该舵机的 [min_pulse, max_pulse]
      4. 通过 Store 持久化 render_mode / started / bindings

    槽位注册表由调用者传入；未传入时尝试从 servo_control/slots.py 读取默认注册。
    本类不硬编码任何具体槽位，保证可拓展性。
    """

    def __init__(self, store=None, source=None, slots_spec=None, config_path=None,
                 servo_pulse_configs=None):
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
        # 舵机脉宽注册表 {通道索引: (min_pulse, max_pulse)}；未传入时从
        # servo_configs.yaml 读取（gui 每次运行都会显式加载并传入）。
        if servo_pulse_configs is None:
            servo_pulse_configs, _ = read_servo_configs(SERVO_CONFIGS_PATH)
        self.servo_pulse_configs = dict(servo_pulse_configs or {})
        self._reverse = {}   # servo_index -> slot_name（去重快速查找）
        self._sync_reverse()

    # ---- 槽位配置持久化（slot_configs.yaml）----
    def default_slot_config(self):
        """返回全部槽位的默认配置 {name: {"alpha", "point_set"}}。

        point_set 的 y 为归一化值 [0,1]（与文件存储格式一致），
        应用时由 _expand_points() 展开为默认脉宽范围。
        """
        return {
            name: {
                "alpha": DEFAULT_ALPHA,
                "point_set": [[0.0, 0.0], [1.0, 1.0]],
            }
            for name in self.slots
        }

    @staticmethod
    def _expand_points(points, out_range=None):
        """把归一化样本点 [(x, y∈[0,1]), ...] 展开到脉宽范围（µs）。

        out_range 缺省为默认脉宽范围；槽位重置时应传入槽位当前 out_range
        （绑定的舵机范围），避免展开结果与槽位输出范围不一致导致曲线被削平、
        保存后无法重新加载。
        """
        lo, hi = out_range if out_range is not None else DEFAULT_PULSE_RANGE
        span = (hi - lo) if hi != lo else 1.0
        return [[float(x), lo + float(y) * span] for x, y in points]

    @staticmethod
    def _sanitize_slot_config(name, cfg, default):
        """校验 / 清洗单个槽位配置；非法字段回退默认，保证加载不抛异常。

        - alpha     ：[0,1] 内的有限浮点数；
        - point_set ：≥2 个 [x,y]，x∈[0,1] 且严格递增，y∈[0,1]（归一化存储）。
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
                and all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for x, y in pts)
                and all(pts[i + 1][0] > pts[i][0] for i in range(len(pts) - 1))):
            out["point_set"] = pts
        return out

    def _slot_to_config(self, name):
        """导出单个槽位当前配置 {alpha, point_set}（point_set 的 y 归一化为 [0,1]）。

        归一化后曲线形状与绑定舵机的脉宽范围无关，换舵机型号无需重调。
        """
        slot = self.slots[name]
        lo, hi = slot.out_range
        span = (hi - lo) if hi != lo else 1.0
        return {"alpha": slot.alpha,
                "point_set": [[x, (y - lo) / span] for x, y in slot.points]}

    def load_slot_configs(self):
        """启动时读取 slot_configs.yaml 并应用到各槽位。

        无配置文件 / 读取失败时，按默认配置生成并新建该文件；
        文件已存在但缺失或含非法字段的槽位项，回退默认配置。
        文件内 point_set 为归一化 y，加载时按默认脉宽范围展开。
        """
        data = read_slot_configs(self.config_path)
        if data is None:
            data = self.default_slot_config()
            write_slot_configs(self.config_path, data)
        defaults = self.default_slot_config()
        for name, slot in self.slots.items():
            cfg = self._sanitize_slot_config(name, data.get(name), defaults[name])
            slot.set_points(self._expand_points(cfg["point_set"]))
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

        先 set_points 再 set_alpha：set_alpha 会以新样条重算 latest_pulse，
        保证重置后槽位状态与 UI 指示完全一致。
        """
        if name not in self.slots:
            return
        default = self.default_slot_config()[name]
        slot = self.slots[name]
        # 按槽位当前输出范围（绑定的舵机脉宽范围）展开默认点，保证重置后
        # 归一化 y 仍落在 [0,1]、与 out_range 一致（否则绑定窄范围舵机时
        # 曲线顶部被削平，且保存的配置在下次加载时会被回退为默认而丢失）。
        slot.set_points(self._expand_points(default["point_set"],
                                            out_range=slot.out_range))
        slot.set_alpha(default["alpha"])
        self.save_slot_config(name)

    # ---- 状态 ----
    def _sync_reverse(self):
        self._reverse.clear()
        for name, idx in self.store.get("bindings", {}).items():
            if (idx is not None and 0 <= idx < SERVO_CHANNELS
                    and idx in self.servo_pulse_configs):
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
        """处理一帧：所有槽位并行走完各自管道，返回 {slot_name: pulse(µs)}。"""
        raw = self.source.read()
        out = {}
        for name, slot in self.slots.items():
            out[name] = slot.process(raw.get(name))
        return out

    # ---- 舵机总线 ----
    def registered_servo_indices(self):
        """返回 servo_configs.yaml 中注册过的舵机通道索引（升序）。"""
        return sorted(self.servo_pulse_configs.keys())

    def get_binding(self, name):
        """返回槽位绑定的舵机通道索引，未绑定返回 None。"""
        return self.store.get("bindings", {}).get(name)

    def bind(self, name, servo_index):
        """绑定 slot_name -> servo_index（servo_index 为 None 表示解绑）。

        去重校验：同一舵机通道只能绑定一个槽位，冲突时拒绝并返回提示；
        只能绑定 servo_configs.yaml 中注册过的舵机（未注册直接拒绝）。
        绑定成功 / 解绑后，槽位输出脉宽范围切换为该舵机的 [min,max] 或默认范围。

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
            # 先应用输出范围再通知 Store：订阅者（槽位窗口）可在同一轮
            # 通知中立即检测到范围变化并同步样条曲线。
            self.slots[name].set_out_range(DEFAULT_PULSE_RANGE)
            self.store.set({"bindings": bindings})
            return (True, f"{name} 已解除绑定")

        if not (0 <= servo_index < SERVO_CHANNELS):
            return (False, f"非法舵机通道: {servo_index}")

        pulse_range = self.servo_pulse_configs.get(servo_index)
        if pulse_range is None:
            return (False, f"servo_{servo_index} 未在 servo_configs.yaml 中注册，禁止绑定")

        existing = self._reverse.get(servo_index)
        if existing is not None and existing != name:
            return (False, f"servo_{servo_index} 已被 {existing} 绑定，禁止重复绑定")

        # 解除该槽位旧绑定（如从 servo_1 改到 servo_0）
        old = bindings.get(name)
        if old is not None and old != servo_index and self._reverse.get(old) == name:
            del self._reverse[old]

        self._reverse[servo_index] = name
        bindings[name] = servo_index
        # 先应用输出范围再通知 Store（见解绑分支的说明）
        self.slots[name].set_out_range(pulse_range)
        self.store.set({"bindings": bindings})
        return (True, f"{name} -> servo_{servo_index}（{pulse_range[0]:.0f}~"
                      f"{pulse_range[1]:.0f}µs）")

    def get_servo_vector(self):
        """构造 16 路舵机脉宽输出向量（µs）。

        - 注册但未绑定槽位的通道：取该舵机脉宽范围的中点 (min+max)/2；
        - 绑定槽位的通道：取该槽位最新输出脉宽（已按该舵机范围钳制）；
        - 未在 servo_configs.yaml 中注册的通道：为 None（表示不写）。

        结果可直接交给 ServoController.set_pulse() 下发。
        """
        vec = [None] * SERVO_CHANNELS
        for idx, (lo, hi) in self.servo_pulse_configs.items():
            if 0 <= idx < SERVO_CHANNELS:
                vec[idx] = (lo + hi) / 2.0   # 注册未绑定 -> 中点脉宽
        for name, idx in self.store.get("bindings", {}).items():
            if idx is None or idx not in self.servo_pulse_configs:
                continue
            slot = self.slots.get(name)
            if slot is not None and slot.latest_pulse is not None:
                vec[idx] = float(slot.latest_pulse)
        return vec


class ServoDebugger:
    """舵机调试工具（纯逻辑层，不依赖 tkinter 与具体舵机硬件）。

    用于在启动 LiveSuit 之前手动调试单个舵机通道（脉宽标定）：
      - 仅维护 servo_configs.yaml 中注册过的通道，每路以自身 [min_pulse,
        max_pulse] 为「推荐范围」（初始值取中点脉宽）；
      - select_channel() 仅切换当前通道，不产生输出指令，并返回该通道
        当前脉宽（供 UI 刷新数字框，防止切换通道时误触发下发）；
      - set_pulse() / adjust() 允许设置推荐范围之外的脉宽（便于实测找
        最佳值写回配置文件），但硬钳制到全局安全范围 [400, 2700]µs；
      - is_out_of_range() 判断当前脉宽是否超出该通道推荐范围（供 UI
        红色警示舵机损坏风险）；
      - get_vector() 返回 16 路完整脉宽向量（未注册通道为 None），
        可直接交给 ServoController.set_pulse() 下发。

    该工具仅在启动 LiveSuit 之前可用（启动后按钮停用且已打开的窗口被关闭），
    因此其手动脉宽不会与管线逐帧下发发生冲突。
    """

    def __init__(self, pulse_configs=None, channels=SERVO_CHANNELS):
        self.channels = int(channels)
        # 只登记合法注册通道：{索引: (min_pulse, max_pulse)}
        self.pulse_configs = {}
        for idx, (lo, hi) in dict(pulse_configs or {}).items():
            idx = int(idx)
            if 0 <= idx < self.channels and 0.0 < lo < hi:
                self.pulse_configs[idx] = (float(lo), float(hi))
        # 每路舵机初始为自身范围中点
        self._pulses = {idx: (lo + hi) / 2.0
                        for idx, (lo, hi) in self.pulse_configs.items()}
        self._selected = min(self.pulse_configs) if self.pulse_configs else 0

    def registered_indices(self):
        """返回已注册舵机通道索引（升序）。"""
        return sorted(self.pulse_configs.keys())

    def select_channel(self, index):
        """切换当前选中通道。

        仅切换选中通道，不产生输出指令；返回该通道当前脉宽，供调用方
        （gui）刷新数字框显示。
        """
        index = int(index)
        if index not in self.pulse_configs:
            raise ValueError(f"舵机通道未注册或非法: {index}")
        self._selected = index
        return self._pulses[index]

    def get_pulse(self, index=None):
        """读取指定通道（缺省为当前选中通道）的脉宽（µs）。"""
        if index is None:
            index = self._selected
        index = int(index)
        if index not in self.pulse_configs:
            raise ValueError(f"舵机通道未注册或非法: {index}")
        return self._pulses[index]

    def set_pulse(self, value, index=None):
        """设置指定通道（缺省为当前选中通道）的脉宽，硬钳制到全局安全范围。

        允许设置 servo_configs.yaml 推荐范围（[min,max]）之外的数值，便于
        实测找最佳脉宽写回配置文件；但不可超过安全范围 [400, 2700]µs，
        防止机械堵转损坏舵机。

        返回钳制后的脉宽（µs）。
        """
        if index is None:
            index = self._selected
        index = int(index)
        if index not in self.pulse_configs:
            raise ValueError(f"舵机通道未注册或非法: {index}")
        pulse = max(PULSE_SAFE_MIN, min(PULSE_SAFE_MAX, float(value)))
        self._pulses[index] = pulse
        return pulse

    def adjust(self, delta, index=None):
        """在当前脉宽基础上增减 delta（步长为 1µs），并返回调整后脉宽。"""
        if index is None:
            index = self._selected
        return self.set_pulse(self.get_pulse(index) + delta, index)

    def is_out_of_range(self, index=None):
        """判断指定通道（缺省为当前选中通道）的当前脉宽是否超出推荐范围。

        返回 True 表示当前值在 servo_configs.yaml 注册的 [min,max] 之外，
        供 UI 红色警示可能存在舵机损坏风险。
        """
        if index is None:
            index = self._selected
        index = int(index)
        if index not in self.pulse_configs:
            raise ValueError(f"舵机通道未注册或非法: {index}")
        lo, hi = self.pulse_configs[index]
        pulse = self._pulses[index]
        return not (lo <= pulse <= hi)

    def config_range(self, index=None):
        """返回指定通道（缺省为当前选中通道）的推荐脉宽范围 (min, max)。"""
        if index is None:
            index = self._selected
        return self.pulse_configs[int(index)]

    def get_vector(self):
        """返回 16 路完整脉宽向量（未注册通道为 None，可直接交给 set_pulse）。"""
        vec = [None] * self.channels
        for idx, pulse in self._pulses.items():
            vec[idx] = pulse
        return vec


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
    "DEFAULT_MIN_PULSE",
    "DEFAULT_MAX_PULSE",
    "DEFAULT_PULSE_RANGE",
    "PULSE_SAFE_MIN",
    "PULSE_SAFE_MAX",
    "DEFAULT_POINTS",
    "SLOT_CONFIGS_PATH",
    "SERVO_CONFIGS_PATH",
    "SERVO_CONFIG_STATUS_OK",
    "SERVO_CONFIG_STATUS_CREATED",
    "SERVO_CONFIG_STATUS_REPLACED",
    "read_slot_configs",
    "write_slot_configs",
    "read_servo_configs",
    "write_servo_configs",
    "default_servo_pulse_configs",
]
