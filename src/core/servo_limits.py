"""舵机脉宽限定注册表（servo_configs.yaml）与调试工具。

servo_configs.yaml 结构：顶层 `global_*` 定义全局默认，`servo_N` 条目只写与
全局不同的字段（留空即继承）。每路有效限定为 ServoLimits：
  - min_pulse / max_pulse  机械行程（绑定后通道输出的脉宽范围）
  - safe_pulse_min / max   安全提示范围（调试告警/钳制用，非硬边界）
"""

import math
from collections import OrderedDict, namedtuple
from pathlib import Path

import yaml

SERVO_CHANNELS = 16   # 舵机总线物理通道数上限（servo_0 ~ servo_15）

# 生成默认 servo_configs.yaml 时使用的占位值（bootstrap）。
_BOOTSTRAP_MIN_PULSE = 500.0
_BOOTSTRAP_MAX_PULSE = 2500.0
_BOOTSTRAP_SAFE_PULSE_MIN = 400.0
_BOOTSTRAP_SAFE_PULSE_MAX = 2700.0

GLOBAL_MIN_PULSE_KEY = "global_min_pulse"
GLOBAL_MAX_PULSE_KEY = "global_max_pulse"
GLOBAL_SAFE_PULSE_MIN_KEY = "global_safe_pulse_min"
GLOBAL_SAFE_PULSE_MAX_KEY = "global_safe_pulse_max"
_GLOBAL_KEYS = (GLOBAL_MIN_PULSE_KEY, GLOBAL_MAX_PULSE_KEY,
                GLOBAL_SAFE_PULSE_MIN_KEY, GLOBAL_SAFE_PULSE_MAX_KEY)

ServoLimits = namedtuple(
    "ServoLimits", "min_pulse max_pulse safe_pulse_min safe_pulse_max")

_BOOTSTRAP_LIMITS = ServoLimits(_BOOTSTRAP_MIN_PULSE, _BOOTSTRAP_MAX_PULSE,
                                _BOOTSTRAP_SAFE_PULSE_MIN,
                                _BOOTSTRAP_SAFE_PULSE_MAX)

_SRC_DIR = Path(__file__).resolve().parent.parent
SERVO_CONFIGS_PATH = _SRC_DIR / "servo_configs.yaml"

SERVO_CONFIG_STATUS_OK = "ok"
SERVO_CONFIG_STATUS_CREATED = "created"
SERVO_CONFIG_STATUS_REPLACED = "replaced"


def default_servo_pulse_configs(channels=SERVO_CHANNELS):
    """返回默认舵机脉宽注册表 {索引: ServoLimits}（仅用于生成默认文件）。"""
    return OrderedDict((i, _BOOTSTRAP_LIMITS) for i in range(int(channels)))


def _as_float(value):
    """把值转为有限浮点数；失败返回 None。"""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _resolve_global_limits(data):
    """解析顶层 global_* 全局默认；缺失回退 bootstrap，非法则告警后回退。"""
    values = []
    for key, fallback in zip(_GLOBAL_KEYS, _BOOTSTRAP_LIMITS):
        if key not in data:
            values.append(fallback)
            continue
        v = _as_float(data.get(key))
        if v is None:
            print(f"[servo_limits] 全局默认 {key} 非法，回退 {fallback:g}")
            values.append(fallback)
        else:
            values.append(v)
    limits = ServoLimits(*values)
    if not (limits.min_pulse < limits.max_pulse):
        print(f"[servo_limits] {GLOBAL_MIN_PULSE_KEY} 需小于 "
              f"{GLOBAL_MAX_PULSE_KEY}，回退默认")
        limits = limits._replace(min_pulse=_BOOTSTRAP_LIMITS.min_pulse,
                                 max_pulse=_BOOTSTRAP_LIMITS.max_pulse)
    if not (limits.safe_pulse_min < limits.safe_pulse_max):
        print(f"[servo_limits] {GLOBAL_SAFE_PULSE_MIN_KEY} 需小于 "
              f"{GLOBAL_SAFE_PULSE_MAX_KEY}，回退默认")
        limits = limits._replace(
            safe_pulse_min=_BOOTSTRAP_LIMITS.safe_pulse_min,
            safe_pulse_max=_BOOTSTRAP_LIMITS.safe_pulse_max)
    return limits


def _resolve_field(value, field, fallback, key):
    """从通道条目取一个字段；缺失返回 fallback，非法则告警后回退 fallback。"""
    if field not in value:
        return fallback
    v = _as_float(value.get(field))
    if v is None:
        print(f"[servo_limits] {key} 的 {field} 非法，回退 {fallback:g}")
        return fallback
    return v


def _resolve_channel_limits(value, global_limits, key):
    """解析单个通道条目：缺失字段继承全局；非法仅告警，不丢弃通道。"""
    if value is None:
        value = {}
    if not isinstance(value, dict):
        print(f"[servo_limits] {key!r} 配置不是字典，按全局默认注册该通道")
        value = {}
    g = global_limits
    min_p = _resolve_field(value, "min_pulse", g.min_pulse, key)
    max_p = _resolve_field(value, "max_pulse", g.max_pulse, key)
    if not (min_p < max_p):
        print(f"[servo_limits] {key} 的 min_pulse 需小于 max_pulse，"
              f"回退全局默认 {g.min_pulse:g}~{g.max_pulse:g}")
        min_p, max_p = g.min_pulse, g.max_pulse
    s_min = _resolve_field(value, "safe_pulse_min", g.safe_pulse_min, key)
    s_max = _resolve_field(value, "safe_pulse_max", g.safe_pulse_max, key)
    if not (s_min < s_max):
        print(f"[servo_limits] {key} 的 safe_pulse_min 需小于 "
              f"safe_pulse_max，回退全局默认")
        s_min, s_max = g.safe_pulse_min, g.safe_pulse_max
    if not (s_min <= min_p <= max_p <= s_max):
        print(f"[servo_limits] 提示：{key} 的安全范围 {s_min:g}~{s_max:g} "
              f"未完整包住机械范围 {min_p:g}~{max_p:g}")
    return ServoLimits(min_p, max_p, s_min, s_max)


def write_servo_configs(path, configs, global_limits=None):
    """把 {索引: ServoLimits} 写为 servo_configs.yaml（每路显式列出全部字段）。"""
    g = global_limits or _BOOTSTRAP_LIMITS
    payload = {
        GLOBAL_MIN_PULSE_KEY: int(round(g.min_pulse)),
        GLOBAL_MAX_PULSE_KEY: int(round(g.max_pulse)),
        GLOBAL_SAFE_PULSE_MIN_KEY: int(round(g.safe_pulse_min)),
        GLOBAL_SAFE_PULSE_MAX_KEY: int(round(g.safe_pulse_max)),
    }
    for idx, lim in (configs or {}).items():
        payload[f"servo_{int(idx)}"] = {
            "min_pulse": int(round(lim.min_pulse)),
            "max_pulse": int(round(lim.max_pulse)),
            "safe_pulse_min": int(round(lim.safe_pulse_min)),
            "safe_pulse_max": int(round(lim.safe_pulse_max)),
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

    configs 为 {通道索引: ServoLimits}（仅合法注册项），status 取值见
    SERVO_CONFIG_STATUS_*。规则见模块 docstring 与 AGENTS 约定。
    """
    path = Path(path)
    defaults = default_servo_pulse_configs()

    if not path.exists():
        print(f"[servo_limits] 未找到 {path.name}，已生成默认 16 路舵机配置。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_CREATED

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as exc:  # noqa: BLE001  文件损坏 / 权限等异常
        print(f"[servo_limits] 读取 {path.name} 失败（{exc}），"
              f"将用默认配置覆盖该文件。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_REPLACED

    if not isinstance(data, dict):
        print(f"[servo_limits] {path.name} 顶层结构非法（不是字典），"
              f"将用默认配置覆盖该文件。")
        write_servo_configs(path, defaults)
        return defaults, SERVO_CONFIG_STATUS_REPLACED

    global_limits = _resolve_global_limits(data)
    configs = OrderedDict()
    for key, value in data.items():
        if key in _GLOBAL_KEYS:
            continue
        idx = _parse_servo_key(key)
        if idx is None:
            print(f"[servo_limits] 忽略未知顶层条目: {key!r}")
            continue
        if not (0 <= idx < SERVO_CHANNELS):
            print(f"[servo_limits] 忽略越界舵机条目: {key!r}"
                  f"（通道需在 0~{SERVO_CHANNELS - 1}）")
            continue
        configs[idx] = _resolve_channel_limits(value, global_limits, key)

    if not configs:
        print(f"[servo_limits] {path.name} 中没有任何舵机条目，"
              f"将用默认配置覆盖该文件。")
        write_servo_configs(path, defaults, global_limits)
        return defaults, SERVO_CONFIG_STATUS_REPLACED

    return configs, SERVO_CONFIG_STATUS_OK


class ServoDebugger:
    """舵机调试工具（纯逻辑层，不依赖 tkinter 与具体舵机硬件）。

    仅维护已注册通道，每路以自身 [min_pulse, max_pulse] 为推荐范围
    （初始值取中点）；set_pulse()/adjust() 允许超出推荐范围，但钳制到该通道
    的安全提示范围。该工具仅在启动 LiveSuit 之前可用。
    """

    def __init__(self, pulse_configs=None, channels=SERVO_CHANNELS):
        self.channels = int(channels)
        self.pulse_configs = {}      # 索引 -> 推荐机械范围 (min, max)
        self.safe_limits = {}        # 索引 -> 安全提示范围 (safe_min, safe_max)
        for idx, lim in dict(pulse_configs or {}).items():
            try:
                idx = int(idx)
                if isinstance(lim, (tuple, list)):
                    if len(lim) >= 4:
                        mp, xp, sm, sx = (float(v) for v in lim[:4])
                    else:
                        mp, xp = float(lim[0]), float(lim[1])
                        sm, sx = mp, xp
                else:
                    mp, xp = float(lim.min_pulse), float(lim.max_pulse)
                    sm, sx = float(lim.safe_pulse_min), float(lim.safe_pulse_max)
            except (TypeError, ValueError, IndexError, AttributeError):
                continue
            if 0 <= idx < self.channels and 0.0 < mp < xp:
                self.pulse_configs[idx] = (mp, xp)
                self.safe_limits[idx] = (sm, sx) if 0.0 < sm < sx else (mp, xp)
        self._pulses = {idx: (mp + xp) / 2.0
                        for idx, (mp, xp) in self.pulse_configs.items()}
        self._selected = min(self.pulse_configs) if self.pulse_configs else 0

    def registered_indices(self):
        """返回已注册舵机通道索引（升序）。"""
        return sorted(self.pulse_configs.keys())

    def select_channel(self, index):
        """切换当前选中通道（不产生输出指令），返回该通道当前脉宽。"""
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
        """设置指定通道脉宽，钳制到该通道安全范围，返回钳制后的值。"""
        if index is None:
            index = self._selected
        index = int(index)
        if index not in self.pulse_configs:
            raise ValueError(f"舵机通道未注册或非法: {index}")
        s_lo, s_hi = self.safe_limits[index]
        pulse = max(s_lo, min(s_hi, float(value)))
        self._pulses[index] = pulse
        return pulse

    def adjust(self, delta, index=None):
        """在当前脉宽基础上增减 delta（步长为 1µs），返回调整后脉宽。"""
        if index is None:
            index = self._selected
        return self.set_pulse(self.get_pulse(index) + delta, index)

    def is_out_of_range(self, index=None):
        """判断当前脉宽是否超出该通道推荐范围（供 UI 红色警示）。"""
        if index is None:
            index = self._selected
        index = int(index)
        if index not in self.pulse_configs:
            raise ValueError(f"舵机通道未注册或非法: {index}")
        lo, hi = self.pulse_configs[index]
        return not (lo <= self._pulses[index] <= hi)

    def config_range(self, index=None):
        """返回指定通道的推荐脉宽范围 (min, max)。"""
        if index is None:
            index = self._selected
        return self.pulse_configs[int(index)]

    def safe_range(self, index=None):
        """返回指定通道的安全提示范围 (min, max)。"""
        if index is None:
            index = self._selected
        return self.safe_limits[int(index)]

    def get_vector(self):
        """返回 16 路完整脉宽向量（未注册通道为 None）。"""
        vec = [None] * self.channels
        for idx, pulse in self._pulses.items():
            vec[idx] = pulse
        return vec
