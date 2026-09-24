"""参数输出层：把数据源的原始值归一化为 [0, 1] 的参数流。

只有参数输出有全局名（如 a_left_eye_x）；效果器端口不在此层命名。
"""

import math


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


class ParamSource:
    """一个命名参数输出（信息流图的源节点）。

    数据来自 DataSource.read() 返回字典中的 source_key；in_range 决定归一化
    范围（归一化只发生在本层）。name 为全局名，供效果器/通道按名引用。
    """

    def __init__(self, name, source_key=None, label=None, in_range=(-1.0, 1.0)):
        self.name = name
        self.source_key = source_key if source_key is not None else name
        self.label = label if label is not None else name
        self.in_range = (float(in_range[0]), float(in_range[1]))
        self._normalizer = Normalizer(self.in_range)
        self.value = None   # 最新归一化值（可能为 None）

    def process(self, raw_dict):
        """从数据源字典取本参数原始值，返回归一化值（写入 self.value）。"""
        raw = (raw_dict or {}).get(self.source_key)
        self.value = self._normalizer.map(raw)
        return self.value
