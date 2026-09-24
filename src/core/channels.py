"""舵机控制层：单输入 -> 三次样条 -> 输出（未绑定归一化 / 绑定脉宽）。

未绑定舵机时输出范围为纯归一化 (0, 1)；绑定后切换为该舵机注册的
[min_pulse, max_pulse]（输出脉宽 µs）。切换范围时样本点按归一化比例重缩放，
曲线形状保持（换舵机型号无需重调）。
"""

from after_process import Mapper

# 未绑定舵机时的输出范围：纯归一化
NORMALIZED_RANGE = (0.0, 1.0)

# 默认三次样条样本点：仅左右边界，即 [0,1] -> [0,1] 线性映射
DEFAULT_POINTS = [(0.0, 0.0), (1.0, 1.0)]


class ServoChannel:
    """一个舵机控制端点：消费一条输入流，经样条映射为归一化/脉宽输出。"""

    def __init__(self, name, points=None, out_range=NORMALIZED_RANGE):
        self.name = name
        self.out_range = (float(out_range[0]), float(out_range[1]))
        self._points = [list(p) for p in (points or DEFAULT_POINTS)]
        self.mapper = Mapper(points=None, range=self.out_range)
        self._rebuild_mapper()
        self.latest_input = None
        self.latest_output = None

    # ---- 输出范围 / 样本点 ----
    def set_out_range(self, out_range):
        """切换输出范围并保持曲线形状（按归一化比例重缩放样本点）。

        绑定 / 解绑舵机时调用：未绑定传 NORMALIZED_RANGE，绑定传脉宽范围。
        """
        lo, hi = self.out_range
        nlo, nhi = float(out_range[0]), float(out_range[1])
        if (lo, hi) == (nlo, nhi) or not (nlo < nhi):
            return
        span = (hi - lo) if hi != lo else 1.0
        nspan = (nhi - nlo) if nhi != nlo else 1.0
        self._points = [[x, nlo + (y - lo) / span * nspan]
                        for x, y in self._points]
        self.out_range = (nlo, nhi)
        self._rebuild_mapper()
        if self.latest_input is not None:
            self.latest_output = self._clamp(self.mapper.get_result(
                self.latest_input))

    def curve_points(self):
        """返回样本点副本 [(x, y), ...]，y 为当前输出范围的数值。"""
        return [tuple(p) for p in self._points]

    def set_curve_points(self, points):
        """整体替换样本点并重建样条。"""
        self._points = [list(p) for p in points]
        self._rebuild_mapper()

    def map_value(self, x):
        """把样条输入 x 映射为当前输出范围的值（供画布绘制曲线）。"""
        return self._clamp(self.mapper.get_result(x))

    def _clamp(self, value):
        lo, hi = self.out_range
        return max(lo, min(hi, value))

    def _rebuild_mapper(self):
        """把输出单位样本点换算为 Mapper 的归一化 y 并重建样条。"""
        lo, hi = self.out_range
        span = (hi - lo) if hi != lo else 1.0
        pts = [(p[0], (p[1] - lo) / span) for p in self._points]
        self.mapper.range = self.out_range
        self.mapper.set_points(pts)

    # ---- 逐帧处理 ----
    def process(self, value):
        """处理一帧输入；上游无效(None)时 hold-last，返回最终输出。"""
        self.latest_input = value
        if value is None:
            return self.latest_output
        output = self._clamp(self.mapper.get_result(value))
        self.latest_output = output
        return output

    # ---- 持久化 ----
    def get_params(self):
        """返回可持久化参数：point_set 以归一化 y 存储（与输出范围无关）。"""
        lo, hi = self.out_range
        span = (hi - lo) if hi != lo else 1.0
        return {"point_set": [[x, (y - lo) / span] for x, y in self._points]}

    def set_params(self, params):
        """从归一化 point_set 恢复样本点（按当前输出范围展开）。"""
        pts = (params or {}).get("point_set")
        if not pts:
            return
        lo, hi = self.out_range
        span = (hi - lo) if hi != lo else 1.0
        self._points = [[float(x), lo + float(y) * span] for x, y in pts]
        self._rebuild_mapper()
