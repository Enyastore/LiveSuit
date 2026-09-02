import numpy as np
from bisect import bisect_right


class Filter:
    """简单的指数移动平均（EMA）滤波器。

    update() 返回 (value, after) 二元组：
        value —— 原始输入值；输入为 None 时保持上一次有效原值（hold-last）
        after —— EMA 滤波后的值；输入为 None 时保持上一次滤波值

    输入为 None 时不更新内部状态，直接返回上一次的有效值，
    避免下游对 None 做算术运算而崩溃，并保证最终输出曲线平滑。
    """
    def __init__(self, alpha: float = 0.1, initial_value: float = 0.0):
        self.alpha = alpha
        self.previous = initial_value        # 上一次滤波输出
        self._last_value = initial_value     # 上一次有效输入原值

    def update(self, value):
        """更新滤波器状态并返回 (value, after)；None 输入时状态不变。"""
        if value is None:
            return (self._last_value, self.previous)
        after = self.alpha * value + (1 - self.alpha) * self.previous
        self.previous = after
        self._last_value = value
        return (value, after)

    def set_state(self, value):
        """重置滤波器内部状态：以 value 同时作为上次有效输入与上次滤波输出。

        用于 EMA 系数变更后以历史初值重新起算，避免出现 0 起始的过渡抖动。
        """
        self.previous = value
        self._last_value = value

class Mapper:
    """三阶样条插值映射器（自然三次样条，numpy 实现）。

    通过样本点建立三次样条曲线，将输入 x 映射到归一化的 y（约定取值 [0, 1]），
    再通过 range 元组线性映射到实际输出范围（如舵机脉宽 (500, 2500)，µs）。

    样本点结构为元组套列表 [(x_0, y_0), (x_1, y_1), ..., (x_i, y_i)]，
    实例化后可通过 set_points() 修改样本点并自动重建样条。
    输入超出样本点范围时钳制到边界值（不外推）。
    """

    def __init__(self, points=None, range=None):
        """初始化映射器。

        Parameters
        ----------
        points : list[tuple[float, float]] | None
            样本点列表，形如 [(x_0, y_0), (x_1, y_1), ...]。
            y 为归一化值（通常取 [0, 1]），默认 [(0.0, 0.0), (1.0, 1.0)]。
        range : tuple[float, float] | None
            输出范围 (lo, hi)。样条求得的归一化 y 会线性映射到该范围，
            即最终输出 = lo + y * (hi - lo)。缺省即默认舵机脉宽范围
            (500, 2500)（µs）。
        """
        if range is None:
            range = (500.0, 2500.0)   # 默认舵机脉宽范围（µs）
        self._xs_list = [0.0, 1.0]
        self._coeffs = np.zeros((1, 4))
        self._points = []
        self.range = (float(range[0]), float(range[1]))
        if points is None:
            points = [(0.0, 0.0), (1.0, 1.0)]
        self.set_points(points)

    def set_points(self, points) -> None:
        """设置/更新样本点并重建样条。

        Parameters
        ----------
        points : list[tuple[float, float]]
            形如 [(x_0, y_0), (x_1, y_1), ..., (x_i, y_i)] 的样本点列表。
            y 为归一化值（通常取 [0, 1]）。
            至少需要 2 个点；内部会按 x 坐标升序排序，且要求 x 严格递增。

        Raises
        ------
        ValueError
            样本点少于 2 个，或存在重复/非递增的 x 坐标。
        """
        if len(points) < 2:
            raise ValueError("样本点至少需要 2 个")
        pts = sorted(points, key=lambda p: p[0])
        xs = np.array([p[0] for p in pts], dtype=float)
        ys = np.array([p[1] for p in pts], dtype=float)
        if np.any(np.diff(xs) <= 0):
            raise ValueError("样本点 x 坐标必须严格递增（不允许重复 x）")
        self._points = pts
        self._xs_list = xs.tolist()
        self._coeffs = self._build_coeffs(xs, ys)

    @staticmethod
    def _build_coeffs(xs, ys):
        """求解自然三次样条，返回每段的局部多项式系数。

        区间 [x_i, x_{i+1}] 内的样条为:
            S(x) = a + b*t + c*t^2 + d*t^3，其中 t = x - x_i
        返回系数数组 shape (n-1, 4)，第 i 行为区间 i 的 (a, b, c, d)。
        """
        n = len(xs)
        # 求解各节点二阶导数 y2（自然边界：两端 y2 = 0）
        y2 = np.zeros(n)
        u = np.zeros(n)
        for i in range(1, n - 1):
            sig = (xs[i] - xs[i - 1]) / (xs[i + 1] - xs[i - 1])
            p = sig * y2[i - 1] + 2.0
            y2[i] = (sig - 1.0) / p
            u[i] = ((ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i])
                    - (ys[i] - ys[i - 1]) / (xs[i] - xs[i - 1]))
            u[i] = (6.0 * u[i] / (xs[i + 1] - xs[i - 1]) - sig * u[i - 1]) / p
        for i in range(n - 2, -1, -1):
            y2[i] = y2[i] * y2[i + 1] + u[i]

        h = np.diff(xs)
        coeffs = np.zeros((n - 1, 4))
        coeffs[:, 0] = ys[:-1]                                    # a = y_i
        coeffs[:, 1] = (ys[1:] - ys[:-1]) / h \
            - h * (y2[1:] + 2.0 * y2[:-1]) / 6.0                  # b
        coeffs[:, 2] = y2[:-1] / 2.0                              # c = y2_i / 2
        coeffs[:, 3] = (y2[1:] - y2[:-1]) / (6.0 * h)             # d
        return coeffs

    def get_result(self, x: float) -> float:
        """将输入值 x 映射为输出值 y。

        Parameters
        ----------
        x : float
            输入值，通常在 [0, 1] 区间。

        Returns
        -------
        float
            最终映射到 self.range 的输出值（如 range=(500, 2500) 时输出脉宽 µs）。
            输入超出样本点范围时钳制到边界值（不外推）。
        """
        x = float(x)
        x0 = self._xs_list[0]
        x1 = self._xs_list[-1]
        if x < x0:
            x = x0
        elif x > x1:
            x = x1
        k = bisect_right(self._xs_list, x) - 1
        if k < 0:
            k = 0
        elif k > len(self._xs_list) - 2:
            k = len(self._xs_list) - 2
        a, b, c, d = self._coeffs[k]
        t = x - self._xs_list[k]
        y = a + t * (b + t * (c + t * d))  # 归一化样条值（约定 [0, 1]）
        lo, hi = self.range
        return float(lo + (hi - lo) * y)
