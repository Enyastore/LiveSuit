"""数据源抽象：read() 返回 {参数全局名: 原始值}。"""


class DataSource:
    """数据源抽象：read() 返回 {参数名: 原始值}。"""

    def read(self):
        raise NotImplementedError


class CallableSource(DataSource):
    """包装任意「get_all_output()」风格的可调用对象作为数据源。"""

    def __init__(self, getter):
        self._getter = getter

    def read(self):
        data = self._getter()
        return dict(data or {})
