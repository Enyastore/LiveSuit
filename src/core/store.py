"""Zustand / Redux 风格的单数据源状态容器（跨组件共享状态）。"""


class Store:
    """所有跨组件共享的状态集中于此：

      - render_mode : "debug"（渲染子面板） | "headless"（无头模式，隐藏并停止渲染）
      - started     : 是否已启动
      - bindings    : {channel_name: servo_index | None} 舵机绑定关系

    通过 subscribe() 订阅变更，通过 set() 浅合并更新并通知订阅者。
    注意：set() 会同步通知订阅者，仅允许在 GUI 主线程调用。
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
