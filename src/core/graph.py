"""效果器依赖图：拓扑排序与环检测。

节点为效果器实例名；有向边 src -> dst 表示 dst 依赖 src 的输出，
求值时必须先算 src。编排层在每次接线变化后重建拓扑序。
"""


class GraphError(Exception):
    """图操作非法（如形成环）。"""


class DependencyGraph:
    """有向无环依赖图（仅记录效果器之间的依赖）。"""

    def __init__(self):
        self._nodes = set()
        self._out = {}   # node -> set(downstream)
        self._in = {}    # node -> set(upstream)

    def add_node(self, node):
        if node in self._nodes:
            return
        self._nodes.add(node)
        self._out[node] = set()
        self._in[node] = set()

    def remove_node(self, node):
        if node not in self._nodes:
            return
        for up in list(self._in.get(node, ())):
            self._out[up].discard(node)
        for down in list(self._out.get(node, ())):
            self._in[down].discard(node)
        self._nodes.discard(node)
        self._out.pop(node, None)
        self._in.pop(node, None)

    def nodes(self):
        return list(self._nodes)

    def add_edge(self, src, dst):
        """新增 src -> dst；若会形成环则抛 GraphError，且不改变图。"""
        self.add_node(src)
        self.add_node(dst)
        if src == dst or self._reaches(dst, src):
            raise GraphError(f"接线会形成环: {src} -> {dst}")
        self._out[src].add(dst)
        self._in[dst].add(src)

    def remove_edge(self, src, dst):
        if src in self._out:
            self._out[src].discard(dst)
        if dst in self._in:
            self._in[dst].discard(src)

    def _reaches(self, start, target):
        """判断从 start 出发能否到达 target（DFS）。"""
        seen = set()
        stack = [start]
        while stack:
            node = stack.pop()
            if node == target:
                return True
            if node in seen:
                continue
            seen.add(node)
            stack.extend(self._out.get(node, ()))
        return False

    def topo_order(self):
        """返回拓扑序节点列表（Kahn 算法）；存在环时抛 GraphError。"""
        indeg = {n: len(self._in[n]) for n in self._nodes}
        queue = [n for n in self._nodes if indeg[n] == 0]
        order = []
        while queue:
            node = queue.pop()
            order.append(node)
            for down in self._out[node]:
                indeg[down] -= 1
                if indeg[down] == 0:
                    queue.append(down)
        if len(order) != len(self._nodes):
            raise GraphError("依赖图中存在环")
        return order
