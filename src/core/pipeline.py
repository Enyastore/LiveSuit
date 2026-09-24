"""编排层：实例化三层对象、维护接线图、逐帧求值与舵机绑定。

职责：
  - 依据参数输出层规格（servo_control/slots.py）构建 ParamSource；
  - 实例化效果器（类型注册表注入）与舵机通道，维护二者之间的接线；
  - 维护效果器依赖图（拓扑序 / 环检测）；
  - tick() 逐帧求值；get_servo_vector() 汇总脉宽；
  - pipeline.yaml 持久化。

线程：所有可变状态由 self._lock 保护。tick() 由数据线程调用；create/connect/
bind/save 由 GUI 主线程调用。本模块不 import tkinter。
"""

import threading
from collections import OrderedDict
from pathlib import Path

from core import config_io
from core.channels import ServoChannel, NORMALIZED_RANGE
from core.effectors import Effector
from core.graph import DependencyGraph, GraphError
from core.params import ParamSource
from core.servo_limits import (SERVO_CHANNELS, SERVO_CONFIGS_PATH,
                               read_servo_configs)
from core.sources import CallableSource
from core.store import Store


class PipelineManager:
    """信息流编排层：参数输出 -> 效果器图 -> 舵机通道。"""

    def __init__(self, store=None, source=None, param_specs=None,
                 effector_types=None, config_path=None, servo_limits=None):
        self.store = store or Store(initial={
            "render_mode": "debug",
            "started": False,
            "bindings": {},
        })
        self._source = source if source is not None else CallableSource(lambda: {})
        self._effector_types = dict(effector_types or {})
        self._config_path = (Path(config_path) if config_path
                             else config_io.PIPELINE_PATH)
        self._lock = threading.RLock()

        # 参数输出层：只能有全局名，来源为 slots.get_slot_specs()
        self.params = OrderedDict()
        for name, spec in (param_specs or {}).items():
            spec = spec or {}
            self.params[name] = ParamSource(
                name,
                source_key=spec.get("source_key", name),
                label=spec.get("label", name),
                in_range=spec.get("in_range", (-1.0, 1.0)),
            )

        # 舵机脉宽限定注册表
        if servo_limits is None:
            servo_limits, _ = read_servo_configs(SERVO_CONFIGS_PATH)
        self.servo_limits = OrderedDict(servo_limits or {})

        # 效果器层 / 舵机控制层
        self.effectors = OrderedDict()
        self.channels = OrderedDict()
        self._eff_name_of = {}          # Effector 实例 -> 实例名
        self._effector_inputs = {}      # 实例名 -> [source_ref | None]
        self._channel_inputs = {}       # 通道名 -> source_ref | None
        self._bindings = {}             # 通道名 -> servo_index | None
        self._reverse = {}              # servo_index -> 通道名
        self._dep = DependencyGraph()
        self._order = []                # 效果器拓扑序（实例名）
        self._latest = {}               # source_ref -> 最新值（快照用）
        self._suppress_save = False     # 构建/载入期间禁止自动写盘

        data = config_io.read_pipeline(self._config_path)
        need_save = False
        self._suppress_save = True
        try:
            if data is None:
                self._build_default_graph()
                need_save = True
            else:
                try:
                    self._load_config(data)
                except Exception as exc:  # noqa: BLE001  配置损坏时回退默认图
                    print(f"[pipeline] 载入 pipeline.yaml 失败（{exc}），改用默认编排")
                    self._reset_graph()
                    self._build_default_graph()
                    need_save = True
        finally:
            self._suppress_save = False
        if need_save:
            try:
                self.save_pipeline()
            except Exception as exc:  # noqa: BLE001  写盘失败不阻塞启动
                print(f"[pipeline] 生成默认 pipeline.yaml 失败（{exc}）")

    def _reset_graph(self):
        """清空效果器/通道/接线（载入失败回退时使用）。"""
        self.effectors.clear()
        self._eff_name_of.clear()
        self._effector_inputs.clear()
        self._channel_inputs.clear()
        self.channels.clear()
        self._bindings.clear()
        self._reverse.clear()
        self._dep = DependencyGraph()
        self._order = []

    # --------------------------------------------------------
    # 参数 / 只读访问
    # --------------------------------------------------------
    def param_names(self):
        return list(self.params.keys())

    def get_params(self):
        return list(self.params.values())

    def get_effectors(self):
        return list(self.effectors.values())

    def get_channels(self):
        return list(self.channels.values())

    def effector_name(self, ref):
        """返回效果器实例的编排名（供 GUI 显示）。"""
        return self._eff_name_of.get(ref)

    def get_connections(self):
        """返回所有连线 [(源, 目标), ...]，源/目标为 source_ref 形式。"""
        with self._lock:
            conns = []
            for name, inputs in self._effector_inputs.items():
                for port, ref in enumerate(inputs):
                    if ref is not None:
                        conns.append((ref, ("effector", name, port)))
            for name, ref in self._channel_inputs.items():
                if ref is not None:
                    conns.append((ref, ("channel", name, 0)))
            return conns

    # --------------------------------------------------------
    # 编排操作（GUI 主线程）
    # --------------------------------------------------------
    def _unique_name(self, mapping, base):
        if base not in mapping:
            return base
        i = 2
        while f"{base}_{i}" in mapping:
            i += 1
        return f"{base}_{i}"

    def create_effector(self, type_name, name=None):
        """按注册表类型实例化一个效果器，加入图并返回其引用。"""
        with self._lock:
            cls = self._effector_types.get(type_name)
            if cls is None:
                raise ValueError(f"未注册的效果器类型: {type_name}")
            eff = cls()
            eff_name = self._unique_name(self.effectors, name or type_name)
            self.effectors[eff_name] = eff
            self._eff_name_of[eff] = eff_name
            self._effector_inputs[eff_name] = [None] * eff.get_input_count()
            self._dep.add_node(eff_name)
            self._rebuild_order()
            self._autosave()
            return eff

    def remove_effector(self, ref):
        """移除效果器，并清空引用它的接线。"""
        with self._lock:
            name = self._eff_name_of.pop(ref, None)
            if name is None:
                return
            self.effectors.pop(name, None)
            self._effector_inputs.pop(name, None)
            for inputs in self._effector_inputs.values():
                for port, r in enumerate(inputs):
                    if r is not None and r[0] == "effector" and r[1] == name:
                        inputs[port] = None
            for cname, r in list(self._channel_inputs.items()):
                if r is not None and r[0] == "effector" and r[1] == name:
                    self._channel_inputs[cname] = None
            self._rebuild_order()
            self._autosave()

    def add_channel(self, name=None, points=None):
        """新增一个舵机通道（汇节点），返回其引用。"""
        with self._lock:
            cname = self._unique_name(self.channels, name or "channel")
            ch = ServoChannel(cname, points=points)
            self.channels[cname] = ch
            self._channel_inputs[cname] = None
            self._autosave()
            return ch

    def remove_channel(self, name):
        with self._lock:
            self.channels.pop(name, None)
            self._channel_inputs.pop(name, None)
            if self._bindings.pop(name, None) is not None:
                for idx, cname in list(self._reverse.items()):
                    if cname == name:
                        del self._reverse[idx]
            self._autosave()

    def connect(self, src, src_port, dst, dst_port):
        """建立一条接线：源端口 -> 目标端口。

        src 可为 ParamSource / Effector / source_ref 元组；
        dst 可为 Effector / ServoChannel / 通道名。形成环时抛 GraphError。
        """
        with self._lock:
            src_ref = self._resolve_source(src, src_port)
            target = self._resolve_target(dst)
            if target[0] == "effector":
                name = target[1]
                inputs = self._effector_inputs[name]
                port = int(dst_port)
                if not (0 <= port < len(inputs)):
                    raise ValueError(f"效果器 {name} 无输入端口 {port}")
                old = inputs[port]
                inputs[port] = src_ref
                try:
                    self._rebuild_order()
                except GraphError:
                    inputs[port] = old
                    self._rebuild_order()
                    raise
            else:
                name = target[1]
                if int(dst_port) != 0:
                    raise ValueError(f"通道 {name} 只有 1 个输入端口")
                self._channel_inputs[name] = src_ref
            self._autosave()

    def disconnect(self, dst, dst_port):
        """断开目标端口上的一条接线。"""
        with self._lock:
            target = self._resolve_target(dst)
            if target[0] == "effector":
                name = target[1]
                inputs = self._effector_inputs.get(name)
                if inputs is not None and 0 <= int(dst_port) < len(inputs):
                    inputs[int(dst_port)] = None
                    self._rebuild_order()
            else:
                self._channel_inputs[target[1]] = None
            self._autosave()

    def _resolve_source(self, node, port):
        if isinstance(node, ParamSource):
            return ("param", node.name)
        if isinstance(node, Effector):
            name = self._eff_name_of.get(node)
            if name is None:
                raise ValueError("效果器未在图内")
            return ("effector", name, int(port))
        if isinstance(node, str) and node in self.params:
            return ("param", node)
        if isinstance(node, (tuple, list)):
            ref = self._list_to_ref(node)
            if ref is not None:
                return ref
        raise ValueError(f"无法解析接线源: {node!r}")

    def _resolve_target(self, node):
        if isinstance(node, Effector):
            name = self._eff_name_of.get(node)
            if name is None:
                raise ValueError("效果器未在图内")
            return ("effector", name)
        if isinstance(node, ServoChannel):
            return ("channel", node.name)
        if isinstance(node, str) and node in self.channels:
            return ("channel", node)
        raise ValueError(f"无法解析接线目标: {node!r}")

    def _rebuild_order(self):
        dep = DependencyGraph()
        for name in self.effectors:
            dep.add_node(name)
        for name, inputs in self._effector_inputs.items():
            for ref in inputs:
                if ref is not None and ref[0] == "effector":
                    dep.add_edge(ref[1], name)
        self._dep = dep
        self._order = dep.topo_order()

    def _autosave(self):
        """编排变化后自动写盘（构建/载入期间由 _suppress_save 抑制）。"""
        if self._suppress_save:
            return
        try:
            self.save_pipeline()
        except Exception as exc:  # noqa: BLE001  写盘失败不影响运行
            print(f"[pipeline] 自动保存 pipeline.yaml 失败（{exc}）")

    # --------------------------------------------------------
    # 逐帧求值（数据线程）
    # --------------------------------------------------------
    def tick(self):
        """处理一帧：参数归一化 -> 效果器拓扑求值 -> 各通道映射。"""
        with self._lock:
            raw = self._source.read()
            values = {}
            for name, param in self.params.items():
                values[("param", name)] = param.process(raw)
            for name in self._order:
                eff = self.effectors[name]
                inputs = [self._lookup(ref, values)
                          for ref in self._effector_inputs[name]]
                outputs = eff.process(inputs)
                for port, value in enumerate(outputs):
                    values[("effector", name, port)] = value
            for name, channel in self.channels.items():
                channel.process(self._lookup(self._channel_inputs.get(name),
                                             values))
            self._latest = values

    @staticmethod
    def _lookup(ref, values):
        if ref is None:
            return None
        if ref[0] == "param":
            return values.get(("param", ref[1]))
        return values.get(("effector", ref[1], ref[2]))

    def snapshot(self):
        """加锁拷贝最新值快照（供 GUI 读取，避免跨线程直接访问）。"""
        with self._lock:
            return {
                "params": {name: self._latest.get(("param", name))
                           for name in self.params},
                "channels": {name: ch.latest_output
                             for name, ch in self.channels.items()},
                "channel_inputs": {name: ch.latest_input
                                   for name, ch in self.channels.items()},
            }

    # --------------------------------------------------------
    # 舵机绑定 / 输出
    # --------------------------------------------------------
    def registered_servo_indices(self):
        """返回 servo_configs.yaml 中注册过的舵机通道索引（升序）。"""
        return sorted(self.servo_limits.keys())

    def get_binding(self, name):
        """返回通道绑定的舵机通道索引，未绑定返回 None。"""
        return self._bindings.get(name)

    def bind(self, name, servo_index):
        """绑定 通道名 -> servo_index（None 表示解绑）。

        去重校验：同一舵机通道只能绑定一个通道；只能绑定已注册舵机。
        绑定后通道输出为该舵机 [min_pulse, max_pulse]；解绑回到归一化 (0,1)。
        返回 (ok: bool, message: str)。仅在 GUI 主线程调用（会通知 Store）。
        """
        with self._lock:
            if name not in self.channels:
                return (False, f"未知通道: {name}")

            if servo_index is None:
                old = self._bindings.get(name)
                if old is not None and self._reverse.get(old) == name:
                    del self._reverse[old]
                self._bindings[name] = None
                self.channels[name].set_out_range(NORMALIZED_RANGE)
                self._notify_bindings()
                self._autosave()
                return (True, f"{name} 已解除绑定（输出归一化 0~1）")

            if not (0 <= servo_index < SERVO_CHANNELS):
                return (False, f"非法舵机通道: {servo_index}")
            limits = self.servo_limits.get(servo_index)
            if limits is None:
                return (False, f"servo_{servo_index} 未在 servo_configs.yaml 中注册，禁止绑定")
            existing = self._reverse.get(servo_index)
            if existing is not None and existing != name:
                return (False, f"servo_{servo_index} 已被 {existing} 绑定，禁止重复绑定")

            old = self._bindings.get(name)
            if (old is not None and old != servo_index
                    and self._reverse.get(old) == name):
                del self._reverse[old]

            self._reverse[servo_index] = name
            self._bindings[name] = servo_index
            self.channels[name].set_out_range((limits.min_pulse,
                                               limits.max_pulse))
            self._notify_bindings()
            self._autosave()
            return (True, f"{name} -> servo_{servo_index}（{limits.min_pulse:.0f}~"
                          f"{limits.max_pulse:.0f}µs）")

    def _notify_bindings(self):
        self.store.set({"bindings": dict(self._bindings)})

    def get_servo_vector(self):
        """构造 16 路舵机脉宽输出向量（µs）。

        已注册未绑定通道取范围中点；已绑定通道取该通道最新输出；未注册为 None。
        """
        with self._lock:
            vec = [None] * SERVO_CHANNELS
            for idx, limits in self.servo_limits.items():
                if 0 <= idx < SERVO_CHANNELS:
                    vec[idx] = (limits.min_pulse + limits.max_pulse) / 2.0
            for name, idx in self._bindings.items():
                if idx is None or idx not in self.servo_limits:
                    continue
                channel = self.channels.get(name)
                if channel is not None and channel.latest_output is not None:
                    vec[idx] = float(channel.latest_output)
            return vec

    # --------------------------------------------------------
    # 状态（Store 代理）
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # 持久化
    # --------------------------------------------------------
    @staticmethod
    def _ref_to_list(ref):
        if ref is None:
            return None
        if ref[0] == "param":
            return ["param", ref[1]]
        return ["effector", ref[1], ref[2]]

    @staticmethod
    def _list_to_ref(value):
        if not value:
            return None
        if value[0] == "param":
            return ("param", value[1])
        if value[0] == "effector":
            return ("effector", value[1], int(value[2]))
        return None

    def _to_config(self):
        effectors = {}
        for name, eff in self.effectors.items():
            effectors[name] = {
                "type": eff.TYPE_NAME,
                "inputs": [self._ref_to_list(r)
                           for r in self._effector_inputs[name]],
                "params": eff.get_params(),
            }
        channels = {}
        for name, channel in self.channels.items():
            params = channel.get_params()
            channels[name] = {
                "input": self._ref_to_list(self._channel_inputs.get(name)),
                "point_set": params.get("point_set"),
                "servo": self._bindings.get(name),
            }
        return {"effectors": effectors, "channels": channels}

    def save_pipeline(self):
        """把当前编排写回 pipeline.yaml。"""
        with self._lock:
            config_io.write_pipeline(self._config_path, self._to_config())

    def _load_config(self, data):
        with self._lock:
            for name, cfg in (data.get("effectors") or {}).items():
                cls = self._effector_types.get((cfg or {}).get("type"))
                if cls is None:
                    print(f"[pipeline] 未知效果器类型，跳过: {name}")
                    continue
                eff = cls()
                if cfg.get("params"):
                    eff.set_params(cfg["params"])
                self.effectors[name] = eff
                self._eff_name_of[eff] = name
                self._effector_inputs[name] = [None] * eff.get_input_count()

            for name, cfg in (data.get("channels") or {}).items():
                cfg = cfg or {}
                channel = ServoChannel(name)
                self.channels[name] = channel
                self._channel_inputs[name] = None
                servo = cfg.get("servo")
                if servo is not None:
                    ok, _msg = self.bind(name, int(servo))
                    if not ok:
                        print(f"[pipeline] 通道 {name} 绑定 servo_{servo} 失败，已忽略")
                if cfg.get("point_set"):
                    channel.set_params({"point_set": cfg["point_set"]})

            for name, cfg in (data.get("effectors") or {}).items():
                if name not in self.effectors:
                    continue
                inputs = (cfg or {}).get("inputs") or []
                for port, ref in enumerate(inputs):
                    if port < len(self._effector_inputs[name]) and ref:
                        self._effector_inputs[name][port] = self._list_to_ref(ref)

            for name, cfg in (data.get("channels") or {}).items():
                if name in self.channels and (cfg or {}).get("input"):
                    self._channel_inputs[name] = self._list_to_ref(
                        cfg["input"])

            self._rebuild_order()

    def _build_default_graph(self):
        """默认编排：param_i -> EMA_i -> channel_i（保持既有行为）。"""
        has_ema = "ema" in self._effector_types
        for name, param in self.params.items():
            if has_ema:
                eff = self.create_effector("ema", name=f"ema_{name}")
                self.connect(param, 0, eff, 0)
                producer, port = eff, 0
            else:
                producer, port = param, 0
            channel = self.add_channel(name)
            self.connect(producer, port, channel, 0)
