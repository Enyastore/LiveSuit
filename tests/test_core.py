"""LiveSuit core 层离线回归测试（无需硬件/tkinter 显示）。

运行：.venv/bin/python -m unittest discover -s tests -v
"""

import sys
import tempfile
import unittest
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core.channels import ServoChannel, NORMALIZED_RANGE          # noqa: E402
from core.effectors import Effector                               # noqa: E402
from core.graph import DependencyGraph, GraphError                # noqa: E402
from core.params import Normalizer, ParamSource                   # noqa: E402
from core.servo_limits import ServoLimits                         # noqa: E402
from core.sources import CallableSource                           # noqa: E402
from effects import EFFECTOR_TYPES                                # noqa: E402
from core.pipeline import PipelineManager                         # noqa: E402


class SimpleEffector(Effector):
    """1->1 增益效果器（用于测试图编排，不依赖 tkinter）。"""

    TYPE_NAME = "gain"

    def __init__(self, gain=1.0):
        self.gain = float(gain)

    def get_input_count(self):
        return 1

    def get_output_count(self):
        return 1

    def process(self, inputs):
        v = inputs[0]
        return [None if v is None else v * self.gain]

    def get_params(self):
        return {"gain": self.gain}

    def set_params(self, params):
        if params and params.get("gain") is not None:
            self.gain = float(params["gain"])

    def show_panel(self, parent=None):
        return None


class MixerEffector(Effector):
    """2->2 效果器（模仿矩形->极坐标），验证多入多出。"""

    TYPE_NAME = "mixer"

    def get_input_count(self):
        return 2

    def get_output_count(self):
        return 2

    def process(self, inputs):
        x, y = inputs
        if x is None or y is None:
            return [None, None]
        return [x + y, x - y]

    def show_panel(self, parent=None):
        return None


def make_limits():
    return {0: ServoLimits(500, 2500, 400, 2700),
            1: ServoLimits(600, 2400, 450, 2550)}


class TestParams(unittest.TestCase):
    def test_normalizer(self):
        n = Normalizer((-1.0, 1.0))
        self.assertAlmostEqual(n.map(-1.0), 0.0)
        self.assertAlmostEqual(n.map(1.0), 1.0)
        self.assertAlmostEqual(n.map(0.0), 0.5)
        self.assertIsNone(n.map(None))
        self.assertIsNone(n.map(float("nan")))
        self.assertAlmostEqual(n.map(99), 1.0)   # 钳制

    def test_param_source(self):
        p = ParamSource("a", source_key="k", label="L", in_range=(0.0, 1.0))
        self.assertAlmostEqual(p.process({"k": 0.25}), 0.25)
        self.assertIsNone(p.process({}))


class TestEMA(unittest.TestCase):
    def test_process_and_replay(self):
        from effects.ema import EMAEffector
        e = EMAEffector(alpha=0.5)
        self.assertEqual(e.get_input_count(), 1)
        self.assertEqual(e.get_output_count(), 1)
        outs = [e.process([1.0])[0] for _ in range(3)]
        # 首个输出 = 0.5*1 + 0.5*0 = 0.5；随后单调趋近 1
        self.assertAlmostEqual(outs[0], 0.5)
        self.assertLess(outs[0], outs[1])
        # alpha 变更后历史重放：末值应与直接滤波一致
        e.set_alpha(1.0)
        hist, alpha, latest = e.snapshot()
        self.assertEqual(alpha, 1.0)
        self.assertAlmostEqual(latest, 1.0)

    def test_params_roundtrip(self):
        from effects.ema import EMAEffector
        e = EMAEffector(alpha=0.2)
        self.assertEqual(e.get_params(), {"alpha": 0.2})
        e.set_params({"alpha": 0.7})
        self.assertAlmostEqual(e.get_params()["alpha"], 0.7)


class TestChannels(unittest.TestCase):
    def test_unbound_normalized(self):
        ch = ServoChannel("c")
        self.assertEqual(ch.out_range, NORMALIZED_RANGE)
        out = ch.process(0.5)
        self.assertAlmostEqual(out, 0.5)

    def test_bound_range_and_shape(self):
        ch = ServoChannel("c")
        ch.set_out_range((500.0, 2500.0))
        self.assertAlmostEqual(ch.process(0.5), 1500.0)
        self.assertAlmostEqual(ch.process(0.0), 500.0)
        self.assertAlmostEqual(ch.process(1.0), 2500.0)
        # 换绑保持归一化形状
        ch.set_out_range(NORMALIZED_RANGE)
        self.assertAlmostEqual(ch.process(0.5), 0.5)

    def test_hold_last_on_none(self):
        ch = ServoChannel("c")
        ch.process(0.3)
        self.assertAlmostEqual(ch.process(None), 0.3)

    def test_params_roundtrip(self):
        ch = ServoChannel("c")
        ch.set_curve_points([(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)])
        saved = ch.get_params()
        ch2 = ServoChannel("c")
        ch2.set_params(saved)
        self.assertEqual(len(ch2.curve_points()), 3)


class TestGraph(unittest.TestCase):
    def test_topo_and_cycle(self):
        g = DependencyGraph()
        for n in ("a", "b", "c"):
            g.add_node(n)
        g.add_edge("a", "b")
        g.add_edge("b", "c")
        order = g.topo_order()
        self.assertLess(order.index("a"), order.index("b"))
        self.assertLess(order.index("b"), order.index("c"))
        with self.assertRaises(GraphError):
            g.add_edge("c", "a")

    def test_remove_node(self):
        g = DependencyGraph()
        g.add_edge("a", "b")
        g.remove_node("a")
        self.assertEqual(g.topo_order(), ["b"])


class TestPipeline(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.cfg = self.tmp / "pipeline.yaml"
        self.limits = make_limits()
        self.state = {"a_left_eye_x": 0.5, "a_left_eye_y": -0.5}
        self.specs = {
            "a_left_eye_x": {"label": "X", "in_range": (-1.0, 1.0)},
            "a_left_eye_y": {"label": "Y", "in_range": (-1.0, 1.0)},
        }
        self.types = dict(EFFECTOR_TYPES)
        self.types["gain"] = SimpleEffector
        self.types["mixer"] = MixerEffector

    def _manager(self, path=None):
        return PipelineManager(
            source=CallableSource(lambda: dict(self.state)),
            param_specs=self.specs, effector_types=self.types,
            config_path=path or self.cfg, servo_limits=self.limits)

    def test_default_graph_and_tick(self):
        mgr = self._manager()
        self.assertEqual(len(mgr.get_effectors()), 2)
        self.assertEqual({c.name for c in mgr.get_channels()},
                         {"a_left_eye_x", "a_left_eye_y"})
        mgr.tick()
        snap = mgr.snapshot()
        self.assertAlmostEqual(snap["params"]["a_left_eye_x"], 0.75)
        self.assertTrue(0.0 <= snap["channels"]["a_left_eye_x"] <= 1.0)

    def test_bind_output_and_persist(self):
        mgr = self._manager()
        ok, _ = mgr.bind("a_left_eye_x", 0)
        self.assertTrue(ok)
        mgr.tick()
        vec = mgr.get_servo_vector()
        self.assertTrue(500.0 <= vec[0] <= 2500.0)
        # 重载：绑定与图均保留
        mgr2 = self._manager()
        self.assertEqual(mgr2.get_binding("a_left_eye_x"), 0)
        self.assertEqual(set(mgr2.effectors), set(mgr.effectors))

    def test_unbind_to_normalized(self):
        mgr = self._manager()
        mgr.bind("a_left_eye_x", 0)
        ok, _ = mgr.bind("a_left_eye_x", None)
        self.assertTrue(ok)
        self.assertAlmostEqual(mgr.get_channels()[0].out_range[0]
                               if mgr.get_channels()[0].name == "a_left_eye_x"
                               else 0.0, 0.0)

    def test_duplicate_bind_rejected(self):
        mgr = self._manager()
        mgr.bind("a_left_eye_x", 0)
        ok, _ = mgr.bind("a_left_eye_y", 0)
        self.assertFalse(ok)

    def test_multi_in_out_effector(self):
        mgr = self._manager()
        mixer = mgr.create_effector("mixer", name="mix")
        eff_x = list(mgr.get_effectors())
        # 找到两个 EMA 的实例名
        ema_names = [n for n, e in mgr.effectors.items()
                     if e.TYPE_NAME == "ema"]
        mgr.connect(("effector", ema_names[0], 0), 0, mixer, 0)
        mgr.connect(("effector", ema_names[1], 0), 0, mixer, 1)
        # 把通道改接到 mixer 的两个输出
        channels = list(mgr.channels.values())
        mgr.connect(mixer, 0, channels[0], 0)
        mgr.connect(mixer, 1, channels[1], 0)
        mgr.tick()
        snap = mgr.snapshot()
        self.assertIn("a_left_eye_x", snap["channels"])
        self.assertIsNotNone(snap["channels"]["a_left_eye_x"])

    def test_cycle_rejected(self):
        mgr = self._manager()
        e1 = mgr.create_effector("gain", name="g1")
        e2 = mgr.create_effector("gain", name="g2")
        ema_names = [n for n, e in mgr.effectors.items()
                     if e.TYPE_NAME == "ema"]
        mgr.connect(("effector", ema_names[0], 0), 0, e1, 0)
        mgr.connect(e1, 0, e2, 0)
        with self.assertRaises(GraphError):
            mgr.connect(e2, 0, e1, 0)

    def test_remove_effector_clears_wiring(self):
        mgr = self._manager()
        ema = [e for e in mgr.get_effectors() if e.TYPE_NAME == "ema"][0]
        mgr.remove_effector(ema)
        self.assertNotIn(ema, mgr.get_effectors())
        mgr.tick()   # 不应崩溃


if __name__ == "__main__":
    unittest.main()
