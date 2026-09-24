"""LiveSuit 主应用：欢迎页 + 子窗口管理 + 数据线程 + 重绘循环。

时钟解耦：
  - 数据线程：固定 30fps 跑 PipelineManager.tick() 与舵机下发；
  - 重绘循环：tkinter 主线程独立定时重绘，读锁保护的快照，不影响数据帧；
  - 相机/眼追：子进程自有帧率，数据线程仅采样其当前输出。

线程约束：tkinter 调用仅在主线程；数据线程只访问 PipelineManager（内部加锁）。
"""

import argparse
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox

_HERE = Path(__file__).resolve().parent          # src/gui
_SRC = _HERE.parent                              # src
_SERVO_CONTROL = _SRC / "servo_control"
for _p in (str(_SRC), str(_SERVO_CONTROL)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from core.pipeline import PipelineManager        # noqa: E402
from core.servo_limits import (                  # noqa: E402
    SERVO_CONFIGS_PATH,
    SERVO_CONFIG_STATUS_REPLACED,
    ServoDebugger,
    read_servo_configs,
)
from core.sources import CallableSource          # noqa: E402
from core.store import Store                     # noqa: E402
from effects import EFFECTOR_TYPES               # noqa: E402
from gui.channel_window import ChannelWindow     # noqa: E402
from gui.node_graph import NodeGraphWindow       # noqa: E402
from gui.servo_panel import ServoBusPanel        # noqa: E402
from gui.servo_tool import ServoToolWindow       # noqa: E402
from gui.welcome import WelcomePage              # noqa: E402

DATA_HZ = 30                 # 数据帧率（Hz）
DRAW_MS = 33                 # 主线程重绘间隔（ms，独立于数据帧）
SERVO_FAIL_STREAK_LIMIT = 30   # 舵机下发连续失败约 1s 后暂停逐帧下发
SERVO_RETRY_INTERVAL = 600     # 降级后周期性重试的帧数间隔


def _load_real_runtime(root):
    """加载真实运行环境：参数规格 + 数据源（眼球追踪 Slots）。

    仅在点击「启动LiveSuit」后调用，此刻才延迟导入 slots（含 cv2 依赖栈）
    并实例化 Slots。返回 (param_specs, source, runtime)。
    """
    from slots import Slots, get_slot_specs
    slots = Slots(root)
    return (get_slot_specs(), CallableSource(slots.get_all_output), slots)


def _load_servo_controller(limits):
    """惰性初始化舵机控制器；无硬件/缺依赖返回 None（跳过下发）。"""
    try:
        from servo_controller import ServoController
        pulse_configs = {i: (l.min_pulse, l.max_pulse)
                         for i, l in limits.items()}
        safe_limits = {i: (l.safe_pulse_min, l.safe_pulse_max)
                       for i, l in limits.items()}
        return ServoController(pulse_configs=pulse_configs,
                               safe_limits=safe_limits)
    except Exception as exc:  # noqa: BLE001  缺库 / 无 I2C 设备
        print(f"[gui] 无法初始化舵机控制器（{exc}），将跳过舵机下发。")
        return None


class LiveSuitApp:
    """主应用。"""

    def __init__(self, root, data_hz=DATA_HZ, draw_ms=DRAW_MS,
                 initial_headless=False):
        self.root = root
        self.data_hz = data_hz
        self.draw_ms = draw_ms
        self._runtime = None

        # 舵机脉宽限定注册表（{索引: ServoLimits}）
        servo_configs, servo_status = read_servo_configs(SERVO_CONFIGS_PATH)
        self.servo_pulse_configs = servo_configs

        # Store 独立于 manager 先建立（欢迎页启动前需读 render_mode）
        self.store = Store(initial={
            "render_mode": "headless" if initial_headless else "debug",
            "started": False,
            "bindings": {},
        })
        self.manager = None

        self._channel_windows = []
        self.servo_panel = None
        self.node_graph = None
        self.servo = None
        self.servo_debug = ServoDebugger(pulse_configs=servo_configs)
        self.servo_tool = None

        # 数据线程状态
        self._data_stop = threading.Event()
        self._data_thread = None
        self._data_frame = 0
        self._servo_fail_streak = 0
        self._servo_degraded = False

        self._draw_job = None
        self._started = False

        if servo_status == SERVO_CONFIG_STATUS_REPLACED:
            messagebox.showwarning(
                "舵机配置提示",
                "servo_configs.yaml 不合法，已用默认配置（servo_0~15，"
                "500~2500µs）覆盖该文件。\n请按实际舵机型号编辑该文件。")

        root.title("LiveSuit")
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.welcome = WelcomePage(root, self)
        self.welcome.pack(fill=tk.BOTH, expand=True)
        self.store.subscribe(self._on_state)

    # --------------------------------------------------------
    # Store 订阅
    # --------------------------------------------------------
    def _on_state(self, _state, patch):
        if "render_mode" in patch:
            self._apply_render_mode(patch["render_mode"])
            self.welcome.update_render_label(patch["render_mode"])
        if "bindings" in patch:
            self._prune_windows()
            for w in self._channel_windows:
                w.refresh()

    # --------------------------------------------------------
    # 启动 / 停止
    # --------------------------------------------------------
    def start(self):
        if self._started:
            return
        self._started = True
        try:
            specs, source, self._runtime = _load_real_runtime(self.root)
        except Exception as exc:  # noqa: BLE001  缺依赖 / 无相机
            import traceback
            traceback.print_exc()
            self._started = False
            messagebox.showerror(
                "启动失败",
                f"无法启动眼球追踪：{exc}\n请检查依赖与摄像头后重新点击启动。")
            return

        self.manager = PipelineManager(
            store=self.store, source=source, param_specs=specs,
            effector_types=EFFECTOR_TYPES,
            servo_limits=self.servo_pulse_configs)
        self.manager.start()

        self._close_servo_tool()
        if self.servo is None:
            self.servo = _load_servo_controller(self.servo_pulse_configs)

        self._build_channel_windows()
        self.servo_panel = ServoBusPanel(self.root, self.manager,
                                         on_close=self._on_servo_panel_closed)
        self._apply_render_mode(self.manager.render_mode)
        self._start_data_thread()
        self._schedule_draw()
        self.welcome.on_started()

    def stop(self):
        self._shutdown()
        self.root.destroy()

    def on_close(self):
        self._shutdown()
        self.root.destroy()

    def _shutdown(self):
        """停止数据线程、追踪进程、舵机与重绘循环。"""
        if self._draw_job is not None:
            try:
                self.root.after_cancel(self._draw_job)
            except tk.TclError:
                pass
            self._draw_job = None
        self._data_stop.set()
        if self._data_thread is not None:
            self._data_thread.join(timeout=2.0)
            self._data_thread = None
        if self.manager is not None:
            for eff in self.manager.get_effectors():
                try:
                    eff.close_panel()
                except Exception:  # noqa: BLE001
                    pass
        if self._runtime is not None:
            try:
                self._runtime.eye_tracker.stop()
            except Exception:  # noqa: BLE001  追踪进程可能已停止
                pass
        if self.servo is not None:
            try:
                self.servo.deinit()
            except Exception:  # noqa: BLE001  硬件可能已断开
                pass
            self.servo = None

    # --------------------------------------------------------
    # 数据线程 / 重绘循环
    # --------------------------------------------------------
    def _start_data_thread(self):
        self._data_stop.clear()
        self._data_thread = threading.Thread(
            target=self._data_loop, name="LiveSuitData", daemon=True)
        self._data_thread.start()

    def _data_loop(self):
        """固定 30fps：管线求值 + 舵机下发（与重绘、相机解耦）。"""
        period = 1.0 / max(1, self.data_hz)
        while not self._data_stop.is_set():
            t0 = time.monotonic()
            manager = self.manager
            if manager is not None:
                try:
                    manager.tick()
                except Exception:  # noqa: BLE001  单帧异常不中断数据循环
                    self._log_throttled("数据管道异常")
                try:
                    self._pump_servo(manager)
                except Exception:  # noqa: BLE001
                    pass
            self._data_frame += 1
            elapsed = time.monotonic() - t0
            self._data_stop.wait(max(0.0, period - elapsed))

    def _pump_servo(self, manager):
        """舵机下发（含失败降级与周期性重试）。"""
        if self.servo is None:
            return
        degraded = self._servo_degraded
        if degraded and self._data_frame % SERVO_RETRY_INTERVAL != 0:
            return
        try:
            self.servo.set_pulse(manager.get_servo_vector())
            if degraded:
                self._servo_degraded = False
                print("[gui] 舵机下发已恢复")
            self._servo_fail_streak = 0
        except Exception as exc:  # noqa: BLE001  硬件故障不中断数据循环
            self._servo_fail_streak += 1
            if self._servo_fail_streak >= SERVO_FAIL_STREAK_LIMIT:
                self._servo_degraded = True
            if (self._servo_fail_streak == 1
                    or self._servo_fail_streak % 300 == 0):
                print(f"[gui] 舵机下发失败（{exc}），连续 "
                      f"{self._servo_fail_streak} 帧"
                      + ("，已暂停逐帧下发" if self._servo_degraded else ""))

    _last_log = 0.0

    def _log_throttled(self, tag):
        now = time.time()
        if now - self._last_log > 2.0:
            import traceback
            traceback.print_exc()
            self._last_log = now

    def _schedule_draw(self):
        self._draw_job = self.root.after(self.draw_ms, self._draw_tick)

    def _draw_tick(self):
        try:
            manager = self.manager
            if manager is not None and manager.render_mode == "debug":
                self._prune_windows()
                for w in self._channel_windows:
                    w.redraw()
        except Exception:  # noqa: BLE001  单次重绘异常不中断循环
            pass
        finally:
            self._draw_job = self.root.after(self.draw_ms, self._draw_tick)

    # --------------------------------------------------------
    # 子窗口
    # --------------------------------------------------------
    def _build_channel_windows(self):
        for channel in self.manager.get_channels():
            w = ChannelWindow(self.root, self.manager, channel,
                              on_close=self._remove_channel_window)
            self._channel_windows.append(w)

    def _remove_channel_window(self, w):
        if w in self._channel_windows:
            self._channel_windows.remove(w)

    def _on_servo_panel_closed(self, panel):
        if self.servo_panel is panel:
            self.servo_panel = None

    def _prune_windows(self):
        self._channel_windows = [w for w in self._channel_windows
                                 if w.winfo_exists()]
        if self.servo_panel is not None and not self.servo_panel.winfo_exists():
            self.servo_panel = None
        if self.node_graph is not None and not self.node_graph.winfo_exists():
            self.node_graph = None

    def _apply_render_mode(self, mode):
        self._prune_windows()
        widgets = list(self._channel_windows)
        if self.servo_panel is not None:
            widgets.append(self.servo_panel)
        if self.node_graph is not None:
            widgets.append(self.node_graph)
        for w in widgets:
            if mode == "debug":
                w.deiconify()
            else:
                w.withdraw()

    def open_node_graph(self):
        """打开效果器编排节点图（启动后可用）。"""
        if self.manager is None:
            messagebox.showinfo("效果器编排", "请先启动 LiveSuit。")
            return
        if self.node_graph is not None and self.node_graph.winfo_exists():
            self.node_graph.deiconify()
            self.node_graph.lift()
            return
        self.node_graph = NodeGraphWindow(self.root, self)

    def toggle_render(self):
        self.manager.toggle_render_mode()

    # ---- 舵机调试工具 ----
    def open_servo_tool(self):
        if self._started:
            return
        if not self.servo_debug.registered_indices():
            messagebox.showwarning(
                "舵机工具",
                "servo_configs.yaml 中没有注册任何舵机，无法调试。\n"
                "请先在配置文件中添加 min_pulse / max_pulse 条目。")
            return
        if self.servo is None:
            self.servo = _load_servo_controller(self.servo_pulse_configs)
        if self.servo_tool is not None and self.servo_tool.winfo_exists():
            self.servo_tool.deiconify()
            self.servo_tool.lift()
            return
        self.servo_tool = ServoToolWindow(self.root, self,
                                          on_close=self._on_servo_tool_closed)

    def _on_servo_tool_closed(self, tool):
        if self.servo_tool is tool:
            self.servo_tool = None

    def _close_servo_tool(self):
        if self.servo_tool is not None:
            try:
                self.servo_tool.destroy()
            except tk.TclError:
                pass
            self.servo_tool = None


def main():
    parser = argparse.ArgumentParser(
        description="LiveSuit 个人可动兽装控制系统前端（tkinter）")
    parser.add_argument("--headless", action="store_true",
                        help="以无头模式启动（隐藏并停止渲染所有子面板）")
    parser.add_argument("--data-hz", type=int, default=DATA_HZ,
                        help=f"数据帧率（Hz），默认 {DATA_HZ}")
    parser.add_argument("--draw-ms", type=int, default=DRAW_MS,
                        help=f"主线程重绘间隔（ms），默认 {DRAW_MS}")
    args = parser.parse_args()

    root = tk.Tk()
    try:
        LiveSuitApp(root, data_hz=args.data_hz, draw_ms=args.draw_ms,
                    initial_headless=args.headless)
    except Exception as exc:  # noqa: BLE001  构造期异常
        print(f"[gui] 启动失败：{exc}")
        root.destroy()
        sys.exit(1)
    root.mainloop()


if __name__ == "__main__":
    main()
