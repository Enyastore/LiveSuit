# 源码根文件夹

## face_tracking/
面捕追踪算法（目前为眼部追踪），提供归一化后的原始信号。

## servo_control/
硬件与信号原语：
- `slots.py` —— 参数输出层入口（`SLOT_SPECS` + `Slots` 数据提供者）
- `after_process.py` —— `Filter`(EMA) / `Mapper`(三次样条) 数学原语
- `servo_controller.py` —— PCA9685 舵机驱动

## core/
纯逻辑层（禁止 import tkinter）：
- `params.py` 参数输出层（原始值 -> 归一化参数流）
- `effectors.py` 效果器抽象基类
- `channels.py` 舵机控制层（Spline + 绑定）
- `graph.py` 效果器依赖图（拓扑 / 环检测）
- `sources.py` / `store.py` / `servo_limits.py` / `config_io.py`
- `pipeline.py` 编排层 `PipelineManager`

## effects/
效果器实现（子类自带 GUI 面板，如 `ema.py` 的 `EMAEffector`）。
新增效果器只需继承 `core.effectors.Effector` 并注册到 `effects.EFFECTOR_TYPES`。

## gui/
tkinter 界面层：
- `app.py` 主应用（数据线程 30fps + 独立重绘循环）
- `spline_canvas.py` / `channel_window.py` / `node_graph.py`
- `servo_panel.py` / `servo_tool.py` / `welcome.py`

## gui.py
用户入口。直接 `python3 src/gui.py` 即可打开欢迎页面。
