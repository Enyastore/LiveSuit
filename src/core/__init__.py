"""LiveSuit 纯逻辑层（禁止 import tkinter）。

分层：
  - params.py        参数输出层（原始值 -> 归一化参数流）
  - effectors.py     效果器层抽象基类
  - channels.py      舵机控制层（Spline + 绑定）
  - sources.py       数据源抽象
  - store.py         跨组件状态容器
  - graph.py         效果器依赖图（拓扑 / 环检测）
  - servo_limits.py  舵机脉宽限定注册表 + 调试工具
  - config_io.py     pipeline.yaml 读写
  - pipeline.py      编排层 PipelineManager

本 __init__ 负责把 servo_control/ 加入 sys.path，使各层可 import
after_process（Filter / Mapper）。
"""

import sys
from pathlib import Path

_SERVO_CONTROL = Path(__file__).resolve().parent.parent / "servo_control"
if str(_SERVO_CONTROL) not in sys.path:
    sys.path.insert(0, str(_SERVO_CONTROL))
