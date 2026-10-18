"""效果器实现注册表。

新增效果器：在本包内新增一个子类，实现 process 与 show_panel，然后把它加入
EFFECTOR_TYPES 即可（无需改动 PipelineManager 或 GUI 框架）。
"""

from effects.ema import EMAEffector
from effects.one_d_generator import OneDGeneratorEffector
from effects.two_d_generator import TwoDGeneratorEffector

# 类型名 -> 效果器类；节点图"新增效果器"菜单据此列出
EFFECTOR_TYPES = {
    "EMA": EMAEffector,
    "1D-Generator": OneDGeneratorEffector,
    "2D-Generator": TwoDGeneratorEffector,
}

__all__ = ["EMAEffector", "OneDGeneratorEffector", "TwoDGeneratorEffector",
           "EFFECTOR_TYPES"]
