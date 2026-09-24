"""效果器实现注册表。

新增效果器：在本包内新增一个子类，实现 process 与 show_panel，然后把它加入
EFFECTOR_TYPES 即可（无需改动 PipelineManager 或 GUI 框架）。
"""

from effects.ema import EMAEffector

# 类型名 -> 效果器类；节点图"新增效果器"菜单据此列出
EFFECTOR_TYPES = {
    "ema": EMAEffector,
}

__all__ = ["EMAEffector", "EFFECTOR_TYPES"]
