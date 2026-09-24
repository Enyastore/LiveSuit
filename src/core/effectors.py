"""效果器层抽象基类。

效果器只声明计算抽象，不关心名字与接线；实例化、命名与接线由编排层
(PipelineManager) 负责。端口为位置索引，具体语义写在子类里：
  - EMA：inputs[0] = 原始输入，outputs[0] = 平滑输出
  - 混合器：inputs[0]/[1] = 眼 X / 眼 Y，outputs[0]/[1] = 半径 / 角度

子类可实现 show_panel() 自带 GUI 面板（允许 import tkinter）。
"""


class Effector:
    """效果器抽象：多输入、多输出、位置式端口。"""

    TYPE_NAME = None   # 子类覆盖：注册表 / 持久化用的类型名

    def get_input_count(self):
        """返回输入端口数量。"""
        raise NotImplementedError

    def get_output_count(self):
        """返回输出端口数量。"""
        raise NotImplementedError

    def process(self, inputs):
        """处理一帧：inputs 为按端口顺序的输入值列表，返回同长度的输出列表。"""
        raise NotImplementedError

    def reset(self):
        """清空内部状态（如滤波器的历史）。"""

    def get_params(self):
        """返回可持久化的参数字典。"""
        return {}

    def set_params(self, params):
        """从参数字典恢复状态。"""

    def snapshot(self):
        """返回供 GUI 读取的遥测快照（线程安全）；无遥测返回 None。"""
        return None

    def show_panel(self, parent=None):
        """显示该效果器自带的面板（子类实现）。"""
        raise NotImplementedError

    def close_panel(self):
        """关闭自带面板（若有）。"""
