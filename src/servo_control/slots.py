"""数据提供者汇总（Slots）+ 输出槽位注册中心。

本文件是 LiveSuit 中「输出槽位」的唯一注册来源：
  - SLOT_SPECS          输出槽位注册表（槽位名 / 显示名 / 归一化范围 / 输出范围）
  - get_slot_specs()    供 pipeline_manager 读取注册表、构建并行管道
  - Slots               数据提供者容器（目前对接眼球追踪模块），
                        get_all_output() 返回 {槽位名: 原始值}

新增输出槽位 / 新数据提供者时，只需在本文件内追加注册表项，
无需改动 pipeline_manager 或 gui。
"""

from collections import OrderedDict
from pathlib import Path
import sys


# 路径解析：兼容本仓库布局（eye_tracker_main.py 位于 src/face_tracking/ 下）
_HERE = Path(__file__).resolve().parent            # src/servo_control/
_FACE_TRACKING = _HERE.parent / "face_tracking"    # src/face_tracking/
sys.path.insert(0, str(_FACE_TRACKING))            # 便于 import eye_tracker_main

# 注意：eye_tracker_main（含 cv2 / numpy / C++ 绑定）延迟到 Slots 实例化时
# 才导入——保证 `import slots` 是轻量的，纯元数据（SLOT_SPECS / PROVIDER_INFO）
# 可被纯 UI（gui.py）直接读取，不拉起硬件依赖栈。

# ============================================================
# 输出槽位注册表
# ============================================================
# 每个槽位对应一条并行管道：
#     原始数据 -> 归一化(in_range) -> EMA滤波 -> 三次样条 -> 舵机总线(脉宽 µs)
#   key      : 槽位名（即舵机总线面板「输入槽位」列的 Label）
#   label    : 显示用中文名
#   in_range : 原始数据取值范围（用于归一化到 [0, 1]）
# 注意：槽位的输出范围（脉宽 MIN~MAX）不再在此注册——它由该槽位绑定
# 的舵机决定（servo_configs.yaml 中每路舵机的 min_pulse/max_pulse）。
# ---此处可拓展其他输出槽位
SLOT_SPECS = OrderedDict([
    ("a_left_eye_x",  {"label": "左眼注视 X", "in_range": (-1.0, 1.0)}),
    ("a_left_eye_y",  {"label": "左眼注视 Y", "in_range": (-1.0, 1.0)}),
    ("a_left_eye_o",  {"label": "左眼开度",    "in_range": (0.0, 1.0)}),
    ("a_right_eye_x", {"label": "右眼注视 X", "in_range": (-1.0, 1.0)}),
    ("a_right_eye_y", {"label": "右眼注视 Y", "in_range": (-1.0, 1.0)}),
    ("a_right_eye_o", {"label": "右眼开度",    "in_range": (0.0, 1.0)}),
])

# 提供者原始键 -> 槽位名（眼球追踪模块输出 left_eye_x 等，映射为 a_left_eye_x 等）
PROVIDER_KEY_MAP = {
    "left_eye_x": "a_left_eye_x",
    "left_eye_y": "a_left_eye_y",
    "left_eye_o": "a_left_eye_o",
    "right_eye_x": "a_right_eye_x",
    "right_eye_y": "a_right_eye_y",
    "right_eye_o": "a_right_eye_o",
}

#显示在开始页面的提示信息
PROVIDER_INFO = "【眼部追踪】左眼left_eye_x, left_eye_y, left_eye_o三个参数，右眼right_eye_x, right_eye_y, right_eye_o三个参数"

def get_slot_specs() -> OrderedDict:
    """返回输出槽位注册表副本（供 pipeline_manager 构建并行管道）。"""
    return OrderedDict(SLOT_SPECS)


class Slots:
    def __init__(self, root):
        """ 实例化提供者
            并将root作为根窗口打开UI
            后续可以添加其他模块
        """
        from eye_tracker_main import launch_debug_panel  # noqa: PLC0415  延迟到实例化时导入（避免 import slots 即拉起 cv2 / eye_tracker 依赖栈）
        self.eye_tracker = launch_debug_panel(master=root)   # 面板挂到已有主窗口（Toplevel）；关面板不停追踪
        #---此处可拓展其他提供者

    def get_all_output(self):
        """获取所有参数输出，返回 {槽位名: 原始值}。

        键名使用 SLOT_SPECS 中注册的槽位名（经 PROVIDER_KEY_MAP 转换），
        可直接交给 pipeline_manager 的管道逐槽处理。
        """
        #获取所有参数输出
        eye_state = self.eye_tracker.get_normalized_eye_state()
        #---此处可拓展其他输出

        #整理为字典格式（provider 原始键）
        result = {"left_eye_x" : eye_state["left"]["eye_x"],
            "left_eye_y" : eye_state["left"]["eye_y"],
            "left_eye_o" : eye_state["left"]["eye_o"],
            "right_eye_x" : eye_state["right"]["eye_x"],
            "right_eye_y" : eye_state["right"]["eye_y"],
            "right_eye_o" : eye_state["right"]["eye_o"],
            #---此处可拓展其他结果
            }

        # 映射为槽位名（未在映射表中的键原样保留）
        return {PROVIDER_KEY_MAP.get(key, key): value for key, value in result.items()}



