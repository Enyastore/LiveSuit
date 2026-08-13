from pathlib import Path
import sys


# 路径解析：兼容本仓库布局（eye_tracker_main.py 位于 src/face_tracking/ 下）
_HERE = Path(__file__).resolve().parent            # src/servo_control/
_FACE_TRACKING = _HERE.parent / "face_tracking"    # src/face_tracking/
sys.path.insert(0, str(_FACE_TRACKING))            # 便于 import eye_tracker_main


from eye_tracker_main import launch_debug_panel


class Slots:
    def __init__(self, root):
        """ 实例化提供者
            并将root作为根窗口打开UI
            后续可以添加其他模块
        """
        self.eye_tracker = launch_debug_panel(master=root)   # 面板挂到已有主窗口（Toplevel）；关面板不停追踪
        #---此处可拓展其他提供者

    def get_all_output(self):
        #获取所有参数输出
        eye_state = self.eye_tracker.get_normalized_eye_state()
        #---此处可拓展其他输出

        #整理为字典格式
        result = {"left_eye_x" : eye_state["left"]["eye_x"], 
            "left_eye_y" : eye_state["left"]["eye_y"],
            "left_eye_o" : eye_state["left"]["eye_o"],
            "right_eye_x" : eye_state["right"]["eye_x"], 
            "right_eye_y" : eye_state["right"]["eye_y"],
            "right_eye_o" : eye_state["right"]["eye_o"],
            
            #---此处可拓展其他结果
            }

        return result