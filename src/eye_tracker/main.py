import tkinter as tk
import cv2
from tkinter import ttk
from PIL import Image, ImageTk
import yaml
import multiprocessing
multiprocessing.set_start_method("spawn", force=True)
from eye_tracker import EyeTracker


def _run_tracker(cam_index, flip, crop, side, command_queue):
    """独立进程中运行 EyeTracker（解决 OpenCV GUI 线程冲突）。
    
    Parameters
    ----------
    command_queue : multiprocessing.Queue
        接收来自主进程的锁定/解锁命令。
    """
    tracker = EyeTracker(cam_index=cam_index, flip=flip, crop=crop, side=side)
    tracker.start_tracking(command_queue=command_queue)


'''初始化'''
left_cam_i = 0
right_cam_i = 0
#画面剪裁，格式为[x1, x2, y1, y2]，默认值如下
right_cam_crop = [0, 640, 0, 480]
left_cam_crop = [0, 640, 0, 480]
#是否反转画面
left_cam_flip=False
right_cam_flip=False
#可用的相机索引
available_camera_index = []



# 检测相机，返回相机索引（最多检测max_cam个）
def detect_cameras(max_cams=6):
    available_cameras = []
    for i in range(max_cams):
        cap = cv2.VideoCapture(i)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras


# 主窗口
class MainWindow:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("主窗口")

        # ---- 命令队列（用于与追踪子进程通信） ----
        self.cmd_queue_left = None
        self.cmd_queue_right = None

        cameras = available_camera_index
        self.read_config()

        # --- 第一行：下拉选择框 ---
        frame_select = tk.Frame(self.window)
        frame_select.pack(pady=(10, 5))

        tk.Label(frame_select, text="左眼相机").pack(side=tk.LEFT, padx=(10, 5))
        self.camera_left = ttk.Combobox(frame_select, values=cameras, state="readonly", width=8)
        self.camera_left.current(available_camera_index.index(left_cam_i))
        self.camera_left.pack(side=tk.LEFT, padx=(0, 20))

        tk.Label(frame_select, text="右眼相机").pack(side=tk.LEFT, padx=(10, 5))
        self.camera_right = ttk.Combobox(frame_select, values=cameras, state="readonly", width=8)
        self.camera_right.current(available_camera_index.index(right_cam_i))
        self.camera_right.pack(side=tk.LEFT, padx=(0, 10))

        # --- 第二行：调试按钮 ---
        frame_debug = tk.Frame(self.window)
        frame_debug.pack(pady=5)

        tk.Button(frame_debug, text="调试剪裁左眼相机",
                  command=lambda: self.setup_cam("left", int(self.camera_left.get()))
                  ).pack(side=tk.LEFT, padx=10)
        tk.Button(frame_debug, text="调试剪裁右眼相机",
                  command=lambda: self.setup_cam("right", int(self.camera_right.get()))
                  ).pack(side=tk.LEFT, padx=10)

        # --- 第三行：功能按钮 ---
        frame_action = tk.Frame(self.window)
        frame_action.pack(pady=(5, 10))
        
        tk.Button(frame_action, text="开始眼球追踪", command=self.start_eye_tracking).pack(side=tk.LEFT, padx=10)
        tk.Button(frame_action, text="锁定控制面板", command=self.open_lock_control_panel).pack(side=tk.LEFT, padx=10)
        tk.Button(frame_action, text="保存配置", command=self.save_config).pack(side=tk.LEFT, padx=10)

    def open_lock_control_panel(self):
        """打开锁定控制面板 - 包含四个切换按钮用于锁定/解锁双眼半径和中心。"""
        panel = tk.Toplevel(self.window)
        panel.title("锁定控制面板")
        panel.resizable(False, False)

        # ---- 状态变量（每个 True=已锁定, False=已解锁） ----
        var_left_radius = tk.BooleanVar(value=False)
        var_right_radius = tk.BooleanVar(value=False)
        var_left_center = tk.BooleanVar(value=False)
        var_right_center = tk.BooleanVar(value=False)

        def _make_toggle(queue, state_var, label_prefix):
            """工厂函数：返回一个 toggle 回调，根据当前状态发送 lock/unlock 并切换按钮文字。"""
            def toggle():
                if queue is None:
                    print(f"[{label_prefix}] 追踪尚未启动，无法发送命令")
                    return
                if state_var.get():
                    # 当前已锁定 → 发送解锁
                    if "半径" in label_prefix:
                        queue.put_nowait("unlock_radius")
                    else:
                        queue.put_nowait("unlock_center")
                    state_var.set(False)
                else:
                    # 当前已解锁 → 发送锁定
                    if "半径" in label_prefix:
                        queue.put_nowait("lock_radius")
                    else:
                        queue.put_nowait("lock_center")
                    state_var.set(True)
            return toggle

        # 辅助：根据 state_var 动态更新按钮文字
        def _make_button(parent, queue, state_var, label_prefix):
            btn_text = tk.StringVar()
            def update_text(*args):
                locked = state_var.get()
                if locked:
                    btn_text.set(f"解锁{label_prefix}")
                else:
                    btn_text.set(f"锁定{label_prefix}")
            state_var.trace_add("write", update_text)
            update_text()  # 初始化
            toggle_cmd = _make_toggle(queue, state_var, label_prefix)
            return tk.Button(parent, textvariable=btn_text, command=toggle_cmd, width=18)

        # ---- 布局 ----
        tk.Label(panel, text="眼球追踪锁定控制", font=("", 12, "bold")).pack(pady=(10, 5))

        tk.Label(panel, text="左眼").pack(anchor="w", padx=20, pady=(5, 0))
        _make_button(panel, self.cmd_queue_left, var_left_radius, "左眼半径").pack(pady=2)
        _make_button(panel, self.cmd_queue_left, var_left_center, "左眼中心").pack(pady=2)

        tk.Label(panel, text="右眼").pack(anchor="w", padx=20, pady=(10, 0))
        _make_button(panel, self.cmd_queue_right, var_right_radius, "右眼半径").pack(pady=2)
        _make_button(panel, self.cmd_queue_right, var_right_center, "右眼中心").pack(pady=2)

        # 底部关闭按钮
        tk.Button(panel, text="关闭面板", command=panel.destroy).pack(pady=(10, 10))

    def setup_cam(self, side, index):
        """打开一个 Toplevel 窗口，使用 OpenCV 显示对应 index 相机的视频流"""
        # 打开摄像头
        cap = cv2.VideoCapture(index)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            print(f"错误：无法打开相机 {index}")
            return

        # 创建 Toplevel 窗口
        cam_window = tk.Toplevel(self.window)
        cam_window.title(f"调试 - {'左眼' if side == 'left' else '右眼'}相机 (索引 {index})")

        # --- 顶部按钮栏 ---
        btn_frame = tk.Frame(cam_window)
        btn_frame.pack(pady=(5, 0))

        # 垂直翻转复选框
        flip_var = tk.BooleanVar(value=(left_cam_flip if side == 'left' else right_cam_flip))
        tk.Checkbutton(btn_frame, text="垂直翻转", variable=flip_var).pack(side=tk.LEFT, padx=5)

        # 裁剪状态
        crop_mode = [False]
        crop_rect = [None]  # [x1, y1, x2, y2]
        drawing = [False]
        start_x = [0]
        start_y = [0]

        def toggle_crop():
            crop_mode[0] = not crop_mode[0]
            if crop_mode[0]:
                crop_rect[0] = None  # 清除旧矩形，准备重新绘制
                btn_crop.config(relief=tk.SUNKEN)
            else:
                btn_crop.config(relief=tk.RAISED)

        def save_crop_config():
            print("配置成功")
            if crop_rect[0] is not None:
                x1, y1, x2, y2 = crop_rect[0]
                print(f"剪裁区域: x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                if side == 'left':
                    global left_cam_crop
                    left_cam_crop = [x1, x2, y1, y2]
                    global left_cam_i
                    left_cam_i = index
                    global left_cam_flip
                    left_cam_flip = flip_var.get()
                else:
                    global right_cam_crop
                    right_cam_crop = [x1, x2, y1, y2]
                    global right_cam_i
                    right_cam_i = index
                    global right_cam_flip
                    right_cam_flip = flip_var.get()

        btn_crop = tk.Button(btn_frame, text="剪裁", command=toggle_crop)
        btn_crop.pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="保存剪裁配置", command=save_crop_config).pack(side=tk.LEFT, padx=5)

        # 用于显示视频的 Label
        video_label = tk.Label(cam_window)
        video_label.pack()

        # --- 鼠标事件（用于绘制裁剪矩形） ---
        def on_press(event):
            if not crop_mode[0]:
                return
            drawing[0] = True
            start_x[0] = event.x
            start_y[0] = event.y
            crop_rect[0] = None  # 按下时清除旧矩形

        TARGET_RATIO = 4.0 / 3.0

        def _constrain_rect(x1, y1, x2, y2):
            """根据鼠标起止点计算固定 4:3 比例的矩形。"""
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0 and dy == 0:
                return None
            # 以绝对值更大的轴为主导，锁定比例
            if abs(dx) / max(abs(dy), 1) > TARGET_RATIO:
                new_w = abs(dx)
                new_h = int(new_w / TARGET_RATIO)
                y2 = y1 + new_h if dy >= 0 else y1 - new_h
                x2 = x1 + dx
            else:
                new_h = abs(dy)
                new_w = int(new_h * TARGET_RATIO)
                x2 = x1 + new_w if dx >= 0 else x1 - new_w
                y2 = y1 + dy
            # 规范化为 (x1, y1) = 左上, (x2, y2) = 右下
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            return [x1, y1, x2, y2]

        def on_drag(event):
            if not crop_mode[0] or not drawing[0]:
                return
            crop_rect[0] = _constrain_rect(start_x[0], start_y[0], event.x, event.y)

        def on_release(event):
            if not crop_mode[0]:
                return
            drawing[0] = False
            crop_rect[0] = _constrain_rect(start_x[0], start_y[0], event.x, event.y)

        video_label.bind("<ButtonPress-1>", on_press)
        video_label.bind("<B1-Motion>", on_drag)
        video_label.bind("<ButtonRelease-1>", on_release)

        # 标志位，用于控制视频循环
        running = [True]

        def show_frame():
            if not running[0]:
                return
            ret, frame = cap.read()
            if flip_var.get():
                frame = cv2.flip(frame, 0)
            if ret:
                # 在帧上绘制黄色裁剪矩形
                if crop_mode[0] and crop_rect[0] is not None:
                    x1, y1, x2, y2 = crop_rect[0]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                # BGR -> RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                video_label.imgtk = imgtk
                video_label.configure(image=imgtk)
                video_label.after(30, show_frame)
            else:
                print("警告：无法读取帧")

        def on_close():
            running[0] = False
            cap.release()
            cam_window.destroy()

        cam_window.protocol("WM_DELETE_WINDOW", on_close)

        # 开始显示第一帧
        show_frame()

    def start_eye_tracking(self):
        """开始眼球追踪 - 在新进程中启动双眼 EyeTracker"""
        # 为每次启动创建新的命令队列
        self.cmd_queue_left = multiprocessing.Queue()
        self.cmd_queue_right = multiprocessing.Queue()

        # 左眼
        p_left = multiprocessing.Process(
            target=_run_tracker,
            args=(left_cam_i, left_cam_flip, left_cam_crop, "left", self.cmd_queue_left),
            daemon=True,
        )
        p_left.start()

        # 右眼
        p_right = multiprocessing.Process(
            target=_run_tracker,
            args=(right_cam_i, right_cam_flip, right_cam_crop, "right", self.cmd_queue_right),
            daemon=True,
        )
        p_right.start()

        print("双眼眼球追踪已启动")




# --- 保存和读取配置 ---
    def save_config(self):
        config = {
            'left_cam_i': left_cam_i,
            'right_cam_i': right_cam_i,
            'left_cam_crop': left_cam_crop,
            'right_cam_crop': right_cam_crop,
            'left_cam_flip': left_cam_flip,
            'right_cam_flip': right_cam_flip
        }
        with open('config.yaml', 'w', encoding='utf-8') as f:
            yaml.dump(config, f, allow_unicode=True)

    def read_config(self):
        global left_cam_i, right_cam_i, left_cam_crop, right_cam_crop
        try:
            with open('config.yaml', 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                if config is not None:
                    if 'left_cam_i' in config:
                        left_cam_i = config['left_cam_i']
                    if 'right_cam_i' in config:
                        right_cam_i = config['right_cam_i']
                    if 'left_cam_crop' in config:
                        left_cam_crop = config['left_cam_crop']
                    if 'right_cam_crop' in config:
                        right_cam_crop = config['right_cam_crop']
                    if 'left_cam_flip' in config:
                        left_cam_flip = config['left_cam_flip']
                    if 'right_cam_flip' in config:
                        right_cam_flip = config['right_cam_flip']
        except FileNotFoundError:
            self.save_config()
            print("无配置文件！已创建")

#启动窗口
    def run(self):
        self.window.mainloop()

#主函数：检测相机并启动主窗口（没有则退出）
if __name__ == "__main__":
    available_camera_index = detect_cameras()
    if available_camera_index == []:
        print("Error:没有可用相机")
        exit(1)
    else:
        print(available_camera_index)
        app = MainWindow()
        app.run()
