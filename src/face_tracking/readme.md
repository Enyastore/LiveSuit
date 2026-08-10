# 面捕追踪代码
- 目前仅做了两眼追踪，每只眼输出eye_x、eye_y、eye_o三个归一化参数
- 未来可能扩展嘴巴追踪等
- 第一次跑眼追后创建config.yaml是相机配置（翻转、裁剪、兴趣椭圆、明度对比度滤镜）
- references.yaml存储保存的参考值（注视极值向量、开闭度参考值）

---

# eye_tracker_main.py 模块文档

`eye_tracker_main.py` 是面捕追踪的**主入口/编排层**（`src/face_tracking/`）。负责相机探测与配置管理、
双进程追踪的启停编排、结果收集，以及三套调试 UI（裁剪调试窗口、控制面板、调试面板）。
算法本体由 `eye_tracker_core.py`（C++ 封装）提供。

## 文件结构（自上而下）

| 行区 | 内容 |
|---|---|
| 59-162 | 相机工具函数：`_setup_camera` / `probe_camera_modes` / `mode_to_label` / `match_current_mode` |
| 164-195 | 数据模型：`CameraConfig` / `AppConfig` |
| 197-313 | 配置持久化：`ConfigPersistence` |
| 315-978 | 裁剪/阈值调试窗口：`CropDebugWindow` |
| 981-1294 | 控制面板：`ControlPanel` |
| 1297-1656 | 核心编排：`EyeTrackingModule` |
| 1658-1704 | 子进程入口：`_run_tracker_in_process`、相机探测：`detect_cameras` |
| 1707-1871 | 调试面板构建：`_build_debug_panel`、公共入口：`launch_debug_panel` |
| 1874-1892 | 独立脚本入口：`main()` |

## 快速上手

```python
# ① 直接运行：自带调试面板
python eye_tracker_main.py

# ② 无 UI 编程使用
from eye_tracker_main import EyeTrackingModule
with EyeTrackingModule(headless=True, config_path="config.yaml") as mod:
    mod.start()
    state = mod.get_normalized_eye_state()   # 每帧读取结果

# ③ 被其他模块调用时唤起调试面板
module = launch_debug_panel(master=root)    # 挂载到已有 Tk 主窗口（Toplevel 子面板，关面板不停止追踪）
module = launch_debug_panel()               # 自建 Tk 根窗口（需 module._master.mainloop()）
```

## 数据模型与配置

### `CameraConfig`（dataclass）— 单相机配置
| 字段 | 默认 | 说明 |
|---|---|---|
| `index` | 0 | 相机索引 |
| `crop` | [0,0,640,480] | 裁剪矩形 `[x1, y1, x2, y2]` |
| `flip` | False | 垂直翻转 |
| `frame_width/height` | 640/480 | 采集分辨率 |
| `frame_rate` | 30 | 帧率 |
| `fourcc` | "" | 像素格式，如 'YUYV'/'MJPG' |
| `use_recommended_resolution` | True | 使用推荐 4:3 分辨率 |
| `dark_search_roi_scale` | 0.70 | 暗区搜索椭圆比例 |
| `brightness` / `contrast` | 0.0 / 1.0 | 明度偏移 / 对比度系数 |
| `openness_threshold_low/high` | 0 / 80 | 眼睛开闭检测双阈值（0-255） |
| `pupil_threshold_low/high` | 0 / 50 | 瞳孔检测双阈值（0-255） |

### `AppConfig`（dataclass）— 双眼配置
- `left` / `right`：两个 `CameraConfig`。

### `ConfigPersistence` — YAML 读写
- `load() -> AppConfig`：读取 `config.yaml`；文件不存在时生成默认配置并保存。
- `save(config)`：写回 `config.yaml`（YAML 键名：`camera_index`、`crop`、`flip`、`frame_width` 等）。

## 相机工具函数

- `_setup_camera(index, fps, width, height, fourcc) -> VideoCapture | None`：V4L2 后端打开相机。被 `detect_cameras()` 与 `CropDebugWindow` 使用。
- `probe_camera_modes(cam_index) -> List[dict]`：调 `v4l2-ctl --list-formats-ext` 探测 `(width, height, fps, format)` 模式列表；供 `CropDebugWindow` 分辨率下拉框使用。
- `mode_to_label(mode) -> str`：模式转显示文本，如 `640x480 @30fps (YUYV)`。
- `match_current_mode(modes, w, h, fps, fmt) -> int | None`：在模式列表中匹配当前配置的下标。
- `detect_cameras(max_cams=6) -> List[int]`：扫描可用相机索引；被 `launch_debug_panel()` 用于左右眼下拉框。

## 核心编排 `EyeTrackingModule`

构造：`EyeTrackingModule(headless=False, master=None, config_path="config.yaml", refs_file="references.yaml")`
- `headless=True`：不创建任何 GUI，`master` 置 None。
- `master`：父 Tk 窗口；缺省自动创建隐藏根窗口。
- 内部持有**模块级共享 `Normalizer`**（开度参考缓存），结果收集线程读取、控制面板写入同一个实例。

| 方法 | 说明 |
|---|---|
| `start()` | 停旧进程 → 创建左右两个追踪子进程（target=`_run_tracker_in_process`）→ 启动结果收集线程；任一侧子进程启动失败即抛 `RuntimeError` |
| `stop()` | 停止子进程与结果线程（内部调 `_stop_internal()`） |
| `is_running()` | 左右子进程是否都存活 |
| `get_normalized_eye_state() -> dict` | 一致快照：`{left:{eye_x,eye_y,eye_o,raw_eye_openness,confidence}, right:{...}, timestamp}` |
| `get_raw_gaze_vector(side)` | 单眼原始注视向量 |
| `enter_headless_mode()` / `exit_headless_mode()` | 向子进程发 `headless_on/off` 命令切换 OpenCV 窗口；`headless_runtime` 属性反映状态 |
| `open_crop_window(side)` | 打开/唤起 `CropDebugWindow`（`side="left"/"right"`） |
| `open_control_panel()` | 打开 `ControlPanel` |
| `save_config()` | 持久化当前 `config`（作为 CropDebugWindow 的 `on_config_changed` 回调） |
| `set_left_camera(index)` / `set_right_camera(index)` | 修改相机索引（写 `config`，`start()` 前调用） |
| `__enter__` / `__exit__` | 上下文管理器（start/stop） |

### 子进程数据流

```
ControlPanel / CropDebugWindow ──lock_radius / lock_center / 阈值 / 标定指令──► cmd_queue_left/right
子进程（GazeVectorTracker）──result_queue 每帧 {side, gaze_rotated, eye_x, eye_y,
                            confidence, raw_eye_openness}──► _collect_results ──► _latest_state
```

## 调试 UI

### `CropDebugWindow`（Toplevel）
单眼相机调试窗口：实时画面 + 固定比例(4:3)剪裁 + 垂直翻转 + 明度/对比度 + 分辨率模式下拉 +
搜索区域比例；右侧为开度/瞳孔二值化调试视口与双阈值滑块（改动实时经 `cmd_queue` 下发子进程）。
- 构造参数：`(master, cam_index, side, cam_config, on_config_changed, cmd_queue)`；由 `EyeTrackingModule.open_crop_window()` 创建。
- 关键方法：`close()` / `get_window()`。

### `ControlPanel`（Toplevel）
双眼锁定/标定面板：锁定/解锁眼球半径与中心、十字方向注视极值向量保存（up/down/inner/outer）、
完全睁眼/闭眼开度参考标定（写入模块共享 Normalizer）、清除参考、最低开度跳过阈值、实时输出显示。
- 构造参数：`(master, cmd_queue_left, cmd_queue_right, gaze_reader, openness_normalizer, module)`；由 `open_control_panel()` 创建。
- 关键方法：`destroy()` / `is_open()`。

### `_build_debug_panel(module, window, cameras, stop_on_close)`（内部）
把调试面板 UI 构建到 `window`（Tk 根窗口或 Toplevel）上：左右眼相机下拉框、开始/停止、
调试剪裁、隐藏/显示 OpenCV。`stop_on_close=True` 时关窗会调 `module.stop()`。
由 `launch_debug_panel()` 调用，外部一般无需直接使用。

## 独立入口与调用关系

- `launch_debug_panel(master, config_path, refs_file) -> EyeTrackingModule`：公共入口。`master=None` 自建根窗口（`stop_on_close=True`）；`master=<Tk>` 挂 Toplevel 子面板（`stop_on_close=False`）。无可用相机时抛 `RuntimeError`。
- `main()`：设置 `multiprocessing` spawn 与 logging → `launch_debug_panel()` → `module._master.mainloop()`；无相机时打印 `Error: 没有可用相机` 并返回 1。`if __name__ == "__main__"` 调 `sys.exit(main())`。

```
launch_debug_panel ──► detect_cameras ──► _setup_camera
       ├─► EyeTrackingModule ──► ConfigPersistence.load
       └─► _build_debug_panel ──► open_crop_window ──► CropDebugWindow ──► _setup_camera / probe_camera_modes / mode_to_label / match_current_mode
                              └──► open_control_panel ──► ControlPanel
EyeTrackingModule.start ──► _run_tracker_in_process ×2 ──► GazeVectorTracker.start_tracking（eye_tracker_core）
EyeTrackingModule.stop  ──► _stop_internal ──► ControlPanel.destroy + 子进程 terminate
```

## 相关文件
- `eye_tracker_core.py`：`GazeVectorTracker`（C++ 算法封装）、`Normalizer`（注视极值/开度参考）。
- `config.yaml` / `references.yaml`：运行时生成/读取的相机配置与标定参考值。