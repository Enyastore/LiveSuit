# 舵机控制代码

本文件夹是 LiveSuit 项目中「把眼球追踪数据变成舵机动作」的控制链路，包含舵机驱动、信号后处理与多模块输出汇总三部分。

## 目录结构

```
src/servo_control/
├── servo_controller.py   # PCA9685 舵机驱动（ServoController 类）
├── after_process.py      # 信号后处理（Filter 平滑 / Mapper 角度映射）
├── slots.py              # 数据提供者汇总（Slots 类，目前对接眼球追踪模块）
├── test_servo.py         # 单通道舵机行程实验脚本（默认 servo_1）
└── readme.md             # 本文档
```

## 整体数据流

```
src/face_tracking/eye_tracker_main.py         src/servo_control/
┌─────────────────────────────┐   get_normalized_    ┌──────────────────────────┐
│ EyeTrackingModule           │───eye_state()──────▶ │ Slots.get_all_output()   │
│  .get_normalized_eye_state()│                     │  返回 {left/right:        │
│  → {left, right, timestamp} │                     │    eye_x, eye_y, eye_o}   │
└─────────────────────────────┘                     └───────────┬──────────────┘
                                                                ▼
                                             ┌──────────────────────────────┐
                                             │ after_process.py              │
                                             │  Filter   → EMA 平滑（抗抖动） │
                                             │  Mapper   → 三次样条映射到角度 │
                                             │             （如 0~180°）      │
                                             └───────────┬──────────────────┘
                                                         ▼
                                             ┌──────────────────────────────┐
                                             │ ServoController.set_angle()  │
                                             │  将角度列表下发到 PCA9685     │
                                             │  对应通道，驱动舵机            │
                                             └──────────────────────────────┘
```

数据从 `face_tracking` 模块获取（左/右眼的注视归一化坐标与开度），经 `after_process`
平滑、映射为舵机角度范围，最后由 `servo_controller` 通过 PCA9685 一次性下发到多路舵机。

## 模块说明

### `servo_controller.py` — PCA9685 舵机驱动

基于 [Adafruit PCA9685](https://docs.circuitpython.org/projects/pca9685/) 的多路舵机控制器。

- `ServoController(channels=16, address=0x40, frequency=50.0, i2c=None,
  min_pulse=500, max_pulse=2500)`
  - 初始化时绑定 PCA9685 的 `channels` 个通道，每个通道对应一个
    `adafruit_motor.servo.Servo` 对象（列表索引即通道号）。
  - 默认使用 50 Hz 的 PWM 频率（模拟舵机常见频率）。
  - `i2c` 为 `None` 时自动检测默认 SCL/SDA 引脚；也可传入已有的
    `busio.I2C` 对象以便测试或复用总线。
  - `min_pulse` / `max_pulse`：0° / 180° 对应的 PWM 脉宽（µs），默认
    500 / 2500（常见 180° 舵机如 SG90 / MG90S / MG996R 的标称范围）。
    创建 `adafruit_motor.servo.Servo` 时显式传入，避免使用库默认的
    750~2250 µs 导致 0°/180° 指令行程不足（实测偏转 < 180°）。
    注意脉宽范围应与舵机机械行程标称一致，设置过宽会在端点堵转。
- `set_angle(angles)`：接收角度列表（如 `[1, 34, 29, ...]`），把第 `i` 个角度
  写到索引为 `i` 的通道；超出 `[0, 180]` 的角度自动钳制到边界值；角度数量超过
  通道数时抛出 `ValueError`。
- `deinit()`：释放 PCA9685 资源（停用 PWM 输出）；同时也实现了上下文管理器
  （`with ServoController() as sc:`），退出 `with` 块自动释放。

### `after_process.py` — 信号后处理

把原始追踪信号转换为舵机角度前需要经过的两道工序：

- `Filter(alpha=0.1, initial_value=0.0)` — 指数移动平均（EMA）滤波器
  - `update(value)` 返回 `(value, after)`：`value` 为原始输入，`after` 为平滑后输出。
  - 输入为 `None` 时不更新内部状态，直接返回上一次的有效值（hold-last），
    避免下游对 `None` 做算术运算，同时保证输出曲线平滑。
  - `alpha` 越小平滑力度越大（响应越慢）。
- `Mapper(points=None, range=(0.0, 180.0))` — 自然三次样条插值映射器
  - 通过样本点建立三次样条曲线，把输入 `x` 映射为归一化 `y`（约定取值 `[0, 1]`），
    再线性映射到 `range` 指定的实际输出范围（默认即舵机角度 `0~180`）。
  - 样本点格式：`[(x_0, y_0), (x_1, y_1), ..., (x_i, y_i)]`，默认
    `[(0,0), (1,1)]`（即原样直通）；可通过 `set_points()` 重新设置并自动重建样条。
  - 要求至少 2 个样本点且 `x` 严格递增，否则抛 `ValueError`。
  - 输入超出样本点范围时钳制到边界值（不外推）。

### `slots.py` — 数据提供者汇总

`Slots` 是「提供者」的容器：在 `__init__` 中实例化各提供者并把 UI 挂到
主窗口，`get_all_output()` 统一汇总所有参数输出。

- 目前内置的唯一提供者是眼球追踪模块
  `launch_debug_panel(master=root)`（来自 `src/face_tracking/eye_tracker_main.py`）：
  面板以 `Toplevel` 挂到已有主窗口，关闭面板不会停止追踪。
  导入时通过 `sys.path` 动态加入 `src/face_tracking/` 目录以兼容本仓库布局。
- `get_all_output()` 调用 `eye_tracker.get_normalized_eye_state()`，返回字典：

  ```python
  {
      "left_eye_x":  ... ,  # 左眼注视 X（约 [-1, 1]）
      "left_eye_y":  ... ,  # 左眼注视 Y（约 [-1, 1]）
      "left_eye_o":  ... ,  # 左眼开度（约 [0, 1]）
      "right_eye_x": ... ,
      "right_eye_y": ... ,
      "right_eye_o": ... ,
  }
  ```

- 代码中标注了 `#---此处可拓展其他提供者/输出`，如需接入新模块只需在
  `__init__` 实例化、在 `get_all_output()` 中追加字段即可。

## 依赖

- CircuitPython 库：`adafruit_pca9685`、`adafruit_motor`（含 `board`、`busio`）
- `numpy`（`after_process.py` 的样条计算）
- 眼球追踪模块（`src/face_tracking/eye_tracker_main.py`）及其依赖
  （OpenCV、PIL、PyYAML、pybind11 编译的追踪核心等，详见该模块文档）

## 快速上手

```python
from servo_controller import ServoController
from after_process import Filter, Mapper

# 1. 角度后处理：平滑 + 映射到 0~180°
smoother = Filter(alpha=0.2)
mapper   = Mapper(points=[(-1.0, 0.0), (0.0, 0.5), (1.0, 1.0)], range=(0.0, 180.0))

# 2. 驱动：with 块结束时自动 deinit
with ServoController(channels=16, address=0x40, frequency=50.0) as sc:
    raw = 0.3                     # 模拟一路追踪信号
    _, smooth = smoother.update(raw)
    angle = mapper.get_result(smooth)
    sc.set_angle([angle, 90.0])   # 通道 0 / 通道 1 同时下发
```

## 注意事项

- `servo_controller.py` 依赖真实 I2C 硬件与 PCA9685，在没有硬件的环境中
  import/实例化会失败；`after_process.py` 与 `slots.py` 不涉及硬件，
  可独立测试。
- `slots.py` 依赖眼球追踪模块，导入前请确保摄像头配置完成、追踪已启动，
  否则 `get_normalized_eye_state()` 返回全 `None` 占位快照。
- 舵机角度范围默认 `[0, 180]`，超出会自动钳制；`Mapper` 的 `range` 应与
  实际舵机机械行程保持一致，避免打舵到限位。
