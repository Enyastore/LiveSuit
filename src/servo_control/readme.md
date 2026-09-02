# 舵机控制代码

本文件夹是 LiveSuit 项目中「把眼球追踪数据变成舵机动作」的控制链路，包含舵机驱动、信号后处理与多模块输出汇总三部分。

> ⚠️ **脉宽标定**：本项目不再使用「角度（0~180°）」标定运动。管线输出、
> 舵机调试工具与样条曲线全部以 **PWM 脉宽（µs）** 为单位；每路舵机在
> `src/servo_configs.yaml` 中注册独立的 `min_pulse` / `max_pulse`，
> 换用其他型号舵机只需修改注册表，无需改动代码或重调曲线形状。

## 目录结构

```
src/servo_control/
├── servo_controller.py   # PCA9685 舵机驱动（ServoController，直接写脉宽 duty）
├── after_process.py      # 信号后处理（Filter 平滑 / Mapper 样条映射）
├── slots.py              # 数据提供者汇总（Slots 类，目前对接眼球追踪模块）
└── readme.md             # 本文档
src/
├── servo_configs.yaml    # 舵机脉宽注册表（每路舵机的 min_pulse/max_pulse）
└── slot_configs.yaml     # 槽位曲线配置（alpha + 归一化样本点）
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
                                              │  Mapper   → 三次样条映射到脉宽 │
                                              │             （µs）             │
                                              └───────────┬──────────────────┘
                                                          ▼
                                              ┌──────────────────────────────┐
                                              │ ServoController.set_pulse()  │
                                              │  将脉宽列表直写 PCA9685 通道， │
                                              │  每路按自身 min/max 钳制       │
                                              └──────────────────────────────┘
```

数据从 `face_tracking` 模块获取（左/右眼的注视归一化坐标与开度），经 `after_process`
平滑、映射为脉宽（µs），最后由 `servo_controller` 通过 PCA9685 一次性下发到多路舵机。

槽位样条曲线的输出范围（MIN~MAX）由该槽位**绑定的舵机**决定：未绑定时为默认
500~2500µs；绑定 `servo_N` 后自动切换为该舵机在 `servo_configs.yaml` 中注册的
`[min_pulse, max_pulse]`。曲线以**归一化形状**存储，换绑不同脉宽范围的舵机时
形状保持不变，仅 Y 轴数值按新范围重新标定。

## servo_configs.yaml —— 舵机脉宽注册表

`src/servo_configs.yaml` 声明实际使用的舵机及其脉宽范围。每次运行 `gui.py` 都会读取：

```yaml
servo_0:
  min_pulse: 450
  max_pulse: 2650
servo_1:
  min_pulse: 450
  max_pulse: 2650
```

规则：
- **只有写入该文件且合法的舵机才可用**（调试工具 / 总线面板 / 逐帧下发均只作用于注册舵机）；
- 文件缺失时自动生成默认 `servo_0 ~ servo_15`（500~2500µs）；
- 文件不合法（YAML 解析失败 / 顶层非字典 / 无任何有效条目）时打印错误并弹出提示，
  同时用默认参数**覆盖**该文件；
- 单条非法（键非 `servo_N` / 通道越界 / 不满足 `0 < min_pulse < max_pulse`）
  会被丢弃并打印提示，其余合法条目照常生效。

## 模块说明

### `servo_controller.py` — PCA9685 舵机驱动

基于 [Adafruit PCA9685](https://docs.circuitpython.org/projects/pca9685/) 的多路舵机控制器。

- `ServoController(pulse_configs=None, address=0x40, frequency=50.0, i2c=None)`
  - `pulse_configs`：`{通道索引: (min_pulse, max_pulse)}`（µs），仅这些注册通道可写。
  - 不再使用 `adafruit_motor.Servo` 的角度换算，直接计算 duty cycle 写入 PCA9685，
    从根本上避免「角度 -> 脉宽」换算误差。
- `set_pulse(pulses)`：接收脉宽列表（如 `[1500, 1450, None, ...]`，µs），第 `i` 个值
  写到索引 `i` 的通道；`None` 或未注册通道跳过不写；已注册通道硬钳制到
  全局安全范围 **400~2700µs**。
  > 钳制到安全范围而非各通道注册范围：运行管线侧的值已在 Slot 中按绑定舵机的
  > 注册范围钳制（⊂ 安全范围），不受影响；而舵机调试工具可以在安全范围内
  > 探索注册范围之外的脉宽，用于实测找最佳值并写回 `servo_configs.yaml`。
- `deinit()`：释放 PCA9685 资源（停用 PWM 输出）；也实现了上下文管理器。

### 舵机调试工具（GUI：打开舵机工具）

- 允许设置 `servo_configs.yaml` 推荐范围之外的脉宽（方便实测机械行程极限），
  但**不可超过全局安全范围 [400, 2700]µs**（超出自动钳制）。
- 当数值超出该通道配置范围时，输入框与提示变**深红色**，并显示
  「⚠ 超出配置范围 …µs，可能有舵机损坏风险」；回到范围内自动恢复正常显示。
- 找到最佳脉宽后，把对应通道的 `min_pulse` / `max_pulse` 写回 `servo_configs.yaml`。

### `after_process.py` — 信号后处理

把原始追踪信号转换为舵机脉宽前需要经过的两道工序：

- `Filter(alpha=0.1, initial_value=0.0)` — 指数移动平均（EMA）滤波器
  - `update(value)` 返回 `(value, after)`：`value` 为原始输入，`after` 为平滑后输出。
  - 输入为 `None` 时不更新内部状态，直接返回上一次的有效值（hold-last）。
  - `alpha` 越小平滑力度越大（响应越慢）。
- `Mapper(points=None, range=(500.0, 2500.0))` — 自然三次样条插值映射器
  - 通过样本点建立三次样条曲线，把输入 `x` 映射为归一化 `y`（约定取值 `[0, 1]`），
    再线性映射到 `range` 指定的实际输出范围（µs，默认即默认舵机脉宽范围）。
  - 样本点格式：`[(x_0, y_0), (x_1, y_1), ...]`，默认 `[(0,0), (1,1)]`。
  - 要求至少 2 个样本点且 `x` 严格递增，否则抛 `ValueError`。
  - 输入超出样本点范围时钳制到边界值（不外推）。

### `slots.py` — 数据提供者汇总

`Slots` 是「提供者」的容器：在 `__init__` 中实例化各提供者并把 UI 挂到主窗口，
`get_all_output()` 统一汇总所有参数输出。

- 目前内置的唯一提供者是眼球追踪模块
  `launch_debug_panel(master=root)`（来自 `src/face_tracking/eye_tracker_main.py`）。
- 槽位注册表 `SLOT_SPECS` 只声明 `label` 与 `in_range`，**不声明输出范围**——
  输出范围由绑定舵机的脉宽注册表决定，保证换舵机型号时无需改代码。
- 代码中标注了 `#---此处可拓展其他提供者/输出`，如需接入新模块只需在
  `__init__` 实例化、在 `get_all_output()` 中追加字段即可。

## 依赖

- CircuitPython 库：`adafruit_pca9685`（含 `board`、`busio`）
- `numpy`（`after_process.py` 的样条计算）
- 眼球追踪模块（`src/face_tracking/eye_tracker_main.py`）及其依赖
  （OpenCV、PIL、PyYAML、pybind11 编译的追踪核心等，详见该模块文档）

## 快速上手

```python
from servo_controller import ServoController
from after_process import Filter, Mapper

# 1. 后处理：平滑 + 映射到默认脉宽范围（500~2500µs）
smoother = Filter(alpha=0.2)
mapper   = Mapper(points=[(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)],
                  range=(500.0, 2500.0))

# 2. 驱动：with 块结束时自动 deinit；只写注册过的通道
with ServoController(pulse_configs={0: (450, 2650), 1: (500, 2500)}) as sc:
    raw = 0.3                     # 模拟一路追踪信号
    _, smooth = smoother.update(raw)
    pulse = mapper.get_result(smooth)
    sc.set_pulse([pulse, None])   # 通道 0 下发，其余不写
```

## 注意事项

- `servo_controller.py` 依赖真实 I2C 硬件与 PCA9685，在没有硬件的环境中
  import/实例化会失败；`after_process.py` 与 `slots.py` 不涉及硬件，
  可独立测试。
- `slots.py` 依赖眼球追踪模块，导入前请确保摄像头配置完成、追踪已启动。
- 每路舵机的 `min_pulse` / `max_pulse` 应与其实际机械行程一致；设置过宽会在
  端点堵转并增大电流（可先用 `experiment/test_servo.py` 的 pulse 模式实测）。
- 槽位样条曲线（`slot_configs.yaml`）的样本点 y 以**归一化 [0,1]** 存储，
  与绑定舵机无关；GUI 中显示为当前绑定舵机的真实脉宽（µs）。
