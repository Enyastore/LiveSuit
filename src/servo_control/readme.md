# 舵机控制代码

本文件夹是 LiveSuit 项目中「硬件与信号原语」的底层：PCA9685 舵机驱动，以及
参数输出层的数据来源。信号处理（EMA 滤波 / 样条映射）与三层编排分别在
`src/core/` 与 `src/effects/` 中实现，见下文。

> ⚠️ **脉宽标定**：本项目不再使用「角度（0~180°）」标定运动。管线输出、
> 舵机调试工具与样条曲线全部以 **PWM 脉宽（µs）** 为单位；每路舵机在
> `src/servo_configs.yaml` 中注册独立的 `min_pulse` / `max_pulse`，
> 换用其他型号舵机只需修改注册表，无需改动代码或重调曲线形状。

## 目录结构

```
src/servo_control/
├── servo_controller.py   # PCA9685 舵机驱动（ServoController，直接写脉宽 duty）
├── slots.py              # 参数输出层入口（SLOT_SPECS + Slots 数据提供者）
└── readme.md             # 本文档
src/core/
├── primitives.py         # EMA Filter / 样条 Mapper（纯数学原语）
├── channels.py           # 舵机控制层（ServoChannel：样条 + 绑定）
└── pipeline.py           # 编排层 PipelineManager（参数 -> 效果器 -> 通道）
src/effects/
└── ema.py                # EMAEffector（效果器范例，自带面板）
src/
├── servo_configs.yaml    # 舵机脉宽注册表（顶层 global_* 默认 + 各通道覆盖）
└── pipeline.yaml         # 编排配置（效果器 / 连线 / 通道 / 绑定 / 布局）
```

## 整体数据流

信息流分三层，由 `core/pipeline.py` 的 `PipelineManager` 编排（接线持久化在
`src/pipeline.yaml`）：

```
src/face_tracking/eye_tracker_main.py        src/core/ + src/effects/
┌─────────────────────────────┐  get_all_output  ┌──────────────────────────────┐
│ EyeTrackingModule           │─────────────────▶│ 参数输出层 ParamSource        │
│  .get_normalized_eye_state()│  {slot_name: val}│  归一化 -> [0,1]（in_range）   │
└─────────────────────────────┘                  └───────────────┬──────────────┘
                                                                 ▼
                                              ┌──────────────────────────────┐
                                              │ 效果器层 Effector（可多入多出）│
                                              │  EMAEffector / 混合器 …       │
                                              └───────────────┬──────────────┘
                                                              ▼
                                              ┌──────────────────────────────┐
                                              │ 舵机控制层 ServoChannel       │
                                              │  Mapper(样条) -> 未绑定归一化  │
                                              │  / 绑定后映射为脉宽（µs）      │
                                              └───────────────┬──────────────┘
                                                              ▼
                                              ┌──────────────────────────────┐
                                              │ ServoController.set_pulse()  │
                                              │  将脉宽列表直写 PCA9685 通道   │
                                              └──────────────────────────────┘
```

数据从 `face_tracking` 模块获取（左/右眼的注视归一化坐标与开度），在**参数输出层**
按 `in_range` 归一化，经**效果器层**（如 EMA 平滑）后由**舵机控制层**的三次样条映射
为脉宽（µs），最后由 `servo_controller` 通过 PCA9685 一次性下发到多路舵机。

通道在**未绑定舵机**时输出为纯归一化值（0~1），不涉及脉宽；**绑定 `servo_N`**
后自动切换为该舵机在 `servo_configs.yaml` 中注册的 `[min_pulse, max_pulse]`，
输出脉宽（µs）。曲线始终以**归一化形状**存储，换绑不同脉宽范围的舵机时形状保持
不变，仅 Y 轴数值按新范围重新标定。

## servo_configs.yaml —— 舵机脉宽注册表

`src/servo_configs.yaml` 声明实际使用的舵机及其脉宽范围。顶层 `global_*` 定义
全局默认，`servo_N` 条目只写与该全局不同的字段，**留空即继承全局**；每次运行
`gui.py` 都会读取：

```yaml
global_min_pulse: 500          # 机械范围下限（µs，全局默认）
global_max_pulse: 2500         # 机械范围上限
global_safe_pulse_min: 400     # 安全提示范围下限（提醒用，非硬边界）
global_safe_pulse_max: 2700    # 安全提示范围上限
servo_0:                       # 存在该键 = 注册该通道
  min_pulse: 450               # 覆盖全局机械下限
  max_pulse: 2650              # 覆盖全局机械上限
  safe_pulse_min: 400          # 覆盖全局安全下限
  safe_pulse_max: 2700         # 覆盖全局安全上限
servo_1: {}                    # 全继承全局
```

规则：
- **只有写入该文件且合法的舵机才可用**（调试工具 / 总线面板 / 逐帧下发均只作用于注册舵机）；
- 顶层 `global_*` 定义全局默认；通道条目中缺失的字段继承全局，便于统一更换型号
  或只替换单路舵机；
- `safe_pulse_*` 是**安全提示范围**（非硬边界），用于调试工具告警与钳制，提醒用户
  别把脉宽设得过于极端；未包住机械范围时仅告警；
- 文件缺失时自动生成默认 `servo_0 ~ servo_15`（全局 500~2500µs，安全 400~2700µs）；
- 文件不合法（YAML 解析失败 / 顶层非字典 / 无任何通道条目）时打印错误并弹出提示，
  同时用默认参数**覆盖**该文件；
- 单条字段非法（非数字 / `min_pulse >= max_pulse`）时**仅告警并回退全局**，仍注册该通道。

## 模块说明

### `servo_controller.py` — PCA9685 舵机驱动

基于 [Adafruit PCA9685](https://docs.circuitpython.org/projects/pca9685/) 的多路舵机控制器。

- `ServoController(pulse_configs=None, address=0x40, frequency=50.0, i2c=None, safe_limits=None)`
  - `pulse_configs`：`{通道索引: (min_pulse, max_pulse)}`（µs），仅这些注册通道可写。
  - `safe_limits`：`{通道索引: (safe_min, safe_max)}`（µs），每路的安全提示范围；
    缺省（或该通道缺省）时回退为该通道的 `[min_pulse, max_pulse]`。
  - 不再使用 `adafruit_motor.Servo` 的角度换算，直接计算 duty cycle 写入 PCA9685，
    从根本上避免「角度 -> 脉宽」换算误差。
- `set_pulse(pulses)`：接收脉宽列表（如 `[1500, 1450, None, ...]`，µs），第 `i` 个值
  写到索引 `i` 的通道；`None` 或未注册通道跳过不写；已注册通道钳制到该通道的
  安全提示范围。
  > 钳制到安全范围而非各通道注册范围：运行管线侧的值已在 ServoChannel 中按绑定
  > 舵机的注册范围钳制（⊂ 安全范围），不受影响；而舵机调试工具可以在安全范围内
  > 探索注册范围之外的脉宽，用于实测找最佳值并写回 `servo_configs.yaml`。
- `deinit()`：释放 PCA9685 资源（停用 PWM 输出）；也实现了上下文管理器。

### 舵机调试工具（GUI：打开舵机工具）

- 允许设置 `servo_configs.yaml` 推荐范围之外的脉宽（方便实测机械行程极限），
  但**不超过该通道的安全提示范围**（缺省为该通道 `[min_pulse, max_pulse]`，
  也可由 `safe_pulse_min/max` 覆盖；超出自动钳制）。
- 当数值超出该通道配置范围时，输入框与提示变**深红色**，并显示
  「⚠ 超出配置范围 …µs，可能有舵机损坏风险」；回到范围内自动恢复正常显示。
- 找到最佳脉宽后，把对应通道的 `min_pulse` / `max_pulse` 写回 `servo_configs.yaml`。

### 信号原语与效果器（不在本文件夹）

EMA 滤波与三次样条已按分层各归其位，不再作为并列的「后处理」：

- `core/primitives.py` —— 两个纯数学原语：
  - `Filter(alpha=0.1, initial_value=0.0)` 指数移动平均（EMA）；`update(value)`
    返回 `(value, after)`，输入 `None` 时 hold-last。
  - `Mapper(points=None, range=(0.0, 1.0))` 自然三次样条插值；把归一化 `x` 映射为
    归一化 `y` 后线性映射到 `range`（缺省 `(0,1)`；绑定舵机时传 `(min,max)` µs）。
- `effects/ema.py` —— `EMAEffector`（效果器范例，内部使用 `Filter`）。
- `core/channels.py` —— `ServoChannel`（舵机端点，内部使用 `Mapper`）。

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
- `numpy`（`core/primitives.py` 的样条计算）
- 眼球追踪模块（`src/face_tracking/eye_tracker_main.py`）及其依赖
  （OpenCV、PIL、PyYAML、pybind11 编译的追踪核心等，详见该模块文档）

## 快速上手

信号链路由 `core/pipeline.py` 的 `PipelineManager` 统一编排（参数输出 -> 效果器
-> 舵机通道），接线、通道样条与节点布局持久化在 `src/pipeline.yaml`。直接
`python3 src/gui.py` 启动，在「效果器编排」窗口里增删/接线效果器即可。

如需单独使用底层数学原语：

```python
from core.primitives import Filter, Mapper

smoother = Filter(alpha=0.2)
mapper = Mapper(points=[(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)],
                range=(500.0, 2500.0))
_, smooth = smoother.update(0.3)
pulse = mapper.get_result(smooth)
```

## 注意事项

- `servo_controller.py` 依赖真实 I2C 硬件与 PCA9685，在没有硬件的环境中
  import/实例化会失败；`core/primitives.py` 与 `slots.py` 不涉及硬件，
  可独立测试。
- `slots.py` 依赖眼球追踪模块，导入前请确保摄像头配置完成、追踪已启动。
- 每路舵机的 `min_pulse` / `max_pulse` 应与其实际机械行程一致；设置过宽会在
  端点堵转并增大电流（可先用 `experiment/test_servo.py` 的 pulse 模式实测）。
- 通道样条曲线（存于 `pipeline.yaml` 各通道的 `point_set`）的样本点 y 以
  **归一化 [0,1]** 存储，与绑定舵机无关；GUI 中显示为当前绑定舵机的真实脉宽（µs）。
