# LiveSuit
A personal face-tracking robotic fursuit project.

这是一个个人级的面捕可动兽装项目。

## 环境依赖

### 系统包 (Debian / Raspberry Pi OS)

```bash
sudo apt install -y cmake g++ libopencv-dev libyaml-cpp-dev pybind11-dev swig liblgpio-dev
```

> 国内网络可换中科大镜像（`/etc/apt/sources.list.d/debian.sources` 中把
> `deb.debian.org` 替换为 `mirrors.ustc.edu.cn`）后加速安装。

### Python 依赖

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` 内含 Python 直接依赖（opencv、numpy、PyYAML、Pillow、
adafruit-circuitpython-pca9685）。硬件舵机驱动会连带安装 Blinka/lgpio 依赖链，
`board`/`busio` 需在真实 I2C 硬件上才可用。

### C++ 追踪核心 (pybind11 模块)

`src/face_tracking/eye_tracker_core_cpp` 需要单独编译，产物是
`eye_tracker_core_cpp.cpython-*.so`：

```bash
cd src/face_tracking/cpp
cmake -S . -B build
cmake --build build -j4
```

编译出的 `.so` 会输出到 `src/face_tracking/`，供 `eye_tracker_core.py` 导入。
请在 venv 激活状态下编译，确保链接到对应 Python 版本。

### 运行

```bash
source .venv/bin/activate
python3 src/gui.py
```

## 项目扩展

数据流自上而下分四层，每层都能替换或扩展：

```
信息源(面部算法) --原始值--> 参数输出层(归一化 [0,1]) --参数--> 效果器图 --参数--> 舵机通道(样条) --脉宽--> 舵机总线
```

### 信息源（面部算法）

- 抽象接口：`core.sources.DataSource.read() -> {参数名: 原始值}`；现成的
  `CallableSource(getter)` 可直接包装任意「返回字典」的可调用对象。
- 面部算法实现放在 `src/face_tracking/`，目前是 `eye_tracker_main.EyeTrackingModule`，
  通过 `get_normalized_eye_state()` 返回实时快照；算法本体跑在独立子进程，扩展算法
  （如嘴部追踪）时沿用这一「子进程算、主进程取快照」的模式即可。
- 对接容器是 `servo_control/slots.py` 的 `Slots`：在 `get_all_output()` 里汇总各提供者
  输出，返回 `{槽位名: 原始值}`。新增信息源时：
  1. 在 `Slots.__init__` 里实例化你的提供者；
  2. 在 `get_all_output()` 里把它的输出并入返回字典（用提供者原始键）；
  3. 在 `SLOT_SPECS` 注册槽位 `槽位名 -> {"label": ..., "in_range": (lo, hi)}`，
     `in_range` 决定该原始值到 `[0,1]` 的线性归一化范围；
  4. 在 `PROVIDER_KEY_MAP` 里把提供者原始键映射为槽位名；
  5. 需要时更新 `PROVIDER_INFO`（欢迎页展示的参数清单）。
- 数据线程以 30fps 调用 `read()`，因此它必须是**非阻塞的快照读取**，且**不得**调用
  `store.set` 或任何 tkinter API。

### 效果器插件

- 继承 `core.effectors.Effector`，至少实现三个方法：
  - `get_input_count()` / `get_output_count()`：端口数量。端口是**位置索引**，具体语义
    写在子类 docstring 里；
  - `process(inputs) -> outputs`：数据线程每帧调用，`inputs` 按端口顺序传入，上游无效值
    为 `None`（需 hold-last 或自行降级）。
- 可选覆盖：
  - `TYPE_NAME`：注册表 / `pipeline.yaml` 使用的类型名；
  - `reset()`：清空内部历史状态；
  - `get_params()` / `set_params(params)`：可持久化参数（写入 `pipeline.yaml`）；
  - `snapshot()`：返回供 GUI 读取的加锁遥测快照（无遥测返回 `None`）；
  - `show_panel(parent)` / `close_panel()`：自带 tkinter 面板（`effects/` 层**允许**
    import tkinter）。
- 有状态效果器必须自行加锁：`process()` 在数据线程、面板在主线程，参考
  `effects/ema.py` 的 `EMAEffector`（`threading.RLock` + 加锁 `snapshot()`）。面板用
  `after` 定时重绘读取 snapshot，不要直接访问内部可变状态。

### 注册效果器

- 在 `effects/__init__.py` 里导入你的类并加入 `EFFECTOR_TYPES`：

  ```python
  from effects.my_effector import MyEffector
  EFFECTOR_TYPES = {"EMA": EMAEffector, "MyEffector": MyEffector}
  ```

- 节点图「新增效果器」菜单会自动列出 `EFFECTOR_TYPES` 的键。实例化、命名、接线与
  持久化均由 `core.pipeline.PipelineManager` 负责，无需改动编排层或 GUI 框架。

### 扩展的共享格式（发布与引入）

本仓库没有二进制插件加载器，扩展以**源码 + 注册片段**的形式分发：发布者交付一组自包含的
源码文件与一段「在哪个注册表加哪一行」的集成说明，使用者把这些文件落到指定目录、补齐注册
即可接入。运行期配置（`pipeline.yaml` 等）由程序自动生成，**不随扩展分发**。

**效果器扩展**

- 交付物：一个自包含模块（如 `src/effects/my_effector.py`），只依赖 `core.*` 与
  `requirements.txt` 中已有的库，**不得**擅自新增第三方依赖（见 `AGENTS.md` 约定）。
- 类继承 `core.effectors.Effector` 并设置 `TYPE_NAME`。`TYPE_NAME` 是
  `pipeline.yaml` 的 `type` 字段，**发布后不要改名**，否则他人已有的编排无法载入。
- 命名空间前缀防冲突：`EFFECTOR_TYPES` 是普通字典，同名会互相覆盖，建议用带前缀的类型名
  （如 `MyProj.MyFilter`），并在说明里写清端口数量与端口语义。
- 引入方式：把文件放进 `src/effects/`，然后在 `effects/__init__.py` 中 import 并加入
  `EFFECTOR_TYPES`。节点图「新增效果器」菜单随即出现该类型。

**信息源（面部算法）扩展**

- 交付物：一个自包含的算法包（如 `src/face_tracking/<name>/`）＋一份「槽位声明」，为它
  暴露的每个参数声明槽位名、显示名 `label`、原始值范围 `in_range`、提供者原始键。
- 算法可以是纯 Python，也可以像现有眼追一样带 C++ 核心（此时需附 `cmake` 构建说明和产物
  的放置位置）；对外只需提供一个**非阻塞**的取值接口（`read()` 风格的
  `{原始键: 原始值}` 快照）。
- 槽位名即参数输出的**全局名**，同样建议加前缀防冲突。
- 引入方式：在 `slots.py` 的 `Slots.__init__` 里实例化该提供者，在 `get_all_output()` 里
  把它的输出并入返回字典，再把槽位声明登记到 `SLOT_SPECS` / `PROVIDER_KEY_MAP` /
  `PROVIDER_INFO`。

**接入契约与兼容性**

- 扩展面向的稳定契约是：`Effector` 的端口语义、`DataSource.read()` 的返回结构、各配置
  文件的字段名，以及「参数/端口值归一化在 `[0, 1]`、绑定舵机后为脉宽 µs」的数值约定。
  遵循它们即可跨版本复用。
- 一个合规的扩展应**只改注册表**，不改动 `core/` 与 `gui/`；若确实需要改，说明其接口不
  兼容，须在扩展说明中显式标注。

### 运行期配置

运行期配置统一放在 `src/` 下，与当前工作目录无关。

- `pipeline.yaml`（`core/config_io.py` 读写）——效果器编排：
  - `effectors`：`实例名 -> {type, label, inputs, params}`，`inputs` 为按端口顺序的接线
    引用，空位为 `null`；
  - `channels`：`通道名 -> {input, point_set, servo}`，`point_set` 为归一化样条样本点，
    `servo` 为绑定的舵机索引（`null` 表示未绑定）；
  - `layout`：节点键 `-> [x, y]` 的画布布局；
  - 接线引用（source_ref）统一为 `["param", 参数名]` 或 `["effector", 实例名, 端口]`。
- `servo_configs.yaml`（`core/servo_limits.py`）——舵机脉宽注册表：顶层
  `global_min_pulse` / `global_max_pulse` / `global_safe_pulse_min` /
  `global_safe_pulse_max` 为默认值，`servo_N` 条目按需覆盖；每路的 `min_pulse` /
  `max_pulse` 即绑定后通道输出的脉宽范围（µs）。
- `references.yaml`（`face_tracking/eye_tracker_core.py`）——面部标定参考值：
  `{side}_inner/outer/up/down` 为注视极值向量，`{side}_open/close` 为开度参考。
- `camera_configs.yaml`（`face_tracking/eye_tracker_main.py`）——左右相机配置
  （翻转、裁剪、分辨率、阈值等）。

数值约定：参数输出层的值与效果器端口的值均归一化在 `[0, 1]`；舵机通道输入为
`[0, 1]`，绑定舵机后输出切换为该舵机的脉宽范围（µs），未绑定则输出 `[0, 1]`。
