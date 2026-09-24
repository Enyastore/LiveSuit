# LiveSuit Agent 约定

## 工具约定（务必遵守）

- **搜索代码一律用 bash 的 grep**（例如 `grep -rn "关键字" src`）。**禁止使用 opencode 内置的 grep 工具**（存在 bug）。
- 找文件用 bash 的 `find` / `ls`，或 opencode 的 glob 工具。
- 运行 Python / 测试统一用仓库虚拟环境：`.venv/bin/python`。
- 不要新增第三方依赖；只用 `requirements.txt` 中已有的库。

## 中英翻译规则

- **代码标识符**（模块名、类名、函数名、变量名、常量名）：使用**英文**。
  - 例：`ServoChannel`、`EMAEffector`、`get_input_count`、`NORMALIZED_RANGE`。
- **注释与 docstring**：使用**中文**，风格、口吻与现有代码保持一致（解释“为什么”，而非复述代码）。
- **面向用户的字符串**（GUI 文本、提示框、`print` 日志、异常消息）：使用**中文**。
- **专有名词**保持英文：EMA、Spline、PCA9685、tkinter、YAML、PWM、I2C 等。
- **不要中英混杂地命名标识符**（如 `get_舵机向量` 是禁止的）；标识符一律纯英文。
- 不要“翻译”已存在的代码；沿用现有命名与措辞。

## 目录与分层约定

- `src/core/`：纯逻辑层，**禁止 import tkinter**。
- `src/effects/`：效果器实现（子类**允许** import tkinter，自带面板）。
- `src/gui/`：tkinter 界面层。
- `src/servo_control/`：硬件与信号原语（`after_process.py`、`servo_controller.py`、`slots.py`）。
- 公共常量/工具放对应层的独立模块，**不要造上帝模块**。

## 线程约定

- 数据流跑在 30fps 工作者线程；tkinter 仅主线程。
- 跨线程共享状态必须加锁/取快照；**数据线程禁止**调用 `store.set` 或任何 tkinter API。
