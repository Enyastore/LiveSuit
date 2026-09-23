# LiveSuit
A personal face-tracking robotic fursuit project.

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
