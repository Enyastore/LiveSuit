"""PCA9685 舵机控制器（直接脉宽标定）。

提供 ServoController 类：管理一块 PCA9685 上的多路舵机，
set_pulse() 接收脉宽值列表（形如 [1500, 1450, None, ...]），把第 i 个脉宽
（µs）直接换算为 duty cycle 写到对应索引 i 的舵机通道。

与旧的 set_angle()（通过 adafruit_motor.Servo 做角度->脉宽换算）不同，
本实现完全绕过角度数学：每个通道只按全局安全范围 [400, 2700]µs 钳制后
直写 PCA9685，避免「厂商标称脉宽与实际偏转不一致」时角度换算引入的二次
误差。各通道的推荐范围（servo_configs.yaml 注册的 min_pulse/max_pulse）
由上层 Slot/调试工具负责约束与警示，换用不同脉宽范围的舵机型号时只需
调整注册表，无需改动控制逻辑。
"""

import board
import busio
from adafruit_pca9685 import PCA9685

# 全局绝对安全脉宽范围（µs）：任何通道下发都不允许越过该硬边界
# （与 pipeline_manager.PULSE_SAFE_MIN / PULSE_SAFE_MAX 保持一致，
#  对应实验脚本实测的 400~2700µs 无堵转区间）。
PULSE_SAFE_MIN = 400.0
PULSE_SAFE_MAX = 2700.0


class ServoController:
    """基于 PCA9685 的多路舵机控制器（直接写脉宽 duty cycle）。

    只向 servo_configs.yaml 中注册过的通道下发；下发时硬钳制到全局安全
    范围 [400, 2700]µs（各通道注册的 [min_pulse, max_pulse] 由上层约束）。
    """

    def __init__(self, pulse_configs=None, address: int = 0x40,
                 frequency: float = 50.0, i2c=None):
        """初始化 PCA9685 并登记各通道的脉宽范围。

        Parameters
        ----------
        pulse_configs : dict | None
            {通道索引: (min_pulse, max_pulse)}（µs），仅这些注册通道可写。
            索引须在 0~15 且 0 < min_pulse < max_pulse，非法项自动忽略。
        address : int
            PCA9685 的 I2C 地址，默认 0x40。
        frequency : float
            PWM 频率（Hz）。模拟舵机通常使用 50 Hz，默认 50。
        i2c : busio.I2C | None
            外部传入的 I2C 总线；为 None 时自动检测默认 I2C 引脚（SCL/SDA）。
            传入已有总线可便于测试/复用。

        Notes
        -----
        每路舵机的 min/max 脉宽来自 servo_configs.yaml 注册表（无该文件时
        gui 会按默认 500~2500 生成），因此不同通道可用不同的脉宽范围，
        换用其他型号舵机只需修改注册表，无需改代码。
        """
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(i2c, address=address)
        self._pca.frequency = frequency
        # 只保留合法注册通道
        self.pulse_configs = {}
        for idx, (lo, hi) in dict(pulse_configs or {}).items():
            idx = int(idx)
            if 0 <= idx < 16 and 0.0 < float(lo) < float(hi):
                self.pulse_configs[idx] = (float(lo), float(hi))
        # 已下发脉宽缓存：值未变化时跳过重复 I2C 写，降低总线流量与 UI 阻塞
        self._last_pulses = {}

    @staticmethod
    def _pulse_to_duty(pulse_us: float, frequency: float) -> int:
        """把脉宽（µs）换算为 PCA9685 16-bit duty cycle（0~65535）。"""
        duty = int(round(pulse_us * frequency * 65535.0 / 1_000_000.0))
        return max(0, min(65535, duty))

    def set_pulse(self, pulses):
        """将脉宽值列表下发到对应索引的舵机。

        Parameters
        ----------
        pulses : list[float | None] | tuple[float | None]
            形如 [1500, 1450, None, ...] 的脉宽列表（µs），第 i 个值作用于
            第 i 路舵机；None 或未注册通道跳过不写；已注册通道硬钳制到
            全局安全范围 [400, 2700]µs。

        Notes
        -----
        钳制到安全范围而非各通道注册的 [min,max]：运行管线侧的值已在
        PipelineManager / Slot 中按绑定舵机注册范围钳制（⊂ 安全范围），
        因此不受影响；而舵机调试工具可以在安全范围内探索注册范围之外的
        脉宽，以实测最佳值。
        """
        for index in self.pulse_configs:
            if index >= len(pulses) or pulses[index] is None:
                continue
            pulse = max(PULSE_SAFE_MIN, min(PULSE_SAFE_MAX, float(pulses[index])))
            if self._last_pulses.get(index) == pulse:
                continue
            self._pca.channels[index].duty_cycle = self._pulse_to_duty(
                pulse, self._pca.frequency)
            self._last_pulses[index] = pulse

    def deinit(self):
        """释放 PCA9685 资源（停用芯片 PWM 输出）。"""
        self._pca.deinit()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.deinit()

