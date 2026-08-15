"""PCA9685 舵机控制器。

提供 ServoController 类：管理一块 PCA9685 上的多路舵机，
set_angle() 接收角度值列表（形如 [1, 34, 29, ...]），将第 i 个角度
写到对应索引 i 的舵机通道；超出 [0, 180] 的角度自动钳制到边界值。

创建舵机对象时显式传入 min_pulse / max_pulse（默认 500 / 2500 µs），
避免 adafruit_motor.servo.Servo 的库默认值（750 / 2250 µs）把 0°/180°
指令映射到偏窄脉宽，导致实测行程不足 180°。
"""

import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class ServoController:
    """基于 PCA9685 的多路舵机控制器。

    每个通道预先绑定一个 adafruit_motor.servo.Servo 对象，
    调用 set_angle() 即可一次下发多路角度。
    """

    MIN_ANGLE = 0.0     # 角度下限（度）
    MAX_ANGLE = 180.0   # 角度上限（度）

    def __init__(self, channels: int = 16, address: int = 0x40,
                 frequency: float = 50.0, i2c=None,
                 min_pulse: int = 500, max_pulse: int = 2500):
        """初始化 PCA9685 并创建各通道的舵机对象。

        Parameters
        ----------
        channels : int
            使用的舵机通道数（PCA9685 最多 16 路），默认 16。
        address : int
            PCA9685 的 I2C 地址，默认 0x40。
        frequency : float
            PWM 频率（Hz）。模拟舵机通常使用 50 Hz，默认 50。
        i2c : busio.I2C | None
            外部传入的 I2C 总线；为 None 时自动检测默认 I2C 引脚（SCL/SDA）。
            传入已有总线可便于测试/复用。
        min_pulse : int
            0° 对应的 PWM 脉宽（µs）。默认 500，对应常见 180° 舵机
            （SG90 / MG90S / MG996R 等）的标称下限；请按舵机数据手册调整。
        max_pulse : int
            180° 对应的 PWM 脉宽（µs）。默认 2500，对应常见 180° 舵机的
            标称上限。注意：脉宽范围应与舵机机械行程标称一致，设置过宽
            会在端点堵转并增大电流。

        Notes
        -----
        默认 500~2500 µs 相比 adafruit_motor.servo.Servo 的库默认
        （750~2250 µs）覆盖了大多数 180° 舵机的完整行程；若沿用库默认，
        0°/180° 指令会被映射到偏窄的脉宽，实测行程不足 180°。
        """
        if i2c is None:
            i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(i2c, address=address)
        self._pca.frequency = frequency
        # 每个通道一个舵机对象，列表索引即通道号。
        # 显式传入 min_pulse/max_pulse：使用库默认 750/2250 µs 会导致
        # 0°/180° 指令对应脉宽偏窄，实测行程小于 180°。
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self._servos = [servo.Servo(self._pca.channels[i],
                                    actuation_range=180,
                                    min_pulse=min_pulse,
                                    max_pulse=max_pulse)
                        for i in range(channels)]

    def set_angle(self, angles):
        """将角度值列表下发到对应索引的舵机。

        Parameters
        ----------
        angles : list[float] | tuple[float]
            形如 [1, 34, 29, ...] 的角度列表，第 i 个值作用于第 i 路舵机；
            超出 [0, 180] 的角度会被钳制到边界值。

        Raises
        ------
        ValueError
            角度数量多于舵机通道数时抛出。
        """
        if len(angles) > len(self._servos):
            raise ValueError(
                f"角度数量({len(angles)})超过舵机通道数({len(self._servos)})"
            )
        for index, angle in enumerate(angles):
            clamped = min(max(float(angle), self.MIN_ANGLE), self.MAX_ANGLE)
            self._servos[index].angle = clamped

    def deinit(self):
        """释放 PCA9685 资源（停用芯片 PWM 输出）。"""
        self._pca.deinit()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.deinit()

