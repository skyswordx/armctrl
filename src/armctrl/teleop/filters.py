from __future__ import annotations


class LowPassFilter:
    """一阶低通滤波器。

    更新公式：
        y_t = α * x_t + (1 - α) * y_(t-1)

    含义：
    - α 越大，输出越跟手；
    - α 越小，输出越平滑。
    """

    def __init__(self, alpha: float = 0.35) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self._value = 0.0
        self._initialized = False

    def update(self, value: float) -> float:
        value = float(value)
        if not self._initialized:
            # 第一次输入直接作为初值，避免起步阶段被 0 人为拉低。
            self._value = value
            self._initialized = True
        else:
            self._value = self.alpha * value + (1.0 - self.alpha) * self._value
        return self._value

    def reset(self) -> None:
        # deadman 松开时通常需要 reset，避免下一次接管沿用旧滤波状态。
        self._value = 0.0
        self._initialized = False
