from __future__ import annotations


class LowPassFilter:
    def __init__(self, alpha: float = 0.35) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self._value = 0.0
        self._initialized = False

    def update(self, value: float) -> float:
        value = float(value)
        if not self._initialized:
            self._value = value
            self._initialized = True
        else:
            self._value = self.alpha * value + (1.0 - self.alpha) * self._value
        return self._value

    def reset(self) -> None:
        self._value = 0.0
        self._initialized = False
