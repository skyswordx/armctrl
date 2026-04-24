"""夹爪标定模块。

这里专门放项目侧的夹爪标定逻辑：
1. 保存和读取标定文件；
2. 把标定结果应用到 SDK `RobotConfig`；
3. 复用 SDK 原生 `calibrate_gripper()` 做交互式标定。

这样可以把“项目持久化配置”和“SDK 运行时控制器”分开。
普通运行命令不再暴露临时覆盖参数，
而是统一读取项目里保存的标定结果。
"""

from .gripper import GripperCalibrationService, derive_gripper_motor_readout
from .models import GripperCalibration, apply_gripper_calibration
from .store import GripperCalibrationStore

__all__ = [
    "GripperCalibration",
    "GripperCalibrationService",
    "GripperCalibrationStore",
    "apply_gripper_calibration",
    "derive_gripper_motor_readout",
]
