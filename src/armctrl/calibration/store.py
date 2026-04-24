"""夹爪标定文件读写。

标定结果保存在项目目录 `configs/calibration/gripper/<model>.json`。
这样做的目的有两个：
1. 把“这台项目机械臂的经验值”从启动命令里拿出来；
2. 让 teleop、health、identification 这些链路共享同一份配置。
"""

from __future__ import annotations

import json
from pathlib import Path

from .models import GripperCalibration


class GripperCalibrationStore:
    """项目侧夹爪标定存储。"""

    _PROJECT_ROOT = Path(__file__).resolve().parents[3]
    _DEFAULT_DIR = _PROJECT_ROOT / "configs" / "calibration" / "gripper"

    def __init__(self, root_dir: Path | None = None) -> None:
        self.root_dir = Path(root_dir) if root_dir is not None else self._DEFAULT_DIR

    def path_for_model(self, model: str) -> Path:
        # 文件名直接用模型名，便于人工查找和版本管理。
        return self.root_dir / f"{model}.json"

    def load(self, model: str) -> GripperCalibration | None:
        path = self.path_for_model(model)
        if not path.is_file():
            return None
        payload = json.loads(path.read_text(encoding="utf-8"))
        return GripperCalibration.from_dict(payload)

    def save(self, calibration: GripperCalibration) -> Path:
        path = self.path_for_model(calibration.model)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(calibration.to_dict(), ensure_ascii=False, sort_keys=True, indent=2) + "\n",
            encoding="utf-8",
        )
        return path

    def clear(self, model: str) -> Path | None:
        path = self.path_for_model(model)
        if not path.exists():
            return None
        path.unlink()
        return path
