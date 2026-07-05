from __future__ import annotations

import subprocess
from pathlib import Path


def test_lab_teleop_teach_script_help_lists_runtime_owned_flow() -> None:
    root = Path(__file__).resolve().parents[1]
    completed = subprocess.run(
        ["bash", "scripts/lab_teleop_teach_smoke.sh", "help"],
        cwd=root,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0
    assert "start" in completed.stdout
    assert "teach-on" in completed.stdout
    assert "teleop-on" in completed.stdout
    assert "xbox-jsonl" in completed.stdout
    assert "xbox-device" in completed.stdout
    assert "SDK/CAN singleton" in completed.stdout

