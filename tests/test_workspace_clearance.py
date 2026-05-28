from pathlib import Path

from armctrl.workspace import (
    WorkspaceSafetyConfig,
    evaluate_workspace_clearance,
    evaluate_workspace_fk_clearance,
)


def test_loads_workspace_bounds_from_safe_yaml() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    assert config.workspace_min_m == (0.05, -0.45, 0.02)
    assert config.workspace_max_m == (0.75, 0.45, 0.65)


def test_workspace_clearance_passes_for_safe_center_proxy() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    decision = evaluate_workspace_clearance(
        config,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        amplitude_rad=0.1,
    )

    assert decision.status == "pass"
    assert decision.violations == []


def test_workspace_clearance_fails_for_downward_joint2_proxy() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    decision = evaluate_workspace_clearance(
        config,
        q_center=(0.0, 0.0, 0.3, 0.0, 0.0, 0.0),
        amplitude_rad=0.2,
    )

    assert decision.status == "fail"
    assert decision.violations[0]["check"] == "min_clearance_proxy"


def test_fk_clearance_checks_each_planned_sample_against_table_height() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    decision = evaluate_workspace_fk_clearance(
        Path("configs/models/X5_camera.urdf"),
        config,
        samples=[
            (0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            (0.0, 1.0, 1.0, 0.0, 0.0, 0.0),
        ],
    )

    assert decision.status == "pass"
    assert decision.method == "urdf_fk_frame_clearance"
    assert decision.violations == []


def test_fk_clearance_fails_when_a_link_frame_crosses_table_height() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    decision = evaluate_workspace_fk_clearance(
        Path("configs/models/X5_camera.urdf"),
        config,
        samples=[(0.0, 3.0, 1.0, 0.0, 0.0, 0.0)],
    )

    assert decision.status == "fail"
    assert decision.method == "urdf_fk_frame_clearance"
    assert decision.violations[0]["check"] == "fk_min_z"
    assert decision.violations[0]["minimum_z_m"] == config.workspace_min_m[2]
