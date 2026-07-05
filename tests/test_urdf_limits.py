from pathlib import Path

from armctrl.limits import UrdfJointLimits, evaluate_joint_limits


def test_loads_x5_revolute_joint_limits_from_urdf() -> None:
    limits = UrdfJointLimits.from_urdf(Path("configs/models/X5_camera.urdf"))

    assert len(limits.joints) == 6
    assert limits.joints[0].name
    assert limits.joints[1].lower < 0.3 < limits.joints[1].upper


def test_evaluates_trajectory_against_urdf_limits() -> None:
    limits = UrdfJointLimits.from_urdf(Path("configs/models/X5_camera.urdf"))

    decision = evaluate_joint_limits(
        limits,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        amplitude_rad=0.1,
    )

    assert decision.status == "pass"
    assert decision.violations == []


def test_reports_limit_violation_for_unsafe_center() -> None:
    limits = UrdfJointLimits.from_urdf(Path("configs/models/X5_camera.urdf"))

    decision = evaluate_joint_limits(
        limits,
        q_center=(0.0, 99.0, 0.3, 0.0, 0.0, 0.0),
        amplitude_rad=0.1,
    )

    assert decision.status == "fail"
    assert decision.violations[0]["joint_index"] == 2
