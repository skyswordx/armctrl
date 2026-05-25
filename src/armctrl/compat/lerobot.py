"""LeRobot-shaped contract metadata for armctrl datasets."""

from __future__ import annotations


def build_lerobot_contract(*, dof: int, gripper: bool = False) -> dict[str, object]:
    if dof <= 0:
        raise ValueError("dof must be positive")

    joint_names = [f"joint_{index}" for index in range(1, dof + 1)]
    observation_features: dict[str, dict[str, str]] = {}
    action_features: dict[str, dict[str, str]] = {}
    column_map: dict[str, dict[str, str]] = {"observation": {}, "action": {}}

    for index in range(1, dof + 1):
        joint = f"joint_{index}"
        observation_features[f"{joint}.pos"] = {"type": "scalar", "unit": "rad"}
        observation_features[f"{joint}.vel"] = {"type": "scalar", "unit": "rad/s"}
        observation_features[f"{joint}.effort"] = {"type": "scalar", "unit": "Nm"}
        action_features[f"{joint}.pos"] = {"type": "scalar", "unit": "rad"}
        column_map["observation"][f"q_{index}"] = f"{joint}.pos"
        column_map["observation"][f"dq_{index}"] = f"{joint}.vel"
        column_map["observation"][f"tau_meas_{index}"] = f"{joint}.effort"
        column_map["action"][f"q_cmd_{index}"] = f"{joint}.pos"
        column_map["action"][f"dq_cmd_{index}"] = f"{joint}.vel"
        column_map["action"][f"ddq_cmd_{index}"] = f"{joint}.effort"

    if gripper:
        observation_features["gripper.pos"] = {"type": "scalar", "unit": "m"}
        action_features["gripper.pos"] = {"type": "scalar", "unit": "m"}
        column_map["observation"]["gripper_pos"] = "gripper.pos"
        column_map["action"]["gripper_cmd"] = "gripper.pos"

    return {
        "schema": "lerobot-compatible",
        "joint_names": joint_names,
        "observation_features": observation_features,
        "action_features": action_features,
        "column_map": column_map,
    }
