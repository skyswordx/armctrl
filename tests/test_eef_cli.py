import json
import subprocess
import sys
from pathlib import Path


def test_cli_eef_doctor_reports_mature_backend_availability_without_motion() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "eef", "doctor", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_doctor.v1"
    assert payload["movement_allowed"] is False
    assert payload["read_only"] is True
    assert payload["backends"]["moveit_servo"]["status"] in {
        "available",
        "missing",
        "installed_not_sourced",
    }
    assert payload["backends"]["pink"]["status"] in {"available", "missing"}
    assert payload["backends"]["pink"]["install_hint"] == "uv sync --extra dev --extra eef"
    assert payload["backends"]["sdk_cartesian"]["status"] in {"available", "missing"}
    assert payload["next_gate"] == "run_eef_plan_before_any_future_execution"


def test_cli_eef_runtime_plan_for_sdk_cartesian_reports_runtime_contract() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "runtime-plan",
            "--backend",
            "sdk_cartesian",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_runtime_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["backend"] == "sdk_cartesian"
    assert payload["runtime"]["owner"] == "arx5_interface_cartesian_controller"
    assert payload["runtime"]["api_contract"]["controller_class"] == "Arx5CartesianController"
    assert payload["runtime"]["api_contract"]["pose_surface"] == "EEFState.pose_6d"
    assert payload["runtime"]["api_contract"]["delta_surface"] == "EEFState.pose_6d_delta"
    assert "cartesian_waypoint_scheduling.py" in payload["runtime"]["api_contract"]["reference_examples"][1]
    assert any("keyboard_teleop.py" in " ".join(command) for command in payload["reference_commands"])


def test_cli_eef_runtime_plan_for_lerobot_rollout_reports_native_command() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "runtime-plan",
            "--backend",
            "lerobot_rollout",
            "--model",
            "X5",
            "--interface",
            "can0",
            "--policy-path",
            "outputs/train/policy",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["backend"] == "lerobot_rollout"
    assert payload["runtime"]["owner"] == "native_lerobot_rollout"
    assert payload["runtime"]["api_contract"]["default_strategy"] == "base"
    assert payload["runtime"]["api_contract"]["processor_hooks"] == [
        "robot_action_processor",
        "robot_observation_processor",
    ]
    assert payload["reference_commands"][0][0] == "lerobot-rollout"
    assert payload["reference_commands"][0][-1] == "--policy.path=outputs/train/policy"


def test_cli_eef_runtime_plan_for_lerobot_plan_dir_includes_bridge_preview(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "runtime-plan",
            "--backend",
            "lerobot_rollout",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--policy-path",
            "outputs/train/policy",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["backend"] == "lerobot_rollout"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "native_lerobot_rollout"
    assert payload["bridge_artifact_preview"]["schema"] == "armctrl.eef_lerobot_action_export.v1"
    assert payload["bridge_artifact_preview"]["lerobot_action"]["control_mode"] == "cartesian_pose_absolute"
    assert any("export-lerobot-action" in step for step in payload["next_steps"])
    assert any("export-processor-contract" in step for step in payload["next_steps"])
    assert any("stage-trajectory" in step for step in payload["next_steps"])


def test_cli_eef_runtime_plan_infers_backend_from_plan_dir(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "runtime-plan",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["backend"] == "sdk_cartesian"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "arx5_interface_cartesian_controller"
    assert payload["agent_runtime_profile"]["preferred_next_surface"] == "armctrl.eef.stage_trajectory"
    assert payload["backend_request"]["schema"] == "armctrl.eef_backend_request.v1"
    assert payload["backend_review_contract"]["schema"] == "armctrl.eef_backend_review_contract.v1"
    assert payload["artifact_contract"]["review_artifact"] == "backend_joint_trajectory.csv"
    assert payload["bridge_artifact_preview"]["schema"] == "armctrl.eef_sdk_cartesian_export.v1"
    assert payload["bridge_artifact_preview"]["sdk_controller"]["controller_class"] == "Arx5CartesianController"
    assert any("stage-trajectory" in step for step in payload["next_steps"])
    assert any("eef review" in step for step in payload["next_steps"])
    assert payload["reference_commands"][0][-1] == "can0"


def test_cli_eef_runtime_plan_for_moveit_plan_dir_includes_bridge_preview(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-twist-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "runtime-plan",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["backend"] == "moveit_servo"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "ros2_moveit_servo"
    assert payload["bridge_artifact_preview"]["schema"] == "armctrl.eef_moveit_servo_export.v1"
    assert payload["bridge_artifact_preview"]["ros_contract"]["message_type"] == "geometry_msgs/msg/TwistStamped"
    assert payload["runtime"]["api_contract"]["input_topics"]["twist"] == "~/delta_twist_cmds"
    assert payload["runtime"]["api_contract"]["switch_command_type_service"] == "~/switch_command_type"
    assert payload["runtime"]["api_contract"]["status_topic"] == "~/status"
    assert "realtime_servo_tutorial.html" in payload["runtime"]["api_contract"]["reference_docs"][0]
    assert any("stage-trajectory" in step for step in payload["next_steps"])


def test_cli_eef_runtime_plan_for_moveit_pose_plan_dir_includes_pose_bridge_preview(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-pose-moveit-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "runtime-plan",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["backend"] == "moveit_servo"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "ros2_moveit_servo"
    assert payload["agent_runtime_profile"]["preferred_next_surface"] == "armctrl.eef.stage_trajectory"
    assert payload["runtime"]["api_contract"]["command_surface"] == "PoseStamped"
    assert payload["runtime"]["api_contract"]["input_topics"]["pose"] == "~/pose_command_in_topic"
    assert "PoseCommand" in payload["runtime"]["api_contract"]["cxx_interface"]["command_types"]
    assert payload["bridge_artifact_preview"]["ros_contract"]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert payload["bridge_artifact_preview"]["ros_contract"]["command_topic_kind"] == "cartesian_servo_pose"


def test_cli_eef_export_lerobot_action_from_pose_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-pose"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-lerobot-action",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_lerobot_action_export.v1"
    assert payload["movement_allowed"] is False
    assert payload["backend"] == "sdk_cartesian"
    assert payload["lerobot_action"]["schema"] == "armctrl.eef_lerobot_action.v1"
    assert payload["lerobot_action"]["control_mode"] == "cartesian_pose_absolute"
    assert payload["lerobot_action"]["feature_order"] == [
        "eef.x",
        "eef.y",
        "eef.z",
        "eef.roll",
        "eef.pitch",
        "eef.yaw",
    ]
    assert payload["lerobot_action"]["action_vector"] == [0.4, 0.0, 0.2, 0.0, 0.0, 0.0]
    assert payload["lerobot_action"]["action_dict"]["eef.z"] == 0.2


def test_cli_eef_export_lerobot_action_writes_json_artifact(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-twist"
    output_path = tmp_path / "lerobot_action.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-lerobot-action",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["output"] == str(output_path)
    assert written["lerobot_action"]["control_mode"] == "cartesian_delta"
    assert written["lerobot_action"]["action_dict"]["eef.dz"] == -0.002
    assert written["lerobot_action"]["safety_contract"]["review_status"] == "plan_only"


def test_cli_eef_export_sdk_cartesian_from_pose_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-sdk"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-sdk-cartesian",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_sdk_cartesian_export.v1"
    assert payload["movement_allowed"] is False
    assert payload["sdk_controller"]["controller_class"] == "Arx5CartesianController"
    assert payload["sdk_controller"]["command_mode"] == "pose_6d_absolute"
    assert payload["sdk_request"]["robot"] == {"model": "X5", "interface": "can0"}
    assert payload["sdk_request"]["eef_waypoints"][0]["pose_6d"] == [0.4, 0.0, 0.2, 0.0, 0.0, 0.0]
    assert any("keyboard_teleop.py" in item for item in payload["reference_examples"])


def test_cli_eef_export_sdk_cartesian_from_twist_plan_writes_artifact(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan-sdk-twist"
    output_path = tmp_path / "sdk_cartesian_request.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-sdk-cartesian",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["output"] == str(output_path)
    assert written["sdk_controller"]["command_mode"] == "pose_6d_delta"
    assert written["sdk_request"]["eef_waypoints"][0]["pose_6d_delta"] == [0.0, 0.0, -0.002, 0.0, 0.0, 0.0]


def test_cli_eef_export_moveit_servo_from_twist_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-moveit"
    output_path = tmp_path / "moveit_servo_request.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-moveit-servo",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_moveit_servo_export.v1"
    assert payload["movement_allowed"] is False
    assert payload["ros_contract"]["message_type"] == "geometry_msgs/msg/TwistStamped"
    assert payload["ros_contract"]["twist"]["linear"] == [0.0, 0.0, -0.02]
    assert written["backend"] == "moveit_servo"
    assert written["reference_docs"][0] == "https://docs.ros.org/en/jazzy/p/moveit_servo/"


def test_cli_eef_export_moveit_servo_from_pose_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-moveit-pose"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-moveit-servo",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["backend"] == "moveit_servo"
    assert payload["ros_contract"]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert payload["ros_contract"]["command_topic_kind"] == "cartesian_servo_pose"
    assert payload["ros_contract"]["frame_id"] == "eef_link"
    assert payload["ros_contract"]["pose_6d"] == [0.4, 0.0, 0.2, 0.0, 0.0, 0.0]


def test_cli_eef_export_runtime_bridge_infers_sdk_backend_from_plan_dir(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runtime-bridge",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_runtime_bridge_export.v1"
    assert payload["resolved_backend"] == "sdk_cartesian"
    assert payload["delegated_export"]["schema"] == "armctrl.eef_sdk_cartesian_export.v1"
    assert payload["delegated_export"]["sdk_controller"]["controller_class"] == "Arx5CartesianController"


def test_cli_eef_export_runtime_bridge_can_override_to_lerobot_rollout(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-generic-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runtime-bridge",
            "--plan-dir",
            str(plan_dir),
            "--backend",
            "lerobot_rollout",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["delegated_export"]["schema"] == "armctrl.eef_lerobot_action_export.v1"
    assert payload["delegated_export"]["lerobot_action"]["control_mode"] == "cartesian_pose_absolute"


def test_cli_eef_export_runner_contract_for_sdk_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-runner-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--model",
            "X5",
            "--interface",
            "can0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_runner_contract.v1"
    assert payload["resolved_backend"] == "sdk_cartesian"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "arx5_interface_cartesian_controller"
    assert payload["bridge_export"]["schema"] == "armctrl.eef_sdk_cartesian_export.v1"
    assert payload["runner_api"]["owner"] == "arx5_interface_cartesian_controller"
    assert payload["review_output_contract"]["expected_trajectory_path"].endswith(
        "backend_joint_trajectory.csv"
    )
    assert "q_cmd_6" in payload["review_output_contract"]["required_columns"]


def test_cli_eef_export_runner_contract_writes_json_artifact(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-sdk-runner-plan-file"
    output_path = tmp_path / "eef_runner_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["output"] == str(output_path)
    assert written["schema"] == "armctrl.eef_runner_contract.v1"
    assert written["resolved_backend"] == "sdk_cartesian"


def test_cli_eef_export_runner_contract_for_moveit_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-moveit-runner-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["resolved_backend"] == "moveit_servo"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "ros2_moveit_servo"
    assert payload["bridge_export"]["schema"] == "armctrl.eef_moveit_servo_export.v1"
    assert payload["runner_api"]["owner"] == "ros2_moveit_servo"
    assert payload["runner_api"]["message_type"] == "geometry_msgs/msg/TwistStamped"
    assert any("eef doctor" in step for step in payload["next_steps"])
    assert any("sim doctor" in step for step in payload["next_steps"])


def test_cli_eef_export_runner_contract_for_moveit_pose_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-moveit-pose-runner-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["resolved_backend"] == "moveit_servo"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "ros2_moveit_servo"
    assert payload["runner_api"]["owner"] == "ros2_moveit_servo"
    assert payload["runner_api"]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert payload["bridge_export"]["ros_contract"]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert any("synthesize-preview" in step for step in payload["next_steps"])


def test_cli_eef_export_runner_contract_for_lerobot_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-runner-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--backend",
            "lerobot_rollout",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["agent_runtime_profile"]["schema"] == "armctrl.agent_runtime_profile.v1"
    assert payload["agent_runtime_profile"]["profile"] == "eef_runtime_handoff"
    assert payload["agent_runtime_profile"]["runtime_owner"] == "native_lerobot_rollout"
    assert payload["bridge_export"]["schema"] == "armctrl.eef_lerobot_action_export.v1"
    assert payload["runner_api"]["owner"] == "native_lerobot_rollout"
    assert payload["runner_api"]["action_hook"] == "robot_action_processor"
    assert payload["runner_api"]["observation_hook"] == "robot_observation_processor"


def test_cli_eef_preview_runner_closes_nonhardware_loop_for_sdk_backend(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-runner-preview-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "preview-runner",
            "--plan-dir",
            str(plan_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_runner_preview.v1"
    assert payload["movement_allowed"] is False
    assert payload["runner_contract"]["schema"] == "armctrl.eef_runner_contract.v1"
    assert payload["runner_contract"]["resolved_backend"] == "sdk_cartesian"
    assert payload["delegated_preview"]["schema"] == "armctrl.eef_preview_synthesis.v1"
    assert payload["review_status"] == "completed"
    assert payload["sim_preview"]["safety"]["allowed"] is True
    assert payload["synthesized_trajectory"].endswith("backend_joint_trajectory.csv")
    assert any("eef review" in step for step in payload["next_steps"])


def test_cli_eef_preview_runner_rejects_missing_plan_artifacts(tmp_path: Path) -> None:
    missing_dir = tmp_path / "missing-eef-runner-plan"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "preview-runner",
            "--plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.eef_runner_preview.v1"
    assert payload["error"]["code"] == "missing_eef_plan_artifacts"


def test_cli_eef_sample_runner_consumes_runner_contract_file(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-runner-sample-plan"
    contract_path = tmp_path / "eef_runner_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "sample-runner",
            "--runner-contract",
            str(contract_path),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_sample_runner.v1"
    assert payload["movement_allowed"] is False
    assert payload["runner_contract_path"] == str(contract_path)
    assert payload["consumed_runner_contract"]["schema"] == "armctrl.eef_runner_contract.v1"
    assert payload["resolved_backend"] == "sdk_cartesian"
    assert payload["review_status"] == "completed"
    assert payload["sim_preview"]["safety"]["allowed"] is True
    assert payload["synthesized_trajectory"].endswith("backend_joint_trajectory.csv")


def test_sdk_cartesian_helper_sample_consumes_runner_contract_artifact(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-helper-plan"
    contract_path = tmp_path / "eef_runner_contract.json"
    output_path = tmp_path / "sdk_helper_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/sdk_cartesian_contract_helper_sample.py",
            "--runner-contract",
            str(contract_path),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["output"] == str(output_path)
    assert payload["sdk_session_plan"]["command_mode"] == "pose_6d_absolute"
    assert payload["sdk_session_plan"]["submit_call"] == "controller.set_eef_cmd(eef_cmd)"
    assert payload["sdk_session_plan"]["eef_waypoints"][0]["pose_6d"] == [0.4, 0.0, 0.2, 0.0, 0.0, 0.0]
    assert payload["review_output_contract"]["expected_trajectory_path"].endswith(
        "backend_joint_trajectory.csv"
    )
    assert any("sample-runner" in step for step in payload["next_steps"])
    assert written["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"


def test_cli_eef_export_sdk_helper_plan_from_runner_contract(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-sdk-helper-cli-plan"
    contract_path = tmp_path / "eef_runner_contract.json"
    output_path = tmp_path / "sdk_helper_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-sdk-helper-plan",
            "--runner-contract",
            str(contract_path),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"
    assert payload["output"] == str(output_path)
    assert payload["sdk_session_plan"]["command_mode"] == "pose_6d_absolute"
    assert payload["ordered_steps"][0]["id"] == "sample_runner"
    assert payload["ordered_steps"][1]["depends_on"] == ["sample_runner"]
    assert written["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"


def test_sdk_cartesian_helper_sample_rejects_non_sdk_runner_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-moveit-helper-plan"
    contract_path = tmp_path / "eef_runner_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/sdk_cartesian_contract_helper_sample.py",
            "--runner-contract",
            str(contract_path),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "unsupported_runner_backend"
    assert payload["resolved_backend"] == "moveit_servo"


def test_cli_eef_export_sdk_helper_plan_rejects_non_sdk_runner_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-moveit-helper-cli-plan"
    contract_path = tmp_path / "eef_runner_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-sdk-helper-plan",
            "--runner-contract",
            str(contract_path),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "unsupported_runner_backend"
    assert payload["resolved_backend"] == "moveit_servo"


def test_moveit_servo_helper_sample_consumes_runner_contract_artifact(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-moveit-helper-plan"
    contract_path = tmp_path / "eef_moveit_runner_contract.json"
    output_path = tmp_path / "moveit_helper_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/moveit_servo_contract_helper_sample.py",
            "--runner-contract",
            str(contract_path),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.moveit_servo_helper_plan.v1"
    assert payload["movement_allowed"] is False
    assert payload["output"] == str(output_path)
    assert payload["servo_session_plan"]["command_mode"] == "pose"
    assert payload["servo_session_plan"]["message_type"] == "geometry_msgs/msg/PoseStamped"
    assert payload["servo_session_plan"]["command_topic"] == "~/pose_command_in_topic"
    assert payload["servo_session_plan"]["command_payload"]["pose_6d"] == [
        0.4,
        0.0,
        0.2,
        0.0,
        0.0,
        0.0,
    ]
    assert payload["servo_session_plan"]["switch_command_type_service"] == "~/switch_command_type"
    assert any("sample-runner" in step for step in payload["next_steps"])
    assert written["schema"] == "armctrl.moveit_servo_helper_plan.v1"


def test_cli_eef_export_moveit_helper_plan_from_runner_contract(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-moveit-helper-cli-plan"
    contract_path = tmp_path / "eef_moveit_runner_contract.json"
    output_path = tmp_path / "moveit_helper_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-moveit-helper-plan",
            "--runner-contract",
            str(contract_path),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.moveit_servo_helper_plan.v1"
    assert payload["output"] == str(output_path)
    assert payload["servo_session_plan"]["command_mode"] == "pose"
    assert payload["ordered_steps"][0]["id"] == "sample_runner"
    assert payload["ordered_steps"][1]["depends_on"] == ["sample_runner"]
    assert written["schema"] == "armctrl.moveit_servo_helper_plan.v1"


def test_moveit_servo_helper_sample_rejects_non_moveit_runner_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-reject-moveit-helper-plan"
    contract_path = tmp_path / "eef_sdk_runner_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/moveit_servo_contract_helper_sample.py",
            "--runner-contract",
            str(contract_path),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "unsupported_runner_backend"
    assert payload["resolved_backend"] == "sdk_cartesian"


def test_cli_eef_export_moveit_helper_plan_rejects_non_moveit_runner_contract(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-reject-moveit-helper-cli-plan"
    contract_path = tmp_path / "eef_sdk_runner_contract.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-runner-contract",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(contract_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-moveit-helper-plan",
            "--runner-contract",
            str(contract_path),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "unsupported_runner_backend"
    assert payload["resolved_backend"] == "sdk_cartesian"


def test_cli_eef_synthesize_preview_rejects_missing_plan_artifacts(
    tmp_path: Path,
) -> None:
    missing_dir = tmp_path / "missing-eef-plan"
    missing_dir.mkdir()

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "synthesize-preview",
            "--plan-dir",
            str(missing_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["schema"] == "armctrl.eef_preview_synthesis.v1"
    assert payload["error"]["code"] == "missing_eef_plan_artifacts"


def test_cli_eef_synthesize_preview_writes_joint_trajectory_and_reuses_review_chain(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-preview-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "synthesize-preview",
            "--plan-dir",
            str(plan_dir),
            "--backend",
            "pink",
            "--sample-hz",
            "20",
            "--duration",
            "1.0",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_preview_synthesis.v1"
    assert payload["backend"] == "pink"
    assert payload["review"]["schema"] == "armctrl.eef_review.v1"
    assert payload["review"]["review_status"] == "completed"
    assert payload["review"]["sim_preview"]["schema"] == "armctrl.trajectory_preview.v1"
    trajectory_path = Path(payload["synthesized_trajectory"])
    assert trajectory_path.exists()
    header = trajectory_path.read_text(encoding="utf-8").splitlines()[0]
    assert header == "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6"


def test_cli_eef_synthesize_preview_can_take_recipe_seed_start_state(
    tmp_path: Path,
) -> None:
    recipe_plan_dir = tmp_path / "recipe-plan"
    eef_plan_dir = tmp_path / "eef-preview-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "recipe",
            "plan",
            "home",
            "--output",
            str(recipe_plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(eef_plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "synthesize-preview",
            "--plan-dir",
            str(eef_plan_dir),
            "--recipe-plan-dir",
            str(recipe_plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["start_joints_source"] == "recipe_eef_seed"
    assert payload["start_joints"] == [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    assert payload["review"]["review_status"] == "completed"


def test_cli_eef_synthesize_preview_rejects_missing_recipe_seed_artifacts(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-preview-plan"
    missing_recipe_plan_dir = tmp_path / "missing-recipe-plan"
    missing_recipe_plan_dir.mkdir()
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "synthesize-preview",
            "--plan-dir",
            str(plan_dir),
            "--recipe-plan-dir",
            str(missing_recipe_plan_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 3
    payload = json.loads(completed.stdout)
    assert payload["status"] == "rejected"
    assert payload["error"]["code"] == "missing_recipe_plan_artifacts"
    assert payload["recipe_plan_dir"] == str(missing_recipe_plan_dir)


def test_cli_eef_plan_twist_is_plan_only_and_routes_to_mature_backend_contract() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_plan.v1"
    assert payload["plan_only"] is True
    assert payload["movement_allowed"] is False
    assert payload["command_type"] == "twist"
    assert payload["backend"]["requested"] == "moveit_servo"
    assert payload["boundary"] == "mature_eef_backend_adapter"
    assert payload["safety_contract"]["workspace_gate_required"] is True
    assert payload["safety_contract"]["backend_execution_deferred"] is True
    assert payload["command"]["frame"] == "eef_link"
    assert payload["command"]["linear_mps"] == [0.0, 0.0, -0.02]


def test_cli_eef_plan_pose_supports_pink_backend_contract() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "pink",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_plan.v1"
    assert payload["command_type"] == "pose"
    assert payload["backend"]["requested"] == "pink"
    assert payload["command"]["position_m"] == [0.4, 0.0, 0.2]
    assert payload["command"]["rpy_rad"] == [0.0, 0.0, 0.0]
    assert payload["lerobot_bridge"]["schema"] == "armctrl.eef_lerobot_bridge.v1"
    assert payload["lerobot_bridge"]["control_mode"] == "cartesian_pose_absolute"
    assert any("MoveIt Servo" in note for note in payload["notes"])
    assert any("Pink" in note for note in payload["notes"])


def test_cli_eef_plan_pose_supports_sdk_cartesian_backend_contract() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_plan.v1"
    assert payload["command_type"] == "pose"
    assert payload["backend"]["requested"] == "sdk_cartesian"
    assert payload["backend_handoff"]["selected"] == "sdk_cartesian_pose_command"
    assert payload["backend_handoff"]["command_surface"] == "EEFState.pose_6d"
    assert payload["backend_handoff"]["reference_runtime"] == "arx5_interface_cartesian_controller"
    assert payload["lerobot_bridge"]["preferred_native_surface"] == "lerobot-rollout"
    assert payload["lerobot_bridge"]["value_mapping"]["eef.z"] == 0.2


def test_cli_eef_plan_pose_supports_moveit_servo_backend_contract() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_plan.v1"
    assert payload["command_type"] == "pose"
    assert payload["backend"]["requested"] == "moveit_servo"
    assert payload["backend_handoff"]["selected"] == "moveit_servo_pose_command"
    assert payload["backend_handoff"]["command_surface"] == "geometry_msgs/msg/PoseStamped"
    assert payload["backend_handoff"]["reference_runtime"] == "ros2_moveit_servo"


def test_cli_eef_plan_pose_emits_agent_action_vocabulary() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["agent_action"]["schema"] == "armctrl.agent_eef_action.v1"
    assert payload["agent_action"]["action_id"] == "eef.pose_absolute"
    assert payload["agent_action"]["frame"] == "eef_link"
    assert payload["agent_action"]["value"] == {
        "position_m": [0.4, 0.0, 0.2],
        "rpy_rad": [0.0, 0.0, 0.0],
    }
    assert payload["agent_action"]["backend_mapping"]["requested_backend"] == "sdk_cartesian"


def test_cli_eef_plan_twist_emits_agent_action_vocabulary() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["agent_action"]["schema"] == "armctrl.agent_eef_action.v1"
    assert payload["agent_action"]["action_id"] == "eef.twist"
    assert payload["agent_action"]["frame"] == "eef_link"
    assert payload["agent_action"]["value"] == {
        "linear_mps": [0.0, 0.0, -0.02],
        "angular_rps": [0.0, 0.0, 0.0],
        "control_period_s": 0.1,
    }
    assert payload["agent_action"]["backend_mapping"]["requested_backend"] == "moveit_servo"


def test_cli_eef_plan_delta_pose_emits_agent_action_vocabulary() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.01",
            "0.00",
            "-0.02",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.1",
            "--backend",
            "sdk_cartesian",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_plan.v1"
    assert payload["command_type"] == "pose_delta"
    assert payload["agent_action"]["schema"] == "armctrl.agent_eef_action.v1"
    assert payload["agent_action"]["action_id"] == "eef.pose_delta"
    assert payload["agent_action"]["frame"] == "eef_link"
    assert payload["agent_action"]["value"] == {
        "delta_position_m": [0.01, 0.0, -0.02],
        "delta_rpy_rad": [0.0, 0.0, 0.1],
        "control_period_s": 0.1,
    }
    assert payload["backend_handoff"]["selected"] == "sdk_cartesian_pose_delta_command"


def test_cli_eef_export_sdk_cartesian_from_delta_pose_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-sdk-delta-pose"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.01",
            "0.00",
            "-0.02",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.1",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert json.loads(completed.stdout)["status"] == "ok"

    exported = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-sdk-cartesian",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(exported.stdout)
    assert payload["status"] == "ok"
    assert payload["sdk_controller"]["command_mode"] == "pose_6d_delta"
    assert payload["sdk_request"]["eef_waypoints"][0]["pose_6d_delta"] == [
        0.01,
        0.0,
        -0.02,
        0.0,
        0.0,
        0.1,
    ]


def test_cli_eef_synthesize_preview_supports_delta_pose_plan(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-delta-pose-preview"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.01",
            "0.00",
            "-0.02",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.1",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "synthesize-preview",
            "--plan-dir",
            str(plan_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_preview_synthesis.v1"
    assert payload["review"]["review_status"] == "completed"


def test_cli_eef_export_agent_runtime_contract_for_moveit_delta_pose_plan(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-moveit-agent-runtime-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--backend",
            "moveit_servo",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_agent_runtime_contract.v1"
    assert payload["resolved_backend"] == "moveit_servo"
    assert payload["runtime_owner"] == "ros2_moveit_servo"
    assert payload["agent_action_schema"]["action_id"] == "eef.pose_delta"
    assert payload["agent_action_schema"]["stream_mode"] == "realtime_frame_sequence"
    assert payload["backend_session_contract"]["message_type"] == "geometry_msgs/msg/TwistStamped"
    assert payload["backend_session_contract"]["command_topic"] == "~/delta_twist_cmds"
    assert payload["ordered_steps"][0]["id"] == "stage_trajectory"
    assert payload["ordered_steps"][1]["depends_on"] == ["stage_trajectory"]
    assert any("stage-trajectory" in step for step in payload["next_steps"])


def test_cli_eef_export_agent_runtime_contract_for_sdk_pose_plan(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-agent-runtime-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-runtime-contract",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["resolved_backend"] == "sdk_cartesian"
    assert payload["runtime_owner"] == "arx5_interface_cartesian_controller"
    assert payload["agent_action_schema"]["action_id"] == "eef.pose_absolute"
    assert payload["backend_session_contract"]["command_object"] == "EEFState"
    assert payload["backend_session_contract"]["command_mode"] == "pose_6d_absolute"
    assert payload["ordered_steps"][0]["id"] == "stage_trajectory"
    assert payload["ordered_steps"][1]["depends_on"] == ["stage_trajectory"]
    assert payload["review_output_contract"]["expected_trajectory_path"].endswith(
        "backend_joint_trajectory.csv"
    )


def test_cli_eef_export_agent_session_plan_for_sdk_pose_plan(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-sdk-agent-session-plan"
    output_path = tmp_path / "eef_sdk_agent_session_plan.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-session-plan",
            "--plan-dir",
            str(plan_dir),
            "--output",
            str(output_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    written = json.loads(output_path.read_text(encoding="utf-8"))

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_agent_session_plan.v1"
    assert payload["resolved_backend"] == "sdk_cartesian"
    assert payload["runtime_owner"] == "arx5_interface_cartesian_controller"
    assert payload["recommended_path"]["schema"] == "armctrl.recommended_path.v1"
    assert payload["recommended_path"]["profile"] == "realtime_eef_loop_with_shared_review"
    assert payload["recommended_path"]["primary_entrypoint"] == "armctrl eef export-agent-session-plan"
    assert payload["recommended_path"]["steps"][1]["id"] == "export_backend_helper_plan"
    assert "export-sdk-helper-plan" in payload["recommended_path"]["steps"][1]["command"]
    assert payload["agent_runtime_contract"]["agent_action_schema"]["action_id"] == "eef.pose_absolute"
    assert payload["helper_plan"]["schema"] == "armctrl.sdk_cartesian_helper_plan.v1"
    assert payload["ordered_steps"][0]["id"] == "export_agent_runtime_contract"
    assert payload["ordered_steps"][1]["id"] == "export_backend_helper_plan"
    assert payload["ordered_steps"][2]["id"] == "review_trajectory"
    assert payload["ordered_steps"][2]["depends_on"] == ["export_backend_helper_plan"]
    assert any("export-sdk-helper-plan" in step for step in payload["next_steps"])
    assert written["schema"] == "armctrl.eef_agent_session_plan.v1"


def test_cli_eef_export_agent_session_plan_for_lerobot_delta_plan(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-lerobot-agent-session-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-delta-pose",
            "--frame",
            "eef_link",
            "--delta-position",
            "0.002",
            "0.000",
            "-0.003",
            "--delta-rpy",
            "0.0",
            "0.0",
            "0.02",
            "--backend",
            "lerobot_rollout",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "export-agent-session-plan",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_agent_session_plan.v1"
    assert payload["resolved_backend"] == "lerobot_rollout"
    assert payload["runtime_owner"] == "native_lerobot_rollout"
    assert payload["recommended_path"]["profile"] == "realtime_eef_loop_with_shared_review"
    assert payload["recommended_path"]["steps"][1]["id"] == "export_processor_contract"
    assert "export-processor-contract" in payload["recommended_path"]["steps"][1]["command"]
    assert payload["agent_runtime_contract"]["agent_action_schema"]["action_id"] == "eef.pose_delta"
    assert payload["helper_plan"] is None
    assert payload["ordered_steps"][0]["id"] == "export_agent_runtime_contract"
    assert payload["ordered_steps"][1]["id"] == "export_processor_contract"
    assert payload["ordered_steps"][2]["id"] == "review_trajectory"
    assert payload["ordered_steps"][2]["depends_on"] == ["export_processor_contract"]
    assert any("export-processor-contract" in step for step in payload["next_steps"])


def test_cli_eef_plan_pose_writes_artifacts_and_workspace_gate(tmp_path: Path) -> None:
    output_dir = tmp_path / "eef-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "pink",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["gate"]["allowed"] is True
    assert payload["gate"]["position_box_check"]["status"] == "pass"
    assert payload["backend_handoff"]["requested"] == "pink"
    assert payload["backend_handoff"]["implementation_boundary"] == (
        "armctrl orchestrates; mature EEF backends own IK/servo math"
    )
    assert payload["safety_contract"]["max_translation_step_m"] == 0.005
    assert payload["safety_contract"]["max_rotation_step_rad"] == 0.05
    assert payload["artifacts"]["manifest"] == str(output_dir / "manifest.json")
    assert payload["artifacts"]["eef_plan"] == str(output_dir / "eef_plan.json")
    assert payload["artifacts"]["backend_request"] == str(output_dir / "backend_request.json")
    assert payload["artifacts"]["backend_review_contract"] == str(
        output_dir / "backend_review_contract.json"
    )
    assert (output_dir / "manifest.json").exists()
    assert (output_dir / "eef_plan.json").exists()
    assert (output_dir / "backend_request.json").exists()
    assert (output_dir / "backend_review_contract.json").exists()


def test_cli_eef_plan_pose_flags_forbidden_workspace_target() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.10",
            "0.00",
            "0.00",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "pink",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["gate"]["allowed"] is False
    assert payload["gate"]["position_box_check"]["status"] == "fail"
    assert payload["gate"]["position_box_check"]["violations"][0]["check"] in {
        "outside_allowed_workspace",
        "inside_forbidden_workspace",
    }


def test_cli_eef_plan_twist_writes_backend_request_for_moveit_servo(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "eef-twist"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "moveit_servo",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    backend_request = json.loads(
        (output_dir / "backend_request.json").read_text(encoding="utf-8")
    )

    assert payload["backend_handoff"]["requested"] == "moveit_servo"
    assert payload["backend_handoff"]["selected"] == "moveit_servo_twist_command"
    assert payload["backend_handoff"]["oob_execution"] is False
    assert backend_request["schema"] == "armctrl.eef_backend_request.v1"
    assert backend_request["backend"]["requested"] == "moveit_servo"
    assert backend_request["backend"]["command_surface"] == "TwistStamped"
    assert backend_request["lerobot_bridge"]["control_mode"] == "cartesian_delta"


def test_cli_eef_plan_twist_writes_backend_request_for_sdk_cartesian(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "eef-twist-sdk"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-twist",
            "--frame",
            "eef_link",
            "--linear",
            "0.0",
            "0.0",
            "-0.02",
            "--angular",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    backend_request = json.loads(
        (output_dir / "backend_request.json").read_text(encoding="utf-8")
    )

    assert payload["backend_handoff"]["requested"] == "sdk_cartesian"
    assert payload["backend_handoff"]["selected"] == "sdk_cartesian_twist_delta"
    assert backend_request["backend"]["requested"] == "sdk_cartesian"
    assert backend_request["backend"]["command_surface"] == "EEFState.pose_6d_delta"
    assert backend_request["lerobot_bridge"]["value_mapping"]["eef.dz"] == -0.002


def test_cli_eef_review_reuses_sim_preview_chain_for_backend_trajectory(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan"
    trajectory_path = tmp_path / "backend_joint_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.020000,0.320000,0.320000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "pink",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "review",
            "--plan-dir",
            str(plan_dir),
            "--trajectory",
            str(trajectory_path),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_review.v1"
    assert payload["movement_allowed"] is False
    assert payload["sim_preview"]["schema"] == "armctrl.trajectory_preview.v1"
    assert payload["sim_preview"]["safety"]["allowed"] is True
    assert payload["backend_request"]["schema"] == "armctrl.eef_backend_request.v1"
    assert payload["reviewed_trajectory"] == str(trajectory_path)


def test_cli_eef_review_reports_missing_backend_trajectory_as_deferred(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "pink",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "review",
            "--plan-dir",
            str(plan_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.eef_review.v1"
    assert payload["review_status"] == "deferred"
    assert payload["sim_preview"] is None
    assert payload["expected_backend_trajectory"] == str(
        plan_dir / "backend_joint_trajectory.csv"
    )
    assert "backend joint trajectory" in payload["next_gate"]


def test_cli_eef_review_auto_discovers_default_backend_trajectory(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    default_trajectory = plan_dir / "backend_joint_trajectory.csv"
    default_trajectory.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.020000,0.320000,0.320000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "review",
            "--plan-dir",
            str(plan_dir),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["review_status"] == "completed"
    assert payload["reviewed_trajectory"] == str(default_trajectory)
    assert payload["sim_preview"]["safety"]["allowed"] is True


def test_cli_eef_stage_trajectory_installs_backend_review_artifact(
    tmp_path: Path,
) -> None:
    plan_dir = tmp_path / "eef-plan-stage"
    source_trajectory = tmp_path / "sdk_backend_joint_trajectory.csv"
    source_trajectory.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "0.100000,0.010000,0.310000,0.310000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    staged = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "stage-trajectory",
            "--plan-dir",
            str(plan_dir),
            "--trajectory",
            str(source_trajectory),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    staged_payload = json.loads(staged.stdout)
    expected_path = plan_dir / "backend_joint_trajectory.csv"

    assert staged_payload["status"] == "ok"
    assert staged_payload["schema"] == "armctrl.eef_stage_trajectory.v1"
    assert staged_payload["movement_allowed"] is False
    assert staged_payload["plan_dir"] == str(plan_dir)
    assert staged_payload["source_trajectory"] == str(source_trajectory)
    assert staged_payload["staged_trajectory"] == str(expected_path)
    assert staged_payload["trajectory_validation"]["status"] == "pass"
    assert staged_payload["trajectory_validation"]["sample_count"] == 2
    assert expected_path.read_text(encoding="utf-8") == source_trajectory.read_text(
        encoding="utf-8"
    )

    review = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "review",
            "--plan-dir",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    review_payload = json.loads(review.stdout)

    assert review_payload["status"] == "ok"
    assert review_payload["reviewed_trajectory"] == str(expected_path)


def test_cli_eef_stage_trajectory_accepts_utf8_bom_csv(tmp_path: Path) -> None:
    plan_dir = tmp_path / "eef-plan-stage-bom"
    source_trajectory = tmp_path / "sdk_backend_joint_trajectory_bom.csv"
    source_trajectory.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8-sig",
    )
    subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "plan-pose",
            "--frame",
            "eef_link",
            "--position",
            "0.40",
            "0.00",
            "0.20",
            "--rpy",
            "0.0",
            "0.0",
            "0.0",
            "--backend",
            "sdk_cartesian",
            "--output",
            str(plan_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    staged = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "eef",
            "stage-trajectory",
            "--plan-dir",
            str(plan_dir),
            "--trajectory",
            str(source_trajectory),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(staged.stdout)

    assert payload["status"] == "ok"
    assert payload["trajectory_validation"]["status"] == "pass"
    assert payload["trajectory_validation"]["sample_count"] == 1
