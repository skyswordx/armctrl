import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

from armctrl.motion_runtime import MotionMode
from armctrl.sysid import SysIdPlanRequest, trajectory_rows
from armctrl.sysid_run import (
    Arx5InterfaceCollectionBackend,
    SDK_CONFIRMATION,
)
from armctrl.runtime_session import (
    record_runtime_hold_tick,
    refresh_runtime_status_payload,
    start_fake_runtime_session,
)

ROOT = Path(__file__).resolve().parents[1]
X5_URDF = ROOT / "configs" / "models" / "X5_camera.urdf"
X5_SAFE_CONFIG = ROOT / "configs" / "x5.safe.yaml"


def _passing_readiness_artifact() -> dict[str, object]:
    return {
        "schema": "armctrl.sysid_agent_smoke_readiness.v1",
        "read_only": True,
        "movement_allowed": False,
        "agent_sysid_smoke_allowed": True,
        "prerequisites": {
            "doctor": "pass",
            "hold_damping": "pass",
            "tiny_motion": "pass",
        },
        "tiny_motion": {
            "controller_dt_s": 0.002,
            "tracking": {
                "q_cmd_delta_max_abs_rad": 0.002,
                "q_meas_delta_max_abs_rad": 0.002,
                "final_tracking_error_max_abs_rad": 0.0,
            }
        },
    }


def _write_fake_runtime_session(path: Path) -> None:
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        fault_flags=(),
        max_heartbeat_age_s=5.0,
    )
    payload = refresh_runtime_status_payload(
        payload,
        max_heartbeat_age_s=5.0,
    )
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _write_live_runtime_status(path: Path) -> None:
    payload = start_fake_runtime_session(
        q_current=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        safe_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    payload = record_runtime_hold_tick(
        payload,
        q_meas=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        fault_flags=(),
        max_heartbeat_age_s=5.0,
    )
    payload = refresh_runtime_status_payload(payload, max_heartbeat_age_s=5.0)
    payload["readiness"] = {
        "agent_sysid_smoke_allowed": True,
        "live_hold_allowed": True,
        "safe_center_allowed": True,
        "failed_checks": [],
        "safe_center_failed_checks": [],
    }
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _install_fake_submit_trajectory_command(monkeypatch, cli, tmp_path: Path) -> None:
    submitted_commands: list[dict[str, object]] = []

    def fake_submit_trajectory_command(**kwargs) -> dict[str, object]:
        submitted_commands.append(kwargs)
        command_dir = tmp_path / "runtime_session_commands"
        pending_dir = command_dir / "pending"
        results_dir = command_dir / "results"
        pending_dir.mkdir(parents=True, exist_ok=True)
        results_dir.mkdir(parents=True, exist_ok=True)
        command_path = pending_dir / f"command-{len(submitted_commands)}.json"
        result_path = results_dir / f"command-{len(submitted_commands)}.json"
        command_payload = {
            "kind": "joint_trajectory",
            "source": str(kwargs.get("owner", "sysid")),
            "owner": str(kwargs.get("owner", "sysid")),
            "expected_q_start": list(kwargs["expected_q_start"]),
            "start_pose_policy": kwargs.get("start_pose_policy", "live_hold"),
            "q_points": [list(point) for point in kwargs.get("q_points", [])],
            "send_hz": kwargs.get("send_hz"),
            "trajectory_sample_hz": kwargs.get("trajectory_sample_hz"),
            "max_heartbeat_age_s": kwargs.get("max_heartbeat_age_s"),
            "start_pose_guard": {
                "policy": kwargs.get("start_pose_policy", "live_hold"),
                "q_hold": list(kwargs["expected_q_start"]),
            },
        }
        if kwargs.get("dq_points") is not None:
            command_payload["dq_points"] = [
                list(point) for point in kwargs["dq_points"]
            ]
        if kwargs.get("ddq_points") is not None:
            command_payload["ddq_points"] = [
                list(point) for point in kwargs["ddq_points"]
            ]
        if kwargs.get("artifact_policy") is not None:
            command_payload["artifact_policy"] = kwargs["artifact_policy"]
        if kwargs.get("max_tracking_error_rad") is not None:
            command_payload["max_tracking_error_rad"] = kwargs[
                "max_tracking_error_rad"
            ]
        if kwargs.get("max_tau_abs") is not None:
            command_payload["max_tau_abs"] = kwargs["max_tau_abs"]
        command_path.write_text(
            json.dumps(command_payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return {
            "command_id": f"command-{len(submitted_commands)}",
            "runtime": {"queue": str(command_dir)},
            "artifacts": {"command": str(command_path), "result": str(result_path)},
            "start_pose_guard": command_payload["start_pose_guard"],
        }

    monkeypatch.setattr(cli, "submit_trajectory_command", fake_submit_trajectory_command)


def _write_execution_trajectory(path: Path, *, include_ddq: bool = False) -> None:
    fieldnames = [
        "time_s",
        "q_cmd_1",
        "q_cmd_2",
        "q_cmd_3",
        "dq_cmd_1",
        "dq_cmd_2",
        "dq_cmd_3",
    ]
    if include_ddq:
        fieldnames.extend(["ddq_cmd_1", "ddq_cmd_2", "ddq_cmd_3"])
    rows = [
        {
            "time_s": "0.0",
            "q_cmd_1": "0.0",
            "q_cmd_2": "0.3",
            "q_cmd_3": "0.3",
            "dq_cmd_1": "0.0",
            "dq_cmd_2": "0.0",
            "dq_cmd_3": "0.0",
        },
        {
            "time_s": "0.01",
            "q_cmd_1": "0.002",
            "q_cmd_2": "0.3",
            "q_cmd_3": "0.3",
            "dq_cmd_1": "0.2",
            "dq_cmd_2": "0.0",
            "dq_cmd_3": "0.0",
        },
    ]
    if include_ddq:
        rows[0].update({"ddq_cmd_1": "0.0", "ddq_cmd_2": "0.0", "ddq_cmd_3": "0.0"})
        rows[1].update({"ddq_cmd_1": "1.0", "ddq_cmd_2": "0.0", "ddq_cmd_3": "0.0"})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class ManualClock:
    def __init__(self) -> None:
        self.now_s = 0.0

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, seconds: float) -> None:
        self.now_s += max(0.0, seconds)


def test_cli_sysid_compile_runtime_writes_compiled_motion_command(
    tmp_path: Path,
) -> None:
    execution_trajectory = tmp_path / "execution_trajectory.csv"
    output_dir = tmp_path / "compiled"
    _write_execution_trajectory(execution_trajectory)

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "compile-runtime",
            "--execution-trajectory",
            str(execution_trajectory),
            "--dof",
            "3",
            "--sample-hz",
            "100",
            "--expected-q-start",
            "0.0",
            "0.3",
            "0.3",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(completed.stdout)
    command = json.loads(
        Path(payload["artifacts"]["compiled_command"]).read_text(encoding="utf-8")
    )

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_runtime_compile.v1"
    assert payload["command_surface"] == "armctrl.motion.submit.v1"
    assert payload["motion_kind"] == "joint-trajectory"
    assert payload["movement_command_sent"] is False
    assert payload["hardware_motion"] is False
    assert payload["next_gate"] == "submit compiled command with armctrl motion submit joint-trajectory"
    assert command["schema"] == "armctrl.compiled_motion_command.v1"
    assert command["source"] == "sysid"
    assert command["owner"] == "sysid"
    assert command["q_points"] == [[0.0, 0.3, 0.3], [0.002, 0.3, 0.3]]
    assert command["dq_points"] == [[0.0, 0.0, 0.0], [0.2, 0.0, 0.0]]
    assert "ddq_points" not in command
    assert command["artifact_policy"] == {
        "schema": "armctrl.sysid_runtime_compiler_policy.v1",
        "trajectory_artifact": str(execution_trajectory),
        "q_cmd": "preserved",
        "dq_cmd": "preserved",
        "ddq_cmd": "missing",
        "sample_hz": 100.0,
    }


def test_cli_sysid_run_fake_writes_raw_samples_and_manifest(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    raw_samples_path = output_dir / "raw_samples.csv"
    manifest_path = output_dir / "manifest.json"

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.sysid_run.v1"
    assert payload["adapter"] == "fake"
    assert payload["artifacts"]["raw_samples"] == str(raw_samples_path)
    assert raw_samples_path.exists()
    assert manifest_path.exists()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "armctrl.sysid_run_manifest.v1"
    assert manifest["adapter"] == "fake"
    assert manifest["sample_count"] == 41
    assert manifest["request"]["urdf_path"] == str(X5_URDF)
    assert manifest["request"]["dof"] == 6
    assert manifest["handoff"]["dataset_contract"] == "lerobot-compatible"
    assert manifest["runtime_policy"] == {
        "formal_real_motion_entrypoint": (
            "armctrl sysid compile-runtime + "
            "armctrl motion submit joint-trajectory"
        ),
        "sysid_run_runtime_session_artifact": "unsupported",
        "reason": (
            "sysid run is offline/fake only and must not acquire a live "
            "runtime owner or mutate runtime_session.json"
        ),
    }

    with raw_samples_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))

    assert rows[0]["time_s"] == "0.000000"
    assert "tau_meas_6" in rows[0]
    assert len(rows) == 41


def test_cli_sysid_run_rejects_runtime_session_artifact(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    runtime_payload = start_fake_runtime_session(
        q_current=safe_center,
        safe_center=safe_center,
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    runtime_payload = record_runtime_hold_tick(
        runtime_payload,
        q_meas=safe_center,
        fault_flags=(),
        max_heartbeat_age_s=60.0,
    )
    runtime_session_artifact.write_text(
        json.dumps(
            runtime_payload,
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    before = runtime_session_artifact.read_text(encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            *[str(value) for value in safe_center],
            "--runtime-session-artifact",
            str(runtime_session_artifact),
            "--max-heartbeat-age-s",
            "60",
            "--output",
            str(output_dir),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert completed.stdout == ""
    assert "unrecognized arguments: --runtime-session-artifact" in completed.stderr
    assert runtime_session_artifact.read_text(encoding="utf-8") == before
    assert not (output_dir / "manifest.json").exists()


def test_cli_sysid_run_fake_ignores_runtime_owner_state(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-run"
    runtime_session_artifact = tmp_path / "runtime-session.json"
    safe_center = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]
    session = start_fake_runtime_session(
        q_current=safe_center,
        safe_center=safe_center,
        send_hz=50.0,
        hold_hz=50.0,
        max_joint_step_rad=0.01,
        max_heartbeat_age_s=5.0,
    )
    session["mode"] = "agent_servo"
    session["owner"] = "agent"
    session["owner_lease"] = {
        "schema": "armctrl.arm_runtime_owner_lease.v1",
        "runtime_session_id": session["runtime_session_id"],
        "owner": "agent",
        "mode": "agent_servo",
        "heartbeat_timeout_s": 1.0,
    }
    runtime_session_artifact.write_text(
        json.dumps(session, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    before = runtime_session_artifact.read_text(encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "fake",
            "--dof",
            "6",
            "--sample-hz",
            "20",
            "--duration",
            "2",
            "--amplitude",
            "0.1",
            "--q-center",
            *[str(value) for value in safe_center],
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
    assert payload["adapter"] == "fake"
    assert (output_dir / "raw_samples.csv").exists()
    assert runtime_session_artifact.read_text(encoding="utf-8") == before


def test_cli_sysid_run_sdk_real_motion_entrypoint_is_removed_at_parser_layer(
    tmp_path: Path,
) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "run",
            "gravity_sweep",
            "--adapter",
            "sdk",
            "--output",
            str(tmp_path / "ident-run"),
            "--json",
        ],
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert completed.stdout == ""
    assert "invalid choice: 'sdk'" in completed.stderr
    assert "--adapter {fake}" in completed.stderr
    assert not (tmp_path / "ident-run").exists()


def test_sysid_run_module_does_not_keep_legacy_sdk_runner_gate() -> None:
    import armctrl.sysid_run as sysid_run

    source = Path(sysid_run.__file__).read_text(encoding="utf-8")

    assert not hasattr(sysid_run, "SdkSysIdRunnerGate")
    assert "sdk-agent-sysid-smoke-readiness" not in source
    assert "complete real tiny motion before sdk sysid run" not in source


def test_cli_sysid_run_sdk_removed_even_with_live_runtime_and_candidate(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    output_dir = tmp_path / "ident-run"
    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    _write_live_runtime_status(readiness_artifact)
    _write_fake_runtime_session(runtime_session)
    candidate = tmp_path / "candidate.csv"
    candidate.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0.000000,0.300000,0.300000,0.000000,0.000000,0.000000\n"
        "1.000000,0.001000,0.300000,0.300000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )

    class ForbiddenArx5Backend:
        def __init__(self, *args, **kwargs) -> None:
            raise AssertionError("sdk sysid must not open SDK without runtime")

    def forbidden_submit(**kwargs) -> dict[str, object]:
        raise AssertionError("removed sysid run must not submit runtime command")

    monkeypatch.setattr(cli, "Arx5InterfaceCollectionBackend", ForbiddenArx5Backend)
    monkeypatch.setattr(cli, "submit_trajectory_command", forbidden_submit)

    with pytest.raises(SystemExit) as error:
        cli.main(
            [
                "sysid",
                "run",
                "gravity_sweep",
                "--adapter",
                "sdk",
                "--dof",
                "6",
                "--sample-hz",
                "20",
                "--duration",
                "2",
                "--amplitude",
                "0.1",
                "--q-center",
                "0",
                "0.3",
                "0.3",
                "0",
                "0",
                "0",
                "--candidate-trajectory",
                str(candidate),
                "--readiness-artifact",
                str(readiness_artifact),
                "--runtime-session-artifact",
                str(runtime_session),
                "--confirm",
                SDK_CONFIRMATION,
                "--output",
                str(output_dir),
                "--json",
            ]
        )

    captured = capsys.readouterr()
    assert error.value.code == 2
    assert captured.out == ""
    assert "invalid choice: 'sdk'" in captured.err
    assert "--adapter {fake}" in captured.err
    assert not (output_dir / "runtime_session_commands").exists()
    assert not (output_dir / "execution_trajectory.csv").exists()
    assert not (output_dir / "manifest.json").exists()


def test_cli_sysid_run_sdk_removed_does_not_report_fake_acceptance_or_faults(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    from armctrl import cli

    readiness_artifact = tmp_path / "readiness.json"
    runtime_session = tmp_path / "runtime-session.json"
    output_dir = tmp_path / "ident-sdk-cli-review-required"
    _write_fake_runtime_session(runtime_session)
    _write_live_runtime_status(readiness_artifact)

    class ForbiddenBackendFactory:
        def __init__(self, **kwargs) -> None:
            raise AssertionError("sdk sysid must not open SDK outside runtime")

    monkeypatch.setattr(cli, "Arx5InterfaceCollectionBackend", ForbiddenBackendFactory)

    with pytest.raises(SystemExit) as error:
        cli.main(
            [
                "sysid",
                "run",
                "gravity_sweep",
                "--adapter",
                "sdk",
                "--dof",
                "6",
                "--sample-hz",
                "20",
                "--duration",
                "1",
                "--amplitude",
                "0.02",
                "--q-center",
                "0",
                "0.3",
                "0.3",
                "0",
                "0",
                "0",
                "--output",
                str(output_dir),
                "--confirm",
                SDK_CONFIRMATION,
                "--readiness-artifact",
                str(readiness_artifact),
                "--runtime-session-artifact",
                str(runtime_session),
                "--max-tracking-error-rad",
                "0.04",
                "--max-tau-abs",
                "2.0",
                "--max-heartbeat-age-s",
                "5",
                "--json",
            ]
        )

    captured = capsys.readouterr()
    assert error.value.code == 2
    assert captured.out == ""
    assert "invalid choice: 'sdk'" in captured.err
    assert "--adapter {fake}" in captured.err
    assert not (output_dir / "manifest.json").exists()


def test_cli_sysid_run_rejects_legacy_real_motion_options(tmp_path: Path) -> None:
    legacy_options = [
        ("--confirm", SDK_CONFIRMATION),
        ("--readiness-artifact", str(tmp_path / "runtime_status.json")),
        ("--max-tracking-error-rad", "0.04"),
        ("--max-tau-abs", "2.0"),
        ("--max-heartbeat-age-s", "5"),
    ]

    for option, value in legacy_options:
        output_dir = tmp_path / option.removeprefix("--")
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "armctrl.cli",
                "sysid",
                "run",
                "gravity_sweep",
                "--adapter",
                "fake",
                "--output",
                str(output_dir),
                option,
                value,
                "--json",
            ],
            capture_output=True,
            text=True,
        )

        assert completed.returncode == 2
        assert completed.stdout == ""
        assert f"unrecognized arguments: {option}" in completed.stderr
        assert not (output_dir / "manifest.json").exists()


def test_cli_sysid_postprocess_solve_blocks_faulted_sdk_manifest(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    dataset_dir = tmp_path / "faulted-sdk-dataset"
    dataset_dir.mkdir()
    (dataset_dir / "raw_samples.csv").write_text(
        "time_s,q_cmd_1,q_1,dq_1,tau_meas_1\n0.000000,0.000000,0.000000,0.000000,0.000000\n",
        encoding="utf-8",
    )
    (dataset_dir / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_run_manifest.v1",
                "adapter": "sdk",
                "run_status": "faulted",
                "handoff": {"status": "blocked"},
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "sysid_smoke",
                    "status": "review_required",
                },
            }
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "sysid",
            "postprocess",
            "--dataset",
            str(dataset_dir),
            "--solve",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.sysid_postprocess.v1"
    assert payload["solver_gate"] == {
        "status": "blocked",
        "reason": "sdk run_status is faulted",
        "next_gate": "review sysid smoke acceptance before solver",
        "checks": {
            "run_status_completed": False,
            "acceptance_passed": False,
        },
    }
    assert "solver" not in payload
    assert (dataset_dir / "processed" / "processed_samples.csv").exists()


def test_cli_sysid_solve_blocks_failed_sdk_acceptance(
    tmp_path: Path,
    capsys,
) -> None:
    from armctrl import cli

    dataset_dir = tmp_path / "failed-acceptance-sdk-dataset"
    dataset_dir.mkdir()
    (dataset_dir / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "armctrl.sysid_run_manifest.v1",
                "adapter": "sdk",
                "run_status": "completed",
                "acceptance": {
                    "schema": "armctrl.real_motion_acceptance.v1",
                    "stage": "sysid_smoke",
                    "status": "review_required",
                },
            }
        ),
        encoding="utf-8",
    )

    exit_code = cli.main(
        [
            "sysid",
            "solve",
            "--dataset",
            str(dataset_dir),
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)

    assert exit_code == 3
    assert payload["status"] == "blocked"
    assert payload["schema"] == "armctrl.sysid_solve.v1"
    assert payload["solver_gate"] == {
        "status": "blocked",
        "reason": "sdk acceptance is not pass",
        "next_gate": "review sysid smoke acceptance before solver",
        "checks": {
            "run_status_completed": True,
            "acceptance_passed": False,
        },
    }


class FakeJointState:
    def __init__(self, dof: int) -> None:
        self._pos = [0.0] * dof
        self._vel = [0.0] * dof
        self._torque = [0.0] * dof
        self.gripper_pos = 0.0

    def pos(self) -> list[float]:
        return self._pos

    def vel(self) -> list[float]:
        return self._vel

    def torque(self) -> list[float]:
        return self._torque


class FakeController:
    def __init__(self, robot_config, controller_config, interface: str) -> None:
        self.commands: list[list[float]] = []
        self.damping_count = 0
        self.reset_home_count = 0

    def reset_to_home(self) -> None:
        self.reset_home_count += 1

    def set_joint_cmd(self, cmd: FakeJointState) -> None:
        self.commands.append(list(cmd.pos()))

    def get_joint_state(self) -> FakeJointState:
        state = FakeJointState(6)
        if self.commands:
            state.pos()[:] = self.commands[-1]
        return state

    def set_to_damping(self) -> None:
        self.damping_count += 1


class FakeRobotConfig:
    joint_dof = 6


class FakeControllerConfig:
    controller_dt = 0.0
    gravity_compensation = False
    background_send_recv = False


class FakeConfigFactory:
    def __init__(self, value) -> None:
        self._value = value

    @classmethod
    def get_instance(cls):
        return cls(cls._value)

    def get_config(self, *args):
        return self._value


class FakeRobotConfigFactory(FakeConfigFactory):
    _value = FakeRobotConfig()


class FakeControllerConfigFactory(FakeConfigFactory):
    _value = FakeControllerConfig()


class FakeArx5Module:
    RobotConfigFactory = FakeRobotConfigFactory
    ControllerConfigFactory = FakeControllerConfigFactory
    JointState = FakeJointState
    last_controller: FakeController | None = None

    @staticmethod
    def Arx5JointController(robot_config, controller_config, interface: str) -> FakeController:
        controller = FakeController(robot_config, controller_config, interface)
        FakeArx5Module.last_controller = controller
        return controller


def test_arx5_interface_backend_sends_joint_commands_and_lands_damping() -> None:
    clock = ManualClock()
    request = SysIdPlanRequest(
        profile_name="gravity_sweep",
        dof=6,
        sample_hz=2,
        duration_s=1,
        amplitude_rad=0.05,
        q_center=(0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        urdf_path=str(X5_URDF),
        safe_config_path=str(X5_SAFE_CONFIG),
        output_dir=Path("unused"),
    )
    backend = Arx5InterfaceCollectionBackend(
        model="X5",
        interface="can0",
        arx5_module=FakeArx5Module,
        max_joint_step_rad=0.2,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    backend.enter_hold_or_damping()
    rows = backend.read_samples(request)
    backend.enter_damping()

    expected_sample_count = len(trajectory_rows(request))

    assert len(rows) == expected_sample_count
    assert backend.last_motion_result is not None
    assert backend.last_motion_result.producer == "sysid"
    assert backend.last_motion_result.trajectory_sample_hz == 2.0
    assert backend.last_motion_result.actual_send_hz == 2.0
    assert backend.last_motion_result.controller_dt_s == 0.01
    assert len(backend.last_motion_result.samples) == expected_sample_count
    assert backend.last_ramp_result is not None
    assert backend.last_ramp_result.producer == "sysid_ramp"
    assert backend.last_ramp_result.trajectory_sample_hz == 2.0
    assert backend.last_ramp_result.actual_send_hz == 2.0
    assert len(backend.last_ramp_result.samples) == 2
    assert rows[0]["q_cmd_2"] == "0.300000"
    assert float(rows[0]["q_2"]) > float(rows[0]["q_cmd_2"])
    assert "tau_meas_6" in rows[0]
    assert FakeArx5Module.last_controller is not None
    assert len(FakeArx5Module.last_controller.commands) > 3
    assert FakeArx5Module.last_controller.reset_home_count == 0
    for previous, current in zip(
        FakeArx5Module.last_controller.commands,
        FakeArx5Module.last_controller.commands[1:],
    ):
        assert max(abs(a - b) for a, b in zip(previous, current)) <= 0.2 + 1e-9
    assert FakeArx5Module.last_controller.damping_count == 1


def test_arx5_interface_backend_preserves_sdk_fault_flags() -> None:
    class FaultFlagJointState(FakeJointState):
        def fault_flags(self) -> list[str]:
            return ["over_current", "encoder_fault"]

    class FaultFlagController(FakeController):
        def get_joint_state(self) -> FaultFlagJointState:
            state = FaultFlagJointState(6)
            if self.commands:
                state.pos()[:] = self.commands[-1]
            return state

    class FaultFlagArx5Module(FakeArx5Module):
        @staticmethod
        def Arx5JointController(robot_config, controller_config, interface: str):
            return FaultFlagController(robot_config, controller_config, interface)

    backend = Arx5InterfaceCollectionBackend(
        model="X5",
        interface="can0",
        arx5_module=FaultFlagArx5Module,
    )

    backend.enter_hold_or_damping()
    backend.send_joint_command(
        (0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        producer="sysid",
        mode=MotionMode.TRAJECTORY_REPLAY,
        monotonic_s=0.0,
    )

    snapshot = backend.read_joint_state()

    assert snapshot.fault_flags == ("over_current", "encoder_fault")


def test_arx5_interface_backend_uses_measured_controller_dt_when_provided() -> None:
    backend = Arx5InterfaceCollectionBackend(
        model="X5",
        interface="can0",
        arx5_module=FakeArx5Module,
        controller_dt_s=0.002,
    )

    backend.enter_hold_or_damping()

    assert backend.controller_dt_s == 0.002
