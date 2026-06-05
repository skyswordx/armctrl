import json
import importlib.util
import subprocess
import sys
import types
from pathlib import Path

import pytest

import armctrl.simulation as simulation
from armctrl.simulation import SimulationDoctor, TrajectoryPreviewer
from armctrl.workspace import WorkspaceSafetyConfig


def test_safe_config_loads_named_allowed_and_forbidden_spaces() -> None:
    config = WorkspaceSafetyConfig.from_yaml(Path("configs/x5.safe.yaml"))

    assert config.allowed_workspace_boxes[0].name == "main_body_sweep_volume"
    assert config.allowed_workspace_boxes[0].min_m == (-0.35, -0.45, 0.02)
    assert config.allowed_workspace_boxes[0].max_m == (0.75, 0.45, 0.65)
    assert config.forbidden_workspace_boxes[0].name == "table_surface"
    assert config.forbidden_workspace_boxes[0].max_m[2] == 0.02
    assert config.simulation_backend_preference == (
        "pinocchio_coal",
        "mujoco",
        "moveit",
        "urdf_fk_fallback",
    )
    assert config.allowed_collision_pairs == (
        ("base_link", "link1"),
        ("link1", "link2"),
        ("link5", "link6"),
    )


def test_allowed_collision_pair_matching_accepts_pinocchio_geometry_suffixes() -> None:
    allowed_pairs = (
        ("base_link", "link1"),
        ("link1", "link2"),
    )

    assert simulation._is_allowed_collision_pair(
        "base_link_0",
        "link1_0",
        allowed_pairs,
    )
    assert simulation._is_allowed_collision_pair(
        "link2_0",
        "link1_0",
        allowed_pairs,
    )
    assert not simulation._is_allowed_collision_pair(
        "link2_0",
        "link6_0",
        allowed_pairs,
    )


def test_simulation_doctor_reports_mature_backend_importability() -> None:
    result = SimulationDoctor().run()

    assert result["schema"] == "armctrl.simulation_doctor.v1"
    assert result["movement_allowed"] is False
    names = [backend["name"] for backend in result["backends"]]
    assert names == ["pinocchio_coal", "mujoco", "moveit", "figaroh"]
    assert result["backends"][0]["role"] == "lightweight URDF geometry collision checks"
    assert result["backends"][1]["role"] == "contact and dynamics simulation preview"
    assert result["backends"][2]["runtime"] == "ros2_moveit"


def test_cli_sim_doctor_is_read_only_json() -> None:
    completed = subprocess.run(
        [sys.executable, "-m", "armctrl.cli", "sim", "doctor", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)

    assert payload["status"] == "ok"
    assert payload["schema"] == "armctrl.simulation_doctor.v1"
    assert payload["movement_allowed"] is False
    assert payload["backends"][2]["name"] == "moveit"


def test_moveit_doctor_detects_ros2_install_when_python_env_is_not_sourced(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_find_spec = importlib.util.find_spec

    def fake_find_spec(name: str) -> object | None:
        if name in {"rclpy", "moveit_msgs", "moveit_configs_utils"}:
            return None
        return original_find_spec(name)

    monkeypatch.setattr(simulation.importlib.util, "find_spec", fake_find_spec)
    monkeypatch.setattr(simulation, "_installed_ros_distribution", lambda: "jazzy")

    result = SimulationDoctor().run()
    moveit = next(backend for backend in result["backends"] if backend["name"] == "moveit")

    assert moveit["status"] == "installed_not_sourced"
    assert moveit["source_hint"] == "source /opt/ros/jazzy/setup.bash"


def test_native_geometry_urdf_rewrites_relative_mesh_paths_to_absolute(
    tmp_path: Path,
) -> None:
    prepared = simulation._prepare_urdf_for_native_geometry(
        Path("configs/models/X5_camera.urdf"),
        output_dir=tmp_path,
    )
    root = simulation.ET.parse(prepared).getroot()
    filenames = [
        mesh.attrib["filename"]
        for mesh in root.findall(".//mesh")
        if "filename" in mesh.attrib
    ]

    assert filenames
    assert all(Path(filename).is_absolute() for filename in filenames)
    assert any(Path(filename).name == "base_link.STL" for filename in filenames)


def test_mujoco_preview_rolls_trajectory_forward_and_reports_contacts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    forwarded_qpos: list[tuple[float, ...]] = []

    class FakeModel:
        nq = 6

        @classmethod
        def from_xml_path(cls, path: str) -> "FakeModel":
            assert path.endswith("X5_camera.urdf")
            return cls()

    class FakeData:
        def __init__(self, model: FakeModel) -> None:
            self.qpos = [0.0] * model.nq
            self.ncon = 0

    def fake_forward(model: FakeModel, data: FakeData) -> None:
        forwarded_qpos.append(tuple(data.qpos))
        data.ncon = 0

    fake_mujoco = types.SimpleNamespace(
        MjModel=FakeModel,
        MjData=FakeData,
        mj_forward=fake_forward,
    )
    monkeypatch.setitem(sys.modules, "mujoco", fake_mujoco)

    result = simulation._mujoco_trajectory_check(
        urdf_path=Path("configs/models/X5_camera.urdf"),
        q_samples=[
            (0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
            (0.1, 0.2, 0.4, 0.0, 0.0, 0.0),
        ],
    )

    assert result["status"] == "pass"
    assert result["method"] == "mujoco_trajectory_rollout"
    assert result["checked_samples"] == 2
    assert result["max_contact_count"] == 0
    assert forwarded_qpos == [
        (0.0, 0.3, 0.3, 0.0, 0.0, 0.0),
        (0.1, 0.2, 0.4, 0.0, 0.0, 0.0),
    ]


def test_mujoco_preview_ignores_only_named_allowed_contacts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeModel:
        nq = 6
        geom_bodyid = [0, 1]

        @classmethod
        def from_xml_path(cls, path: str) -> "FakeModel":
            return cls()

    class FakeContact:
        geom1 = 0
        geom2 = 1

    class FakeData:
        def __init__(self, model: FakeModel) -> None:
            self.qpos = [0.0] * model.nq
            self.ncon = 1
            self.contact = [FakeContact()]

    def fake_forward(model: FakeModel, data: FakeData) -> None:
        data.ncon = 1

    def fake_name(model: FakeModel, obj_type: object, object_id: int) -> str | None:
        if obj_type == "body":
            return {0: "world", 1: "link1"}[object_id]
        return None

    fake_mujoco = types.SimpleNamespace(
        MjModel=FakeModel,
        MjData=FakeData,
        mj_forward=fake_forward,
        mj_id2name=fake_name,
        mjtObj=types.SimpleNamespace(mjOBJ_GEOM="geom", mjOBJ_BODY="body"),
    )
    monkeypatch.setitem(sys.modules, "mujoco", fake_mujoco)

    result = simulation._mujoco_trajectory_check(
        urdf_path=Path("configs/models/X5_camera.urdf"),
        q_samples=[(0.0, 0.3, 0.3, 0.0, 0.0, 0.0)],
        allowed_collision_pairs=(("base_link", "link1"),),
    )

    assert result["status"] == "pass"
    assert result["raw_max_contact_count"] == 1
    assert result["max_contact_count"] == 0
    assert result["ignored_allowed_collision_pairs"] == [
        {"first": "base_link", "second": "link1"}
    ]


def test_sysid_plan_writes_trajectory_preview_and_includes_simulation_gate(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "ident-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "2",
            "--amplitude",
            "0.05",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    preview = json.loads(
        (output_dir / "trajectory_preview.json").read_text(encoding="utf-8")
    )

    assert payload["artifacts"]["trajectory_preview"] == str(
        output_dir / "trajectory_preview.json"
    )
    assert payload["artifact_safety"]["simulation_check"]["status"] == "pass"
    assert manifest["safety"]["checks"]["simulation_check"]["status"] == "pass"
    assert preview["schema"] == "armctrl.trajectory_preview.v1"
    assert preview["backend"]["selected"] in {
        "pinocchio_coal",
        "mujoco",
        "moveit",
        "urdf_fk_fallback",
    }
    assert preview["safety"]["zone_check"]["status"] == "pass"
    assert preview["trajectory_metrics"]["joint_ranges_rad"]["joint_1"] > 0.09


def test_sysid_plan_can_render_trajectory_preview_svg(tmp_path: Path) -> None:
    output_dir = tmp_path / "ident-plan"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sysid",
            "plan",
            "gravity_sweep",
            "--dof",
            "6",
            "--sample-hz",
            "100",
            "--duration",
            "2",
            "--amplitude",
            "0.05",
            "--q-center",
            "0",
            "0.3",
            "0.3",
            "0",
            "0",
            "0",
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--output",
            str(output_dir),
            "--render",
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    render_path = output_dir / "trajectory_preview.svg"

    assert payload["artifacts"]["trajectory_render"] == str(render_path)
    assert render_path.exists()
    assert "armctrl trajectory preview" in render_path.read_text(encoding="utf-8")


def test_trajectory_preview_rejects_forbidden_workspace_entry(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "planned_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0,3.0,1.0,0,0,0\n",
        encoding="utf-8",
    )

    result = TrajectoryPreviewer().preview(
        trajectory_path=trajectory_path,
        urdf_path=Path("configs/models/X5_camera.urdf"),
        safe_config_path=Path("configs/x5.safe.yaml"),
    )

    assert result["safety"]["allowed"] is False
    assert result["safety"]["zone_check"]["status"] == "fail"
    assert result["safety"]["zone_check"]["violations"][0]["check"] in {
        "outside_allowed_workspace",
        "inside_forbidden_workspace",
    }


def test_cli_sim_preview_can_render_unsafe_trajectory_warning_svg(
    tmp_path: Path,
) -> None:
    trajectory_path = tmp_path / "planned_trajectory.csv"
    render_path = tmp_path / "preview.svg"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0,3.0,1.0,0,0,0\n",
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "armctrl.cli",
            "sim",
            "preview",
            "--trajectory",
            str(trajectory_path),
            "--urdf-path",
            "configs/models/X5_camera.urdf",
            "--safe-config",
            "configs/x5.safe.yaml",
            "--render",
            str(render_path),
            "--json",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    payload = json.loads(completed.stdout)
    svg = render_path.read_text(encoding="utf-8")

    assert payload["render"]["path"] == str(render_path)
    assert payload["render"]["status"] == "written"
    assert payload["safety"]["allowed"] is False
    assert "WARNING" in svg
    assert "outside_allowed_workspace" in svg


def test_auto_preview_records_failed_mature_backend_attempt_and_falls_back(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    trajectory_path = tmp_path / "planned_trajectory.csv"
    trajectory_path.write_text(
        "time_s,q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6\n"
        "0.000000,0,0.3,0.3,0,0,0\n"
        "0.010000,0.001,0.3,0.3,0,0,0\n",
        encoding="utf-8",
    )

    original_find_spec = importlib.util.find_spec

    def fake_find_spec(name: str) -> object | None:
        if name in {"pinocchio", "coal"}:
            return object()
        if name in {"mujoco", "rclpy", "moveit_msgs", "moveit_configs_utils"}:
            return None
        return original_find_spec(name)

    monkeypatch.setattr(simulation.importlib.util, "find_spec", fake_find_spec)
    monkeypatch.setattr(
        simulation,
        "_pinocchio_coal_check",
        lambda *, urdf_path, q_samples, allowed_collision_pairs: {
            "status": "not_evaluated",
            "method": "pinocchio_coal",
            "reason": "Mesh ./meshes/base_link.STL could not be found.",
        },
    )

    result = TrajectoryPreviewer().preview(
        trajectory_path=trajectory_path,
        urdf_path=Path("configs/models/X5_camera.urdf"),
        safe_config_path=Path("configs/x5.safe.yaml"),
    )

    assert result["backend"]["selected"] == "urdf_fk_fallback"
    assert result["backend"]["attempts"][0]["backend"] == "pinocchio_coal"
    assert result["backend"]["attempts"][0]["result"]["status"] == "not_evaluated"
    assert result["backend"]["result"]["status"] == "pass"
    assert result["safety"]["allowed"] is True
