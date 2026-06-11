from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_runtime_gateway_docs_describe_removed_sysid_sdk_parser_entrypoint() -> None:
    current_docs = [
        ROOT / "docs" / "runtime_gateway" / "README.md",
        ROOT / "docs" / "runtime_gateway" / "sysid_integration.md",
    ]

    combined = "\n".join(path.read_text(encoding="utf-8") for path in current_docs)

    assert "sysid run --adapter sdk" in combined
    assert "sysid compile-runtime" in combined
    assert "motion submit joint-trajectory" in combined
    assert "migration/rejected payload" not in combined
    assert "返回迁移拒绝" not in combined
    assert "仅返回迁移拒绝" not in combined
    assert "parser" in combined
    assert "--adapter {fake}" in combined


def test_runtime_gateway_docs_describe_fake_agent_eef_adapter_rehearsal() -> None:
    current_docs = [
        ROOT / "docs" / "runtime_gateway" / "README.md",
        ROOT / "docs" / "runtime_gateway" / "agent_eef_control.md",
    ]

    combined = "\n".join(path.read_text(encoding="utf-8") for path in current_docs)

    assert "ARMCTRL_BACKEND=fake scripts/lab_agent_runtime_smoke.sh start" in combined
    assert "--eef-adapter moveit_servo" in combined
    assert "run-eef" in combined
    assert "check-eef" in combined
    assert "adapter registry" in combined
    assert "不能证明真实 ARX5 EEF 运动质量" in combined
    assert "heuristic fallback" in combined


def test_lab_docs_do_not_publish_executable_removed_sysid_sdk_commands() -> None:
    lab_docs = [
        ROOT / "README.md",
        ROOT / "docs" / "others" / "lab_hardware_validation_checklist.md",
        ROOT / "docs" / "others" / "hardware_sysid_operator_manual.md",
    ]

    for path in lab_docs:
        text = path.read_text(encoding="utf-8")
        assert "uv run armctrl sysid run gravity_sweep --adapter sdk" not in text
        assert "--adapter sdk` remains rejected" not in text
        assert "armctrl sysid compile-runtime" in text
        assert "armctrl motion submit joint-trajectory" in text


def test_lab_operator_docs_do_not_publish_removed_sysid_run_command_block() -> None:
    lab_docs = [
        ROOT / "docs" / "others" / "lab_hardware_validation_checklist.md",
        ROOT / "docs" / "others" / "hardware_sysid_operator_manual.md",
    ]

    for path in lab_docs:
        text = path.read_text(encoding="utf-8")
        assert "uv run armctrl sysid run gravity_sweep \\" not in text


def test_lab_checklist_uses_formal_motion_result_surface() -> None:
    checklist = ROOT / "docs" / "others" / "lab_hardware_validation_checklist.md"
    text = checklist.read_text(encoding="utf-8")

    assert "uv run armctrl runtime result-check" not in text
    assert "uv run armctrl motion result" in text


def test_sysid_operator_manual_keeps_readiness_and_transition_runtime_owned() -> None:
    manual = ROOT / "docs" / "others" / "hardware_sysid_operator_manual.md"
    text = manual.read_text(encoding="utf-8")

    assert "sysid run --adapter sdk` 会用同一个阈值" not in text
    assert "--tiny-motion-artifact \"$RUN_DIR/tiny_motion_real.json\"" not in text
    assert "Runtime-owned SysID execution no longer uses `sysid run --adapter sdk`" in text
    assert "--runtime-status-artifact \"$RUN_DIR/runtime_status.json\"" in text
