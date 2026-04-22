from armctrl.protocol.enums import CommandStatus
from armctrl.protocol.models import CommandResponse, EEFStateModel, TeleopCommand


def test_teleop_command_rejects_wrong_vector_length():
    try:
        TeleopCommand(translation_m=(0.0, 0.0), rotation_rad=(0.0, 0.0, 0.0))
    except ValueError as exc:
        assert "translation_m" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_response_json_roundtrip():
    response = CommandResponse(status=CommandStatus.COMPLETED, message="ok")
    payload = response.to_json()
    assert '"completed"' in payload
    assert CommandResponse.from_json(payload).status is CommandStatus.COMPLETED


def test_eef_state_has_six_dof_pose():
    state = EEFStateModel(pose_6d=(0.3, 0.0, 0.2, 0.0, 0.0, 0.0))
    assert state.pose_6d[0] == 0.3
