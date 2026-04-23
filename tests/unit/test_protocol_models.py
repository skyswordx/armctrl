"""协议模型测试。

这类测试看起来简单，但它们锁定的是整个工程最底层的数据契约。
数据契约一旦漂移，CLI、GUI、adapter 和测试会一起出问题。
"""

from armctrl.protocol.enums import CommandStatus
from armctrl.protocol.models import CommandResponse, EEFStateModel, TeleopCommand


def test_teleop_command_rejects_wrong_vector_length():
    # `TeleopCommand` 的平移和旋转向量长度必须固定，
    # 否则后面的 6D 位姿运算会出现索引错误或语义错位。
    try:
        TeleopCommand(translation_m=(0.0, 0.0), rotation_rad=(0.0, 0.0, 0.0))
    except ValueError as exc:
        assert "translation_m" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_response_json_roundtrip():
    # 当前 roundtrip 至少保证状态码能还原。
    response = CommandResponse(status=CommandStatus.COMPLETED, message="ok")
    payload = response.to_json()
    assert '"completed"' in payload
    assert CommandResponse.from_json(payload).status is CommandStatus.COMPLETED


def test_eef_state_has_six_dof_pose():
    # 末端状态模型的第一性原则：必须是完整 6D pose。
    state = EEFStateModel(pose_6d=(0.3, 0.0, 0.2, 0.0, 0.0, 0.0))
    assert state.pose_6d[0] == 0.3
