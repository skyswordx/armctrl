"""项目侧 URDF payload 模型测试。

ARX5 SDK 的 KDL 逆动力学会使用 `link6` 作为最后一个动力学链路。
因此 D435i payload 不能只写在 `eef_link` 的 fixed joint 后面，
必须合并进 `link6` 的惯性参数，重力补偿才能读到它。
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


MODEL_PATH = Path("configs/models/X5_camera.urdf")


def _link(root: ET.Element, name: str) -> ET.Element:
    for link in root.findall("link"):
        if link.attrib.get("name") == name:
            return link
    raise AssertionError(f"link not found: {name}")


def test_d435i_payload_is_merged_into_link6_inertial():
    root = ET.parse(MODEL_PATH).getroot()
    link6 = _link(root, "link6")
    inertial = link6.find("inertial")
    assert inertial is not None

    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    assert origin is not None
    assert mass is not None
    assert inertia is not None

    assert float(mass.attrib["value"]) == pytest.approx(0.653)
    assert [float(value) for value in origin.attrib["xyz"].split()] == pytest.approx(
        [0.06434756, 0.0, 0.00441041],
    )
    assert float(inertia.attrib["ixx"]) == pytest.approx(0.000444848009)
    assert float(inertia.attrib["ixz"]) == pytest.approx(-0.000232279015)
    assert float(inertia.attrib["iyy"]) == pytest.approx(0.000916399242)
    assert float(inertia.attrib["izz"]) == pytest.approx(0.001028756233)


def test_eef_link_no_longer_owns_payload_inertial():
    root = ET.parse(MODEL_PATH).getroot()
    eef_link = _link(root, "eef_link")
    assert eef_link.find("inertial") is None
