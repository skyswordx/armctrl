from pathlib import Path
import xml.etree.ElementTree as ET


def test_x5_camera_urdf_mesh_assets_exist() -> None:
    urdf_path = Path("configs/models/X5_camera.urdf")
    root = ET.parse(urdf_path).getroot()

    mesh_paths = [
        urdf_path.parent / mesh.attrib["filename"]
        for mesh in root.findall(".//mesh")
        if not mesh.attrib["filename"].startswith("package://")
    ]

    assert mesh_paths
    missing = [path for path in mesh_paths if not path.exists()]
    assert missing == []


def test_d435_visual_mesh_asset_is_present() -> None:
    assert Path(
        "configs/models/reference/realsense2_description/meshes/d435.dae"
    ).exists()
