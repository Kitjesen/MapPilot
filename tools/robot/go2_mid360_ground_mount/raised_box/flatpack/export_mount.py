"""Export the installed six-piece mount in assembly coordinates, not print coordinates."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1]))
# Import the shared exporter by path because this script has the same filename.
import importlib.util

spec = importlib.util.spec_from_file_location("box_export", HERE.parent / "export_mount.py")
shared = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shared)
from generate_flatpack import OPTICAL, ROT

ASSET = "mid360_flatpack"
PARTS = ("base", "left_side", "right_side", "lid", "left_foot", "right_foot")
MOUNT = np.array([0.19, 0.0, 0.101])


def main():
    robot = ET.parse(shared.ASSETS / "urdf/go2_description.urdf").getroot()
    robot.set("name", "go2_mid360_flatpack")
    robot.insert(0, ET.Comment(" Installed mount CAD nominal; inertia is a solid-material approximation. "))
    mesh_dir = shared.ASSETS / "meshes" / ASSET
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for name in PARTS:
        data = np.load(HERE / f"{name}.npz")
        vertices, faces = data["vertices"], data["faces"]
        link = ET.SubElement(robot, "link", name=f"mid360_{name}")
        shared.inertial(link, *shared.mass_properties(vertices, faces, 1240.0))
        triangles = shared.simplify_for_preview(vertices[faces] / 1000, cell=0.0015)
        shared.dae(mesh_dir / f"{name}.dae", triangles.reshape(-1, 3) * 1000,
                   np.arange(triangles.size // 3).reshape(-1, 3), color="0.04 0.28 0.80")
        shared.visual(link, f"{ASSET}/{name}.dae")
        shared.collision_box(link, vertices.min(axis=0), vertices.max(axis=0))
        shared.fixed(robot, "base", f"mid360_{name}", MOUNT)

    sensor = ET.SubElement(robot, "link", name="livox_frame")
    shared.visual(sensor, "mid360_ground_mount/mid360_official.dae")
    low, high = np.array([-32.5, -32.5, -47.0]), np.array([32.5, 32.5, 13.0])
    shared.collision_box(sensor, low, high)
    size = (high - low) / 1000
    inertia = 0.265 / 12 * np.diag([size[1]**2 + size[2]**2,
                                    size[0]**2 + size[2]**2, size[0]**2 + size[1]**2])
    shared.inertial(sensor, 0.265, (low + high) / 2000, inertia)
    pitch = float(np.arctan2(ROT[0, 2], ROT[0, 0]))
    shared.fixed(robot, "base", "livox_frame", MOUNT + OPTICAL / 1000, (0, pitch, 0))
    slam = yaml.safe_load((shared.ROOT / "config/robots/unitree/go2/sensors/mid360_ltlio.yaml").read_text())
    r_il = np.array(slam["r_il"]).reshape(3, 3)
    if not np.allclose(r_il, np.eye(3), atol=1e-12):
        raise ValueError("Update IMU joint rotation for changed package calibration")
    ET.SubElement(robot, "link", name="livox_imu")
    shared.fixed(robot, "livox_frame", "livox_imu", -np.array(slam["t_il"]))
    ET.indent(robot, space="  ")
    target = shared.ASSETS / "urdf/go2_mid360_flatpack.urdf"
    ET.ElementTree(robot).write(target, encoding="utf-8", xml_declaration=True)
    print(target)


if __name__ == "__main__":
    main()
