"""Export V7 nominal mounting assets; never activate calibration on the robot."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE.parent))
from generate import dae, mass_properties, ry, vector
from preview import simplify_for_preview

ASSETS = ROOT / "web/public/assets/robots/go2"


def fixed(robot, parent, child, xyz, rpy=(0, 0, 0)):
    joint = ET.SubElement(robot, "joint", name=f"{child}_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=vector(xyz), rpy=vector(rpy))


def inertial(link, mass, center, inertia):
    node = ET.SubElement(link, "inertial")
    ET.SubElement(node, "origin", xyz=vector(center), rpy="0 0 0")
    ET.SubElement(node, "mass", value=f"{mass:.12g}")
    ET.SubElement(
        node,
        "inertia",
        **{
            key: f"{inertia[i, j]:.12g}"
            for key, i, j in (("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2), ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2))
        },
    )


def visual(link, mesh):
    node = ET.SubElement(link, "visual")
    ET.SubElement(ET.SubElement(node, "geometry"), "mesh", filename=f"package://go2_description/meshes/{mesh}")


def collision_box(link, low, high):
    node = ET.SubElement(link, "collision")
    ET.SubElement(node, "origin", xyz=vector((low + high) / 2000), rpy="0 0 0")
    ET.SubElement(ET.SubElement(node, "geometry"), "box", size=vector((high - low) / 1000))


def export(directory=HERE, asset_name="mid360_raised_box"):
    mesh_dir = ASSETS / "meshes" / asset_name
    candidate = directory / "candidate"
    p = json.loads((directory / "parameters.json").read_text())
    rotation = ry(p["pitch_deg"])
    pitch = math.radians(p["pitch_deg"])
    mount = np.array(p["mount_xyz_m"])
    optical = np.array(p["optical_xyz_m"])
    height = p["optical_above_bottom_mm"] / 1000
    seat = optical - mount - rotation @ np.array([0, 0, height])
    robot = ET.parse(ASSETS / "urdf/go2_description.urdf").getroot()
    robot.set("name", f"go2_{asset_name}_v7_nominal")
    robot.insert(0, ET.Comment(" V7 nominal prototype; physical fit and calibration not field verified. "))
    mesh_dir.mkdir(parents=True, exist_ok=True)
    candidate.mkdir(exist_ok=True)
    bounds = []
    for name in ("box", "lid", "left_pad", "right_pad"):
        data = np.load(directory / f"{name}.npz")
        vertices, faces = data["vertices"], data["faces"]
        bounds.append(vertices / 1000 + mount)
        if name == "lid":
            vertices = (vertices - seat * 1000) @ rotation
        child = f"mid360_{name}"
        link = ET.SubElement(robot, "link", name=child)
        inertial(link, *mass_properties(vertices, faces, p["density_kg_m3"]))
        # Display-only clustering; print files and mass calculations remain full resolution.
        triangles = simplify_for_preview(vertices[faces] / 1000, cell=0.0015)
        display_vertices = triangles.reshape(-1, 3) * 1000
        display_faces = np.arange(len(display_vertices)).reshape(-1, 3)
        dae(mesh_dir / f"{name}.dae", display_vertices, display_faces)
        visual(link, f"{asset_name}/{name}.dae")
        collision_box(link, vertices.min(axis=0), vertices.max(axis=0))
        fixed(
            robot,
            "base" if name == "box" else "mid360_box",
            child,
            mount if name == "box" else seat if name == "lid" else (0, 0, 0),
            (0, pitch, 0) if name == "lid" else (0, 0, 0),
        )

    sensor = ET.SubElement(robot, "link", name="livox_frame")
    visual(sensor, "mid360_ground_mount/mid360_official.dae")
    low, high = np.array([-32.5, -32.5, -47.0]), np.array([32.5, 32.5, 13.0])
    collision_box(sensor, low, high)
    # Box approximation for dynamics, not a measured sensor inertia tensor.
    size = (high - low) / 1000
    inertia = (
        0.265 / 12 * np.diag([size[1] ** 2 + size[2] ** 2, size[0] ** 2 + size[2] ** 2, size[0] ** 2 + size[1] ** 2])
    )
    inertial(sensor, 0.265, (low + high) / 2000, inertia)
    fixed(robot, "mid360_lid", "livox_frame", (0, 0, height))

    model_dir = ROOT / "config/robots/unitree/go2"
    slam = yaml.safe_load((model_dir / "sensors/mid360_ltlio.yaml").read_text(encoding="utf-8"))
    r_il = np.array(slam["r_il"]).reshape(3, 3)
    t_il = np.array(slam["t_il"])
    r_bi = rotation @ r_il.T
    t_bi = optical - r_bi @ t_il
    # The existing package calibration is identity in rotation. Fail visibly if that changes.
    if not np.allclose(r_il, np.eye(3), atol=1e-12):
        raise ValueError("Update livox_imu joint RPY for the changed package calibration")
    ET.SubElement(robot, "link", name="livox_imu")
    fixed(robot, "livox_frame", "livox_imu", -r_il.T @ t_il)
    urdf_path = ASSETS / "urdf" / f"go2_{asset_name}.urdf"
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)

    config = yaml.safe_load((model_dir / "robot.yaml").read_text(encoding="utf-8"))
    config["lidar"].update(dict(zip(("offset_x", "offset_y", "offset_z"), optical.tolist())))
    config["lidar"].update(roll=0.0, pitch=pitch, yaw=0.0)
    config["calibration"]["slam"] = {
        "status": "unverified",
        "reason": "V7 raised-box CAD nominal; install, measure and validate before field use",
    }
    slam["navigation_body_from_imu_rotation"] = r_bi.flatten().tolist()
    slam["navigation_body_from_imu_translation"] = t_bi.tolist()
    for filename, value in (("robot.yaml", config), ("mid360_ltlio.yaml", slam)):
        (candidate / filename).write_text(
            "# V7 nominal candidate, NOT installed or field verified.\n"
            + yaml.safe_dump(value, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

    spec = importlib.util.spec_from_file_location("go2_visual_export", ROOT / "web/scripts/generate-go2-urdf.py")
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    exporter.SOURCE = urdf_path
    ts = exporter.generate().replace("urdf/go2_description.urdf", f"urdf/go2_{asset_name}.urdf")
    script = directory / ("export_mount.py" if directory == HERE else "generate_fast.py")
    ts = ts.replace(
        "python web/scripts/generate-go2-urdf.py",
        f"python {script.resolve().relative_to(ROOT).as_posix()}",
    )
    (candidate / "go2UrdfSpec.ts").write_text(ts, encoding="utf-8")

    corners = np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
    bounds.append(corners / 1000 @ rotation.T + optical)
    all_points = np.concatenate(bounds)
    report = {
        "status": "nominal_not_installed_or_field_verified",
        "body_from_lidar_xyz_m": optical.tolist(),
        "body_from_lidar_rpy_rad": [0, pitch, 0],
        "body_from_imu_rotation": r_bi.flatten().tolist(),
        "body_from_imu_translation": t_bi.tolist(),
        "assembly_bounds_in_body_m": [all_points.min(axis=0).tolist(), all_points.max(axis=0).tolist()],
        "collision_note": "URDF boxes are model approximations; SCAN does not consume these collision links",
        "active_configs_modified": False,
    }
    (candidate / "transforms.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    export()
