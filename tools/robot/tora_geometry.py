"""Recompute Tora's locked-zero-pose collision envelope from its supplied URDF."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

MODEL_DIR = Path(__file__).resolve().parents[2] / "config/robots/paxini/tora_double_one_x"
DEFAULT_URDF = MODEL_DIR / "description/urdf/tora_double_one_x.urdf"
REFERENCE_LINK = "base_imu_link_0"
HORIZONTAL_MARGIN_M = 0.05


def origin(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    roll, pitch, yaw = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    if not np.isfinite(transform).all():
        raise ValueError("URDF origin must contain finite values")
    return transform


def load_collision_meshes(urdf: Path = DEFAULT_URDF, reference: str = REFERENCE_LINK) -> tuple[dict, dict]:
    """Return every collision vertex in the selected link frame at zero joint angles."""
    urdf = Path(urdf)
    robot = ET.parse(urdf).getroot()
    package_root = urdf.parent.parent
    package = ET.parse(package_root / "package.xml").getroot()
    package_name = package.findtext("name")
    mesh_paths = {}
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        prefix = f"package://{package_name}/"
        if not filename.startswith(prefix):
            raise ValueError(f"Unsupported mesh reference {filename!r}")
        path = package_root / filename.removeprefix(prefix)
        if not path.is_file():
            raise FileNotFoundError(f"URDF mesh is missing: {path}")
        mesh_paths[filename] = path

    links = {link.get("name"): link for link in robot.findall("link")}
    joints = robot.findall("joint")
    children = [joint.find("child").get("link") for joint in joints]
    roots = links.keys() - set(children)
    if len(roots) != 1 or len(children) != len(set(children)):
        raise ValueError("URDF must have one root and one parent per child")
    root = roots.pop()
    transforms = {root: np.eye(4)}
    pending = list(joints)
    while pending:
        remaining = []
        for joint in pending:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in links or child not in links:
                raise ValueError(f"Unknown link in joint {joint.get('name')}")
            if joint.get("type") not in {"fixed", "revolute", "continuous"} or joint.find("mimic") is not None:
                raise ValueError(f"Unsupported joint {joint.get('name')}")
            if joint.get("type") == "revolute":
                limit = joint.find("limit")
                if limit is None or not float(limit.get("lower")) <= 0 <= float(limit.get("upper")):
                    raise ValueError(f"Zero pose violates joint {joint.get('name')}")
            if parent not in transforms:
                remaining.append(joint)
                continue
            transforms[child] = transforms[parent] @ origin(joint.find("origin"))
        if len(remaining) == len(pending):
            raise ValueError("URDF joint tree is disconnected or cyclic")
        pending = remaining
    if transforms.keys() != links.keys() or reference not in transforms:
        raise ValueError("URDF contains disconnected links or lacks the reference link")

    reference_from_root = np.linalg.inv(transforms[reference])
    meshes = {}
    triangle_count = collision_count = 0
    triangle_dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    for name, link in links.items():
        batches = []
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) != 1 or geometry[0].tag != "mesh":
                raise ValueError(f"Unsupported collision geometry on {name}")
            mesh = geometry[0]
            raw = mesh_paths[mesh.get("filename")].read_bytes()
            count = int.from_bytes(raw[80:84], "little")
            if len(raw) != 84 + count * 50 or count == 0:
                raise ValueError(f"Invalid binary STL on {name}")
            vertices = np.frombuffer(raw, dtype=triangle_dtype, offset=84)["vertices"].reshape(-1, 3)
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                raise ValueError(f"Invalid mesh scale on {name}")
            transform = reference_from_root @ transforms[name] @ origin(collision.find("origin"))
            vertices = (vertices * scale) @ transform[:3, :3].T + transform[:3, 3]
            if not np.isfinite(vertices).all():
                raise ValueError(f"Nonfinite collision vertices on {name}")
            batches.append(vertices)
            triangle_count += count
            collision_count += 1
        if batches:
            meshes[name] = np.concatenate(batches)
    if not meshes:
        raise ValueError("URDF contains no collision meshes")
    metadata = {
        "model": robot.get("name"),
        "source_package": package_name,
        "source_version": package.findtext("version"),
        "license": package.findtext("license"),
        "urdf": "description/urdf/" + urdf.name,
        "root_link": root,
        "reference_link": reference,
        "root_from_reference": transforms[reference].tolist(),
        "pose": "all joints zero; waist and arms locked during navigation",
        "link_count": len(links),
        "joint_count": len(joints),
        "collision_mesh_count": collision_count,
        "triangle_count": triangle_count,
        "field_calibrated": False,
    }
    return meshes, metadata


def envelope_report(meshes: dict, metadata: dict) -> dict:
    bounds = {name: {"minimum_m": points.min(axis=0).tolist(), "maximum_m": points.max(axis=0).tolist()}
              for name, points in meshes.items()}
    minimum = np.min([value["minimum_m"] for value in bounds.values()], axis=0)
    maximum = np.max([value["maximum_m"] for value in bounds.values()], axis=0)
    radius = max(float(np.linalg.norm(points[:, :2], axis=1).max()) for points in meshes.values())
    return {
        **metadata,
        "bounds_m": {"minimum": minimum.tolist(), "maximum": maximum.tolist()},
        "dimensions_m": (maximum - minimum).tolist(),
        "raw_radius_m": radius,
        "horizontal_margin_m": HORIZONTAL_MARGIN_M,
        "cylinder_radius_m": math.ceil((radius + HORIZONTAL_MARGIN_M) * 100) / 100,
        "cylinder_offset_m": 0.0,
        "full_mesh_clearance_below_m": math.ceil(-minimum[2] * 100) / 100,
        "full_mesh_clearance_above_m": math.ceil(maximum[2] * 100) / 100,
        "nonwheel_minimum_z_m": min(value["minimum_m"][2] for name, value in bounds.items() if "wheel" not in name),
        "ground_contact_scope": "full mesh envelope includes wheel contact; runtime ground clearance needs qualification",
        "links": bounds,
    }


def save_preview(meshes: dict, report: dict, geometry: dict, output: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    figure, axes = plt.subplots(1, 3, figsize=(13, 6), constrained_layout=True)
    radius = geometry["collision_cylinder_radius"]
    bottom, top = -geometry["collision_clearance_below"], geometry["collision_clearance_above"]
    for axis, (horizontal, vertical, title) in zip(axes, [(0, 1, "Top / XY"), (0, 2, "Side / XZ"), (1, 2, "Front / YZ")]):
        for name, points in meshes.items():
            sample = points[::max(1, len(points) // 1400)]
            axis.scatter(sample[:, horizontal], sample[:, vertical], s=0.35,
                         color="#526b85" if "wheel" not in name else "#253341", rasterized=True)
        if vertical == 1:
            axis.add_patch(Circle((0, 0), radius, fill=False, edgecolor="#d57b25", linewidth=2))
        else:
            axis.add_patch(Rectangle((-radius, bottom), 2 * radius, top - bottom,
                                    fill=False, edgecolor="#d57b25", linewidth=2))
            ground = -report["root_from_reference"][2][3]
            axis.axhline(ground, color="#7d9678", linestyle="--", linewidth=1)
        axis.scatter([0], [0], s=20, marker="+", color="#b83e3e")
        axis.set(title=title, xlabel=f'{"XYZ"[horizontal]} (m)', ylabel=f'{"XYZ"[vertical]} (m)')
        axis.set_aspect("equal")
        axis.grid(alpha=0.15)
        axis.margins(0.1)
    figure.suptitle(f"Tora Double One X / locked URDF zero pose\n{radius:.2f} m cylinder radius / "
                   f"{HORIZONTAL_MARGIN_M:.2f} m XY allowance / base IMU reference")
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--profile", type=Path, default=MODEL_DIR / "navigation.yaml")
    parser.add_argument("--output-dir", type=Path, default=MODEL_DIR)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    meshes, metadata = load_collision_meshes(args.urdf, profile["reference_frame"])
    report = envelope_report(meshes, metadata)
    report["navigation_geometry"] = profile["geometry"]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "envelope.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    save_preview(meshes, report, profile["geometry"], args.output_dir / "preview.png")
    print(json.dumps({key: report[key] for key in ("collision_mesh_count", "triangle_count", "bounds_m", "cylinder_radius_m", "nonwheel_minimum_z_m")}))


if __name__ == "__main__":
    main()
