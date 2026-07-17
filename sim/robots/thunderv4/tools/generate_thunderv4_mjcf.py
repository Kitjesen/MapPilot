"""Generate a Thunder V4 small-wheel MuJoCo XML from the canonical V4 URDF.

The upstream Thunder V4 asset currently ships a URDF but no validated MJCF.
MuJoCo can compile the URDF, but a direct compile welds the root link to the
world and drops the base inertial from the articulated model.  This generator
keeps the URDF-derived collision/joint tree, wraps it in a free ``base_link``,
adds the fixed sensor masses, and preserves the RobotLab joint naming/order
used by the existing policies and playback scripts.
"""

from __future__ import annotations

import argparse
import copy
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[6]
ASSET_DIR = Path(__file__).resolve().parents[1]
THIRDPART_SOURCE_DIR = ROOT / "thirdpart" / "thunder_assets" / "thunder_v4"
EXTERNAL_SOURCE_DIR = ROOT / ".external" / "thunder_assets" / "thunder_v4"
DEFAULT_SOURCE_DIR = THIRDPART_SOURCE_DIR if THIRDPART_SOURCE_DIR.exists() else EXTERNAL_SOURCE_DIR
DEFAULT_URDF = DEFAULT_SOURCE_DIR / "urdf" / "thunder_v4.urdf"
DEFAULT_SOURCE_MESH_DIR = DEFAULT_SOURCE_DIR / "meshes"
DEFAULT_OUTPUT_XML = ASSET_DIR / "mjcf" / "thunderv4.xml"
DEFAULT_STAIRS_OUTPUT_XML = ASSET_DIR / "mjcf" / "thunderv4_stairs.xml"
DEFAULT_OUTPUT_MESH_DIR = ASSET_DIR / "meshes"
HARDWARE_VELOCITY_NUMERIC = "hardware_joint_velocity_limit_rad_s"


LINK_NAME_MAP = {
    "fr_hip_link": "FR_hip",
    "fr_thigh_Link": "FR_thigh",
    "fr_calf_Link": "FR_calf",
    "fr_foot_Link": "FR_foot",
    "fl_hip_Link": "FL_hip",
    "fl_thigh_Link": "FL_thigh",
    "fl_calf_Link": "FL_calf",
    "fl_foot_Link": "FL_foot",
    "rr_hip_Link": "RR_hip",
    "rr_thigh_Link": "RR_thigh",
    "rr_calf_Link": "RR_calf",
    "rr_foot_Link": "RR_foot",
    "rl_hip_Link": "RL_hip",
    "rl_thigh_Link": "RL_thigh",
    "rl_calf_Link": "RL_calf",
    "rl_foot_Link": "RL_foot",
    "lidar1_Link": "lidar1_link",
    "camera1_Link": "camera1_link",
    "lidar2_Link": "lidar2_link",
    "camera2_Link": "camera2_link",
}


JOINT_NAME_MAP = {
    "fr_hip_joint": "FR_hip_joint",
    "fr_thigh_joint": "FR_thigh_joint",
    "fr_calf_joint": "FR_calf_joint",
    "fr_foot_joint": "FR_foot_joint",
    "fl_hip_joint": "FL_hip_joint",
    "fl_thigh_joint": "FL_thigh_joint",
    "fl_calf_joint": "FL_calf_joint",
    "fl_foot_joint": "FL_foot_joint",
    "rr_hip_joint": "RR_hip_joint",
    "rr_thigh_joint": "RR_thigh_joint",
    "rr_calf_joint": "RR_calf_joint",
    "rr_foot_joint": "RR_foot_joint",
    "rl_hip_joint": "RL_hip_joint",
    "rl_thigh_joint": "RL_thigh_joint",
    "rl_calf_joint": "RL_calf_joint",
    "rl_foot_joint": "RL_foot_joint",
}
SOURCE_JOINT_NAME_MAP = {canonical: source for source, canonical in JOINT_NAME_MAP.items()}


JOINT_ORDER = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FR_foot_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FL_foot_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RR_foot_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RL_foot_joint",
]


ACTUATOR_ORDER = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint",
]


# This is a MuJoCo-validated stand reference for the real V4 small-wheel
# geometry.  It is deliberately not the policy's observation reference: old
# policies were trained against the previous mixed asset and must not have
# their input coordinate frame silently changed.
NOMINAL_STAND_JOINT_POS = {
    "FR_hip_joint": -0.10,
    "FR_thigh_joint": -1.20,
    "FR_calf_joint": 2.70,
    "FR_foot_joint": 0.0,
    "FL_hip_joint": 0.10,
    "FL_thigh_joint": 1.20,
    "FL_calf_joint": -2.70,
    "FL_foot_joint": 0.0,
    "RR_hip_joint": 0.10,
    "RR_thigh_joint": 1.20,
    "RR_calf_joint": -2.70,
    "RR_foot_joint": 0.0,
    "RL_hip_joint": -0.10,
    "RL_thigh_joint": -1.20,
    "RL_calf_joint": 2.70,
    "RL_foot_joint": 0.0,
}


def _float_list(value: str | None, default: str = "0 0 0") -> list[float]:
    return [float(x) for x in (value or default).split()]


def _fmt(values: list[float] | tuple[float, ...]) -> str:
    return " ".join(f"{v:.10g}" for v in values)


def _rpy_to_quat(rpy: str | None) -> str | None:
    roll, pitch, yaw = _float_list(rpy)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quat = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    if max(abs(quat[0] - 1.0), abs(quat[1]), abs(quat[2]), abs(quat[3])) < 1e-10:
        return None
    return _fmt(quat)


def _parse_urdf(urdf_path: Path) -> tuple[ET.Element, dict[str, ET.Element], dict[str, ET.Element]]:
    root = ET.parse(urdf_path).getroot()  # noqa: S314 - canonical local URDF
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    return root, links, joints


def _source_joint_velocity_limits(joints: dict[str, ET.Element]) -> tuple[float, ...]:
    limits = []
    for canonical_name in ACTUATOR_ORDER:
        source_name = SOURCE_JOINT_NAME_MAP[canonical_name]
        limit = joints[source_name].find("limit")
        if limit is None or "velocity" not in limit.attrib:
            raise ValueError(f"V4 URDF joint {source_name} is missing a velocity limit")
        limits.append(float(limit.attrib["velocity"]))
    return tuple(limits)


def _link_inertial(link: ET.Element) -> ET.Element:
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"Link {link.attrib['name']} has no inertial")

    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError(f"Link {link.attrib['name']} has incomplete inertial")

    return ET.Element(
        "inertial",
        {
            "pos": origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
            "mass": mass.attrib["value"],
            "fullinertia": " ".join(inertia.attrib[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
        },
    )


def _fixed_joint_pose(joint: ET.Element) -> dict[str, str]:
    origin = joint.find("origin")
    pose: dict[str, str] = {}
    if origin is not None:
        pose["pos"] = origin.attrib.get("xyz", "0 0 0")
        quat = _rpy_to_quat(origin.attrib.get("rpy"))
        if quat:
            pose["quat"] = quat
    return pose


def _visual_meshes(links: dict[str, ET.Element]) -> tuple[ET.Element, dict[str, list[ET.Element]]]:
    asset = ET.Element("asset")
    ET.SubElement(
        asset,
        "material",
        {"name": "matplane", "reflectance": "0.3", "texture": "texplane", "texrepeat": "1 1", "texuniform": "true"},
    )
    ET.SubElement(asset, "material", {"name": "body_white", "rgba": "0.898039 0.917647 0.929412 1"})
    ET.SubElement(asset, "material", {"name": "wheel_black", "rgba": "0.12 0.12 0.12 1"})
    ET.SubElement(
        asset,
        "texture",
        {
            "name": "texplane",
            "type": "2d",
            "builtin": "checker",
            "rgb1": ".2 .3 .4",
            "rgb2": ".1 .15 .2",
            "width": "512",
            "height": "512",
        },
    )
    ET.SubElement(
        asset,
        "texture",
        {"type": "skybox", "builtin": "gradient", "rgb1": "1 1 1", "rgb2": ".6 .8 1", "width": "256", "height": "256"},
    )

    visual_by_link: dict[str, list[ET.Element]] = {}
    for source_name, link in links.items():
        canonical = LINK_NAME_MAP.get(source_name, source_name)
        for index, visual in enumerate(link.findall("visual")):
            mesh = visual.find("./geometry/mesh")
            if mesh is None:
                continue
            filename = Path(mesh.attrib["filename"]).name
            mesh_name = f"{canonical}_visual" if index == 0 else f"{canonical}_visual_{index}"
            ET.SubElement(asset, "mesh", {"name": mesh_name, "file": filename})

            origin = visual.find("origin")
            attrs = {
                "type": "mesh",
                "mesh": mesh_name,
                "contype": "0",
                "conaffinity": "0",
                "group": "1",
                "density": "0",
                "material": "wheel_black" if "foot" in canonical else "body_white",
            }
            if origin is not None:
                if "xyz" in origin.attrib:
                    attrs["pos"] = origin.attrib["xyz"]
                quat = _rpy_to_quat(origin.attrib.get("rpy"))
                if quat:
                    attrs["quat"] = quat
            visual_by_link.setdefault(canonical, []).append(ET.Element("geom", attrs))
    return asset, visual_by_link


def _compile_raw_urdf(urdf_path: Path) -> ET.Element:
    with tempfile.TemporaryDirectory(prefix="thunderv4_mjcf_") as tmp:
        raw_path = Path(tmp) / "raw.xml"
        model = mujoco.MjModel.from_xml_path(str(urdf_path))
        mujoco.mj_saveLastXML(str(raw_path), model)
        return ET.parse(raw_path).getroot()  # noqa: S314 - MuJoCo-generated XML


def _transparent_rgba(rgba: str | None) -> str:
    values = _float_list(rgba, default="0.8 0.8 0.8 1")
    if len(values) < 3:
        values = [0.8, 0.8, 0.8]
    return _fmt((values[0], values[1], values[2], 0.0))


def _rename_tree(elem: ET.Element, show_collisions: bool) -> None:
    if elem.tag == "body" and "name" in elem.attrib:
        elem.attrib["name"] = LINK_NAME_MAP.get(elem.attrib["name"], elem.attrib["name"])
    if elem.tag == "joint" and "name" in elem.attrib:
        elem.attrib["name"] = JOINT_NAME_MAP.get(elem.attrib["name"], elem.attrib["name"])
        if elem.attrib["name"].endswith("_foot_joint"):
            elem.attrib["type"] = "hinge"
            elem.attrib["limited"] = "false"
            elem.attrib["actuatorfrcrange"] = "-17 17"
            elem.attrib["class"] = "wheel_joint_param"
        else:
            elem.attrib["limited"] = "true"
            elem.attrib["class"] = "leg_joint_param"
    if elem.tag == "geom":
        elem.attrib.setdefault("group", "3")
        elem.attrib.setdefault("contype", "1")
        elem.attrib.setdefault("conaffinity", "15")
        elem.attrib.setdefault("solref", "0.004 1")
        elem.attrib.setdefault("friction", "0.9 0.2 0.2")
        if not show_collisions:
            # MuJoCo continues to use these geoms for contact.  Only their
            # render alpha is zeroed, so the CAD mesh remains the visible body.
            elem.attrib["rgba"] = _transparent_rgba(elem.attrib.get("rgba"))
    for child in list(elem):
        _rename_tree(child, show_collisions=show_collisions)


def _insert_visual_geoms(body: ET.Element, visual_by_link: dict[str, list[ET.Element]]) -> None:
    name = body.attrib.get("name")
    for visual in reversed(visual_by_link.get(name, [])):
        insert_at = 0
        for i, child in enumerate(list(body)):
            if child.tag in {"inertial", "joint", "freejoint"}:
                insert_at = i + 1
        body.insert(insert_at, copy.deepcopy(visual))
    for child in body.findall("body"):
        _insert_visual_geoms(child, visual_by_link)


def _mark_wheel_collisions(body: ET.Element) -> None:
    name = body.attrib.get("name", "")
    if name.endswith("_foot"):
        leg = name.split("_", 1)[0]
        for geom in body.findall("geom"):
            if geom.attrib.get("type") == "cylinder" and geom.attrib.get("contype") != "0":
                geom.attrib["name"] = f"{leg}_wheel"
                geom.attrib["class"] = "rubber_wheel"
    for child in body.findall("body"):
        _mark_wheel_collisions(child)


def _body_by_name(body: ET.Element, name: str) -> ET.Element | None:
    if body.attrib.get("name") == name:
        return body
    for child in body.findall("body"):
        found = _body_by_name(child, name)
        if found is not None:
            return found
    return None


def _add_scene_terrain(worldbody: ET.Element, scene: str) -> None:
    if scene == "flat":
        return
    if scene != "stairs":
        raise ValueError(f"Unsupported scene: {scene}")

    # Keep this terrain intentionally simple.  The robot is identical to the
    # flat scene; only the environment changes, so a stairs playback cannot
    # accidentally fall back to the legacy mixed V3/V4 model.
    for name, size, pos in (
        ("step_1", "0.35 2.0 0.05", "1.0 0 0.05"),
        ("step_2", "0.35 2.0 0.10", "1.7 0 0.10"),
        ("step_3", "0.35 2.0 0.15", "2.4 0 0.15"),
    ):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": name,
                "type": "box",
                "size": size,
                "pos": pos,
                "material": "matplane",
                "condim": "4",
                "conaffinity": "15",
                "friction": "0.9 0.2 0.2",
            },
        )


def _build_model(urdf_path: Path, scene: str = "flat", show_collisions: bool = False) -> ET.ElementTree:
    _, links, joints = _parse_urdf(urdf_path)
    joint_velocity_limits = _source_joint_velocity_limits(joints)
    raw_root = _compile_raw_urdf(urdf_path)
    raw_worldbody = raw_root.find("worldbody")
    if raw_worldbody is None:
        raise ValueError("Raw MuJoCo compile produced no worldbody")

    model_name = "thunderv4" if scene == "flat" else f"thunderv4_{scene}"
    mujoco_root = ET.Element("mujoco", {"model": model_name})
    mujoco_root.append(ET.Comment("Generated from Kitjesen/thunder_assets thunder_v4 URDF; do not hand-edit."))
    ET.SubElement(
        mujoco_root, "compiler", {"angle": "radian", "meshdir": "../meshes", "eulerseq": "zyx", "autolimits": "true"}
    )
    ET.SubElement(
        mujoco_root, "option", {"timestep": "0.001", "iterations": "50", "solver": "PGS", "gravity": "0 0 -9.81"}
    )

    visual = ET.SubElement(mujoco_root, "visual")
    ET.SubElement(visual, "quality", {"shadowsize": "4096"})
    ET.SubElement(visual, "map", {"znear": "0.05"})

    default = ET.SubElement(mujoco_root, "default")
    ET.SubElement(default, "joint", {"limited": "true"})
    ET.SubElement(default, "motor", {"ctrllimited": "true"})
    ET.SubElement(
        default,
        "geom",
        {"condim": "4", "contype": "1", "conaffinity": "15", "solref": "0.004 1", "friction": "0.9 0.2 0.2"},
    )
    leg_default = ET.SubElement(default, "default", {"class": "leg_joint_param"})
    ET.SubElement(leg_default, "joint", {"damping": "0.01", "frictionloss": "0.01", "armature": "0.01"})
    wheel_joint_default = ET.SubElement(default, "default", {"class": "wheel_joint_param"})
    ET.SubElement(wheel_joint_default, "joint", {"damping": "0.02", "frictionloss": "0.01", "armature": "0.01"})
    wheel_default = ET.SubElement(default, "default", {"class": "rubber_wheel"})
    ET.SubElement(
        wheel_default,
        "geom",
        {"condim": "4", "solref": "0.004 1", "friction": "1.0 0.005 0.0001", "rgba": "0.12 0.12 0.12 1"},
    )

    custom = ET.SubElement(mujoco_root, "custom")
    ET.SubElement(
        custom,
        "numeric",
        {
            "name": HARDWARE_VELOCITY_NUMERIC,
            "size": str(len(joint_velocity_limits)),
            "data": _fmt(joint_velocity_limits),
        },
    )

    asset, visual_by_link = _visual_meshes(links)
    mujoco_root.append(asset)

    worldbody = ET.SubElement(mujoco_root, "worldbody")
    ET.SubElement(
        worldbody,
        "light",
        {
            "directional": "true",
            "diffuse": ".4 .4 .4",
            "specular": "0.1 0.1 0.1",
            "pos": "0 0 5",
            "dir": "0 0 -1",
            "castshadow": "false",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {"directional": "true", "diffuse": ".6 .6 .6", "specular": "0.2 0.2 0.2", "pos": "0 0 4", "dir": "0 0 -1"},
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "ground",
            "type": "plane",
            "size": "0 0 1",
            "material": "matplane",
            "condim": "4",
            "conaffinity": "15",
        },
    )
    _add_scene_terrain(worldbody, scene)

    base = ET.SubElement(worldbody, "body", {"name": "base_link", "pos": "0 0 0.55"})
    base.append(_link_inertial(links["base_link"]))
    ET.SubElement(base, "joint", {"name": "floating_base_joint", "type": "free", "limited": "false"})
    ET.SubElement(base, "site", {"name": "imu", "size": "0.01", "pos": "0 0 0", "quat": "1 0 0 0"})
    ET.SubElement(
        base, "camera", {"name": "front_camera", "pos": "0.35 0 0.15", "xyaxes": "0 -1 0 0 0 1", "fovy": "60"}
    )

    for visual_geom in visual_by_link.get("base_link", []):
        base.append(copy.deepcopy(visual_geom))

    for geom in raw_worldbody.findall("geom"):
        base.append(copy.deepcopy(geom))

    for body in raw_worldbody.findall("body"):
        renamed = copy.deepcopy(body)
        _rename_tree(renamed, show_collisions=show_collisions)
        _insert_visual_geoms(renamed, visual_by_link)
        _mark_wheel_collisions(renamed)
        base.append(renamed)

    for fixed_joint_name in ("lidar1_joint", "camera1_joint", "lidar2_joint", "camera2_joint"):
        joint = joints.get(fixed_joint_name)
        if joint is None:
            continue
        source_link = joint.find("child").attrib["link"]
        canonical = LINK_NAME_MAP.get(source_link, source_link)
        fixed_body = ET.SubElement(base, "body", {"name": canonical, **_fixed_joint_pose(joint)})
        fixed_body.append(_link_inertial(links[source_link]))
        for visual_geom in visual_by_link.get(canonical, []):
            fixed_body.append(copy.deepcopy(visual_geom))
        if canonical.startswith("lidar"):
            ET.SubElement(fixed_body, "site", {"name": f"{canonical}_site", "pos": "0 0 0", "size": "0.001"})

    ET.SubElement(base, "site", {"name": "lidar_site", "pos": "-0.30638 0 0.19417", "size": "0.001"})

    actuator = ET.SubElement(mujoco_root, "actuator")
    for joint_name in ACTUATOR_ORDER:
        limit = "17" if joint_name.endswith("_foot_joint") else "120"
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": joint_name,
                "joint": joint_name,
                "gear": "1",
                "ctrllimited": "true",
                "ctrlrange": f"-{limit} {limit}",
            },
        )

    sensor = ET.SubElement(mujoco_root, "sensor")
    ET.SubElement(sensor, "framequat", {"name": "orientation", "objtype": "site", "noise": "0.001", "objname": "imu"})
    ET.SubElement(sensor, "framepos", {"name": "position", "objtype": "site", "noise": "0.001", "objname": "imu"})
    ET.SubElement(sensor, "gyro", {"name": "angular-velocity", "site": "imu", "noise": "0.005", "cutoff": "34.9"})
    ET.SubElement(sensor, "velocimeter", {"name": "linear-velocity", "site": "imu", "noise": "0.001", "cutoff": "30"})
    ET.SubElement(
        sensor, "accelerometer", {"name": "linear-acceleration", "site": "imu", "noise": "0.005", "cutoff": "157"}
    )
    ET.SubElement(
        sensor, "framequat", {"name": "lidar-orientation", "objtype": "site", "noise": "0.001", "objname": "lidar_site"}
    )
    ET.SubElement(
        sensor, "framepos", {"name": "lidar-position", "objtype": "site", "noise": "0.001", "objname": "lidar_site"}
    )
    ET.SubElement(
        sensor, "gyro", {"name": "lidar-angular-velocity", "site": "lidar_site", "noise": "0.005", "cutoff": "34.9"}
    )
    ET.SubElement(
        sensor, "velocimeter", {"name": "lidar-linear-velocity", "site": "lidar_site", "noise": "0.001", "cutoff": "30"}
    )
    ET.SubElement(
        sensor,
        "accelerometer",
        {"name": "lidar-linear-acceleration", "site": "lidar_site", "noise": "0.005", "cutoff": "157"},
    )
    ET.SubElement(sensor, "magnetometer", {"name": "magnetometer", "site": "imu"})

    stand_qpos = [0.0, 0.0, 0.60, 1.0, 0.0, 0.0, 0.0]
    stand_qpos.extend(NOMINAL_STAND_JOINT_POS[joint_name] for joint_name in JOINT_ORDER)
    keyframe = ET.SubElement(mujoco_root, "keyframe")
    ET.SubElement(
        keyframe,
        "key",
        {"name": "v4_nominal_stand", "qpos": _fmt(stand_qpos)},
    )

    return ET.ElementTree(mujoco_root)


def _copy_meshes(source_mesh_dir: Path, output_mesh_dir: Path) -> None:
    output_mesh_dir.mkdir(parents=True, exist_ok=True)
    for mesh in source_mesh_dir.glob("*.STL"):
        shutil.copy2(mesh, output_mesh_dir / mesh.name)


def _validate(
    output_xml: Path,
    scene: str,
    expected_velocity_limits: tuple[float, ...],
    show_collisions: bool,
) -> None:
    model = mujoco.MjModel.from_xml_path(str(output_xml))
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    missing = [name for name in ["floating_base_joint", *JOINT_ORDER] if name not in joint_names]
    if missing:
        raise RuntimeError(f"Generated XML is missing joints: {missing}")
    if actuator_names != ACTUATOR_ORDER:
        raise RuntimeError(f"Unexpected actuator order:\n  got:      {actuator_names}\n  expected: {ACTUATOR_ORDER}")
    mass = float(model.body_mass.sum())
    if abs(mass - 45.8086) > 1e-3:
        raise RuntimeError(f"Unexpected V4 mass {mass:.6f}; expected 45.8086 kg")
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "v4_nominal_stand")
    if key_id < 0:
        raise RuntimeError("Generated XML is missing the v4_nominal_stand keyframe")
    sensor_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id) for sensor_id in range(model.nsensor)
    }
    required_lidar_sensors = {
        "lidar-orientation",
        "lidar-position",
        "lidar-angular-velocity",
        "lidar-linear-velocity",
        "lidar-linear-acceleration",
    }
    missing_lidar_sensors = sorted(required_lidar_sensors - sensor_names)
    if missing_lidar_sensors:
        raise RuntimeError(f"Generated XML is missing lidar IMU sensors: {missing_lidar_sensors}")
    numeric_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, HARDWARE_VELOCITY_NUMERIC)
    if numeric_id < 0:
        raise RuntimeError("Generated XML is missing hardware joint velocity limits")
    numeric_start = model.numeric_adr[numeric_id]
    numeric_size = model.numeric_size[numeric_id]
    velocity_limits = model.numeric_data[numeric_start : numeric_start + numeric_size]
    if not np.allclose(velocity_limits, expected_velocity_limits):
        raise RuntimeError(
            "Generated XML velocity limits do not match the source V4 URDF: "
            f"got {velocity_limits.tolist()}, expected {list(expected_velocity_limits)}"
        )
    if not show_collisions:
        collision_alpha = model.geom_rgba[model.geom_group == 3, 3]
        if collision_alpha.size == 0 or not np.allclose(collision_alpha, 0.0):
            raise RuntimeError("Collision geoms must be hidden in the default render")
    if scene == "stairs":
        missing_steps = [
            name
            for name in ("step_1", "step_2", "step_3")
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) < 0
        ]
        if missing_steps:
            raise RuntimeError(f"Generated stairs XML is missing terrain: {missing_steps}")
    print(f"validated {output_xml}")
    print(f"  nq={model.nq} nv={model.nv} nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom} nu={model.nu}")
    print(f"  total_mass={mass:.6f} kg")
    print("  joints=" + ", ".join(joint_names))
    print("  actuators=" + ", ".join(actuator_names))


def main() -> None:
    """Generate and validate one Thunder V4 MuJoCo scene."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", choices=("flat", "stairs"), default="flat")
    parser.add_argument(
        "--show-collisions",
        action="store_true",
        help="Render collision primitives for geometry debugging; physics is unchanged.",
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--source-mesh-dir", type=Path, default=DEFAULT_SOURCE_MESH_DIR)
    parser.add_argument("--output-xml", type=Path)
    parser.add_argument("--output-mesh-dir", type=Path, default=DEFAULT_OUTPUT_MESH_DIR)
    parser.add_argument("--skip-mesh-copy", action="store_true")
    args = parser.parse_args()

    if not args.urdf.exists():
        raise FileNotFoundError(args.urdf)
    if not args.source_mesh_dir.exists():
        raise FileNotFoundError(args.source_mesh_dir)

    if args.output_xml is None:
        args.output_xml = DEFAULT_OUTPUT_XML if args.scene == "flat" else DEFAULT_STAIRS_OUTPUT_XML

    if not args.skip_mesh_copy:
        _copy_meshes(args.source_mesh_dir, args.output_mesh_dir)

    _, _, source_joints = _parse_urdf(args.urdf)
    expected_velocity_limits = _source_joint_velocity_limits(source_joints)
    tree = _build_model(args.urdf, scene=args.scene, show_collisions=args.show_collisions)
    args.output_xml.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(args.output_xml, encoding="utf-8", xml_declaration=True)
    _validate(
        args.output_xml,
        scene=args.scene,
        expected_velocity_limits=expected_velocity_limits,
        show_collisions=args.show_collisions,
    )


if __name__ == "__main__":
    main()
