"""Check V7 CAD-to-URDF-to-SLAM nominal frame consistency, without hardware claims."""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
DESIGN = ROOT / "tools/robot/go2_mid360_ground_mount/raised_box"
ASSETS = ROOT / "web/public/assets/robots/go2"
MODEL = ROOT / "config/robots/unitree/go2"


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def transform(origin):
    result = np.eye(4)
    if origin is None:
        return result
    roll, pitch, yaw = map(float, origin.get("rpy", "0 0 0").split())
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
    return result


def chain(robot, child):
    if child == "base":
        return np.eye(4)
    joints = [j for j in robot.findall("joint") if j.find("child").get("link") == child]
    assert len(joints) == 1
    joint = joints[0]
    return chain(robot, joint.find("parent").get("link")) @ transform(joint.find("origin"))


@pytest.fixture(params=[("", "mid360_raised_box"), ("fast", "mid360_raised_frame")])
def variant(request):
    directory, asset_name = request.param
    return DESIGN / directory, asset_name


def candidate_urdf(variant):
    return ET.parse(ASSETS / "urdf" / f"go2_{variant[1]}.urdf").getroot()


def structural(element):
    return element.tag, element.attrib, (element.text or "").strip(), [structural(c) for c in element]


def test_original_robot_links_and_joints_are_preserved(variant):
    original = ET.parse(ASSETS / "urdf/go2_description.urdf").getroot()
    candidate = candidate_urdf(variant)
    for tag in ("link", "joint"):
        elements = {e.get("name"): e for e in candidate.findall(tag)}
        assert len(elements) == len(candidate.findall(tag))
        for element in original.findall(tag):
            assert structural(elements[element.get("name")]) == structural(element)


def test_nominal_optical_and_imu_chains_match_slam_and_robot_config(variant):
    design, _ = variant
    p = json.loads((design / "parameters.json").read_text())
    candidate = candidate_urdf(variant)
    lidar = load_yaml(design / "candidate/robot.yaml")["lidar"]
    slam = load_yaml(design / "candidate/mid360_ltlio.yaml")
    body_lidar = chain(candidate, "livox_frame")
    assert body_lidar[:3, 3] == pytest.approx(p["optical_xyz_m"], abs=1e-10)
    expected = transform(
        ET.Element(
            "origin",
            xyz=" ".join(str(lidar[f"offset_{a}"]) for a in "xyz"),
            rpy=f"{lidar['roll']} {lidar['pitch']} {lidar['yaw']}",
        )
    )
    assert body_lidar == pytest.approx(expected, abs=1e-10)
    body_imu = np.eye(4)
    body_imu[:3, :3] = np.array(slam["navigation_body_from_imu_rotation"]).reshape(3, 3)
    body_imu[:3, 3] = slam["navigation_body_from_imu_translation"]
    imu_lidar = np.eye(4)
    imu_lidar[:3, :3] = np.array(slam["r_il"]).reshape(3, 3)
    imu_lidar[:3, 3] = slam["t_il"]
    assert chain(candidate, "livox_imu") == pytest.approx(body_imu, abs=1e-10)
    assert body_imu @ imu_lidar == pytest.approx(body_lidar, abs=1e-10)


def test_candidate_changes_only_mount_calibration_and_clearance(variant):
    design, _ = variant
    original = load_yaml(MODEL / "robot.yaml")
    candidate = load_yaml(design / "candidate/robot.yaml")
    assert candidate["calibration"]["slam"]["status"] == "unverified"
    candidate["calibration"]["slam"] = original["calibration"]["slam"]
    for key in ("offset_x", "offset_y", "offset_z", "roll", "pitch", "yaw"):
        candidate["lidar"][key] = original["lidar"][key]
    # These pre-installation snapshots predate the installed mount's taller envelope.
    candidate["geometry"]["collision_clearance_above"] = original["geometry"]["collision_clearance_above"]
    assert candidate == original
    original_slam = load_yaml(MODEL / "sensors/mid360_ltlio.yaml")
    candidate_slam = load_yaml(design / "candidate/mid360_ltlio.yaml")
    for key in ("navigation_body_from_imu_rotation", "navigation_body_from_imu_translation"):
        candidate_slam[key] = original_slam[key]
    assert candidate_slam == original_slam


def test_visual_resources_exist_and_print_orientation_does_not_leak_into_urdf(variant):
    design, asset_name = variant
    robot = candidate_urdf(variant)
    for mesh in robot.findall("link/visual/geometry/mesh"):
        assert (ASSETS / mesh.get("filename").removeprefix("package://go2_description/")).is_file()
    mount = np.array(json.loads((design / "parameters.json").read_text())["mount_xyz_m"])
    for name in ("box", "lid", "left_pad", "right_pad"):
        namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        dae = ET.parse(ASSETS / f"meshes/{asset_name}/{name}.dae")
        assert dae.find("c:asset/c:unit", namespace).get("meter") == "1"
        positions = np.fromstring(dae.find(".//c:float_array", namespace).text, sep=" ").reshape(-1, 3)
        pose = chain(robot, f"mid360_{name}")
        world = positions @ pose[:3, :3].T + pose[:3, 3]
        original = np.load(design / f"{name}.npz")["vertices"] / 1000 + mount
        assert world.min(axis=0) == pytest.approx(original.min(axis=0), abs=0.0016)
        assert world.max(axis=0) == pytest.approx(original.max(axis=0), abs=0.0016)
