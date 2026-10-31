"""Verify the installed Go2 mount's frame chain and physical model resources."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest
from test_go2_raised_box import ASSETS, MODEL, chain, load_yaml, structural, transform


def test_installed_frames_and_sensor_internal_calibration_agree():
    robot = ET.parse(ASSETS / "urdf/go2_mid360_flatpack.urdf").getroot()
    config = load_yaml(MODEL / "robot.yaml")
    lidar = config["lidar"]
    slam = load_yaml(MODEL / "sensors/mid360_ltlio.yaml")
    pose = transform(ET.Element("origin", xyz=" ".join(str(lidar[f"offset_{a}"]) for a in "xyz"),
                                rpy=f"{lidar['roll']} {lidar['pitch']} {lidar['yaw']}"))
    body_imu = np.eye(4)
    body_imu[:3, :3] = np.array(slam["navigation_body_from_imu_rotation"]).reshape(3, 3)
    body_imu[:3, 3] = slam["navigation_body_from_imu_translation"]
    imu_lidar = np.eye(4)
    imu_lidar[:3, :3] = np.array(slam["r_il"]).reshape(3, 3)
    imu_lidar[:3, 3] = slam["t_il"]
    assert chain(robot, "livox_frame") == pytest.approx(pose, abs=1e-10)
    assert chain(robot, "livox_imu") == pytest.approx(body_imu, abs=1e-10)
    assert body_imu @ imu_lidar == pytest.approx(pose, abs=1e-10)
    assert imu_lidar[:3, 3] == pytest.approx([-0.011, -0.02329, 0.04412])
    sensor_top = max((pose @ [x, y, z, 1])[2] for x in [-.0325, .0325]
                     for y in [-.0325, .0325] for z in [-.047, .013])
    assert sensor_top <= config["geometry"]["collision_clearance_above"]


def test_six_parts_preserve_robot_and_have_metre_meshes():
    robot = ET.parse(ASSETS / "urdf/go2_mid360_flatpack.urdf").getroot()
    original = ET.parse(ASSETS / "urdf/go2_description.urdf").getroot()
    for tag in ("link", "joint"):
        elements = {e.get("name"): e for e in robot.findall(tag)}
        assert len(elements) == len(robot.findall(tag))
        for element in original.findall(tag):
            assert structural(elements[element.get("name")]) == structural(element)
    for mesh in robot.findall("link/visual/geometry/mesh"):
        path = ASSETS / mesh.get("filename").removeprefix("package://go2_description/")
        assert path.is_file()
    for name in ("base", "left_side", "right_side", "lid", "left_foot", "right_foot"):
        path = ASSETS / "meshes/mid360_flatpack" / f"{name}.dae"
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        dae = ET.parse(path)
        assert dae.find("c:asset/c:unit", ns).get("meter") == "1"
        pose = chain(robot, f"mid360_{name}")
        assert pose[:3, 3] == pytest.approx([.19, 0, .101])
