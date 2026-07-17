from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from sim.engine.mujoco.engine import _freeze_scene_euler_orientations


def test_scene_euler_is_preserved_across_robot_compiler_sequence() -> None:
    scene = ET.fromstring(
        """
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <geom name="rail" type="box" size="0.1 1.6 0.85"
                  pos="5.2 6.5 0.85" euler="0 0 0.55"/>
          </worldbody>
        </mujoco>
        """
    )

    _freeze_scene_euler_orientations(scene, mujoco)

    geom = scene.find("./worldbody/geom")
    assert geom is not None
    assert "euler" not in geom.attrib
    quaternion = np.fromstring(geom.attrib["quat"], sep=" ")
    np.testing.assert_allclose(
        quaternion,
        [math.cos(0.55 / 2.0), 0.0, 0.0, math.sin(0.55 / 2.0)],
        atol=1e-12,
    )

    merged = f"""
    <mujoco>
      <compiler angle="radian" eulerseq="zyx"/>
      <worldbody>{ET.tostring(geom, encoding="unicode")}</worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(merged)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rotation = data.geom_xmat[0].reshape(3, 3)

    np.testing.assert_allclose(
        rotation[:2, :2],
        [[math.cos(0.55), -math.sin(0.55)], [math.sin(0.55), math.cos(0.55)]],
        atol=1e-12,
    )
    np.testing.assert_allclose(rotation[2], [0.0, 0.0, 1.0], atol=1e-12)


def test_scene_degree_euler_uses_scene_compiler_unit() -> None:
    scene = ET.fromstring(
        """
        <mujoco>
          <compiler angle="degree"/>
          <worldbody><body name="fixture" euler="0 0 90"/></worldbody>
        </mujoco>
        """
    )

    _freeze_scene_euler_orientations(scene, mujoco)

    body = scene.find("./worldbody/body")
    assert body is not None
    quaternion = np.fromstring(body.attrib["quat"], sep=" ")
    np.testing.assert_allclose(
        quaternion,
        [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
        atol=1e-12,
    )
