from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from drivers.sim.mujoco.runtime import build_engine, focus_presentation_viewer
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


def test_presentation_viewer_renders_robot_and_obstacle_pixels(tmp_path: Path) -> None:
    scene = tmp_path / "obstacle_stop.xml"
    scene.write_text(
        """
<mujoco model="visibility_test">
  <worldbody>
    <geom name="floor" type="box" pos="1.5 0 -0.025" size="2.5 0.9 0.025"/>
    <geom name="left_rail" type="box" pos="1.5 0.9 0.4" size="2.5 0.05 0.4"/>
    <geom name="right_rail" type="box" pos="1.5 -0.9 0.4" size="2.5 0.05 0.4"/>
    <geom name="acceptance_obstacle_stop" type="box" pos="0.49 0 0.4"
          size="0.04 0.25 0.4" contype="0" conaffinity="0"
          rgba="0.95 0.1 0.1 1"/>
  </worldbody>
</mujoco>
""".strip(),
        encoding="utf-8",
    )
    engine = build_engine(
        world=scene,
        drive_mode="kinematic",
        start=[0.0, 0.0, 0.48],
        mujoco_memory="64M",
        mid360_pattern=None,
        mid360_samples_per_frame=32,
        require_product_lidar_backend=False,
    )
    renderer = None
    try:
        base_body = mujoco.mj_name2id(
            engine.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "base_link",
        )
        obstacle_geom = mujoco.mj_name2id(
            engine.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "acceptance_obstacle_stop",
        )

        def belongs_to_robot(body_id: int) -> bool:
            while body_id > 0:
                if body_id == base_body:
                    return True
                body_id = int(engine.model.body_parentid[body_id])
            return False

        robot_geoms = [
            geom_id for geom_id in range(engine.model.ngeom) if belongs_to_robot(int(engine.model.geom_bodyid[geom_id]))
        ]
        viewer = SimpleNamespace(cam=mujoco.MjvCamera(), opt=mujoco.MjvOption())
        focus_presentation_viewer(
            viewer,
            engine.get_robot_state().position,
            initialize=True,
        )
        renderer = mujoco.Renderer(engine.model, height=360, width=640)
        renderer.update_scene(
            engine.data,
            camera=viewer.cam,
            scene_option=viewer.opt,
        )
        renderer.enable_segmentation_rendering()
        segmentation = renderer.render()
        renderer.disable_segmentation_rendering()
        object_ids = segmentation[..., 0]
        object_types = segmentation[..., 1]
        geom_pixels = object_types == int(mujoco.mjtObj.mjOBJ_GEOM)
        robot_pixels = int((geom_pixels & np.isin(object_ids, robot_geoms)).sum())
        obstacle_pixels = int((geom_pixels & (object_ids == obstacle_geom)).sum())

        assert robot_pixels > 3_000
        assert obstacle_pixels > 200
    finally:
        if renderer is not None:
            renderer.close()
        engine.close()
