"""Explicit static MJCF geometry is terrain, regardless of its names."""

import base64
from pathlib import Path

import mujoco
import numpy as np
import pytest
from sim.compat.engine.core.robot import RobotConfig
from sim.compat.engine.core.sensor import LidarConfig
from sim.compat.engine.mujoco.engine import MuJoCoEngine


@pytest.fixture
def robot_xml(tmp_path: Path) -> Path:
    path = tmp_path / "robot.xml"
    path.write_text(
        """<mujoco model="bare_robot">
          <compiler angle="radian" eulerseq="zyx"/>
          <option timestep=".004" gravity="0 0 -9.7"/>
          <worldbody>
            <body name="base_link" pos="10 0 1">
              <freejoint name="root"/>
              <geom name="shell" type="box" size=".2 .1 .1"/>
              <site name="lidar_site"/>
              <body name="tool">
                <joint name="tool_joint"/>
                <geom name="tool_shape" type="sphere" size=".05"/>
              </body>
            </body>
          </worldbody>
          <actuator><motor name="tool_motor" joint="tool_joint"/></actuator>
          <keyframe><key name="ready" qpos="10 0 1 1 0 0 0 .3"/></keyframe>
        </mujoco>""",
        encoding="utf-8",
    )
    return path


def _engine(robot_xml: Path) -> MuJoCoEngine:
    return MuJoCoEngine(
        robot_config=RobotConfig(robot_xml=str(robot_xml), leg_joint_names=["tool_joint"]),
        lidar_config=LidarConfig(body_name="base_link", samples_per_frame=8),
        drive_mode="kinematic",
    )


def _ray_distance(model: mujoco.MjModel, data: mujoco.MjData, x: float, y: float) -> float:
    return float(mujoco.mj_ray(
        model, data, np.array([x, y, 5.0]), np.array([0.0, 0.0, -1.0]),
        None, True, -1, np.array([-1], dtype=np.int32),
    ))


@pytest.mark.parametrize("nested", [False, True])
def test_unnamed_terrain_preserves_hole_elevations_and_single_robot(
    tmp_path: Path, robot_xml: Path, nested: bool,
) -> None:
    world = tmp_path / "world.xml"
    opening = '<body name="site" pos="0 0 0"><body name="structure">' if nested else ""
    closing = "</body></body>" if nested else ""
    world.write_text(
        f"""<mujoco><compiler angle="degree"/>
          <worldbody>{opening}
            <geom name="source_geom_0" type="box" pos="-2 0 -.1" size="1 2 .1"/>
            <geom name="source_geom_1" type="box" pos="3 0 .4" size="1 2 .1"/>
            <geom name="source_geom_2" type="box" pos="0 2 .1" size="1 1 .1"
                  euler="0 0 15"/>
          {closing}</worldbody>
        </mujoco>""",
        encoding="utf-8",
    )
    original = mujoco.MjModel.from_xml_path(str(world))
    original_data = mujoco.MjData(original)
    mujoco.mj_forward(original, original_data)
    engine = _engine(robot_xml)
    try:
        engine.load(str(world))
        model, data = engine.model, engine.data
        mujoco.mj_forward(model, data)
        assert np.count_nonzero(model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE) == 0
        assert model.ngeom == original.ngeom + 2
        assert model.nq == 8
        assert model.nu == 1
        assert np.count_nonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE) == 1
        for index in range(original.ngeom):
            name = mujoco.mj_id2name(original, mujoco.mjtObj.mjOBJ_GEOM, index)
            merged_id = model.geom(name).id
            for field in ("geom_type", "geom_size", "geom_contype", "geom_conaffinity", "geom_friction"):
                np.testing.assert_allclose(getattr(model, field)[merged_id], getattr(original, field)[index])
            np.testing.assert_allclose(data.geom_xpos[merged_id], original_data.geom_xpos[index])
            np.testing.assert_allclose(data.geom_xmat[merged_id], original_data.geom_xmat[index], atol=1e-12)
        for x, y, expected in [(-2, 0, 5.0), (3, 0, 4.5), (0, 2, 4.8), (0, 0, -1.0)]:
            assert _ray_distance(model, data, x, y) == pytest.approx(expected)
        assert model.opt.timestep == pytest.approx(.004)
        np.testing.assert_allclose(model.opt.gravity, [0, 0, -9.7])
        np.testing.assert_allclose(model.key("ready").qpos, [10, 0, 1, 1, 0, 0, 0, .3])
    finally:
        engine.close()


@pytest.mark.parametrize("root_joint", ['<freejoint name="root"/>', '<joint name="root" type="free"/>'])
@pytest.mark.parametrize("shape_name", ["shell", "floor_sensor_cover"])
def test_bare_robot_keeps_legacy_auto_floor_despite_fixed_children(
    robot_xml: Path, root_joint: str, shape_name: str,
) -> None:
    robot_xml.write_text(
        robot_xml.read_text(encoding="utf-8")
        .replace('<freejoint name="root"/>', root_joint)
        .replace('name="shell"', f'name="{shape_name}"')
        .replace('<site name="lidar_site"/>', '''<site name="lidar_site"/>
          <body name="fixed_cover"><geom name="cover" type="sphere" size=".02"/></body>'''),
        encoding="utf-8",
    )
    engine = _engine(robot_xml)
    try:
        engine.load(str(robot_xml))
        model = engine.model
        mujoco.mj_forward(model, engine.data)
        assert model.ngeom == 4
        assert np.count_nonzero(model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE) == 1
        assert model.geom("floor").type == mujoco.mjtGeom.mjGEOM_PLANE
        assert _ray_distance(model, engine.data, 0, 0) == pytest.approx(5.0)
        assert model.nu == 1
        assert model.nq == 8
    finally:
        engine.close()


def test_complete_scene_with_actuator_whitespace_is_loaded_without_merging(
    tmp_path: Path, robot_xml: Path,
) -> None:
    scene = tmp_path / "complete.xml"
    scene.write_text(
        robot_xml.read_text(encoding="utf-8")
        .replace('<actuator>', '<actuator >')
        .replace('<worldbody>', '<worldbody><geom name="source_geom_0" type="box" size="2 2 .1"/>'),
        encoding="utf-8",
    )
    # Use a missing configured robot to prove a complete scene is self-contained.
    engine = _engine(tmp_path / "not_used.xml")
    try:
        engine.load(str(scene))
        assert engine.model.ngeom == 3
        assert engine.model.nu == 1
        assert engine.model.nq == 8
        assert np.count_nonzero(engine.model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE) == 0
        np.testing.assert_allclose(engine.model.key("ready").qpos, [10, 0, 1, 1, 0, 0, 0, .3])
    finally:
        engine.close()


@pytest.mark.parametrize("compiler_dirs", [False, True])
def test_explicit_static_world_keeps_relative_mesh_and_texture_paths(
    tmp_path: Path, robot_xml: Path, compiler_dirs: bool,
) -> None:
    physics, visual = tmp_path / "physics", tmp_path / "visual"
    physics.mkdir()
    visual.mkdir()
    (visual / "part.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
        "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n", encoding="utf-8",
    )
    (visual / "paint.png").write_bytes(base64.b64decode(
        "iVBORw0KGgoAAAANSUhEUgAAAAIAAAACCAIAAAD91JpzAAAAEElEQVR4nGNoONAARAwQCgAxDgcBOLkn1wAAAABJRU5ErkJggg=="
    ))
    dirs = 'meshdir="../visual" texturedir="../visual"' if compiler_dirs else ""
    prefix = "" if compiler_dirs else "../visual/"
    world = physics / "world.xml"
    world.write_text(f"""<mujoco><compiler {dirs}/>
      <asset>
        <mesh name="part" file="{prefix}part.obj"/>
        <texture name="paint" type="2d" file="{prefix}paint.png"/>
        <material name="finish" texture="paint"/>
      </asset>
      <worldbody><body name="fixture">
        <geom name="source_geom_0" type="mesh" mesh="part" material="finish"/>
      </body></worldbody>
    </mujoco>""", encoding="utf-8")
    engine = _engine(robot_xml)
    try:
        engine.load(str(world))
        assert engine.model.nmesh == 1
        assert engine.model.ntex == 1
        assert engine.model.ngeom == 3
        assert engine.model.geom("source_geom_0").type == mujoco.mjtGeom.mjGEOM_MESH
        assert np.count_nonzero(engine.model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE) == 0
    finally:
        engine.close()
