"""World files keep their resource base when the formal runtime adds a robot."""

import base64
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest
from sim.scripts.mujoco import formal_feeder

from drivers.sim.mujoco.runtime import build_engine


@pytest.mark.parametrize("compiler_dirs", [False, True])
def test_formal_engine_loads_world_relative_mesh_and_texture(tmp_path, monkeypatch, compiler_dirs):
    physics = tmp_path / "physics"
    assets = tmp_path / "visual"
    physics.mkdir()
    assets.mkdir()
    (assets / "part.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
        "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n", encoding="utf-8"
    )
    (assets / "paint.png").write_bytes(base64.b64decode(
        "iVBORw0KGgoAAAANSUhEUgAAAAIAAAACCAIAAAD91JpzAAAAEElEQVR4nGNoONAARAwQCgAxDgcBOLkn1wAAAABJRU5ErkJggg=="
    ))
    compiler = 'meshdir="../visual" texturedir="../visual"' if compiler_dirs else ''
    prefix = '' if compiler_dirs else '../visual/'
    scene = physics / "scene.xml"
    scene.write_text(f'''<mujoco>
      <compiler {compiler}/>
      <asset>
        <mesh name="world_part" file="{prefix}part.obj"/>
        <texture name="world_paint" type="2d" file="{prefix}paint.png"/>
        <material name="world_material" texture="world_paint"/>
      </asset>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 .1"/>
        <geom name="world_mesh" type="mesh" mesh="world_part" material="world_material"
              pos="3 0 0" group="2" contype="0" conaffinity="0"/>
      </worldbody>
    </mujoco>''', encoding="utf-8")
    # Exercise the actual snapshot + merge chain, not only direct scene loading.
    (tmp_path / "policy.bin").write_bytes(b"unused-in-kinematic-test")
    robot_package = Path(__file__).resolve().parents[2] / "sim/packages/robots/doso/thunder_v4"
    monkeypatch.setattr(formal_feeder, "_REPOSITORY_ROOT", tmp_path)
    monkeypatch.setattr(formal_feeder._Services, "resolve_directory", staticmethod(lambda _: robot_package))
    session = tmp_path / "session"
    session.mkdir()
    config = SimpleNamespace(world="physics/scene.xml", robot=SimpleNamespace(
        package_root="robot", model="robot/mjcf/thunderv4.xml", policy="policy.bin"
    ))
    world, robot, _ = formal_feeder._Services.snapshot_artifacts(session, config)
    (assets / "part.obj").unlink()
    (assets / "paint.png").unlink()
    engine = build_engine(
        world=world, robot_xml=robot, drive_mode="kinematic", start=[0, 0, .5],
        mujoco_memory="64M", mid360_pattern=None, mid360_samples_per_frame=32,
        require_product_lidar_backend=False,
    )
    try:
        assert mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_MESH, "world_part") >= 0
        assert mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_TEXTURE, "world_paint") >= 0
        assert mujoco.mj_name2id(engine.model, mujoco.mjtObj.mjOBJ_BODY, "base_link") > 0
    finally:
        engine.close()


@pytest.mark.parametrize("compiler_dirs", [False, True])
def test_headless_snapshot_skips_unused_static_visual_assets_preserving_physics(
    tmp_path, monkeypatch, compiler_dirs,
):
    physics = tmp_path / "physics"
    assets = tmp_path / "visual"
    robot = tmp_path / "robot"
    for directory in (physics, assets, robot):
        directory.mkdir()
    (robot / "robot.xml").write_text("<mujoco/>", encoding="utf-8")
    (tmp_path / "policy.bin").write_bytes(b"policy")
    for name in ("unused", "shared"):
        (assets / f"{name}.obj").write_text(
            "v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\n"
            "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n", encoding="utf-8",
        )
        (assets / f"{name}.png").write_bytes(base64.b64decode(
            "iVBORw0KGgoAAAANSUhEUgAAAAIAAAACCAIAAAD91JpzAAAAEElEQVR4nGNoONAARAwQCgAxDgcBOLkn1wAAAABJRU5ErkJggg=="
        ))
    dirs = 'meshdir="../visual" texturedir="../visual"' if compiler_dirs else ""
    prefix = "" if compiler_dirs else "../visual/"
    world_source = physics / "scene.xml"
    world_source.write_text(f'''<mujoco>
      <compiler {dirs}/><option timestep=".002"/>
      <asset>
        <mesh name="unused" file="{prefix}unused.obj"/>
        <texture name="unused" type="2d" file="{prefix}unused.png"/>
        <material name="unused" texture="unused"/>
        <mesh name="shared" file="{prefix}shared.obj"/>
        <texture name="shared" type="2d" file="{prefix}shared.png"/>
        <material name="shared" texture="shared"/>
      </asset>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 .1" group="4"/>
        <geom name="tread" type="box" pos="3 0 .2" size=".4 .4 .2" group="4"/>
        <geom name="visual" type="mesh" mesh="unused" material="unused"
              group="2" contype="0" conaffinity="0"/>
        <geom name="shared_visual" type="mesh" mesh="shared" material="shared"
              group="2" contype="0" conaffinity="0"/>
        <geom name="referenced" type="sphere" size=".1" pos="5 0 .1"
              group="2" contype="0" conaffinity="0"/>
        <geom name="lidar_visible" type="box" size=".1 .1 .1" pos="2 0 .1"
              contype="0" conaffinity="0"/>
        <geom name="group2_contact" type="box" size=".1 .1 .1" pos="-2 0 .1"
              group="2" contype="1" conaffinity="1"/>
        <body name="robot" pos="0 0 .19"><freejoint/>
          <geom name="robot_collision" type="sphere" size=".2"/>
          <geom name="dynamic_visual" type="mesh" mesh="shared" material="shared"
                group="2" contype="0" conaffinity="0" density="1000"/>
        </body>
        <body name="platform" mocap="true" pos="4 0 .4">
          <geom name="platform_collision" type="box" size=".3 .3 .1" group="4"/>
        </body>
      </worldbody>
      <sensor><framepos name="visual_reference" objtype="geom" objname="referenced"/></sensor>
    </mujoco>''', encoding="utf-8")
    config = SimpleNamespace(world="physics/scene.xml", robot=SimpleNamespace(
        package_root="robot", model="robot/robot.xml", policy="policy.bin",
    ))
    monkeypatch.setattr(formal_feeder, "_REPOSITORY_ROOT", tmp_path)
    worlds = []
    for retain in (True, False):
        session = tmp_path / f"session-{retain}"
        session.mkdir()
        world, _, _ = formal_feeder._Services.snapshot_artifacts(
            session, config, retain_world_visuals=retain,
        )
        worlds.append(world)
        assert (world.parent.parent / "visual/shared.obj").is_file()
        assert (world.parent.parent / "visual/shared.png").is_file()
        assert (world.parent.parent / "visual/unused.obj").exists() is retain
        assert (world.parent.parent / "visual/unused.png").exists() is retain
    # Both snapshots are self-contained even after the original files are gone.
    for path in assets.iterdir():
        path.unlink()
    models = [mujoco.MjModel.from_xml_path(str(world)) for world in worlds]
    full, stripped = models
    for field in ("body_mass", "body_inertia", "body_ipos", "body_iquat"):
        np.testing.assert_allclose(getattr(full, field), getattr(stripped, field), atol=0, rtol=0)
    for name in ("referenced", "lidar_visible", "group2_contact", "dynamic_visual"):
        assert mujoco.mj_name2id(stripped, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
    assert mujoco.mj_name2id(stripped, mujoco.mjtObj.mjOBJ_GEOM, "visual") == -1
    assert stripped.nmesh == full.nmesh - 1
    data = [mujoco.MjData(model) for model in models]
    ray_mask = np.array([1, 1, 0, 0, 1, 0], dtype=np.uint8)

    def contact_names(model, state):
        return sorted((
            tuple(sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                         for g in contact.geom)), float(contact.dist),
        ) for contact in state.contact)

    for _ in range(10):
        for model, state in zip(models, data):
            mujoco.mj_step(model, state)
        np.testing.assert_allclose(data[0].qpos, data[1].qpos, atol=0, rtol=0)
        np.testing.assert_allclose(data[0].sensordata, data[1].sensordata, atol=0, rtol=0)
        assert contact_names(full, data[0]) == contact_names(stripped, data[1])
        assert data[0].ncon > 0
    for model, state in zip(models, data):
        state.mocap_pos[0, 2] = .8
        mujoco.mj_forward(model, state)
    for x, expected in ((2., 1.8), (3., 1.6), (4., 1.1)):
        distances = [mujoco.mj_ray(
            model, state, np.array([x, 0., 2.]), np.array([0., 0., -1.]),
            ray_mask, True, -1, np.array([-1], dtype=np.int32),
        ) for model, state in zip(models, data)]
        assert distances == pytest.approx([expected, expected])


def test_static_visual_pruning_preserves_included_geometry_references():
    root = ET.fromstring('''<mujoco><include file="sensors.xml"/><worldbody>
      <geom name="visual" type="sphere" size="1" group="2" contype="0" conaffinity="0"/>
    </worldbody></mujoco>''')
    assert not formal_feeder._Services._strip_static_world_visuals(root)
    assert root.find("./worldbody/geom") is not None
