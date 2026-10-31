"""Real MuJoCo contacts and nearest support, not visual bounds or a floor fallback."""

import pytest


def test_pose_probe_accepts_actual_support_and_rejects_an_obstacle(tmp_path):
    from sim.tools.worlds.factory_environment_collection.pose_probe import FactoryPoseProbe

    world = tmp_path / "world.xml"
    world.write_text("""<mujoco><worldbody>
      <geom name="platform" type="box" pos="0 0 0" size="3 3 .1"/>
      <geom name="machine" type="box" pos="1 0 .5" size=".3 .4 .5"/>
      </worldbody></mujoco>""")
    robot = tmp_path / "robot.xml"
    robot.write_text("""<mujoco><worldbody><body name="base"><freejoint name="root"/>
      <geom name="torso" type="box" size=".2 .2 .15"/>
      <geom name="wheel" type="sphere" pos="0 0 -.25" size=".1"/>
      </body></worldbody><keyframe><key name="stand" qpos="0 0 .45 1 0 0 0"/></keyframe></mujoco>""")
    probe = FactoryPoseProbe(
        world, robot, root_joint="root", initial_keyframe="stand", support_geoms=["platform"], foot_geoms=["wheel"]
    )
    clear = probe.probe(position_m=[0, 0, 0.45], quaternion_wxyz=[1, 0, 0, 0])
    assert clear["placement_clear"] is True and clear["motion_admitted"] is False
    assert clear["support"][0]["world_geom"] == "platform"
    assert clear["support"][0]["height_m"] == pytest.approx(0.1)
    assert clear["support"][0]["gap_m"] == pytest.approx(0, abs=1e-10)
    blocked = probe.probe(position_m=[1, 0, 0.45], quaternion_wxyz=[1, 0, 0, 0])
    assert blocked["placement_clear"] is False
    assert any(c["world_geom"] == "machine" for c in blocked["blocking_contacts"])


def test_robot_development_plane_is_not_kept_and_missing_support_remains_invalid(tmp_path):
    from sim.tools.worlds.factory_environment_collection.pose_probe import FactoryPoseProbe

    world, robot = tmp_path / "world.xml", tmp_path / "robot.xml"
    world.write_text(
        '<mujoco><worldbody><geom name="platform" type="box" pos="2 0 0" size=".5 .5 .1"/></worldbody></mujoco>'
    )
    robot.write_text("""<mujoco><worldbody><geom name="ground" type="plane" size="0 0 1"/>
      <body name="base"><freejoint name="root"/><geom name="wheel" type="sphere" size=".1"/></body>
      </worldbody><keyframe><key name="stand" qpos="0 0 .2 1 0 0 0"/></keyframe></mujoco>""")
    probe = FactoryPoseProbe(
        world, robot, root_joint="root", initial_keyframe="stand", support_geoms=["platform"], foot_geoms=["wheel"]
    )
    assert not any(probe.model.geom_type == 0)
    assert probe.model.ngeom == 2
    result = probe.probe(position_m=[0, 0, 0.2], quaternion_wxyz=[1, 0, 0, 0])
    assert result["placement_clear"] is False
    assert result["support"][0]["world_geom"] is None
    assert result["physics_steps"] == 0
