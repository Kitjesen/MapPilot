"""Exercise measured articulated frames and physical support classification."""

import json
from itertools import pairwise
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from tools.bench.body_physics import capture_bad_substep, contacts, part_snapshot, requested_route
from tools.bench.body_scenarios import DynamicBox, prepare_world, read_dynamic_boxes, read_scene_boxes, write_fixtures


@pytest.mark.usefixtures("allow_unbuilt_process_artifacts")
@pytest.mark.parametrize("navigation_body", ["parts", "obb"])
def test_native_models_share_resolved_sim_geometry_and_control(
    monkeypatch, tmp_path, navigation_body
):
    from tools.bench import body_physics

    from lingtu.assembly import compile_plan as compiler

    compile_plan = compiler.compile_run_plan
    plans = []

    def record_plan(*args, **kwargs):
        plan = compile_plan(*args, **kwargs)
        plans.append(plan)
        return plan

    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base_link" pos="0 0 .435"><freejoint/>
        <geom type="box" size=".1 .1 .1"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    state = SimpleNamespace(position=data.xpos[1], orientation=np.array([0., 0., 0., 1.]))
    engine = SimpleNamespace(model=model, data=data, sim_time=0., dt=.005,
                             get_robot_state=lambda: state, close=lambda: None)
    engine_inputs = {}
    native_inputs = {}

    def build_engine(**kwargs):
        engine_inputs.update(kwargs)
        return engine

    class NativeLaunchCaptured(Exception):
        pass

    def capture_native(command, **kwargs):
        native_inputs.update(command=command, **kwargs)
        raise NativeLaunchCaptured

    monkeypatch.setattr(compiler, "compile_run_plan", record_plan)
    monkeypatch.setattr(body_physics, "build_engine", build_engine)
    monkeypatch.setattr(body_physics.subprocess, "Popen", capture_native)
    monkeypatch.setenv("LINGTU_NAV_BODY_FILE", str(tmp_path / "unrelated-model.body2"))
    monkeypatch.setenv("LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS", "2")
    args = SimpleNamespace(out=tmp_path, native=Path("native-probe"), map=Path("fixture.ot"),
                           world=body_physics.WORLD, scene=None, ccd_tolerance=None,
                           settle=0., dt=.02, goal_offset=(1.2, 0.), goal_yaw_offset_deg=90.,
                           waypoints=None, goal_yaw_tolerance_deg=3., navigation_body=navigation_body,
                           use_route=False, keep_turn_stages=False, pace=False, duration=1.)
    with pytest.raises(NativeLaunchCaptured):
        body_physics.run(args)

    assert len(plans) == 1
    plan = plans[0]
    environment = plan.native_process_environment
    simulation = plan.simulation
    assert (plan.env, plan.robot) == ("sim", "doso/thunder_v4")
    assert float(environment["LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M"]) == pytest.approx(.05)
    assert float(environment["LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M"]) == pytest.approx(.35)
    assert environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == "scan"
    # Ambient BODY_FILE is deliberately discarded; the compiled plan owns geometry.
    assert environment.get("LINGTU_NAV_BODY_FILE") is None
    expected_body_file = None
    assert native_inputs["env"].get("LINGTU_NAV_BODY_FILE") == expected_body_file
    assert native_inputs["env"]["LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS"] == environment[
        "LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS"
    ]
    assert {key: native_inputs["env"][key] for key in environment if key != "LINGTU_NAV_BODY_FILE"} == {
        key: value for key, value in environment.items() if key != "LINGTU_NAV_BODY_FILE"}
    assert ("--obb" in native_inputs["command"]) == (navigation_body == "obb")
    assert engine_inputs["robot_xml"] == body_physics.ROOT / simulation["physics_plan"]["robots"][0]["model"]["mjcf"]
    assert engine_inputs["policy_path"] == body_physics.ROOT / simulation["control_plan"]["controllers"][0]["policy"]["artifact"]
    recorded = json.loads((tmp_path / "navigation-config.json").read_text(encoding="utf-8"))
    assert recorded["run_plan"]["identity"]["env"] == "sim"
    assert recorded["environment"] == environment
    assert recorded["navigation_body"] == navigation_body
    assert recorded["effective_body_file"] == expected_body_file
    assert "LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS" not in recorded["native_control_overrides"]
    invocation = json.loads((tmp_path / "invocation.json").read_text(encoding="utf-8"))
    assert invocation["native_control_overrides"] == recorded["native_control_overrides"]


def test_default_goal_preserves_measured_start_and_translation_then_turn():
    route = requested_route(np.array([.031, -.052, .447]), .13)
    assert route[0] == pytest.approx([.031, -.052, .447, .13])
    assert route[1] == pytest.approx([1.231, -.052, .447, .13])
    assert route[2] == pytest.approx([1.231, -.052, .447, .13 + np.pi / 2])


def test_long_world_frame_goal_uses_settled_origin_without_rotating_offset():
    route = requested_route([.031, -.052, .447], .4, (6., .25), -45.)
    assert len(route) == 3
    assert route[0] == pytest.approx([.031, -.052, .447, .4])
    assert route[1] == pytest.approx([6.031, .198, .447, .4])
    assert route[2] == pytest.approx([6.031, .198, .447, .4 - np.pi / 4])


@pytest.mark.parametrize(("offset", "yaw_offset"), [((float("inf"), 0.), 90.), ((6., 0.), float("nan"))])
def test_nonfinite_goal_configuration_is_rejected(offset, yaw_offset):
    with pytest.raises(ValueError, match="finite"):
        requested_route([0., 0., .45], 0., offset, yaw_offset)


def test_snapshot_reconstructs_current_primitive_with_roll_pitch_and_shifted_origin():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base_link" pos=".5 -.2 .8" euler=".3 -.2 .7"><freejoint/>
        <geom type="box" size=".1 .1 .1"/>
        <body name="FL_foot" pos=".2 0 -.2"><joint axis="0 1 0"/>
          <geom type="cylinder" size=".05 .02"/>
        </body>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qpos[-1] = .4
    mujoco.mj_forward(model, data)
    quaternion = np.asarray(data.xquat[1])[[1, 2, 3, 0]]
    state = SimpleNamespace(position=np.asarray(data.xpos[1]) + np.array([.01, .02, .03]),
                            orientation=quaternion)
    part = {"geom_id": 1, "body": "FL_foot", "source_type": "cylinder"}
    snapshot = part_snapshot(SimpleNamespace(model=model, data=data), [part], state)
    from sim.scripts.mujoco.stair_policy_probe import rpy

    yaw = rpy(quaternion)[2]
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    assert len(snapshot) == 13
    assert rotation @ snapshot[:3] + state.position == pytest.approx(data.geom_xpos[1])
    assert rotation @ np.asarray(snapshot[3:12]).reshape(3, 3) == pytest.approx(
        data.geom_xmat[1].reshape(3, 3))
    assert snapshot[-1] == 0  # Reserved protocol slot, not a support-admission flag.


def test_only_explicit_loaded_foot_contact_is_allowed(monkeypatch):
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".002"/>
      <worldbody><geom name="floor" type="plane" size="1 1 .1"/>
        <body name="base_link" pos="0 0 .11"><freejoint/>
          <body name="FL_foot"><geom type="sphere" size=".1" mass="1"/></body>
        </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    for _ in range(300):
        mujoco.mj_step(model, data)
    engine = SimpleNamespace(model=model, data=data)
    supported = contacts(engine, ("FL_foot",))
    assert supported["feet"] == ["FL_foot"]
    assert supported["support_geoms"] == ["floor"]
    assert supported["non_foot_contacts"] == []
    blocked = contacts(engine, ())
    assert blocked["feet"] == []
    assert len(blocked["non_foot_contacts"]) == 1
    monkeypatch.setattr(mujoco, "mj_contactForce", lambda _m, _d, _i, force: force.fill(0))
    unloaded = contacts(engine, ("FL_foot",))
    assert unloaded["feet"] == []
    assert unloaded["non_foot_contacts"] == []


def test_designated_foot_touching_wall_is_still_a_collision():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
      <worldbody><geom name="wall" type="plane" size="1 1 .1"
                        quat=".707106781 0 .707106781 0"/>
        <body name="base_link" pos=".098 0 .4"><freejoint/>
          <body name="FL_foot"><geom type="sphere" size=".1" mass="1"/></body>
        </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    evidence = contacts(SimpleNamespace(model=model, data=data), ("FL_foot",))
    assert evidence["feet"] == []
    assert len(evidence["non_foot_contacts"]) == 1
    assert evidence["non_foot_contacts"][0]["normal"][0] == pytest.approx(1)


def test_scene_obstacles_exist_in_actual_physics_with_exact_native_bounds(tmp_path):
    base = tmp_path / "base.xml"
    base.write_text('<mujoco><worldbody><geom name="floor" type="plane" size="40 40 .1"/>'
                    '<geom name="marker" type="sphere" size=".01" contype="0" conaffinity="0"/>'
                    '</worldbody></mujoco>', encoding="utf-8")
    scene = tmp_path / "scene.txt"
    scene.write_text('.1 -2 -2 -.1 10 2 2 2 -2 -2 -.1 10 2 0 2 .55 0 6 2 1.5', encoding="utf-8")
    output = prepare_world(base, scene, [], tmp_path / "world.xml")
    model = mujoco.MjModel.from_xml_path(str(output))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor") == -1
    for index, (low, high) in enumerate(read_scene_boxes(scene)):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"fixture_static_{index}")
        if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_HFIELD:
            size = model.hfield_size[int(model.geom_dataid[geom])]
            assert [*(data.geom_xpos[geom][:2] - size[:2]), -size[3]] == pytest.approx(low)
            assert [*(data.geom_xpos[geom][:2] + size[:2]), data.geom_xpos[geom][2]] == pytest.approx(high)
            assert np.all(model.hfield_data == 0)
        else:
            assert data.geom_xpos[geom] - model.geom_size[geom] == pytest.approx(low)
            assert data.geom_xpos[geom] + model.geom_size[geom] == pytest.approx(high)
        assert model.geom_contype[geom] == model.geom_conaffinity[geom] == 1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "marker") >= 0


def test_dynamic_box_moves_continuously_and_remains_after_motion_finishes(tmp_path):
    box = DynamicBox("actor", (.25, .25, .75), (3, -2, .75), (3, 0, .75), 2, 4)
    assert box.at(0) == ([3, -2, .75], [0, 0, 0])
    assert box.at(4) == ([3, -1, .75], [0, .5, 0])
    assert box.at(6) == ([3, 0, .75], [0, 0, 0])
    assert box.at(100) == box.at(6)
    base = tmp_path / "base.xml"
    base.write_text('<mujoco><worldbody><geom type="plane" size="10 10 .1"/></worldbody></mujoco>',
                    encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(prepare_world(base, None, [box], tmp_path / "world.xml")))
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "actor")
    mocap = int(model.body_mocapid[body])
    assert mocap >= 0
    data.mocap_pos[mocap] = box.at(4)[0]
    mujoco.mj_forward(model, data)
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "actor_geom")
    assert data.geom_xpos[geom] == pytest.approx([3, -1, .75])
    assert model.geom_contype[geom] == model.geom_conaffinity[geom] == 1


def test_scenarios_define_shared_geometry_and_22m_segmented_route(tmp_path):
    manifest = write_fixtures(tmp_path)
    assert len(manifest["scenarios"]) == 5
    assert manifest["scenarios"]["corridor-positive"]["width"] == 1.10
    assert manifest["scenarios"]["corridor-negative"]["width"] == .60
    points = manifest["scenarios"]["multi-turn-22m"]["waypoints"]
    points = json.loads(Path(points).read_text(encoding="utf-8"))
    assert sum(np.linalg.norm(np.array(b[:2]) - a[:2]) for a, b in pairwise(points)) == 22
    for name, scenario in manifest["scenarios"].items():
        assert read_scene_boxes(Path(scenario["scene"]))
        if name in ("crossing", "persistent"):
            assert len(read_dynamic_boxes(tmp_path / name / "dynamic.json")) == 1


@pytest.mark.parametrize("text", [".1 0 0 0 1 1 1 1", ".1 0 0 0 1 1 1 1 0 0 0 0 1 1"])
def test_incomplete_or_flat_scene_boxes_are_rejected_before_physics(tmp_path, text):
    scene = tmp_path / "scene.txt"
    scene.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError):
        read_scene_boxes(scene)


def test_equivalent_finite_floor_removes_recorded_inverted_wheel_contact(tmp_path):
    base = tmp_path / "base.xml"
    base.write_text('''<mujoco><worldbody><body name="base_link"
      pos="8.144773221821529 2.777957519464802 .09624075296583011"
      quat="-.14373981997913238 .7634635163180474 -.1818954640054947 .602807069946824">
      <freejoint/><body name="FR_foot"><geom type="cylinder" size=".093 .025585" mass="1"/>
      </body></body></worldbody></mujoco>''', encoding="utf-8")
    scene = tmp_path / "scene.txt"
    scene.write_text('.1 -2 -3 -.1 19 9 2 1 -2 -3 -.1 19 9 0', encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(prepare_world(base, scene, [], tmp_path / "world.xml")))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    evidence = contacts(SimpleNamespace(model=model, data=data), ("FR_foot",))
    assert data.ncon > 0
    assert evidence["non_foot_contacts"] == []
    assert evidence["feet"] == ["FR_foot"]
    assert model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD) == 0


def test_equivalent_floor_is_finite_and_does_not_invent_support_outside_map(tmp_path):
    base = tmp_path / "base.xml"
    base.write_text('''<mujoco><worldbody><body name="base_link" pos="0 0 .0995"><freejoint/>
      <body name="FR_foot"><geom type="sphere" size=".1" mass="1"/></body>
      </body></worldbody></mujoco>''', encoding="utf-8")
    scene = tmp_path / "scene.txt"
    scene.write_text('.1 -2 -2 -.1 2 2 2 1 -2 -2 -.1 2 2 0', encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(prepare_world(base, scene, [], tmp_path / "world.xml")))
    data = mujoco.MjData(model)
    for position in ((0, 0), (1.8, 0), (0, -1.8)):
        data.qpos[:2] = position
        mujoco.mj_forward(model, data)
        evidence = contacts(SimpleNamespace(model=model, data=data), ("FR_foot",))
        assert evidence["feet"] == ["FR_foot"]
        assert evidence["non_foot_contacts"] == []
    for position in ((2.2, 0), (0, -2.2)):
        data.qpos[:2] = position
        mujoco.mj_forward(model, data)
        assert data.ncon == 0


def test_finite_floor_subdivision_removes_retained_v4_inverted_wheel_contact(tmp_path):
    # Retained cached FL_wheel geometry from the failed 10.955 s physical substep.
    position = np.array([3.724334426339725, .25551172941826145, .0953680068594222])
    rotation = np.array([[.9406839823023027, -.33927874841828365, .0018911138524295434],
                         [.0442225937988561, .11708143039196758, -.9921372389216478],
                         [.33638966636563206, .9333712388589508, .12514041247065205]])
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, rotation.ravel())
    base = tmp_path / "base.xml"
    base.write_text('''<mujoco><worldbody><body name="base_link"><freejoint/>
      <body name="FL_foot"><geom name="FL_wheel" type="cylinder" size=".093 .025585" mass="1"/>
      </body></body></worldbody></mujoco>''', encoding="utf-8")
    scene = tmp_path / "scene.txt"
    scene.write_text('.1 -2 -3 -.1 19 9 2 1 -2 -3 -.1 19 9 0', encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(prepare_world(base, scene, [], tmp_path / "world.xml")))
    assert model.hfield_nrow[0] == 25 and model.hfield_ncol[0] == 43
    assert model.hfield_size[0] == pytest.approx([10.5, 6, 1, .1])
    assert np.all(model.hfield_data == 0)
    assert model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD) == 0
    data = mujoco.MjData(model)
    data.qpos[:3], data.qpos[3:7] = position, quaternion
    mujoco.mj_forward(model, data)
    evidence = contacts(SimpleNamespace(model=model, data=data), ("FL_foot",))
    assert evidence["non_foot_contacts"] == []
    assert evidence["feet"] == ["FL_foot"]
    # Replay exact retained cached transforms without quaternion roundoff.
    wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "FL_wheel")
    data.geom_xpos[wheel], data.geom_xmat[wheel] = position, rotation.ravel()
    mujoco.mj_collision(model, data)
    assert data.ncon > 0
    for contact in data.contact[:data.ncon]:
        side = 0 if int(contact.geom1) == wheel else 1
        normal = np.asarray(contact.frame[:3]) * (-1 if side == 0 else 1)
        assert normal[2] >= .98
        assert contact.dist >= -.005


@pytest.mark.parametrize("backend", ["native", "libccd"])
@pytest.mark.parametrize("part", ["wheel", "body"])
def test_collision_backend_diagnostic_still_detects_physical_side_wall(backend, part, tmp_path):
    shape = ('<geom name="FR_wheel" type="cylinder" size=".093 .025585" euler="90 0 0"/>'
             if part == "wheel" else '<geom name="body_box" type="box" size=".093 .05 .05"/>')
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <geom name="wall" type="box" pos="0 0 .4" size=".01 1 1"/>
      <body name="base_link" pos=".102 0 .4"><freejoint/>
        <body name="FR_foot">{shape}</body></body></worldbody></mujoco>''')
    if backend == "libccd":
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    engine = SimpleNamespace(model=model, data=data, sim_time=0.0)
    evidence = contacts(engine, ("FR_foot",) if part == "wheel" else ())
    assert evidence["non_foot_contacts"]
    assert evidence["feet"] == []
    assert abs(evidence["non_foot_contacts"][0]["normal"][0]) > .98
    output = tmp_path / "first-bad-substep.json"
    capture_bad_substep(engine, output, evidence["non_foot_contacts"])
    record = json.loads(output.read_text(encoding="utf-8"))
    assert record["qpos"] == data.qpos.tolist()
    assert record["qvel"] == data.qvel.tolist()
    assert len(record["contact_geometries"]) == 2
    assert set(record["contacts"][0]["geom_names"]) == {"wall", "FR_wheel" if part == "wheel" else "body_box"}
