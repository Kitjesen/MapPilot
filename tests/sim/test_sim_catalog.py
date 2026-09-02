"""Contract tests for deterministic simulation package resolution."""

from __future__ import annotations

import json
import shutil
from fractions import Fraction
from pathlib import Path

import pytest

from sim.catalog import CatalogError, CatalogResolver, DiagnosticCode

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunder_omni_contract" / "session.yaml"
THUNDER_UNREAL_SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunderv4_unreal" / "session.yaml"
THUNDER_CONTROLLED_HEADLESS_SESSION = (
    REPO_ROOT / "sim" / "sessions" / "examples" / "thunderv4_controlled_headless" / "session.yaml"
)
THUNDER_INDUSTRIAL_PARK_HEADLESS_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "products"
    / "doso"
    / "thunder_v4"
    / "default.yaml"
)
THUNDER_PEDESTRIAN_CROSSING_SESSION = (
    REPO_ROOT / "sim" / "sessions" / "examples" / "open_field_pedestrian_crossing" / "session.yaml"
)
THUNDER_FACTORY_PARK_PEDESTRIAN_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_factory_park_pedestrian_unreal"
    / "session.yaml"
)


def _resolver() -> CatalogResolver:
    return CatalogResolver.from_repository(REPO_ROOT)


def _write_scenario_package(root: Path, body: str) -> Path:
    package = root / "scenarios" / "invalid" / "scenario.package.yaml"
    package.parent.mkdir(parents=True)
    package.write_text(body, encoding="utf-8")
    return package


def _write_robot_package(root: Path, semantic: str) -> Path:
    package = root / "robots" / "test_robot"
    package.mkdir(parents=True)
    (package / "robot.xml").write_text(
        '<mujoco><worldbody><body name="base_link"><joint name="root_joint"/></body></worldbody><actuator/></mujoco>',
        encoding="utf-8",
    )
    (package / "robot.package.yaml").write_text(
        f"""schema: lingtu.sim.robot-package.v1
id: test_robot
version: 1.0.0
kind: robot
physics:
  mjcf: robot.xml
  attach_root: base_link
  root_joint: root_joint
  global_options: inherit_session
visual:
  binding: RobotVisual:TestRobot
{semantic}frames:
  - name: base_link
    role: body
interfaces:
  state: [lingtu.sim.base-state.v1]
  command: [lingtu.sim.base-velocity.v1]
defaults:
  controller: null
  sensor_rig: null
declared_capabilities: {{}}
""",
        encoding="utf-8",
    )
    return package


def _write_world_package(root: Path, level: str) -> Path:
    package = root / "worlds" / "test_world"
    package.mkdir(parents=True)
    (package / "world.xml").write_text(
        '<mujoco><option timestep="0.002" integrator="RK4" solver="Newton" '
        'iterations="100" gravity="0 0 -9.81"/><worldbody/></mujoco>\n',
        encoding="utf-8",
    )
    (package / "world.package.yaml").write_text(
        f"""schema: lingtu.sim.world-package.v1
id: test_world
version: 1.0.0
kind: world
physics:
  mjcf: world.xml
  global_policy:
    timestep_s: 0.002
    integrator: rk4
    solver: newton
    iterations: 100
    gravity_mps2: [0.0, 0.0, -9.81]
visual:
  binding: WorldVisual:TestWorld
  level: {json.dumps(level)}
entities: []
""",
        encoding="utf-8",
    )
    return package


def test_world_package_rejects_level_outside_game_root(tmp_path: Path) -> None:
    world_package = _write_world_package(tmp_path, "/Engine/Maps/Forbidden")

    with pytest.raises(CatalogError, match=r"visual\.level must start with /Game/"):
        CatalogResolver(tmp_path, [world_package])


@pytest.mark.parametrize("level", ["", "   "])
def test_world_package_rejects_blank_level(tmp_path: Path, level: str) -> None:
    world_package = _write_world_package(tmp_path, level)

    with pytest.raises(CatalogError, match=r"visual\.level must be a non-empty string"):
        CatalogResolver(tmp_path, [world_package])


@pytest.mark.parametrize(
    "level",
    [
        "/Game/",
        "/Game/   ",
        " /Game/Maps/OpenField",
        "/Game/Maps/OpenField ",
        "/Game/Maps\\OpenField",
        "/Game/Maps//OpenField",
        "/Game/Maps/./OpenField",
        "/Game/Maps/../OpenField",
        "/Game/Maps/OpenField/",
    ],
)
def test_world_package_rejects_malformed_game_level(tmp_path: Path, level: str) -> None:
    world_package = _write_world_package(tmp_path, level)

    with pytest.raises(CatalogError, match=r"visual\.level must be a valid /Game/\.\.\. level"):
        CatalogResolver(tmp_path, [world_package])


@pytest.mark.parametrize(
    "level",
    ["/Game/A", "/Game/Maps/OpenField", "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"],
)
def test_world_package_accepts_strict_game_level(tmp_path: Path, level: str) -> None:
    world_package = _write_world_package(tmp_path, level)

    resolver = CatalogResolver(tmp_path, [world_package])

    record = next(iter(resolver._records_by_path.values()))
    assert record.data["visual"]["level"] == level


@pytest.mark.parametrize("relative", ["C:/outside.xml", "nested/file:ads", "nested:dir/file.xml"])
def test_catalog_rejects_colon_anywhere_in_package_relative_paths(relative: str) -> None:
    resolver = _resolver()
    for validator in (resolver._require_safe_package_path, resolver._reject_colon_in_path):
        with pytest.raises(CatalogError) as exc_info:
            validator(relative, "fixture.path")

        assert exc_info.value.code is DiagnosticCode.PATH_TRAVERSAL
        assert exc_info.value.context == "fixture.path"


def test_legacy_world_mjcf_path_rejects_nested_colon_at_runtime(tmp_path: Path) -> None:
    world_package = _write_world_package(tmp_path, "/Game/Maps/TestWorld")
    manifest = world_package / "world.package.yaml"
    text = manifest.read_text(encoding="utf-8").replace("mjcf: world.xml", "mjcf: nested/world.xml:ads")
    manifest.write_text(text, encoding="utf-8")

    with pytest.raises(CatalogError, match="must not contain ':'") as exc_info:
        CatalogResolver(tmp_path, [world_package])

    assert exc_info.value.code is DiagnosticCode.PATH_TRAVERSAL
    assert exc_info.value.context == f"{manifest}.physics.mjcf"


def test_resolves_two_robot_session_into_one_physics_plan() -> None:
    resolved = _resolver().resolve(SESSION)

    assert resolved.physics_plan["session_id"] == resolved.session_id
    assert resolved.physics_plan["schema"] == "lingtu.sim.physics-plan.v1"
    assert resolved.physics_plan["composition"]["model_kind"] == "single_mjmodel"
    assert [robot["instance_id"] for robot in resolved.physics_plan["robots"]] == [
        "thunder_01",
        "cart_01",
    ]
    assert all(robot["namespace"] for robot in resolved.physics_plan["robots"])
    assert all("body_count" not in robot for robot in resolved.physics_plan["robots"])
    assert resolved.physics_plan["robots"][0]["model"]["initial_keyframe"] == "v4_nominal_stand"
    assert resolved.physics_plan["robots"][1]["model"]["initial_keyframe"] is None
    assert "model_generation" not in resolved.physics_plan
    assert "reset_generation" not in resolved.physics_plan


def test_resolves_world_owned_global_physics_policy_deterministically() -> None:
    resolver = _resolver()

    first = resolver.resolve(SESSION)
    second = resolver.resolve(SESSION)

    expected = {
        "owner": "world",
        "timestep_s": 0.002,
        "integrator": "rk4",
        "solver": "newton",
        "iterations": 100,
        "gravity_mps2": [0.0, 0.0, -9.81],
    }
    assert first.physics_plan["global_policy"] == expected
    assert second.physics_plan["global_policy"] == expected
    assert first.physics_json == second.physics_json


def test_resolves_visual_plan_without_robot_specific_body_assumptions() -> None:
    resolved = _resolver().resolve(SESSION)

    assert resolved.visual_plan["schema"] == "lingtu.sim.visual-plan.v1"
    assert resolved.visual_plan["session_id"] == resolved.session_id
    assert resolved.visual_plan["coordinate_system"] == {
        "source": "mujoco_rh_z_up_m",
        "target": "unreal_lh_z_up_cm",
        "position_scale": 100.0,
        "axis_mapping": ["x", "-y", "z"],
        "quaternion_order": "wxyz",
    }
    assert resolved.visual_plan["world"]["binding"] == "WorldVisual:OpenField"
    assert resolved.visual_plan["world"]["level"] == "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"
    assert [robot["binding"] for robot in resolved.visual_plan["robots"]] == [
        "RobotVisual:ThunderV4",
        "RobotVisual:OmniCart",
    ]
    assert all("body_count" not in robot for robot in resolved.visual_plan["robots"])
    assert all("links" not in robot for robot in resolved.visual_plan["robots"])
    assert resolved.visual_plan["backends"] == {"physics": "mujoco", "visual": None}
    assert "model_generation" not in resolved.visual_plan
    for robot in resolved.visual_plan["robots"]:
        projection = robot["projection"]
        assert projection["schema"] == "lingtu.sim.robot-visual-projection.v1"
        assert (REPO_ROOT / projection["path"]).is_file()


def test_thunder_camera_streams_compile_ros_optical_extrinsics() -> None:
    resolved = _resolver().resolve(THUNDER_UNREAL_SESSION)

    camera_streams = [
        *resolved.sensor_plan["streams"]["rgb"],
        *resolved.sensor_plan["streams"]["depth"],
    ]
    assert {stream["sensor_id"] for stream in camera_streams} == {
        "thunder_01.front_rgb",
        "thunder_01.front_depth",
    }
    for stream in camera_streams:
        assert stream["frame_id"] == "thunder_01/front_camera"
        assert stream["parent_frame_id"] == "thunder_01/base_link"
        assert stream["extrinsic"] == {
            "position_m": [0.423358800364963, -0.000496202974186816, 0.11370714960317],
            # ROS optical: +X right, +Y down, +Z forward.
            "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
        }


def test_resolution_is_byte_stable_and_excludes_run_allocation() -> None:
    resolver = _resolver()
    first = resolver.resolve(THUNDER_PEDESTRIAN_CROSSING_SESSION)
    second = resolver.resolve(THUNDER_PEDESTRIAN_CROSSING_SESSION)

    assert first.session_id == second.session_id
    assert first.physics_json == second.physics_json
    assert first.visual_json == second.visual_json
    assert first.sensor_json == second.sensor_json
    assert first.control_json == second.control_json
    assert first.transport_json == second.transport_json
    assert first.scenario_json == second.scenario_json


@pytest.mark.parametrize(
    ("semantic", "error"),
    (
        ("", r"missing required key\(s\): semantic"),
        ("semantic: {}\n", r"semantic is missing required key\(s\): class"),
        (
            "semantic:\n  class: robot\n  taxonomy: forbidden\n",
            r"semantic has unknown key\(s\): taxonomy",
        ),
        ("semantic:\n  class: '   '\n", r"semantic\.class must be a non-empty string"),
    ),
)
def test_robot_package_requires_strict_semantic_class(
    tmp_path: Path,
    semantic: str,
    error: str,
) -> None:
    package = _write_robot_package(tmp_path, semantic)

    with pytest.raises(CatalogError, match=error):
        CatalogResolver(tmp_path, [package])


def test_catalog_rejects_robot_owned_global_options_before_projection_or_compose(
    tmp_path: Path,
) -> None:
    fixture_root = tmp_path / "repository"
    package_root = fixture_root / "sim" / "packages" / "robots" / "omni_cart"
    shutil.copytree(REPO_ROOT / "sim/packages/robots/omni_cart", package_root)
    mjcf = package_root / "omni_cart.xml"
    mjcf.write_text(
        mjcf.read_text(encoding="utf-8").replace(
            '  <compiler angle="radian" autolimits="true"/>',
            '  <compiler angle="radian" autolimits="true"/>\n'
            '  <option timestep="0.001" solver="PGS" iterations="40"/>',
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(CatalogError) as exc_info:
        CatalogResolver(fixture_root, (package_root,))

    assert exc_info.value.code is DiagnosticCode.GLOBAL_PHYSICS_OWNERSHIP
    assert exc_info.value.context == f"{package_root / 'robot.package.yaml'}.physics.mjcf.option"
    assert exc_info.value.details == {
        "fields": ["iterations", "solver", "timestep"],
        "required_policy": "inherit_session",
    }


def test_robot_semantic_class_projects_generically_into_scenario_plan(
    tmp_path: Path,
) -> None:
    source = SESSION.read_text(encoding="utf-8").replace(
        "world: open_field@1.0.0\n",
        "world: open_field@1.0.0\nscenario: open_field_pedestrian_crossing@1.0.0\n",
        1,
    )
    session = tmp_path / "semantic_projection.yaml"
    session.write_text(source, encoding="utf-8")

    resolved = _resolver().resolve(session)

    assert resolved.scenario_plan is not None
    assert {
        entity["entity_id"]: entity["semantic_class"]
        for entity in resolved.scenario_plan["entities"]
        if entity["entity_type"] == "robot"
    } == {"thunder_01": "quadruped", "cart_01": "wheeled_robot"}
    assert {robot["instance_id"]: robot["semantic"]["class"] for robot in resolved.physics_plan["robots"]} == {
        "thunder_01": "quadruped",
        "cart_01": "wheeled_robot",
    }


def _isolated_thunderv4_catalog(tmp_path: Path) -> tuple[Path, Path]:
    root = tmp_path / "repo"
    package_dir = root / "sim" / "packages" / "robots" / "doso" / "thunder_v4"
    package_dir.parent.mkdir(parents=True)
    shutil.copytree(
        REPO_ROOT / "sim" / "packages" / "robots" / "doso" / "thunder_v4",
        package_dir,
    )
    return root, package_dir


@pytest.mark.parametrize("projection", [None, "../robot.visual-projection.json"])
def test_robot_projection_path_is_required_safe_and_package_relative(
    tmp_path: Path,
    projection: str | None,
) -> None:
    root, package_dir = _isolated_thunderv4_catalog(tmp_path)
    manifest = package_dir / "robot.package.yaml"
    source = manifest.read_text(encoding="utf-8")
    if projection is None:
        source = source.replace("  projection: visual/robot.visual-projection.json\n", "")
    else:
        source = source.replace(
            "  projection: visual/robot.visual-projection.json",
            f"  projection: {projection}",
        )
    manifest.write_text(source, encoding="utf-8")

    with pytest.raises(CatalogError, match=r"visual.*projection"):
        CatalogResolver(root, [root / "sim" / "packages" / "robots"])


def test_robot_projection_binding_must_match_package(tmp_path: Path) -> None:
    root, package_dir = _isolated_thunderv4_catalog(tmp_path)
    projection_path = package_dir / "visual" / "robot.visual-projection.json"
    document = json.loads(projection_path.read_text(encoding="utf-8"))
    document["binding"] = "RobotVisual:Tampered"
    projection_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(CatalogError, match=r"binding does not match RobotPackage"):
        CatalogResolver(root, [root / "sim" / "packages" / "robots"])


def test_catalog_discovers_packages_in_their_domain_directories() -> None:
    resolver = _resolver()

    assert resolver.find_package("thunderv4@1.0.3", kind="robot").manifest_path == (
        REPO_ROOT / "sim/packages/robots/doso/thunder_v4/robot.package.yaml"
    )
    assert resolver.find_package(
        "thunderv4_locomotion@1.0.0", kind="controller"
    ).manifest_path == (
        REPO_ROOT / "sim/packages/controllers/doso/thunder_v4/locomotion/controller.package.yaml"
    )
    assert resolver.find_package(
        "thunderv4_navigation@1.0.0", kind="sensor_rig"
    ).manifest_path == (
        REPO_ROOT / "sim/packages/sensor_rigs/doso/thunder_v4/navigation/sensor-rig.package.yaml"
    )


def test_legacy_controller_package_interface_does_not_require_new_command_contract(
    tmp_path: Path,
) -> None:
    root = tmp_path / "repo"
    package_dir = root / "sim" / "packages" / "controllers" / "legacy_controller"
    package_dir.mkdir(parents=True)
    (package_dir / "policy").mkdir()
    (package_dir / "policy" / "model.bin").write_bytes(b"legacy-model-v1")
    (package_dir / "policy" / "manifest.json").write_text('{"policy":"legacy"}\n', encoding="utf-8")
    (package_dir / "controller.package.yaml").write_text(
        """schema: lingtu.sim.controller-package.v1
id: legacy_controller
version: 1.0.0
kind: controller
adapter:
  plugin: legacy
  abi: legacy.v1
policy:
  runtime: local
  artifact: policy/model.bin
  manifest: policy/manifest.json
timing:
  inference_hz: 50
  low_level_hz: 200
robot_interface:
  requires_state: [legacy_state]
  produces_command:
    type: actuator_command
actuators:
  channels: [hip]
""",
        encoding="utf-8",
    )

    resolver = CatalogResolver(root, [root / "sim" / "packages" / "controllers"])
    controller = resolver.find_package("legacy_controller@1.0.0", kind="controller")

    assert controller.data["robot_interface"]["requires_state"] == ["legacy_state"]


def test_writes_modular_session_bundle(tmp_path: Path) -> None:
    resolved = _resolver().resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")

    assert bundle == tmp_path / "bundle"
    assert (bundle / "physics.plan.json").read_text(encoding="utf-8") == resolved.physics_json
    assert (bundle / "visual.plan.json").read_text(encoding="utf-8") == resolved.visual_json
    assert (bundle / "sensor.plan.json").read_text(encoding="utf-8") == resolved.sensor_json
    assert (bundle / "control.plan.json").read_text(encoding="utf-8") == resolved.control_json
    assert (bundle / "transport.intent.json").read_text(encoding="utf-8") == resolved.transport_json

    physics = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))
    visual = json.loads((bundle / "visual.plan.json").read_text(encoding="utf-8"))
    sensor = json.loads((bundle / "sensor.plan.json").read_text(encoding="utf-8"))
    control = json.loads((bundle / "control.plan.json").read_text(encoding="utf-8"))
    transport = json.loads((bundle / "transport.intent.json").read_text(encoding="utf-8"))
    assert (
        physics["session_id"]
        == visual["session_id"]
        == sensor["session_id"]
        == control["session_id"]
        == transport["session_id"]
    )
    assert not (bundle / "scenario.plan.json").exists()


def test_selected_scenario_writes_deterministic_plan(tmp_path: Path) -> None:
    resolved = _resolver().resolve(THUNDER_PEDESTRIAN_CROSSING_SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")

    assert resolved.scenario_plan is not None
    assert resolved.scenario_json is not None
    assert (bundle / "scenario.plan.json").read_text(encoding="utf-8") == resolved.scenario_json
    assert resolved.session_id
    assert resolved.scenario_plan["session_id"] == resolved.session_id
    assert resolved.scenario_plan["clock"] == {
        "unit": "ns",
        "source": "mujoco_sim_time",
        "sim_time_ns": 0,
    }
    assert resolved.scenario_plan["model_generation"] == 0
    assert resolved.scenario_plan["reset_generation"] == 0
    assert resolved.scenario_plan["entities"] == [
        {
            "entity_id": "thunder_01",
            "entity_type": "robot",
            "authority": "mujoco",
            "source_epoch": 0,
            "initial_transform": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "physics_proxy": "mujoco",
            "semantic_class": "quadruped",
        },
        {
            "entity_id": "pedestrian_01",
            "entity_type": "pedestrian",
            "authority": "scenario",
            "source_epoch": 0,
            "initial_transform": {
                "position_m": [4.0, -6.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "physics_proxy": {
                "mode": "kinematic",
                "body_stable_id": "pedestrian_01/proxy_root",
            },
            "behavior": {
                "profile": "linear_crossing",
                "seed": 20260805,
                "parameters": {
                    "duration_s": 8.0,
                    "end_position_m": [4.0, 6.0, 0.0],
                    "start_time_s": 2.0,
                    "speed_mps": 1.5,
                },
            },
            "semantic_class": "pedestrian",
        },
    ]
def test_scenario_kinematic_entity_is_compiled_into_the_shared_physics_scene() -> None:
    resolved = _resolver().resolve(THUNDER_PEDESTRIAN_CROSSING_SESSION)

    assert resolved.physics_plan["kinematic_entities"] == [
        {
            "entity_id": "pedestrian_01",
            "namespace": "pedestrian_01",
            "package": resolved.scenario_plan["package"],
            "model": {
                "mjcf": (
                    "sim/packages/scenarios/open_field_pedestrian_crossing/"
                    "physics/pedestrian_capsule.xml"
                ),
                "attach_root": "proxy_root",
            },
            "initial_transform": {
                "position_m": [4.0, -6.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
        }
    ]
    assert resolved.scenario_plan is not None
    pedestrian = next(
        entity
        for entity in resolved.scenario_plan["entities"]
        if entity["entity_id"] == "pedestrian_01"
    )
    assert pedestrian["physics_proxy"] == {
        "mode": "kinematic",
        "body_stable_id": "pedestrian_01/proxy_root",
    }


def test_scenario_visual_is_compiled_as_a_generic_plan_driven_entity() -> None:
    resolved = _resolver().resolve(THUNDER_PEDESTRIAN_CROSSING_SESSION)

    assert resolved.scenario_plan is not None
    visuals = resolved.visual_plan["scenario_entities"]
    assert len(visuals) == 1
    pedestrian = visuals[0]
    assert pedestrian["entity_id"] == "pedestrian_01"
    assert pedestrian["namespace"] == "pedestrian_01"
    assert pedestrian["package"] == resolved.scenario_plan["package"]
    assert pedestrian["binding"] == "EntityVisual:PedestrianCapsule"
    assert pedestrian["spawn"] == {
        "position_m": [4.0, -6.0, 0.0],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    projection = pedestrian["projection"]
    assert projection["schema"] == "lingtu.sim.entity-visual-projection.v1"
    projection_path = REPO_ROOT / projection["path"]
    assert projection_path.is_file()


def test_factory_park_pedestrian_scenario_composes_with_the_hf_world() -> None:
    resolved = _resolver().resolve(THUNDER_FACTORY_PARK_PEDESTRIAN_SESSION)

    assert resolved.visual_plan["world"]["package"]["id"] == "factory_park_hf"
    assert resolved.visual_plan["world"]["level"] == "/Game/RobotSim/Maps/FactoryPark_HF"
    assert resolved.physics_plan["world"]["package"]["id"] == "factory_park_hf"
    assert resolved.physics_plan["kinematic_entities"][0]["initial_transform"]["position_m"] == [
        4.0,
        -72.0,
        0.0,
    ]
    assert resolved.visual_plan["scenario_entities"][0]["entity_id"] == "pedestrian_01"
    assert resolved.visual_plan["scenario_entities"][0]["binding"] == (
        "EntityVisual:PedestrianCapsule"
    )


def test_no_scenario_session_has_no_scenario_plan() -> None:
    resolved = _resolver().resolve(SESSION)

    assert resolved.scenario_plan is None
    assert resolved.scenario_json is None


def test_session_scenario_reference_must_be_exact_id_version(tmp_path: Path) -> None:
    source = THUNDER_PEDESTRIAN_CROSSING_SESSION.read_text(encoding="utf-8")
    source = source.replace(
        "scenario: open_field_pedestrian_crossing@1.0.0",
        "scenario: ../../../packages/scenarios/open_field_pedestrian_crossing/scenario.package.yaml",
    )
    session = tmp_path / "session.yaml"
    session.write_text(source, encoding="utf-8")

    with pytest.raises(CatalogError, match="exact id@version"):
        _resolver().resolve(session)


def test_scenario_package_rejects_duplicate_entity_ids(tmp_path: Path) -> None:
    _write_scenario_package(
        tmp_path,
        """
schema: lingtu.sim.scenario-package.v1
id: invalid
version: 1.0.0
kind: scenario
world: open_field@1.0.0
entities:
  - entity_id: pedestrian_01
    entity_type: pedestrian
    authority: scenario
    initial_transform:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    physics_proxy: kinematic
    behavior:
      profile: linear_crossing
      seed: 1
      parameters: {speed_mps: 1.0}
    semantic_class: pedestrian
  - entity_id: pedestrian_01
    entity_type: pedestrian
    authority: scenario
    initial_transform:
      position_m: [1.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    physics_proxy: kinematic
    behavior:
      profile: linear_crossing
      seed: 2
      parameters: {speed_mps: 1.0}
    semantic_class: pedestrian
""".lstrip(),
    )

    with pytest.raises(CatalogError, match="duplicate entity_id"):
        CatalogResolver(tmp_path, [tmp_path / "scenarios"])


def test_scenario_package_rejects_non_finite_behavior_parameters(tmp_path: Path) -> None:
    _write_scenario_package(
        tmp_path,
        """
schema: lingtu.sim.scenario-package.v1
id: invalid
version: 1.0.0
kind: scenario
world: open_field@1.0.0
entities:
  - entity_id: pedestrian_01
    entity_type: pedestrian
    authority: scenario
    initial_transform:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    physics_proxy: kinematic
    behavior:
      profile: linear_crossing
      seed: 1
      parameters:
        speed_mps: .inf
    semantic_class: pedestrian
""".lstrip(),
    )

    with pytest.raises(CatalogError, match="finite numeric"):
        CatalogResolver(tmp_path, [tmp_path / "scenarios"])


def test_scenario_package_rejects_unknown_dynamic_entity_authority(tmp_path: Path) -> None:
    _write_scenario_package(
        tmp_path,
        """
schema: lingtu.sim.scenario-package.v1
id: invalid
version: 1.0.0
kind: scenario
world: open_field@1.0.0
entities:
  - entity_id: pedestrian_01
    entity_type: pedestrian
    authority: animation
    initial_transform:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    physics_proxy: kinematic
    behavior:
      profile: linear_crossing
      seed: 1
      parameters: {speed_mps: 1.0}
    semantic_class: pedestrian
""".lstrip(),
    )

    with pytest.raises(CatalogError, match="authority is unsupported"):
        CatalogResolver(tmp_path, [tmp_path / "scenarios"])


def test_headless_physics_session_outputs_empty_sensor_control_and_transport_plans() -> None:
    resolved = _resolver().resolve(SESSION)

    assert resolved.sensor_plan["backends"] == {"physics": "mujoco", "visual": None}
    assert all(streams == [] for streams in resolved.sensor_plan["streams"].values())
    assert resolved.control_plan["controllers"] == []
    assert resolved.control_plan["command_channels"] == []
    assert resolved.transport_intent["channels"] == []


def test_controlled_headless_session_compiles_the_navigation_sensor_rig() -> None:
    resolved = _resolver().resolve(THUNDER_CONTROLLED_HEADLESS_SESSION)

    assert resolved.session["world"] == "open_field@1.1.0"
    assert resolved.session["robots"][0]["sensor_rig"] == (
        "thunderv4_headless_navigation@1.0.0"
    )
    assert resolved.sensor_plan["backends"] == {
        "physics": "mujoco",
        "visual": None,
    }
    assert {name: len(streams) for name, streams in resolved.sensor_plan["streams"].items()} == {
        "rgb": 1,
        "depth": 1,
        "imu": 1,
        "mid360": 1,
        "truth_odom": 1,
    }
    rgb = resolved.sensor_plan["streams"]["rgb"][0]
    depth = resolved.sensor_plan["streams"]["depth"][0]
    assert rgb["rate_hz"] == depth["rate_hz"] == 30.0
    assert (rgb["width"], rgb["height"]) == (640, 480)
    assert (depth["width"], depth["height"]) == (640, 480)
    assert rgb["extrinsic"] == depth["extrinsic"] == {
        "position_m": [0.423358800364963, -0.000496202974186816, 0.11370714960317],
        "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
    }
    assert resolved.sensor_plan["streams"]["imu"][0]["rate_hz"] == 200.0
    mid360 = resolved.sensor_plan["streams"]["mid360"][0]
    assert mid360["rate_hz"] == 10.0
    assert mid360["navigation_fixture_raw_overlay"] is True
    truth = resolved.sensor_plan["streams"]["truth_odom"][0]
    assert truth["sensor_id"] == "thunder_01.truth_odom"
    assert truth["owner"] == "physics"
    assert truth["transport"] == "typed_dds"
    assert resolved.session["runtime"]["required_bindings"] == [
        "physics",
        "sensors",
        "control",
    ]


def test_industrial_park_headless_session_is_the_common_local_planner_world() -> None:
    resolved = _resolver().resolve(THUNDER_INDUSTRIAL_PARK_HEADLESS_SESSION)

    assert resolved.session["world"] == "industrial_park@1.0.0"
    assert resolved.session["robots"][0]["package"] == "thunderv4@1.0.3"
    assert resolved.physics_plan["world"]["mjcf"] == (
        "sim/packages/worlds/industrial_park/physics/industrial_park_scene.xml"
    )
    assert resolved.physics_plan["global_policy"] == {
        "owner": "world",
        "timestep_s": 0.005,
        "integrator": "euler",
        "solver": "newton",
        "iterations": 100,
        "gravity_mps2": [0.0, 0.0, -9.81],
    }
    assert resolved.physics_plan["robots"][0]["model"]["mjcf"] == (
        "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml"
    )
    assert resolved.physics_plan["robots"][0]["spawn"]["position_m"] == [3.0, 4.0, 0.0]


def test_thunderv4_unreal_session_resolves_full_sensor_control_transport_slice() -> None:
    resolved = _resolver().resolve(THUNDER_UNREAL_SESSION)

    assert resolved.sensor_plan["backends"] == {"physics": "mujoco", "visual": "unreal"}
    assert {name: len(streams) for name, streams in resolved.sensor_plan["streams"].items()} == {
        "rgb": 1,
        "depth": 1,
        "imu": 1,
        "mid360": 1,
        "truth_odom": 1,
    }
    rgb = resolved.sensor_plan["streams"]["rgb"][0]
    assert rgb["instance_id"] == "thunder_01"
    assert rgb["sensor_id"] == "thunder_01.front_rgb"
    assert rgb["frame_id"] == "thunder_01/front_camera"
    assert rgb["parent_frame_id"] == "thunder_01/base_link"
    assert rgb["extrinsic"] == {
        "position_m": [0.423358800364963, -0.000496202974186816, 0.11370714960317],
        "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
    }
    assert rgb["rate_hz"] == 30
    assert rgb["width"] == 640
    assert rgb["height"] == 480
    depth = resolved.sensor_plan["streams"]["depth"][0]
    assert depth["parent_frame_id"] == "thunder_01/base_link"
    assert depth["extrinsic"] == {
        "position_m": [0.423358800364963, -0.000496202974186816, 0.11370714960317],
        "quaternion_wxyz": [0.5, -0.5, 0.5, -0.5],
    }
    assert depth["width"] == 640
    assert depth["height"] == 480
    assert depth["encoding"] == "16UC1"
    assert depth["requested_encoding"] == "32FC1"
    assert depth["depth_scale"] == 0.001
    assert depth["unit"] == "m"
    imu = resolved.sensor_plan["streams"]["imu"][0]
    assert imu["rate_hz"] == 200
    assert imu["fields"] == ["orientation", "angular_velocity", "linear_acceleration"]
    mid360 = resolved.sensor_plan["streams"]["mid360"][0]
    assert mid360["frame_id"] == "thunder_01/lidar1_link"
    assert mid360["parent_frame_id"] == "thunder_01/base_link"
    assert mid360["extrinsic"] == {
        "position_m": [0.402876074867229, 0, 0.0582019450665819],
        "quaternion_wxyz": [0, -0.9238795325, 0, -0.3826834324],
    }
    assert mid360["raycast_frame_stable_id"] == "thunder_01/lidar1_link_site"
    assert mid360["point_fields"] == ["x", "y", "z", "offset_time_ns", "reflectivity", "tag", "line"]
    assert mid360["offset_time_unit"] == "ns"
    assert mid360["line_semantics"] == "livox_channel"
    truth = resolved.sensor_plan["streams"]["truth_odom"][0]
    assert truth["source"] == "mujoco_truth"
    assert truth["estimator_input"] is False

    assert len(resolved.control_plan["controllers"]) == 1
    controller = resolved.control_plan["controllers"][0]
    assert controller["instance_id"] == "thunder_01"
    assert controller["package"]["id"] == "thunderv4_locomotion"
    assert controller["adapter"] == {"plugin": "quadruped_him", "abi": "lingtu.sim.controller-adapter.v1"}
    assert controller["policy"] == {
        "runtime": "onnxruntime",
        "artifact": "sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx",
        "manifest": "sim/packages/controllers/doso/thunder_v4/locomotion/policy/policy_manifest.json",
    }
    assert controller["timing"] == {"inference_hz": 50, "low_level_hz": 200}
    assert controller["command_channels"] == ["thunder_01.control.base_twist", "thunder_01.control.joint_torque"]
    assert "FR_hip_joint" in controller["actuator_channels"]
    assert resolved.control_plan["command_channels"] == [
        {
            "channel_id": "thunder_01.control.base_twist",
            "direction": "subscribe",
            "owner": "simulation",
            "source": "dds_base_twist/thunder_01",
            "transport": "typed_dds",
            "message_type": "lingtu.dds.FinalVelocityCommand",
            "command_type": "base_twist",
            "target": "base",
        },
        {
            "channel_id": "thunder_01.control.joint_torque",
            "direction": "publish",
            "owner": "physics",
            "source": "thunder_01.thunderv4_locomotion",
            "transport": "in_process",
            "message_type": "lingtu.sim.joint-torque.v1",
            "command_type": "joint_torque",
            "target": "actuators",
        },
    ]
    assert len(resolved.transport_intent["channels"]) == 8
    truth_snapshot = resolved.transport_intent["channels"][0]
    assert truth_snapshot == {
        "channel_id": "runtime.truth_snapshot",
        "direction": "publish",
        "owner": "physics",
        "source": "mujoco_runtime",
        "transport": "udp_loopback_json",
        "delivery": "latest_wins",
        "message_type": "lingtu.sim.truth-snapshot.v1",
        "payload_role": "truth_snapshot",
        "contract": "lingtu.sim.truth-snapshot.v1",
        "frame_policy": "sim_time",
    }
    assert not any(
        "dds_domain" in channel or "port" in channel or "shm_name" in channel
        for channel in resolved.transport_intent["channels"]
    )


@pytest.mark.parametrize(
    "session",
    (THUNDER_UNREAL_SESSION, THUNDER_CONTROLLED_HEADLESS_SESSION),
    ids=("unreal-navigation", "controlled-headless-product"),
)
def test_navigation_sensor_session_uses_an_exact_physics_timebase(
    session: Path,
) -> None:
    resolved = _resolver().resolve(session)

    timestep_ns = Fraction(
        str(resolved.physics_plan["global_policy"]["timestep_s"])
    ) * 1_000_000_000
    assert timestep_ns == 1_000_000

    physics_streams = [
        stream
        for streams in resolved.sensor_plan["streams"].values()
        for stream in streams
        if stream["owner"] == "physics"
    ]
    assert {stream["sensor_id"] for stream in physics_streams} == {
        "thunder_01.imu",
        "thunder_01.mid360",
        "thunder_01.truth_odom",
    }
    for stream in physics_streams:
        period_ns = Fraction(1_000_000_000, 1) / Fraction(str(stream["rate_hz"]))
        assert period_ns.denominator == 1
        assert period_ns.numerator % timestep_ns.numerator == 0


def test_resolution_rejects_a_physics_sensor_off_the_world_timebase(
    tmp_path: Path,
) -> None:
    session = tmp_path / "incompatible-timebase.session.yaml"
    session.write_text(
        THUNDER_UNREAL_SESSION.read_text(encoding="utf-8").replace(
            "world: open_field@1.1.0",
            "world: open_field@1.0.0",
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(
        CatalogError,
        match=r"thunder_01\.imu.*200 Hz.*world timestep 0\.002 s",
    ) as exc_info:
        _resolver().resolve(session)

    assert exc_info.value.code == "SIMCATALOG_SENSOR_TIMEBASE_INCOMPATIBLE"
    assert exc_info.value.context == "thunder_01.imu"
    assert exc_info.value.details == {
        "period_ns": 5_000_000,
        "rate_hz": 200,
        "required_relation": "sensor_period_ns % physics_timestep_ns == 0",
        "sensor_id": "thunder_01.imu",
        "timestep_ns": 2_000_000,
        "world": "open_field@1.0.0",
    }


@pytest.mark.parametrize(
    ("configuration", "error"),
    (
        ({}, r"requires configuration\.raycast_frame"),
        (
            {"raycast_frame": "missing_lidar_site"},
            r"raycast_frame 'missing_lidar_site'.*absent from robot frame map",
        ),
    ),
)
def test_mid360_raycast_frame_fails_closed(
    configuration: dict[str, str],
    error: str,
) -> None:
    resolver = _resolver()
    base_dir = THUNDER_UNREAL_SESSION.parent
    robot = resolver._resolve_package("thunderv4@1.0.3", "robot", base_dir)
    rig = resolver._resolve_package(
        "thunderv4_navigation@1.0.0",
        "sensor_rig",
        base_dir,
    )
    rig_sensor = next(item for item in rig.data["sensors"] if item["id"] == "mid360")
    sensor = resolver._resolve_package(
        rig_sensor["package"],
        "sensor",
        rig.manifest_path.parent,
    )

    with pytest.raises(CatalogError, match=error):
        resolver._sensor_stream(
            {
                "instance_id": "thunder_01",
                "rig_sensor": {**rig_sensor, "configuration": configuration},
                "sensor": sensor,
                "robot_frames": resolver._robot_frame_map(robot),
            }
        )


def test_multi_robot_control_inputs_have_distinct_instance_routes(
    tmp_path: Path,
) -> None:
    session = tmp_path / "two_thunder.yaml"
    session.write_text(
        """schema: lingtu.sim.session.v1
session_id: two_thunder_control
mujoco_version: 3.10.0
seed: 11
world: open_field@1.0.0
robots:
  - instance_id: thunder_01
    package: thunderv4@1.0.3
    controller: thunderv4_locomotion@1.0.0
    spawn:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
  - instance_id: thunder_02
    package: thunderv4@1.0.3
    controller: thunderv4_locomotion@1.0.0
    spawn:
      position_m: [2.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
runtime:
  backend: mujoco
  mode: headless
  required_bindings: [physics, control]
""",
        encoding="utf-8",
    )

    resolved = _resolver().resolve(session)
    inputs = [channel for channel in resolved.control_plan["command_channels"] if channel["direction"] == "subscribe"]
    assert [channel["channel_id"] for channel in inputs] == [
        "thunder_01.control.base_twist",
        "thunder_02.control.base_twist",
    ]
    assert [channel["source"] for channel in inputs] == [
        "dds_base_twist/thunder_01",
        "dds_base_twist/thunder_02",
    ]
    assert len({channel["source"] for channel in inputs}) == 2


def test_sensor_stream_uses_mount_frame_parent_extrinsic_and_rig_overrides(tmp_path: Path) -> None:
    catalog = tmp_path / "catalog"
    sensor_dir = catalog / "sensors" / "camera"
    rig_dir = catalog / "sensor_rigs" / "override_rig"
    sensor_dir.mkdir(parents=True)
    rig_dir.mkdir(parents=True)
    (sensor_dir / "sensor.package.yaml").write_text(
        """schema: lingtu.sim.sensor-package.v1
id: test_camera
version: 1.0.0
kind: sensor
sensor_type: rgb_camera
outputs: [rgb]
timing:
  rate_hz: 30
interface:
  transport: camera_shm
  message_type: lingtu.dds.Image
configuration:
  width: 1280
  height: 720
""",
        encoding="utf-8",
    )
    (rig_dir / "sensor-rig.package.yaml").write_text(
        """schema: lingtu.sim.sensor-rig-package.v1
id: override_rig
version: 1.0.0
kind: sensor_rig
sensors:
  - id: front_rgb
    package: test_camera@1.0.0
    parent_frame: front_camera
    runtime: {owner: visual, source: unreal_camera}
    frequency_hz: 15
    extrinsic:
      position_m: [0.1, 0.2, 0.3]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
    configuration:
      width: 640
      exposure: manual
""",
        encoding="utf-8",
    )
    resolver = CatalogResolver(REPO_ROOT, [sensor_dir, rig_dir])
    rig = resolver._resolve_package("override_rig@1.0.0", "sensor_rig", tmp_path)
    sensor = resolver._resolve_package("test_camera@1.0.0", "sensor", tmp_path)

    stream = resolver._sensor_stream(
        {
            "instance_id": "thunder_01",
            "rig_sensor": rig.data["sensors"][0],
            "sensor": sensor,
            "robot_frames": {
                "base_link": {"name": "base_link", "role": "body"},
                "front_camera": {
                    "name": "front_camera",
                    "role": "sensor_mount",
                    "parent_frame": "base_link",
                    "extrinsic": {
                        "position_m": [0.0, 0.0, 0.0],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    },
                },
            },
        }
    )

    assert stream["frame_id"] == "thunder_01/front_camera"
    assert stream["parent_frame_id"] == "thunder_01/base_link"
    assert stream["extrinsic"] == {
        "position_m": [0.1, 0.2, 0.3],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    assert stream["rate_hz"] == 15
    assert stream["width"] == 640
    assert stream["height"] == 720
    assert stream["exposure"] == "manual"


def test_robot_frame_hierarchy_rejects_cycles(tmp_path: Path) -> None:
    robot_dir = tmp_path / "robots" / "cycle_bot"
    asset_dir = robot_dir / "assets"
    asset_dir.mkdir(parents=True)
    (asset_dir / "robot.xml").write_text(
        """<mujoco><worldbody><body name="base_link"><body name="camera_mount"/></body></worldbody>"""
        """<actuator/></mujoco>""",
        encoding="utf-8",
    )
    (robot_dir / "robot.package.yaml").write_text(
        """schema: lingtu.sim.robot-package.v1
id: cycle_bot
version: 1.0.0
kind: robot
physics:
  mjcf: assets/robot.xml
  attach_root: base_link
  root_joint: base_link
  global_options: inherit_session
visual:
  binding: RobotVisual:CycleBot
  projection: visual/robot.visual-projection.json
semantic:
  class: test_robot
frames:
  - name: base_link
    role: body
    parent_frame: camera_mount
  - name: camera_mount
    role: sensor_mount
    parent_frame: base_link
interfaces:
  state: [lingtu.sim.base-state.v1]
  command: [lingtu.sim.joint-torque.v1]
defaults:
  controller:
  sensor_rig:
declared_capabilities: {}
""",
        encoding="utf-8",
    )

    with pytest.raises(CatalogError, match="parent_frame cycle"):
        CatalogResolver(tmp_path, [robot_dir])


def test_robot_package_rejects_duplicate_frame_names(tmp_path: Path) -> None:
    robot_dir = tmp_path / "robots" / "duplicate_frame_bot"
    asset_dir = robot_dir / "assets"
    asset_dir.mkdir(parents=True)
    (asset_dir / "robot.xml").write_text(
        '<mujoco><worldbody><body name="base_link"/></worldbody><actuator/></mujoco>',
        encoding="utf-8",
    )
    (robot_dir / "robot.package.yaml").write_text(
        """schema: lingtu.sim.robot-package.v1
id: duplicate_frame_bot
version: 1.0.0
kind: robot
physics:
  mjcf: assets/robot.xml
  attach_root: base_link
  root_joint: base_link
  global_options: inherit_session
visual:
  binding: RobotVisual:DuplicateFrameBot
  projection: visual/robot.visual-projection.json
semantic:
  class: test_robot
frames:
  - name: base_link
    role: body
  - name: base_link
    role: sensor_origin
interfaces:
  state: [lingtu.sim.base-state.v1]
  command: [lingtu.sim.joint-torque.v1]
defaults:
  controller:
  sensor_rig:
declared_capabilities: {}
""",
        encoding="utf-8",
    )

    with pytest.raises(CatalogError, match=r"duplicate frame 'base_link'"):
        CatalogResolver(tmp_path, [robot_dir])


def test_sensor_rig_rejects_non_normalized_extrinsic_quaternion(tmp_path: Path) -> None:
    rig_dir = tmp_path / "sensor_rigs" / "bad_rig"
    rig_dir.mkdir(parents=True)
    (rig_dir / "sensor-rig.package.yaml").write_text(
        """schema: lingtu.sim.sensor-rig-package.v1
id: bad_rig
version: 1.0.0
kind: sensor_rig
sensors:
  - id: front_rgb
    package: test_camera@1.0.0
    parent_frame: front_camera
    runtime: {owner: visual, source: unreal_camera}
    extrinsic:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [2.0, 0.0, 0.0, 0.0]
""",
        encoding="utf-8",
    )

    with pytest.raises(CatalogError, match="normalized"):
        CatalogResolver(tmp_path, [rig_dir])


def test_unknown_sensor_rig_is_rejected(tmp_path: Path) -> None:
    source = SESSION.read_text(encoding="utf-8")
    broken = source.replace(
        "sensor_rig: thunderv4_headless_navigation@1.0.0",
        "sensor_rig: broken_rig@1.0.0",
    )
    session = tmp_path / "broken.yaml"
    session.write_text(broken, encoding="utf-8")

    with pytest.raises(CatalogError, match="broken_rig"):
        _resolver().resolve(session)


def test_unknown_session_key_is_rejected(tmp_path: Path) -> None:
    source = SESSION.read_text(encoding="utf-8")
    session = tmp_path / "broken.yaml"
    session.write_text(source + "\nextra_key: forbidden\n", encoding="utf-8")

    with pytest.raises(CatalogError, match=r"unknown key.*extra_key"):
        _resolver().resolve(session)


@pytest.mark.parametrize("session_id", ["bad id", "a" * 64, "会话"])
def test_invalid_session_id_is_rejected(tmp_path: Path, session_id: str) -> None:
    source = SESSION.read_text(encoding="utf-8")
    source = source.replace("session_id: thunder_omni_contract", f"session_id: {session_id}")
    session = tmp_path / "broken.yaml"
    session.write_text(source, encoding="utf-8")

    with pytest.raises(CatalogError, match="session_id"):
        _resolver().resolve(session)


def test_package_path_that_escapes_repo_root_is_rejected(tmp_path: Path) -> None:
    external_manifest = Path(tmp_path.anchor) / "external_sensor.package.yaml"
    source = THUNDER_UNREAL_SESSION.read_text(encoding="utf-8")
    escaped = source.replace("sensor_rig: thunderv4_navigation@1.0.0", f"sensor_rig: {external_manifest.as_posix()}")
    session = tmp_path / "broken.yaml"
    session.write_text(escaped, encoding="utf-8")

    with pytest.raises(CatalogError, match="package path escapes repository root"):
        _resolver().resolve(session)


def test_mujoco_compatibility_is_checked(tmp_path: Path) -> None:
    source = SESSION.read_text(encoding="utf-8").replace("mujoco_version: 3.10.0", "mujoco_version: 3.9.0")
    session = tmp_path / "incompatible.yaml"
    session.write_text(source, encoding="utf-8")

    with pytest.raises(CatalogError, match=r"requires MuJoCo 3\.10\.x"):
        _resolver().resolve(session)
