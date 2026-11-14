"""Sensor authoring handoff tests: processes are replaced, not started."""

from __future__ import annotations

import copy
import json
import shutil
import sys
from pathlib import Path

import pytest
import yaml
from sim.catalog import CatalogResolver
from sim.catalog.importers.contracts import canonical_json_bytes, digest_document
from sim.tools import game_selection_launcher as launcher
from sim.tools.sensor_selection import build_sensor_setup

from tests.sim.test_game_selection_launcher import _catalog, _external_files, _intent_document
from tests.sim.test_sim_catalog import _write_robot_package, _write_world_package


@pytest.fixture
def sensor_plan(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    catalog_path, bundle, session_id = _catalog(repo)
    source = repo / "sim/sessions/example/session.yaml"
    source.parent.mkdir(parents=True)
    shutil.copyfile(bundle / "session.yaml", source)
    setup = {
        "schema": "lingtu.sim.sensor-setup.v1",
        "instance_id": "robot_01",
        "rig": {"id": "fixture_rig", "version": "1.0.0"},
        "sensors": [
            {"id": "imu", "enabled": True, "frequency_hz": 100, "required": True},
            {"id": "rgb", "enabled": True, "frequency_hz": 30, "required": False},
        ],
    }
    document = json.loads(catalog_path.read_text(encoding="utf-8"))
    document["entries"][0].update(sensor_setup=setup, session_source=source.relative_to(repo).as_posix())
    document["digest"] = digest_document({key: value for key, value in document.items() if key != "digest"})
    catalog_path.write_bytes(canonical_json_bytes(document))
    unreal, project = _external_files(repo)
    plan = launcher.create_selector_launch_plan(
        repo_root=repo,
        catalog_path=catalog_path,
        unreal_executable=unreal,
        unreal_project=project,
        run_id="sensor-test",
    )
    return plan, bundle, session_id, source, setup


def _choice(setup, *, changed=True):
    return {
        "schema": "lingtu.sim.sensor-selection.v1",
        "instance_id": setup["instance_id"],
        "sensors": [
            {
                "id": row["id"],
                "enabled": row["enabled"],
                "frequency_hz": 15 if changed and row["id"] == "rgb" else row["frequency_hz"],
            }
            for row in setup["sensors"]
        ],
    }


def _select(fixture, choice):
    plan, bundle, session_id, _source, _setup = fixture

    def runner(_argv, *, cwd, timeout):
        assert cwd == plan.repo_root
        assert timeout > 0
        intent = _intent_document(bundle, session_id)
        if choice is not None:
            intent["sensor_selection"] = choice
        plan.intent_path.write_bytes(canonical_json_bytes(intent))
        return 0

    return launcher.execute_selector(plan, process_runner=runner)


def test_sensor_intent_is_validated_and_carried_with_its_authored_source(sensor_plan):
    choice = _choice(sensor_plan[4])
    selection = _select(sensor_plan, choice)
    assert selection.sensor_selection == choice
    assert selection.session_source == sensor_plan[3]
    evidence = json.loads(selection.evidence_path.read_text(encoding="utf-8"))
    assert evidence["sensor_selection"] == choice
    assert evidence["session_source"] == str(sensor_plan[3])


@pytest.mark.parametrize("handoff", ["argv", "execute"])
def test_legacy_playable_cannot_discard_sensor_selection(sensor_plan, handoff):
    selection = _select(sensor_plan, _choice(sensor_plan[4]))
    with pytest.raises(launcher.GameSelectionLauncherError, match="ProductControl"):
        if handoff == "argv":
            launcher.build_playable_coordinator_argv(selection)
        else:
            launcher.execute_playable_coordinator(
                sensor_plan[0],
                selection,
                coordinator_argv=("unused",),
                process_runner=lambda *_args, **_kwargs: pytest.fail("must not start"),
            )


@pytest.fixture
def authored_plan(sensor_plan):
    plan, bundle, session_id, source, _setup = sensor_plan
    root = Path(__file__).resolve().parents[2]
    for directory in ("sensors", "sensor_rigs"):
        shutil.copytree(root / "sim/packages" / directory, plan.repo_root / "sim/packages" / directory)
    packages = plan.repo_root / "sim/packages"
    robot_package = _write_robot_package(packages, "semantic: {class: quadruped}\n")
    robot_manifest = robot_package / "robot.package.yaml"
    robot_document = yaml.safe_load(robot_manifest.read_text(encoding="utf-8"))
    robot_document["visual"]["projection"] = "visual.json"
    robot_manifest.write_text(yaml.safe_dump(robot_document), encoding="utf-8")
    (robot_package / "visual.json").write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.robot-visual-projection.v1",
                "binding": "RobotVisual:TestRobot",
                "package": {"id": "test_robot", "version": "1.0.0"},
                "mjcf": {},
                "components": [{}],
            }
        ),
        encoding="utf-8",
    )
    world_package = _write_world_package(packages, "/Game/Maps/TestWorld")
    world_manifest = world_package / "world.package.yaml"
    world_document = yaml.safe_load(world_manifest.read_text(encoding="utf-8"))
    # The real 200 Hz IMU has a 5 ms period, which requires this 1 ms fixture step.
    world_document["physics"]["global_policy"]["timestep_s"] = 0.001
    world_manifest.write_text(yaml.safe_dump(world_document), encoding="utf-8")
    world_mjcf = world_package / "world.xml"
    world_mjcf.write_text(
        world_mjcf.read_text(encoding="utf-8").replace('timestep="0.002"', 'timestep="0.001"'),
        encoding="utf-8",
    )
    session = yaml.safe_load(
        (root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml").read_text(encoding="utf-8")
    )
    session["world"] = "test_world@1.0.0"
    session["session_id"] = session_id
    session["robots"][0]["package"] = "test_robot@1.0.0"
    session["robots"][0].pop("controller", None)
    source.write_text(yaml.safe_dump(session), encoding="utf-8")
    # The catalog compiler preserves the exact authored source in the bundle.
    shutil.copyfile(source, bundle / "session.yaml")
    setup = build_sensor_setup(CatalogResolver.from_repository(plan.repo_root), source)
    plan.catalog_document["entries"][0]["sensor_setup"] = setup
    document = plan.catalog_document
    document["digest"] = digest_document({key: value for key, value in document.items() if key != "digest"})
    plan.catalog_path.write_bytes(canonical_json_bytes(document))
    return plan, bundle, session_id, source, setup


def test_product_control_authors_changed_session_without_compilation_or_processes(authored_plan, monkeypatch):
    plan, bundle, _session_id, source, setup = authored_plan
    choice = _choice(setup, changed=False)
    next(row for row in choice["sensors"] if row["id"] == "front_rgb")["frequency_hz"] = 15
    selection = _select(authored_plan, choice)
    original_source = source.read_bytes()
    original_bundle = {path.name: path.read_bytes() for path in bundle.iterdir()}
    monkeypatch.setattr(CatalogResolver, "resolve", lambda *_args, **_kwargs: pytest.fail("must not compile"))
    monkeypatch.setattr(launcher.subprocess, "run", lambda *_args, **_kwargs: pytest.fail("must not run"))

    argv = launcher.build_product_control_argv(
        plan,
        selection,
        product="nav",
        robot="doso/thunder_v4",
        map_name="factory",
    )

    authored = plan.repo_root / "sim/sessions/user_selection/sensor-test/session.yaml"
    assert argv == (
        str(Path(sys.executable)),
        "-m",
        "lingtu.control",
        "switch",
        "nav",
        "--env",
        "sim",
        "--backend",
        "mujoco",
        "--robot",
        "doso/thunder_v4",
        "--sim-session",
        authored.relative_to(plan.repo_root).as_posix(),
        "--map",
        "factory",
        "--json",
    )
    document = yaml.safe_load(authored.read_text(encoding="utf-8"))
    assert document["robots"][0]["sensor_overrides"]["front_rgb"]["frequency_hz"] == 15
    assert document["robots"][0]["sensor_rig"] == "thunderv4_navigation@1.1.0"
    assert document["session_id"] != _session_id
    assert source.read_bytes() == original_source
    assert {path.name: path.read_bytes() for path in bundle.iterdir()} == original_bundle


def test_product_control_rejects_stale_controls_before_authoring(authored_plan):
    plan, _bundle, _session_id, source, setup = authored_plan
    choice = _choice(setup, changed=False)
    next(row for row in choice["sensors"] if row["id"] == "front_rgb")["frequency_hz"] = 15
    selection = _select(authored_plan, choice)
    session = yaml.safe_load(source.read_text(encoding="utf-8"))
    session["robots"][0]["sensor_overrides"] = {"front_rgb": {"frequency_hz": 20}}
    source.write_text(yaml.safe_dump(session), encoding="utf-8")

    with pytest.raises(launcher.GameSelectionLauncherError, match=r"sensor_setup.*changed|stale"):
        launcher.build_product_control_argv(
            plan,
            selection,
            product="nav",
            robot="doso/thunder_v4",
            map_name="factory",
        )
    assert not (plan.repo_root / "sim/sessions/user_selection").exists()


@pytest.mark.parametrize("field", ["world", "seed", "spawn", "runtime"])
@pytest.mark.parametrize("explicit", [False, True])
def test_product_control_rejects_source_changes_outside_sensor_setup(authored_plan, field, explicit, monkeypatch):
    plan, bundle, _session_id, source, setup = authored_plan
    selection = _select(authored_plan, _choice(setup, changed=False) if explicit else None)
    saved_session = (bundle / "session.yaml").read_bytes()
    session = yaml.safe_load(source.read_text(encoding="utf-8"))
    if field == "world":
        world_package = plan.repo_root / "sim/packages/worlds/test_world"
        alternate_world = world_package.with_name("alternate_world")
        shutil.copytree(world_package, alternate_world)
        manifest = alternate_world / "world.package.yaml"
        world_document = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        world_document["id"] = "alternate_world"
        manifest.write_text(yaml.safe_dump(world_document), encoding="utf-8")
        session["world"] = "alternate_world@1.0.0"
    elif field == "seed":
        session["seed"] += 1
    elif field == "spawn":
        session["robots"][0]["spawn"]["position_m"][0] += 1
    else:
        session["runtime"]["mode"] = "headless"
    source.write_text(yaml.safe_dump(session), encoding="utf-8")
    assert build_sensor_setup(CatalogResolver.from_repository(plan.repo_root), source) == setup
    monkeypatch.setattr(CatalogResolver, "resolve", lambda *_args, **_kwargs: pytest.fail("must not compile"))
    monkeypatch.setattr(launcher.subprocess, "run", lambda *_args, **_kwargs: pytest.fail("must not run"))

    with pytest.raises(launcher.GameSelectionLauncherError, match=r"session_source.*stale catalog"):
        launcher.build_product_control_argv(
            plan, selection, product="nav", robot="doso/thunder_v4", map_name="factory",
        )

    assert not (plan.repo_root / "sim/sessions/user_selection").exists()
    assert (bundle / "session.yaml").read_bytes() == saved_session


def test_product_control_accepts_semantically_unchanged_source(authored_plan):
    plan, bundle, _session_id, source, _setup = authored_plan
    selection = _select(authored_plan, None)
    session = yaml.safe_load(source.read_text(encoding="utf-8"))
    source.write_text(
        "# Only presentation changed.\n" + yaml.safe_dump(dict(reversed(list(session.items()))), sort_keys=False),
        encoding="utf-8",
    )
    assert source.read_bytes() != (bundle / "session.yaml").read_bytes()
    argv = launcher.build_product_control_argv(
        plan, selection, product="nav", robot="doso/thunder_v4", map_name="factory",
    )
    assert argv[argv.index("--sim-session") + 1] == source.relative_to(plan.repo_root).as_posix()
    assert not (plan.repo_root / "sim/sessions/user_selection").exists()


@pytest.mark.parametrize("explicit", [False, True])
def test_no_changed_sensor_choices_reuse_source_without_authoring(authored_plan, explicit):
    plan, _bundle, _session_id, source, setup = authored_plan
    selection = _select(authored_plan, _choice(setup, changed=False) if explicit else None)
    argv = launcher.build_product_control_argv(
        plan,
        selection,
        product="teleop",
        robot="doso/thunder_v4",
    )
    assert argv[argv.index("--sim-session") + 1] == source.relative_to(plan.repo_root).as_posix()
    assert "--map" not in argv
    assert not (plan.repo_root / "sim/sessions/user_selection").exists()


@pytest.mark.parametrize("product", ["nav", "inspection"])
def test_map_products_require_map_before_any_authoring(authored_plan, product):
    plan, _bundle, _session_id, _source, setup = authored_plan
    selection = _select(authored_plan, _choice(setup, changed=False))
    with pytest.raises(launcher.GameSelectionLauncherError, match="map"):
        launcher.build_product_control_argv(plan, selection, product=product, robot="doso/thunder_v4")
    assert not (plan.repo_root / "sim/sessions/user_selection").exists()


def _cli(fixture):
    plan = fixture[0]
    return [
        "--repo-root",
        str(plan.repo_root),
        "--catalog",
        str(plan.catalog_path),
        "--unreal-executable",
        plan.selector_argv[0],
        "--unreal-project",
        plan.selector_argv[1],
        "--run-id",
        "cli-selection",
    ]


def test_execute_product_cli_only_hands_off_to_one_product_control_process(authored_plan, capsys):
    plan, bundle, session_id, _source, setup = authored_plan
    choice = _choice(setup, changed=False)
    next(row for row in choice["sensors"] if row["id"] == "front_depth")["enabled"] = False
    calls = []

    def runner(argv, *, cwd, timeout=None):
        assert cwd == plan.repo_root
        calls.append(argv)
        if len(calls) == 1:
            intent_path = next(arg.split("=", 1)[1] for arg in argv if arg.startswith("-LingTuGameSelectionIntent="))
            Path(intent_path).write_bytes(
                canonical_json_bytes(
                    _intent_document(
                        bundle,
                        session_id,
                        sensor_selection=choice,
                    )
                )
            )
            assert timeout == 900.0
        else:
            assert timeout is None
        return 0

    result = launcher.main(
        [
            *_cli(authored_plan),
            "--execute-selector",
            "--execute-product",
            "--product",
            "nav",
            "--robot",
            "doso/thunder_v4",
            "--map",
            "factory",
        ],
        process_runner=runner,
    )
    assert result == 0
    assert len(calls) == 2
    assert calls[1][1:5] == ("-m", "lingtu.control", "switch", "nav")
    assert calls[1][calls[1].index("--sim-session") + 1] == "sim/sessions/user_selection/cli-selection/session.yaml"
    report = json.loads(capsys.readouterr().out)
    assert report["mode"] == "product_complete"
    assert report["product_control_argv"] == list(calls[1])


def test_product_control_enforces_requested_inspection_dependencies(authored_plan):
    plan, _bundle, _session_id, _source, setup = authored_plan
    choice = _choice(setup, changed=False)
    next(row for row in choice["sensors"] if row["id"] == "front_rgb")["enabled"] = False
    selection = _select(authored_plan, choice)
    with pytest.raises(launcher.GameSelectionLauncherError, match=r"required.*sensor"):
        launcher.build_product_control_argv(
            plan,
            selection,
            product="inspection",
            robot="doso/thunder_v4",
            map_name="factory",
        )
    assert not (plan.repo_root / "sim/sessions/user_selection").exists()


@pytest.mark.parametrize(
    "invalid", ["instance", "schema", "unknown", "duplicate", "missing", "required", "rate", "boolean", "extra", "null"]
)
def test_invalid_sensor_intent_never_produces_validated_evidence(sensor_plan, invalid):
    choice = _choice(sensor_plan[4])
    if invalid == "instance":
        choice["instance_id"] = "wrong_robot"
    elif invalid == "schema":
        choice["schema"] = "lingtu.sim.sensor-selection.v2"
    elif invalid == "unknown":
        choice["sensors"][0]["id"] = "unknown"
    elif invalid == "duplicate":
        choice["sensors"].append(copy.deepcopy(choice["sensors"][0]))
    elif invalid == "missing":
        choice["sensors"].pop()
    elif invalid == "required":
        choice["sensors"][0]["enabled"] = False
    elif invalid == "rate":
        choice["sensors"][1]["frequency_hz"] = 0
    elif invalid == "boolean":
        choice["sensors"][1]["enabled"] = 1
    elif invalid == "extra":
        choice["arbitrary"] = True
    else:
        choice = None
        plan, bundle, session_id, _source, _setup = sensor_plan

        def runner(_argv, *, cwd, timeout):
            plan.intent_path.write_bytes(
                canonical_json_bytes(_intent_document(bundle, session_id, sensor_selection=None))
            )
            return 0

        with pytest.raises(launcher.GameSelectionLauncherError, match="sensor_selection"):
            launcher.execute_selector(plan, process_runner=runner)
        assert not (plan.run_dir / "selection.validated.json").exists()
        return
    with pytest.raises(launcher.GameSelectionLauncherError, match="sensor_selection"):
        _select(sensor_plan, choice)
    assert not (sensor_plan[0].run_dir / "selection.validated.json").exists()


@pytest.mark.parametrize("metadata", ["sensor_setup", "session_source"])
def test_sensor_intent_requires_catalog_authoring_metadata(sensor_plan, metadata):
    plan = sensor_plan[0]
    document = plan.catalog_document
    document["entries"][0].pop(metadata)
    document["digest"] = digest_document({key: value for key, value in document.items() if key != "digest"})
    plan.catalog_path.write_bytes(canonical_json_bytes(document))
    with pytest.raises(launcher.GameSelectionLauncherError, match="sensor_setup and session_source"):
        _select(sensor_plan, _choice(sensor_plan[4]))


@pytest.mark.parametrize(
    "options, missing",
    [
        (["--robot", "doso/thunder_v4"], "--product"),
        (["--product", "nav", "--map", "factory"], "--robot"),
        (["--product", "nav", "--robot", "doso/thunder_v4"], "--map"),
        (["--product", "inspection", "--robot", "doso/thunder_v4"], "--map"),
    ],
)
def test_execute_product_requires_explicit_arguments_before_selector(sensor_plan, options, missing, capsys):
    result = launcher.main(
        [*_cli(sensor_plan), "--execute-selector", "--execute-product", *options],
        process_runner=lambda *_args, **_kwargs: pytest.fail("must not start selector"),
    )
    assert result == 1
    assert missing in json.loads(capsys.readouterr().err)["error"]
    assert not (sensor_plan[0].run_dir.parent / "cli-selection").exists()


@pytest.mark.parametrize(
    "flags", [["--execute-product"], ["--execute-selector", "--execute-product", "--execute-playable"]]
)
def test_product_cli_requires_opt_in_and_excludes_legacy_execution(sensor_plan, flags):
    with pytest.raises(SystemExit):
        launcher.main(
            [*_cli(sensor_plan), *flags], process_runner=lambda *_args, **_kwargs: pytest.fail("must not start")
        )
    assert not (sensor_plan[0].run_dir.parent / "cli-selection").exists()


def test_sensor_selector_only_returns_product_handoff_without_authoring(sensor_plan, capsys):
    plan, bundle, session_id, _source, setup = sensor_plan
    calls = []

    def runner(argv, *, cwd, timeout):
        calls.append(argv)
        path = next(arg.split("=", 1)[1] for arg in argv if arg.startswith("-LingTuGameSelectionIntent="))
        Path(path).write_bytes(
            canonical_json_bytes(_intent_document(bundle, session_id, sensor_selection=_choice(setup)))
        )
        return 0

    assert launcher.main([*_cli(sensor_plan), "--execute-selector"], process_runner=runner) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["mode"] == "selector_complete"
    assert report["handoff"] == "ProductControl"
    assert "coordinator_argv" not in report
    assert len(calls) == 1
    assert not (plan.repo_root / "sim/sessions/user_selection").exists()


@pytest.mark.parametrize("returncode", [17, True, None])
def test_product_process_failure_is_reported_not_success(authored_plan, capsys, returncode):
    _plan, bundle, session_id, _source, _setup = authored_plan
    calls = []

    def runner(argv, *, cwd, timeout=None):
        calls.append(argv)
        if len(calls) == 1:
            path = next(arg.split("=", 1)[1] for arg in argv if arg.startswith("-LingTuGameSelectionIntent="))
            Path(path).write_bytes(canonical_json_bytes(_intent_document(bundle, session_id)))
            return 0
        return returncode

    result = launcher.main(
        [
            *_cli(authored_plan),
            "--execute-selector",
            "--execute-product",
            "--product",
            "teleop",
            "--robot",
            "doso/thunder_v4",
        ],
        process_runner=runner,
    )
    assert result == 1
    assert len(calls) == 2
    assert "ProductControl" in json.loads(capsys.readouterr().err)["error"]
