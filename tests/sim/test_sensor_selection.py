from pathlib import Path

import pytest
from sim.catalog import CatalogResolver
from sim.tools.sensor_selection import build_sensor_setup


def test_setup_projects_the_actual_rig_and_navigation_dependencies():
    root = Path(__file__).resolve().parents[2]
    resolver = CatalogResolver.from_repository(root)
    source = root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
    setup = build_sensor_setup(resolver, source, product="nav")
    assert setup["schema"] == "lingtu.sim.sensor-setup.v1"
    assert setup["instance_id"] == "thunder_01"
    assert setup["rig"] == {"id": "thunderv4_navigation", "version": "1.1.0"}
    rows = {row["id"]: row for row in setup["sensors"]}
    assert set(rows) == {"imu", "mid360", "front_rgb", "front_depth", "truth_odom",
                         "gnss", "front_thermal", "front_semantic", "microphone"}
    assert rows["imu"]["required"] is True
    assert rows["mid360"]["required"] is True
    assert rows["front_rgb"]["required"] is False
    assert rows["front_rgb"]["frequency_hz"] == 30
    assert rows["front_rgb"]["parent_frame"] == "front_camera"


def test_authored_selection_changes_compiled_streams_not_source_packages(tmp_path):
    import copy
    import tempfile

    import yaml
    from sim.tools.sensor_selection import write_sensor_selection

    root = Path(__file__).resolve().parents[2]
    source = root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
    original = source.read_bytes()
    resolver = CatalogResolver.from_repository(root)
    setup = build_sensor_setup(resolver, source)
    selection = dict(
        schema="lingtu.sim.sensor-selection.v1",
        instance_id=setup["instance_id"],
        sensors=[
            dict(
                id=row["id"],
                enabled=row["enabled"] and row["id"] != "front_depth",
                frequency_hz=15 if row["id"] == "front_rgb" else row["frequency_hz"],
            )
            for row in setup["sensors"]
        ],
    )
    unchanged = copy.deepcopy(selection)
    with tempfile.TemporaryDirectory(prefix="sensor-choice-test-", dir=root / "build") as temp:
        authored = write_sensor_selection(root, source, selection, Path(temp) / "authored", product="nav")
        resolved = resolver.resolve(authored)
        streams = [stream for group in resolved.sensor_plan["streams"].values() for stream in group]
        assert not any(stream["sensor_id"].endswith(".front_depth") for stream in streams)
        rgb = next(stream for stream in streams if stream["sensor_id"].endswith(".front_rgb"))
        assert rgb["rate_hz"] == 15
        session = yaml.safe_load(authored.read_text(encoding="utf-8"))
        assert session["robots"][0]["sensor_rig"] == "thunderv4_navigation@1.1.0"
    assert source.read_bytes() == original
    assert selection == unchanged


def test_real_compiled_selector_catalog_exposes_editable_source_bound_rig():
    import tempfile

    import yaml
    from sim.tools.game_selection_catalog import build_game_selection_catalog

    root = Path(__file__).resolve().parents[2]
    spec = yaml.safe_load((root / "sim/sessions/game-selection.v1.yaml").read_text(encoding="utf-8"))
    spec["entries"] = spec["entries"][:1]
    with tempfile.TemporaryDirectory(prefix="sensor-catalog-test-", dir=root / "build") as temp:
        catalog = build_game_selection_catalog(repo_root=root, selection_spec=spec, output_root=Path(temp) / "catalog")
        entry = catalog["entries"][0]
        assert entry["session_source"] == spec["entries"][0]["session_spec"]
        assert entry["sensor_setup"]["instance_id"] == "thunder_01"
        assert len(entry["sensor_setup"]["sensors"]) == 9


def test_invalid_physics_rate_rejected_before_any_authoring():
    import tempfile

    from sim.tools.sensor_selection import write_sensor_selection

    root = Path(__file__).resolve().parents[2]
    source = root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
    setup = build_sensor_setup(CatalogResolver.from_repository(root), source)
    selection = dict(
        schema="lingtu.sim.sensor-selection.v1",
        instance_id=setup["instance_id"],
        sensors=[
            dict(id=row["id"], enabled=True, frequency_hz=30 if row["id"] == "imu" else row["frequency_hz"])
            for row in setup["sensors"]
        ],
    )
    with tempfile.TemporaryDirectory(dir=root / "build") as temp:
        destination = Path(temp) / "invalid"
        with pytest.raises(ValueError, match="physics timestep"):
            write_sensor_selection(root, source, selection, destination, product="nav")
        assert not destination.exists()


def test_unchanged_selection_still_checks_requested_product_dependencies():
    import tempfile

    from sim.tools.sensor_selection import write_sensor_selection

    root = Path(__file__).resolve().parents[2]
    source = root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
    setup = build_sensor_setup(CatalogResolver.from_repository(root), source)
    selection = dict(
        schema="lingtu.sim.sensor-selection.v1",
        instance_id=setup["instance_id"],
        sensors=[dict(id=row["id"], enabled=row["enabled"] and row["id"] != "front_rgb") for row in setup["sensors"]],
    )
    with tempfile.TemporaryDirectory(dir=root / "build") as temp:
        authored = write_sensor_selection(root, source, selection, Path(temp) / "nav", product="nav")
        destination = Path(temp) / "inspection"
        with pytest.raises(ValueError, match="required navigation/inspection sensor"):
            write_sensor_selection(root, authored, None, destination, product="inspection")
        assert not destination.exists()
        assert write_sensor_selection(root, source, None, destination, product="inspection") == source
        assert not destination.exists()


def test_session_resolver_rejects_explicit_null_override_rate(tmp_path):
    import yaml
    from sim.catalog import CatalogError

    root = Path(__file__).resolve().parents[2]
    source = root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    document["robots"][0]["sensor_overrides"] = {"imu": {"frequency_hz": None}}
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(CatalogError, match="positive and finite"):
        CatalogResolver.from_repository(root).resolve(path)


@pytest.mark.parametrize("version", [1, 2])
def test_session_override_schema_accepts_choices_and_rejects_invalid_fields(version):
    import copy
    import json

    import yaml

    jsonschema = pytest.importorskip("jsonschema")
    root = Path(__file__).resolve().parents[2]
    source = root / "sim/sessions/examples/thunderv4_factory_park_hf/session.yaml"
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    document["schema"] = f"lingtu.sim.session.v{version}"
    document["robots"][0]["sensor_overrides"] = {"front_depth": {"enabled": False}, "front_rgb": {"frequency_hz": 15}}
    schema = json.loads((root / f"sim/contracts/schemas/session.v{version}.json").read_text())
    jsonschema.validate(document, schema)
    for override in (
        {"frequency_hz": None},
        {"frequency_hz": True},
        {"frequency_hz": 0},
        {"enabled": "false"},
        {"unknown": 1},
    ):
        invalid = copy.deepcopy(document)
        invalid["robots"][0]["sensor_overrides"] = {"imu": override}
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(invalid, schema)
