"""Startup choices must change physical configuration, not just UI labels."""

from copy import deepcopy

import pytest
import yaml
from sim.catalog import CatalogError, CatalogResolver
from sim.tools.sensor_selection import build_sensor_setup, validate_sensor_selection, write_sensor_selection

from tests.sim.test_multimodal_sensor_packages import fixture_world, isolated_repository, repository


def choice_for(setup):
    return dict(
        schema="lingtu.sim.sensor-selection.v1", instance_id=setup["instance_id"],
        sensors=[{key: row[key] for key in (
            "id", "enabled", "frequency_hz", "camera_preset", "pan_deg", "tilt_deg", "gnss_fix_available"
        ) if key in row} for row in setup["sensors"]],
    )


@pytest.mark.parametrize("projection", ["pinhole", "equidistant_fisheye", "equirectangular"])
def test_camera_choice_reaches_compiled_projection_and_gimbal(fixture_world, projection):
    resolver, source, _ = fixture_world
    original = source.read_bytes()
    setup = build_sensor_setup(resolver, source)
    row = next(row for row in setup["sensors"] if row["modality"] == "thermal")
    assert row["camera_preset"] == "pinhole"
    assert row["pan_limits_deg"] == [-180, 180]
    selection = choice_for(setup)
    selected = next(row for row in selection["sensors"] if row["id"] == "front_thermal")
    selected.update(camera_preset=projection, pan_deg=15.0, tilt_deg=-10.0)
    authored = write_sensor_selection(resolver.repo_root, source, selection,
                                     resolver.repo_root / f"authored-{projection}", product="nav")
    stream = resolver.resolve(authored).sensor_plan["streams"]["thermal"][0]
    assert stream["measurement"]["camera"]["model"] == projection
    assert stream["measurement"]["gimbal"]["pan_deg"] == 15.0
    assert stream["measurement"]["gimbal"]["tilt_deg"] == -10.0
    assert source.read_bytes() == original


@pytest.mark.parametrize("settings", [
    {"camera_preset": "filter"}, {"pan_deg": 181}, {"tilt_deg": float("nan")},
    {"gnss_fix_available": True},
])
def test_invalid_camera_choice_rejected_without_authoring(fixture_world, settings):
    resolver, source, _ = fixture_world
    setup = build_sensor_setup(resolver, source)
    selection = choice_for(setup)
    next(row for row in selection["sensors"] if row["id"] == "front_thermal").update(settings)
    destination = resolver.repo_root / "rejected"
    with pytest.raises(ValueError):
        write_sensor_selection(resolver.repo_root, source, selection, destination, product="nav")
    assert not destination.exists()


def test_acoustic_rate_is_block_cadence_not_unrestricted_sample_rate(fixture_world):
    resolver, source, _ = fixture_world
    setup = build_sensor_setup(resolver, source)
    microphone = next(row for row in setup["sensors"] if row["modality"] == "acoustic")
    assert microphone["fixed_frequency_hz"] == 100
    assert "16000" in microphone["notice"]
    selection = choice_for(setup)
    next(row for row in selection["sensors"] if row["id"] == "microphone")["frequency_hz"] = 50
    with pytest.raises(ValueError, match="cadence"):
        validate_sensor_selection(setup, selection)


def test_missing_world_properties_are_visible_and_cannot_be_enabled(fixture_world):
    resolver, source, _ = fixture_world
    world = resolver.find_package("multimodal_contract_fixture@1.0.0")
    del world.data["measurements"]
    setup = build_sensor_setup(resolver, source)
    for row in setup["sensors"]:
        if row["modality"] in {"thermal", "semantic", "acoustic"}:
            assert row["unavailable_reason"]
    with pytest.raises(ValueError, match="World"):
        validate_sensor_selection(setup, choice_for(setup))


def test_gnss_simulation_fix_is_explicit_and_reaches_native_configuration(repository):
    import tempfile

    resolver = repository
    root = resolver.repo_root
    source = root / "sim/sessions/products/doso/thunder_v4/gnss.yaml"
    setup = build_sensor_setup(resolver, source)
    gnss = next(row for row in setup["sensors"] if row["modality"] == "gnss")
    assert gnss["gnss_fix_available"] is False
    assert "WGS84" in gnss["notice"]
    selection = choice_for(setup)
    next(row for row in selection["sensors"] if row["id"] == "gnss")["gnss_fix_available"] = True
    with tempfile.TemporaryDirectory(dir=root / "build") as directory:
        from pathlib import Path

        authored = write_sensor_selection(root, source, selection, Path(directory) / "authored-gnss", product="nav")
        stream = resolver.resolve(authored).sensor_plan["streams"]["gnss"][0]
    assert stream["gnss"]["fix_available"] is True
    assert stream["gnss"]["origin_latitude_deg"] == 0


def test_direct_session_rejects_camera_controls_on_imu(fixture_world):
    resolver, source, _ = fixture_world
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    document["robots"][0]["sensor_overrides"] = {"imu": {"camera_preset": "pinhole"}}
    source.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(CatalogError, match="camera"):
        resolver.resolve(source)


def test_default_explicit_camera_settings_do_not_rewrite_session(fixture_world):
    resolver, source, _ = fixture_world
    setup = build_sensor_setup(resolver, source)
    choice = choice_for(setup)
    original = deepcopy(choice)
    assert write_sensor_selection(resolver.repo_root, source, choice, resolver.repo_root / "unused", product="nav") == source
    assert choice == original
