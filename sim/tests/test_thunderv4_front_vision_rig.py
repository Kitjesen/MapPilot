"""Front-vision SensorRig package contract."""

# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

from sim.catalog import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_front_vision_rig_is_a_camera_only_640x480_30hz_preset() -> None:
    resolver = CatalogResolver.from_repository(REPO_ROOT)

    record = resolver.find_package(
        "thunderv4_front_vision@1.0.0",
        kind="sensor_rig",
    )

    assert record.data["id"] == "thunderv4_front_vision"
    assert [sensor["id"] for sensor in record.data["sensors"]] == [
        "front_rgb",
        "front_depth",
    ]
    for sensor in record.data["sensors"]:
        assert sensor["frequency_hz"] == 30
        assert sensor["configuration"] == {"height": 480, "width": 640}
