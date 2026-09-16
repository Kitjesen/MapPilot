from __future__ import annotations

import base64
import json

import numpy as np
import pytest

from gateway.services.cloud_viewer import CloudViewerService
from runtime.msgs.map import MapSceneFrame


def _service(events: list[dict]) -> CloudViewerService:
    return CloudViewerService(
        queue_put_latest=lambda queue, data, loop, record: queue.put_nowait(data),
        current_loop=lambda: None,
        push_event=events.append,
        session_mode=lambda: "mapping",
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )


def _frame(grid: np.ndarray, *, stamp_s: float = 100.0, reset_epoch: int = 2) -> MapSceneFrame:
    return MapSceneFrame(
        ts=stamp_s,
        frame_id="map",
        source="mapd",
        sequence=20,
        metadata={
            "producer_boot_id": "mapd-boot",
            "reset_epoch": reset_epoch,
            "generation": 15,
            "observation_sequence": 10,
            "live": True,
        },
        layers=[{
            "id": "maps.occupancy",
            "type": "grid",
            "topic": "/maps/occupancy",
            "source": "mapd",
            "metadata": {
                "height": grid.shape[0],
                "width": grid.shape[1],
                "resolution": 0.05,
                "origin": {"x": -5.0, "y": -3.0, "z": 0.25, "yaw": 0.2},
            },
            "grid": grid,
        }],
    )


def _cells(layer: dict) -> np.ndarray:
    return np.frombuffer(base64.b64decode(layer["grid_b64"]), dtype=np.int8).reshape(layer["rows"], layer["cols"])


def test_occupancy_preserves_cells_geometry_identity_and_source_stamp() -> None:
    events: list[dict] = []
    source = np.array([[-1, 0, 100], [100, -1, 0]], dtype=np.float32)
    _service(events).on_map_scene(_frame(source))
    layer = events[-1]["layers"][0]

    np.testing.assert_array_equal(_cells(layer), source)
    assert layer["encoding"] == "int8"
    assert layer["value_semantics"] == "height_band_occupancy_not_traversability"
    assert layer["scope"] == "rolling_window"
    assert layer["resolution"] == 0.05
    assert layer["origin"] == [-5.0, -3.0, 0.25]
    assert layer["yaw"] == 0.2
    assert layer["downsample_factor"] == 1
    assert layer["frame_id"] == "map"
    assert layer["producer_boot_id"] == "mapd-boot"
    assert layer["stamp_s"] == 100.0
    assert layer["reset_epoch"] == "2"
    assert layer["generation"] == 15
    assert layer["observation_sequence"] == 10
    assert layer["live"] is True
    assert (layer["unknown_count"], layer["free_count"], layer["occupied_count"]) == (2, 2, 2)
    assert layer["payload"] == "inline"
    assert layer["retain_previous"] is False
    assert "grid" not in layer


def test_browser_scene_preserves_field_uint64_epochs_without_changing_native_identity(monkeypatch) -> None:
    field_epoch = 117269136217079808
    assert float(field_epoch) == float(field_epoch + 1)
    now = [100.0]
    monkeypatch.setattr("gateway.services.cloud_viewer.time.time", lambda: now[0])
    events: list[dict] = []
    service = _service(events)

    for epoch in (field_epoch, field_epoch + 1):
        frame = _frame(np.array([[0, 100]], dtype=np.float32), reset_epoch=epoch)
        source_metadata = {**frame.layers[0]["metadata"], **frame.metadata}
        frame.layers.extend([
            {
                "id": "maps.elevation", "type": "grid", "source": "mapd",
                "metadata": dict(source_metadata),
                "grid": np.array([[-0.375, 1.0]], dtype=np.float32),
            },
            {"id": "maps.esdf", "type": "grid", "metadata": dict(source_metadata)},
        ])
        service.on_map_scene(frame)
        wire = json.loads(json.dumps(events[-1]))
        assert wire["metadata"]["reset_epoch"] == str(epoch)
        occupancy, elevation, esdf = wire["layers"]
        assert occupancy["reset_epoch"] == str(epoch)
        assert elevation["reset_epoch"] == str(epoch)
        assert esdf["metadata"]["reset_epoch"] == str(epoch)
        assert occupancy["generation"] == elevation["generation"] == 15
        assert occupancy["observation_sequence"] == elevation["observation_sequence"] == 10
        assert frame.metadata["reset_epoch"] == epoch
        assert frame.layers[1]["metadata"]["reset_epoch"] == epoch
        now[0] += 0.1

    assert events[0]["layers"][1]["payload"] == "inline"
    reset_elevation = events[1]["layers"][1]
    assert reset_elevation["reason"] == "rate_limited"
    assert reset_elevation["retain_previous"] is False


def test_occupancy_coarsens_only_oversized_grids_and_never_frees_unknown_blocks() -> None:
    events: list[dict] = []
    source = np.zeros((513, 513), dtype=np.float32)
    source[0, 0] = 100
    source[1, 0] = -1
    source[0, 2] = -1
    source[-1, -1] = 100
    _service(events).on_map_scene(_frame(source))
    layer = events[-1]["layers"][0]
    cells = _cells(layer)

    assert cells.size <= 131_072
    assert layer["downsample_factor"] == 2
    assert layer["resolution"] == 0.1
    assert layer["origin"] == [-5.0, -3.0, 0.25]
    assert cells[0, :3].tolist() == [100, -1, 0]
    assert cells[-1, -1] == 100
    assert np.all(cells[-1, :-1] == -1)
    assert np.all(cells[:-1, -1] == -1)
    for name, value in (("unknown", -1), ("free", 0), ("occupied", 100)):
        assert layer[f"{name}_count"] == np.count_nonzero(cells == value)


def test_occupancy_keeps_native_resolution_at_cell_budget() -> None:
    events: list[dict] = []
    source = np.zeros((256, 512), dtype=np.float32)
    _service(events).on_map_scene(_frame(source))
    layer = events[-1]["layers"][0]
    assert layer["downsample_factor"] == 1
    assert layer["resolution"] == 0.05
    assert _cells(layer).shape == source.shape


@pytest.mark.parametrize("bad_value", [1.0, 255.0, 256.0, np.nan])
def test_occupancy_rejects_invalid_cells_before_int8_encoding(bad_value: float) -> None:
    events: list[dict] = []
    _service(events).on_map_scene(_frame(np.array([[0.0, bad_value]], dtype=np.float32)))
    layer = events[-1]["layers"][0]
    assert layer["payload"] == "omitted"
    assert layer["retain_previous"] is False
    assert "grid_b64" not in layer


def test_occupancy_metadata_only_reset_does_not_reuse_previous_cells() -> None:
    events: list[dict] = []
    service = _service(events)
    service.on_map_scene(_frame(np.array([[100, 0]], dtype=np.float32)))
    reset = _frame(np.array([[0, 0]], dtype=np.float32), stamp_s=101.0, reset_epoch=3)
    reset.layers[0].pop("grid")
    service.on_map_scene(reset)
    assert events[-1]["layers"][0]["payload"] == "omitted"
    assert events[-1]["layers"][0]["retain_previous"] is False
    assert "grid_b64" not in events[-1]["layers"][0]
    service.on_map_scene(_frame(np.array([[0, -1]], dtype=np.float32), stamp_s=102.0, reset_epoch=3))
    layer = events[-1]["layers"][0]
    assert layer["reset_epoch"] == "3"
    assert layer["stamp_s"] == 102.0
    assert _cells(layer).tolist() == [[0, -1]]


@pytest.mark.parametrize("problem", ["stamp", "producer", "shape", "origin"])
def test_occupancy_missing_identity_or_invalid_geometry_omits_payload(problem: str) -> None:
    events: list[dict] = []
    frame = _frame(np.array([[0, 100]], dtype=np.float32))
    if problem == "stamp":
        frame.ts = 0.0
        frame.layers[0]["ts"] = 0.0
    elif problem == "producer":
        frame.metadata.pop("producer_boot_id")
    elif problem == "shape":
        frame.layers[0]["metadata"]["width"] = 3
    else:
        frame.layers[0]["metadata"]["origin"]["x"] = np.nan
    _service(events).on_map_scene(frame)
    assert events[-1]["layers"][0]["payload"] == "omitted"
    assert "grid_b64" not in events[-1]["layers"][0]


def test_surface_projection_is_preserved_with_distinct_semantics() -> None:
    events: list[dict] = []
    source = np.array([[-1, 0, 100]], dtype=np.float32)
    frame = _frame(source)
    frame.layers[0]["id"] = "maps.surface_projection"
    frame.layers[0].pop("topic")
    _service(events).on_map_scene(frame)
    layer = events[-1]["layers"][0]
    assert layer["value_semantics"] == "ground_relative_surface_not_traversability"
    assert "topic" not in layer
    np.testing.assert_array_equal(_cells(layer), source)


def test_ground_diagnostics_remain_distinct_float_grids() -> None:
    events: list[dict] = []
    frame = _frame(np.array([[-1, 0]], dtype=np.float32))
    metadata = {**frame.layers[0]["metadata"], **frame.metadata}
    frame.layers.extend([
        {
            "id": "maps.ground_height", "type": "grid", "source": "mapd",
            "metadata": dict(metadata), "grid": np.array([[0.1, np.nan]], dtype=np.float32),
        },
        {
            "id": "maps.ground_roughness", "type": "grid", "source": "mapd",
            "metadata": dict(metadata), "grid": np.array([[0.01, np.nan]], dtype=np.float32),
        },
        {
            "id": "maps.ground_support", "type": "grid", "source": "mapd",
            "metadata": dict(metadata), "grid": np.array([[3.0, 0.0]], dtype=np.float32),
        },
    ])
    _service(events).on_map_scene(frame)
    height, roughness, support = events[-1]["layers"][1:]
    assert [layer["value_semantics"] for layer in (height, roughness, support)] == [
        "local_surface_fit_height_m",
        "local_surface_fit_residual_rms_m",
        "distinct_fine_xy_support_count",
    ]
    assert [layer["encoding"] for layer in (height, roughness, support)] == ["float32_le"] * 3
    assert [layer["generation"] for layer in (height, roughness, support)] == [15] * 3
    assert [layer["reset_epoch"] for layer in (height, roughness, support)] == ["2"] * 3
    for layer in (height, roughness, support):
        assert layer["payload"] == "inline"
        assert layer["retain_previous"] is False
        assert "grid" not in layer
