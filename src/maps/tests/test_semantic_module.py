from __future__ import annotations

from pathlib import Path

import pytest

from maps.modules import semantic as semantic_module
from runtime.msgs.geometry import Pose, Transform, Vector3
from runtime.msgs.map import (
    MapObservationFrame,
    SemanticLabelsFrame,
    SemanticSaveRequest,
)
from runtime.msgs.numpy_compat import np


class _FakeNative:
    def __init__(self, **_config) -> None:
        self.calls = []
        self.current_generation = 0
        self.labels = None
        self.saved = []

    def close(self) -> None:
        return None

    def update(self, points, **kwargs):
        self.current_generation += 1
        self.labels = kwargs.get("labels")
        self.calls.append((np.asarray(points).copy(), kwargs))
        return {
            "generation_before": self.current_generation - 1,
            "generation_after": self.current_generation,
            "duplicate_sequence": 0,
            "stale_sequence": 0,
        }

    def snapshot(self, *, min_occupancy: float):
        assert min_occupancy == pytest.approx(0.5)
        labels = np.asarray([int(self.labels[0])] if self.labels is not None else [0], dtype=np.uint16)
        confidence = np.asarray([0.9] if self.labels is not None else [0.0], dtype=np.float32)
        return {
            "center_x_m": np.asarray([1.0], dtype=np.float32),
            "center_y_m": np.asarray([2.0], dtype=np.float32),
            "center_z_m": np.asarray([0.5], dtype=np.float32),
            "mean_x_m": np.asarray([1.1], dtype=np.float32),
            "mean_y_m": np.asarray([2.1], dtype=np.float32),
            "mean_z_m": np.asarray([0.4], dtype=np.float32),
            "point_count": np.asarray([1], dtype=np.uint32),
            "dominant_label": labels,
            "semantic_confidence": confidence,
            "generation": self.current_generation,
        }

    def metadata(self):
        return {
            "generation": self.current_generation,
            "voxel_count": 1,
            "voxel_size_m": 0.2,
            "frame_id": "map",
            "taxonomy": "lingtu.semantic" if self.labels is not None else "",
            "taxonomy_version": 1 if self.labels is not None else 0,
        }

    def save(self, path: str, *, expected_generation: int | None = None) -> None:
        assert expected_generation == self.current_generation
        Path(path).write_bytes(b"semantic-map")
        self.saved.append((path, expected_generation))

    @classmethod
    def validate_file(cls, path: str) -> tuple[bool, str]:
        return Path(path).read_bytes() == b"semantic-map", "validated"


def _observation(sequence: int = 1, ts: float = 10.0) -> MapObservationFrame:
    transform = Transform(
        translation=Vector3(1.0, 2.0, 0.5),
        frame_id="map",
        child_frame_id="body",
        ts=ts,
    )
    return MapObservationFrame(
        points=np.asarray([[0.1, 0.0, 0.0]], dtype=np.float32),
        sequence=sequence,
        ts=ts,
        frame_id="map",
        sensor_frame_id="body",
        sensor_origin=transform.translation,
        map_sensor_pose=Pose(transform.translation, transform.rotation),
        map_sensor_transform=transform,
    )


def _module(monkeypatch) -> tuple[semantic_module.SemanticMapModule, _FakeNative]:
    monkeypatch.setattr(semantic_module, "NativeSemanticLayer", _FakeNative)
    module = semantic_module.SemanticMapModule(publish_interval=0.0)
    module.setup()
    assert isinstance(module._native, _FakeNative)
    return module, module._native


def test_semantic_module_geometry_only_updates_native_map(monkeypatch) -> None:
    module, native = _module(monkeypatch)
    scenes = []
    module.scene.subscribe(scenes.append)

    module._on_observation(_observation())

    assert len(native.calls) == 1
    points, kwargs = native.calls[0]
    assert points[0].tolist() == pytest.approx([1.1, 2.0, 0.5])
    assert kwargs["labels"] is None
    assert module.health()["semantic_map"]["geometry_only"] == 1
    assert scenes[-1].metadata["geometry_only"] is True
    assert "labels" not in scenes[-1].layers[0]


def test_semantic_module_accepts_only_exact_uint16_labels(monkeypatch) -> None:
    module, native = _module(monkeypatch)
    scenes = []
    module.scene.subscribe(scenes.append)
    labels = SemanticLabelsFrame(
        labels=[7],
        confidence=[0.8],
        sequence=2,
        ts=11.0,
        frame_id="body",
        taxonomy="lingtu.semantic",
        taxonomy_version=1,
    )
    module._on_labels(labels)

    module._on_observation(_observation(sequence=2, ts=11.0))

    assert native.calls[-1][1]["labels"].dtype == np.uint16
    assert native.calls[-1][1]["taxonomy"] == "lingtu.semantic"
    assert module.health()["semantic_map"]["semantic"] == 1
    assert scenes[-1].layers[0]["labels"].tolist() == [7]


def test_semantic_module_rejects_misaligned_labels_without_poisoning_geometry(
    monkeypatch,
) -> None:
    module, native = _module(monkeypatch)
    module._on_labels(
        SemanticLabelsFrame(
            labels=[9],
            sequence=3,
            ts=12.1,
            frame_id="body",
            taxonomy="lingtu.semantic",
            taxonomy_version=1,
        )
    )

    module._on_observation(_observation(sequence=3, ts=12.0))

    assert native.calls[-1][1]["labels"] is None
    health = module.health()["semantic_map"]
    assert health["labels_rejected"] == 1
    assert health["last_rejection"] == "timestamp_mismatch"
    assert health["geometry_only"] == 1


def test_semantic_module_saves_native_generation_atomically(monkeypatch, tmp_path) -> None:
    module, native = _module(monkeypatch)
    module._on_observation(_observation())
    results = []
    module.semantic_save_result.subscribe(results.append)
    artifact = tmp_path / "semantic_map.bin"

    module._on_save_request(
        SemanticSaveRequest(
            request_id="save-1",
            map_id="warehouse",
            path=str(artifact),
        )
    )

    assert results[-1].success is True
    assert results[-1].generation == 1
    assert results[-1].voxel_count == 1
    assert native.saved == [(str(artifact), 1)]
    assert module.health()["semantic_map"]["saves"] == 1


def test_semantic_save_handshake_is_explicitly_wired() -> None:
    from lingtu.assembly.wires.mapping import traversability_specs

    wires = {f"{spec.out_module}.{spec.out_port}->{spec.in_module}.{spec.in_port}" for spec in traversability_specs()}

    assert ("maps.service.semantic_save_request->SemanticMapModule.semantic_save_request") in wires
    assert ("SemanticMapModule.semantic_save_result->maps.service.semantic_save_result") in wires
