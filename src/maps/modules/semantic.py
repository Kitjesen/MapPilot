"""Runtime ports for the native semantic occupancy map."""

from __future__ import annotations

import time
from collections import OrderedDict
from pathlib import Path
from typing import Any

from maps.adapters.python.semantic import NativeSemanticLayer, SemanticNativeUnavailable
from maps.taxonomy import load_semantic_taxonomy
from runtime.module import Module
from runtime.msgs.map import (
    MapObservationFrame,
    MapSceneFrame,
    SemanticLabelsFrame,
    SemanticSaveRequest,
    SemanticSaveResult,
)
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.stream import In, Out


@register("map", "semantic", description="Native semantic occupancy map")
class SemanticMapModule(Module, layer=2):
    """Join accepted scans with exact labels and update the C++ map core."""

    map_observation: In[MapObservationFrame]
    semantic_labels: In[SemanticLabelsFrame]
    semantic_save_request: In[SemanticSaveRequest]
    scene: Out[MapSceneFrame]
    semantic_stats: Out[dict]
    semantic_save_result: Out[SemanticSaveResult]

    def __init__(
        self,
        *,
        voxel_size: float = 0.2,
        max_range: float = 50.0,
        min_z: float = -3.0,
        max_z: float = 5.0,
        publish_interval: float = 1.0,
        label_sync_tolerance_s: float = 0.001,
        max_pending_labels: int = 32,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._native_config = {
            "voxel_size": float(voxel_size),
            "max_range": float(max_range),
            "min_z": float(min_z),
            "max_z": float(max_z),
            "raycast_free_space": True,
        }
        self._publish_interval = max(0.0, float(publish_interval))
        self._label_sync_tolerance_s = max(0.0, float(label_sync_tolerance_s))
        self._max_pending_labels = max(1, int(max_pending_labels))
        self._native: NativeSemanticLayer | None = None
        self._taxonomy = load_semantic_taxonomy()
        self._labels: OrderedDict[tuple[int, int], SemanticLabelsFrame] = OrderedDict()
        self._last_observation_epoch = 0
        self._last_observation_sequence = 0
        self._last_publish_s = 0.0
        self._scene_sequence = 0
        self._stats: dict[str, Any] = {
            "observations": 0,
            "geometry_only": 0,
            "semantic": 0,
            "duplicate": 0,
            "stale": 0,
            "labels_rejected": 0,
            "last_rejection": "",
            "generation": 0,
            "saves": 0,
            "save_failures": 0,
        }

    def setup(self) -> None:
        try:
            self._native = NativeSemanticLayer(**self._native_config)
        except SemanticNativeUnavailable as exc:
            raise RuntimeError("SemanticMapModule requires the lingtu_maps semantic C ABI") from exc
        self.map_observation.subscribe(self._on_observation)
        self.semantic_labels.subscribe(self._on_labels)
        self.semantic_save_request.subscribe(self._on_save_request)
        self.map_observation.set_policy("latest")
        self.semantic_labels.set_policy("buffer", size=self._max_pending_labels)

    def stop(self) -> None:
        if self._native is not None:
            self._native.close()
            self._native = None
        super().stop()

    def _on_labels(self, frame: SemanticLabelsFrame) -> None:
        key = (int(frame.metadata.get("reset_epoch", 1) or 1), frame.sequence)
        self._labels[key] = frame
        self._labels.move_to_end(key)
        while len(self._labels) > self._max_pending_labels:
            self._labels.popitem(last=False)

    def _on_save_request(self, request: SemanticSaveRequest) -> None:
        native = self._require_native()
        generation = 0
        voxel_count = 0
        try:
            metadata = native.metadata()
            generation = int(metadata["generation"])
            voxel_count = int(metadata["voxel_count"])
            if generation <= 0 or voxel_count <= 0:
                raise RuntimeError("semantic_map_empty")
            target = Path(request.path)
            if target.name != "semantic_map.bin":
                raise ValueError("semantic artifact filename must be semantic_map.bin")
            native.save(str(target), expected_generation=generation)
            valid, validation_message = NativeSemanticLayer.validate_file(str(target))
            if not valid:
                raise RuntimeError(validation_message)
            self._stats["saves"] += 1
            result = SemanticSaveResult(
                request_id=request.request_id,
                map_id=request.map_id,
                path=str(target),
                success=True,
                generation=generation,
                voxel_count=voxel_count,
                message=validation_message or "semantic_map.bin committed",
            )
        except Exception as exc:
            self._stats["save_failures"] += 1
            result = SemanticSaveResult(
                request_id=request.request_id,
                map_id=request.map_id,
                path=request.path,
                success=False,
                generation=generation,
                voxel_count=voxel_count,
                message=str(exc),
            )
        self.semantic_save_result.publish(result)

    def _on_observation(self, observation: MapObservationFrame) -> None:
        native = self._require_native()
        epoch = int(observation.reset_epoch)
        sequence = int(observation.sequence)
        if epoch < self._last_observation_epoch or (
            epoch == self._last_observation_epoch
            and sequence <= self._last_observation_sequence
        ):
            self._stats["stale"] += 1
            return
        if self._last_observation_epoch and epoch > self._last_observation_epoch:
            native.reset()
            self._labels.clear()
        self._last_observation_epoch = epoch
        self._last_observation_sequence = sequence
        labels = self._labels.pop((epoch, sequence), None)
        label_values = None
        taxonomy = ""
        taxonomy_version = 0
        if labels is not None:
            rejection = self._label_rejection(observation, labels)
            if rejection:
                self._stats["labels_rejected"] += 1
                self._stats["last_rejection"] = rejection
            else:
                label_values = labels.labels
                taxonomy = labels.taxonomy
                taxonomy_version = labels.taxonomy_version

        map_points = observation.map_points()
        update = native.update(
            map_points[:, :3],
            sequence=observation.sequence,
            stamp_ns=int(round(observation.ts * 1_000_000_000)),
            frame_id=observation.frame_id,
            origin_xyz=(
                observation.sensor_origin.x,
                observation.sensor_origin.y,
                observation.sensor_origin.z,
            ),
            labels=label_values,
            taxonomy=taxonomy,
            taxonomy_version=taxonomy_version,
        )
        self._stats["observations"] += 1
        self._stats["generation"] = int(update["generation_after"])
        if update["duplicate_sequence"]:
            self._stats["duplicate"] += 1
        elif update["stale_sequence"]:
            self._stats["stale"] += 1
        elif label_values is None:
            self._stats["geometry_only"] += 1
        else:
            self._stats["semantic"] += 1
        self._stats["last_update"] = dict(update)
        now = time.monotonic()
        if self._publish_interval == 0.0 or now - self._last_publish_s >= self._publish_interval:
            self._last_publish_s = now
            self._publish_scene(observation.frame_id)

    def _label_rejection(self, observation: MapObservationFrame, labels: SemanticLabelsFrame) -> str:
        if int(labels.metadata.get("reset_epoch", 1) or 1) != observation.reset_epoch:
            return "reset_epoch_mismatch"
        if labels.sequence != observation.sequence:
            return "sequence_mismatch"
        if abs(labels.ts - observation.ts) > self._label_sync_tolerance_s:
            return "timestamp_mismatch"
        if labels.frame_id != observation.sensor_frame_id:
            return "frame_mismatch"
        if labels.num_labels != observation.num_points:
            return "point_count_mismatch"
        return ""

    def _publish_scene(self, frame_id: str) -> None:
        snapshot = self._require_native().snapshot(min_occupancy=0.5)
        metadata = self._require_native().metadata()
        count = int(snapshot["center_x_m"].size)
        if count == 0:
            self.semantic_stats.publish(dict(self._stats))
            return
        use_mean = snapshot["point_count"] > 0
        xyz = np.column_stack(
            (
                np.where(use_mean, snapshot["mean_x_m"], snapshot["center_x_m"]),
                np.where(use_mean, snapshot["mean_y_m"], snapshot["center_y_m"]),
                np.where(use_mean, snapshot["mean_z_m"], snapshot["center_z_m"]),
            )
        ).astype(np.float32, copy=False)
        labels = snapshot["dominant_label"]
        confidence = snapshot["semantic_confidence"]
        has_semantics = bool(np.any((labels != 0) & (confidence > 0.0)))
        layer: dict[str, Any] = {
            "id": "maps.semantic_occupancy",
            "type": "pointcloud",
            "source": "maps.semantic",
            "payload": PointCloud2.from_numpy(xyz, frame_id=frame_id),
            "generation": int(snapshot["generation"]),
            "point_count": count,
            "semantic": "semantic_occupancy" if has_semantics else "geometry_occupancy",
        }
        if has_semantics:
            layer.update(
                {
                    "labels": labels,
                    "confidence": confidence,
                    "taxonomy": metadata["taxonomy"],
                    "taxonomy_version": metadata["taxonomy_version"],
                    "palette": self._taxonomy.palette,
                }
            )
        self._scene_sequence += 1
        self.scene.publish(
            MapSceneFrame(
                frame_id=frame_id,
                source="maps.semantic",
                sequence=self._scene_sequence,
                metadata={
                    "generation": int(snapshot["generation"]),
                    "geometry_only": not has_semantics,
                },
                layers=[layer],
            )
        )
        self.semantic_stats.publish(dict(self._stats))

    def _require_native(self) -> NativeSemanticLayer:
        if self._native is None:
            raise RuntimeError("SemanticMapModule is not initialized")
        return self._native

    def health(self) -> dict[str, Any]:
        return {
            **super().port_summary(),
            "semantic_map": {
                **self._stats,
                "backend": "cpp",
                "pending_labels": len(self._labels),
                "label_sync_tolerance_s": self._label_sync_tolerance_s,
            },
        }
