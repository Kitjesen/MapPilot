"""Sim-only semantic observer backed by MuJoCo scene XML metadata."""
from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_SIM_ROOT = Path(__file__).resolve().parents[3] / "sim"
_WORLD_FILES = {
    "building": _SIM_ROOT / "packages/worlds/building/physics/building_scene.xml",
    "building_scene": _SIM_ROOT / "packages/worlds/building/physics/building_scene.xml",
    "factory": _SIM_ROOT / "compat/engine/worlds/factory_scene.xml",
    "factory_scene": _SIM_ROOT / "compat/engine/worlds/factory_scene.xml",
    "open_field": _SIM_ROOT / "packages/worlds/open_field/physics/open_field.xml",
}
_AGGREGATE_LABELS = {"stairs", "goal", "forklift"}


@dataclass
class _SceneObject:
    track_id: str
    label: str
    position: np.ndarray
    extent: float


@dataclass
class SimDetection3D:
    position: np.ndarray
    label: str
    track_id: str
    score: float = 1.0
    bbox_2d: np.ndarray = field(default_factory=lambda: np.array([]))
    depth: float = 0.0
    features: np.ndarray = field(default_factory=lambda: np.array([]))
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))


class SimSceneObserver:
    """Generate semantic detections from scene geometry for sim-only runtime smoke tests."""

    def __init__(
        self,
        world: str = "",
        scenario_entities: Sequence[Mapping[str, object]] = (),
        scenario_snapshot: str | Path | None = None,
    ) -> None:
        self._world = world
        scenario_objects = self._load_scenario_objects(scenario_entities)
        self._objects = [
            *self._load_objects(world),
            *scenario_objects,
        ]
        self._scenario_objects = {item.track_id: item for item in scenario_objects}
        self._scenario_snapshot = self._resolve_snapshot_path(scenario_snapshot)
        self._snapshot_mtime_ns = -1

    def detect(self, rgb: np.ndarray, text_prompt: str) -> list:
        return []

    def shutdown(self) -> None:
        return None

    def observe(self, tf_camera_to_world: np.ndarray, intrinsics, text_prompt: str = "") -> list[SimDetection3D]:
        self._refresh_scenario_snapshot()
        if not self._objects:
            return []

        pos = tf_camera_to_world[:3, 3]
        rot = tf_camera_to_world[:3, :3]

        # Different MuJoCo/runtime paths have exposed the forward axis as either
        # +X or -X in odometry. Try both conventions and keep the one that sees
        # the most objects so sim-only semantics stays stable across platforms.
        forward_candidates: list[np.ndarray] = []
        for raw_forward in (rot[:, 0].astype(np.float64), (-rot[:, 0]).astype(np.float64)):
            forward = self._normalize(raw_forward, np.array([1.0, 0.0, 0.0], dtype=np.float64))
            if not any(np.allclose(forward, prev) for prev in forward_candidates):
                forward_candidates.append(forward)

        right = self._normalize(rot[:, 1].astype(np.float64), np.array([0.0, 1.0, 0.0], dtype=np.float64))
        up = self._normalize(rot[:, 2].astype(np.float64), np.array([0.0, 0.0, 1.0], dtype=np.float64))

        allowed = {
            token.strip().lower()
            for token in text_prompt.replace(",", ".").split(".")
            if token.strip()
        }
        fx = float(getattr(intrinsics, "fx", 415.0) or 415.0)
        fy = float(getattr(intrinsics, "fy", 415.0) or 415.0)
        cx = float(getattr(intrinsics, "cx", 320.0) or 320.0)
        cy = float(getattr(intrinsics, "cy", 240.0) or 240.0)
        width = int(getattr(intrinsics, "width", 640) or 640)
        height = int(getattr(intrinsics, "height", 480) or 480)

        best: list[SimDetection3D] = []
        for forward in forward_candidates:
            detections = self._collect_detections(
                pos=pos,
                forward=forward,
                right=right,
                up=up,
                allowed=allowed,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                width=width,
                height=height,
            )
            if len(detections) > len(best):
                best = detections
        return best

    def _collect_detections(
        self,
        *,
        pos: np.ndarray,
        forward: np.ndarray,
        right: np.ndarray,
        up: np.ndarray,
        allowed: set[str],
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        width: int,
        height: int,
    ) -> list[SimDetection3D]:
        detections: list[SimDetection3D] = []
        for obj in self._objects:
            if allowed and obj.label.lower() not in allowed:
                continue
            rel = obj.position - pos
            depth_forward = float(np.dot(rel, forward))
            if depth_forward <= 0.5 or depth_forward > 25.0:
                continue
            lateral = float(np.dot(rel, right))
            vertical = float(np.dot(rel, up))
            px = fx * (lateral / depth_forward) + cx
            py = cy - fy * (vertical / depth_forward)
            if px < -80 or px > width + 80 or py < -80 or py > height + 80:
                continue
            half_extent = max(12.0, fx * obj.extent / max(depth_forward, 1e-3))
            bbox = np.array([
                max(0.0, px - half_extent),
                max(0.0, py - half_extent),
                min(float(width - 1), px + half_extent),
                min(float(height - 1), py + half_extent),
            ], dtype=np.float32)
            detections.append(
                SimDetection3D(
                    position=obj.position.copy(),
                    label=obj.label,
                    track_id=obj.track_id,
                    score=1.0,
                    bbox_2d=bbox,
                    depth=depth_forward,
                )
            )
        return detections

    @staticmethod
    def _normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vec))
        if norm < 1e-6:
            return fallback.copy()
        return (vec / norm).astype(np.float64)

    @classmethod
    def _load_objects(cls, world: str) -> list[_SceneObject]:
        if not world:
            return []
        world_file = _WORLD_FILES.get(world, Path(world))
        scene_xml = world_file
        if not scene_xml.exists():
            return []

        root = ET.fromstring(scene_xml.read_text(encoding="utf-8", errors="ignore"))
        worldbody = root.find("worldbody")
        if worldbody is None:
            return []

        raw: list[_SceneObject] = []
        for geom in worldbody.iter("geom"):
            geom_name = geom.attrib.get("name", "")
            label = cls._map_label(geom_name)
            if not label:
                continue
            pos = cls._parse_vector(geom.attrib.get("pos", ""))
            if pos is None:
                continue
            extent = cls._extent_from_geom(geom)
            raw.append(
                _SceneObject(
                    track_id=geom_name,
                    label=label,
                    position=pos,
                    extent=extent,
                )
            )

        grouped: list[_SceneObject] = []
        for label in sorted({obj.label for obj in raw}):
            members = [obj for obj in raw if obj.label == label]
            if label in _AGGREGATE_LABELS:
                positions = np.vstack([obj.position for obj in members])
                grouped.append(
                    _SceneObject(
                        track_id=members[0].track_id,
                        label=label,
                        position=np.mean(positions, axis=0),
                        extent=max(obj.extent for obj in members),
                    )
                )
            else:
                grouped.extend(members)
        return grouped

    @classmethod
    def _load_scenario_objects(
        cls,
        entities: Sequence[Mapping[str, object]],
    ) -> list[_SceneObject]:
        objects: list[_SceneObject] = []
        for entity in entities:
            entity_id = str(entity.get("entity_id") or "").strip()
            entity_type = str(entity.get("entity_type") or "").strip().lower()
            semantic_class = str(entity.get("semantic_class") or "").strip().lower()
            if not entity_id or entity_type == "robot":
                continue
            transform = entity.get("initial_transform")
            if not isinstance(transform, Mapping):
                continue
            position = transform.get("position_m")
            if not isinstance(position, list | tuple) or len(position) < 3:
                continue
            try:
                point = np.array(position[:3], dtype=np.float32)
            except (TypeError, ValueError):
                continue
            label = "person" if semantic_class in {"person", "pedestrian", "human"} else semantic_class
            if not label:
                label = entity_type
            objects.append(
                _SceneObject(
                    track_id=entity_id,
                    label=label,
                    position=point,
                    extent=0.9 if entity_type == "pedestrian" else 0.5,
                )
            )
        return objects

    @staticmethod
    def _resolve_snapshot_path(value: str | Path | None) -> Path | None:
        if value is not None:
            return Path(value)
        run_plan = str(os.environ.get("LINGTU_RUN_PLAN") or "").strip()
        return Path(run_plan).parent / "scenario.current.json" if run_plan else None

    def _refresh_scenario_snapshot(self) -> None:
        path = self._scenario_snapshot
        if path is None:
            return
        try:
            modified = path.stat().st_mtime_ns
        except FileNotFoundError:
            return
        if modified == self._snapshot_mtime_ns:
            return
        payload = json.loads(path.read_text(encoding="utf-8"))
        entities = payload.get("entities") if isinstance(payload, Mapping) else None
        if not isinstance(entities, list):
            raise ValueError("scenario snapshot entities must be a list")
        for entity in entities:
            if not isinstance(entity, Mapping):
                continue
            target = self._scenario_objects.get(str(entity.get("entity_id") or ""))
            transform = entity.get("transform")
            position = transform.get("position_m") if isinstance(transform, Mapping) else None
            if target is None or not isinstance(position, list) or len(position) < 3:
                continue
            target.position = np.asarray(position[:3], dtype=np.float32)
        self._snapshot_mtime_ns = modified

    @staticmethod
    def _parse_vector(value: str) -> np.ndarray | None:
        parts = [float(v) for v in value.split()] if value else []
        if len(parts) < 3:
            return None
        return np.array(parts[:3], dtype=np.float32)

    @staticmethod
    def _extent_from_geom(geom) -> float:
        size_str = geom.attrib.get("size", "")
        parts = [float(v) for v in size_str.split()] if size_str else []
        geom_type = geom.attrib.get("type", "box")
        if geom_type == "box" and len(parts) >= 3:
            return max(parts[:3])
        if geom_type == "cylinder" and len(parts) >= 2:
            return max(parts[0], parts[1])
        if geom_type == "sphere" and parts:
            return parts[0]
        return max(parts) if parts else 0.5

    @staticmethod
    def _map_label(name: str) -> str:
        lower = name.lower()
        if lower.startswith("step_"):
            return "stairs"
        if lower.startswith("goal_"):
            return "goal"
        if lower.startswith("fork_"):
            return "forklift"
        if lower.startswith("cont_"):
            return "container"
        if "drum" in lower:
            return "barrel"
        if "pallet" in lower:
            return "pallet"
        if "crate" in lower:
            return "crate"
        return ""
