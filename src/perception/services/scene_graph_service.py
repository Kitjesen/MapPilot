"""Scene-graph service skeleton — pass-through and SceneGraph construction."""

from __future__ import annotations

import json
import logging
from typing import Any

import numpy as np

from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import (
    Detection3D as CoreDetection3D,
)
from runtime.msgs.semantic import (
    Region,
    Relation,
    SceneGraph,
)

logger = logging.getLogger(__name__)


class SceneGraphService:
    """Encapsulates scene-graph generation.

    Phase 2 behavior: the scene graph is still produced by the tracker.  This
    class reserves the boundary so Phase 3 can split scene-graph construction
    into a standalone service without changing the port contract.
    """

    _UNSET = object()

    def __init__(self, tracker: Any | None = None) -> None:
        self._tracker = tracker

    @property
    def tracker(self) -> Any | None:
        return self._tracker

    @tracker.setter
    def tracker(self, value: Any | None) -> None:
        self._tracker = value

    def get_scene_graph_json(self) -> str:
        """Return the current scene graph as a JSON string."""
        if self._tracker is None:
            return "{}"
        try:
            return self._tracker.get_scene_graph_json()
        except Exception as e:
            logger.warning("SceneGraphService get_scene_graph_json() failed: %s", e)
            return "{}"

    def get_objects(self) -> list[Any]:
        """Return the current tracked objects if the tracker exposes them."""
        if self._tracker is None:
            return []
        try:
            return self._tracker.get_objects()
        except Exception:
            return []

    def build_scene_graph(
        self,
        latest_core_detections: list[CoreDetection3D],
        frame_id: str,
        tracker: Any | object = _UNSET,
    ) -> SceneGraph:
        """Build core SceneGraph message from tracker state or detections.

        Args:
            latest_core_detections: Fallback detection list when tracker is None.
            frame_id: Coordinate frame id for the resulting SceneGraph.
            tracker: Tracker to query.  When omitted, the tracker injected at
                construction time (self._tracker) is used for compatibility
                with old callers.  Explicitly passing None forces the detection
                fallback path.
        """
        active_tracker = self._tracker if tracker is self._UNSET else tracker
        if active_tracker is None:
            return self._build_detection_scene_graph(latest_core_detections, frame_id)
        try:
            sg_json = active_tracker.get_scene_graph_json()
            data = json.loads(sg_json)
        except Exception:
            return self._build_detection_scene_graph(latest_core_detections, frame_id)

        objects = []
        for obj in data.get("objects", []):
            pos = obj.get("position", [0, 0, 0])
            if isinstance(pos, dict):
                px = float(pos.get("x", 0))
                py = float(pos.get("y", 0))
                pz = float(pos.get("z", 0))
            elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            else:
                px, py, pz = 0.0, 0.0, 0.0
            label = str(obj.get("label", ""))
            matched_det = self._match_detection_metadata(
                latest_core_detections,
                label,
                px,
                py,
                pz,
            )
            object_id = str(obj.get("id", ""))
            bbox_2d: list[float] = []
            clip_feature = None
            if matched_det is not None:
                if matched_det.id:
                    object_id = matched_det.id
                bbox_2d = [float(x) for x in matched_det.bbox_2d]
                if matched_det.clip_feature is not None:
                    clip_feature = np.array(matched_det.clip_feature, copy=True)
            objects.append(
                CoreDetection3D(
                    id=object_id,
                    label=label,
                    confidence=float(obj.get("confidence", obj.get("score", 0))),
                    position=Vector3(px, py, pz),
                    bbox_2d=bbox_2d,
                    clip_feature=clip_feature,
                )
            )

        relations = []
        for rel in data.get("relations", []):
            relations.append(
                Relation(
                    subject_id=str(rel.get("subject_id", rel.get("subject", ""))),
                    predicate=str(rel.get("predicate", rel.get("relation", ""))),
                    object_id=str(rel.get("object_id", rel.get("object", ""))),
                )
            )

        regions = []
        for reg in data.get("rooms", data.get("regions", [])):
            regions.append(
                Region(
                    name=str(reg.get("name", reg.get("room_type", ""))),
                    object_ids=[str(oid) for oid in reg.get("object_ids", [])],
                )
            )

        return SceneGraph(
            objects=objects,
            relations=relations,
            regions=regions,
            frame_id=frame_id,
        )

    def _build_detection_scene_graph(
        self,
        latest_core_detections: list[CoreDetection3D],
        frame_id: str,
    ) -> SceneGraph:
        """Fallback scene graph when tracker state is unavailable."""
        return SceneGraph(
            objects=[self._clone_core_detection(det) for det in latest_core_detections],
            frame_id=frame_id,
        )

    def _match_detection_metadata(
        self,
        latest_core_detections: list[CoreDetection3D],
        label: str,
        px: float,
        py: float,
        pz: float,
    ) -> CoreDetection3D | None:
        best_det: CoreDetection3D | None = None
        best_dist = 1.5
        label_lower = label.lower()

        for det in latest_core_detections:
            if label_lower and det.label.lower() != label_lower:
                continue
            dx = det.position.x - px
            dy = det.position.y - py
            dz = det.position.z - pz
            dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
            if dist < best_dist:
                best_dist = dist
                best_det = det

        return best_det

    @staticmethod
    def _clone_core_detection(det: CoreDetection3D) -> CoreDetection3D:
        clip_feature = None
        if det.clip_feature is not None:
            clip_feature = np.array(det.clip_feature, copy=True)
        return CoreDetection3D(
            id=det.id,
            label=det.label,
            confidence=det.confidence,
            position=Vector3(det.position.x, det.position.y, det.position.z),
            bbox_2d=[float(x) for x in det.bbox_2d],
            clip_feature=clip_feature,
            ts=det.ts,
        )
