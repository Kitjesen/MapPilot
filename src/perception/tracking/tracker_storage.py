"""Scene-graph persistence (save / load) for InstanceTracker.

Extracted from ``instance_tracker.py`` (Phase 3.5) to isolate the
persistence concern: writing the in-memory scene graph to disk and
restoring it later for long-term memory.

The :class:`InstanceTrackerStorage` is stateless — it reads and writes
tracker state through the tracker instance passed to each method call.
"""

from __future__ import annotations

import json
import logging
from typing import TYPE_CHECKING

import numpy as np

from runtime.utils.sanitize import safe_json_dump

if TYPE_CHECKING:
    from .instance_tracker import InstanceTracker

logger = logging.getLogger(__name__)


class InstanceTrackerStorage:
    """Persist and restore an InstanceTracker's scene graph to/from disk.

    All methods accept the *tracker* instance as their first argument so
    the storage itself remains stateless and re-usable.
    """

    def save(self, tracker: InstanceTracker, path: str) -> None:
        """Save the scene graph to a JSON file (long-term memory).

        Equivalent to the original ``InstanceTracker.save_to_file()``.
        """
        room_name_cache = tracker._room_inferencer.room_name_cache

        data: dict = {
            "version": "2.0",
            "objects": {},
            "views": {},
            "room_names": dict(room_name_cache),
            "next_id": tracker._next_id,
            "next_view_id": tracker._next_view_id,
        }

        for oid, obj in tracker._objects.items():
            data["objects"][str(oid)] = {
                "label": obj.label,
                "position": obj.position.tolist(),
                "best_score": obj.best_score,
                "detection_count": obj.detection_count,
                "last_seen": obj.last_seen,
                "extent": obj.extent.tolist(),
                "belief_alpha": obj.belief_alpha,
                "belief_beta": obj.belief_beta,
                "position_variance": obj.position_variance,
                "miss_streak": obj.miss_streak,
                "region_id": obj.region_id,
                "kg_concept_id": obj.kg_concept_id,
                "safety_level": obj.safety_level,
                "affordances": obj.affordances,
                "floor_level": obj.floor_level,
                "features": (obj.features.tolist() if obj.features is not None and obj.features.size > 0 else []),
                "points": (obj.points.tolist() if obj.points is not None and len(obj.points) > 0 else []),
            }

        for vid, view in tracker._views.items():
            data["views"][str(vid)] = {
                "position": view.position.tolist(),
                "timestamp": view.timestamp,
                "room_id": view.room_id,
                "object_ids": view.object_ids,
                "key_labels": view.key_labels,
            }

        safe_json_dump(data, path)
        logger.info(
            "Scene graph saved to %s (%d objects, %d views)",
            path,
            len(tracker._objects),
            len(tracker._views),
        )

    def load(self, tracker: InstanceTracker, path: str) -> bool:
        """Restore the scene graph from a JSON file (long-term memory).

        Equivalent to the original ``InstanceTracker.load_from_file()``.
        """
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logger.warning("Failed to load scene graph from %s: %s", path, e)
            return False

        from .tracked_objects import TrackedObject, ViewNode

        tracker._objects.clear()
        tracker._views.clear()

        for oid_str, odata in data.get("objects", {}).items():
            oid = int(oid_str)
            obj = TrackedObject(
                object_id=oid,
                label=odata["label"],
                position=np.array(odata["position"]),
                best_score=odata["best_score"],
                detection_count=odata.get("detection_count", 1),
                last_seen=odata.get("last_seen", 0.0),
                extent=np.array(odata.get("extent", [0.2, 0.2, 0.2])),
                belief_alpha=odata.get("belief_alpha", 1.5),
                belief_beta=odata.get("belief_beta", 1.0),
                position_variance=odata.get("position_variance", 1.0),
                miss_streak=odata.get("miss_streak", 0),
                region_id=odata.get("region_id", -1),
                kg_concept_id=odata.get("kg_concept_id", ""),
                safety_level=odata.get("safety_level", "safe"),
                affordances=odata.get("affordances", []),
                floor_level=odata.get("floor_level", 0),
                features=np.array(odata.get("features", [])),
                points=(np.array(odata.get("points", [])).reshape(-1, 3) if odata.get("points") else np.empty((0, 3))),
            )
            obj._update_credibility()
            tracker._enrich_from_kg(obj)
            obj.source = "loaded"
            tracker._objects[oid] = obj

        for vid_str, vdata in data.get("views", {}).items():
            vid = int(vid_str)
            tracker._views[vid] = ViewNode(
                view_id=vid,
                position=np.array(vdata["position"]),
                timestamp=vdata["timestamp"],
                room_id=vdata.get("room_id", -1),
                object_ids=vdata.get("object_ids", []),
                key_labels=vdata.get("key_labels", []),
            )

        # Restore room-name cache into the RoomInferencer
        room_names = data.get("room_names", {})
        room_names = {int(k): v for k, v in room_names.items()}
        tracker._room_inferencer._room_name_cache = room_names

        tracker._next_id = data.get("next_id", max(tracker._objects.keys(), default=-1) + 1)
        tracker._next_view_id = data.get("next_view_id", max(tracker._views.keys(), default=-1) + 1)

        logger.info(
            "Scene graph loaded from %s (%d objects, %d views)",
            path,
            len(tracker._objects),
            len(tracker._views),
        )
        return True
