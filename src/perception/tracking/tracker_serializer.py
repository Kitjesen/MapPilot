"""Scene-graph JSON serialization for InstanceTracker.

Extracted from ``instance_tracker.py`` (Phase 3.5) to isolate the
serialization concern: converting the in-memory scene graph into
JSON strings for LLM consumption, gateway transport, and diff reporting.

The :class:`InstanceTrackerSerializer` is stateless — it reads tracker
state through the tracker instance passed to each method call.
"""

from __future__ import annotations

import json
import logging
import time
from typing import TYPE_CHECKING

import numpy as np

from runtime.runtime_interface import map_frame_id
from runtime.utils.sanitize import sanitize_position

if TYPE_CHECKING:
    from .instance_tracker import InstanceTracker
    from .tracked_objects import TrackedObject

logger = logging.getLogger(__name__)

_INSTANCE_TRACKER_MAP_FRAME_ID = map_frame_id()


class InstanceTrackerSerializer:
    """Serialize an InstanceTracker's scene graph to JSON.

    All methods accept the *tracker* instance as their first argument so
    the serializer itself remains stateless and re-usable.
    """

    # ──────────────────────────────────────────────────────────
    #  Full scene-graph JSON  (SG-Nav / ConceptGraphs style)
    # ──────────────────────────────────────────────────────────

    def to_json(self, tracker: InstanceTracker) -> str:
        """Export the complete hierarchical scene graph as a JSON string.

        Equivalent to the original ``InstanceTracker.get_scene_graph_json()``.
        """
        builder = tracker._scene_graph_builder
        room_inferencer = tracker._room_inferencer
        objects = tracker._objects
        views_store = tracker._views

        # Compute regions first so that object.region_id is updated.
        regions = builder.compute_regions(room_inferencer.room_name_cache)

        relations = builder.compute_spatial_relations()
        relations_list = [
            {
                "subject_id": r.subject_id,
                "relation": r.relation,
                "object_id": r.object_id,
                "distance": r.distance,
            }
            for r in relations
        ]

        groups = builder.compute_groups(regions)
        rooms = room_inferencer.compute_rooms(objects, regions, groups)

        # View layer
        from .tracked_objects import ViewNode

        views = [
            ViewNode(
                view_id=v.view_id,
                position=v.position.copy(),
                timestamp=v.timestamp,
                room_id=v.room_id,
                object_ids=list(v.object_ids),
                key_labels=list(v.key_labels),
            )
            for v in views_store.values()
        ]
        views = builder.assign_view_rooms(views, rooms)

        hierarchy_edges = builder.build_hierarchy_edges(rooms, groups)
        hierarchy_edges.extend(builder.build_view_edges(views))

        topology_edges = builder.compute_topology_edges(rooms)
        frontier_nodes = builder.estimate_frontier_directions(rooms)

        # Objects (region_id already updated above)
        objects_list = self._build_objects_list(objects)

        regions_list = [
            {
                "region_id": r.region_id,
                "name": r.name,
                "center": {
                    "x": round(float(r.center[0]), 2),
                    "y": round(float(r.center[1]), 2),
                },
                "object_ids": r.object_ids,
            }
            for r in regions
        ]

        rooms_list = [
            {
                "room_id": rm.room_id,
                "name": rm.name,
                "center": {
                    "x": round(float(rm.center[0]), 2),
                    "y": round(float(rm.center[1]), 2),
                },
                "object_ids": rm.object_ids,
                "group_ids": rm.group_ids,
            }
            for rm in rooms
        ]

        groups_list = [
            {
                "group_id": g.group_id,
                "room_id": g.room_id,
                "name": g.name,
                "center": {
                    "x": round(float(g.center[0]), 2),
                    "y": round(float(g.center[1]), 2),
                },
                "object_ids": g.object_ids,
            }
            for g in groups
        ]

        views_list = [
            {
                "view_id": v.view_id,
                "room_id": v.room_id,
                "timestamp": round(float(v.timestamp), 3),
                "position": {
                    "x": round(float(v.position[0]), 2),
                    "y": round(float(v.position[1]), 2),
                    "z": round(float(v.position[2]), 2),
                },
                "object_ids": v.object_ids,
                "key_labels": v.key_labels[:10],
            }
            for v in views
        ]

        subgraphs = builder.build_subgraphs(
            rooms=rooms,
            groups=groups,
            views=views,
            objects_by_id=objects,
            relations_list=relations_list,
        )

        # Floor layer
        floors = builder.compute_floors()
        builder.assign_rooms_to_floors(floors, rooms)
        tracker._cached_floors = floors
        tracker._cached_regions = regions
        floors_list = [
            {
                "floor_id": f.floor_id,
                "floor_level": f.floor_level,
                "z_range": [round(f.z_range[0], 2), round(f.z_range[1], 2)],
                "center_z": round(f.center_z, 2),
                "room_ids": f.room_ids,
                "object_count": len(f.object_ids),
            }
            for f in floors
        ]

        # KG stats
        kg_stats = {"enriched": 0, "dangerous": 0, "graspable": 0}
        for obj in objects.values():
            if obj.kg_concept_id:
                kg_stats["enriched"] += 1
            if obj.safety_level in ("dangerous", "forbidden"):
                kg_stats["dangerous"] += 1
            if "graspable" in obj.affordances:
                kg_stats["graspable"] += 1

        # Natural-language summary (SG-Nav key insight for LLM)
        summary_parts = self._build_summary(
            objects,
            rooms,
            relations,
            topology_edges,
            floors_list,
            objects_list,
            rooms_list,
            groups_list,
            views_list,
            kg_stats,
            tracker,
        )

        # Belief Propagation diagnostics + Phantom Nodes
        phantom_list = self._build_phantom_list(tracker)
        room_posteriors_list = self._build_room_posteriors(tracker)
        bp_diag = {
            "iterations": tracker._bp_iteration_count,
            "convergence": tracker._bp_convergence_history[-5:] if tracker._bp_convergence_history else [],
        }

        if phantom_list:
            summary_parts.append(f"Phantom (blind) nodes: {len(phantom_list)} expected but unseen objects")
            dangerous_phantoms = [p for p in phantom_list if p["safety_level"] in ("dangerous", "forbidden")]
            if dangerous_phantoms:
                summary_parts.append(f"⚠️ {len(dangerous_phantoms)} dangerous phantom objects predicted")

        from runtime.utils.sanitize import safe_json_dumps

        scene_graph_dict = {
            "timestamp": time.time(),
            "frame_id": _INSTANCE_TRACKER_MAP_FRAME_ID,
            "graph_level": "hierarchical",
            "graph_version": "3.0",
            "object_count": len(objects_list),
            "objects": objects_list,
            "relations": relations_list,
            "regions": regions_list,
            "floors": floors_list,
            "rooms": rooms_list,
            "groups": groups_list,
            "views": views_list,
            "hierarchy_edges": hierarchy_edges,
            "topology_edges": topology_edges,
            "frontier_nodes": frontier_nodes,
            "subgraphs": subgraphs,
            "kg_stats": kg_stats,
            "phantom_nodes": phantom_list,
            "room_type_posteriors": room_posteriors_list,
            "belief_propagation": bp_diag,
            "summary": " | ".join(summary_parts),
        }
        return safe_json_dumps(scene_graph_dict)

    # ──────────────────────────────────────────────────────────
    #  Scene diff (DovSG dynamic update)
    # ──────────────────────────────────────────────────────────

    def compute_scene_diff(
        self,
        tracker: InstanceTracker,
        prev_snapshot: dict,
    ) -> dict:
        """Compute differences between the current scene and *prev_snapshot*.

        Equivalent to the original ``InstanceTracker.compute_scene_diff()``.
        """
        events: list[dict] = []
        prev_objects = {o["id"]: o for o in prev_snapshot.get("objects", [])}
        curr_objects = dict(tracker._objects)

        # Newly appeared objects
        for oid, obj in curr_objects.items():
            if oid not in prev_objects:
                events.append(
                    {
                        "type": "object_added",
                        "object_id": oid,
                        "label": obj.label,
                        "position": obj.position.tolist(),
                        "confidence": round(obj.credibility, 3),
                    }
                )

        # Disappeared objects
        for pid, pdata in prev_objects.items():
            if pid not in curr_objects:
                events.append(
                    {
                        "type": "object_removed",
                        "object_id": pid,
                        "label": pdata.get("label", "unknown"),
                        "last_position": [
                            pdata["position"]["x"],
                            pdata["position"]["y"],
                            pdata["position"]["z"],
                        ]
                        if "position" in pdata
                        else [],
                    }
                )

        # Objects with significant position change (possibly moved by human)
        move_threshold = 0.8  # metres
        for oid, obj in curr_objects.items():
            if oid in prev_objects:
                prev_pos = prev_objects[oid].get("position", {})
                px = prev_pos.get("x", 0)
                py = prev_pos.get("y", 0)
                pz = prev_pos.get("z", 0)
                dist = float(np.linalg.norm(obj.position - np.array([px, py, pz])))
                if dist > move_threshold:
                    events.append(
                        {
                            "type": "object_moved",
                            "object_id": oid,
                            "label": obj.label,
                            "prev_position": [px, py, pz],
                            "curr_position": obj.position.tolist(),
                            "displacement": round(dist, 3),
                        }
                    )

        # Significant belief change
        belief_change_threshold = 0.3
        for oid, obj in curr_objects.items():
            if oid in prev_objects:
                prev_belief = prev_objects[oid].get("belief", {})
                prev_cred = prev_belief.get("credibility", 0.5)
                if abs(obj.credibility - prev_cred) > belief_change_threshold:
                    events.append(
                        {
                            "type": "belief_changed",
                            "object_id": oid,
                            "label": obj.label,
                            "prev_credibility": round(prev_cred, 3),
                            "curr_credibility": round(obj.credibility, 3),
                        }
                    )

        return {
            "timestamp": time.time(),
            "total_events": len(events),
            "events": events,
            "summary": self._summarize_diff(events),
        }

    def diff_to_json(
        self,
        tracker: InstanceTracker,
        prev_snapshot: dict,
    ) -> str:
        """Return a JSON string describing changes since *prev_snapshot*.

        Equivalent to the original ``InstanceTracker.get_scene_graph_diff_json()``.
        """
        raw = self.compute_scene_diff(tracker, prev_snapshot)

        added: list[dict] = []
        updated: list[dict] = []
        removed: list[dict] = []
        for evt in raw.get("events", []):
            t = evt.get("type", "")
            if t == "object_added":
                pos = evt.get("position", [0, 0, 0])
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    pos_dict = {"x": pos[0], "y": pos[1], "z": pos[2]}
                else:
                    pos_dict = {"x": 0.0, "y": 0.0, "z": 0.0}
                obj = tracker._objects.get(evt["object_id"])
                added.append(
                    {
                        "id": evt["object_id"],
                        "label": evt.get("label", ""),
                        "position": pos_dict,
                        "credibility": obj.credibility if obj else evt.get("confidence", 0.0),
                    }
                )
            elif t == "object_removed":
                removed.append(
                    {
                        "id": evt["object_id"],
                        "label": evt.get("label", ""),
                    }
                )
            elif t == "object_moved":
                updated.append(
                    {
                        "id": evt["object_id"],
                        "label": evt.get("label", ""),
                        "old_position": evt.get("old_position", []),
                        "new_position": evt.get("new_position", []),
                    }
                )

        result = {
            "added": added,
            "updated": updated,
            "removed": removed,
            "summary": raw.get("summary", ""),
            "timestamp": raw.get("timestamp", 0.0),
        }
        return json.dumps(result, ensure_ascii=False)

    # ──────────────────────────────────────────────────────────
    #  Private helpers
    # ──────────────────────────────────────────────────────────

    @staticmethod
    def _build_objects_list(
        objects: dict[int, TrackedObject],
    ) -> list[dict]:
        """Serialize tracked objects into a JSON-friendly list."""
        objects_list: list[dict] = []
        for obj in objects.values():
            sp = sanitize_position(obj.position)
            obj_pos = {
                "x": round(sp[0], 3),
                "y": round(sp[1], 3),
                "z": round(sp[2], 3),
            }
            entry: dict = {
                "id": obj.object_id,
                "label": obj.label,
                "position": obj_pos,
                "score": round(obj.best_score, 3),
                "detection_count": obj.detection_count,
                "region_id": obj.region_id,
                "floor_level": obj.floor_level,
                "belief": obj.to_belief_dict(),
                "source": obj.source,
                "last_observed_time": round(obj.last_observed_time, 2),
                "is_simulated": obj.is_simulated,
            }
            if obj.kg_concept_id:
                entry["kg"] = {
                    "concept_id": obj.kg_concept_id,
                    "safety": obj.safety_level,
                    "affordances": obj.affordances,
                }
            objects_list.append(entry)
        return objects_list

    @staticmethod
    def _build_summary(
        objects,
        rooms,
        relations,
        topology_edges,
        floors_list,
        objects_list,
        rooms_list,
        groups_list,
        views_list,
        kg_stats,
        tracker: InstanceTracker,
    ) -> list[str]:
        """Build natural-language summary parts for the scene graph."""
        summary_parts: list[str] = []
        floor_desc = f"{len(floors_list)} floors, " if len(floors_list) > 1 else ""
        summary_parts.append(
            f"Scene has {len(objects_list)} objects, {floor_desc}"
            f"{len(rooms_list)} rooms, {len(groups_list)} groups, {len(views_list)} views."
        )
        if kg_stats["dangerous"] > 0:
            summary_parts.append(f"⚠️ {kg_stats['dangerous']} dangerous/forbidden objects detected.")
        for rm in rooms:
            labels = [objects[oid].label for oid in rm.object_ids if oid in objects]
            if labels:
                summary_parts.append(f"{rm.name}: contains {', '.join(labels[:8])}")

        # Key close-range relations
        for rel in relations[:10]:
            subj = objects.get(rel.subject_id)
            obj_ = objects.get(rel.object_id)
            if subj and obj_:
                summary_parts.append(f"{subj.label} is {rel.relation} {obj_.label} ({rel.distance}m)")

        # Topology summary
        if topology_edges:
            summary_parts.append(f"Topology: {len(topology_edges)} room connections")
            for te in topology_edges[:5]:
                fr = te.get("from_room", "?")
                to = te.get("to_room", "?")
                et = te.get("type", "?")
                fr_name = next((r.name for r in rooms if r.room_id == fr), f"room_{fr}")
                to_name = next((r.name for r in rooms if r.room_id == to), f"room_{to}")
                summary_parts.append(f"{fr_name} ↔ {to_name} ({et})")

        return summary_parts

    @staticmethod
    def _build_phantom_list(tracker: InstanceTracker) -> list[dict]:
        """Serialize phantom (blind-spot prediction) nodes."""
        return [
            {
                "phantom_id": p.phantom_id,
                "label": p.label,
                "room_id": p.room_id,
                "room_type": p.room_type,
                "position": {
                    "x": round(float(p.position[0]), 2),
                    "y": round(float(p.position[1]), 2) if len(p.position) > 1 else 0.0,
                },
                "P_exist": round(p.existence_prob, 3),
                "kg_prior_strength": round(p.kg_prior_strength, 3),
                "safety_level": p.safety_level,
                "source": p.source,
            }
            for p in tracker.get_phantom_nodes()
        ]

    @staticmethod
    def _build_room_posteriors(tracker: InstanceTracker) -> dict:
        """Serialize room-type belief posteriors."""
        result: dict = {}
        for rid, rtp in tracker._room_type_posteriors.items():
            top3 = sorted(rtp.hypotheses.items(), key=lambda x: -x[1])[:3]
            result[str(rid)] = {
                "best_type": rtp.best_type,
                "confidence": round(rtp.best_confidence, 3),
                "entropy": round(rtp.entropy, 3),
                "top3": [{t: round(p, 3)} for t, p in top3],
            }
        return result

    @staticmethod
    def _summarize_diff(events: list[dict]) -> str:
        """Generate a natural-language summary of scene changes."""
        if not events:
            return "No changes detected."
        parts: list[str] = []
        added = [e for e in events if e["type"] == "object_added"]
        removed = [e for e in events if e["type"] == "object_removed"]
        moved = [e for e in events if e["type"] == "object_moved"]
        if added:
            labels = [e["label"] for e in added[:5]]
            parts.append(f"{len(added)} added: {', '.join(labels)}")
        if removed:
            labels = [e["label"] for e in removed[:5]]
            parts.append(f"{len(removed)} removed: {', '.join(labels)}")
        if moved:
            descs = [f"{e['label']}({e['displacement']:.1f}m)" for e in moved[:5]]
            parts.append(f"{len(moved)} moved: {', '.join(descs)}")
        return " | ".join(parts)
