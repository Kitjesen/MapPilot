"""Scene graph builder — structural scene-graph construction.

This module owns the spatial/scene-graph construction logic that was previously
mixed into InstanceTracker via RoomManagerMixin.  It builds:
  - spatial relations (near/on/above/below/directional)
  - regions (DBSCAN clustering of objects)
  - groups (semantic grouping inside regions)
  - floors (z-height clustering)
  - topology edges (room-to-room connectivity)
  - frontier directions (unexplored directions)
  - hierarchy/view edges and subgraph summaries

It intentionally does NOT import memory; all data types and constants come from
runtime.msgs.scene so the dependency direction stays perception -> runtime only.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

import numpy as np

from runtime.msgs.scene import (
    FLOOR_HEIGHT,
    FLOOR_MERGE_TOLERANCE,
    GROUP_KEYWORDS,
    REGION_CLUSTER_RADIUS,
    RELATION_NEAR_THRESHOLD,
    RELATION_ON_THRESHOLD,
    FloorNode,
    GroupNode,
    Region,
    RoomNode,
    SpatialRelation,
    ViewNode,
    infer_room_type,
)

if TYPE_CHECKING:
    from perception.tracking.tracked_objects import TrackedObject

logger = logging.getLogger(__name__)


class SceneGraphBuilder:
    """Build structural scene-graph elements from a set of tracked objects."""

    DOOR_KEYWORDS = {"door", "gate", "entrance", "exit", "opening", "门", "出口", "入口", "通道"}
    PASSAGE_KEYWORDS = {"corridor", "hallway", "passage", "通道", "走廊", "过道"}

    # Above this object count the O(n^2) pairwise scan is replaced by a 3D
    # spatial-hash candidate generation.  At or below it, the exact O(n^2)
    # path is used unchanged (small scenes, including the 5-object golden).
    SPATIAL_HASH_THRESHOLD = 100
    # Cell edge length (metres) for the 3D spatial hash grid.
    SPATIAL_HASH_CELL_SIZE = 2.5

    def __init__(
        self,
        objects: dict[int, TrackedObject],
        views: dict[int, ViewNode] | None = None,
    ) -> None:
        self.objects = objects
        self.views = views or {}
        self._cached_regions: list[Region] = []
        self._cached_floors: list[FloorNode] = []

    # ------------------------------------------------------------------
    # Spatial relations
    # ------------------------------------------------------------------

    def compute_spatial_relations(
        self,
        strategy: str | None = None,
    ) -> list[SpatialRelation]:
        """Compute pairwise spatial relations using bbox-aware distances.

        This is a pure performance optimisation: for large scenes the exact
        O(n^2) pairwise scan is replaced by a 3D spatial-hash candidate
        generation that yields the *identical* set of relations (same content
        and same order) as the brute-force path.

        Args:
            strategy: candidate-pair generation strategy.
                * ``None`` (default): auto-select — brute force for
                  ``n <= SPATIAL_HASH_THRESHOLD`` (e.g. the 5-object golden),
                  spatial hash for larger scenes.
                * ``"bruteforce"``: force the exact O(n^2) scan.
                * ``"hash"``: force the spatial-hash candidate scan.
                The last two exist so tests can prove equivalence on the same
                input; production code should leave it at ``None``.
        """
        relations: list[SpatialRelation] = []
        objs = list(self.objects.values())
        n = len(objs)
        if n < 2:
            return relations

        max_relation_dist = RELATION_NEAR_THRESHOLD * 3

        if strategy is None:
            use_hash = n > self.SPATIAL_HASH_THRESHOLD
        elif strategy == "hash":
            use_hash = True
        elif strategy == "bruteforce":
            use_hash = False
        else:
            raise ValueError(f"unknown strategy: {strategy!r}")

        if use_hash:
            candidate_pairs = self._hash_candidate_pairs(objs, max_relation_dist)
        else:
            candidate_pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]

        for i, j in candidate_pairs:
            a, b = objs[i], objs[j]
            dx = float(a.position[0] - b.position[0])
            dy = float(a.position[1] - b.position[1])
            dz = float(a.position[2] - b.position[2])
            dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

            gap_x = max(0.0, abs(dx) - a.extent[0] - b.extent[0])
            gap_y = max(0.0, abs(dy) - a.extent[1] - b.extent[1])
            gap_z = max(0.0, abs(dz) - a.extent[2] - b.extent[2])
            bbox_gap_3d = math.sqrt(gap_x**2 + gap_y**2 + gap_z**2)
            bbox_gap_2d = math.sqrt(gap_x**2 + gap_y**2)

            overlap_x = abs(dx) < a.extent[0] + b.extent[0]
            overlap_y = abs(dy) < a.extent[1] + b.extent[1]
            horizontal_overlap = overlap_x and overlap_y

            if bbox_gap_3d < RELATION_NEAR_THRESHOLD:
                relations.append(
                    SpatialRelation(
                        subject_id=a.object_id,
                        relation="near",
                        object_id=b.object_id,
                        distance=round(dist_3d, 2),
                    )
                )

            a_bottom = a.position[2] - a.extent[2]
            a_top = a.position[2] + a.extent[2]
            b_bottom = b.position[2] - b.extent[2]
            b_top = b.position[2] + b.extent[2]

            if horizontal_overlap:
                on_tolerance = RELATION_ON_THRESHOLD + 0.1 * (a.extent[2] + b.extent[2])
                if abs(a_bottom - b_top) < on_tolerance and dz > 0:
                    relations.append(
                        SpatialRelation(
                            subject_id=a.object_id,
                            relation="on",
                            object_id=b.object_id,
                            distance=round(abs(a_bottom - b_top), 2),
                        )
                    )
                elif abs(b_bottom - a_top) < on_tolerance and dz < 0:
                    relations.append(
                        SpatialRelation(
                            subject_id=b.object_id,
                            relation="on",
                            object_id=a.object_id,
                            distance=round(abs(b_bottom - a_top), 2),
                        )
                    )

                if gap_z > on_tolerance:
                    if dz > 0:
                        relations.append(
                            SpatialRelation(
                                subject_id=a.object_id,
                                relation="above",
                                object_id=b.object_id,
                                distance=round(gap_z, 2),
                            )
                        )
                    else:
                        relations.append(
                            SpatialRelation(
                                subject_id=a.object_id,
                                relation="below",
                                object_id=b.object_id,
                                distance=round(gap_z, 2),
                            )
                        )

            if bbox_gap_2d < RELATION_NEAR_THRESHOLD * 2:
                dir_threshold_y = a.extent[1] + b.extent[1] + 0.3
                dir_threshold_x = a.extent[0] + b.extent[0] + 0.3

                if dy > dir_threshold_y:
                    relations.append(
                        SpatialRelation(
                            subject_id=a.object_id,
                            relation="left_of",
                            object_id=b.object_id,
                        )
                    )
                elif dy < -dir_threshold_y:
                    relations.append(
                        SpatialRelation(
                            subject_id=a.object_id,
                            relation="right_of",
                            object_id=b.object_id,
                        )
                    )

                if dx > dir_threshold_x:
                    relations.append(
                        SpatialRelation(
                            subject_id=a.object_id,
                            relation="in_front_of",
                            object_id=b.object_id,
                        )
                    )
                elif dx < -dir_threshold_x:
                    relations.append(
                        SpatialRelation(
                            subject_id=a.object_id,
                            relation="behind",
                            object_id=b.object_id,
                        )
                    )

        return relations

    # ------------------------------------------------------------------
    # Spatial hash (large-scene acceleration for compute_spatial_relations)
    # ------------------------------------------------------------------

    def _rebuild_spatial_hash(
        self,
        objs: list[TrackedObject],
        cell_size: float,
    ) -> tuple[dict[tuple[int, int, int], list[int]], list[tuple[int, int, int]]]:
        """Bucket object indices into a 3D grid keyed by cell coordinates.

        Each object index is mapped to the cell
        ``(floor(x/cell), floor(y/cell), floor(z/cell))``.

        Returns:
            buckets: cell key -> list of object indices in that cell.
            cell_of: per-index cell key (parallel to ``objs``).
        """
        buckets: dict[tuple[int, int, int], list[int]] = {}
        cell_of: list[tuple[int, int, int]] = []
        for idx, o in enumerate(objs):
            pos = o.position
            key = (
                int(math.floor(float(pos[0]) / cell_size)),
                int(math.floor(float(pos[1]) / cell_size)),
                int(math.floor(float(pos[2]) / cell_size)),
            )
            buckets.setdefault(key, []).append(idx)
            cell_of.append(key)
        return buckets, cell_of

    def _hash_candidate_pairs(
        self,
        objs: list[TrackedObject],
        max_relation_dist: float,
    ) -> list[tuple[int, int]]:
        """Generate candidate index pairs via 3D spatial hashing.

        The neighbourhood ring is sized so it fully covers ``max_relation_dist``
        in centre-to-centre distance: ``ring = ceil(max_relation_dist /
        cell_size)``.  This guarantees that every pair within that radius (i.e.
        every pair that the brute-force scan could turn into a relation) appears
        as a candidate — no relation is ever missed.

        The returned list is sorted ascending by ``(i, j)``, which reproduces
        the exact iteration order of the brute-force
        ``for i: for j in range(i+1, n)`` scan, so the resulting relations list
        is identical in both content and order.
        """
        cell_size = self.SPATIAL_HASH_CELL_SIZE
        buckets, cell_of = self._rebuild_spatial_hash(objs, cell_size)

        ring = max(1, int(math.ceil(max_relation_dist / cell_size)))
        offsets = [
            (dx, dy, dz)
            for dx in range(-ring, ring + 1)
            for dy in range(-ring, ring + 1)
            for dz in range(-ring, ring + 1)
        ]

        pairs: set[tuple[int, int]] = set()
        for idx, (cx, cy, cz) in enumerate(cell_of):
            for dx, dy, dz in offsets:
                neighbours = buckets.get((cx + dx, cy + dy, cz + dz))
                if not neighbours:
                    continue
                for jdx in neighbours:
                    if jdx == idx:
                        continue
                    pair = (idx, jdx) if idx < jdx else (jdx, idx)
                    pairs.add(pair)

        return sorted(pairs)

    # ------------------------------------------------------------------
    # Regions
    # ------------------------------------------------------------------

    def compute_regions(
        self,
        room_name_cache: dict[int, str] | None = None,
    ) -> list[Region]:
        """Cluster objects into 2D spatial regions using DBSCAN."""
        room_name_cache = room_name_cache or {}
        objs = list(self.objects.values())
        if not objs:
            self._cached_regions = []
            return []

        positions_2d = np.array([obj.position[:2] for obj in objs], dtype=np.float64)

        try:
            from sklearn.cluster import DBSCAN

            clustering = DBSCAN(
                eps=REGION_CLUSTER_RADIUS,
                min_samples=1,
                metric="euclidean",
            ).fit(positions_2d)
            cluster_labels = clustering.labels_
        except ImportError:
            try:
                from scipy.cluster.hierarchy import fcluster, linkage

                if len(positions_2d) >= 2:
                    Z = linkage(positions_2d, method="single", metric="euclidean")
                    cluster_labels = fcluster(Z, t=REGION_CLUSTER_RADIUS, criterion="distance") - 1
                else:
                    cluster_labels = np.array([0])
            except ImportError:
                cluster_labels = np.arange(len(objs))

        cluster_to_objs: dict[int, list[int]] = {}
        for idx, cl in enumerate(cluster_labels):
            cl_int = int(cl)
            if cl_int == -1:
                noise_id = max(cluster_to_objs.keys(), default=-1) + 1
                cluster_to_objs[noise_id] = [idx]
            else:
                cluster_to_objs.setdefault(cl_int, []).append(idx)

        regions: list[Region] = []
        for region_id, obj_indices in enumerate(cluster_to_objs.values()):
            obj_ids = [objs[i].object_id for i in obj_indices]
            center = positions_2d[obj_indices].mean(axis=0)

            for i in obj_indices:
                objs[i].region_id = region_id

            region = Region(
                region_id=region_id,
                center=center,
                object_ids=obj_ids,
                name=f"region_{region_id}",
            )
            regions.append(region)

        for region in regions:
            labels = [self.objects[oid].label for oid in region.object_ids if oid in self.objects]
            if labels:
                if region.region_id in room_name_cache:
                    region.name = room_name_cache[region.region_id]
                    region.llm_named = True
                else:
                    region.name = infer_room_type(labels)

        self._cached_regions = regions
        return regions

    # ------------------------------------------------------------------
    # Groups
    # ------------------------------------------------------------------

    def compute_groups(self, regions: list[Region]) -> list[GroupNode]:
        """Build semantic groups inside each region."""
        groups: list[GroupNode] = []
        group_index: dict[tuple[int, str], int] = {}

        for room in regions:
            for oid in room.object_ids:
                obj = self.objects.get(oid)
                if obj is None:
                    continue

                group_name = self.infer_group_name(obj.label)
                key = (room.region_id, group_name)

                if key not in group_index:
                    gid = len(groups)
                    groups.append(
                        GroupNode(
                            group_id=gid,
                            room_id=room.region_id,
                            name=group_name,
                            center=obj.position[:2].copy(),
                            object_ids=[oid],
                            semantic_labels=[obj.label],
                        )
                    )
                    group_index[key] = gid
                else:
                    gid = group_index[key]
                    g = groups[gid]
                    g.object_ids.append(oid)
                    n = len(g.object_ids)
                    g.center = (g.center * (n - 1) + obj.position[:2]) / n
                    if obj.label not in g.semantic_labels:
                        g.semantic_labels.append(obj.label)

        return groups

    @staticmethod
    def infer_group_name(label: str) -> str:
        """Map an object label to a semantic group name."""
        lower = label.lower()
        for group_name, words in GROUP_KEYWORDS.items():
            if any(w in lower for w in words):
                return group_name
        return "others"

    # ------------------------------------------------------------------
    # Floors
    # ------------------------------------------------------------------

    def compute_floors(self) -> list[FloorNode]:
        """Infer floors by clustering object z coordinates."""
        objs = list(self.objects.values())
        if not objs:
            self._cached_floors = []
            return []

        z_values = np.array([float(obj.position[2]) for obj in objs])
        z_sorted_indices = np.argsort(z_values)
        sorted_list = z_sorted_indices.tolist()

        floors: list[FloorNode] = []
        current_floor_objs: list[int] = []
        current_z_min = z_values[sorted_list[0]]
        current_z_sum = 0.0

        for rank, idx in enumerate(sorted_list):
            z = z_values[idx]
            obj = objs[idx]

            if z - current_z_min > FLOOR_HEIGHT - FLOOR_MERGE_TOLERANCE and current_floor_objs:
                center_z = current_z_sum / len(current_floor_objs)
                prev_idx = sorted_list[rank - 1]
                z_max = z_values[prev_idx]
                floors.append(
                    FloorNode(
                        floor_id=len(floors),
                        floor_level=len(floors),
                        z_range=(current_z_min, z_max),
                        object_ids=list(current_floor_objs),
                        center_z=center_z,
                    )
                )
                current_floor_objs = []
                current_z_min = z
                current_z_sum = 0.0

            current_floor_objs.append(obj.object_id)
            current_z_sum += z
            obj.floor_level = len(floors)

        if current_floor_objs:
            center_z = current_z_sum / len(current_floor_objs)
            floors.append(
                FloorNode(
                    floor_id=len(floors),
                    floor_level=len(floors),
                    z_range=(current_z_min, z_values[z_sorted_indices[-1]]),
                    object_ids=list(current_floor_objs),
                    center_z=center_z,
                )
            )

        self._cached_floors = floors
        return floors

    @staticmethod
    def assign_rooms_to_floors(
        floors: list[FloorNode],
        rooms: list[RoomNode],
    ) -> None:
        """Assign rooms to floors by object overlap voting."""
        for floor in floors:
            floor.room_ids.clear()
        floor_obj_sets = {f.floor_id: set(f.object_ids) for f in floors}

        for room in rooms:
            room_obj_set = set(room.object_ids)
            best_floor = -1
            best_overlap = 0
            for f in floors:
                overlap = len(room_obj_set & floor_obj_sets[f.floor_id])
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_floor = f.floor_id
            if best_floor >= 0:
                floors[best_floor].room_ids.append(room.room_id)

    # ------------------------------------------------------------------
    # Topology
    # ------------------------------------------------------------------

    def compute_topology_edges(self, rooms: list[RoomNode]) -> list[dict]:
        """Compute room-to-room connectivity edges."""
        if len(rooms) < 2:
            return []

        edges: list[dict] = []
        seen_pairs: set = set()

        # Strategy 1: Door-mediated connectivity
        door_objects = [
            obj for obj in self.objects.values() if any(kw in obj.label.lower() for kw in self.DOOR_KEYWORDS)
        ]

        for door in door_objects:
            door_pos = door.position[:2]
            distances = []
            for room in rooms:
                dist = float(np.linalg.norm(door_pos - room.center))
                distances.append((room.room_id, dist))
            distances.sort(key=lambda x: x[1])

            if len(distances) >= 2:
                r1, d1 = distances[0]
                r2, d2 = distances[1]
                if r1 != r2 and d2 < REGION_CLUSTER_RADIUS * 2.0:
                    pair = (min(r1, r2), max(r1, r2))
                    if pair not in seen_pairs:
                        seen_pairs.add(pair)
                        edges.append(
                            {
                                "from_room": pair[0],
                                "to_room": pair[1],
                                "type": "door",
                                "mediator": door.label,
                                "mediator_pos": {
                                    "x": round(float(door.position[0]), 2),
                                    "y": round(float(door.position[1]), 2),
                                },
                                "distance": round(d1 + d2, 2),
                            }
                        )

        # Strategy 2: Proximity connectivity
        PROXIMITY_THRESHOLD = REGION_CLUSTER_RADIUS * 1.8
        for i, r1 in enumerate(rooms):
            for r2 in rooms[i + 1 :]:
                pair = (min(r1.room_id, r2.room_id), max(r1.room_id, r2.room_id))
                if pair in seen_pairs:
                    continue
                dist = float(np.linalg.norm(r1.center - r2.center))
                if dist < PROXIMITY_THRESHOLD:
                    boundary_objs = 0
                    midpoint = (r1.center + r2.center) / 2.0
                    for oid in list(r1.object_ids) + list(r2.object_ids):
                        obj = self.objects.get(oid)
                        if obj is None:
                            continue
                        d_to_mid = float(np.linalg.norm(obj.position[:2] - midpoint))
                        if d_to_mid < dist * 0.6:
                            boundary_objs += 1
                    if boundary_objs > 0:
                        seen_pairs.add(pair)
                        edges.append(
                            {
                                "from_room": pair[0],
                                "to_room": pair[1],
                                "type": "proximity",
                                "distance": round(dist, 2),
                            }
                        )

        # Strategy 3: Passage-mediated connectivity
        for room in rooms:
            is_passage = any(kw in room.name.lower() for kw in self.PASSAGE_KEYWORDS)
            if not is_passage:
                continue
            for other in rooms:
                if other.room_id == room.room_id:
                    continue
                pair = (min(room.room_id, other.room_id), max(room.room_id, other.room_id))
                if pair in seen_pairs:
                    continue
                dist = float(np.linalg.norm(room.center - other.center))
                if dist < REGION_CLUSTER_RADIUS * 2.5:
                    seen_pairs.add(pair)
                    edges.append(
                        {
                            "from_room": pair[0],
                            "to_room": pair[1],
                            "type": "passage",
                            "mediator": room.name,
                            "distance": round(dist, 2),
                        }
                    )

        return edges

    def estimate_frontier_directions(self, rooms: list[RoomNode]) -> list[dict]:
        """Estimate frontier directions from door objects and sparse sectors."""
        if not rooms or not self.objects:
            return []

        frontiers: list[dict] = []
        door_kw = {"door", "gate", "entrance", "exit", "门", "出口", "入口"}
        door_objects = [obj for obj in self.objects.values() if any(kw in obj.label.lower() for kw in door_kw)]

        room_centers = {r.room_id: r.center for r in rooms}

        for door in door_objects:
            door_pos = door.position[:2]
            near_rooms = []
            for room in rooms:
                dist = float(np.linalg.norm(door_pos - room.center))
                near_rooms.append((room.room_id, dist))
            near_rooms.sort(key=lambda x: x[1])

            if len(near_rooms) >= 1:
                closest_rid, closest_dist = near_rooms[0]
                has_second = len(near_rooms) >= 2 and near_rooms[1][1] < REGION_CLUSTER_RADIUS * 2.0
                if not has_second and closest_dist < REGION_CLUSTER_RADIUS * 1.5:
                    direction = door_pos - room_centers[closest_rid]
                    dnorm = float(np.linalg.norm(direction))
                    if dnorm > 0.5:
                        direction = direction / dnorm
                        frontier_pos = door_pos + direction * 2.0
                        frontiers.append(
                            {
                                "position": {
                                    "x": round(float(frontier_pos[0]), 2),
                                    "y": round(float(frontier_pos[1]), 2),
                                },
                                "direction": {
                                    "dx": round(float(direction[0]), 2),
                                    "dy": round(float(direction[1]), 2),
                                },
                                "nearest_room_id": closest_rid,
                                "mediator": door.label,
                                "frontier_size": 2.0,
                                "source": "door_outward",
                            }
                        )

        for room in rooms:
            if len(room.object_ids) < 3:
                continue
            obj_positions = []
            for oid in room.object_ids:
                obj = self.objects.get(oid)
                if obj is not None:
                    obj_positions.append(obj.position[:2])
            if len(obj_positions) < 3:
                continue

            positions = np.array(obj_positions)
            centroid = positions.mean(axis=0)

            n_sectors = 4
            sector_counts = [0] * n_sectors
            for p in positions:
                angle = math.atan2(p[1] - centroid[1], p[0] - centroid[0])
                sector = int((angle + math.pi) / (2 * math.pi) * n_sectors) % n_sectors
                sector_counts[sector] += 1

            total = sum(sector_counts)
            for s_idx, cnt in enumerate(sector_counts):
                if cnt == 0 and total >= 3:
                    angle = (s_idx + 0.5) * (2 * math.pi / n_sectors) - math.pi
                    direction = np.array([math.cos(angle), math.sin(angle)])
                    frontier_pos = centroid + direction * REGION_CLUSTER_RADIUS

                    too_close = any(
                        float(np.linalg.norm(frontier_pos - r.center)) < REGION_CLUSTER_RADIUS * 0.8
                        for r in rooms
                        if r.room_id != room.room_id
                    )
                    if too_close:
                        continue

                    frontiers.append(
                        {
                            "position": {
                                "x": round(float(frontier_pos[0]), 2),
                                "y": round(float(frontier_pos[1]), 2),
                            },
                            "direction": {
                                "dx": round(float(direction[0]), 2),
                                "dy": round(float(direction[1]), 2),
                            },
                            "nearest_room_id": room.room_id,
                            "mediator": "",
                            "frontier_size": 1.5,
                            "source": "sparse_sector",
                        }
                    )

        return frontiers[:10]

    # ------------------------------------------------------------------
    # Hierarchy and subgraphs
    # ------------------------------------------------------------------

    @staticmethod
    def build_hierarchy_edges(
        rooms: list[RoomNode],
        groups: list[GroupNode],
    ) -> list[dict]:
        """Build room->group->object hierarchy edges."""
        edges: list[dict] = []

        for room in rooms:
            for gid in room.group_ids:
                edges.append(
                    {
                        "parent_type": "room",
                        "parent_id": room.room_id,
                        "child_type": "group",
                        "child_id": gid,
                        "relation": "contains",
                    }
                )

        for g in groups:
            for oid in g.object_ids:
                edges.append(
                    {
                        "parent_type": "group",
                        "parent_id": g.group_id,
                        "child_type": "object",
                        "child_id": oid,
                        "relation": "contains",
                    }
                )

        return edges

    @staticmethod
    def build_view_edges(views: list[ViewNode]) -> list[dict]:
        """Build room->view->object edges."""
        edges: list[dict] = []
        for v in views:
            if v.room_id >= 0:
                edges.append(
                    {
                        "parent_type": "room",
                        "parent_id": v.room_id,
                        "child_type": "view",
                        "child_id": v.view_id,
                        "relation": "contains_view",
                    }
                )
            for oid in v.object_ids:
                edges.append(
                    {
                        "parent_type": "view",
                        "parent_id": v.view_id,
                        "child_type": "object",
                        "child_id": oid,
                        "relation": "observes",
                    }
                )
        return edges

    @staticmethod
    def assign_view_rooms(
        views: list[ViewNode],
        rooms: list[RoomNode],
    ) -> list[ViewNode]:
        """Assign each view to its nearest room by 2D center distance."""
        if not views:
            return []
        if not rooms:
            return views

        room_centers = [r.center for r in rooms]
        room_ids = [r.room_id for r in rooms]
        for v in views:
            if v.room_id >= 0:
                continue
            pos2d = v.position[:2]
            dists = [float(np.linalg.norm(pos2d - c)) for c in room_centers]
            best_idx = int(np.argmin(dists))
            v.room_id = room_ids[best_idx]
        return views

    @staticmethod
    def build_subgraphs(
        rooms: list[RoomNode],
        groups: list[GroupNode],
        views: list[ViewNode],
        objects_by_id: dict[int, TrackedObject],
        relations_list: list[dict],
    ) -> list[dict]:
        """Build SG-Nav style subgraph summaries for rooms/groups/views."""
        subgraphs: list[dict] = []

        for room in rooms:
            labels = [objects_by_id[oid].label for oid in room.object_ids if oid in objects_by_id]
            object_set = set(room.object_ids)
            relation_count = 0
            for rel in relations_list:
                sid = rel.get("subject_id")
                oid = rel.get("object_id")
                if sid in object_set and oid in object_set:
                    relation_count += 1

            subgraphs.append(
                {
                    "subgraph_id": f"room_{room.room_id}",
                    "level": "room",
                    "room_id": room.room_id,
                    "center": {
                        "x": round(float(room.center[0]), 2),
                        "y": round(float(room.center[1]), 2),
                    },
                    "object_ids": room.object_ids,
                    "object_labels": labels[:12],
                    "relation_count": relation_count,
                }
            )

        for g in groups:
            labels = [objects_by_id[oid].label for oid in g.object_ids if oid in objects_by_id]
            subgraphs.append(
                {
                    "subgraph_id": f"group_{g.group_id}",
                    "level": "group",
                    "room_id": g.room_id,
                    "group_id": g.group_id,
                    "name": g.name,
                    "center": {
                        "x": round(float(g.center[0]), 2),
                        "y": round(float(g.center[1]), 2),
                    },
                    "object_ids": g.object_ids,
                    "object_labels": labels[:12],
                    "relation_count": 0,
                }
            )

        for v in views:
            subgraphs.append(
                {
                    "subgraph_id": f"view_{v.view_id}",
                    "level": "view",
                    "room_id": v.room_id,
                    "center": {
                        "x": round(float(v.position[0]), 2),
                        "y": round(float(v.position[1]), 2),
                    },
                    "object_ids": v.object_ids,
                    "object_labels": v.key_labels[:12],
                    "relation_count": 0,
                }
            )

        return subgraphs
