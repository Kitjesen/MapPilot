"""lingtu.runtime.msgs.semantic 鈥?semantic navigation message types.

Defines Detection3D, SceneGraph, GoalResult, and NavigationCommand.
for LingTu semantic perception and command contracts.
"""

from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Any, ClassVar

from runtime.tf.frames import map_frame_id

from .geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from .numpy_compat import np
from .protocol import json_message_decode, json_message_encode
from .sensor import CameraIntrinsics, Image

SEMANTIC_MSG_MAP_FRAME_ID = map_frame_id()


@dataclass(frozen=True)
class SemanticObservation:
    """One Host-local perception result, or an immediate context invalidation.

    Images and geometry are shared read-only references, not a new wire format.
    Sequence orders publications in this Host; camera stream sequences remain
    on the source messages. Stable person identity belongs to Follow.
    """

    product_session_id: str = ""
    sequence: int = 0
    source_ts: float = 0.0
    completed_ts: float = 0.0
    source_generation: int = 0
    map_id: str = ""
    content_epoch: int = 0
    status: str = "unavailable"
    reason: str = ""
    image: Image | None = None
    depth: Image | None = None
    camera_info: CameraIntrinsics | None = None
    robot_pose: PoseStamped | None = None
    map_from_camera: Any = None
    detections: tuple[Detection3D, ...] = ()
    scene_graph: SceneGraph | None = None
    odometry_ts: float = 0.0
    map_odom_ts: float = 0.0

# ---------------------------------------------------------------------------
# Detection3D
# ---------------------------------------------------------------------------

@dataclass
class Detection3D:
    """3D detection result."""

    msg_name: ClassVar[str] = "vision_msgs.Detection3D"
    id: str = ""
    label: str = ""
    confidence: float = 0.0
    position: Vector3 = field(default_factory=Vector3)
    bbox_2d: list[float] = field(default_factory=list)  # [x1, y1, x2, y2]
    clip_feature: np.ndarray | None = None  # 512-dim
    dino_feature: np.ndarray | None = None  # DINOv3 appearance feature
    ts: float = 0.0

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    def distance_to(self, other: Detection3D) -> float:
        """Euclidean distance to another detection."""
        dx = self.position.x - other.position.x
        dy = self.position.y - other.position.y
        dz = self.position.z - other.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    # -- serialization ---------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "confidence": self.confidence,
            "position": [self.position.x, self.position.y, self.position.z],
            "bbox_2d": self.bbox_2d,
            "has_clip_feature": self.clip_feature is not None,
            "has_dino_feature": self.dino_feature is not None,
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Detection3D:
        pos = d.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            position = Vector3(float(pos.get("x", 0)), float(pos.get("y", 0)),
                               float(pos.get("z", 0)))
        else:
            position = Vector3(float(pos[0]), float(pos[1]), float(pos[2]))
        return cls(
            id=str(d.get("id", "")),
            label=str(d.get("label", "")),
            confidence=float(d.get("confidence", 0)),
            position=position,
            bbox_2d=list(d.get("bbox_2d", [])),
            ts=float(d.get("ts", 0)),
        )

    def encode(self) -> bytes:
        """Binary encoding."""
        id_b = self.id.encode("utf-8")
        label_b = self.label.encode("utf-8")
        bbox = self.bbox_2d + [0.0] * max(0, 4 - len(self.bbox_2d))
        has_clip = 1 if self.clip_feature is not None else 0
        header = struct.pack(
            "<HHf3d4ddB",
            len(id_b), len(label_b), self.confidence,
            self.position.x, self.position.y, self.position.z,
            bbox[0], bbox[1], bbox[2], bbox[3],
            self.ts, has_clip,
        )
        parts = [header, id_b, label_b]
        if self.clip_feature is not None:
            clip_arr = np.asarray(self.clip_feature, dtype=np.float32)
            parts.append(struct.pack("<I", clip_arr.size))
            parts.append(clip_arr.tobytes())
        if self.dino_feature is not None:
            dino_arr = np.asarray(self.dino_feature, dtype=np.float32)
            parts.append(struct.pack("<BI", 1, dino_arr.size))
            parts.append(dino_arr.tobytes())
        return b"".join(parts)

    @classmethod
    def decode(cls, data: bytes) -> Detection3D:
        # header: 2H(4) + f(4) + 3d(24) + 4d(32) + d(8) + B(1) = 73 bytes
        hdr_fmt = struct.Struct("<HHf3d4ddB")
        vals = hdr_fmt.unpack(data[: hdr_fmt.size])
        id_len, label_len, conf = vals[0], vals[1], vals[2]
        px, py, pz = vals[3], vals[4], vals[5]
        b0, b1, b2, b3 = vals[6], vals[7], vals[8], vals[9]
        ts_val = vals[10]
        has_clip = vals[11]
        off = hdr_fmt.size
        det_id = data[off: off + id_len].decode("utf-8")
        off += id_len
        label = data[off: off + label_len].decode("utf-8")
        off += label_len
        clip_feature = None
        if has_clip:
            clip_size = struct.unpack("<I", data[off: off + 4])[0]
            off += 4
            clip_feature = np.frombuffer(data[off: off + clip_size * 4], dtype=np.float32).copy()
            off += clip_size * 4
        dino_feature = None
        if len(data) > off:
            has_dino = struct.unpack("<B", data[off: off + 1])[0]
            off += 1
            if has_dino:
                dino_size = struct.unpack("<I", data[off: off + 4])[0]
                off += 4
                dino_feature = np.frombuffer(
                    data[off: off + dino_size * 4], dtype=np.float32
                ).copy()
        return cls(
            id=det_id, label=label, confidence=conf,
            position=Vector3(px, py, pz),
            bbox_2d=[b0, b1, b2, b3], clip_feature=clip_feature,
            dino_feature=dino_feature, ts=ts_val,
        )

    def __repr__(self) -> str:
        clip_str = f", clip={self.clip_feature.shape}" if self.clip_feature is not None else ""
        dino_str = f", dino={self.dino_feature.shape}" if self.dino_feature is not None else ""
        return (
            f"Detection3D(id='{self.id}', label='{self.label}', "
            f"conf={self.confidence:.2f}, pos=[{self.position.x:.2f}, "
            f"{self.position.y:.2f}, {self.position.z:.2f}]{clip_str}{dino_str})"
        )


# ---------------------------------------------------------------------------
# Relation / Region (SceneGraph sub-structures)
# ---------------------------------------------------------------------------

@dataclass
class Relation:
    """Scene graph relation edge."""

    msg_name: ClassVar[str] = "lingtu_msgs.Relation"
    subject_id: str = ""
    predicate: str = ""  # near, on, left_of, right_of, ...
    object_id: str = ""
    confidence: float = 1.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "subject_id": self.subject_id,
            "predicate": self.predicate,
            "object_id": self.object_id,
            "confidence": self.confidence,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Relation:
        return cls(
            subject_id=str(d.get("subject_id", "")),
            predicate=str(d.get("predicate", "")),
            object_id=str(d.get("object_id", "")),
            confidence=float(d.get("confidence", 1.0)),
        )

    def __repr__(self) -> str:
        return f"Relation({self.subject_id} --{self.predicate}--> {self.object_id})"

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> Relation:
        return json_message_decode(data, cls)


@dataclass
class Region:
    """Scene region."""

    msg_name: ClassVar[str] = "lingtu_msgs.Region"
    name: str = ""
    object_ids: list[str] = field(default_factory=list)
    center: Vector3 | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "object_ids": self.object_ids,
            "center": [self.center.x, self.center.y, self.center.z] if self.center else None,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Region:
        c = d.get("center")
        center = Vector3(float(c[0]), float(c[1]), float(c[2])) if c else None
        return cls(
            name=str(d.get("name", "")),
            object_ids=list(d.get("object_ids", [])),
            center=center,
        )

    def __repr__(self) -> str:
        return f"Region('{self.name}', objects={len(self.object_ids)})"

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> Region:
        return json_message_decode(data, cls)


# ---------------------------------------------------------------------------
# SceneGraph
# ---------------------------------------------------------------------------

@dataclass
class SceneGraph:
    """Scene graph, including an optional hierarchical perception snapshot.

    ``objects``/``relations``/``regions`` remain the compatibility surface for
    flat consumers.  Tracker-produced hierarchy data is carried as JSON-shaped
    records so the runtime message does not import perception-owned classes.
    The records are read-only semantic context; native map and navigation
    services remain the authority for geometry and motion.
    """

    msg_name: ClassVar[str] = "lingtu_msgs.SceneGraph"
    objects: list[Detection3D] = field(default_factory=list)
    relations: list[Relation] = field(default_factory=list)
    regions: list[Region] = field(default_factory=list)
    ts: float = 0.0
    frame_id: str = SEMANTIC_MSG_MAP_FRAME_ID
    graph_level: str = "flat"
    graph_version: str = "1.0"
    map_id: str = ""
    content_epoch: int = 0
    floors: list[dict[str, Any]] = field(default_factory=list)
    rooms: list[dict[str, Any]] = field(default_factory=list)
    groups: list[dict[str, Any]] = field(default_factory=list)
    views: list[dict[str, Any]] = field(default_factory=list)
    hierarchy_edges: list[dict[str, Any]] = field(default_factory=list)
    topology_edges: list[dict[str, Any]] = field(default_factory=list)
    frontier_nodes: list[dict[str, Any]] = field(default_factory=list)
    subgraphs: list[dict[str, Any]] = field(default_factory=list)
    phantom_nodes: list[dict[str, Any]] = field(default_factory=list)
    room_type_posteriors: list[dict[str, Any]] = field(default_factory=list)
    belief_propagation: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    # -- query -----------------------------------------------------------------

    def get_object_by_id(self, obj_id: str) -> Detection3D | None:
        """Get object by ID."""
        for obj in self.objects:
            if obj.id == obj_id:
                return obj
        return None

    def get_objects_by_label(self, label: str) -> list[Detection3D]:
        """Get objects by label (case-insensitive)."""
        return [o for o in self.objects if o.label.lower() == label.lower()]

    def validate_hierarchy(self) -> tuple[str, ...]:
        """Return deterministic errors for a hierarchical scene snapshot.

        This is a structural check only.  It verifies references and the
        parent-to-child direction; it does not claim that a semantic relation
        is geometrically true or that a navigation target is safe.
        """
        if self.graph_level != "hierarchical":
            if any((self.floors, self.rooms, self.groups, self.views, self.hierarchy_edges,
                   self.topology_edges, self.subgraphs)):
                return ("hierarchy_data_requires_graph_level_hierarchical",)
            return ()

        errors: list[str] = []
        object_ids = {str(obj.id) for obj in self.objects if obj.id}

        def ids(records: list[dict[str, Any]], key: str) -> set[str]:
            return {str(record[key]) for record in records if isinstance(record, dict) and key in record}

        floor_ids = ids(self.floors, "floor_id")
        room_ids = ids(self.rooms, "room_id")
        group_ids = ids(self.groups, "group_id")
        view_ids = ids(self.views, "view_id")

        def check_refs(records: list[dict[str, Any]], field_name: str, valid: set[str], owner: str) -> None:
            for record in records:
                if not isinstance(record, dict):
                    errors.append(f"{owner}_record_not_object")
                    continue
                values = record.get(field_name, [])
                if not isinstance(values, list):
                    errors.append(f"{owner}_{field_name}_not_list")
                    continue
                for value in values:
                    if str(value) not in valid:
                        errors.append(f"{owner}_{field_name}_unknown:{value}")

        check_refs(self.floors, "room_ids", room_ids, "floor")
        check_refs(self.floors, "object_ids", object_ids, "floor")
        check_refs(self.rooms, "object_ids", object_ids, "room")
        check_refs(self.rooms, "group_ids", group_ids, "room")
        check_refs(self.groups, "object_ids", object_ids, "group")

        node_sets = {
            "floor": floor_ids,
            "room": room_ids,
            "group": group_ids,
            "view": view_ids,
            "object": object_ids,
        }
        parent_types = set(node_sets)
        graph: dict[tuple[str, str], set[tuple[str, str]]] = {}
        for edge in self.hierarchy_edges:
            if not isinstance(edge, dict):
                errors.append("hierarchy_edge_not_object")
                continue
            parent_type = str(edge.get("parent_type", ""))
            child_type = str(edge.get("child_type", ""))
            parent = str(edge.get("parent_id", ""))
            child = str(edge.get("child_id", ""))
            if parent_type not in parent_types or child_type not in parent_types:
                errors.append(f"hierarchy_edge_unknown_type:{parent_type}->{child_type}")
                continue
            if parent not in node_sets[parent_type]:
                errors.append(f"hierarchy_edge_unknown_parent:{parent_type}:{parent}")
            if child not in node_sets[child_type]:
                errors.append(f"hierarchy_edge_unknown_child:{child_type}:{child}")
            graph.setdefault((parent_type, parent), set()).add((child_type, child))

        # A hierarchy must be acyclic.  Relations such as ``near`` are kept in
        # ``relations`` and are intentionally excluded from this check.
        visiting: set[tuple[str, str]] = set()
        visited: set[tuple[str, str]] = set()

        def visit(node: tuple[str, str]) -> None:
            if node in visiting:
                errors.append(f"hierarchy_cycle:{node[0]}:{node[1]}")
                return
            if node in visited:
                return
            visiting.add(node)
            for child in graph.get(node, ()):
                visit(child)
            visiting.remove(node)
            visited.add(node)

        for node in graph:
            visit(node)
        return tuple(dict.fromkeys(errors))

    # -- serialization ---------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "objects": [o.to_dict() for o in self.objects],
            "relations": [r.to_dict() for r in self.relations],
            "regions": [r.to_dict() for r in self.regions],
            "ts": self.ts,
            "frame_id": self.frame_id,
            "graph_level": self.graph_level,
            "graph_version": self.graph_version,
            "map_id": self.map_id,
            "content_epoch": self.content_epoch,
            "floors": self.floors,
            "rooms": self.rooms,
            "groups": self.groups,
            "views": self.views,
            "hierarchy_edges": self.hierarchy_edges,
            "topology_edges": self.topology_edges,
            "frontier_nodes": self.frontier_nodes,
            "subgraphs": self.subgraphs,
            "phantom_nodes": self.phantom_nodes,
            "room_type_posteriors": self.room_type_posteriors,
            "belief_propagation": self.belief_propagation,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SceneGraph:
        return cls(
            objects=[Detection3D.from_dict(o) for o in d.get("objects", [])],
            relations=[Relation.from_dict(r) for r in d.get("relations", [])],
            regions=[Region.from_dict(r) for r in d.get("regions", [])],
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", SEMANTIC_MSG_MAP_FRAME_ID)),
            graph_level=str(d.get("graph_level", "flat")),
            graph_version=str(d.get("graph_version", "1.0")),
            map_id=str(d.get("map_id", "")),
            content_epoch=int(d.get("content_epoch", 0)),
            floors=[dict(v) for v in d.get("floors", []) if isinstance(v, dict)],
            rooms=[dict(v) for v in d.get("rooms", []) if isinstance(v, dict)],
            groups=[dict(v) for v in d.get("groups", []) if isinstance(v, dict)],
            views=[dict(v) for v in d.get("views", []) if isinstance(v, dict)],
            hierarchy_edges=[dict(v) for v in d.get("hierarchy_edges", []) if isinstance(v, dict)],
            topology_edges=[dict(v) for v in d.get("topology_edges", []) if isinstance(v, dict)],
            frontier_nodes=[dict(v) for v in d.get("frontier_nodes", []) if isinstance(v, dict)],
            subgraphs=[dict(v) for v in d.get("subgraphs", []) if isinstance(v, dict)],
            phantom_nodes=[dict(v) for v in d.get("phantom_nodes", []) if isinstance(v, dict)],
            room_type_posteriors=[dict(v) for v in d.get("room_type_posteriors", []) if isinstance(v, dict)],
            belief_propagation=dict(d.get("belief_propagation", {})),
        )

    def to_json(self, **kwargs: Any) -> str:
        """Serialize to JSON string."""
        return json.dumps(self.to_dict(), ensure_ascii=False, **kwargs)

    @classmethod
    def from_json(cls, s: str) -> SceneGraph:
        """Deserialize from JSON string."""
        return cls.from_dict(json.loads(s))

    def encode(self) -> bytes:
        """Binary encoding via JSON intermediate."""
        payload = self.to_json().encode("utf-8")
        return struct.pack("<I", len(payload)) + payload

    @classmethod
    def decode(cls, data: bytes) -> SceneGraph:
        length = struct.unpack("<I", data[:4])[0]
        payload = data[4: 4 + length].decode("utf-8")
        return cls.from_json(payload)

    def __repr__(self) -> str:
        return (
            f"SceneGraph(objects={len(self.objects)}, relations={len(self.relations)}, "
            f"regions={len(self.regions)}, frame='{self.frame_id}')"
        )


# ---------------------------------------------------------------------------
# GoalResult
# ---------------------------------------------------------------------------

@dataclass
class GoalResult:
    """Goal resolution result from Fast/Slow Path."""

    msg_name: ClassVar[str] = "lingtu_msgs.GoalResult"
    action: str = "navigate"  # "navigate" | "explore"
    target_label: str = ""
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    confidence: float = 0.0
    reasoning: str = ""
    is_valid: bool = False
    path: str = ""  # "fast" | "slow"
    hint_room: str = ""
    score_entropy: float = 0.0

    # -- conversion ------------------------------------------------------------

    def as_pose_stamped(self, frame_id: str = SEMANTIC_MSG_MAP_FRAME_ID) -> PoseStamped:
        """Convert to PoseStamped (identity quaternion orientation)."""
        return PoseStamped(
            pose=Pose(
                position=Vector3(self.target_x, self.target_y, self.target_z),
                orientation=Quaternion(),
            ),
            frame_id=frame_id,
        )

    # -- serialization ---------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "action": self.action,
            "target_label": self.target_label,
            "target_x": self.target_x,
            "target_y": self.target_y,
            "target_z": self.target_z,
            "confidence": self.confidence,
            "reasoning": self.reasoning,
            "is_valid": self.is_valid,
            "path": self.path,
            "hint_room": self.hint_room,
            "score_entropy": self.score_entropy,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> GoalResult:
        return cls(
            action=str(d.get("action", "navigate")),
            target_label=str(d.get("target_label", "")),
            target_x=float(d.get("target_x", 0)),
            target_y=float(d.get("target_y", 0)),
            target_z=float(d.get("target_z", 0)),
            confidence=float(d.get("confidence", 0)),
            reasoning=str(d.get("reasoning", "")),
            is_valid=bool(d.get("is_valid", False)),
            path=str(d.get("path", "")),
            hint_room=str(d.get("hint_room", "")),
            score_entropy=float(d.get("score_entropy", 0)),
        )

    def encode(self) -> bytes:
        payload = json.dumps(self.to_dict(), ensure_ascii=False).encode("utf-8")
        return struct.pack("<I", len(payload)) + payload

    @classmethod
    def decode(cls, data: bytes) -> GoalResult:
        length = struct.unpack("<I", data[:4])[0]
        return cls.from_dict(json.loads(data[4: 4 + length].decode("utf-8")))

    def __repr__(self) -> str:
        return (
            f"GoalResult(action='{self.action}', label='{self.target_label}', "
            f"conf={self.confidence:.2f}, valid={self.is_valid}, path='{self.path}')"
        )


# ---------------------------------------------------------------------------
# NavigationCommand
# ---------------------------------------------------------------------------

@dataclass
class NavigationCommand:
    """Navigation command 鈥?convertible to Twist for cmd_vel."""

    msg_name: ClassVar[str] = "lingtu_msgs.NavigationCommand"
    command_type: str = "velocity"  # velocity | goto | label
    target_position: Vector3 | None = None
    target_label: str | None = None
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    confidence: float = 1.0

    def to_twist(self) -> Twist:
        """Convert to Twist velocity command."""
        return Twist(
            linear=Vector3(self.linear_x, self.linear_y, 0.0),
            angular=Vector3(0.0, 0.0, self.angular_z),
        )

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "command_type": self.command_type,
            "linear_x": self.linear_x,
            "linear_y": self.linear_y,
            "angular_z": self.angular_z,
            "confidence": self.confidence,
        }
        if self.target_position is not None:
            d["target_position"] = [self.target_position.x, self.target_position.y,
                                    self.target_position.z]
        if self.target_label is not None:
            d["target_label"] = self.target_label
        return d

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> NavigationCommand:
        tp = d.get("target_position")
        target_position = Vector3(float(tp[0]), float(tp[1]), float(tp[2])) if tp else None
        return cls(
            command_type=str(d.get("command_type", "velocity")),
            target_position=target_position,
            target_label=d.get("target_label"),
            linear_x=float(d.get("linear_x", 0)),
            linear_y=float(d.get("linear_y", 0)),
            angular_z=float(d.get("angular_z", 0)),
            confidence=float(d.get("confidence", 1.0)),
        )

    def __repr__(self) -> str:
        return (
            f"NavigationCommand(type='{self.command_type}', "
            f"vx={self.linear_x:.2f}, wz={self.angular_z:.2f})"
        )

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> NavigationCommand:
        return json_message_decode(data, cls)
