"""lingtu.runtime.msgs.semantic 鈥?semantic navigation message types.

Defines Detection3D, SceneGraph, GoalResult, NavigationCommand,
SafetyState, MissionStatus 鈥?aligned with LingTu semantic perception/planning port contracts.
"""

from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Any, ClassVar

from runtime.runtime_interface import map_frame_id

from .geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from .numpy_compat import np
from .protocol import json_message_decode, json_message_encode

SEMANTIC_MSG_MAP_FRAME_ID = map_frame_id()

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
        return cls(
            id=det_id, label=label, confidence=conf,
            position=Vector3(px, py, pz),
            bbox_2d=[b0, b1, b2, b3], clip_feature=clip_feature, ts=ts_val,
        )

    def __repr__(self) -> str:
        clip_str = f", clip={self.clip_feature.shape}" if self.clip_feature is not None else ""
        return (
            f"Detection3D(id='{self.id}', label='{self.label}', "
            f"conf={self.confidence:.2f}, pos=[{self.position.x:.2f}, "
            f"{self.position.y:.2f}, {self.position.z:.2f}]{clip_str})"
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
    """Scene graph 鈥?the primary perception output."""

    msg_name: ClassVar[str] = "lingtu_msgs.SceneGraph"
    objects: list[Detection3D] = field(default_factory=list)
    relations: list[Relation] = field(default_factory=list)
    regions: list[Region] = field(default_factory=list)
    ts: float = 0.0
    frame_id: str = SEMANTIC_MSG_MAP_FRAME_ID

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

    # -- serialization ---------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "objects": [o.to_dict() for o in self.objects],
            "relations": [r.to_dict() for r in self.relations],
            "regions": [r.to_dict() for r in self.regions],
            "ts": self.ts,
            "frame_id": self.frame_id,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SceneGraph:
        return cls(
            objects=[Detection3D.from_dict(o) for o in d.get("objects", [])],
            relations=[Relation.from_dict(r) for r in d.get("relations", [])],
            regions=[Region.from_dict(r) for r in d.get("regions", [])],
            ts=float(d.get("ts", 0)),
            frame_id=str(d.get("frame_id", SEMANTIC_MSG_MAP_FRAME_ID)),
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


# ---------------------------------------------------------------------------
# SafetyState
# ---------------------------------------------------------------------------

@dataclass
class SafetyState:
    """Safety state for Gateway telemetry."""

    msg_name: ClassVar[str] = "lingtu_msgs.SafetyState"
    level: str = "safe"  # safe | warn | danger | estop
    issues: list[str] = field(default_factory=list)
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        if self.timestamp == 0.0:
            self.timestamp = time.time()

    @property
    def is_safe(self) -> bool:
        return self.level == "safe"

    def to_dict(self) -> dict[str, Any]:
        return {"level": self.level, "issues": self.issues, "timestamp": self.timestamp}

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SafetyState:
        return cls(
            level=str(d.get("level", "safe")),
            issues=list(d.get("issues", [])),
            timestamp=float(d.get("timestamp", 0)),
        )

    def __repr__(self) -> str:
        return f"SafetyState(level='{self.level}', issues={len(self.issues)})"

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> SafetyState:
        return json_message_decode(data, cls)


# ---------------------------------------------------------------------------
# MissionStatus
# ---------------------------------------------------------------------------

@dataclass
class MissionStatus:
    """Mission status 鈥?corresponds to nav.mission.mission_status Out port."""

    msg_name: ClassVar[str] = "lingtu_msgs.MissionStatus"
    state: str = "idle"  # idle | planning | executing | success | failed
    goal: str | None = None
    progress_pct: float = 0.0
    elapsed_sec: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "goal": self.goal,
            "progress_pct": self.progress_pct,
            "elapsed_sec": self.elapsed_sec,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> MissionStatus:
        return cls(
            state=str(d.get("state", "idle")),
            goal=d.get("goal"),
            progress_pct=float(d.get("progress_pct", 0)),
            elapsed_sec=float(d.get("elapsed_sec", 0)),
        )

    def __repr__(self) -> str:
        return (
            f"MissionStatus(state='{self.state}', goal='{self.goal}', "
            f"progress={self.progress_pct:.1f}%)"
        )

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> MissionStatus:
        return json_message_decode(data, cls)


# ---------------------------------------------------------------------------
# ExecutionEval 鈥?Ring 2 closed-loop assessment
# ---------------------------------------------------------------------------

@dataclass
class ExecutionEval:
    """Execution evaluation 鈥?closed-loop assessment from nav.services.safety.

    Produced by SafetyRing (Ring 2), compares execution vs plan.
    """

    msg_name: ClassVar[str] = "lingtu_msgs.ExecutionEval"
    assessment: str = "IDLE"  # IDLE | ON_TRACK | DRIFTING | STALLED | REGRESSING
    cross_track_error: float = 0.0      # m 鈥?cross-track error
    distance_to_goal: float = 0.0       # m 鈥?distance to goal
    progress_rate: float = 0.0          # m/s 鈥?negative means approaching goal
    heading_error_deg: float = 0.0      # deg 鈥?heading error
    stall_time: float = 0.0             # s 鈥?time without progress
    velocity_efficiency: float = 1.0    # actual/commanded speed ratio
    ts: float = 0.0

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    def to_dict(self) -> dict[str, Any]:
        return {
            "assessment": self.assessment,
            "cross_track_error": self.cross_track_error,
            "distance_to_goal": self.distance_to_goal,
            "progress_rate": self.progress_rate,
            "heading_error_deg": self.heading_error_deg,
            "stall_time": self.stall_time,
            "velocity_efficiency": self.velocity_efficiency,
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> ExecutionEval:
        return cls(
            assessment=str(d.get("assessment", "IDLE")),
            cross_track_error=float(d.get("cross_track_error", 0)),
            distance_to_goal=float(d.get("distance_to_goal", 0)),
            progress_rate=float(d.get("progress_rate", 0)),
            heading_error_deg=float(d.get("heading_error_deg", 0)),
            stall_time=float(d.get("stall_time", 0)),
            velocity_efficiency=float(d.get("velocity_efficiency", 1.0)),
            ts=float(d.get("ts", 0)),
        )

    def __repr__(self) -> str:
        return (
            f"ExecutionEval(assessment='{self.assessment}', "
            f"cte={self.cross_track_error:.2f}m, "
            f"dist={self.distance_to_goal:.1f}m, "
            f"rate={self.progress_rate:.3f}m/s)"
        )

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> ExecutionEval:
        return json_message_decode(data, cls)


# ---------------------------------------------------------------------------
# DialogueState 鈥?Ring 3 user-facing dialogue state
# ---------------------------------------------------------------------------

@dataclass
class DialogueState:
    """Dialogue state 鈥?unified conversation state for Gateway SSE.

    Aggregated by DialogueManager (Ring 3): safety + eval + mission state.
    Aimed at end-user UI (Flutter App / Gateway / logs).
    """

    msg_name: ClassVar[str] = "lingtu_msgs.DialogueState"
    understood: str | None = None       # user intent
    doing: str = "idle"                    # current action description
    progress_pct: float = 0.0              # progress percentage
    distance_m: float | None = None     # distance to goal
    issue: str | None = None            # current issue
    safety: str = "OK"                     # safety level
    safety_text: str = "safe"               # safety description
    eta_sec: float | None = None        # estimated time of arrival
    mission_state: str = "IDLE"            # mission FSM state
    response: str | None = None         # voice response
    position_x: float = 0.0
    position_y: float = 0.0
    ts: float = 0.0

    def __post_init__(self) -> None:
        if self.ts == 0.0:
            self.ts = time.time()

    def to_dict(self) -> dict[str, Any]:
        return {
            "understood": self.understood,
            "doing": self.doing,
            "progress_pct": self.progress_pct,
            "distance_m": self.distance_m,
            "issue": self.issue,
            "safety": self.safety,
            "safety_text": self.safety_text,
            "eta_sec": self.eta_sec,
            "mission_state": self.mission_state,
            "response": self.response,
            "position": {"x": self.position_x, "y": self.position_y},
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> DialogueState:
        pos = d.get("position", {})
        return cls(
            understood=d.get("understood"),
            doing=str(d.get("doing", "idle")),
            progress_pct=float(d.get("progress_pct", 0)),
            distance_m=d.get("distance_m"),
            issue=d.get("issue"),
            safety=str(d.get("safety", "OK")),
            safety_text=str(d.get("safety_text", "safe")),
            eta_sec=d.get("eta_sec"),
            mission_state=str(d.get("mission_state", "IDLE")),
            response=d.get("response"),
            position_x=float(pos.get("x", 0)) if isinstance(pos, dict) else 0.0,
            position_y=float(pos.get("y", 0)) if isinstance(pos, dict) else 0.0,
            ts=float(d.get("ts", 0)),
        )

    def __repr__(self) -> str:
        return (
            f"DialogueState(doing='{self.doing}', safety='{self.safety}', "
            f"mission='{self.mission_state}', progress={self.progress_pct:.1f}%)"
        )

    def encode(self) -> bytes:
        return json_message_encode(self)

    @classmethod
    def decode(cls, data: bytes) -> DialogueState:
        return json_message_decode(data, cls)
