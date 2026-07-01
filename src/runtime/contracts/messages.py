"""Runtime-checkable contracts for high-risk dict messages.

The first phase keeps existing module behavior unchanged: ports can still pass
plain dicts, while tests and composition tooling can validate the fields that
matter for wiring and control-loop safety.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field, is_dataclass
import math
from typing import Any, Callable


CURRENT_SCHEMA_VERSION = 1


class ContractError(ValueError):
    """Raised when a message payload violates a named contract."""


@dataclass(frozen=True)
class ValidationIssue:
    """A single validation error found on a message payload field."""
    path: str
    code: str
    message: str


Validator = Callable[[Mapping[str, Any]], list[ValidationIssue]]


@dataclass(frozen=True)
class MessageContract:
    """Defines the contract for a message type: required fields and a validator function.

    Used for backward-compatibility checks and payload validation
    across module boundaries.
    """
    name: str
    required_fields: tuple[str, ...]
    validate: Validator


@dataclass(frozen=True)
class MessageEnvelope:
    """Portable message wrapper for non-Python runtime adapters.

    Existing Modules may continue to pass plain dict payloads. The envelope is
    the stable boundary shape for schema export, replay, and future Dart/Rust
    adapters.
    """

    payload: Any = field(default_factory=dict)
    topic: str | None = None
    schema: str | None = None
    message_contract: str | None = None
    frame_id: str | None = None
    ts: float | None = None
    format: str | None = None
    type: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)
    encoding: str = "dict"
    schema_version: int = CURRENT_SCHEMA_VERSION

    @classmethod
    def from_payload(
        cls,
        message_type: str,
        payload: Any,
        *,
        topic: str | None = None,
        schema: str | None = None,
        frame_id: str | None = None,
        encoding: str = "dict",
        format: str | None = None,
        schema_version: int | None = None,
        ts: float | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> "MessageEnvelope":
        msg = _to_mapping(payload)
        version = schema_version
        if version is None:
            raw_version = msg.get("schema_version", CURRENT_SCHEMA_VERSION)
            version = int(raw_version)
        return cls(
            topic=topic,
            schema=schema,
            message_contract=str(message_type),
            type=str(message_type),
            payload=msg,
            schema_version=version,
            frame_id=frame_id,
            encoding=encoding,
            format=format,
            ts=ts,
            metadata=dict(metadata or {}),
        )

    @classmethod
    def from_mapping(cls, envelope: Mapping[str, Any]) -> "MessageEnvelope":
        """Create an envelope from an existing LCM/JSON-style mapping."""

        return cls(
            payload=envelope.get("payload", {}),
            topic=_optional_str(envelope.get("topic")),
            schema=_optional_str(envelope.get("schema")),
            message_contract=_optional_str(envelope.get("message_contract")),
            frame_id=_optional_str(envelope.get("frame_id")),
            ts=envelope.get("ts"),
            format=_optional_str(envelope.get("format")),
            type=_optional_str(envelope.get("type")),
            metadata=envelope.get("metadata", {}) if isinstance(envelope.get("metadata"), Mapping) else {},
            encoding=_optional_str(envelope.get("encoding")) or "dict",
            schema_version=int(envelope.get("schema_version", CURRENT_SCHEMA_VERSION)),
        )

    def validate(self) -> list[ValidationIssue]:
        """Validate the wrapped payload against its named contract."""

        contract = self.message_contract or self.type
        if not contract:
            return []
        return validate_message(contract, self.payload)

    def assert_valid(self) -> None:
        """Raise ContractError if the wrapped payload violates its contract."""

        contract = self.message_contract or self.type
        if not contract:
            return
        assert_valid_message(contract, self.payload)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready envelope dict."""

        data: dict[str, Any] = {
            "schema_version": self.schema_version,
            "encoding": self.encoding,
            "payload": dict(self.payload) if isinstance(self.payload, Mapping) else self.payload,
        }
        if self.topic is not None:
            data["topic"] = self.topic
        if self.schema is not None:
            data["schema"] = self.schema
        if self.message_contract is not None:
            data["message_contract"] = self.message_contract
        if self.type is not None:
            data["type"] = self.type
        if self.frame_id is not None:
            data["frame_id"] = self.frame_id
        if self.format is not None:
            data["format"] = self.format
        if self.ts is not None:
            data["ts"] = self.ts
        if self.metadata:
            data["metadata"] = dict(self.metadata)
        return data


MISSION_STATES = frozenset(
    {
        "IDLE",
        "PLANNING",
        "EXECUTING",
        "PAUSED",
        "RECOVERING",
        "SUCCESS",
        "FAILED",
        "STUCK",
        "CANCELLED",
        "PATROLLING",
        "REPLANNING",
    }
)
LOCALIZATION_STATES = frozenset(
    {
        "UNINIT",
        "TRACKING",
        "DEGRADED",
        "LOST",
        "FALLBACK_GNSS_ONLY",
        "RELOCALIZING",
        "OK",
    }
)
DEGENERACY_LEVELS = frozenset({"NONE", "MILD", "SEVERE", "CRITICAL", "UNKNOWN"})
TRAVERSABILITY_CLASSES = frozenset(
    {
        "unknown",
        "safe",
        "normal",
        "passthrough",
        "narrow",
        "corridor",
        "cliff",
        "unsafe_forward",
        "stuck_in_soft",
        "grip_loss",
    }
)


def validate_message(name: str, payload: Any) -> list[ValidationIssue]:
    """Return validation issues for a named message contract."""

    contract = CONTRACTS.get(name)
    if contract is None:
        raise ContractError(f"unknown message contract: {name}")

    msg = _to_mapping(payload)
    issues = _validate_schema_version(msg)
    issues.extend(_validate_required(msg, contract.required_fields))
    issues.extend(contract.validate(msg))
    return issues


def assert_valid_message(name: str, payload: Any) -> None:
    """Raise ContractError if the payload violates a named contract."""

    issues = validate_message(name, payload)
    if issues:
        detail = "; ".join(f"{i.path}: {i.message}" for i in issues)
        raise ContractError(f"{name} contract violation: {detail}")


def _to_mapping(payload: Any) -> Mapping[str, Any]:
    if is_dataclass(payload):
        payload = asdict(payload)
    elif hasattr(payload, "to_dict") and callable(payload.to_dict):
        payload = payload.to_dict()

    if not isinstance(payload, Mapping):
        raise ContractError(f"message payload must be a mapping, got {type(payload).__name__}")
    return payload


def _optional_str(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text or None


def _validate_schema_version(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    if "schema_version" not in msg:
        return []
    if msg["schema_version"] == CURRENT_SCHEMA_VERSION:
        return []
    return [
        ValidationIssue(
            "schema_version",
            "version_mismatch",
            f"expected {CURRENT_SCHEMA_VERSION}, got {msg['schema_version']!r}",
        )
    ]


def _validate_required(msg: Mapping[str, Any], fields: tuple[str, ...]) -> list[ValidationIssue]:
    return [
        ValidationIssue(field, "missing", "required field is missing")
        for field in fields
        if field not in msg
    ]


def _number_issue(
    msg: Mapping[str, Any],
    field: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> ValidationIssue | None:
    if field not in msg:
        return None
    value = msg[field]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return ValidationIssue(field, "invalid_number", "must be a finite number")
    if minimum is not None and value < minimum:
        return ValidationIssue(field, "out_of_range", f"must be >= {minimum}")
    if maximum is not None and value > maximum:
        return ValidationIssue(field, "out_of_range", f"must be <= {maximum}")
    return None


def _enum_issue(
    msg: Mapping[str, Any],
    field: str,
    allowed: frozenset[str],
    *,
    upper: bool = True,
) -> ValidationIssue | None:
    if field not in msg:
        return None
    value = str(msg[field])
    normalized = value.upper() if upper else value
    if normalized in allowed:
        return None
    return ValidationIssue(field, "invalid_value", f"unsupported value {value!r}")


def _shape_like(value: Any) -> bool:
    if hasattr(value, "shape"):
        return True
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return True
    return False


def _sequence_issue(msg: Mapping[str, Any], field: str, *, min_len: int = 1) -> ValidationIssue | None:
    if field not in msg:
        return None
    value = msg[field]
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        return ValidationIssue(field, "invalid_sequence", "must be a sequence")
    if len(value) < min_len:
        return ValidationIssue(field, "invalid_sequence", f"must have at least {min_len} items")
    return None


def _array_len(value: Any) -> int | None:
    shape = getattr(value, "shape", None)
    if shape is not None and len(shape) >= 1:
        return int(shape[0])
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return len(value)
    return None


def _point_array_width(value: Any) -> int | None:
    shape = getattr(value, "shape", None)
    if shape is not None and len(shape) >= 2:
        return int(shape[1])
    if isinstance(value, (str, bytes, bytearray)):
        return None
    try:
        if len(value) == 0:  # type: ignore[arg-type]
            return None
        first = value[0]  # type: ignore[index]
    except (IndexError, TypeError):
        return None
    if isinstance(first, Sequence) and not isinstance(first, (str, bytes, bytearray)):
        return len(first)
    return None


def _validate_localization_status(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    for issue in (
        _enum_issue(msg, "state", LOCALIZATION_STATES),
        _number_issue(msg, "confidence", minimum=0.0, maximum=1.0),
        _enum_issue(msg, "degeneracy", DEGENERACY_LEVELS),
        _number_issue(msg, "ts", minimum=0.0),
    ):
        if issue:
            issues.append(issue)
    return issues


def _validate_mission_status(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    for issue in (
        _enum_issue(msg, "state", MISSION_STATES),
        _number_issue(msg, "replan_count", minimum=0.0),
        _number_issue(msg, "wp_index", minimum=0.0),
        _number_issue(msg, "wp_total", minimum=0.0),
        _number_issue(msg, "speed_scale", minimum=0.0, maximum=1.0),
        _enum_issue(msg, "degeneracy", DEGENERACY_LEVELS),
        _number_issue(msg, "ts", minimum=0.0),
    ):
        if issue:
            issues.append(issue)
    return issues


def _validate_fused_cost(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    if "grid" in msg and not _shape_like(msg["grid"]):
        issues.append(ValidationIssue("grid", "invalid_grid", "must be array-like"))
    for issue in (
        _number_issue(msg, "resolution", minimum=0.0),
        _sequence_issue(msg, "origin", min_len=2),
        _number_issue(msg, "ts", minimum=0.0),
    ):
        if issue:
            issues.append(issue)
    if "resolution" in msg and msg["resolution"] == 0:
        issues.append(ValidationIssue("resolution", "out_of_range", "must be > 0"))
    return issues


def _validate_traversability(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    has_summary = any(k in msg for k in ("status", "traversability_class", "class", "score"))
    grid_key = None
    for key in ("grid", "traversability", "cost", "costmap"):
        if key not in msg:
            continue
        if _shape_like(msg[key]):
            grid_key = key
            break
        if key != "traversability":
            issues.append(ValidationIssue(key, "invalid_grid", "must be array-like"))
    if not has_summary and grid_key is None:
        issues.append(
            ValidationIssue(
                "traversability",
                "missing_summary",
                "requires status, traversability_class, class, score, or grid",
            )
        )
    klass = msg.get("traversability_class", msg.get("class", msg.get("status")))
    if klass is not None:
        issue = _enum_issue({"class": klass}, "class", TRAVERSABILITY_CLASSES, upper=False)
        if issue:
            issues.append(issue)
    score_issue = _number_issue(msg, "score", minimum=0.0, maximum=1.0)
    if score_issue:
        issues.append(score_issue)
    return issues


def _validate_scene_graph(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    for field in ("objects", "relations", "regions"):
        if field in msg:
            issue = _sequence_issue(msg, field, min_len=0)
            if issue:
                issues.append(issue)
    if "frame_id" in msg and not isinstance(msg["frame_id"], str):
        issues.append(ValidationIssue("frame_id", "invalid_value", "must be a string"))
    ts_issue = _number_issue(msg, "ts", minimum=0.0)
    if ts_issue:
        issues.append(ts_issue)
    return issues


def _validate_height_rays(msg: Mapping[str, Any]) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    lengths: dict[str, int] = {}
    for field in ("heights", "points_body", "points_world", "valid_mask"):
        value = msg.get(field)
        if value is None:
            continue
        length = _array_len(value)
        if length is None:
            issues.append(ValidationIssue(field, "invalid_array", "must be array-like"))
        else:
            lengths[field] = length

    expected = lengths.get("heights")
    if expected is not None:
        for field, length in lengths.items():
            if length != expected:
                issues.append(
                    ValidationIssue(
                        field,
                        "shape_mismatch",
                        f"first dimension must match heights length {expected}",
                    )
                )

    for field in ("points_body", "points_world"):
        if field in msg:
            width = _point_array_width(msg[field])
            if width != 3:
                issues.append(
                    ValidationIssue(field, "shape_mismatch", "must have shape [N, 3]")
                )

    if "frame_id" in msg and not isinstance(msg["frame_id"], str):
        issues.append(ValidationIssue("frame_id", "invalid_value", "must be a string"))
    ts_issue = _number_issue(msg, "ts", minimum=0.0)
    if ts_issue:
        issues.append(ts_issue)
    return issues


CONTRACTS: dict[str, MessageContract] = {
    "localization_status": MessageContract(
        "localization_status",
        ("state", "confidence", "degeneracy", "ts"),
        _validate_localization_status,
    ),
    "mission_status": MessageContract(
        "mission_status",
        ("state", "replan_count", "wp_index", "wp_total", "speed_scale", "degeneracy", "ts"),
        _validate_mission_status,
    ),
    "fused_cost": MessageContract(
        "fused_cost",
        ("grid", "resolution", "origin", "ts"),
        _validate_fused_cost,
    ),
    "traversability": MessageContract(
        "traversability",
        (),
        _validate_traversability,
    ),
    "height_rays": MessageContract(
        "height_rays",
        ("heights", "points_body", "points_world", "valid_mask"),
        _validate_height_rays,
    ),
    "scene_graph": MessageContract(
        "scene_graph",
        ("objects", "relations", "regions", "ts", "frame_id"),
        _validate_scene_graph,
    ),
}
