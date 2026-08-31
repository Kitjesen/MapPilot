"""Shared values for real and simulation Product switches."""

from __future__ import annotations

import re
import uuid
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from lingtu.products import ProductName

SWITCH_REPORT_SCHEMA = "lingtu.product_switch.v1"
PROCESS_REPORT_SCHEMA = "lingtu.process_report.v2"
MAP_ACTIVATION_TOKEN_SCHEMA = "lingtu.map_activation.v2"  # noqa: S105
_PRODUCT_SESSION_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,62}\Z")


class ProcessError(RuntimeError):
    """Base error for Product process operations."""


class ProcessFailed(ProcessError):
    """Raised after a process operation fails and rollback has run."""

    def __init__(self, report: ProcessReport):
        super().__init__(report.error or "product launch failed")
        self.report = report


@dataclass
class ProcessReport:
    """Machine-readable evidence for one process operation."""

    product: str
    env: str
    action: str
    dry_run: bool = False
    ok: bool = False
    status: str = "pending"
    planned: list[str] = field(default_factory=list)
    stopped: list[str] = field(default_factory=list)
    started: list[str] = field(default_factory=list)
    preserved: list[str] = field(default_factory=list)
    identities: dict[str, Mapping[str, Any]] = field(default_factory=dict)
    ready: dict[str, Mapping[str, Any]] = field(default_factory=dict)
    stop_evidence: dict[str, Mapping[str, Any]] = field(default_factory=dict)
    rolled_back: list[str] = field(default_factory=list)
    rollback_errors: list[str] = field(default_factory=list)
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the stable JSON contract used by CLI and Gateway."""

        return {
            "schema_version": PROCESS_REPORT_SCHEMA,
            "product": self.product,
            "env": self.env,
            "action": self.action,
            "dry_run": self.dry_run,
            "ok": self.ok,
            "status": self.status,
            "planned": list(self.planned),
            "stopped": list(self.stopped),
            "started": list(self.started),
            "preserved": list(self.preserved),
            "ready": dict(self.ready),
            "stop_evidence": dict(self.stop_evidence),
            "rolled_back": list(self.rolled_back),
            "rollback_errors": list(self.rollback_errors),
            "error": self.error,
        }


def is_product_session_id(value: object) -> bool:
    """Return whether value is a valid Product run identifier."""

    return isinstance(value, str) and _PRODUCT_SESSION_ID.fullmatch(value) is not None


def new_product_session_id() -> str:
    """Create a readable Product run identifier."""

    return f"product-{uuid.uuid4().hex}"


@dataclass(frozen=True)
class SwitchRequest:
    """Operator intent for one Product transition."""

    target_product: ProductName
    map_name: str | None = None
    relocalize: bool = True
    initial_pose: tuple[float, float, float] | None = None
    local_planner: str | None = None
    parameter_overrides: Mapping[str, Any] = field(default_factory=dict)

    @property
    def product_variant(self) -> str | None:
        """Select the internal Explore route without exposing another Product."""

        if self.target_product != "explore":
            return None
        return "map" if str(self.map_name or "").strip() else "live"


@dataclass
class SwitchReport:
    """Machine-readable result and phase journal for a Product switch."""

    current_product: ProductName | None
    target_product: ProductName
    env: str
    product_variant: str | None = None
    local_planner: str | None = None
    dry_run: bool = False
    ok: bool = False
    status: str = "preflight"
    run_plan_path: str | None = None
    product_session_id: str | None = None
    phases: list[str] = field(default_factory=list)
    cleanup: list[str] = field(default_factory=list)
    error: str | None = None
    readiness: Mapping[str, Mapping[str, Any]] | None = field(
        default=None,
        repr=False,
    )

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready switch evidence."""

        return {
            "schema_version": SWITCH_REPORT_SCHEMA,
            "ok": self.ok,
            "status": self.status,
            "dry_run": self.dry_run,
            "current_product": self.current_product,
            "target_product": self.target_product,
            "product_variant": self.product_variant,
            "local_planner": self.local_planner,
            "env": self.env,
            "run_plan_path": self.run_plan_path,
            "product_session_id": self.product_session_id,
            "phases": list(self.phases),
            "cleanup": list(self.cleanup),
            "error": self.error,
        }


@dataclass(frozen=True)
class MapArtifactIdentity:
    """Stable identity fields exposed by one native map artifact record."""

    artifact_type: str
    uri: str | None = None


@dataclass(frozen=True)
class MapIdentity:
    """Exact saved-map identity returned by native mapd."""

    map_id: str
    content_epoch: int
    frame_id: str
    artifacts: tuple[MapArtifactIdentity, ...] = ()


def pointcloud_artifact(identity: MapIdentity) -> MapArtifactIdentity:
    """Return the unique point-cloud artifact for one map identity."""

    matches = [
        artifact
        for artifact in identity.artifacts
        if artifact.artifact_type.strip().upper()
        in {"POINTCLOUD", "POINT_CLOUD", "PCD"}
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"MapService map {identity.map_id!r} must have exactly one "
            "point-cloud artifact"
        )
    return matches[0]


def map_identity_environment(identity: MapIdentity) -> dict[str, str]:
    """Return the session environment bound to one exact map identity."""

    pointcloud_artifact(identity)
    return {
        "LINGTU_MAP_ID": identity.map_id,
        "LINGTU_MAP_CONTENT_EPOCH": str(identity.content_epoch),
        "LINGTU_MAP_FRAME": identity.frame_id,
    }


def map_identity_as_record(identity: MapIdentity | None) -> dict[str, Any] | None:
    """Return the stable current-run representation of a map identity."""

    if identity is None:
        return None
    return {
        "map_id": identity.map_id,
        "content_epoch": identity.content_epoch,
        "frame_id": identity.frame_id,
        "artifacts": [
            {
                "artifact_type": artifact.artifact_type,
                "uri": artifact.uri,
            }
            for artifact in identity.artifacts
        ],
    }


def optional_map_identity_from_native(
    value: Any,
    *,
    field_name: str,
) -> MapIdentity | None:
    """Parse one native optional-map record."""

    if not isinstance(value, Mapping):
        raise RuntimeError(f"native {field_name} identity is missing")
    present = required_bool(value.get("present"), f"native {field_name} present")
    return map_identity_from_native(value, field_name=field_name) if present else None


def map_identity_from_native(value: Any, *, field_name: str) -> MapIdentity:
    """Parse one required native map record."""

    if not isinstance(value, Mapping) or value.get("present") is not True:
        raise RuntimeError(f"native {field_name} identity is not present")
    raw_artifacts = value.get("artifacts")
    if not isinstance(raw_artifacts, list) or not raw_artifacts:
        raise RuntimeError(f"native {field_name} artifacts must be a non-empty list")
    artifacts: list[MapArtifactIdentity] = []
    for index, artifact in enumerate(raw_artifacts):
        if not isinstance(artifact, Mapping):
            raise RuntimeError(
                f"native {field_name} artifact {index} must be an object"
            )
        artifacts.append(
            MapArtifactIdentity(
                artifact_type=required_text(
                    artifact.get("type"),
                    f"native {field_name} artifact {index} type",
                ),
                uri=required_text(
                    artifact.get("uri"),
                    f"native {field_name} artifact {index} uri",
                ),
            )
        )
    return MapIdentity(
        map_id=required_text(value.get("map_id"), f"native {field_name} map_id"),
        content_epoch=required_positive_int(
            value.get("content_epoch"),
            f"native {field_name} content_epoch",
        ),
        frame_id=required_text(
            value.get("frame_id"),
            f"native {field_name} frame_id",
        ),
        artifacts=tuple(artifacts),
    )


def map_identity_from_record(value: Any, *, field_name: str) -> MapIdentity:
    """Parse and validate one persisted map identity record."""

    if not isinstance(value, Mapping):
        raise RuntimeError(f"{field_name} is missing")
    raw_artifacts = value.get("artifacts")
    if not isinstance(raw_artifacts, list) or not raw_artifacts:
        raise RuntimeError(f"{field_name} artifacts are missing")
    artifacts: list[MapArtifactIdentity] = []
    for index, raw_artifact in enumerate(raw_artifacts):
        if not isinstance(raw_artifact, Mapping):
            raise RuntimeError(f"{field_name} artifact {index} must be an object")
        artifacts.append(
            MapArtifactIdentity(
                artifact_type=required_text(
                    raw_artifact.get("artifact_type"),
                    f"{field_name} artifact {index} type",
                ),
                uri=required_text(
                    raw_artifact.get("uri"),
                    f"{field_name} artifact {index} URI",
                ),
            )
        )
    identity = MapIdentity(
        map_id=required_text(value.get("map_id"), f"{field_name} map ID"),
        content_epoch=required_positive_int(
            value.get("content_epoch"),
            f"{field_name} content epoch",
        ),
        frame_id=required_text(value.get("frame_id"), f"{field_name} frame ID"),
        artifacts=tuple(artifacts),
    )
    # Every saved-map Product localizes against the point cloud.  The selected
    # planner validates its own artifact when the runtime environment is built.
    pointcloud_artifact(identity)
    return identity


def octomap_artifact(identity: MapIdentity) -> MapArtifactIdentity:
    """Return the unique supported 3D planning artifact."""

    matches = _map_artifacts(
        identity,
        accepted_types={"OCTOMAP", "OCTOMAP3D", "OCTOMAP_3D"},
    )
    if len(matches) != 1:
        raise RuntimeError(
            f"MapService map {identity.map_id!r} must have exactly one 3D planning artifact"
        )
    artifact = matches[0]
    uri = required_text(artifact.uri, "MapService 3D planning artifact URI")
    if not uri.lower().endswith((".ot", ".bt")):
        raise RuntimeError(
            f"MapService map {identity.map_id!r} has an unsupported 3D planning artifact"
        )
    return artifact


def occupancy_artifact(identity: MapIdentity) -> MapArtifactIdentity:
    """Return the unique 2D planning artifact."""

    matches = _map_artifacts(
        identity,
        accepted_types={"OCCUPANCY", "OCCUPANCY2D", "OCCUPANCY_2D"},
    )
    if len(matches) != 1:
        raise RuntimeError(
            f"MapService map {identity.map_id!r} must have exactly one 2D planning artifact"
        )
    return matches[0]


def _map_artifacts(
    identity: MapIdentity,
    *,
    accepted_types: set[str],
) -> list[MapArtifactIdentity]:
    return [
        artifact
        for artifact in identity.artifacts
        if artifact.artifact_type.strip().upper() in accepted_types
    ]


def required_positive_int(value: Any, field_name: str) -> int:
    """Return a required positive integer contract field."""

    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise RuntimeError(f"{field_name} must be a positive integer")
    return value


def required_bool(value: Any, field_name: str) -> bool:
    """Return a required boolean contract field."""

    if not isinstance(value, bool):
        raise RuntimeError(f"{field_name} must be boolean")
    return value


def required_text(value: Any, field_name: str) -> str:
    """Return a required non-empty text contract field."""

    text = str(value).strip() if value is not None else ""
    if not text:
        raise RuntimeError(f"{field_name} is required")
    return text


class SwitchFailed(RuntimeError):
    """Raised after a switch is rejected or forced into a stopped state."""

    def __init__(self, report: SwitchReport):
        super().__init__(report.error or "Product switch failed")
        self.report = report


__all__ = [
    "MAP_ACTIVATION_TOKEN_SCHEMA",
    "PROCESS_REPORT_SCHEMA",
    "SWITCH_REPORT_SCHEMA",
    "MapArtifactIdentity",
    "MapIdentity",
    "ProcessError",
    "ProcessFailed",
    "ProcessReport",
    "SwitchFailed",
    "SwitchReport",
    "SwitchRequest",
    "is_product_session_id",
    "map_identity_as_record",
    "map_identity_environment",
    "map_identity_from_native",
    "map_identity_from_record",
    "new_product_session_id",
    "occupancy_artifact",
    "octomap_artifact",
    "optional_map_identity_from_native",
    "pointcloud_artifact",
    "required_bool",
    "required_positive_int",
    "required_text",
]
