"""Session authoring services for the isolated local SimStudio application.

This module owns the authoring boundary between opaque Studio records and the
canonical simulation catalog.  It deliberately has no HTTP, process, DDS, UE,
or field Product dependencies.
"""

from __future__ import annotations

import copy
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from sim.catalog.composer import SessionComposer, SessionIntent
from sim.catalog.management import SimCatalog
from sim.catalog.resolver import CatalogError

from .models import (
    BundleRecord,
    RevisionConflict,
    SessionDraftRecord,
    StoreValidationError,
)
from .store import StudioStore

_SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
_FORBIDDEN_KEY_NAMES = frozenset(
    {
        "absolute_path",
        "filesystem_path",
        "output",
        "output_dir",
        "output_path",
        "source_path",
        "working_dir",
    }
)


def _record_dict(record: Any) -> dict[str, Any]:
    """Return the detached public representation of one Studio record."""

    return record.to_dict()


def _safe_identifier(value: Any, *, context: str) -> str:
    if not isinstance(value, str) or _SAFE_IDENTIFIER.fullmatch(value) is None:
        raise StoreValidationError(f"{context} must be a safe identifier")
    return value


def _reject_unmanaged_keys(value: Any, *, context: str = "intent") -> None:
    """Reject filesystem/output controls even in nested future extensions."""

    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise StoreValidationError(f"{context} contains a non-string field name")
            lowered = key.lower()
            if lowered in _FORBIDDEN_KEY_NAMES or lowered.endswith("_path") or "output" in lowered:
                raise StoreValidationError(f"{context}.{key} is not a managed SessionIntent field")
            _reject_unmanaged_keys(child, context=f"{context}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_unmanaged_keys(child, context=f"{context}[{index}]")


class SessionAuthoringService:
    """Create and compose versioned session drafts inside one Studio store.

    The service accepts only JSON-like authoring documents. Bundle names use
    the draft ID and revision and stay below the service-owned bundles root;
    callers never provide an output path.
    """

    def __init__(
        self,
        store: StudioStore,
        composer: SessionComposer,
        catalog: SimCatalog | None = None,
        *,
        bundles_root: Path | None = None,
    ) -> None:
        if not isinstance(store, StudioStore):
            raise TypeError("store must be a StudioStore")
        if not hasattr(composer, "resolver"):
            raise TypeError("composer must expose the canonical CatalogResolver")
        if catalog is not None and catalog.resolver is not composer.resolver:
            raise ValueError("SimCatalog and SessionComposer must share one CatalogResolver")

        self.store = store
        self.catalog = catalog or SimCatalog(composer.resolver)
        self.bundles_root = self._prepare_bundles_root(bundles_root or store.root / "bundles", store)

        # Rebind the existing resolver to the service-owned artifact root.  No
        # package-resolution logic is duplicated here.
        self.composer = SessionComposer(composer.resolver, artifact_root=self.bundles_root)

    @staticmethod
    def _prepare_bundles_root(root: Path, store: StudioStore) -> Path:
        candidate = Path(root)
        if candidate.exists() and candidate.is_symlink():
            raise StoreValidationError("SimStudio bundles root must not be a symbolic link")
        resolved = candidate.resolve()
        try:
            resolved.relative_to(store.root.resolve())
        except ValueError as exc:
            raise StoreValidationError("SimStudio bundles root must be below the Studio store root") from exc
        resolved.mkdir(parents=True, exist_ok=True)
        if resolved.is_symlink():
            raise StoreValidationError("SimStudio bundles root must not be a symbolic link")
        return resolved

    def _validate_json_intent(self, intent: Mapping[str, Any]) -> SessionIntent:
        if not isinstance(intent, Mapping):
            raise StoreValidationError("session intent must be a JSON object")
        _reject_unmanaged_keys(intent)
        normalized = SessionIntent.load(copy.deepcopy(dict(intent)))
        session = normalized.document["session"]
        _safe_identifier(session.get("session_id"), context="SessionIntent.session.session_id")
        robots = session.get("robots")
        if not isinstance(robots, list):
            raise StoreValidationError("SessionIntent.session.robots must be an array")
        for index, robot in enumerate(robots):
            if not isinstance(robot, Mapping):
                raise StoreValidationError(f"SessionIntent.session.robots[{index}] must be an object")
            _safe_identifier(robot.get("instance_id"), context=f"SessionIntent.session.robots[{index}].instance_id")

        # Validate exact references against the same catalog used by the
        # Composer.  Resolver compilation remains the authoritative complete
        # check, while drafts fail early on typos and missing package IDs.
        resolver = self.catalog.resolver
        resolver.find_package(session["world"], kind="world")
        if "scenario" in session:
            resolver.find_package(session["scenario"], kind="scenario")
        for robot in robots:
            robot_record = resolver.find_package(robot["package"], kind="robot")
            defaults = robot_record.data.get("defaults", {})
            if not isinstance(defaults, Mapping):
                raise CatalogError(f"robot package {robot['package']!r} has invalid defaults")
            controller_ref = robot.get("controller", defaults.get("controller"))
            if controller_ref is not None:
                resolver.find_package(controller_ref, kind="controller")
            rig_ref = robot.get("sensor_rig", defaults.get("sensor_rig"))
            if rig_ref is not None:
                resolver.find_package(rig_ref, kind="sensor_rig")
        return normalized

    def _normalize_intent(self, intent: Mapping[str, Any]) -> dict[str, Any]:
        validated = self._validate_json_intent(intent)
        return copy.deepcopy(dict(validated.document))

    @staticmethod
    def _draft_payload(document: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "schema": "lingtu.sim.studio.session-draft-payload.v1",
            "intent": copy.deepcopy(dict(document)),
        }

    def create_session_draft(
        self,
        intent: Mapping[str, Any],
        *,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Create a validated draft and return its opaque record envelope."""

        document = self._normalize_intent(intent)
        record = self.store.create_session_draft(
            self._draft_payload(document),
            idempotency_key=idempotency_key,
        )
        return _record_dict(record)

    def get_session_draft(self, draft_id: str) -> dict[str, Any]:
        """Retrieve one draft by opaque ID."""

        return _record_dict(self.store.get_session_draft(draft_id))

    def list_session_drafts(self) -> list[dict[str, Any]]:
        """List drafts in the store's deterministic order."""

        return [_record_dict(record) for record in self.store.list_session_drafts()]

    def update_session_draft(
        self,
        draft_id: str,
        *,
        revision: int,
        intent: Mapping[str, Any],
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """CAS-update one draft; stale revisions are rejected by StudioStore."""

        document = self._normalize_intent(intent)
        record = self.store.update_session_draft(
            draft_id,
            expected_revision=revision,
            payload=self._draft_payload(document),
            idempotency_key=idempotency_key,
        )
        return _record_dict(record)

    def _existing_bundle_locked(self, draft: SessionDraftRecord) -> BundleRecord | None:
        """Find a bundle while the caller owns the Store operation lock."""

        for record in self.store._list("bundle"):
            payload = record.payload
            if payload.get("draft_id") == draft.id and payload.get("draft_revision") == draft.revision:
                return record
        return None

    def compose_session(
        self,
        draft_id: str,
        *,
        revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Compile one draft into a deterministic, service-owned bundle."""

        # The Store lock is cross-process, not merely an in-memory mutex.  The
        # draft read, existing-record check, deterministic compose, and record
        # creation therefore form one serialization point for each Store root.
        # In particular, do not replace this with separate public Store calls:
        # that would reintroduce the check-then-compose race.
        with self.store._operation_lock():
            draft = self.store._load_record("session_draft", draft_id)
            if revision is not None and revision != draft.revision:
                raise RevisionConflict(
                    f"stale session draft revision for {draft_id}: expected {revision}, current {draft.revision}",
                    expected=revision,
                    actual=draft.revision,
                )
            existing = self._existing_bundle_locked(draft)
            if existing is not None:
                return _record_dict(existing)

            intent = SessionIntent.load(draft.payload["intent"])
            output_dir = Path(f"bundle-{draft.id}-{draft.revision}")
            composed = self.composer.compose(intent, output_dir=output_dir)
            bundle_dir = Path(composed.bundle_dir).resolve()
            try:
                relative_bundle = bundle_dir.relative_to(self.store.root.resolve()).as_posix()
                bundle_dir.relative_to(self.bundles_root)
            except ValueError as exc:
                raise StoreValidationError("composer returned a bundle outside the service-owned root") from exc

            result = composed.to_dict()
            artifacts: list[str] = []
            for artifact in result.get("artifacts", []):
                artifact_path = Path(artifact).resolve()
                try:
                    artifacts.append(artifact_path.relative_to(self.store.root.resolve()).as_posix())
                    artifact_path.relative_to(bundle_dir)
                except ValueError as exc:
                    raise StoreValidationError("composer returned an artifact outside the bundle root") from exc

            payload = {
                "schema": "lingtu.sim.studio.bundle-payload.v1",
                "draft_id": draft.id,
                "draft_revision": draft.revision,
                "session_id": result["session_id"],
                "bundle_path": relative_bundle,
                "artifacts": artifacts,
            }
            record = self.store._create(
                "bundle",
                payload,
                status="ready",
                idempotency_key=idempotency_key,
            )
            return _record_dict(record)

    def get_bundle(self, bundle_id: str) -> dict[str, Any]:
        """Retrieve bundle metadata by opaque ID."""

        return _record_dict(self.store.get_bundle(bundle_id))

    def list_bundles(self) -> list[dict[str, Any]]:
        """List compiled bundle metadata."""

        return [_record_dict(record) for record in self.store.list_bundles()]


__all__ = ["SessionAuthoringService"]
