"""Text output for the saved-map artifact gate."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def _items(value: Any) -> str:
    if isinstance(value, str):
        return value
    values = tuple(str(item) for item in (value or ()) if item)
    return ",".join(values) if values else "none"


def _bool(value: Any) -> str:
    if value is None:
        return "unknown"
    return str(value).lower()


def _mapping(payload: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = payload.get(key)
    return value if isinstance(value, Mapping) else {}


def format_saved_map_artifact_gate_payload(payload: Mapping[str, Any]) -> str:
    """Render the operator summary used by the saved-map gate."""

    status = "PASS" if payload.get("ok") is True else "FAIL" if payload.get("ok") is False else "UNKNOWN"
    lines = [f"Saved map artifact gate: {status}"]
    if payload.get("checked_frame_id"):
        frame = f"Frame: observed={payload['checked_frame_id']}"
        if payload.get("checked_allowed_frame_ids"):
            frame += f" allowed={_items(payload['checked_allowed_frame_ids'])}"
        lines.append(frame)
    expected = _mapping(payload, "checked_expected")
    if expected:
        lines.append(
            "Expected: "
            f"data_source={expected.get('data_source', 'unknown')} "
            f"frame={expected.get('frame_id', 'unknown')} "
            f"artifacts={_items(expected.get('required_artifacts'))}"
        )
    metadata = _mapping(payload, "metadata")
    validation = _mapping(payload, "metadata_validation")
    if metadata or validation:
        lines.append(
            "Metadata: "
            f"exists={_bool(metadata.get('exists'))} "
            f"ok={_bool(validation.get('ok'))}"
        )
    artifacts = payload.get("artifacts")
    if isinstance(artifacts, Mapping):
        entries = []
        for name, raw in artifacts.items():
            if not isinstance(raw, Mapping):
                continue
            entries.append(
                f"{name} exists={_bool(raw.get('exists'))} "
                f"format_ok={_bool(raw.get('format_ok'))}"
            )
        if entries:
            lines.append("Artifacts: " + " | ".join(entries))
    blockers = payload.get("blockers")
    if blockers:
        lines.append("Blockers: " + " | ".join(str(item) for item in blockers))
    return "\n".join(lines)
