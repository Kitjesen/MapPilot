"""Product-facing field readiness summary.

Aggregates Gateway acceptance, saved-map artifacts, and real-runtime evidence
into one read-only field summary.

The underlying evidence remains Gateway acceptance, saved-map artifact validation,
and real-runtime-evidence.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

FIELD_CHECK_SCHEMA_VERSION = "lingtu.product_field_check.v1"


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _status(ok: Any, *, unchecked: bool = False) -> str:
    if unchecked:
        return "UNCHECKED"
    return "PASS" if ok is True else "FAIL"


def _artifact_status(
    map_gate: Mapping[str, Any] | None,
    name: str,
    *aliases: str,
) -> str:
    if not isinstance(map_gate, Mapping):
        return "UNCHECKED"
    artifacts = _mapping(map_gate.get("artifacts"))
    artifact = {}
    for artifact_name in (name, *aliases):
        artifact = _mapping(artifacts.get(artifact_name))
        if artifact:
            break
    if not artifact:
        return "UNCHECKED"
    return _status(artifact.get("exists") is True and artifact.get("format_ok") is True)


def validate_map(
    map_id: str,
    *,
    require_octomap: bool,
    require_occupancy: bool,
    expected_data_source: str | None,
    expected_source_profile: str | None,
    expected_frame_id: str | None,
) -> dict[str, Any]:
    """Validate one saved map through native mapd plus requested metadata fields."""

    from runtime.endpoints.mapd import MapClient

    try:
        response = MapClient().service(
            "validate_artifacts",
            map_id=map_id,
            require_octomap=require_octomap,
            require_occupancy=require_occupancy,
            expected_data_source=str(expected_data_source or ""),
            expected_source_profile=str(expected_source_profile or ""),
            expected_frame_id=str(expected_frame_id or ""),
        )
    except Exception as exc:
        return {"ok": False, "map_id": map_id, "blockers": [str(exc)]}

    if response.get("success") is not True:
        return {
            "ok": False,
            "map_id": map_id,
            "blockers": [str(response.get("message") or "native map validation failed")],
        }
    gate = dict(response.get("gate") or {})
    gate["map_id"] = map_id
    return gate


def build_product_field_check(
    gateway_acceptance: Mapping[str, Any],
    *,
    map_gate: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the read-only field readiness summary."""

    mode = str(gateway_acceptance.get("mode") or "unknown")
    checks = _mapping(gateway_acceptance.get("checks"))
    gateway = _mapping(checks.get("gateway_contract"))
    observability = _mapping(checks.get("gateway_observability"))
    readiness = _mapping(checks.get("readiness"))
    localization = _mapping(checks.get("localization"))
    navigation = _mapping(checks.get("navigation"))
    evidence = _mapping(checks.get("real_runtime_evidence"))

    map_checked = isinstance(map_gate, Mapping)
    motion_mode = mode in {"simulation", "field"}
    field_mode = mode == "field"
    blockers = [str(item) for item in (gateway_acceptance.get("blockers") or ()) if item]
    if map_checked and map_gate.get("ok") is not True:
        blockers.extend(f"map: {item}" for item in (map_gate.get("blockers") or ()) if item)
    advisories = [str(item) for item in (gateway_acceptance.get("advisories") or ()) if item]
    if not map_checked:
        advisories.append("map artifacts not checked; activate a saved map when one is required")

    return {
        "schema_version": FIELD_CHECK_SCHEMA_VERSION,
        "ok": not blockers,
        "mode": mode,
        "summary": "PASS" if not blockers else "FAIL",
        "map": {
            "active": str(map_gate.get("map_id") or "") if map_checked else "unchecked",
            "artifacts": _status(map_checked and map_gate.get("ok") is True, unchecked=not map_checked),
            "octomap": _artifact_status(map_gate, "octomap"),
            "occupancy": _artifact_status(map_gate, "occupancy_grid", "occupancy"),
        },
        "runtime": {
            "gateway": _status(gateway.get("ok") is True),
            "readiness": _status(readiness.get("ok") is True),
            "localization": _status(localization.get("ok") is True),
            "gateway_observability": _status(observability.get("ok") is True),
        },
        "navigation": {
            "can_send_goal": _status(
                navigation.get("can_send_goal") is True,
                unchecked=not motion_mode,
            ),
            "driver_command": _status(
                evidence.get("cmd_vel_sent_to_hardware") is True,
                unchecked=not field_mode,
            ),
        },
        "evidence": {
            "field_runtime": _status(evidence.get("ok") is True, unchecked=not field_mode),
            "age_s": evidence.get("report_age_s"),
            "mode": mode,
            "runtime_contract": evidence.get("runtime_contract"),
            "map": dict(map_gate) if map_checked else None,
        },
        "blockers": list(dict.fromkeys(blockers)),
        "advisories": list(dict.fromkeys(advisories)),
    }
