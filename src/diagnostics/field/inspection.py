"""Inspection-point acceptance pack built on Gateway-only product gates."""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from typing import Any

INSPECTION_ACCEPTANCE_SCHEMA_VERSION = "lingtu.inspection_acceptance.v1"
def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _as_list(value: Any) -> list[Any]:
    return list(value) if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) else []


def _status(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def _http_ok(snapshot: Mapping[str, Any]) -> bool:
    if snapshot.get("_fetch_error"):
        return False
    status = snapshot.get("_http_status")
    try:
        return status is None or int(status) < 400
    except (TypeError, ValueError):
        return False


def _target_name(target: Mapping[str, Any]) -> str:
    return str(target.get("location_name") or target.get("name") or target.get("label") or "coordinate")


def _target_from_location(location: Mapping[str, Any]) -> dict[str, Any]:
    return _saved_location_target(str(location.get("name") or ""))


def _saved_location_target(name: str, *, missing: bool = False) -> dict[str, Any]:
    target = {
        "location_name": name,
        "label": name,
        "source": "saved_location",
        "target_type": "saved_location",
        "preview": True,
    }
    if missing:
        target["_missing_from_locations"] = True
    return target


def _invalid_target(raw: Any) -> dict[str, Any]:
    label = "invalid_inspection_target"
    if isinstance(raw, Mapping):
        label = str(raw.get("location_name") or raw.get("name") or raw.get("label") or label)
    elif raw is not None:
        label = str(raw)
    return {
        "label": label,
        "source": "operator_payload",
        "target_type": "invalid",
        "preview": True,
        "_invalid_payload": True,
    }


def _normalize_target(raw: Any) -> dict[str, Any] | None:
    if isinstance(raw, str):
        name = raw.strip()
        if not name:
            return None
        return {
            "location_name": name,
            "label": name,
            "source": "saved_location",
            "target_type": "saved_location",
            "preview": True,
        }
    if not isinstance(raw, Mapping):
        return None
    target = dict(raw)
    name = target.get("location_name") or target.get("name")
    if not name:
        return None
    location_name = str(name).strip()
    if not location_name:
        return None
    return {
        "location_name": location_name,
        "label": str(target.get("label") or location_name),
        "source": "saved_location",
        "target_type": "saved_location",
        "preview": True,
    }


def _targets_from_locations(
    locations_payload: Mapping[str, Any],
    *,
    requested_names: Sequence[str] = (),
    tag: str | None = None,
) -> list[dict[str, Any]]:
    locations = [_mapping(item) for item in locations_payload.get("locations") or [] if isinstance(item, Mapping)]
    requested = {name.strip() for name in requested_names if name.strip()}
    selected: list[dict[str, Any]] = []
    for location in locations:
        name = str(location.get("name") or "")
        tags = {str(item) for item in location.get("tags") or []}
        if requested and name not in requested:
            continue
        if tag and tag not in tags:
            continue
        if name:
            selected.append(_target_from_location(location))
    missing = sorted(requested - {item["location_name"] for item in selected})
    selected.extend(
        _saved_location_target(name, missing=True)
        for name in missing
    )
    return selected


def inspection_targets_from_payload(
    locations_payload: Mapping[str, Any],
    *,
    points: Sequence[Any] = (),
    tag: str | None = None,
) -> list[dict[str, Any]]:
    """Resolve operator inspection targets from explicit points or saved locations."""

    explicit_targets: list[dict[str, Any]] = []
    invalid_targets: list[dict[str, Any]] = []
    for item in points:
        target = _normalize_target(item)
        if target:
            explicit_targets.append(target)
        else:
            invalid_targets.append(_invalid_target(item))
    if invalid_targets:
        return [*explicit_targets, *invalid_targets]
    if not explicit_targets:
        return _targets_from_locations(locations_payload, tag=tag)

    requested_names = [
        str(target.get("location_name") or "")
        for target in explicit_targets
        if target.get("location_name") and "x" not in target and "y" not in target
    ]
    if not requested_names and not tag:
        return explicit_targets

    known_targets = _targets_from_locations(
        locations_payload,
        requested_names=requested_names,
        tag=tag,
    )
    known_names = {
        str(target.get("location_name") or "")
        for target in known_targets
        if target.get("location_name") and not target.get("_missing_from_locations")
    }
    missing_by_name = {
        str(target.get("location_name") or ""): target
        for target in known_targets
        if target.get("_missing_from_locations")
    }

    targets: list[dict[str, Any]] = []
    for target in explicit_targets:
        location_name = str(target.get("location_name") or "")
        if not location_name:
            targets.append(target)
        elif location_name in known_names:
            targets.append(target)
        else:
            targets.append(
                missing_by_name.get(location_name)
                or _saved_location_target(location_name, missing=True)
            )
    return targets


def goal_candidate_body_for_target(target: Mapping[str, Any]) -> dict[str, Any]:
    """Return the Gateway goal_candidate request body for one inspection target."""

    return {
        **dict(target),
        "preview": True,
    }


def _candidate_result(target: Mapping[str, Any], candidate: Mapping[str, Any]) -> dict[str, Any]:
    preview = _mapping(candidate.get("preview"))
    target_payload = _mapping(candidate.get("target"))
    reasons = [str(item) for item in (candidate.get("reasons") or preview.get("reasons") or ()) if item]
    fetch_ok = _http_ok(candidate)
    feasible = preview.get("feasible") is True
    path_count = preview.get("count")
    try:
        count_ok = int(path_count or 0) > 0
    except (TypeError, ValueError):
        count_ok = False
    ok = (
        fetch_ok
        and candidate.get("ok") is True
        and bool(target_payload)
        and preview.get("ok") is not False
        and feasible
        and count_ok
    )
    error = candidate.get("_fetch_error") or candidate.get("error") or preview.get("error")
    if not reasons and not ok:
        if not fetch_ok:
            reasons.append(str(error or "goal_candidate_endpoint_unavailable"))
        elif not feasible:
            reasons.append("plan_preview_not_feasible")
        elif not count_ok:
            reasons.append("plan_preview_empty_path")
        else:
            reasons.append("goal_candidate_not_ok")
    return {
        "name": _target_name(target),
        "status": _status(ok),
        "ok": ok,
        "target_type": target.get("target_type") or target_payload.get("target_type"),
        "source": target.get("source") or target_payload.get("source"),
        "location_name": target.get("location_name") or target_payload.get("location_name"),
        "preview_feasible": feasible,
        "preview_count": path_count,
        "planner": preview.get("planner"),
        "distance_m": preview.get("distance_m"),
        "reasons": reasons,
        "error": error,
    }


def build_inspection_acceptance(
    *,
    field_check: Mapping[str, Any],
    targets: Sequence[Mapping[str, Any]],
    candidates: Sequence[Mapping[str, Any]],
    locations: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a product-facing inspection acceptance payload."""

    target_results = [
        _candidate_result(target, candidate) for target, candidate in zip(targets, candidates, strict=False)
    ]
    blockers = [f"field: {item}" for item in (field_check.get("blockers") or ()) if item]
    if not targets:
        blockers.append("inspection targets are empty")
    if len(candidates) < len(targets):
        blockers.append("not all inspection targets were previewed")
    for result in target_results:
        if result["ok"] is not True:
            reason = "; ".join(result["reasons"]) or "target preview failed"
            blockers.append(f"target {result['name']} failed: {reason}")
    field_ok = field_check.get("ok") is True
    points_ok = (
        bool(targets) and len(target_results) == len(targets) and all(item["ok"] is True for item in target_results)
    )
    if not field_ok:
        summary = "BLOCKED"
    elif points_ok and not blockers:
        summary = "PASS"
    else:
        summary = "FAIL"

    return {
        "schema_version": INSPECTION_ACCEPTANCE_SCHEMA_VERSION,
        "ok": summary == "PASS",
        "summary": summary,
        "mode": field_check.get("mode") or "simulation",
        "field_ready": field_ok,
        "field_summary": field_check.get("summary") or ("PASS" if field_ok else "FAIL"),
        "target_count": len(targets),
        "pass_count": sum(1 for item in target_results if item["ok"] is True),
        "fail_count": sum(1 for item in target_results if item["ok"] is not True),
        "locations_count": _mapping(locations).get("count"),
        "targets": target_results,
        "blockers": list(dict.fromkeys(str(item) for item in blockers if item)),
        "advisories": [str(item) for item in (field_check.get("advisories") or ()) if item],
        "evidence": {
            "field_check": field_check,
        },
        "ts": time.time(),
    }
