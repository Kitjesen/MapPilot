"""Inspection-point acceptance pack built on Gateway-only product gates."""

from __future__ import annotations

import json
import time
from collections.abc import Mapping, Sequence
from typing import Any
from urllib.error import URLError
from urllib.parse import urljoin
from urllib.request import Request, urlopen

from diagnostics.field.field_check import collect_product_field_check
from diagnostics.field.gateway_acceptance import DEFAULT_GATEWAY_URL

INSPECTION_ACCEPTANCE_SCHEMA_VERSION = "lingtu.inspection_acceptance.v1"
DEFAULT_CLIENT_ID = "lingtu-inspection-acceptance"


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
    return {
        "location_name": str(location.get("name") or ""),
        "label": str(location.get("name") or ""),
        "source": "saved_location",
        "target_type": "saved_location",
        "preview": True,
    }


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
        {
            "location_name": name,
            "label": name,
            "source": "saved_location",
            "target_type": "saved_location",
            "preview": True,
            "_missing_from_locations": True,
        }
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
                or {
                    "location_name": location_name,
                    "label": location_name,
                    "source": "saved_location",
                    "target_type": "saved_location",
                    "preview": True,
                    "_missing_from_locations": True,
                }
            )
    return targets


def goal_candidate_body_for_target(target: Mapping[str, Any]) -> dict[str, Any]:
    """Return the Gateway goal_candidate request body for one inspection target."""

    return {
        **dict(target),
        "preview": True,
        "client_id": DEFAULT_CLIENT_ID,
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
        "planner": preview.get("selected_planner") or preview.get("planner"),
        "distance_m": preview.get("distance_m"),
        "non_motion": True,
        "command_published": False,
        "reasons": reasons,
        "error": error,
    }


def _published_counters(value: Any) -> dict[str, int | None]:
    raw = _mapping(value)
    if not raw:
        return {}
    counters: dict[str, int | None] = {}
    for name in ("goal_pose", "cmd_vel", "stop_cmd"):
        if name not in raw:
            counters[name] = None
            continue
        try:
            counters[name] = int(raw.get(name))
        except (TypeError, ValueError):
            counters[name] = None
    return counters


def _motion_safety(
    field_check: Mapping[str, Any],
    target_results: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    navigation = _mapping(field_check.get("navigation"))
    published = _published_counters(navigation.get("route_preview_published"))
    target_command_published = any(item.get("command_published") is True for item in target_results)
    published_command = any(count is not None and count > 0 for count in published.values())
    missing_counter = any(count is None for count in published.values())
    route_non_motion = navigation.get("route_preview_non_motion")
    non_motion = route_non_motion is not False and all(item.get("non_motion") is True for item in target_results)
    command_published = target_command_published or published_command
    ok = non_motion and not command_published and not missing_counter
    return {
        "status": _status(ok),
        "ok": ok,
        "non_motion": non_motion,
        "command_published": command_published,
        "published": published,
    }


def build_inspection_acceptance(
    *,
    field_check: Mapping[str, Any],
    targets: Sequence[Mapping[str, Any]],
    candidates: Sequence[Mapping[str, Any]],
    locations: Mapping[str, Any] | None = None,
    gateway_url: str = DEFAULT_GATEWAY_URL,
) -> dict[str, Any]:
    """Build a product-facing inspection acceptance payload."""

    target_results = [
        _candidate_result(target, candidate) for target, candidate in zip(targets, candidates, strict=False)
    ]
    frontier_preview = _mapping(field_check.get("frontier_preview"))
    runtime_switch = _mapping(field_check.get("runtime_switch"))
    motion_safety = _motion_safety(field_check, target_results)
    blockers = [f"field: {item}" for item in (field_check.get("blockers") or ()) if item]
    if not targets:
        blockers.append("inspection targets are empty")
    if len(candidates) < len(targets):
        blockers.append("not all inspection targets were previewed")
    for result in target_results:
        if result["ok"] is not True:
            reason = "; ".join(result["reasons"]) or "target preview failed"
            blockers.append(f"target {result['name']} failed: {reason}")
    if motion_safety["ok"] is not True:
        blockers.append("motion safety failed")

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
        "gateway_url": gateway_url,
        "mode": field_check.get("mode") or "simulation",
        "field_ready": field_ok,
        "field_summary": field_check.get("summary") or ("PASS" if field_ok else "FAIL"),
        "target_count": len(targets),
        "pass_count": sum(1 for item in target_results if item["ok"] is True),
        "fail_count": sum(1 for item in target_results if item["ok"] is not True),
        "locations_count": _mapping(locations).get("count"),
        "frontier_preview": dict(frontier_preview),
        "runtime_switch": dict(runtime_switch),
        "motion_safety": motion_safety,
        "targets": target_results,
        "blockers": list(dict.fromkeys(str(item) for item in blockers if item)),
        "advisories": [str(item) for item in (field_check.get("advisories") or ()) if item],
        "evidence": {
            "field_check": field_check,
            "frontier_preview": dict(frontier_preview),
            "runtime_switch": dict(runtime_switch),
            "locations": dict(locations or {}),
        },
        "commands": {
            "field_check": (
                "python lingtu.py field-check "
                "--gateway-url http://<server>:5050 --acceptance-mode simulation "
                "--require-octomap"
            ),
            "inspection_check": (
                "python lingtu.py inspection-check "
                "--gateway-url http://<robot>:5050 --point <location> "
                "--require-octomap"
            ),
        },
        "ts": time.time(),
    }


def _request_json(
    base_url: str,
    path: str,
    *,
    timeout_sec: float,
    body: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    url = urljoin(base_url.rstrip("/") + "/", path.lstrip("/"))
    data = None
    headers = {"Accept": "application/json"}
    method = "GET"
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
        method = "POST"
    request = Request(url, data=data, headers=headers, method=method)
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            status = getattr(response, "status", None) or response.getcode()
            raw = response.read().decode("utf-8")
    except (OSError, URLError, TimeoutError) as exc:
        return {"_fetch_error": str(exc), "_http_status": None}
    try:
        payload = json.loads(raw) if raw else {}
    except json.JSONDecodeError as exc:
        payload = {"_fetch_error": f"invalid_json: {exc}"}
    if not isinstance(payload, dict):
        payload = {"value": payload}
    payload["_http_status"] = int(status)
    return payload


def collect_inspection_acceptance(
    *,
    gateway_url: str = DEFAULT_GATEWAY_URL,
    timeout_sec: float = 2.0,
    mode: str = "field",
    map_dir: str | None = None,
    points: Sequence[Any] = (),
    tag: str | None = None,
    require_octomap: bool = False,
    require_occupancy: bool = False,
    expected_data_source: str | None = None,
    expected_source_profile: str | None = None,
    expected_frame_id: str | None = None,
) -> dict[str, Any]:
    """Collect field readiness plus non-motion previews for inspection points."""

    field_check = collect_product_field_check(
        gateway_url=gateway_url,
        timeout_sec=timeout_sec,
        mode=mode,
        map_dir=map_dir,
        require_octomap=require_octomap,
        require_occupancy=require_occupancy,
        expected_data_source=expected_data_source,
        expected_source_profile=expected_source_profile,
        expected_frame_id=expected_frame_id,
    )
    locations = _request_json(
        gateway_url,
        "/api/v1/locations",
        timeout_sec=timeout_sec,
    )
    targets = inspection_targets_from_payload(
        locations,
        points=points,
        tag=tag,
    )

    candidates = []
    for target in targets:
        if target.get("_invalid_payload"):
            candidates.append(
                {
                    "ok": False,
                    "error": "invalid_inspection_target",
                    "reasons": ["inspection targets must be saved location names"],
                }
            )
            continue
        if target.get("_missing_from_locations"):
            candidates.append(
                {
                    "ok": False,
                    "error": "location_not_found",
                    "reasons": ["location_not_found"],
                }
            )
            continue
        candidates.append(
            _request_json(
                gateway_url,
                "/api/v1/navigation/goal_candidate",
                timeout_sec=timeout_sec,
                body=goal_candidate_body_for_target(target),
            )
        )

    return build_inspection_acceptance(
        field_check=field_check,
        targets=targets,
        candidates=candidates,
        locations=locations,
        gateway_url=gateway_url,
    )
