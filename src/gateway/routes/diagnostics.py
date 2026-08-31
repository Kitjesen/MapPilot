"""Diagnostic export routes for GatewayModule."""

from __future__ import annotations

import io
import json
import os
import pathlib
import subprocess
import tarfile
import tempfile
import time
from collections.abc import Mapping
from datetime import datetime
from typing import Any

from gateway.schemas import (
    InspectionAcceptanceRequest,
    InspectionAcceptanceResponse,
    ProductFieldCheckRequest,
    ProductFieldCheckResponse,
    RealRuntimeEvidenceLatestResponse,
    RoutecheckLatestResponse,
    RuntimeContractResponse,
)

# Simple TTL cache for expensive diagnostic builders.
# Cache key: function name; cache value: (timestamp, result).
# TTL is 5 seconds; diagnostics are read-only and stale-by-seconds is acceptable.
_CACHE: dict[str, tuple[float, Any]] = {}
_CACHE_TTL = 5.0


def _cached(key: str, factory, ttl: float = _CACHE_TTL):
    now = time.time()
    if key in _CACHE:
        ts, result = _CACHE[key]
        if now - ts < ttl:
            return result
    result = factory()
    _CACHE[key] = (now, result)
    return result


def clear_diagnostics_cache() -> None:
    """Clear the diagnostics TTL cache.  Used in tests for isolation."""
    _CACHE.clear()


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[3]


def _json_ready(value: Any) -> Any:
    try:
        json.dumps(value, ensure_ascii=False, default=str)
        return value
    except Exception:
        return str(value)


def _json_payload(value: Any) -> Any:
    return json.loads(json.dumps(value, ensure_ascii=False, default=str))


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _location_entries(gw: Any) -> list[Any]:
    tlm = getattr(gw, "_tagged_loc_module", None)
    if tlm is None:
        return []
    try:
        return list(tlm.store.list_all())
    except Exception:
        try:
            return list(tlm.store._store.values())
        except Exception:
            return []


def _snapshot_or_error(name: str, builder) -> dict[str, Any]:
    try:
        value = builder()
        return {"ok": True, "data": _json_ready(value)}
    except Exception as exc:
        return {"ok": False, "error": str(exc), "snapshot": name}


def _maps_snapshot(gw: Any) -> dict[str, Any]:
    source = "mapd"
    try:
        from runtime.endpoints.mapd import MapClient

        manager_snapshot = MapClient().service("list_maps")
    except Exception as exc:
        manager_snapshot = {
            "action": "list_maps",
            "success": False,
            "message": str(exc),
            "maps": [],
            "active": "",
        }

    maps = (
        manager_snapshot.get("maps")
        if isinstance(manager_snapshot, dict) and isinstance(manager_snapshot.get("maps"), list)
        else []
    )
    active = str(manager_snapshot.get("active") or "") if isinstance(manager_snapshot, dict) else ""

    return {
        "schema_version": 1,
        "active": active,
        "maps": maps,
        "count": len(maps),
        "has_manager": False,
        "source": source,
        "manager": manager_snapshot,
        "ts": time.time(),
    }


def _command_snapshot(gw: Any) -> dict[str, Any]:
    if callable(getattr(gw, "_command_stats_snapshot", None)):
        snapshot = gw._command_stats_snapshot()
        if isinstance(snapshot, dict):
            return snapshot
    return {
        "idempotency_supported": False,
        "stored_requests": 0,
        "accepted_commands": 0,
        "replayed_commands": 0,
        "rate_policy_hz": {},
        "rate_policy_enforcement": "unknown",
    }


def build_plugin_catalog() -> dict[str, Any]:
    """Read-only view of registered plugin categories and provider metadata."""
    return _cached("build_plugin_catalog", _build_plugin_catalog_impl)


def _build_plugin_catalog_impl() -> dict[str, Any]:
    from runtime.registry import get_metadata, list_categories, list_plugins

    categories: dict[str, list[dict[str, Any]]] = {}
    for category in list_categories():
        entries: list[dict[str, Any]] = []
        for name in list_plugins(category):
            entries.append(
                {
                    "name": name,
                    "metadata": _json_ready(get_metadata(category, name)),
                }
            )
        categories[category] = entries
    return {
        "schema_version": 1,
        "categories": categories,
        "category_count": len(categories),
        "ts": time.time(),
    }


_BACKEND_STATUS_KEYS = {
    "configured_backend",
    "backend",
    "degraded",
    "degraded_reason",
}


def _is_backend_status_payload(value: Mapping[str, Any]) -> bool:
    return _BACKEND_STATUS_KEYS <= set(value.keys())


def _backend_status_payload(value: Mapping[str, Any]) -> dict[str, Any]:
    payload = {str(key): _json_ready(raw_value) for key, raw_value in value.items()}
    payload.update(
        {
            "configured_backend": _json_ready(value.get("configured_backend")),
            "backend": _json_ready(value.get("backend")),
            "degraded": bool(value.get("degraded")),
            "degraded_reason": str(value.get("degraded_reason") or ""),
        }
    )
    return payload


def _collect_backend_statuses(
    value: Any,
    *,
    prefix: str = "",
) -> dict[str, dict[str, Any]]:
    if not isinstance(value, Mapping):
        return {}
    backends: dict[str, dict[str, Any]] = {}
    if _is_backend_status_payload(value):
        backends[prefix or "backend"] = _backend_status_payload(value)
    for key, child in value.items():
        if not isinstance(child, Mapping):
            continue
        child_prefix = f"{prefix}.{key}" if prefix else str(key)
        backends.update(_collect_backend_statuses(child, prefix=child_prefix))
    return backends


def build_active_backend_status(gw: Any) -> dict[str, Any]:
    """Read-only view of configured/effective backends from live module health."""
    return _cached("build_active_backend_status", lambda: _build_active_backend_status_impl(gw))


def _build_active_backend_status_impl(gw: Any) -> dict[str, Any]:
    modules: dict[str, Any] = {}
    for name, module in (getattr(gw, "_all_modules", None) or {}).items():
        try:
            if not hasattr(module, "health"):
                continue
            health = module.health()
            backends = _collect_backend_statuses(health)
            if backends:
                modules[str(name)] = {"backends": backends}
        except Exception as exc:
            modules[str(name)] = {"error": str(exc)}
    return {
        "schema_version": 1,
        "modules": modules,
        "module_count": len(modules),
        "ts": time.time(),
    }


def _routecheck_artifacts_root(explicit: str | os.PathLike[str] | None = None) -> pathlib.Path:
    if explicit:
        return pathlib.Path(explicit).expanduser()
    env_root = os.environ.get("LINGTU_ROUTECHECK_ARTIFACT_ROOT")
    if env_root:
        return pathlib.Path(env_root).expanduser()
    return pathlib.Path.home() / "data" / "SLAM" / "navigation" / "artifacts"


def _routecheck_artifacts_roots(
    explicit: str | os.PathLike[str] | None = None,
) -> list[pathlib.Path]:
    roots = [_routecheck_artifacts_root(explicit)]
    if explicit or os.environ.get("LINGTU_ROUTECHECK_ARTIFACT_ROOT"):
        return roots
    roots.append(PROJECT_ROOT / "artifacts")

    unique: list[pathlib.Path] = []
    seen: set[str] = set()
    for root in roots:
        key = str(root)
        if key not in seen:
            seen.add(key)
            unique.append(root)
    return unique


def _iter_routecheck_summary_paths(root: pathlib.Path) -> list[pathlib.Path]:
    paths: list[pathlib.Path] = []
    seen: set[str] = set()
    for pattern in ("*/summary.json", "*/*/summary.json"):
        for path in root.glob(pattern):
            key = str(path)
            if key not in seen:
                seen.add(key)
                paths.append(path)
    return paths


def build_routecheck_latest_summary(
    artifacts_root: str | os.PathLike[str] | None = None,
) -> dict[str, Any]:
    roots = _routecheck_artifacts_roots(artifacts_root)
    summaries: list[tuple[float, pathlib.Path, dict[str, Any]]] = []
    for root in roots:
        if root.is_dir():
            for summary_path in _iter_routecheck_summary_paths(root):
                try:
                    data = json.loads(summary_path.read_text(encoding="utf-8"))
                except Exception:
                    continue
                if not isinstance(data, dict):
                    continue
                if data.get("mode") != "routecheck_non_motion":
                    continue
                try:
                    mtime = summary_path.stat().st_mtime
                except OSError:
                    mtime = 0.0
                summaries.append((mtime, summary_path, data))

    primary_root = roots[0]
    searched_roots = [str(root) for root in roots]
    if not summaries:
        return {
            "schema_version": 1,
            "ok": False,
            "artifacts_root": str(primary_root),
            "searched_roots": searched_roots,
            "count": 0,
            "artifact_dir": None,
            "summary_path": None,
            "latest": None,
            "reason": "routecheck_summary_not_found",
            "ts": time.time(),
        }

    summaries.sort(key=lambda item: item[0], reverse=True)
    report_mtime, summary_path, latest = summaries[0]
    generated_at = time.time()
    published = latest.get("published") if isinstance(latest.get("published"), dict) else None
    selected_root = next(
        (root for root in roots if summary_path.is_relative_to(root)),
        summary_path.parent,
    )
    return {
        "schema_version": 1,
        "ok": True,
        "artifacts_root": str(selected_root),
        "searched_roots": searched_roots,
        "count": len(summaries),
        "artifact_dir": str(summary_path.parent),
        "summary_path": str(summary_path),
        "report_mtime": report_mtime,
        "report_age_s": max(0.0, generated_at - report_mtime),
        "non_motion": latest.get("non_motion"),
        "simulation_only": latest.get("simulation_only"),
        "real_robot_motion": latest.get("real_robot_motion"),
        "cmd_vel_sent_to_hardware": latest.get("cmd_vel_sent_to_hardware"),
        "gateway_used": latest.get("gateway_used"),
        "driver_used": latest.get("driver_used"),
        "published": published,
        "latest": latest,
        "reason": None,
        "ts": generated_at,
    }


def _real_runtime_evidence_artifacts_root(
    explicit: str | os.PathLike[str] | None = None,
) -> pathlib.Path:
    if explicit:
        return pathlib.Path(explicit).expanduser()
    env_root = os.environ.get("LINGTU_REAL_RUNTIME_EVIDENCE_ROOT")
    if env_root:
        return pathlib.Path(env_root).expanduser()
    env_artifacts = os.environ.get("LINGTU_ARTIFACT_ROOT")
    if env_artifacts:
        return pathlib.Path(env_artifacts).expanduser() / "real_runtime"
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    return repo_root / "artifacts" / "real_runtime"


def _real_runtime_evidence_max_age_s(explicit: float | None = None) -> float:
    if explicit is not None:
        return float(explicit)
    raw = os.environ.get("LINGTU_REAL_RUNTIME_EVIDENCE_MAX_AGE_SEC")
    if raw:
        try:
            return float(raw)
        except ValueError:
            pass
    return 3600.0


def _load_json_file(path: pathlib.Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def _real_runtime_report_candidates(root: pathlib.Path) -> list[pathlib.Path]:
    if not root.is_dir():
        return []
    candidates: list[pathlib.Path] = []
    direct = root / "report.json"
    if direct.is_file():
        candidates.append(direct)
    for path in root.glob("*/report.json"):
        if path.is_file() and path not in candidates:
            candidates.append(path)
    return candidates


def _real_runtime_evidence_summary_from_report(
    report_path: pathlib.Path,
    *,
    now: float,
    max_age_s: float,
) -> dict[str, Any] | None:
    from runtime.runtime_interface import REAL_RUNTIME_CONTRACT

    report = _load_json_file(report_path)
    if report is None:
        return None

    validation = report.get("runtime_evidence")
    if not isinstance(validation, dict):
        validation = {}

    try:
        report_mtime = report_path.stat().st_mtime
    except OSError:
        report_mtime = 0.0
    age_s = max(0.0, now - report_mtime)
    contract_name = validation.get("expected_contract")
    blockers: list[str] = []
    if not validation:
        blockers.append("real-runtime-evidence validation payload missing")
    if validation.get("ok") is not True:
        blockers.append("real-runtime-evidence gate did not pass")
        blockers.extend(str(item) for item in (validation.get("blockers") or []) if item)
    if contract_name != REAL_RUNTIME_CONTRACT:
        blockers.append(f"real-runtime-evidence contract is not {REAL_RUNTIME_CONTRACT}")
    if report.get("simulation_only") is not False:
        blockers.append("real-runtime-evidence simulation_only is not false")
    if report.get("real_robot_motion") is not True:
        blockers.append("real-runtime-evidence real_robot_motion is not true")
    if report.get("cmd_vel_sent_to_hardware") is not True:
        blockers.append("real-runtime-evidence cmd_vel_sent_to_hardware is not true")
    if age_s > max_age_s:
        blockers.append("real-runtime-evidence is stale")

    return {
        "schema_version": 1,
        "ok": not blockers,
        "artifacts_root": str(report_path.parent.parent),
        "artifact_dir": str(report_path.parent),
        "report_path": str(report_path),
        "report_mtime": report_mtime,
        "report_age_s": age_s,
        "max_age_s": max_age_s,
        "runtime_contract": contract_name,
        "runtime_evidence_ok": validation.get("ok") is True,
        "simulation_only": report.get("simulation_only"),
        "real_robot_motion": report.get("real_robot_motion"),
        "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
        "blockers": list(dict.fromkeys(blockers)),
        "reason": blockers[0] if blockers else None,
        "ts": now,
    }


def build_real_runtime_evidence_latest_summary(
    artifacts_root: str | os.PathLike[str] | None = None,
    *,
    max_age_s: float | None = None,
    now: float | None = None,
) -> dict[str, Any]:
    root = _real_runtime_evidence_artifacts_root(artifacts_root)
    age_limit = _real_runtime_evidence_max_age_s(max_age_s)
    generated_at = time.time() if now is None else now
    candidates: list[tuple[float, pathlib.Path, dict[str, Any]]] = []
    for report_path in _real_runtime_report_candidates(root):
        summary = _real_runtime_evidence_summary_from_report(
            report_path,
            now=generated_at,
            max_age_s=age_limit,
        )
        if summary is None:
            continue
        try:
            mtime = report_path.stat().st_mtime
        except OSError:
            mtime = 0.0
        candidates.append((mtime, report_path, summary))

    if not candidates:
        return {
            "schema_version": 1,
            "ok": False,
            "artifacts_root": str(root),
            "count": 0,
            "artifact_dir": None,
            "report_path": None,
            "report_mtime": None,
            "report_age_s": None,
            "max_age_s": age_limit,
            "runtime_contract": None,
            "runtime_evidence_ok": False,
            "simulation_only": None,
            "real_robot_motion": None,
            "cmd_vel_sent_to_hardware": None,
            "blockers": ["real_runtime_evidence_report_not_found"],
            "reason": "real_runtime_evidence_report_not_found",
            "ts": generated_at,
        }

    candidates.sort(key=lambda item: item[0], reverse=True)
    summary = dict(candidates[0][2])
    summary["count"] = len(candidates)
    summary["artifacts_root"] = str(root)
    return summary


def _runtime_contract_snapshot() -> dict[str, Any]:
    from runtime.runtime_interface import runtime_contract_manifest

    return {
        "schema_version": 1,
        "source": "runtime.runtime_interface.runtime_contract_manifest",
        "manifest": runtime_contract_manifest(),
        "ts": time.time(),
    }


def _first_path_frame(path: Any) -> str | None:
    if not path:
        return None
    first = path[0]
    if isinstance(first, dict):
        frame = first.get("frame_id") or first.get("frame")
        if frame:
            return str(frame)
        header = first.get("header")
        if isinstance(header, dict):
            frame = header.get("frame_id") or header.get("frame")
            if frame:
                return str(frame)
    return None


def _frame_contract_snapshot(gw: Any) -> dict[str, Any]:
    from gateway.services.runtime_status import build_navigation_status
    from runtime.runtime_interface import runtime_contract_manifest

    manifest = runtime_contract_manifest()
    runtime_frames = manifest.get("frames", {})
    if not isinstance(runtime_frames, dict):
        runtime_frames = {}
    runtime_links = manifest.get("frame_links", {})
    if not isinstance(runtime_links, dict):
        runtime_links = {}

    nav_status = build_navigation_status(gw)
    frames = nav_status.get("frames", {})
    if not isinstance(frames, dict):
        frames = {}
    runtime_boundary = nav_status.get("runtime", {})
    if not isinstance(runtime_boundary, dict):
        runtime_boundary = {}
    frame_tree = getattr(gw, "_frame_tree", None)
    frame_tree_snapshot = frame_tree.snapshot() if frame_tree is not None and hasattr(frame_tree, "snapshot") else None

    with gw._state_lock:
        odom = dict(gw._odom) if isinstance(gw._odom, dict) else gw._odom
        mission = (
            dict(gw._navigation_state)
            if isinstance(gw._navigation_state, dict)
            else gw._navigation_state
        )
        localization = (
            dict(gw._localization_status) if isinstance(gw._localization_status, dict) else gw._localization_status
        )
        path = list(gw._last_path or [])

    return {
        "schema_version": 1,
        "contract": manifest,
        "expected": {
            "map_frame": runtime_frames.get("map"),
            "odom_frame": runtime_frames.get("odom"),
            "body_frame": runtime_frames.get("body"),
            "links": runtime_links,
        },
        "observed": {
            "odometry_frame_id": (odom.get("frame_id") if isinstance(odom, dict) else None),
            "odometry_child_frame_id": (odom.get("child_frame_id") if isinstance(odom, dict) else None),
            "mission_frame_id": (mission.get("frame_id") if isinstance(mission, dict) else None),
            "mission_planning_frame_id": (mission.get("planning_frame_id") if isinstance(mission, dict) else None),
            "mission_odom_frame_id": (mission.get("odom_frame_id") if isinstance(mission, dict) else None),
            "path_frame_id": _first_path_frame(path),
            "path_point_count": len(path),
            "has_map_odom_tf": bool(getattr(gw, "_has_map_odom_tf", False)),
            "localization_backend": (localization.get("backend") if isinstance(localization, dict) else None),
            "localization_state": (localization.get("state") if isinstance(localization, dict) else None),
        },
        "navigation_frames": frames,
        "frame_tree": _json_payload(frame_tree_snapshot),
        "runtime_boundary": _json_payload(runtime_boundary),
        "mismatches": frames.get("mismatches", []),
        "ok": bool(frames.get("ok", True)),
        "ts": time.time(),
    }


def _build_app_web_snapshots(gw: Any) -> dict[str, dict[str, Any]]:
    from gateway.services.app_bootstrap import (
        build_app_bootstrap,
        build_app_capabilities,
        build_app_traffic,
    )
    from gateway.services.media_status import build_media_status
    from gateway.services.readiness import build_readiness_snapshot
    from gateway.services.runtime_status import (
        build_localization_status,
        build_navigation_status,
    )
    from gateway.services.state_snapshot import build_state_snapshot
    from gateway.services.telemetry_normalizers import (
        build_locations_response,
        build_path_response,
        build_scene_graph_response,
    )

    def _scene_graph():
        with gw._state_lock:
            sg = gw._sg_json
        return build_scene_graph_response(sg)

    def _path():
        with gw._state_lock:
            path = gw._last_path
            robot = gw._odom
        return build_path_response(path, robot)

    readiness = _snapshot_or_error(
        "readiness",
        lambda: build_readiness_snapshot(gw, include_details=True)[0],
    )
    return {
        "bootstrap": _snapshot_or_error(
            "bootstrap",
            lambda: build_app_bootstrap(gw),
        ),
        "capabilities": _snapshot_or_error(
            "capabilities",
            lambda: build_app_capabilities(gw),
        ),
        "traffic": _snapshot_or_error(
            "traffic",
            lambda: build_app_traffic(gw),
        ),
        "readiness": readiness,
        "state": _snapshot_or_error("state", lambda: build_state_snapshot(gw)),
        "localization": _snapshot_or_error(
            "localization",
            lambda: build_localization_status(gw),
        ),
        "navigation": _snapshot_or_error(
            "navigation",
            lambda: build_navigation_status(gw),
        ),
        "path": _snapshot_or_error("path", _path),
        "scene_graph": _snapshot_or_error("scene_graph", _scene_graph),
        "locations": _snapshot_or_error(
            "locations",
            lambda: build_locations_response(_location_entries(gw)),
        ),
        "media": _snapshot_or_error(
            "media",
            lambda: build_media_status(gw),
        ),
        "session": _snapshot_or_error(
            "session",
            lambda: gw._session_snapshot(),
        ),
        "maps": _snapshot_or_error("maps", lambda: _maps_snapshot(gw)),
        "commands": _snapshot_or_error("commands", lambda: _command_snapshot(gw)),
        "routecheck": _snapshot_or_error(
            "routecheck",
            lambda: build_routecheck_latest_summary(),
        ),
        "real_runtime_evidence": _snapshot_or_error(
            "real_runtime_evidence",
            lambda: build_real_runtime_evidence_latest_summary(),
        ),
        "frame_contract": _snapshot_or_error(
            "frame_contract",
            lambda: _frame_contract_snapshot(gw),
        ),
        "runtime_contract": _snapshot_or_error(
            "runtime_contract",
            _runtime_contract_snapshot,
        ),
    }


def _gateway_acceptance_snapshots(gw: Any) -> dict[str, dict[str, Any]]:
    from gateway.services.app_bootstrap import build_app_capabilities
    from gateway.services.readiness import build_readiness_snapshot
    from gateway.services.runtime_dataflow import build_runtime_dataflow_snapshot
    from gateway.services.runtime_status import (
        build_localization_status,
        build_navigation_status,
    )

    return {
        "capabilities": build_app_capabilities(gw),
        "readiness": build_readiness_snapshot(gw, include_details=True)[0],
        "runtime_dataflow": build_runtime_dataflow_snapshot(gw),
        "localization_status": build_localization_status(gw),
        "navigation_status": build_navigation_status(gw),
        "real_runtime_evidence": build_real_runtime_evidence_latest_summary(),
    }


def _inspection_map_gate(gw: Any, body: Any) -> dict[str, Any] | None:
    from gateway.services.mapd_transport import (
        active_map,
        validate_map_artifacts,
    )

    map_name = active_map(gw)
    if not map_name:
        return None
    response = validate_map_artifacts(
        gw,
        map_name,
        require_octomap=bool(getattr(body, "require_octomap", False)),
        require_occupancy=bool(getattr(body, "require_occupancy", False)),
        expected_data_source=getattr(body, "expected_data_source", None),
        expected_source_profile=getattr(body, "expected_source_profile", None),
        expected_frame_id=getattr(body, "expected_frame_id", None),
    )
    if response is None:
        return None
    gate = dict(response.get("gate") or {})
    gate["map_id"] = map_name
    return gate


def _inspection_candidate(gw: Any, target: dict[str, Any]) -> dict[str, Any]:
    from diagnostics.field.inspection import goal_candidate_body_for_target
    from gateway.schemas import GoalCandidateRequest
    from gateway.services.control_commands import ControlCommandService
    from gateway.services.goal_builder import construct_goal_from_request

    if target.get("_invalid_payload"):
        return {
            "ok": False,
            "status": "invalid",
            "target": None,
            "preview": None,
            "reasons": ["inspection targets must be saved location names"],
            "error": "invalid_inspection_target",
            "ts": time.time(),
        }

    if target.get("_missing_from_locations"):
        return {
            "ok": False,
            "status": "invalid",
            "target": None,
            "preview": None,
            "reasons": ["location_not_found"],
            "error": "location_not_found",
            "ts": time.time(),
        }

    ts = time.time()
    payload = goal_candidate_body_for_target(target)
    try:
        body = GoalCandidateRequest.model_validate(payload)
        goal = construct_goal_from_request(
            body,
            gw=gw,
            default_source=body.source,
            default_target_type=body.target_type,
        )
        preview = ControlCommandService(gw).preview_plan(goal.preview_request())
    except Exception as exc:
        return {
            "schema_version": 1,
            "ok": False,
            "status": "invalid",
            "target": None,
            "preview": None,
            "reasons": [str(exc)],
            "error": str(exc),
            "ts": ts,
        }

    reasons = list(preview.get("reasons") or [])
    if preview.get("ok") is not True:
        return {
            "schema_version": 1,
            "ok": False,
            "status": "preview_unavailable",
            "target": goal.target_payload(ts=ts),
            "preview": preview,
            "reasons": reasons,
            "error": preview.get("error") or "preview_unavailable",
            "ts": ts,
        }
    return {
        "schema_version": 1,
        "ok": True,
        "status": ("preview_feasible" if bool(preview.get("feasible", False)) else "preview_infeasible"),
        "target": goal.target_payload(ts=ts),
        "preview": preview,
        "reasons": reasons,
        "error": None,
        "ts": ts,
    }


def field_check(gw: Any, body: Any) -> dict[str, Any]:
    from diagnostics.field.field_check import build_product_field_check
    from diagnostics.field.gateway_acceptance import evaluate_gateway_runtime_acceptance

    gateway_acceptance = evaluate_gateway_runtime_acceptance(
        _gateway_acceptance_snapshots(gw),
        mode=getattr(body, "mode", "field"),
    )
    return build_product_field_check(
        gateway_acceptance,
        map_gate=_inspection_map_gate(gw, body),
    )


def inspection_check(gw: Any, body: Any) -> dict[str, Any]:
    from diagnostics.field.inspection import (
        build_inspection_acceptance,
        inspection_targets_from_payload,
    )
    from gateway.services.telemetry_normalizers import build_locations_response

    product_check = field_check(gw, body)
    locations = build_locations_response(_location_entries(gw))
    targets = inspection_targets_from_payload(
        locations,
        points=list(getattr(body, "points", None) or []),
        tag=getattr(body, "tag", None),
    )
    candidates = [_inspection_candidate(gw, target) for target in targets]
    return build_inspection_acceptance(
        field_check=product_check,
        targets=targets,
        candidates=candidates,
        locations=locations,
    )


def register_diagnostic_routes(app, gw) -> None:
    from fastapi import Body
    from fastapi.responses import FileResponse
    from starlette.background import BackgroundTask

    @app.get(
        "/api/v1/diagnostics/routecheck/latest",
        response_model=RoutecheckLatestResponse,
        summary="Read latest non-motion routecheck summary",
    )
    async def routecheck_latest():
        return build_routecheck_latest_summary()

    @app.get(
        "/api/v1/diagnostics/real-runtime-evidence/latest",
        response_model=RealRuntimeEvidenceLatestResponse,
        summary="Read latest Thunder field runtime evidence gate summary",
    )
    async def real_runtime_evidence_latest():
        return build_real_runtime_evidence_latest_summary()

    @app.get(
        "/api/v1/diagnostics/runtime-contract",
        response_model=RuntimeContractResponse,
        summary="Read canonical runtime interface contract",
    )
    async def runtime_contract():
        return _runtime_contract_snapshot()

    @app.get(
        "/api/v1/diagnostics/plugins",
        summary="Read registered plugin categories and providers",
    )
    async def plugin_catalog():
        payload = build_plugin_catalog()
        payload["active"] = build_active_backend_status(gw)
        return payload

    @app.post(
        "/api/v1/diagnostics/field-check",
        response_model=ProductFieldCheckResponse,
        summary="Run read-only product field readiness check",
    )
    async def product_field_check(
        body: ProductFieldCheckRequest = Body(default_factory=ProductFieldCheckRequest),
    ):
        return field_check(gw, body)

    @app.post(
        "/api/v1/inspection/acceptance",
        response_model=InspectionAcceptanceResponse,
        summary="Run read-only inspection acceptance without publishing motion commands",
    )
    async def inspection_acceptance(
        body: InspectionAcceptanceRequest = Body(default_factory=InspectionAcceptanceRequest),
    ):
        return inspection_check(
            gw,
            body,
        )

    @app.get(
        "/api/v1/diagnostic_pack",
        summary="Export diagnostic tarball",
        responses={200: {"content": {"application/gzip": {"schema": {"type": "string", "format": "binary"}}}}},
    )
    async def diagnostic_pack():
        repo_root = pathlib.Path(__file__).resolve().parents[3]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tmp_path = pathlib.Path(tempfile.gettempdir()) / f"diag_{stamp}.tar.gz"

        def _add_text(tar: tarfile.TarFile, arcname: str, text: str) -> None:
            data = text.encode("utf-8")
            info = tarfile.TarInfo(arcname)
            info.size = len(data)
            info.mtime = int(time.time())
            tar.addfile(info, io.BytesIO(data))

        with tarfile.open(tmp_path, "w:gz") as tar:
            modules_info: dict[str, Any] = {}
            for name, module in (getattr(gw, "_all_modules", None) or {}).items():
                try:
                    if hasattr(module, "health"):
                        modules_info[name] = module.health()
                    elif hasattr(module, "port_summary"):
                        modules_info[name] = module.port_summary()
                except Exception as exc:
                    modules_info[name] = {"error": str(exc)}

            _add_text(
                tar,
                "diag/modules.json",
                json.dumps(
                    modules_info,
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )
            _add_text(
                tar,
                "diag/health.json",
                json.dumps(
                    {
                        "modules_ok": sum(1 for v in modules_info.values() if "error" not in v),
                        "modules_fail": sum(1 for v in modules_info.values() if "error" in v),
                        "count": len(modules_info),
                    },
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )

            try:
                git_out = (
                    subprocess.check_output(
                        ["git", "rev-parse", "HEAD"],
                        cwd=repo_root,
                        stderr=subprocess.DEVNULL,
                        timeout=3,
                    )
                    .decode()
                    .strip()
                )
                short = git_out[:12]
            except Exception:
                git_out, short = "unknown", "unknown"
            _add_text(tar, "diag/git_head.txt", f"{git_out}\nshort: {short}\n")

            app_web_snapshots = _build_app_web_snapshots(gw)
            _add_text(
                tar,
                "diag/runtime_contract.json",
                json.dumps(
                    _runtime_contract_snapshot(),
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )
            _add_text(
                tar,
                "diag/app_web_snapshots.json",
                json.dumps(
                    app_web_snapshots,
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )
            for name, snapshot in app_web_snapshots.items():
                _add_text(
                    tar,
                    f"diag/app_web/{name}.json",
                    json.dumps(
                        snapshot,
                        indent=2,
                        ensure_ascii=False,
                        default=str,
                    ),
                )

            configured_path = os.environ.get("LINGTU_CONFIG_PATH")
            cfg_path = (
                pathlib.Path(configured_path)
                if configured_path
                else repo_root / "config" / "robots" / "unitree" / "go2" / "robot.yaml"
            )
            if not cfg_path.is_absolute():
                cfg_path = repo_root / cfg_path
            if cfg_path.exists():
                tar.add(str(cfg_path), arcname="diag/robot_config.yaml")

            logs_root = repo_root / "logs"
            if logs_root.exists():
                latest_log_dirs = sorted(
                    [p for p in logs_root.iterdir() if p.is_dir()],
                    key=lambda p: p.stat().st_mtime,
                    reverse=True,
                )[:1]
                for log_dir in latest_log_dirs:
                    for logfile in log_dir.glob("*.log"):
                        try:
                            tar.add(
                                str(logfile),
                                arcname=f"diag/logs/{log_dir.name}/{logfile.name}",
                            )
                        except OSError:
                            pass

        return FileResponse(
            path=str(tmp_path),
            filename=f"lingtu_diag_{stamp}.tar.gz",
            media_type="application/gzip",
            background=BackgroundTask(tmp_path.unlink, missing_ok=True),
        )
