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
from datetime import datetime
from collections.abc import Mapping
from typing import Any

from core.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from core.algorithm_gates import INSPECTION_MVP_REQUIRED_GATES
from gateway.schemas import AlgorithmBenchmarkLatestResponse
from gateway.schemas import InspectionAcceptanceRequest
from gateway.schemas import InspectionAcceptanceResponse
from gateway.schemas import ProductFieldCheckRequest
from gateway.schemas import ProductFieldCheckResponse
from gateway.schemas import RealRuntimeEvidenceLatestResponse
from gateway.schemas import RoutecheckLatestResponse
from gateway.schemas import RuntimeContractResponse


# Simple TTL cache for expensive diagnostic builders.
# Cache key: function name; cache value: (timestamp, result).
# TTL is 5 seconds — diagnostics are read-only and stale-by-seconds is acceptable.
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


ALGORITHM_BENCHMARK_SCHEMA_VERSION = "lingtu.algorithm_benchmark_latest.v1"
PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[3]
ALGORITHM_PRODUCT_PROFILES: dict[str, dict[str, Any]] = {
    "inspection_mvp": {
        "label": "Inspection MVP",
        "claim": "routine_inspection_simulation_readiness",
        "purpose": (
            "Product-facing patrol readiness through Gateway/ModulePorts; "
            "ROS2 evidence is adapter evidence, not the product API."
        ),
        "required_gate_sequence": INSPECTION_MVP_REQUIRED_GATES,
    },
    "dimos_benchmark": {
        "label": "Dimos benchmark",
        "claim": "strict_full_algorithm_reference",
        "purpose": (
            "Strict reference suite for whole-algorithm closure across "
            "mapping, localization, planning, dynamic obstacles, and saved maps."
        ),
        "required_gate_sequence": DIMOS_BENCHMARK_REQUIRED_GATES,
    },
}


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
    from gateway.services.map_paths import active_map_name, nav_map_root_str

    root = pathlib.Path(nav_map_root_str())
    maps: list[dict[str, Any]] = []
    if root.is_dir():
        for entry in sorted(root.iterdir()):
            if not entry.is_dir() or entry.name == "active":
                continue
            pcd = entry / "map.pcd"
            patches = entry / "patches"
            maps.append(
                {
                    "name": entry.name,
                    "has_pcd": pcd.is_file(),
                    "has_tomogram": (entry / "tomogram.pickle").is_file(),
                    "has_occupancy": (entry / "occupancy.npz").is_file(),
                    "patch_count": (
                        len(list(patches.iterdir())) if patches.is_dir() else 0
                    ),
                    "size_mb": (
                        round(pcd.stat().st_size / 1024 / 1024, 1)
                        if pcd.is_file()
                        else None
                    ),
                }
            )

    manager_snapshot: dict[str, Any] | None = None
    manager = getattr(gw, "_map_mgr", None)
    if manager is not None and callable(getattr(manager, "_map_list", None)):
        manager_snapshot = manager._map_list()

    return {
        "schema_version": 1,
        "map_dir": str(root),
        "active": active_map_name() or "",
        "maps": maps,
        "count": len(maps),
        "has_manager": manager is not None,
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
    from core.registry import get_metadata, list_categories, list_plugins

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
    payload = {
        str(key): _json_ready(raw_value)
        for key, raw_value in value.items()
    }
    payload.update({
        "configured_backend": _json_ready(value.get("configured_backend")),
        "backend": _json_ready(value.get("backend")),
        "degraded": bool(value.get("degraded")),
        "degraded_reason": str(value.get("degraded_reason") or ""),
    })
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
        return pathlib.Path(env_artifacts).expanduser() / "real_s100p_runtime"
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    return repo_root / "artifacts" / "real_s100p_runtime"


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


def _algorithm_benchmark_artifacts_root(
    explicit: str | os.PathLike[str] | None = None,
) -> pathlib.Path:
    if explicit:
        return pathlib.Path(explicit).expanduser()
    env_root = os.environ.get("LINGTU_ALGORITHM_BENCHMARK_ROOT")
    if env_root:
        return pathlib.Path(env_root).expanduser()
    env_artifacts = os.environ.get("LINGTU_ARTIFACT_ROOT")
    if env_artifacts:
        return pathlib.Path(env_artifacts).expanduser() / "server_sim_closure"
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    return repo_root / "artifacts" / "server_sim_closure"


def _algorithm_benchmark_max_age_s(explicit: float | None = None) -> float:
    if explicit is not None:
        return float(explicit)
    raw = os.environ.get("LINGTU_ALGORITHM_BENCHMARK_MAX_AGE_SEC")
    if raw:
        try:
            return float(raw)
        except ValueError:
            pass
    return 86400.0


def _is_algorithm_benchmark_summary(data: dict[str, Any]) -> bool:
    validation = data.get("algorithm_validation")
    if not isinstance(validation, dict):
        return False
    return isinstance(data.get("missing_or_failed"), list)


def _algorithm_benchmark_candidates(
    root: pathlib.Path,
) -> list[tuple[float, pathlib.Path, dict[str, Any]]]:
    patterns = (
        "summary_inspection_mvp*.json",
        "summary_dimos_benchmark*.json",
        "summary_core_algorithm*.json",
        "summary*.json",
    )
    seen: set[pathlib.Path] = set()
    candidates: list[tuple[float, pathlib.Path, dict[str, Any]]] = []
    if not root.is_dir():
        return candidates
    for pattern in patterns:
        for summary_path in root.glob(pattern):
            if summary_path in seen or not summary_path.is_file():
                continue
            seen.add(summary_path)
            data = _load_json_file(summary_path)
            if data is None or not _is_algorithm_benchmark_summary(data):
                continue
            try:
                mtime = summary_path.stat().st_mtime
            except OSError:
                mtime = 0.0
            candidates.append((mtime, summary_path, data))
    return sorted(candidates, key=lambda item: item[0], reverse=True)


def _algorithm_profile_unavailable(reason: str) -> dict[str, dict[str, Any]]:
    profiles: dict[str, dict[str, Any]] = {}
    for name, profile in ALGORITHM_PRODUCT_PROFILES.items():
        profiles[name] = {
            "name": name,
            "label": profile["label"],
            "claim": profile["claim"],
            "purpose": profile["purpose"],
            "ok": False,
            "status": "FAIL",
            "read_only": True,
            "ros2_topic_required": False,
            "publishes": [],
            "claim_allowed": False,
            "required_gate_sequence": list(profile["required_gate_sequence"]),
            "missing_or_failed": list(profile["required_gate_sequence"]),
            "blockers": [reason],
        }
    return profiles


def _algorithm_gate_failing(gate: Mapping[str, Any]) -> bool:
    status = str(gate.get("status") or "").lower()
    return gate.get("ok") is False or status in {
        "fail",
        "failed",
        "missing",
        "stale",
        "error",
    }


def _algorithm_product_profile_status(
    name: str,
    profile: Mapping[str, Any],
    *,
    latest: Mapping[str, Any],
    missing_or_failed: list[str],
    required_sequence: list[str],
    report_age_s: float,
    freshness: float,
    claim_boundary_ok: bool,
) -> dict[str, Any]:
    required = [str(item) for item in profile.get("required_gate_sequence") or () if item]
    required_set = set(required_sequence)
    missing_set = set(missing_or_failed)
    gates = _mapping(latest.get("gates"))
    profile_missing: list[str] = []
    blockers: list[str] = []

    if report_age_s > freshness:
        blockers.append("algorithm benchmark summary is stale")
    if not claim_boundary_ok:
        blockers.append("algorithm claim boundary must keep live_costmap local-only")

    for gate in required:
        gate_payload = _mapping(gates.get(gate))
        gate_failed = bool(gate_payload) and _algorithm_gate_failing(gate_payload)
        if gate not in required_set:
            profile_missing.append(gate)
            blockers.append(f"profile required gate missing from summary: {gate}")
        elif gate in missing_set or gate_failed:
            profile_missing.append(gate)
            blockers.append(f"profile required gate is not passing: {gate}")

    if name == "dimos_benchmark" and latest.get("ok") is not True:
        blockers.append("strict benchmark summary is not passing")

    profile_missing = list(dict.fromkeys(profile_missing))
    blockers = list(dict.fromkeys(blockers))
    ok = not blockers and not profile_missing
    return {
        "name": name,
        "label": profile.get("label") or name,
        "claim": profile.get("claim") or name,
        "purpose": profile.get("purpose") or "",
        "ok": ok,
        "status": "PASS" if ok else "FAIL",
        "read_only": True,
        "ros2_topic_required": False,
        "publishes": [],
        "claim_allowed": ok,
        "required_gate_sequence": required,
        "missing_or_failed": profile_missing,
        "blockers": blockers,
    }


def build_algorithm_benchmark_latest_summary(
    artifacts_root: str | os.PathLike[str] | None = None,
    *,
    max_age_s: float | None = None,
) -> dict[str, Any]:
    root = _algorithm_benchmark_artifacts_root(artifacts_root)
    freshness = _algorithm_benchmark_max_age_s(max_age_s)
    now = time.time()
    candidates = _algorithm_benchmark_candidates(root)
    base = {
        "schema_version": ALGORITHM_BENCHMARK_SCHEMA_VERSION,
        "ok": False,
        "read_only": True,
        "ros2_topic_required": False,
        "publishes": [],
        "artifacts_root": str(root),
        "count": len(candidates),
        "summary_path": None,
        "report_mtime": None,
        "report_age_s": None,
        "max_age_s": freshness,
        "preset": "dimos_benchmark",
        "source": "server_sim_closure",
        "active_product_profile": "inspection_mvp",
        "strict_benchmark_profile": "dimos_benchmark",
        "summary_schema_version": None,
        "claim_allowed": False,
        "missing_or_failed": [],
        "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
        "validation_flow": [],
        "claim_boundary": {},
        "product_profiles": _algorithm_profile_unavailable(
            "algorithm benchmark summary not found"
        ),
        "blocking_categories": {},
        "blockers": [],
        "reason": None,
        "latest": None,
        "ts": now,
    }
    if not candidates:
        base["reason"] = "algorithm_benchmark_report_not_found"
        base["blockers"] = ["algorithm benchmark summary not found"]
        return base

    report_mtime, summary_path, latest = candidates[0]
    report_age_s = max(0.0, now - report_mtime)
    validation = latest.get("algorithm_validation")
    validation = validation if isinstance(validation, dict) else {}
    missing_or_failed = [
        str(item) for item in latest.get("missing_or_failed") or [] if item
    ]
    required_sequence = [
        str(item)
        for item in (
            validation.get("required_gate_sequence")
            or latest.get("required")
            or DIMOS_BENCHMARK_REQUIRED_GATES
        )
        if item
    ]
    claim_boundary = _mapping(validation.get("claim_boundary"))
    claim_allowed = validation.get("claim_allowed") is True
    blockers: list[str] = []
    if report_age_s > freshness:
        blockers.append("algorithm benchmark summary is stale")
    if latest.get("ok") is not True:
        blockers.append("algorithm benchmark summary is not passing")
    if claim_allowed is not True:
        blockers.append("algorithm validation claim_allowed is not true")
    if missing_or_failed:
        blockers.append(
            "algorithm benchmark missing_or_failed: "
            + ",".join(missing_or_failed)
        )
    if set(DIMOS_BENCHMARK_REQUIRED_GATES) - set(required_sequence):
        blockers.append("algorithm benchmark required gate set is incomplete")
    if (
        claim_boundary.get("live_costmap_role")
        != "local_planning_and_safety_only"
        or claim_boundary.get("global_planning_source") == "live_costmap"
    ):
        blockers.append(
            "algorithm claim boundary must keep live_costmap local-only"
        )
    claim_boundary_ok = (
        claim_boundary.get("live_costmap_role")
        == "local_planning_and_safety_only"
        and claim_boundary.get("global_planning_source") != "live_costmap"
    )
    product_profiles = {
        name: _algorithm_product_profile_status(
            name,
            profile,
            latest=latest,
            missing_or_failed=missing_or_failed,
            required_sequence=required_sequence,
            report_age_s=report_age_s,
            freshness=freshness,
            claim_boundary_ok=claim_boundary_ok,
        )
        for name, profile in ALGORITHM_PRODUCT_PROFILES.items()
    }
    reason = None
    if blockers:
        reason = (
            "algorithm_benchmark_report_stale"
            if report_age_s > freshness
            else "algorithm_benchmark_not_passing"
        )

    return {
        **base,
        "ok": not blockers,
        "summary_path": str(summary_path),
        "report_mtime": report_mtime,
        "report_age_s": report_age_s,
        "summary_schema_version": latest.get("schema_version"),
        "claim_allowed": claim_allowed,
        "missing_or_failed": missing_or_failed,
        "required_gate_sequence": required_sequence,
        "validation_flow": list(validation.get("validation_flow") or []),
        "claim_boundary": claim_boundary,
        "product_profiles": product_profiles,
        "blocking_categories": _mapping(validation.get("blocking_categories")),
        "blockers": blockers,
        "reason": reason,
        "latest": latest,
        "ts": now,
    }


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


def _all_required_section_entries_ok(section: Any) -> bool:
    if not isinstance(section, dict) or not section:
        return False
    for entry in section.values():
        if isinstance(entry, dict):
            if entry.get("required") is False:
                continue
            if entry.get("ok") is not True:
                return False
        elif entry is not True:
            return False
    return True


def _real_runtime_evidence_summary_from_report(
    report_path: pathlib.Path,
    *,
    now: float,
    max_age_s: float,
) -> dict[str, Any] | None:
    report = _load_json_file(report_path)
    if report is None:
        return None

    validation = report.get("runtime_evidence")
    validation_path = report_path.parent / "runtime_evidence.json"
    if not isinstance(validation, dict) and validation_path.is_file():
        validation = _load_json_file(validation_path)
    if not isinstance(validation, dict):
        validation = {}

    try:
        report_mtime = report_path.stat().st_mtime
    except OSError:
        report_mtime = 0.0
    if validation_path.is_file():
        try:
            report_mtime = max(report_mtime, validation_path.stat().st_mtime)
        except OSError:
            pass

    age_s = max(0.0, now - report_mtime)
    runtime_contract = report.get("runtime_contract")
    runtime_contract = runtime_contract if isinstance(runtime_contract, dict) else {}
    contract_name = (
        runtime_contract.get("name")
        or validation.get("expected_contract")
        or validation.get("runtime_contract")
    )
    real_motion = validation.get("checked_real_motion_evidence")
    real_motion = real_motion if isinstance(real_motion, dict) else {}
    hardware = validation.get("checked_hardware_boundary_evidence")
    hardware = hardware if isinstance(hardware, dict) else {}
    freshness = validation.get("checked_live_topic_freshness")
    data_flow = validation.get("checked_runtime_data_flow_evidence")
    frame_links = validation.get("checked_frame_link_evidence")

    blockers: list[str] = []
    if not validation:
        blockers.append("real-runtime-evidence validation payload missing")
    if validation.get("ok") is not True:
        blockers.append("real-runtime-evidence gate did not pass")
        blockers.extend(str(item) for item in (validation.get("blockers") or []) if item)
    if contract_name != "real_s100p":
        blockers.append("real-runtime-evidence contract is not real_s100p")
    if report.get("simulation_only") is not False:
        blockers.append("real-runtime-evidence simulation_only is not false")
    if report.get("real_robot_motion") is not True:
        blockers.append("real-runtime-evidence real_robot_motion is not true")
    if report.get("cmd_vel_sent_to_hardware") is not True:
        blockers.append("real-runtime-evidence cmd_vel_sent_to_hardware is not true")
    if age_s > max_age_s:
        blockers.append("real-runtime-evidence is stale")
    if real_motion.get("ok") is not True:
        blockers.append("real-runtime-evidence real motion section missing or failed")
    if hardware.get("ok") is not True:
        blockers.append("real-runtime-evidence hardware boundary section missing or failed")
    if not _all_required_section_entries_ok(freshness):
        blockers.append("real-runtime-evidence live topic freshness missing or failed")
    if not _all_required_section_entries_ok(data_flow):
        blockers.append("real-runtime-evidence data-flow section missing or failed")
    if not _all_required_section_entries_ok(frame_links):
        blockers.append("real-runtime-evidence frame-link section missing or failed")

    return {
        "schema_version": 1,
        "ok": not blockers,
        "artifacts_root": str(report_path.parent.parent),
        "artifact_dir": str(report_path.parent),
        "report_path": str(report_path),
        "validation_path": str(validation_path) if validation_path.is_file() else None,
        "report_mtime": report_mtime,
        "report_age_s": age_s,
        "max_age_s": max_age_s,
        "runtime_contract": contract_name,
        "runtime_evidence_ok": validation.get("ok") is True,
        "simulation_only": report.get("simulation_only"),
        "real_robot_motion": report.get("real_robot_motion"),
        "cmd_vel_sent_to_hardware": report.get("cmd_vel_sent_to_hardware"),
        "checked_real_motion_evidence": real_motion,
        "checked_hardware_boundary_evidence": hardware,
        "checked_live_topic_freshness": freshness if isinstance(freshness, dict) else {},
        "checked_runtime_data_flow_evidence": data_flow if isinstance(data_flow, dict) else {},
        "checked_frame_link_evidence": frame_links if isinstance(frame_links, dict) else {},
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
            "validation_path": None,
            "report_mtime": None,
            "report_age_s": None,
            "max_age_s": age_limit,
            "runtime_contract": None,
            "runtime_evidence_ok": False,
            "simulation_only": None,
            "real_robot_motion": None,
            "cmd_vel_sent_to_hardware": None,
            "checked_real_motion_evidence": {},
            "checked_hardware_boundary_evidence": {},
            "checked_live_topic_freshness": {},
            "checked_runtime_data_flow_evidence": {},
            "checked_frame_link_evidence": {},
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
    from core.runtime_interface import runtime_contract_manifest

    return {
        "schema_version": 1,
        "source": "core.runtime_interface.runtime_contract_manifest",
        "manifest": runtime_contract_manifest(),
        "ts": time.time(),
    }


def _load_topic_contract() -> dict[str, Any]:
    from core.yaml_helpers import load_yaml

    path = (
        pathlib.Path(__file__).resolve().parents[3]
        / "config"
        / "topic_contract.yaml"
    )
    data = load_yaml(path, default={})
    if not isinstance(data, dict):
        data = {}
    return {
        "path": str(path),
        "exists": path.is_file(),
        "data": data,
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
    from core.runtime_interface import runtime_contract_manifest
    from gateway.services.runtime_status import build_navigation_status

    manifest = runtime_contract_manifest()
    runtime_frames = manifest.get("frames", {})
    if not isinstance(runtime_frames, dict):
        runtime_frames = {}
    runtime_links = manifest.get("frame_links", {})
    if not isinstance(runtime_links, dict):
        runtime_links = {}

    contract = _load_topic_contract()
    contract_data = contract["data"]
    tf_contract = contract_data.get("tf", {}) if isinstance(contract_data, dict) else {}
    if not isinstance(tf_contract, dict):
        tf_contract = {}

    nav_status = build_navigation_status(gw)
    frames = nav_status.get("frames", {})
    if not isinstance(frames, dict):
        frames = {}
    runtime_boundary = nav_status.get("runtime", {})
    if not isinstance(runtime_boundary, dict):
        runtime_boundary = {}

    with gw._state_lock:
        odom = dict(gw._odom) if isinstance(gw._odom, dict) else gw._odom
        mission = dict(gw._mission) if isinstance(gw._mission, dict) else gw._mission
        localization = (
            dict(gw._localization_status)
            if isinstance(gw._localization_status, dict)
            else gw._localization_status
        )
        path = list(gw._last_path or [])

    links = tf_contract.get("links")
    if not isinstance(links, dict) or not links:
        links = runtime_links

    return {
        "schema_version": 1,
        "contract": contract,
        "expected": {
            "map_frame": tf_contract.get("map_frame") or runtime_frames.get("map"),
            "odom_frame": tf_contract.get("odom_frame") or runtime_frames.get("odom"),
            "body_frame": tf_contract.get("body_frame") or runtime_frames.get("body"),
            "links": links,
        },
        "observed": {
            "odometry_frame_id": (
                odom.get("frame_id") if isinstance(odom, dict) else None
            ),
            "odometry_child_frame_id": (
                odom.get("child_frame_id") if isinstance(odom, dict) else None
            ),
            "mission_frame_id": (
                mission.get("frame_id") if isinstance(mission, dict) else None
            ),
            "mission_planning_frame_id": (
                mission.get("planning_frame_id")
                if isinstance(mission, dict)
                else None
            ),
            "mission_odom_frame_id": (
                mission.get("odom_frame_id") if isinstance(mission, dict) else None
            ),
            "path_frame_id": _first_path_frame(path),
            "path_point_count": len(path),
            "has_map_odom_tf": bool(getattr(gw, "_has_map_odom_tf", False)),
            "localization_backend": (
                localization.get("backend")
                if isinstance(localization, dict)
                else None
            ),
            "localization_state": (
                localization.get("state")
                if isinstance(localization, dict)
                else None
            ),
        },
        "navigation_frames": frames,
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
        "algorithm_benchmark": _snapshot_or_error(
            "algorithm_benchmark",
            lambda: build_algorithm_benchmark_latest_summary(),
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
        "routecheck_latest": build_routecheck_latest_summary(),
        "real_runtime_evidence": build_real_runtime_evidence_latest_summary(),
        "algorithm_benchmark_latest": build_algorithm_benchmark_latest_summary(),
    }


def _inspection_map_gate(body: Any) -> dict[str, Any] | None:
    map_dir = getattr(body, "map_dir", None)
    if not map_dir:
        from gateway.services.map_paths import (
            active_map_link,
            active_map_name,
            map_dir_for,
            nav_map_root,
        )

        root = nav_map_root()
        active_name = active_map_name(root)
        if active_name:
            candidate = map_dir_for(active_name, root)
            if candidate.is_dir():
                map_dir = str(candidate)
        if not map_dir:
            active_dir = active_map_link(root)
            if active_dir.is_dir():
                map_dir = str(active_dir)
    if not map_dir:
        return None
    from core.runtime_validation_gates import runtime_validation_gates
    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )

    gate = validate_saved_map_artifact_dir(
        pathlib.Path(str(map_dir)),
        require_tomogram=bool(getattr(body, "require_tomogram", False)),
        require_occupancy=bool(getattr(body, "require_occupancy", False)),
        expected_data_source=getattr(body, "expected_data_source", None),
        expected_source_profile=getattr(body, "expected_source_profile", None),
        expected_frame_id=getattr(body, "expected_frame_id", None),
    )
    gate["validation_gate"] = runtime_validation_gates()["saved_map_artifact_gate"]
    return gate


def _inspection_candidate(gw: Any, target: dict[str, Any], client_id: str) -> dict[str, Any]:
    from core.inspection_acceptance import goal_candidate_body_for_target
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
    if client_id and client_id != "unknown":
        payload["client_id"] = client_id
    try:
        body = GoalCandidateRequest.model_validate(payload)
        goal = construct_goal_from_request(
            body,
            gw=gw,
            default_source=body.source,
            default_target_type=body.target_type,
        )
        preview = (
            ControlCommandService(gw)
            .preview_navigation_plan(goal.preview_request(client_id=body.client_id))
        )
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
    return {
        "schema_version": 1,
        "ok": True,
        "status": (
            "preview_feasible"
            if bool(preview.get("feasible", False))
            else "preview_infeasible"
        ),
        "target": goal.target_payload(ts=ts),
        "preview": preview,
        "reasons": reasons,
        "error": None,
        "ts": ts,
    }


def build_product_field_check_gateway_summary(gw: Any, body: Any) -> dict[str, Any]:
    from core.gateway_runtime_acceptance import evaluate_gateway_runtime_acceptance
    from core.product_field_check import build_product_field_check

    gateway_acceptance = evaluate_gateway_runtime_acceptance(
        _gateway_acceptance_snapshots(gw),
        mode=getattr(body, "mode", "field"),
    )
    return build_product_field_check(
        gateway_acceptance,
        map_gate=_inspection_map_gate(body),
        algorithm_gate=build_algorithm_benchmark_latest_summary(),
    )


def build_inspection_acceptance_gateway_summary(gw: Any, body: Any) -> dict[str, Any]:
    from core.inspection_acceptance import (
        build_inspection_acceptance,
        inspection_targets_from_payload,
    )
    from gateway.services.telemetry_normalizers import build_locations_response

    field_check = build_product_field_check_gateway_summary(gw, body)
    locations = build_locations_response(_location_entries(gw))
    targets = inspection_targets_from_payload(
        locations,
        points=list(getattr(body, "points", None) or []),
        tag=getattr(body, "tag", None),
    )
    candidates = [
        _inspection_candidate(gw, target, str(getattr(body, "client_id", "unknown")))
        for target in targets
    ]
    return build_inspection_acceptance(
        field_check=field_check,
        targets=targets,
        candidates=candidates,
        locations=locations,
        gateway_url="gateway://local",
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
        summary="Read latest real S100P runtime evidence gate summary",
    )
    async def real_runtime_evidence_latest():
        return build_real_runtime_evidence_latest_summary()

    @app.get(
        "/api/v1/diagnostics/algorithm-benchmark/latest",
        response_model=AlgorithmBenchmarkLatestResponse,
        summary="Read latest read-only algorithm benchmark summary",
    )
    async def algorithm_benchmark_latest():
        return build_algorithm_benchmark_latest_summary()

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
        body: ProductFieldCheckRequest = Body(
            default_factory=ProductFieldCheckRequest
        ),
    ):
        return build_product_field_check_gateway_summary(gw, body)

    @app.post(
        "/api/v1/inspection/acceptance",
        response_model=InspectionAcceptanceResponse,
        summary="Run read-only inspection acceptance without publishing motion commands",
    )
    async def inspection_acceptance(
        body: InspectionAcceptanceRequest = Body(
            default_factory=InspectionAcceptanceRequest
        ),
    ):
        return build_inspection_acceptance_gateway_summary(
            gw,
            body,
        )

    @app.get(
        "/api/v1/diagnostic_pack",
        summary="Export diagnostic tarball",
        responses={
            200: {
                "content": {
                    "application/gzip": {
                        "schema": {"type": "string", "format": "binary"}
                    }
                }
            }
        },
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
                        "modules_ok": sum(
                            1 for v in modules_info.values() if "error" not in v
                        ),
                        "modules_fail": sum(
                            1 for v in modules_info.values() if "error" in v
                        ),
                        "count": len(modules_info),
                    },
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )

            try:
                git_out = subprocess.check_output(
                    ["git", "rev-parse", "HEAD"],
                    cwd=repo_root,
                    stderr=subprocess.DEVNULL,
                    timeout=3,
                ).decode().strip()
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

            cfg_path = repo_root / "config" / "robot_config.yaml"
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
