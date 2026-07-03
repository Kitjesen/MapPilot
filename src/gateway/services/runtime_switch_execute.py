"""Runtime product-mode switch execution boundary for Gateway/App clients."""

from __future__ import annotations

import os
import subprocess
import tempfile
import time
import uuid
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from runtime.profiles.product_mode_contracts import PRODUCT_MODE_CONTRACTS
from runtime.profiles.resolver import canonical_profile_name

from gateway.services.runtime_switch_plan import build_runtime_switch_plan


RUNTIME_SWITCH_SCHEMA_VERSION = "lingtu.runtime_switch.v1"
_MAP_REQUIRED_PROFILES = frozenset({
    "teleop_avoid",
    "tracking",
    "nav",
    "inspection",
})


def _request_mapping(request: Any) -> dict[str, Any]:
    if request is None:
        return {}
    if isinstance(request, Mapping):
        return dict(request)
    if hasattr(request, "model_dump"):
        return dict(request.model_dump())
    return {
        key: getattr(request, key)
        for key in (
            "current_profile",
            "target_profile",
            "current_endpoint",
            "target_endpoint",
            "endpoint",
            "map_name",
            "relocalize",
            "initial_pose",
            "allow_restart",
            "client_id",
            "request_id",
        )
        if hasattr(request, key)
    }


def _clean_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _script_path() -> Path:
    override = _clean_text(os.environ.get("LINGTU_RUNTIME_SWITCH_SCRIPT"))
    if override:
        return Path(override).expanduser()
    return _repo_root() / "scripts" / "lingtu"


def _log_path(command_id: str) -> Path:
    root = Path(
        os.environ.get("LINGTU_RUNTIME_SWITCH_LOG_DIR")
        or Path(tempfile.gettempdir()) / "lingtu_runtime_switch"
    )
    root.mkdir(parents=True, exist_ok=True)
    return root / f"{command_id}.log"


def _build_command(raw: Mapping[str, Any], target_profile: str) -> list[str]:
    script = _script_path()
    endpoint = (
        _clean_text(raw.get("target_endpoint"))
        or _clean_text(raw.get("endpoint"))
        or "thunder_field"
    )
    command = [
        "bash",
        str(script),
        "mode",
        "switch",
        target_profile,
        "--endpoint",
        endpoint,
    ]
    current_profile = _clean_text(raw.get("current_profile"))
    if current_profile:
        command.extend(["--current", canonical_profile_name(current_profile)])
    map_name = _clean_text(raw.get("map_name"))
    if map_name:
        command.extend(["--map", map_name])
    if bool(raw.get("relocalize", True)):
        command.append("--relocalize")
    else:
        command.append("--no-relocalize")
    initial_pose = raw.get("initial_pose")
    if isinstance(initial_pose, list) and len(initial_pose) == 3:
        command.extend(["--initial-pose", *(str(item) for item in initial_pose)])
    return command


def _base_response(
    raw: Mapping[str, Any],
    *,
    plan: Mapping[str, Any],
    status: str,
    ok: bool,
    accepted: bool = False,
    blockers: list[str] | None = None,
    command: list[str] | None = None,
    command_id: str | None = None,
    pid: int | None = None,
    log_path: str | None = None,
    error: str | None = None,
) -> dict[str, Any]:
    target_profile = canonical_profile_name(str(raw.get("target_profile") or "nav"))
    lifecycle = "cold_restart"
    product_switch = plan.get("product_mode_switch")
    if isinstance(product_switch, Mapping):
        lifecycle = str(product_switch.get("required_lifecycle") or lifecycle)
    plan_inputs = plan.get("inputs") if isinstance(plan.get("inputs"), Mapping) else {}
    current_profile = _clean_text(raw.get("current_profile")) or _clean_text(
        plan_inputs.get("current_profile")
    )
    return {
        "schema_version": RUNTIME_SWITCH_SCHEMA_VERSION,
        "ts": time.time(),
        "ok": ok,
        "accepted": accepted,
        "status": status,
        "read_only": not accepted,
        "dry_run": not accepted,
        "motion": False,
        "lifecycle": lifecycle,
        "current_profile": current_profile,
        "target_profile": target_profile,
        "map_name": _clean_text(raw.get("map_name")),
        "relocalize": bool(raw.get("relocalize", True)),
        "plan": dict(plan),
        "command": list(command or []),
        "command_id": command_id,
        "pid": pid,
        "log_path": log_path,
        "blockers": list(blockers or []),
        "links": {
            "runtime_switch": "/api/v1/runtime/switch",
            "runtime_switch_plan": "/api/v1/runtime/switch-plan",
            "readiness": "/ready",
            "health": "/api/v1/health",
        },
        "error": error,
    }


def build_runtime_switch_response(request: Any) -> dict[str, Any]:
    """Validate and optionally launch the robot-side cold-restart mode switch."""

    raw = _request_mapping(request)
    target_profile = canonical_profile_name(str(raw.get("target_profile") or "nav"))
    raw["target_profile"] = target_profile
    raw["target_endpoint"] = (
        _clean_text(raw.get("target_endpoint"))
        or _clean_text(raw.get("endpoint"))
        or "thunder_field"
    )
    raw["endpoint"] = raw["target_endpoint"]

    plan = build_runtime_switch_plan(raw)
    blockers = list(plan.get("blockers") or [])
    if target_profile not in PRODUCT_MODE_CONTRACTS:
        blockers.append(f"unsupported product mode: {target_profile}")
    if target_profile in _MAP_REQUIRED_PROFILES and not _clean_text(raw.get("map_name")):
        blockers.append(f"{target_profile} requires map_name")
    if not bool(plan.get("ok")):
        blockers.extend(str(item) for item in plan.get("blockers") or [])
    if blockers:
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            blockers=blockers,
        )

    command = _build_command(raw, target_profile)
    if not bool(raw.get("allow_restart", False)):
        return _base_response(
            raw,
            plan=plan,
            status="planned",
            ok=True,
            command=command,
        )

    script = Path(command[1])
    if not script.exists():
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            command=command,
            blockers=[f"robot-side switch script not found: {script}"],
        )

    command_id = _clean_text(raw.get("request_id")) or uuid.uuid4().hex
    log_path = _log_path(command_id)
    env = os.environ.copy()
    env.setdefault("GW", "http://localhost:5050")
    try:
        with log_path.open("ab") as log:
            kwargs: dict[str, Any] = {
                "cwd": str(_repo_root()),
                "env": env,
                "stdout": log,
                "stderr": subprocess.STDOUT,
                "stdin": subprocess.DEVNULL,
                "close_fds": os.name != "nt",
            }
            if os.name != "nt":
                kwargs["start_new_session"] = True
            proc = subprocess.Popen(command, **kwargs)
    except Exception as exc:
        return _base_response(
            raw,
            plan=plan,
            status="error",
            ok=False,
            command=command,
            command_id=command_id,
            log_path=str(log_path),
            blockers=[f"failed to launch robot-side switch: {exc}"],
            error=str(exc),
        )

    return _base_response(
        raw,
        plan=plan,
        status="accepted",
        ok=True,
        accepted=True,
        command=command,
        command_id=command_id,
        pid=proc.pid,
        log_path=str(log_path),
    )
