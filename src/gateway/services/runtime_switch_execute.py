"""Runtime product-mode switch execution boundary for Gateway/App clients."""

from __future__ import annotations

import os
import re
import subprocess
import tempfile
import time
import uuid
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from gateway.services.runtime_switch_plan import build_runtime_switch_plan
from runtime.msgs.geometry import Twist
from runtime.profiles.product_mode_contracts import PRODUCT_MODE_CONTRACTS
from runtime.profiles.resolver import canonical_profile_name

RUNTIME_SWITCH_SCHEMA_VERSION = "lingtu.runtime_switch.v1"
_MAP_REQUIRED_PROFILES = frozenset(
    {
        "teleop_avoid",
        "tracking",
        "nav",
        "inspection",
    }
)
_RUNTIME_SWITCH_INFLIGHT_TTL_S = 300.0


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
            "strategy",
            "execute",
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


def _resolve_active_map_name(gw: Any) -> str | None:
    if gw is None:
        return None
    session_map = _clean_text(getattr(gw, "_session_map", None))
    if session_map:
        return session_map
    active_map = getattr(gw, "_session_active_map_name", None)
    if callable(active_map):
        try:
            return _clean_text(active_map())
        except Exception:
            return None
    return None


def _log_path(command_id: str) -> Path:
    root = Path(
        os.environ.get("LINGTU_RUNTIME_SWITCH_LOG_DIR") or Path(tempfile.gettempdir()) / "lingtu_runtime_switch"
    )
    root.mkdir(parents=True, exist_ok=True)
    safe_id = re.sub(r"[^A-Za-z0-9_.-]+", "-", command_id).strip("-.")
    return root / f"{safe_id[:96] or 'switch'}.log"


def _requires_transient_unit() -> bool:
    if os.name == "nt":
        return False
    return any(_clean_text(os.environ.get(name)) for name in ("INVOCATION_ID", "SYSTEMD_EXEC_PID"))


def _transient_unit_name(command_id: str) -> str:
    configured = _clean_text(os.environ.get("LINGTU_RUNTIME_SWITCH_UNIT"))
    safe_base = (
        re.sub(
            r"[^A-Za-z0-9_.-]+",
            "-",
            configured or "lingtu-runtime-switch",
        ).strip("-.")
        or "lingtu-runtime-switch"
    )
    safe_id = re.sub(r"[^A-Za-z0-9_.-]+", "-", command_id).strip("-.")
    unit_name = f"{safe_base}-{safe_id}" if safe_id else safe_base
    return unit_name[:96].rstrip("-.") or "lingtu-runtime-switch"


def _active_cold_switch(gw: Any) -> dict[str, Any] | None:
    state = getattr(gw, "_runtime_switch_pending", None) if gw is not None else None
    if not isinstance(state, Mapping):
        return None
    try:
        age_s = time.monotonic() - float(state.get("started_monotonic"))
    except (TypeError, ValueError):
        age_s = _RUNTIME_SWITCH_INFLIGHT_TTL_S + 1.0
    if 0.0 <= age_s <= _RUNTIME_SWITCH_INFLIGHT_TTL_S:
        return dict(state)
    gw._runtime_switch_pending = None
    return None


def runtime_switch_pending(gw: Any) -> bool:
    """Return current cold-switch state while reaping expired in-memory state."""

    return _active_cold_switch(gw) is not None


def _begin_cold_switch(gw: Any, command_id: str) -> dict[str, Any] | None:
    active = _active_cold_switch(gw)
    if active is not None:
        return active
    if gw is not None:
        gw._runtime_switch_pending = {"command_id": command_id, "started_monotonic": time.monotonic()}
    return None


def _finish_cold_switch(gw: Any, command_id: str) -> None:
    state = getattr(gw, "_runtime_switch_pending", None) if gw is not None else None
    if not isinstance(state, Mapping) or state.get("command_id") == command_id:
        if gw is not None:
            gw._runtime_switch_pending = None


def _quiesce_for_cold_switch(gw: Any, command_id: str) -> list[str]:
    if gw is None:
        raise RuntimeError("gateway instance is required for a cold runtime switch")

    from gateway.services.native_control import endpoint_only_enabled
    from gateway.services.native_control import stop as native_stop

    reason = "runtime_switch_pending"
    wrote_native = native_stop(reason, request_id=command_id)
    if wrote_native:
        return ["native_navigation.stop_ack", "runtime_switch.pending"]
    if endpoint_only_enabled():
        raise RuntimeError("native navigation endpoint did not acknowledge stop")

    gw.cancel.publish(reason)
    gw.stop_cmd.publish(1)
    gw.cmd_vel.publish(Twist())
    return [
        "navigation.cancel",
        "safety.soft_stop",
        "cmd_vel.zero",
        "runtime_switch.pending",
    ]


def _systemd_run_command(
    command: list[str],
    *,
    command_id: str,
    log_path: Path,
    env: Mapping[str, str],
) -> list[str]:
    unit_name = _transient_unit_name(command_id)
    user = _clean_text(env.get("USER")) or _clean_text(os.environ.get("USER")) or "sunrise"
    group = _clean_text(env.get("GROUP")) or _clean_text(os.environ.get("GROUP")) or user
    home = _clean_text(env.get("HOME")) or ("/root" if user == "root" else f"/home/{user}")
    return [
        "sudo",
        "-n",
        "systemd-run",
        f"--unit={unit_name}",
        "--collect",
        "--no-block",
        "--service-type=exec",
        f"--working-directory={_repo_root()}",
        f"--property=User={user}",
        f"--property=Group={group}",
        f"--property=StandardOutput=append:{log_path}",
        f"--property=StandardError=append:{log_path}",
        "--",
        "/usr/bin/env",
        f"HOME={home}",
        f"USER={user}",
        f"LOGNAME={user}",
        f"GW={env.get('GW', 'http://localhost:5050')}",
        *command,
    ]


def _build_command(raw: Mapping[str, Any], target_profile: str) -> list[str]:
    script = _script_path()
    endpoint = _clean_text(raw.get("target_endpoint")) or _clean_text(raw.get("endpoint")) or "thunder_field"
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
    effects: list[str] | None = None,
    error: str | None = None,
) -> dict[str, Any]:
    target_profile = canonical_profile_name(str(raw.get("target_profile") or "nav"))
    strategy = str(raw.get("strategy") or "auto").strip().lower() or "auto"
    lifecycle = "cold_restart"
    product_switch = plan.get("product_mode_switch")
    if isinstance(product_switch, Mapping):
        lifecycle = str(product_switch.get("required_lifecycle") or lifecycle)
    plan_inputs = plan.get("inputs") if isinstance(plan.get("inputs"), Mapping) else {}
    current_profile = _clean_text(raw.get("current_profile")) or _clean_text(plan_inputs.get("current_profile"))
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
        "strategy": strategy,
        "current_profile": current_profile,
        "target_profile": target_profile,
        "map_name": _clean_text(raw.get("map_name")),
        "relocalize": bool(raw.get("relocalize", True)),
        "plan": dict(plan),
        "product_mode_switch": (dict(product_switch) if isinstance(product_switch, Mapping) else None),
        "effects": list(effects or []),
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


def _hot_switch_blockers(gw: Any) -> list[str]:
    blockers: list[str] = []
    if gw is None:
        blockers.append("gateway instance is required for hot switch")
        return blockers
    if str(getattr(gw, "_session_mode", "") or "").lower() != "navigating":
        blockers.append("hot switch requires an active navigating session")
    if bool(getattr(gw, "_session_pending", False)):
        blockers.append("session transition already pending")
    return blockers


def _execute_hot_switch(
    gw: Any,
    raw: Mapping[str, Any],
    *,
    plan: Mapping[str, Any],
    target_profile: str,
) -> dict[str, Any]:
    blockers = _hot_switch_blockers(gw)
    if blockers:
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            blockers=blockers,
        )

    reason = f"product_mode_switch:{target_profile}"
    contract = PRODUCT_MODE_CONTRACTS[target_profile]
    mux = getattr(gw, "_cmd_vel_mux", None)
    freeze = getattr(mux, "freeze", None)
    unfreeze = getattr(mux, "unfreeze", None)
    effects: list[str] = []
    if callable(freeze):
        freeze()
        effects.append("velocity_mux.freeze")
    try:
        gw.cancel.publish(reason)
        effects.append("navigation.cancel")
        gw.stop_cmd.publish(1)
        effects.append("safety.soft_stop")
        gw.cmd_vel.publish(Twist())
        effects.append("cmd_vel.zero")
        gw.mode_cmd.publish("autonomous")
        effects.append("mode.autonomous")
        with gw._state_lock:
            gw._mode = "autonomous"
            gw._runtime_product_profile = target_profile
            gw._runtime_product_mode = contract.product_mode
            gw._session_product_profile = target_profile
            gw._session_product_session = contract.product_session
            gw._runtime_switch_ts = time.time()
    finally:
        if callable(unfreeze):
            unfreeze()
            effects.append("velocity_mux.unfreeze")

    response = _base_response(
        raw,
        plan=plan,
        status="hot_switched",
        ok=True,
        accepted=True,
        effects=effects,
    )
    if hasattr(gw, "push_event"):
        gw.push_event({"type": "runtime_switch", "data": response})
    return response


def build_runtime_switch_response(
    gw_or_request: Any,
    request: Any | None = None,
) -> dict[str, Any]:
    """Validate and optionally execute a product-mode switch."""

    gw = None if request is None else gw_or_request
    raw = _request_mapping(gw_or_request if request is None else request)
    target_profile = canonical_profile_name(str(raw.get("target_profile") or "nav"))
    raw["target_profile"] = target_profile
    raw["target_endpoint"] = (
        _clean_text(raw.get("target_endpoint")) or _clean_text(raw.get("endpoint")) or "thunder_field"
    )
    raw["endpoint"] = raw["target_endpoint"]
    raw["strategy"] = str(raw.get("strategy") or "auto").strip().lower() or "auto"
    raw["execute"] = bool(raw.get("execute", False))
    active_map = _resolve_active_map_name(gw)
    if target_profile in _MAP_REQUIRED_PROFILES and not _clean_text(raw.get("map_name")):
        if active_map:
            raw["map_name"] = active_map

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
    product_switch = plan.get("product_mode_switch")
    lifecycle = ""
    if isinstance(product_switch, Mapping):
        lifecycle = str(product_switch.get("required_lifecycle") or "")
    strategy = str(raw.get("strategy") or "auto").lower()
    execute = bool(raw.get("execute", False))
    allow_restart = bool(raw.get("allow_restart", False))

    if strategy == "warm":
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            command=command,
            blockers=["warm switch is not implemented; use hot or cold"],
        )
    if execute and strategy in {"auto", "hot"} and lifecycle == "hot_switch":
        return _execute_hot_switch(
            gw,
            raw,
            plan=plan,
            target_profile=target_profile,
        )
    if strategy == "hot" and lifecycle != "hot_switch":
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            command=command,
            blockers=[f"hot switch is not supported for {target_profile}"],
        )
    if execute and not allow_restart:
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            command=command,
            blockers=["execution requires hot switch support or allow_restart=true"],
        )

    if not allow_restart:
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
    active_switch = _begin_cold_switch(gw, command_id)
    if active_switch is not None:
        active_command_id = _clean_text(active_switch.get("command_id")) or "unknown"
        return _base_response(
            raw,
            plan=plan,
            status="rejected",
            ok=False,
            command=command,
            command_id=command_id,
            blockers=[f"runtime switch already in progress: {active_command_id}"],
        )

    log_path = _log_path(command_id)
    env = os.environ.copy()
    env.setdefault("GW", "http://localhost:5050")
    proc_pid: int | None = None
    effects: list[str] = []
    try:
        effects = _quiesce_for_cold_switch(gw, command_id)
        if _requires_transient_unit():
            # systemd opens StandardOutput before ExecStart.  On the S100P image,
            # reusing a caller-created file fails with status 209/STDOUT, while an
            # absent path is created correctly by PID 1.
            log_path.unlink(missing_ok=True)
            sudo_check = subprocess.run(
                ["sudo", "-n", "true"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            if sudo_check.returncode != 0:
                detail = str(sudo_check.stderr or sudo_check.stdout or "").strip()
                _finish_cold_switch(gw, command_id)
                return _base_response(
                    raw,
                    plan=plan,
                    status="rejected",
                    ok=False,
                    command=command,
                    command_id=command_id,
                    log_path=str(log_path),
                    blockers=["non-interactive sudo is required for an independent runtime switch"],
                    error=detail or "sudo -n preflight failed",
                )
            transient_command = _systemd_run_command(
                command,
                command_id=command_id,
                log_path=log_path,
                env=env,
            )
            launched = subprocess.run(
                transient_command,
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
            )
            if launched.returncode != 0:
                detail = str(launched.stderr or launched.stdout or "").strip()
                _finish_cold_switch(gw, command_id)
                return _base_response(
                    raw,
                    plan=plan,
                    status="rejected",
                    ok=False,
                    command=command,
                    command_id=command_id,
                    log_path=str(log_path),
                    blockers=["failed to create independent systemd runtime-switch unit"],
                    error=detail or f"systemd-run exited {launched.returncode}",
                )
        else:
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
                proc_pid = proc.pid
    except Exception as exc:
        _finish_cold_switch(gw, command_id)
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
        pid=proc_pid,
        log_path=str(log_path),
        effects=effects,
    )
