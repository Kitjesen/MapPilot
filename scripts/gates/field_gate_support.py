"""Shared stdlib helpers for stateful field acceptance gates.

Product and process lifecycle is deliberately delegated to ``lingtu.control``.
The helpers in this module only collect read-only evidence, submit an explicit
navigation goal, and persist gate artifacts.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[2]
GATES_DIR = Path(__file__).resolve().parent
TERMINAL_NAV_STATES = {"SUCCESS", "FAILED", "STUCK", "CANCELLED"}


class GateError(RuntimeError):
    """A fail-closed gate error suitable for operator output."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def python_environment() -> dict[str, str]:
    environment = dict(os.environ)
    src = str(ROOT / "src")
    existing = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = src + (os.pathsep + existing if existing else "")
    return environment


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def safe_map_dir(maps_root: Path, raw_name: str) -> tuple[str, Path]:
    name = str(raw_name or "").strip()
    candidate = Path(name)
    if (
        not name
        or candidate.is_absolute()
        or len(candidate.parts) != 1
        or name in {".", ".."}
        or "/" in name
        or "\\" in name
    ):
        raise GateError(f"unsafe map name: {raw_name}")
    root = maps_root.expanduser().resolve()
    map_dir = (root / name).resolve()
    try:
        map_dir.relative_to(root)
    except ValueError as exc:
        raise GateError(f"map escapes maps root: {raw_name}") from exc
    if not (map_dir / "map.pcd").is_file():
        raise GateError(f"map.pcd not found: {map_dir / 'map.pcd'}")
    return name, map_dir


def request_json(
    gateway_url: str,
    path: str,
    *,
    method: str = "GET",
    payload: Mapping[str, Any] | None = None,
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    body = None
    headers: dict[str, str] = {}
    if payload is not None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(
        gateway_url.rstrip("/") + path,
        data=body,
        headers=headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            raw = response.read().decode("utf-8")
    except (OSError, urllib.error.URLError) as exc:
        raise GateError(f"Gateway request failed: {method} {path}: {exc}") from exc
    try:
        decoded = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise GateError(f"Gateway returned invalid JSON: {method} {path}") from exc
    if not isinstance(decoded, dict):
        raise GateError(f"Gateway returned non-object JSON: {method} {path}")
    return decoded


def request_json_evidence(
    gateway_url: str,
    path: str,
    *,
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    try:
        return request_json(gateway_url, path, timeout_s=timeout_s)
    except GateError as exc:
        return {"ok": False, "error": str(exc), "path": path}


def _capture_command(path: Path, command: Sequence[str]) -> None:
    try:
        completed = subprocess.run(
            list(command),
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        text = completed.stdout
        if completed.stderr:
            text += ("\n" if text else "") + completed.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        text = f"{type(exc).__name__}: {exc}\n"
    path.write_text(text, encoding="utf-8")


def capture_phase(gateway_url: str, phase: str, directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    endpoints = {
        "state.json": "/api/v1/state",
        "localization.json": "/api/v1/localization/status",
        "navigation.json": "/api/v1/navigation/status",
        "health.json": "/api/v1/health",
    }
    for filename, endpoint in endpoints.items():
        write_json(directory / filename, request_json_evidence(gateway_url, endpoint))
    _capture_command(
        directory / "services.txt",
        (
            "systemctl",
            "show",
            "-p",
            "Id",
            "-p",
            "ActiveState",
            "-p",
            "SubState",
            "-p",
            "NRestarts",
            "lingtu-livox-dds.service",
            "lingtu-slam-dds.service",
            "lingtu-nav-dds.service",
            "lingtu.service",
        ),
    )
    _capture_command(directory / "resources.txt", ("ps", "-eo", "pid,comm,%cpu,%mem,rss,vsz,args"))
    (directory / "phase.txt").write_text(phase + "\n", encoding="utf-8")


def active_command_source(navigation: Mapping[str, Any]) -> str:
    control = navigation.get("control")
    control = control if isinstance(control, Mapping) else {}
    source: Any = control.get("active_cmd_source") or control.get("active_source") or "none"
    if isinstance(source, Mapping):
        source = source.get("name") or source.get("source") or source.get("owner") or "none"
    return str(source or "none")


def require_no_active_command_source(navigation: Mapping[str, Any], phase: str) -> None:
    source = active_command_source(navigation).strip().lower()
    if source in {"", "none", "null", "-"}:
        return
    if source in {"path_follower", "recovery"}:
        control = navigation.get("control")
        control = control if isinstance(control, Mapping) else {}
        mux = control.get("cmd_vel_mux")
        mux = mux if isinstance(mux, Mapping) else {}
        command = mux.get("last_driver_cmd_vel")
        command = command if isinstance(command, Mapping) else {}
        values: list[float] = []
        for group in ("linear", "angular"):
            vector = command.get(group)
            vector = vector if isinstance(vector, Mapping) else {}
            for axis in ("x", "y", "z"):
                try:
                    values.append(float(vector.get(axis, 0.0)))
                except (TypeError, ValueError):
                    values.append(0.0)
        if sum(value * value for value in values) <= 1e-6:
            return
    raise GateError(f"{phase} refused while active_cmd_source={source}")


def plan_preview(
    gateway_url: str,
    goal: Mapping[str, float],
    directory: Path,
    *,
    timeout_s: float = 30.0,
) -> dict[str, Any]:
    directory.mkdir(parents=True, exist_ok=True)
    navigation = request_json(gateway_url, "/api/v1/navigation/status", timeout_s=5)
    write_json(directory / "navigation.pre_plan.json", navigation)
    require_no_active_command_source(navigation, "plan preview")
    readiness = navigation.get("readiness")
    readiness = readiness if isinstance(readiness, Mapping) else {}
    can_accept = readiness.get("can_accept_goal", navigation.get("can_accept_goal", False))
    if can_accept is not True:
        blockers = readiness.get("blockers") or []
        raise GateError(f"navigation is not ready for plan preview: blockers={blockers}")
    plan = request_json(
        gateway_url,
        "/api/v1/navigation/plan",
        method="POST",
        payload=goal,
        timeout_s=timeout_s,
    )
    write_json(directory / "plan.json", plan)
    path_safety = plan.get("path_safety")
    path_safety = path_safety if isinstance(path_safety, Mapping) else {}
    rejected = plan.get("rejected_plans")
    rejected = rejected if isinstance(rejected, list) else []
    summary = {
        "schema_version": 1,
        "phase": "motion_smoke",
        "non_motion": True,
        "navigation_state": navigation.get("state"),
        "can_accept_goal": can_accept is True,
        "active_cmd_source_before": active_command_source(navigation),
        "feasible": plan.get("feasible") is True,
        "count": plan.get("count"),
        "planner": plan.get("planner"),
        "selected_planner": plan.get("selected_planner") or plan.get("planner"),
        "plan_safety_policy": plan.get("plan_safety_policy"),
        "path_safety_ok": path_safety.get("ok"),
        "fallback_reason": plan.get("fallback_reason") or "",
        "rejected_plan_count": len(rejected),
        "reasons": plan.get("reasons") if isinstance(plan.get("reasons"), list) else [],
    }
    write_json(directory / "plan_summary.json", summary)
    if plan.get("feasible") is not True:
        raise GateError(f"navigation plan preview is not feasible: {summary['reasons']}")
    return plan


def validate_saved_map_plan(
    gateway_url: str,
    map_name: str,
    goal: Mapping[str, float],
    output: Path,
) -> dict[str, Any]:
    report = request_json(
        gateway_url,
        f"/api/v1/maps/{map_name}/validate_plan",
        method="POST",
        payload=goal,
        timeout_s=30,
    )
    write_json(output, report)
    ok = report.get("map_plan_ok", report.get("success", report.get("ok", False)))
    if ok is not True:
        gate = report.get("no_motion_gate")
        gate = gate if isinstance(gate, Mapping) else {}
        raise GateError(f"saved-map plan validation failed: {gate.get('blockers') or []}")
    return report


def strict_goal_ack(payload: Mapping[str, Any]) -> bool:
    command = payload.get("command")
    return bool(
        payload.get("ok") is True
        and payload.get("accepted") is True
        and isinstance(command, Mapping)
        and command.get("accepted") is True
    )


def poll_navigation_terminal(
    gateway_url: str,
    directory: Path,
    *,
    timeout_s: float,
    poll_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    samples = directory / "navigation.samples.jsonl"
    samples.parent.mkdir(parents=True, exist_ok=True)
    with samples.open("w", encoding="utf-8", newline="\n") as stream:
        while True:
            navigation = request_json(gateway_url, "/api/v1/navigation/status", timeout_s=5)
            stream.write(json.dumps(navigation, ensure_ascii=False) + "\n")
            stream.flush()
            write_json(directory / "navigation.latest.json", navigation)
            state = str(navigation.get("state") or "unknown").upper()
            if state in TERMINAL_NAV_STATES:
                if state != "SUCCESS":
                    raise GateError(
                        f"navigation ended in {state}: {navigation.get('failure_reason') or ''}"
                    )
                return navigation
            if time.monotonic() >= deadline:
                raise GateError(
                    f"navigation did not reach a terminal state before {timeout_s:g}s "
                    f"(last_state={state})"
                )
            time.sleep(poll_s)


def run_command(
    command: Sequence[str],
    *,
    stdout_path: Path,
    stderr_path: Path | None = None,
    timeout_s: float = 180.0,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(
        list(command),
        cwd=ROOT,
        env=python_environment(),
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    stdout_path.write_text(completed.stdout, encoding="utf-8")
    (stderr_path or stdout_path.with_suffix(stdout_path.suffix + ".stderr")).write_text(
        completed.stderr,
        encoding="utf-8",
    )
    if check and completed.returncode != 0:
        raise GateError(
            f"command failed ({completed.returncode}): {' '.join(command)}; "
            f"see {stderr_path or stdout_path.with_suffix(stdout_path.suffix + '.stderr')}"
        )
    return completed


def run_product_control(
    arguments: Sequence[str],
    output_path: Path,
    *,
    timeout_s: float = 240.0,
) -> dict[str, Any]:
    command = [sys.executable, "-m", "lingtu.control", *arguments, "--json"]
    completed = run_command(
        command,
        stdout_path=output_path,
        stderr_path=output_path.with_suffix(output_path.suffix + ".stderr"),
        timeout_s=timeout_s,
        check=False,
    )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError:
        payload = {"ok": False, "error": "ProductControl returned invalid JSON"}
    if not isinstance(payload, dict):
        payload = {"ok": False, "error": "ProductControl returned non-object JSON"}
    if completed.returncode != 0:
        raise GateError(
            f"ProductControl failed ({completed.returncode}): {' '.join(arguments)}: "
            f"{payload.get('error') or payload.get('status') or 'unknown error'}"
        )
    return payload


def start_evidence_collector(
    *,
    gateway_url: str,
    duration_s: float,
    min_motion_m: float,
    expected_subscribers: Sequence[str],
    output_path: Path,
    validate: bool = True,
) -> tuple[subprocess.Popen[str], Any, Any]:
    command = [
        sys.executable,
        str(GATES_DIR / "real_runtime_evidence_collect.py"),
        "--gateway-url",
        gateway_url,
        "--duration-sec",
        str(duration_s),
        "--min-motion-m",
        str(min_motion_m),
        "--json-out",
        str(output_path),
        "--json",
    ]
    if not validate:
        command.append("--no-validate")
    for subscriber in expected_subscribers:
        command.extend(("--expected-command-subscriber", subscriber))
    stdout = output_path.with_suffix(".stdout").open("w", encoding="utf-8")
    stderr = output_path.with_suffix(".stderr").open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        env=python_environment(),
        stdout=stdout,
        stderr=stderr,
        text=True,
    )
    return process, stdout, stderr
