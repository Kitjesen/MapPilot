"""Explain resolved Product parameters and the current field run."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

from lingtu.control import ProductControl
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.product_lock import resolve_current_run_path
from lingtu.product_switch import session_explanation
from lingtu.runtime_parameters import resolve_runtime_parameters


_MODE_KEYS = (
    "LINGTU_PRODUCT_SESSION_ID",
    "LINGTU_RUN_PLAN_FINGERPRINT",
    "LINGTU_NAV_CONTROL_MODE",
    "LINGTU_SLAM_MODE",
    "LINGTU_SLAM_MAP",
    "LINGTU_EXPLORE_ROUTE",
)


def explain_product(
    product: str,
    *,
    env: str,
    env_config: Mapping[str, Any] | None = None,
    session_overrides: Mapping[str, Any] | None = None,
    process_environment: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Resolve one Product once and expose parameter values with provenance."""

    plan = ProductControl(
        env=env,
        env_config=env_config,
        process_env=process_environment,
    ).resolve(product)
    parameters = resolve_runtime_parameters(
        parameter_profile=plan.parameter_profile,
        env_overrides=plan.parameter_overrides,
        session_overrides=session_overrides,
    )
    return {
        "product": plan.product,
        "env": plan.env,
        "fingerprint": plan.fingerprint,
        "run_plan_schema": plan.schema_version,
        "contracts": list(plan.contract_ids),
        "processes": [process.as_dict() for process in plan.processes],
        "parameters": parameters.explanation(),
    }


def explain_status(
    *,
    state_dir: str | Path | None = None,
    process_environment: Mapping[str, str] | None = None,
    systemd_show: Callable[[str], Mapping[str, str]] | None = None,
) -> dict[str, Any]:
    """Explain committed identity and the actual transient systemd session."""

    environment = process_environment if process_environment is not None else os.environ
    current_path = resolve_current_run_path(state_dir, environment=environment)
    persistent_overrides = _persistent_overrides()
    if not current_path.is_file():
        return {
            "state": "standby",
            "reason": "no_current_run",
            "current_run": str(current_path),
            "persistent_overrides": persistent_overrides,
        }

    current = _load_object(current_path, label="current run")
    if current.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError("current run record has unsupported schema")
    plan_path = Path(_required_text(current, "run_plan_path")).expanduser()
    plan = RunPlan.load(plan_path)
    committed_fingerprint = _required_text(current, "fingerprint")
    if plan.fingerprint != committed_fingerprint:
        raise RuntimeError("current run fingerprint does not match its plan")

    configured_session_root = environment.get(
        "LINGTU_SESSION_ROOT",
        "/run/lingtu",
    )
    session = session_explanation(
        plan,
        runtime_root=configured_session_root,
    )
    session_file = Path(str(session["session_file"]))
    file_environment = (
        _read_environment_file(session_file) if session_file.is_file() else {}
    )
    show = systemd_show or _systemd_show
    process_rows: list[dict[str, Any]] = []
    sessions: set[str] = set()
    actual_parameters: dict[str, str] = {}
    for process in plan.processes:
        unit = process.target
        try:
            observed = dict(show(unit))
            show_error = None
        except Exception as exc:  # status must report a broken unit, not hide all peers
            observed = {}
            show_error = str(exc)
        unit_environment = _parse_systemd_environment(observed.get("Environment", ""))
        actual_environment = {**file_environment, **unit_environment}
        session_id = actual_environment.get("LINGTU_PRODUCT_SESSION_ID")
        if session_id:
            sessions.add(session_id)
        for key, value in actual_environment.items():
            if key.startswith("LINGTU_NAV_SEGMENT_"):
                actual_parameters[key] = value
        process_rows.append(
            {
                "name": process.name,
                "unit": unit,
                "lifecycle": process.lifecycle,
                "active_state": observed.get("ActiveState", "unknown"),
                "sub_state": observed.get("SubState", "unknown"),
                "main_pid": observed.get("MainPID", "0"),
                "mode": {
                    key: actual_environment[key]
                    for key in _MODE_KEYS
                    if key in actual_environment
                },
                "dropin_paths": _split_systemd_words(observed.get("DropInPaths", "")),
                "session_file": str(session_file),
                "session_file_present": session_file.is_file(),
                "error": show_error,
            }
        )

    resolved_parameters = resolve_runtime_parameters(
        parameter_profile=plan.parameter_profile,
        env_overrides=plan.parameter_overrides,
    )
    session_consistent = len(sessions) == 1
    return {
        "state": "active" if session_consistent else "invalid_session",
        "product": plan.product,
        "env": plan.env,
        "fingerprint": plan.fingerprint,
        "plan": str(plan_path),
        "session_id": next(iter(sessions)) if session_consistent else None,
        "session_consistent": session_consistent,
        "session": session,
        "processes": process_rows,
        "parameters": resolved_parameters.explanation(),
        "actual_parameter_environment": dict(sorted(actual_parameters.items())),
        "persistent_overrides": persistent_overrides,
    }


def _systemd_show(unit: str) -> Mapping[str, str]:
    result = subprocess.run(
        [
            "systemctl",
            "show",
            unit,
            "--no-pager",
            "--property=ActiveState",
            "--property=SubState",
            "--property=MainPID",
            "--property=DropInPaths",
            "--property=Environment",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"systemctl show failed for {unit}")
    return _parse_properties(result.stdout)


def _parse_properties(value: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in value.splitlines():
        key, separator, raw = line.partition("=")
        if separator and key:
            result[key] = raw
    return result


def _parse_systemd_environment(value: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in _split_systemd_words(value):
        key, separator, raw = token.partition("=")
        if separator and re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            result[key] = raw
    return result


def _split_systemd_words(value: str) -> list[str]:
    import shlex

    try:
        return shlex.split(value)
    except ValueError:
        return [value] if value else []


def _read_environment_file(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, raw = line.partition("=")
        if not separator or not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            continue
        value = raw.strip()
        if len(value) >= 2 and value[0] == value[-1] == '"':
            value = value[1:-1]
        result[key] = value.replace('\\"', '"').replace("\\\\", "\\")
    return result


def _persistent_overrides(
    root: Path = Path("/etc/systemd/system"),
) -> list[str]:
    if not root.is_dir():
        return []
    return sorted(
        str(path)
        for path in root.glob("lingtu*.service.d/*.conf")
        if path.is_file()
    )


def _load_object(path: Path, *, label: str) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"could not read {label}: {path}") from exc
    if not isinstance(payload, Mapping):
        raise RuntimeError(f"{label} must be an object: {path}")
    return payload


def _required_text(payload: Mapping[str, Any], field: str) -> str:
    value = payload.get(field)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f"current run requires {field}")
    return value.strip()


def _session_overrides(values: Sequence[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for value in values:
        key, separator, raw = value.partition("=")
        if not separator or not key.strip() or not raw.strip():
            raise ValueError(f"invalid parameter override: {value!r}")
        result[key.strip()] = raw.strip()
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    inspect_parser = subparsers.add_parser("inspect")
    inspect_parser.add_argument("product")
    inspect_parser.add_argument("--env", choices=("real", "sim"), default="real")
    inspect_parser.add_argument("--backend")
    inspect_parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    inspect_parser.add_argument("--explain", action="store_true")
    status_parser = subparsers.add_parser("status")
    status_parser.add_argument("--state-dir", type=Path)
    status_parser.add_argument("--explain", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.action == "inspect":
            env_config = {"backend": args.backend} if args.backend else None
            payload = explain_product(
                args.product,
                env=args.env,
                env_config=env_config,
                session_overrides=_session_overrides(args.set),
            )
        else:
            payload = explain_status(state_dir=args.state_dir)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["explain_product", "explain_status", "main"]
