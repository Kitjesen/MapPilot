#!/usr/bin/env python3
"""Write a read-only PCT native runtime preflight report."""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from global_planning.pct_planner_runnable.runtime import inspect_pct_runtime


def _python_tag() -> str:
    return f"py{sys.version_info.major}{sys.version_info.minor}"


def _check(
    name: str,
    *,
    ok: bool,
    blocker: str = "",
    evidence: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "name": name,
        "ok": bool(ok),
        "blocker": "" if ok else blocker,
        "evidence": evidence or {},
    }


def _reason_code(
    *,
    host_platform_supported: bool,
    python_abi_ok: bool,
    missing: list[str],
    shared_missing: list[str],
    runtime_ok: bool,
) -> str:
    if not host_platform_supported:
        return "unsupported_host_platform"
    if not python_abi_ok:
        return "python_abi_mismatch"
    if missing:
        return "missing_extension_modules"
    if shared_missing:
        return "missing_shared_libraries"
    if not runtime_ok:
        return "runtime_unavailable"
    return "ready"


def _runtime_fingerprint(
    runtime: dict[str, Any],
    *,
    platform_system: str,
    python_tag: str,
    host_platform_supported: bool,
    python_abi_ok: bool,
    missing: list[str],
    shared_missing: list[str],
    runtime_ok: bool,
) -> dict[str, Any]:
    searched = [str(item) for item in runtime.get("searched") or []]
    lib_dir = str(runtime.get("lib_dir") or "")
    selected_index = searched.index(lib_dir) if lib_dir in searched else -1
    return {
        "reason_code": _reason_code(
            host_platform_supported=host_platform_supported,
            python_abi_ok=python_abi_ok,
            missing=missing,
            shared_missing=shared_missing,
            runtime_ok=runtime_ok,
        ),
        "python": {
            "executable": sys.executable,
            "version": platform.python_version(),
            "tag": python_tag,
            "known_good_tag": runtime.get("known_good_python_tag"),
            "abi_matches_known_good": python_abi_ok,
        },
        "host": {
            "platform_system": platform_system,
            "os_name": runtime.get("os_name"),
            "machine": runtime.get("machine") or platform.machine().lower(),
            "canonical_arch": runtime.get("canonical_arch"),
            "native_binary_format": runtime.get("native_binary_format"),
            "platform_supported": host_platform_supported,
        },
        "lib_discovery": {
            "selected_lib_dir": lib_dir,
            "selected_lib_dir_exists": Path(lib_dir).is_dir() if lib_dir else False,
            "selected_candidate_index": selected_index,
            "searched_count": len(searched),
            "searched": searched,
        },
        "extension_modules": {
            "required": list(runtime.get("required") or []),
            "missing": missing,
        },
        "shared_libraries": {
            "missing": shared_missing,
        },
        "ros2_environment": {
            "ROS_DISTRO": os.environ.get("ROS_DISTRO", ""),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", ""),
        },
    }


def build_report(*, repo_root: Path = ROOT, machine: str | None = None) -> dict[str, Any]:
    runtime = inspect_pct_runtime(repo_root, machine=machine)
    platform_system = str(runtime.get("platform_system") or platform.system().lower())
    python_tag = str(runtime.get("python_tag") or _python_tag())
    host_platform_supported = runtime.get("host_platform_supported") is True
    python_abi_ok = runtime.get("python_abi_matches_known_good") is True
    missing = [str(item) for item in runtime.get("missing") or []]
    shared_missing = [str(item) for item in runtime.get("shared_missing") or []]
    runtime_ok = runtime.get("ok") is True
    runtime_fingerprint = _runtime_fingerprint(
        runtime,
        platform_system=platform_system,
        python_tag=python_tag,
        host_platform_supported=host_platform_supported,
        python_abi_ok=python_abi_ok,
        missing=missing,
        shared_missing=shared_missing,
        runtime_ok=runtime_ok,
    )

    checks = {
        "host_platform": _check(
            "host_platform",
            ok=host_platform_supported,
            blocker=str(
                runtime.get("host_platform_blocker")
                or "PCT native runtime requires Linux ELF-compatible host"
            ),
            evidence={
                "platform_system": platform_system,
                "native_binary_format": runtime.get("native_binary_format"),
            },
        ),
        "python_abi": _check(
            "python_abi",
            ok=python_abi_ok,
            blocker=(
                "PCT native runtime requires CPython 3.10 ABI; "
                f"current {python_tag}"
            ),
            evidence={
                "python_tag": python_tag,
                "known_good_python_tag": runtime.get("known_good_python_tag"),
            },
        ),
        "extension_modules": _check(
            "extension_modules",
            ok=not missing,
            blocker="missing PCT extension modules: " + ", ".join(missing),
            evidence={
                "lib_dir": runtime.get("lib_dir"),
                "missing": missing,
                "required": list(runtime.get("required") or []),
            },
        ),
        "shared_libraries": _check(
            "shared_libraries",
            ok=not shared_missing,
            blocker="missing PCT shared libraries: " + ", ".join(shared_missing),
            evidence={
                "lib_dir": runtime.get("lib_dir"),
                "shared_missing": shared_missing,
            },
        ),
    }
    blockers = [
        str(check["blocker"])
        for check in checks.values()
        if check.get("ok") is not True and str(check.get("blocker") or "")
    ]
    runtime_error = str(runtime.get("error") or "")
    if runtime_error and runtime_error not in blockers:
        blockers.append(runtime_error)
    build_command = str(runtime.get("recommended_build_command") or "")
    return {
        "schema_version": "lingtu.pct_runtime_preflight.v1",
        "ok": runtime_ok and not blockers,
        "execution_mode": "pct_runtime_preflight",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "current_host": {
            "platform_system": platform_system,
            "machine": runtime.get("machine") or platform.machine().lower(),
            "canonical_arch": runtime.get("canonical_arch"),
            "python_tag": python_tag,
        },
        "runtime_fingerprint": runtime_fingerprint,
        "native_runtime": runtime,
        "checks": checks,
        "blockers": blockers,
        "recommended_setup_commands": [
            command
            for command in (
                build_command,
                "bash scripts/deploy/setup_server_ros_pct.sh",
                "PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py "
                "--preset dimos_benchmark --required-only --host-preflight "
                "--json-out artifacts/server_sim_closure/host_preflight_dimos_benchmark.json",
            )
            if command
        ],
        "claim_boundary": (
            "pct_native_runtime_ready"
            if runtime_ok and not blockers
            else "environment_blocked_no_algorithm_claim"
        ),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--machine", default=None, help="Override machine/arch for diagnostics.")
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/pct_runtime_preflight/report.json",
    )
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = build_report(machine=args.machine)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") is True else 1 if args.strict else 0


if __name__ == "__main__":
    raise SystemExit(main())
