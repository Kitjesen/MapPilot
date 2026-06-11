#!/usr/bin/env python3
"""Guard DimOS runtime launches behind a green host preflight report."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

EXPECTED_SCHEMA_VERSION = "lingtu.server_sim_host_preflight.v1"
EXPECTED_EXECUTION_MODE = "host_preflight_only"
DIMOS_REQUIRED_GATE_SEQUENCE = (
    "gateway_runtime_acceptance",
    "routecheck_preflight",
    "blocked_route_replan_preflight",
    "navigation_replay_deviation",
    "large_terrain",
    "native_pct_mujoco",
    "dynamic_obstacle_local_planner",
    "fastlio2_dynamic_inspection",
    "moving_obstacle_sweep",
    "large_loop_closure",
    "gazebo_runtime",
    "saved_map_relocalize",
    "pct_saved_map_navigation",
)
DEFAULT_MAX_REPORT_AGE_S = 86_400.0


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, dict):
        return payload
    return {"value": payload}


def format_refusal(payload: dict[str, Any]) -> str:
    blocked = ", ".join(str(item) for item in payload.get("blocked_gates") or [])
    lines = [
        (
            "Host preflight is not green; refusing to launch runtime gates. "
            f"Blocked gates: {blocked or 'unknown'}"
        )
    ]
    setup_plan = payload.get("host_setup_plan") or {}
    failed_checks = setup_plan.get("failed_checks") or []
    if failed_checks:
        lines.append("Failed host checks:")
        for raw_item in failed_checks:
            item = raw_item if isinstance(raw_item, dict) else {}
            check = item.get("check") or "unknown"
            gates = ", ".join(str(gate) for gate in item.get("gates") or [])
            lines.append(f"- {check}: gates={gates or 'unknown'}")
            for blocker in item.get("blockers") or []:
                lines.append(f"  blocker: {blocker}")
            for command in item.get("diagnostic_commands") or []:
                lines.append(f"  diagnostic: {command}")
    return "\n".join(lines)


def _as_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _validation_blockers(
    path: Path,
    payload: dict[str, Any],
    *,
    max_age_s: float,
) -> list[str]:
    blockers: list[str] = []
    now = time.time()
    if payload.get("schema_version") != EXPECTED_SCHEMA_VERSION:
        blockers.append("schema_version is not lingtu.server_sim_host_preflight.v1")
    if payload.get("execution_mode") != EXPECTED_EXECUTION_MODE:
        blockers.append("execution_mode is not host_preflight_only")
    if payload.get("ok") is not True:
        blockers.append("ok is not true")
    if payload.get("blocked_gates"):
        blockers.append("blocked_gates is not empty")
    gate_sequence = tuple(str(item) for item in payload.get("required_gate_sequence") or ())
    if gate_sequence != DIMOS_REQUIRED_GATE_SEQUENCE:
        blockers.append("required_gate_sequence does not match DimOS required gates")

    file_age_s = max(0.0, now - path.stat().st_mtime)
    if file_age_s > max_age_s:
        blockers.append(f"host_preflight_file_age_s {file_age_s:.3f} > {max_age_s:.3f}")
    generated_at = _as_float(payload.get("generated_at"))
    if generated_at is None:
        blockers.append("generated_at missing or invalid")
    else:
        payload_age_s = max(0.0, now - generated_at)
        if payload_age_s > max_age_s:
            blockers.append(
                f"host_preflight_generated_age_s {payload_age_s:.3f} > {max_age_s:.3f}"
            )
        if generated_at - now > 300.0:
            blockers.append("generated_at is too far in the future")
    return blockers


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Refuse DimOS runtime gate execution unless host preflight is green.",
    )
    parser.add_argument("host_preflight_report", type=Path)
    parser.add_argument("--max-age-s", type=float, default=DEFAULT_MAX_REPORT_AGE_S)
    args = parser.parse_args(argv)

    payload = _load_json(args.host_preflight_report)
    blockers = _validation_blockers(
        args.host_preflight_report,
        payload,
        max_age_s=float(args.max_age_s),
    )
    if not blockers:
        return 0

    payload = dict(payload)
    payload["blocked_gates"] = list(payload.get("blocked_gates") or [])
    setup_plan = payload.setdefault("host_setup_plan", {})
    failed_checks = setup_plan.setdefault("failed_checks", [])
    failed_checks.append(
        {
            "check": "host_preflight_report_freshness",
            "gates": list(DIMOS_REQUIRED_GATE_SEQUENCE),
            "blockers": blockers,
            "diagnostic_commands": [
                (
                    "python3 sim/scripts/server_sim_closure.py --preset "
                    "dimos_benchmark --required-only --host-preflight "
                    "--json-out <fresh-host-preflight.json>"
                )
            ],
        }
    )
    print(format_refusal(payload), file=sys.stderr)
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
