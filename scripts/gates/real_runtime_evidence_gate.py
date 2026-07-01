#!/usr/bin/env python3
"""Validate a Thunder field runtime evidence report.

This script is intentionally read-only. It does not publish goals, cmd_vel, or
any robot-control topic. A separate robot-side collector can produce the JSON
report; this gate decides whether the report proves the real runtime contract:
Thunder field data source, hardware command boundary, frame chain, and resolved
data-flow stages.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]


def _ensure_import_path() -> None:
    for candidate in (ROOT / "src", ROOT):
        path = str(candidate)
        if path not in sys.path:
            sys.path.insert(0, path)


def _load_report(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("runtime evidence report must be a JSON object")
    return data


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a Thunder field runtime evidence report JSON.",
    )
    parser.add_argument("report", type=Path, help="Path to the collected report JSON")
    _ensure_import_path()
    from runtime.runtime_evidence import REAL_RUNTIME_CONTRACT

    parser.add_argument("--expected-contract", default=REAL_RUNTIME_CONTRACT)
    parser.add_argument("--allow-missing-paths", action="store_true")
    parser.add_argument("--allow-missing-command", action="store_true")
    parser.add_argument("--allow-missing-frame-links", action="store_true")
    parser.add_argument("--allow-missing-data-flow", action="store_true")
    parser.add_argument("--allow-missing-hardware-boundary", action="store_true")
    parser.add_argument("--json-out", type=Path, help="Optional path for validation JSON")
    args = parser.parse_args(argv)
    if args.expected_contract != REAL_RUNTIME_CONTRACT:
        parser.error(
            f"real runtime evidence only supports expected contract "
            f"{REAL_RUNTIME_CONTRACT}"
        )
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    _ensure_import_path()

    from runtime.runtime_evidence import (
        real_runtime_evidence_payload,
        validate_real_runtime_evidence,
    )
    from runtime.runtime_validation_gates import runtime_validation_gates

    report = _load_report(args.report)
    result = validate_real_runtime_evidence(
        report,
        args.expected_contract,
        require_paths=not args.allow_missing_paths,
        require_command=not args.allow_missing_command,
        require_frame_links=not args.allow_missing_frame_links,
        require_data_flow=not args.allow_missing_data_flow,
        require_hardware_boundary=not args.allow_missing_hardware_boundary,
    )
    payload = real_runtime_evidence_payload(
        result,
        args.expected_contract,
        report=report,
        paths_required=not args.allow_missing_paths,
        command_required=not args.allow_missing_command,
        frame_links_required=not args.allow_missing_frame_links,
        data_flow_required=not args.allow_missing_data_flow,
        hardware_boundary_required=not args.allow_missing_hardware_boundary,
    )
    payload["validation_gate"] = runtime_validation_gates()["real_runtime_evidence"]
    text = json.dumps(payload, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if result.ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
