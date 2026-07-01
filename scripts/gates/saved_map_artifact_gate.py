#!/usr/bin/env python3
"""Validate saved map artifact provenance for map.pcd/tomogram/occupancy."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _ensure_import_path() -> None:
    for candidate in (ROOT / "src", ROOT):
        path = str(candidate)
        if path not in sys.path:
            sys.path.insert(0, path)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate saved-map metadata.json and artifact checksums.",
    )
    parser.add_argument("map_dir", type=Path, help="Directory containing metadata.json")
    parser.add_argument("--require-tomogram", action="store_true")
    parser.add_argument("--require-occupancy", action="store_true")
    parser.add_argument("--expected-data-source")
    parser.add_argument("--expected-source-profile")
    parser.add_argument("--expected-frame-id")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    _ensure_import_path()

    from core.same_source_map_artifacts import (
        validate_saved_map_artifact_dir,
    )
    from core.runtime_validation_gates import runtime_validation_gates
    from cli.runtime_display import format_saved_map_artifact_gate_payload

    payload = validate_saved_map_artifact_dir(
        args.map_dir,
        require_tomogram=args.require_tomogram,
        require_occupancy=args.require_occupancy,
        expected_data_source=args.expected_data_source,
        expected_source_profile=args.expected_source_profile,
        expected_frame_id=args.expected_frame_id,
    )
    payload["validation_gate"] = runtime_validation_gates()["saved_map_artifact_gate"]
    text = json.dumps(payload, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    else:
        print(format_saved_map_artifact_gate_payload(payload))
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
