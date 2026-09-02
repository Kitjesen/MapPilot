#!/usr/bin/env python3
# ruff: noqa: D103
"""Ask mapd to validate one saved map's planning artifacts."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate one saved map through native mapd.",
    )
    parser.add_argument("map_id", help="Saved map ID")
    parser.add_argument("--require-octomap", action="store_true")
    parser.add_argument("--require-occupancy", action="store_true")
    parser.add_argument("--expected-data-source")
    parser.add_argument("--expected-source-profile")
    parser.add_argument("--expected-frame-id")
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    from diagnostics.field.field_check import validate_map
    from diagnostics.saved_map_display import format_saved_map_artifact_gate_payload

    payload = validate_map(
        args.map_id,
        require_octomap=args.require_octomap,
        require_occupancy=args.require_occupancy,
        expected_data_source=args.expected_data_source,
        expected_source_profile=args.expected_source_profile,
        expected_frame_id=args.expected_frame_id,
    )
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
