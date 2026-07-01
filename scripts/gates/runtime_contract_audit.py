#!/usr/bin/env python3
"""CLI wrapper for the offline LingTu runtime contract audit."""

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
        description="Audit LingTu runtime interface/data-flow/frame contracts.",
    )
    parser.add_argument(
        "--topic-contract",
        type=Path,
        default=ROOT / "config" / "topic_contract.yaml",
    )
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--json", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    _ensure_import_path()
    from cli.runtime_audit import build_runtime_contract_audit

    args = parse_args(list(argv or sys.argv[1:]))
    payload = build_runtime_contract_audit(args.topic_contract)
    text = json.dumps(payload, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json or not args.json_out:
        print(text)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
