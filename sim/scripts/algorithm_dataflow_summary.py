#!/usr/bin/env python3
"""Summarize one live navigation run as a compact dataflow verdict."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.diagnostics.dimos_runtime_dataflow import resolve_report_path
from runtime.diagnostics.dimos_runtime_dataflow import summarize_live_report as summarize_report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", help="Path to report.json, a run dir, or latest.txt")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--require-video-file", action="store_true")
    args = parser.parse_args(argv)

    report_path = resolve_report_path(args.report)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    summary = summarize_report(
        report,
        report_path=report_path,
        require_video_file=args.require_video_file,
    )
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
