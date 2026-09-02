#!/usr/bin/env python3
"""Render the current DimOS-style diagnostics gap report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for path in (SRC, ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES  # noqa: E402
from sim.diagnostics import summary as sim_diagnostics  # noqa: E402
from sim.diagnostics.dataflow_report import build_runtime_dataflow_from_summary  # noqa: E402
from sim.diagnostics.gap_report import build_dimos_gap_report  # noqa: E402


def _load_summary(path: Path | None, max_age_s: float | None) -> tuple[dict[str, Any], str]:
    if path is None:
        return (
            sim_diagnostics.summarize(
                report_overrides={},
                required=set(DIMOS_BENCHMARK_REQUIRED_GATES),
                max_report_age_s=max_age_s,
                include_optional=False,
            ),
            "current_reports",
        )
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return payload, str(path)


def _markdown(report: dict[str, Any]) -> str:
    readiness = report["lingtu_readiness"]
    counts = report["gap_counts"]
    lines = [
        "# DimOS Gap Report",
        "",
        f"Source: `{report['source']}`",
        f"Passed: `{counts['passed']}/{counts['required']}`",
        f"Ready: `{str(readiness['ok']).lower()}`",
        "",
        "| Gate | Status | Blocker |",
        "| --- | --- | --- |",
    ]
    for row in report["gap_matrix"]:
        blocker = "; ".join(str(item) for item in row.get("blockers") or []).replace("|", "/")
        lines.append(f"| `{row['gate']}` | {row['status']} | {blocker or '-'} |")
    return "\n".join(lines) + "\n"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, help="Existing diagnostics summary; defaults to current reports.")
    parser.add_argument("--max-report-age-s", type=float)
    parser.add_argument("--include-dataflow", action="store_true")
    parser.add_argument("--format", choices=("json", "markdown"), default="json")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/sim_diagnostics/dimos_gap_report.json",
        help="Output path, or '-' for stdout.",
    )
    return parser


def main() -> int:
    args = _parser().parse_args()
    summary, source = _load_summary(args.summary, args.max_report_age_s)
    report = build_dimos_gap_report(
        summary,
        source=source,
        gate_metadata={
            spec.name: {
                "description": spec.description,
                "expected_report_path": sim_diagnostics._expected_report_path(spec),
                "host_requirements": list(spec.host_requirements),
            }
            for spec in sim_diagnostics.GATES
        },
        runtime_dataflow=(build_runtime_dataflow_from_summary(summary, root=ROOT) if args.include_dataflow else None),
    )
    text = _markdown(report) if args.format == "markdown" else json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if str(args.output) == "-":
        print(text, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    return 0 if report["lingtu_readiness"]["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
