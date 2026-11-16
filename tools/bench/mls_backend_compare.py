#!/usr/bin/env python3
"""Compare offline MLS backend evidence without running a planner.

The input is deliberately JSON-only so a C++ or Rust runner can produce the
same report without adding a Python dependency.  The accepted shape is shown
in ``docs/research/mls-backend-acceptance-20261115.md``.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


SCHEMA = "lingtu.mls_backend_compare.v1"
LAYERS = ("surface", "graph", "geometry", "performance")


class CompareError(ValueError):
    """The offline evidence manifest is not usable."""


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _number(value: Any, *, field: str, case: str, backend: str) -> float:
    if not _finite(value):
        raise CompareError(f"{case}/{backend}: {field} must be finite")
    return float(value)


def _optional_number(value: Any) -> float | None:
    return float(value) if _finite(value) else None


def _load(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CompareError(f"cannot read JSON input {path}: {exc}") from exc
    if not isinstance(value, dict) or value.get("schema") != SCHEMA:
        raise CompareError(f"input schema must be {SCHEMA}")
    if not isinstance(value.get("cases"), list) or not value["cases"]:
        raise CompareError("input must contain a non-empty cases list")
    return value


def _backend_evidence(case: str, name: str, value: Any) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CompareError(f"{case}/{name}: backend evidence must be an object")
    result = dict(value)
    for group in LAYERS:
        if group in result and not isinstance(result[group], dict):
            raise CompareError(f"{case}/{name}: {group} must be an object")
    return result


def _layer_status(name: str, evidence: dict[str, Any]) -> tuple[str, list[str]]:
    missing: list[str] = []
    if name == "surface":
        group = evidence.get("surface", {})
        for key in ("resolution_m", "surface_count"):
            if key not in group:
                missing.append(key)
        if missing:
            return "PARTIAL", missing
        return "PASS", []
    if name == "graph":
        group = evidence.get("graph", {})
        for key in ("node_count", "edge_count"):
            if key not in group:
                missing.append(key)
        if missing:
            return "PARTIAL", missing
        return "PASS", []
    if name == "geometry":
        group = evidence.get("geometry", {})
        if group.get("route_valid") is True and group.get("sweep_valid") is True:
            return "PASS", []
        if group.get("route_valid") is False:
            return "FAIL", [str(group.get("route_reason", "route_valid=false"))]
        return "PARTIAL", ["route_valid and sweep_valid are required"]
    group = evidence.get("performance", {})
    if not any(_finite(group.get(key)) for key in ("cold_prepare_ms", "warm_query_ms", "cold_total_ms")):
        return "PARTIAL", ["cold_prepare_ms or warm_query_ms is required"]
    return "PASS", []


def _case_report(case: dict[str, Any]) -> dict[str, Any]:
    case_id = case.get("id")
    if not isinstance(case_id, str) or not case_id:
        raise CompareError("every case needs a non-empty id")
    raw_backends = case.get("backends")
    if not isinstance(raw_backends, dict) or len(raw_backends) < 2:
        raise CompareError(f"{case_id}: at least two backends are required")

    backends: dict[str, dict[str, Any]] = {}
    layers: dict[str, dict[str, Any]] = {layer: {} for layer in LAYERS}
    gaps: list[str] = []
    for name, raw in raw_backends.items():
        evidence = _backend_evidence(case_id, str(name), raw)
        backends[str(name)] = evidence
        for layer in LAYERS:
            status, missing = _layer_status(layer, evidence)
            layers[layer][str(name)] = {"status": status}
            if missing:
                layers[layer][str(name)]["details"] = missing
                gaps.append(f"{name}/{layer}: {', '.join(missing)}")

    performance: dict[str, Any] = {}
    names = list(backends)
    if len(names) >= 2:
        base = backends[names[0]].get("performance", {})
        for other_name in names[1:]:
            other = backends[other_name].get("performance", {})
            for key in ("cold_prepare_ms", "warm_query_ms", "cold_total_ms"):
                lhs = _optional_number(base.get(key))
                rhs = _optional_number(other.get(key))
                if lhs is not None and rhs is not None and rhs > 0:
                    performance[f"{names[0]}_to_{other_name}_{key}_ratio"] = lhs / rhs

    return {
        "id": case_id,
        "backends": backends,
        "layers": layers,
        "performance_ratios": performance,
        "gaps": gaps,
    }


def compare(value: dict[str, Any]) -> dict[str, Any]:
    reports = [_case_report(case) for case in value["cases"]]
    gaps = [f"{report['id']}: {gap}" for report in reports for gap in report["gaps"]]
    evidence = value.get("evidence", {})
    if not isinstance(evidence, dict):
        raise CompareError("evidence must be an object when present")
    latest = evidence.get("latest_backend_commit")
    measured = evidence.get("measured_backend_commit")
    if latest and measured and latest != measured:
        gaps.append(f"measured backend {measured} differs from latest backend {latest}")
    if evidence.get("execution_validated") is not True:
        gaps.append("execution_validated is false or missing")

    counts = {layer: {"PASS": 0, "FAIL": 0, "PARTIAL": 0} for layer in LAYERS}
    for report in reports:
        for layer in LAYERS:
            for outcome in report["layers"][layer].values():
                counts[layer][outcome["status"]] += 1
    return {
        "schema": SCHEMA,
        "evidence": evidence,
        "cases": reports,
        "layer_counts": counts,
        "gaps": gaps,
        "claims": {
            "offline_route_geometry": all(
                report["layers"]["geometry"].get(name, {}).get("status") == "PASS"
                for report in reports
                for name in report["backends"]
            ),
            "execution": evidence.get("execution_validated") is True,
        },
    }


def _fmt(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.3g}"
    return str(value)


def markdown(report: dict[str, Any]) -> str:
    lines = [
        "# MLS backend comparison",
        "",
        "This report is generated from offline JSON evidence. It does not infer execution success from a route preview.",
        "",
        "## Layer summary",
        "",
        "| Layer | PASS | FAIL | PARTIAL |",
        "| --- | ---: | ---: | ---: |",
    ]
    for layer in LAYERS:
        counts = report["layer_counts"][layer]
        lines.append(f"| {layer} | {counts['PASS']} | {counts['FAIL']} | {counts['PARTIAL']} |")
    lines.extend(["", "## Cases", ""])
    for case in report["cases"]:
        lines.extend([f"### {case['id']}", "", "| Backend | Surface | Graph | Geometry | Performance |", "| --- | --- | --- | --- | --- |"])
        for name in case["backends"]:
            values = [case["layers"][layer][name]["status"] for layer in LAYERS]
            lines.append(f"| {name} | {values[0]} | {values[1]} | {values[2]} | {values[3]} |")
        if case["performance_ratios"]:
            lines.extend(["", "Performance ratios use the first backend as numerator.", ""])
            for key, value in case["performance_ratios"].items():
                lines.append(f"- `{key}` = {_fmt(value)}")
        if case["gaps"]:
            lines.extend(["", "Evidence gaps", ""])
            lines.extend(f"- {gap}" for gap in case["gaps"])
        lines.append("")
    lines.extend(["## Global evidence gaps", ""])
    if report["gaps"]:
        lines.extend(f"- {gap}" for gap in report["gaps"])
    else:
        lines.append("- None recorded in the manifest.")
    lines.extend(["", "## Claims", "", f"- Offline route geometry evidence: **{report['claims']['offline_route_geometry']}**", f"- Execution evidence: **{report['claims']['execution']}**", ""])
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="offline comparison JSON")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    args = parser.parse_args(argv)
    try:
        result = compare(_load(args.input))
    except CompareError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    encoded = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(encoded, encoding="utf-8")
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(markdown(result), encoding="utf-8")
    if not args.output_json and not args.output_md:
        print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
