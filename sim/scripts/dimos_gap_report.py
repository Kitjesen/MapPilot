#!/usr/bin/env python3
"""Build a DimOS-style gap report from LingTu simulation closure evidence."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from runtime.dimos_gap import build_dimos_gap_report
from runtime.dimos_runtime_dataflow import build_runtime_dataflow_from_summary
from sim.scripts import server_sim_closure

DEFAULT_HOST_PREFLIGHT_MAX_AGE_S = 86_400.0


def _load_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} does not contain a JSON object")
    return data


def _float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _runtime_dataflow_from_summary(summary: dict[str, Any]) -> dict[str, Any]:
    return build_runtime_dataflow_from_summary(summary, root=ROOT)


def _gate_metadata() -> dict[str, dict[str, Any]]:
    return {
        spec.name: {
            "description": spec.description,
            "command": spec.command,
            "expected_report_path": server_sim_closure._expected_report_path(spec),
            "host_requirements": list(spec.host_requirements),
        }
        for spec in server_sim_closure.GATES
    }


def _summary_from_reports(max_report_age_s: float | None) -> tuple[dict[str, Any], str]:
    summary = server_sim_closure.summarize(
        report_overrides={},
        required=set(DIMOS_BENCHMARK_REQUIRED_GATES),
        max_report_age_s=max_report_age_s,
        include_optional=False,
    )
    return summary, "current_reports"


def _summary_from_path(path: Path) -> tuple[dict[str, Any], str]:
    return _load_json(path), str(path)


def _summary_file_freshness(
    path: Path,
    *,
    max_report_age_s: float | None,
) -> dict[str, Any]:
    checked_at = time.time()
    mtime = path.stat().st_mtime
    age_s = max(0.0, checked_at - mtime)
    threshold = None if max_report_age_s is None else float(max_report_age_s)
    fresh = True if threshold is None else age_s <= threshold
    return {
        "checked": True,
        "source": "summary_file_mtime",
        "path": str(path),
        "mtime": mtime,
        "age_s": round(age_s, 3),
        "max_age_s": threshold,
        "fresh": fresh,
        "stale": not fresh,
        "blocker": "" if fresh else "algorithm benchmark summary file is stale",
    }


def _summary_payload_freshness(
    payload: dict[str, Any],
    *,
    source: str,
    max_report_age_s: float | None,
) -> dict[str, Any]:
    checked_at = time.time()
    threshold = None if max_report_age_s is None else float(max_report_age_s)
    generated_at = _float_or_none(payload.get("generated_at"))
    blockers: list[str] = []
    age_s: float | None = None
    if generated_at is None:
        blockers.append("summary generated_at missing or invalid")
    else:
        age_s = max(0.0, checked_at - generated_at)
        if threshold is not None and age_s > threshold:
            blockers.append(f"summary_generated_age_s {age_s:.3f} > {threshold:.3f}")
        if generated_at - checked_at > 300.0:
            blockers.append("summary generated_at is too far in the future")
    return {
        "checked": True,
        "source": source,
        "generated_at": generated_at,
        "age_s": None if age_s is None else round(age_s, 3),
        "max_age_s": threshold,
        "fresh": not blockers,
        "stale": bool(blockers),
        "blocker": "" if not blockers else "; ".join(blockers),
        "blockers": blockers,
    }


def _host_preflight_file_freshness(
    path: Path,
    payload: dict[str, Any],
    *,
    max_report_age_s: float | None,
) -> dict[str, Any]:
    checked_at = time.time()
    threshold = (
        DEFAULT_HOST_PREFLIGHT_MAX_AGE_S
        if max_report_age_s is None
        else float(max_report_age_s)
    )
    mtime = path.stat().st_mtime
    file_age_s = max(0.0, checked_at - mtime)
    generated_at = _float_or_none(payload.get("generated_at"))
    blockers: list[str] = []
    if file_age_s > threshold:
        blockers.append(f"host_preflight_file_age_s {file_age_s:.3f} > {threshold:.3f}")
    if generated_at is None:
        blockers.append("generated_at missing or invalid")
    else:
        generated_age_s = max(0.0, checked_at - generated_at)
        if generated_age_s > threshold:
            blockers.append(
                f"host_preflight_generated_age_s {generated_age_s:.3f} > {threshold:.3f}"
            )
        if generated_at - checked_at > 300.0:
            blockers.append("generated_at is too far in the future")
    return {
        "checked": True,
        "source": "host_preflight_file",
        "path": str(path),
        "mtime": mtime,
        "file_age_s": round(file_age_s, 3),
        "generated_at": generated_at,
        "generated_age_s": (
            None
            if generated_at is None
            else round(max(0.0, checked_at - generated_at), 3)
        ),
        "max_age_s": threshold,
        "fresh": not blockers,
        "stale": bool(blockers),
        "blockers": blockers,
    }


def _load_host_preflight_report(
    path: Path,
    *,
    max_report_age_s: float | None,
) -> dict[str, Any]:
    payload = _load_json(path)
    payload = dict(payload)
    payload["report_contract_checked"] = True
    payload["report_freshness"] = _host_preflight_file_freshness(
        path,
        payload,
        max_report_age_s=max_report_age_s,
    )
    return payload


def build_gap_report(
    *,
    summary_path: Path | None = None,
    max_report_age_s: float | None = None,
    host_preflight: bool = False,
    host_preflight_report: Path | None = None,
    include_dataflow: bool = False,
) -> dict[str, Any]:
    """Return a DimOS-style gap matrix from a closure summary or current reports."""
    if summary_path is None:
        summary, source = _summary_from_reports(max_report_age_s)
        summary_freshness = _summary_payload_freshness(
            summary,
            source=source,
            max_report_age_s=max_report_age_s,
        )
    else:
        summary, source = _summary_from_path(summary_path)
        summary_freshness = _summary_file_freshness(
            summary_path,
            max_report_age_s=max_report_age_s,
        )
    preflight_source = ""
    if host_preflight:
        preflight = server_sim_closure.host_preflight(
            required=set(DIMOS_BENCHMARK_REQUIRED_GATES)
        )
        preflight_source = "argument"
    elif host_preflight_report is not None:
        preflight = _load_host_preflight_report(
            host_preflight_report,
            max_report_age_s=max_report_age_s,
        )
        preflight_source = f"file:{host_preflight_report}"
    else:
        preflight = None
    runtime_dataflow = _runtime_dataflow_from_summary(summary) if include_dataflow else None
    return build_dimos_gap_report(
        summary,
        source=source,
        gate_metadata=_gate_metadata(),
        host_preflight=preflight,
        host_preflight_source=preflight_source,
        summary_freshness=summary_freshness,
        runtime_dataflow=runtime_dataflow,
        include_summary=True,
    )


def _markdown_table(report: dict[str, Any]) -> str:
    host = report.get("host_preflight") or {}
    setup_plan = report.get("host_setup_plan") or {}
    execution_plan = report.get("execution_plan") or {}
    host_status = ""
    if host.get("checked"):
        host_status = "pass" if host.get("ok") else "blocked"
    lines = [
        "# DimOS Gap Report",
        "",
        f"- Source: `{report['source']}`",
        *(
            [
                f"- Host preflight: `{host_status}`",
                f"- Current host: `{host.get('current_host', {})}`",
            ]
            if host.get("checked")
            else []
        ),
        f"- Claim allowed: `{str(report['lingtu_readiness']['claim_allowed']).lower()}`",
        f"- Passed/required: `{report['gap_counts']['passed']}/{report['gap_counts']['required']}`",
        f"- Missing or failed: `{', '.join(report['lingtu_readiness']['missing_or_failed']) or 'none'}`",
        "",
        "| Priority | Gate | Status | Host | Category | Dataflow blocker | DimOS surface | DimOS-style requirement | Next action |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for row in report["gap_matrix"]:
        requirement = str(row["dimos_requirement"]).replace("|", "/")
        surface = str(row["dimos_surface"]).replace("|", "/")
        action = str(row["recommended_action"]).replace("|", "/")
        dataflow_blocker = str(row.get("runtime_dataflow_blocker") or "").replace(
            "|", "/"
        )
        row_host = row.get("host_preflight") or {}
        row_host_status = (
            "ok"
            if row_host.get("ok") is True
            else "blocked"
            if row_host.get("ok") is False
            else "not checked"
        )
        lines.append(
            f"| {row['priority']} | `{row['gate']}` | {row['status']} | {row_host_status} | "
            f"{row['primary_category']} | `{dataflow_blocker}` | {surface} | {requirement} | {action} |"
        )
    if setup_plan.get("checked") and not setup_plan.get("ok"):
        lines.extend(
            [
                "",
                "## Host Setup Blockers",
                "",
                "| Check | Gates | Recommended action | Blockers |",
                "| --- | --- | --- | --- |",
            ]
        )
        for check in setup_plan.get("failed_checks") or []:
            gates = ", ".join(f"`{gate}`" for gate in check.get("gates") or [])
            blockers = "; ".join(str(item) for item in check.get("blockers") or [])
            action = str(check.get("recommended_action") or "").replace("|", "/")
            lines.append(
                f"| `{check.get('check')}` | {gates} | {action} | {blockers} |"
            )
    if execution_plan.get("phases"):
        lines.extend(
            [
                "",
                "## Execution Plan",
                "",
                "| Phase | Status | Order | Purpose | Command count | Stop condition |",
                "| --- | --- | --- | --- | ---: | --- |",
            ]
        )
        for phase in execution_plan.get("phases") or []:
            purpose = str(phase.get("purpose") or "").replace("|", "/")
            stop = str(phase.get("stop_condition") or "").replace("|", "/")
            order = str(phase.get("order") or "").replace("|", "/")
            lines.append(
                f"| `{phase.get('id')}` | {phase.get('status')} | {order or 'default'} | {purpose} | "
                f"{len(phase.get('commands') or [])} | {stop} |"
            )
    dataflow = report.get("runtime_dataflow") or {}
    if dataflow.get("checked"):
        lines.extend(
            [
                "",
                "## Runtime Dataflow",
                "",
                "| Gate | Checked | OK | First blocker | Failed edges | Source report |",
                "| --- | --- | --- | --- | --- | --- |",
            ]
        )
        for gate_name, payload in (dataflow.get("gates") or {}).items():
            row = payload if isinstance(payload, dict) else {}
            failed_edges = ", ".join(
                f"`{edge.get('id')}`"
                for edge in row.get("flow") or []
                if isinstance(edge, dict) and edge.get("ok") is not True
            )
            source = str(row.get("source_report") or "").replace("|", "/")
            reason = str(row.get("primary_blocker") or row.get("reason") or "").replace("|", "/")
            lines.append(
                f"| `{gate_name}` | {row.get('checked', True) is not False} | "
                f"{row.get('ok') is True} | `{reason}` | {failed_edges or 'none'} | `{source}` |"
            )
        cross_gate_chains = dataflow.get("cross_gate_chains") or {}
        if cross_gate_chains:
            lines.extend(
                [
                    "",
                    "### Cross-Gate Chains",
                    "",
                    "| Chain | OK | Selected Fast-LIO gate | Same-run proven | Blockers |",
                    "| --- | --- | --- | --- | --- |",
                ]
            )
            for chain_name, payload in cross_gate_chains.items():
                row = payload if isinstance(payload, dict) else {}
                blockers = "; ".join(str(item) for item in row.get("blockers") or [])
                lines.append(
                    f"| `{chain_name}` | {row.get('ok') is True} | "
                    f"`{row.get('selected_fastlio_gate') or ''}` | "
                    f"{row.get('same_run_proven') is True} | {blockers or 'none'} |"
                )
    trace = report.get("pipeline_trace") or {}
    if trace.get("primary_chain"):
        lines.extend(
            [
                "",
                "## Pipeline Trace",
                "",
                "| Step | Runtime gate | Role | Code |",
                "| --- | --- | --- | --- |",
            ]
        )
        for step in trace.get("primary_chain") or []:
            role = str(step.get("role") or "").replace("|", "/")
            code = ", ".join(f"`{item}`" for item in step.get("code") or [])
            lines.append(
                f"| `{step.get('id')}` | `{step.get('runtime_evidence_gate')}` | "
                f"{role} | {code} |"
            )
    return "\n".join(lines) + "\n"


def _phase_command_items(phase: dict[str, Any]) -> list[dict[str, Any]]:
    commands: list[dict[str, Any]] = []
    for item in phase.get("commands") or []:
        if isinstance(item, dict):
            command = str(item.get("command") or "")
            if command:
                copied = dict(item)
                copied["command"] = command
                commands.append(copied)
        else:
            command = str(item)
            if command:
                commands.append({"command": command})
    return commands


def _phase_commands(phase: dict[str, Any]) -> list[str]:
    return [str(item["command"]) for item in _phase_command_items(phase)]


def _shell_plan(report: dict[str, Any]) -> str:
    plan = report.get("execution_plan") or {}
    readiness = report.get("lingtu_readiness") or {}
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "",
        "# Generated from LingTu DimOS gap report.",
        f"# Source: {report.get('source')}",
        f"# Claim allowed: {str(report['lingtu_readiness']['claim_allowed']).lower()}",
        f"# Readiness ok: {str(readiness.get('ok') is True).lower()}",
        "# Runtime dataflow checked: "
        f"{str(readiness.get('runtime_dataflow_checked') is True).lower()}",
        "# Runtime dataflow complete: "
        f"{str(readiness.get('runtime_dataflow_complete') is True).lower()}",
        "# Runtime dataflow ok: "
        f"{str(readiness.get('runtime_dataflow_ok') is True).lower()}",
        f"# Passed/required: {report['gap_counts']['passed']}/{report['gap_counts']['required']}",
        f"# ok_to_run_missing: {str(plan.get('ok_to_run_missing')).lower()}",
        "",
    ]
    for failure in readiness.get("runtime_dataflow_gate_failures") or []:
        lines.append(f"# Runtime dataflow gate failure: {failure}")
    for failure in readiness.get("cross_gate_failures") or []:
        lines.append(f"# Cross-gate failure: {failure}")
    dataflow = report.get("runtime_dataflow") or {}
    for chain_name, raw_chain in (dataflow.get("cross_gate_chains") or {}).items():
        chain = raw_chain if isinstance(raw_chain, dict) else {}
        lines.append(
            f"# Chain {chain_name}: ok={str(chain.get('ok') is True).lower()} "
            f"same_run_proven={str(chain.get('same_run_proven') is True).lower()} "
            f"claim_boundary={chain.get('claim_boundary') or ''}"
        )
    if len(lines) > 10:
        lines.append("")
    for phase in plan.get("phases") or []:
        phase_id = str(phase.get("id") or "")
        status = str(phase.get("status") or "")
        lines.append(f"# Phase: {phase_id} [{status}]")
        if phase.get("order"):
            lines.append(f"# Order: {phase.get('order')}")
        lines.append(f"# Purpose: {phase.get('purpose') or ''}")
        lines.append(f"# Stop: {phase.get('stop_condition') or ''}")
        command_items = _phase_command_items(phase)
        if not command_items:
            lines.append("# No commands emitted for this phase.")
        for item in command_items:
            check = str(item.get("check") or "")
            gate = str(item.get("gate") or "")
            gates = [
                str(value)
                for value in item.get("gates") or []
                if str(value)
            ]
            priority = str(item.get("priority") or "")
            expected = str(item.get("expected_report_path") or "")
            recommended_action = str(item.get("recommended_action") or "")
            host_preflight_ok = item.get("host_preflight_ok")
            failed_checks = [
                str(value)
                for value in item.get("host_failed_checks") or []
                if str(value)
            ]
            host_blockers = [
                str(value)
                for value in item.get("host_preflight_blockers") or []
                if str(value)
            ]
            dependency_blockers = [
                str(value)
                for value in item.get("dependency_blockers") or []
                if str(value)
            ]
            dependency_status = item.get("dependency_blocker_status") or {}
            runtime_blocker = str(item.get("runtime_dataflow_blocker") or "")
            runtime_failed_edges = [
                str(value)
                for value in item.get("runtime_dataflow_failed_edges") or []
                if str(value)
            ]
            if gate:
                suffix = f" priority={priority}" if priority else ""
                lines.append(f"# Gate: {gate}{suffix}")
            if check:
                lines.append(f"# Host setup check: {check}")
            if gates:
                lines.append(f"# Host setup gates: {', '.join(gates)}")
            if recommended_action:
                lines.append(f"# Recommended action: {recommended_action}")
            if expected:
                lines.append(f"# Expected report: {expected}")
            if host_preflight_ok is not None:
                lines.append(
                    f"# Host preflight ok: {str(host_preflight_ok).lower()}"
                )
            if failed_checks:
                lines.append(f"# Host failed checks: {', '.join(failed_checks)}")
            for blocker in host_blockers:
                lines.append(f"# Host blocker: {blocker}")
            if dependency_blockers:
                lines.append(
                    "# Dependency blockers: " + ", ".join(dependency_blockers)
                )
                for blocker in dependency_blockers:
                    if blocker in dependency_status:
                        lines.append(
                            f"# Dependency blocker status: {blocker}="
                            f"{dependency_status[blocker]}"
                        )
            if runtime_blocker:
                lines.append(f"# Runtime dataflow blocker: {runtime_blocker}")
            if runtime_failed_edges:
                lines.append(
                    "# Runtime dataflow failed edges: "
                    + ", ".join(runtime_failed_edges)
                )
            command = str(item["command"])
            should_comment_command = (
                phase_id
                in {
                    "blocked_gate_commands",
                    "dependency_blocked_gate_commands",
                    "preflight_required_gate_commands",
                }
                or (status == "blocked" and phase_id != "host_setup")
            )
            if should_comment_command:
                lines.append(f"# BLOCKED: {command}")
            else:
                lines.append(command)
        lines.append("")
    return "\n".join(lines)


def _write_output(path: Path, text: str) -> None:
    if str(path) == "-":
        print(text, end="")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Existing server_sim_closure summary to compare; defaults to current reports.",
    )
    parser.add_argument(
        "--max-report-age-s",
        type=float,
        default=server_sim_closure.DEFAULT_REQUIRED_MAX_REPORT_AGE_S,
        help="Freshness threshold used when summarizing current reports.",
    )
    parser.add_argument(
        "--format",
        choices=("json", "markdown", "shell"),
        default="json",
        help="Output format.",
    )
    parser.add_argument(
        "--host-preflight",
        action="store_true",
        help=(
            "Include read-only host suitability checks for the DimOS gate set. "
            "This does not launch gate commands."
        ),
    )
    parser.add_argument(
        "--host-preflight-report",
        type=Path,
        default=None,
        help=(
            "Attach an existing server_sim_closure --host-preflight JSON report "
            "instead of running host checks in this process. --host-preflight "
            "takes precedence when both are provided."
        ),
    )
    parser.add_argument(
        "--include-dataflow",
        action="store_true",
        help=(
            "Inspect existing runtime gate reports and attach Fast-LIO/global path/"
            "local path/cmd_vel/checkpoint dataflow blockers when report shapes allow it."
        ),
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/dimos_gap_report.json",
        help="Output path, or '-' for stdout.",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    include_dataflow = args.include_dataflow or args.format == "shell"
    report = build_gap_report(
        summary_path=args.summary,
        max_report_age_s=args.max_report_age_s,
        host_preflight=args.host_preflight,
        host_preflight_report=args.host_preflight_report,
        include_dataflow=include_dataflow,
    )
    if args.format == "markdown":
        _write_output(args.json_out, _markdown_table(report))
    elif args.format == "shell":
        _write_output(args.json_out, _shell_plan(report))
    else:
        _write_output(args.json_out, json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    return 0 if report["lingtu_readiness"]["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
