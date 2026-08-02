#!/usr/bin/env python3
"""Write a read-only PCT planner runtime preflight report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from nav.services.plan.global_planner.algorithm.pct.runtime.api import inspect_pct_runtime  # noqa: E402


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


def _pct_planner_runtime_report(inspection: dict[str, Any]) -> dict[str, Any]:
    selection = (
        dict(inspection.get("planner_runtime"))
        if isinstance(inspection.get("planner_runtime"), dict)
        else {}
    )
    runtime = str(selection.get("resolved") or selection.get("requested") or "")
    report: dict[str, Any] = {
        "runtime": runtime,
        "ok": inspection.get("ok") is True,
        "requested": str(selection.get("requested") or runtime),
        "supported": selection.get("supported") is True,
        "error": str(inspection.get("error") or selection.get("error") or ""),
    }
    optional_fields = {
        "call_mode": inspection.get("rust_optimizer_call_mode"),
        "searched": inspection.get("searched"),
        "required": inspection.get("required"),
        "missing": inspection.get("missing"),
        "recommended_build_command": inspection.get("recommended_build_command"),
    }
    report.update({key: value for key, value in optional_fields.items() if value not in (None, "")})
    return report


def build_report(*, repo_root: Path = ROOT) -> dict[str, Any]:
    inspection = dict(inspect_pct_runtime(repo_root))
    pct_planner_runtime = _pct_planner_runtime_report(inspection)
    runtime_ok = pct_planner_runtime["ok"] is True
    blocker = "PCT planner runtime unavailable"
    checks = {
        "pct_planner_runtime": _check(
            "pct_planner_runtime",
            ok=runtime_ok,
            blocker=blocker,
            evidence=pct_planner_runtime,
        )
    }
    build_command = str(pct_planner_runtime.get("recommended_build_command") or "")
    return {
        "schema_version": "lingtu.pct_runtime_preflight.v1",
        "ok": runtime_ok,
        "execution_mode": "pct_runtime_preflight",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "pct_planner_runtime": pct_planner_runtime,
        "pct_planner_runtime_ok": runtime_ok,
        "checks": checks,
        "blockers": [] if runtime_ok else [blocker],
        "recommended_setup_commands": [build_command] if build_command else [],
        "claim_boundary": (
            "pct_planner_runtime_ready"
            if runtime_ok
            else "environment_blocked_no_algorithm_claim"
        ),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/pct_runtime_preflight/report.json",
    )
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = build_report()
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") is True else 1 if args.strict else 0


if __name__ == "__main__":
    raise SystemExit(main())
