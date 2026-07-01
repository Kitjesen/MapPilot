#!/usr/bin/env python3
"""Run the MuJoCo navigation chain audit through the existing live gate."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
GATE = ROOT / "sim" / "scripts" / "mujoco_live_gate.py"


def _tail(text: str, limit: int = 4000) -> str:
    return text[-limit:]


def _preflight() -> list[str]:
    blockers: list[str] = []
    if not GATE.exists():
        blockers.append(f"missing gate script: {GATE}")
    try:
        import mujoco  # noqa: F401
    except Exception as exc:
        blockers.append(f"mujoco import failed: {exc}")
    return blockers


def _default_gate_command(args: argparse.Namespace, child_json: Path) -> list[str]:
    return [
        sys.executable,
        str(GATE),
        "--duration",
        str(args.duration),
        "--max-wall-time-s",
        str(args.max_wall_time_s),
        "--duration-clock",
        "wall",
        "--drive-source",
        "nav_cmd_vel",
        "--run-lingtu-inspection",
        "--inspection-goals",
        args.goal,
        "--inspection-min-checkpoints",
        "1",
        "--inspection-planner",
        args.planner,
        "--nav-data-source",
        args.nav_data_source,
        "--localization-backend",
        args.localization_backend,
        "--json-out",
        str(child_json),
    ]


def run_audit(args: argparse.Namespace) -> dict[str, Any]:
    blockers = _preflight()
    with tempfile.TemporaryDirectory(prefix="lingtu-mujoco-nav-audit-") as tmp:
        child_json = Path(tmp) / "mujoco_gate.json"
        command = _default_gate_command(args, child_json)
        if blockers:
            return {
                "ok": False,
                "stage": "preflight",
                "blockers": blockers,
                "command": command,
            }

        env = dict(os.environ)
        env["PYTHONPATH"] = (
            str(ROOT / "src")
            + os.pathsep
            + str(ROOT)
            + os.pathsep
            + env.get("PYTHONPATH", "")
        )
        result = subprocess.run(
            command,
            cwd=ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=max(args.max_wall_time_s + 30.0, args.duration + 30.0),
            check=False,
        )
        child_report: dict[str, Any] = {}
        if child_json.exists():
            try:
                child_report = json.loads(child_json.read_text(encoding="utf-8"))
            except json.JSONDecodeError as exc:
                child_report = {"json_error": str(exc)}

        child_ok = child_report.get("ok") is True if child_report else False
        return {
            "ok": result.returncode == 0 and child_ok,
            "stage": "mujoco_gate",
            "returncode": result.returncode,
            "command": command,
            "stdout_tail": _tail(result.stdout),
            "stderr_tail": _tail(result.stderr),
            "child_report": child_report,
        }


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate MuJoCo navigation flow: localization/map data -> goal -> "
            "global path -> waypoint/path follower -> nav cmd_vel."
        )
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--max-wall-time-s", type=float, default=60.0)
    parser.add_argument("--goal", default="2.0,0.0,0.0")
    parser.add_argument("--planner", choices=("astar", "pct"), default="astar")
    parser.add_argument(
        "--nav-data-source",
        choices=("fastlio2", "mujoco_ground_truth"),
        default="mujoco_ground_truth",
    )
    parser.add_argument(
        "--localization-backend",
        default="",
        help="No default; the old portable_lio backend was removed.",
    )
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    report = run_audit(args)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
