#!/usr/bin/env python3
"""Run the native/Gateway system acceptance sequence from Python."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Any

from field_gate_support import (
    GATES_DIR,
    ROOT,
    GateError,
    capture_phase,
    run_command,
    run_product_control,
    safe_map_dir,
    utc_now,
    validate_saved_map_plan,
    write_json,
)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run native runtime, saved-map, plan, and optional motion acceptance.",
    )
    parser.add_argument("--env", choices=("real", "sim"), default=os.environ.get("LINGTU_ENV", "real"))
    parser.add_argument("--backend", default=os.environ.get("LINGTU_SIM_BACKEND"))
    parser.add_argument("--gateway-url", default=os.environ.get("GW", "http://localhost:5050"))
    parser.add_argument("--maps-root", type=Path, required=True)
    parser.add_argument("--map", dest="map_name", required=True)
    parser.add_argument("--goal", nargs="+", required=True, metavar="VALUE")
    parser.add_argument("--initial-pose", "--init-pose", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--soak-duration", type=int, default=20)
    parser.add_argument("--soak-interval", type=int, default=2)
    parser.add_argument("--route-timeout", type=int, default=90)
    parser.add_argument("--with-relocalization", action="store_true")
    parser.add_argument("--allow-motion", action="store_true")
    args = parser.parse_args(argv)
    if len(args.goal) not in (2, 3):
        parser.error("--goal requires X Y [YAW]")
    try:
        args.goal = tuple(float(value) for value in args.goal)
    except ValueError:
        parser.error("--goal requires numeric X Y [YAW]")
    if len(args.goal) == 2:
        args.goal = (*args.goal, 0.0)
    if args.soak_duration < 0:
        parser.error("--soak-duration must be non-negative")
    if args.soak_interval <= 0 or args.route_timeout <= 0:
        parser.error("--soak-interval and --route-timeout must be positive")
    if args.env == "sim" and not args.backend:
        parser.error("--backend is required with --env sim")
    return args


def _default_artifact_dir() -> Path:
    stamp = utc_now().replace(":", "").replace("-", "").split(".", 1)[0]
    return Path.home() / "data" / "SLAM" / "navigation" / "artifacts" / f"system_acceptance_{stamp}"


def _robot_cli(args: argparse.Namespace, *arguments: str) -> list[str]:
    return [
        os.environ.get("BASH", "bash"),
        str(ROOT / "scripts" / "lingtu"),
        "--env",
        args.env,
        *arguments,
    ]


def _write_summary(
    root: Path,
    *,
    args: argparse.Namespace,
    map_name: str,
    ok: bool,
    error: str | None,
) -> None:
    summary: dict[str, Any] = {
        "schema_version": 1,
        "mode": "system_acceptance",
        "ok": ok,
        "map": map_name,
        "goal": {"x": args.goal[0], "y": args.goal[1], "yaw": args.goal[2]},
        "checks": {
            "runtime_audit": str(root / "runtime_audit.json"),
            "doctor": str(root / "doctor.json"),
            "soak": str(root / "soak.json"),
            "saved_map_artifact_gate": str(root / "saved_map_artifact_gate.json"),
            "requested_plan_preview": str(root / "requested_plan_preview.txt"),
            "requested_map_validate_plan": str(root / "requested_map_validate_plan.json"),
            "saved_map_relocalization": str(root / "relocalization") if args.with_relocalization else None,
            "motion_smoke": str(root / "motion_smoke") if args.allow_motion else None,
            "product_switch": str(root / "product_switch.json"),
            "stop_session": str(root / "stop_session.json"),
        },
        "error": error,
    }
    write_json(root / "summary.json", summary)


def run(args: argparse.Namespace) -> int:
    map_name, map_dir = safe_map_dir(args.maps_root, args.map_name)
    root = (args.artifact_dir or _default_artifact_dir()).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    goal = {"x": args.goal[0], "y": args.goal[1], "z": 0.0, "yaw": args.goal[2]}

    print("=== LingTu system acceptance ===")
    print(f"Env: {args.env}")
    print(f"Gateway: {args.gateway_url}")
    print(f"Map: {map_name}")
    print(f"Artifact dir: {root}")
    print("MOTION ENABLED" if args.allow_motion else "No motion commands are sent.")

    session_owned = False
    failure: str | None = None
    try:
        print("[1/6] Static runtime contract")
        run_command(
            [sys.executable, str(ROOT / "lingtu.py"), "runtime-audit", "--json"],
            stdout_path=root / "runtime_audit.json",
            timeout_s=120,
        )

        print("[2/6] Gateway/ModulePort readiness")
        run_command(
            _robot_cli(args, "doctor", "--non-motion", "--strict", "--json"),
            stdout_path=root / "doctor.json",
            timeout_s=120,
        )

        print("[3/6] Sensor -> SLAM soak")
        run_command(
            _robot_cli(
                args,
                "soak",
                "--duration",
                str(args.soak_duration),
                "--interval",
                str(args.soak_interval),
                "--strict",
                "--json",
            ),
            stdout_path=root / "soak.json",
            timeout_s=max(120, args.soak_duration + 60),
        )

        print("[4/6] Saved-map artifacts")
        artifact_command = [
            sys.executable,
            str(GATES_DIR / "saved_map_artifact_gate.py"),
            str(map_dir),
            "--require-occupancy",
            "--expected-frame-id",
            "map",
            "--json",
        ]
        if args.env == "real":
            artifact_command.extend(("--expected-data-source", "thunder"))
        run_command(
            artifact_command,
            stdout_path=root / "saved_map_artifact_gate.json",
            timeout_s=120,
        )

        print("[5/6] Product switch and requested global plan preview")
        switch_args = ["switch", "nav", "--env", args.env, "--map", map_name]
        if args.backend:
            switch_args.extend(("--backend", args.backend))
        if args.with_relocalization:
            switch_args.extend(("--initial-pose", *(str(value) for value in args.initial_pose)))
        else:
            switch_args.append("--no-relocalize")
        run_product_control(switch_args, root / "product_switch.json")
        session_owned = True
        if args.with_relocalization:
            capture_phase(args.gateway_url, "system_acceptance_relocalization", root / "relocalization")
        validate_saved_map_plan(
            args.gateway_url,
            map_name,
            goal,
            root / "requested_map_validate_plan.json",
        )
        (root / "requested_plan_preview.txt").write_text(
            "PASS: saved-map plan validation succeeded\n",
            encoding="utf-8",
        )
    except (GateError, OSError) as exc:
        failure = str(exc)
    finally:
        if session_owned:
            try:
                stop_args = ["stop-session", "--env", args.env]
                if args.backend:
                    stop_args.extend(("--backend", args.backend))
                run_product_control(stop_args, root / "stop_session.json")
            except GateError as exc:
                failure = failure or str(exc)

    if failure is None and args.allow_motion:
        print("[6/6] Motion gate")
        motion_command = [
            sys.executable,
            str(GATES_DIR / "motion_smoke_gate.py"),
            "--env",
            args.env,
            "--gateway-url",
            args.gateway_url,
            "--maps-root",
            str(args.maps_root),
            "--map",
            map_name,
            "--goal",
            *(str(value) for value in args.goal),
            "--initial-pose",
            *(str(value) for value in args.initial_pose),
            "--allow-motion",
            "--timeout",
            str(args.route_timeout),
            "--artifact-dir",
            str(root / "motion_smoke"),
        ]
        if args.backend:
            motion_command.extend(("--backend", args.backend))
        try:
            run_command(
                motion_command,
                stdout_path=root / "motion_smoke.stdout",
                stderr_path=root / "motion_smoke.stderr",
                timeout_s=args.route_timeout + 180,
            )
        except GateError as exc:
            failure = str(exc)
    elif failure is None:
        print("[6/6] Motion gate skipped; add --allow-motion only in a controlled test.")

    _write_summary(
        root,
        args=args,
        map_name=map_name,
        ok=failure is None,
        error=failure,
    )
    if failure:
        print(f"FAIL: {failure}", file=sys.stderr)
        print(f"Summary: {root / 'summary.json'}", file=sys.stderr)
        return 2
    print("PASS: system acceptance gate completed")
    print(f"Summary: {root / 'summary.json'}")
    return 0


def main(argv: list[str] | None = None) -> int:
    try:
        return run(parse_args(list(argv or sys.argv[1:])))
    except GateError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
