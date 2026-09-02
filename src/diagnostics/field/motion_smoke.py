#!/usr/bin/env python3
# ruff: noqa: D103
"""Run one controlled navigation motion smoke through ProductControl."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

from diagnostics.field.gate_support import (
    GateError,
    capture_phase,
    plan_preview,
    poll_navigation_terminal,
    read_json,
    request_json,
    require_no_active_command_source,
    run_command,
    run_product_control,
    safe_map_dir,
    start_evidence_collector,
    strict_goal_ack,
    utc_now,
    write_json,
)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one low-speed native navigation loop with explicit motion consent.",
    )
    parser.add_argument("--env", choices=("real", "sim"), default=os.environ.get("LINGTU_ENV", "real"))
    parser.add_argument("--backend", default=os.environ.get("LINGTU_SIM_BACKEND"))
    parser.add_argument("--gateway-url", default=os.environ.get("GW", "http://localhost:5050"))
    parser.add_argument("--maps-root", type=Path, required=True)
    parser.add_argument("--map", dest="map_name", required=True)
    parser.add_argument("--goal", nargs="+", required=True, metavar="VALUE")
    parser.add_argument("--initial-pose", "--init-pose", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    parser.add_argument("--reuse-tracking", action="store_true")
    parser.add_argument("--slam-profile")
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--poll", type=float, default=1.0)
    parser.add_argument("--evidence-duration", "--evidence-duration-sec", type=float, default=20.0)
    parser.add_argument("--min-motion-m", type=float, default=0.05)
    parser.add_argument("--expected-command-subscriber", action="append", default=[])
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
    if not args.allow_motion:
        parser.error("motion-smoke requires --allow-motion")
    if args.env == "sim" and not args.backend:
        parser.error("--backend is required with --env sim")
    if args.slam_profile:
        parser.error("--slam-profile bypasses the Product contract; select a Product instead")
    for name in ("timeout", "poll", "evidence_duration"):
        if float(getattr(args, name)) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if float(args.min_motion_m) < 0:
        parser.error("--min-motion-m must be non-negative")
    return args


def _default_artifact_dir() -> Path:
    stamp = utc_now().replace(":", "").replace("-", "").split(".", 1)[0]
    return Path.home() / "data" / "SLAM" / "navigation" / "artifacts" / f"motion_smoke_{stamp}"


def _evidence_command(args: argparse.Namespace, output: Path, duration_s: float) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "diagnostics.field.runtime_evidence",
        "--gateway-url",
        args.gateway_url,
        "--duration-sec",
        str(duration_s),
        "--min-motion-m",
        str(args.min_motion_m),
        "--json-out",
        str(output),
        "--json",
    ]
    if args.env == "sim":
        command.append("--no-validate")
    for subscriber in args.expected_command_subscriber:
        command.extend(("--expected-command-subscriber", subscriber))
    return command


def _write_summary(
    root: Path,
    *,
    outcome: str,
    map_name: str,
    goal: tuple[float, float, float],
    error: str | None,
) -> None:
    evidence = read_json(root / "evidence.json")
    latest = read_json(root / "navigation.latest.json")
    summary: dict[str, Any] = {
        "schema_version": 1,
        "generated_at": utc_now(),
        "mode": "motion_smoke",
        "outcome": outcome,
        "map": map_name,
        "goal": {"x": goal[0], "y": goal[1], "yaw": goal[2]},
        "slam_profile": None,
        "artifacts": {
            "before": str(root / "before"),
            "session": str(root / "session"),
            "plan": str(root / "plan"),
            "goal": str(root / "goal.json"),
            "navigation_samples": str(root / "navigation.samples.jsonl"),
            "evidence": str(root / "evidence.json"),
            "after_stop": str(root / "after_stop"),
            "product_switch": str(root / "product_switch.json"),
            "stop_session": str(root / "stop_session.json"),
        },
        "terminal_state": latest.get("state"),
        "failure_reason": latest.get("failure_reason"),
        "real_runtime_evidence": {
            "ok": evidence.get("ok"),
            "real_robot_motion": evidence.get("real_robot_motion"),
            "cmd_vel_sent_to_hardware": evidence.get("cmd_vel_sent_to_hardware"),
            "motion": evidence.get("motion"),
            "outputs": evidence.get("outputs"),
            "blockers": evidence.get("blockers"),
        },
        "error": error,
    }
    write_json(root / "motion_smoke_summary.json", summary)


def run(args: argparse.Namespace) -> int:
    map_name, _ = safe_map_dir(args.maps_root, args.map_name)
    artifact_dir = (args.artifact_dir or _default_artifact_dir()).expanduser().resolve()
    for child in ("before", "session", "plan", "after_stop"):
        (artifact_dir / child).mkdir(parents=True, exist_ok=True)
    goal = {"x": args.goal[0], "y": args.goal[1], "z": 0.0, "yaw": args.goal[2]}

    print("=== LingTu motion smoke ===")
    print(f"Env: {args.env}")
    print(f"Gateway: {args.gateway_url}")
    print(f"Map: {map_name}")
    print(f"Goal: x={args.goal[0]} y={args.goal[1]} yaw={args.goal[2]}")
    print(f"Artifact dir: {artifact_dir}")
    print("MOTION ENABLED: this switches to nav and submits one explicit goal.")

    session_owned = False
    evidence_process: subprocess.Popen[str] | None = None
    evidence_stdout: Any = None
    evidence_stderr: Any = None
    evidence_status: int | None = None
    failure: str | None = None

    try:
        capture_phase(args.gateway_url, "motion_smoke_before", artifact_dir / "before")
        navigation = request_json(args.gateway_url, "/api/v1/navigation/status", timeout_s=5)
        require_no_active_command_source(navigation, "motion-smoke before Product switch")

        switch_args = ["switch", "nav", "--env", args.env, "--map", map_name]
        if args.backend:
            switch_args.extend(("--backend", args.backend))
        if args.reuse_tracking:
            switch_args.append("--no-relocalize")
        else:
            switch_args.extend(("--initial-pose", *(str(value) for value in args.initial_pose)))
        run_product_control(switch_args, artifact_dir / "product_switch.json")
        session_owned = True
        capture_phase(args.gateway_url, "motion_smoke_session_started", artifact_dir / "session")

        run_command(
            _evidence_command(args, artifact_dir / "evidence_preflight.json", 8.0),
            stdout_path=artifact_dir / "evidence_preflight.stdout",
            stderr_path=artifact_dir / "evidence_preflight.stderr",
            timeout_s=45,
            check=False,
        )
        plan_preview(args.gateway_url, goal, artifact_dir / "plan")

        goal_response = request_json(
            args.gateway_url,
            "/api/v1/goal",
            method="POST",
            payload=goal,
            timeout_s=float(os.environ.get("LINGTU_GOAL_POST_TIMEOUT", "35")),
        )
        write_json(artifact_dir / "goal.json", goal_response)
        if not strict_goal_ack(goal_response):
            raise GateError("motion-smoke goal was rejected")

        evidence_process, evidence_stdout, evidence_stderr = start_evidence_collector(
            gateway_url=args.gateway_url,
            duration_s=args.evidence_duration,
            min_motion_m=args.min_motion_m,
            expected_subscribers=args.expected_command_subscriber,
            output_path=artifact_dir / "evidence.json",
            validate=args.env == "real",
        )
        poll_navigation_terminal(
            args.gateway_url,
            artifact_dir,
            timeout_s=args.timeout,
            poll_s=args.poll,
        )
    except (GateError, subprocess.TimeoutExpired) as exc:
        failure = str(exc)
    finally:
        if session_owned:
            try:
                stop_args = ["stop", "--env", args.env]
                run_product_control(stop_args, artifact_dir / "stop.json")
            except GateError as exc:
                failure = failure or str(exc)
        if evidence_process is not None:
            try:
                evidence_status = evidence_process.wait(timeout=args.evidence_duration + 30)
            except subprocess.TimeoutExpired:
                evidence_process.kill()
                evidence_status = evidence_process.wait(timeout=10)
                failure = failure or "runtime evidence collector timed out"
            finally:
                evidence_stdout.close()
                evidence_stderr.close()
        capture_phase(args.gateway_url, "motion_smoke_after_stop", artifact_dir / "after_stop")

    if evidence_status not in (None, 0):
        failure = failure or f"real-runtime evidence failed with exit code {evidence_status}"
    outcome = "pass" if failure is None else "failed"
    _write_summary(
        artifact_dir,
        outcome=outcome,
        map_name=map_name,
        goal=args.goal,
        error=failure,
    )
    if failure:
        print(f"FAIL: {failure}", file=sys.stderr)
        print(f"Summary: {artifact_dir / 'motion_smoke_summary.json'}", file=sys.stderr)
        return 2
    print("PASS: motion smoke completed")
    print(f"Summary: {artifact_dir / 'motion_smoke_summary.json'}")
    return 0


def main(argv: list[str] | None = None) -> int:
    try:
        return run(parse_args(list(argv or sys.argv[1:])))
    except GateError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
