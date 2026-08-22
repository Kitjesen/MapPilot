"""Run one bounded operator-drive command through the active LingTu Product."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from lingtu.control import ProductControl
from lingtu.product_lock import ProductControlLock

_TRANSLATION_SPEED_MPS = 0.20
_TURN_RATE_RAD_S = 0.35
_DURATION_S = 2.0
_PUBLISH_RATE_HZ = 20.0
_MAX_DURATION_S = 5.0
_ADMISSION_TIMEOUT_S = 5.0
_DIRECTIONS: Mapping[str, tuple[float, float, float, bool]] = {
    "forward": (1.0, 0.0, 0.0, True),
    "backward": (-1.0, 0.0, 0.0, True),
    "left": (0.0, 1.0, 0.0, True),
    "right": (0.0, -1.0, 0.0, True),
    "turn-left": (0.0, 0.0, 1.0, False),
    "turn-right": (0.0, 0.0, -1.0, False),
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="lingtu-drive",
        description=(
            "Move the active teleop Product in one direction, then hold and release control."
        ),
    )
    parser.add_argument("direction", choices=tuple(_DIRECTIONS))
    parser.add_argument(
        "--speed",
        type=float,
        help="Translation speed in m/s, or turn rate in rad/s",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=_DURATION_S,
        help=f"Command duration in seconds (maximum {_MAX_DURATION_S:g})",
    )
    parser.add_argument(
        "--robot",
        default=os.environ.get("LINGTU_ROBOT"),
        help="Robot model, for example unitree/go2",
    )
    parser.add_argument("--state-dir", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser


def _finite_positive(value: float, name: str) -> float:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return value


def _number(value: float) -> str:
    return f"{value:g}"


def _process_detail(stdout: str, stderr: str, fallback: str) -> str:
    return (stderr or stdout or fallback).strip()


def _stop_process(process: subprocess.Popen[str]) -> tuple[str, str]:
    process.terminate()
    try:
        return process.communicate(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.communicate()


def _wait_for_admission(
    process: subprocess.Popen[str],
    ready_path: Path,
) -> None:
    deadline = time.monotonic() + _ADMISSION_TIMEOUT_S
    while True:
        if ready_path.is_file():
            return
        if process.poll() is not None:
            stdout, stderr = process.communicate()
            raise RuntimeError(
                _process_detail(
                    stdout,
                    stderr,
                    "operator-motion ended before confirmed admission",
                )
            )
        if time.monotonic() >= deadline:
            stdout, stderr = _stop_process(process)
            detail = _process_detail(stdout, stderr, "no native error detail")
            raise RuntimeError(f"operator-motion admission timed out: {detail}")
        time.sleep(0.02)


def _drive(
    *,
    direction: str,
    speed: float | None,
    seconds: float,
    robot: str | None,
    state_dir: Path | None,
    dry_run: bool,
    environment: Mapping[str, str],
) -> dict[str, Any]:
    control = ProductControl(robot=robot, env="real", process_env=environment)
    lock = ProductControlLock(state_dir, environment=environment)
    process: subprocess.Popen[str] | None = None
    ready_path: Path | None = None
    with lock:
        plan, _plan_path, _product_session_id = control._current_plan_and_path(lock.state_dir)
        if plan.product not in {"teleop", "teleop_avoid"}:
            raise RuntimeError(
                "lingtu-drive requires the active Product to be teleop or teleop_avoid"
            )

        axis_x, axis_y, axis_yaw, translation = _DIRECTIONS[direction]
        selected_speed = _finite_positive(
            speed
            if speed is not None
            else (_TRANSLATION_SPEED_MPS if translation else _TURN_RATE_RAD_S),
            "speed",
        )
        selected_seconds = _finite_positive(seconds, "seconds")
        if selected_seconds > _MAX_DURATION_S:
            raise ValueError(f"seconds must not exceed {_MAX_DURATION_S:g}")

        native_environment = plan.native_process_environment
        limit_key = (
            "LINGTU_DRIVER_MAX_LINEAR_MPS"
            if translation
            else "LINGTU_DRIVER_MAX_ANGULAR_RPS"
        )
        limit = _finite_positive(float(native_environment[limit_key]), limit_key)
        if selected_speed > limit:
            unit = "m/s" if translation else "rad/s"
            raise ValueError(
                f"speed {selected_speed:g} {unit} exceeds the active RunPlan limit {limit:g} {unit}"
            )

        vx = axis_x * selected_speed
        vy = axis_y * selected_speed
        yaw = axis_yaw * selected_speed
        domain_id = str(native_environment.get("LINGTU_DDS_DOMAIN_ID") or "").strip()
        if not domain_id:
            raise RuntimeError("active RunPlan is missing LINGTU_DDS_DOMAIN_ID")
        binary = Path(
            environment.get("LINGTU_NAV_CONTROL_BIN")
            or "/opt/lingtu/current/build/nav_endpoint/lingtu_nav_control"
        )
        command: list[str] = [
            str(binary),
            "operator-motion",
            _number(vx),
            _number(vy),
            _number(yaw),
            "--duration-s",
            _number(selected_seconds),
            "--rate-hz",
            _number(_PUBLISH_RATE_HZ),
            "--source-id",
            "lingtu-drive",
            "--domain-id",
            domain_id,
        ]

        status = "planned" if dry_run else "completed"
        if not dry_run:
            if not binary.is_file():
                raise RuntimeError(f"native operator-motion client is missing: {binary}")
            descriptor, ready_name = tempfile.mkstemp(
                prefix=".operator-drive-",
                suffix=".ready",
                dir=lock.state_dir,
            )
            os.close(descriptor)
            ready_path = Path(ready_name)
            ready_path.unlink()
            command.extend(("--ready-file", str(ready_path)))
            try:
                process = subprocess.Popen(  # noqa: S603 - exact native client, no shell
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )
                _wait_for_admission(process, ready_path)
            except BaseException:
                if process is not None and process.poll() is None:
                    _stop_process(process)
                ready_path.unlink(missing_ok=True)
                raise

    if process is not None:
        try:
            try:
                stdout, stderr = process.communicate(timeout=selected_seconds + 10.0)
            except subprocess.TimeoutExpired as exc:
                stdout, stderr = _stop_process(process)
                detail = _process_detail(stdout, stderr, "no native error detail")
                raise RuntimeError(f"operator-motion timed out: {detail}") from exc
            if process.returncode != 0:
                raise RuntimeError(
                    _process_detail(stdout, stderr, "native command failed")
                )
        finally:
            if ready_path is not None:
                ready_path.unlink(missing_ok=True)

    payload: dict[str, Any] = {
        "ok": True,
        "status": status,
        "direction": direction,
        "speed": selected_speed,
        "seconds": selected_seconds,
        "robot": control.robot,
        "env": plan.env,
        "product": plan.product,
    }
    if translation:
        payload["nominal_distance_m"] = selected_speed * selected_seconds
    else:
        payload["nominal_turn_rad"] = selected_speed * selected_seconds
    return payload


def _print(payload: Mapping[str, Any], *, json_output: bool) -> None:
    if json_output:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return
    status = str(payload["status"])
    direction = str(payload["direction"])
    speed = float(payload["speed"])
    seconds = float(payload["seconds"])
    if "nominal_distance_m" in payload:
        distance = float(payload["nominal_distance_m"])
        print(
            f"{status}: {direction} at {speed:.2f} m/s for {seconds:.1f} s "
            f"(nominal open-loop distance {distance:.2f} m)"
        )
        return
    turn = float(payload["nominal_turn_rad"])
    print(
        f"{status}: {direction} at {speed:.2f} rad/s for {seconds:.1f} s "
        f"(nominal open-loop turn {math.degrees(turn):.1f} deg)"
    )


def main(argv: list[str] | None = None) -> int:
    """Run the short operator-facing drive command."""

    args = _parser().parse_args(argv)
    try:
        payload = _drive(
            direction=args.direction,
            speed=args.speed,
            seconds=args.seconds,
            robot=args.robot,
            state_dir=args.state_dir,
            dry_run=args.dry_run,
            environment=os.environ,
        )
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    _print(payload, json_output=args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
