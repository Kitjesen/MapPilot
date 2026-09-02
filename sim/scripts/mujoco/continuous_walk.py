"""Headless Thunder V4 continuous-walk qualification over the public sim seam."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from sim.compat.engine.core.engine import RobotState, VelocityCommand

from drivers.sim.mujoco.runtime import build_engine

REPO_ROOT = Path(__file__).resolve().parents[3]
WORLD_XML = REPO_ROOT / "sim" / "packages" / "worlds" / "open_field" / "physics" / "open_field.xml"
ROBOT_XML = REPO_ROOT / "sim" / "packages" / "robots" / "doso" / "thunder_v4" / "mjcf" / "thunderv4.xml"
POLICY = (
    REPO_ROOT
    / "sim"
    / "packages"
    / "controllers"
    / "doso"
    / "thunder_v4"
    / "locomotion"
    / "policy"
    / "policy_1119.onnx"
)
FORWARD_COMMAND_X_MPS = 0.6
POLICY_STARTUP_STAND_HOLD_S = 0.5
MAX_WARMUP_DISPLACEMENT_M = 0.08
MIN_WARMUP_END_HEIGHT_M = 0.35
MAX_WARMUP_END_HEIGHT_M = 0.50
MIN_FORWARD_DISPLACEMENT_M = 0.30
MIN_AVERAGE_FORWARD_SPEED_MPS = 0.50
MIN_BASE_HEIGHT_M = 0.30
MAX_BASE_HEIGHT_M = 0.65
MAX_ABS_ROLL_PITCH_RAD = 0.25
MAX_RELEASE_DISPLACEMENT_M = 0.10
MAX_RELEASE_FINAL_SPEED_MPS = 0.05


class QualificationInputError(ValueError):
    """A qualification input is not a regular file in this repository."""


def _repo_artifact_path(
    path: Path,
    *,
    label: str,
) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = REPO_ROOT / candidate
    message = f"{label} must be a regular repository file"
    try:
        absolute = candidate.resolve(strict=True)
        absolute.relative_to(REPO_ROOT.resolve())
        if not absolute.is_file():
            raise ValueError(message)
    except (OSError, ValueError) as exc:
        raise QualificationInputError(message) from exc
    return absolute


def _rpy_from_xyzw(orientation: np.ndarray) -> tuple[float, float, float]:
    """Convert the public RobotState quaternion to roll, pitch, and yaw."""

    x, y, z, w = (float(value) for value in orientation)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _state_is_finite(state: RobotState) -> bool:
    """Check every numeric field exposed by the public RobotState snapshot."""

    fields = (
        "position",
        "orientation",
        "linear_velocity",
        "angular_velocity",
        "joint_positions",
        "joint_velocities",
        "imu_gyro",
        "imu_projected_gravity",
        "imu_linear_acceleration",
    )
    return all(bool(np.isfinite(np.asarray(getattr(state, field))).all()) for field in fields)


def _build_engine(
    world_xml: Path = WORLD_XML,
    robot_xml: Path = ROBOT_XML,
    policy_path: Path = POLICY,
):
    """Build the real Thunder V4 policy engine through the runtime factory."""

    return build_engine(
        world=world_xml,
        robot_xml=robot_xml,
        drive_mode="policy",
        start=[0.0, 0.0, 0.0],
        start_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        initial_keyframe="v4_nominal_stand",
        policy_path=policy_path,
        policy_freq_hz=50.0,
        mujoco_memory="64M",
        mid360_pattern=None,
        mid360_samples_per_frame=16,
        require_product_lidar_backend=False,
    )


def run_qualification(
    *,
    seed: int = 7,
    warmup_s: float = 0.5,
    walk_s: float = 6.0,
    release_s: float = 2.0,
    forward_x_mps: float = FORWARD_COMMAND_X_MPS,
    robot_xml: Path = ROBOT_XML,
) -> dict[str, Any]:
    """Run nominal stand, continuous forward-walk, and release phases."""

    world_xml_path = _repo_artifact_path(
        WORLD_XML,
        label="world MJCF",
    )
    robot_xml_path = _repo_artifact_path(
        robot_xml,
        label="robot MJCF",
    )
    policy_path = _repo_artifact_path(
        POLICY,
        label="policy",
    )
    random_state = np.random.get_state()
    engine = None
    np.random.seed(seed)
    try:
        engine = _build_engine(
            world_xml=world_xml_path,
            robot_xml=robot_xml_path,
            policy_path=policy_path,
        )
        warmup_steps = max(1, round(float(warmup_s) / engine.control_dt))
        walk_steps = max(1, round(float(walk_s) / engine.control_dt))
        release_steps = max(1, round(float(release_s) / engine.control_dt))
        zero = VelocityCommand()
        warmup_states = [engine.get_robot_state()]
        for _ in range(warmup_steps):
            warmup_states.append(engine.step(zero))

        walk_start = warmup_states[-1]
        walk_states = []
        if not math.isfinite(float(forward_x_mps)) or float(forward_x_mps) <= 0.0:
            raise QualificationInputError("forward_x_mps must be finite and positive")
        walk_command = VelocityCommand(linear_x=float(forward_x_mps))
        for _ in range(walk_steps):
            walk_states.append(engine.step(walk_command))
        walk_end = walk_states[-1]

        release_start = walk_end
        release_states = [engine.step(zero)]
        for _ in range(release_steps - 1):
            release_states.append(engine.step())
        release_end = release_states[-1]

        start = warmup_states[0]
        end = warmup_states[-1]
        positions = np.asarray([state.position for state in warmup_states], dtype=np.float64)
        attitudes = np.asarray([_rpy_from_xyzw(state.orientation) for state in warmup_states], dtype=np.float64)
        heights = positions[:, 2]
        walk_signed_displacement = float(walk_end.position[0] - walk_start.position[0])
        walk_lateral_displacement = float(walk_end.position[1] - walk_start.position[1])
        walk_duration_s = float(walk_steps * engine.control_dt)
        phase_average_forward_speed_mps = walk_signed_displacement / walk_duration_s
        active_walk_duration_s = max(
            walk_duration_s - POLICY_STARTUP_STAND_HOLD_S,
            engine.control_dt,
        )
        active_average_forward_speed_mps = walk_signed_displacement / active_walk_duration_s
        motion_states = warmup_states + walk_states + release_states
        motion_positions = np.asarray([state.position for state in motion_states], dtype=np.float64)
        motion_attitudes = np.asarray(
            [_rpy_from_xyzw(state.orientation) for state in motion_states],
            dtype=np.float64,
        )
        report: dict[str, Any] = {
            "schema": "lingtu.sim.thunderv4-continuous-walk-qualification.v1",
            "seed": int(seed),
            "artifacts": {
                "world_mjcf": world_xml_path.relative_to(REPO_ROOT).as_posix(),
                "robot_mjcf": robot_xml_path.relative_to(REPO_ROOT).as_posix(),
                "policy": policy_path.relative_to(REPO_ROOT).as_posix(),
            },
            "timing": {
                "physics_timestep_s": float(engine.dt),
                "low_level_hz": float(1.0 / engine.dt),
                "inference_hz": float(1.0 / engine.control_dt),
                "startup_stand_hold_s": POLICY_STARTUP_STAND_HOLD_S,
            },
            "warmup": {
                "steps": int(warmup_steps),
                "duration_s": float(warmup_steps * engine.control_dt),
                "start_position_m": [float(value) for value in start.position],
                "end_position_m": [float(value) for value in end.position],
                "horizontal_displacement_m": float(np.linalg.norm(end.position[:2] - start.position[:2])),
                "base_height_min_m": float(np.min(heights)),
                "base_height_max_m": float(np.max(heights)),
                "max_abs_roll_rad": float(np.max(np.abs(attitudes[:, 0]))),
                "max_abs_pitch_rad": float(np.max(np.abs(attitudes[:, 1]))),
                "all_state_values_finite": all(_state_is_finite(state) for state in warmup_states),
            },
            "walk": {
                "steps": int(walk_steps),
                "duration_s": walk_duration_s,
                "active_duration_s": active_walk_duration_s,
                "command": {
                    "linear_x": float(forward_x_mps),
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                },
                "start_position_m": [float(value) for value in walk_start.position],
                "end_position_m": [float(value) for value in walk_end.position],
                "signed_forward_displacement_m": walk_signed_displacement,
                "lateral_displacement_m": walk_lateral_displacement,
                "travel_direction_error_rad": float(
                    math.atan2(walk_lateral_displacement, walk_signed_displacement)
                ),
                "phase_average_forward_speed_mps": phase_average_forward_speed_mps,
                "active_average_forward_speed_mps": active_average_forward_speed_mps,
                "horizontal_displacement_m": float(np.linalg.norm(walk_end.position[:2] - walk_start.position[:2])),
                "all_state_values_finite": all(_state_is_finite(state) for state in walk_states),
            },
            "release": {
                "steps": int(release_steps),
                "duration_s": float(release_steps * engine.control_dt),
                "command": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
                "start_position_m": [float(value) for value in release_start.position],
                "end_position_m": [float(value) for value in release_end.position],
                "horizontal_displacement_m": float(
                    np.linalg.norm(release_end.position[:2] - release_start.position[:2])
                ),
                "final_horizontal_speed_mps": float(np.linalg.norm(release_end.linear_velocity[:2])),
                "all_state_values_finite": all(_state_is_finite(state) for state in release_states),
            },
            "trajectory": {
                "base_height_min_m": float(np.min(motion_positions[:, 2])),
                "base_height_max_m": float(np.max(motion_positions[:, 2])),
                "max_abs_roll_rad": float(np.max(np.abs(motion_attitudes[:, 0]))),
                "max_abs_pitch_rad": float(np.max(np.abs(motion_attitudes[:, 1]))),
                "all_state_values_finite": all(_state_is_finite(state) for state in motion_states),
            },
            "checks": {
                "nominal_stand_warmup": bool(
                    all(_state_is_finite(state) for state in warmup_states)
                    and float(np.linalg.norm(end.position[:2] - start.position[:2])) <= MAX_WARMUP_DISPLACEMENT_M
                    and MIN_WARMUP_END_HEIGHT_M <= float(end.position[2]) <= MAX_WARMUP_END_HEIGHT_M
                    and float(np.max(np.abs(attitudes[:, 0]))) <= MAX_ABS_ROLL_PITCH_RAD
                    and float(np.max(np.abs(attitudes[:, 1]))) <= MAX_ABS_ROLL_PITCH_RAD
                ),
                "continuous_forward_walk": bool(
                    walk_signed_displacement >= MIN_FORWARD_DISPLACEMENT_M
                    and active_average_forward_speed_mps >= MIN_AVERAGE_FORWARD_SPEED_MPS
                ),
                "base_height_and_attitude": bool(
                    float(np.min(motion_positions[:, 2])) >= MIN_BASE_HEIGHT_M
                    and float(np.max(motion_positions[:, 2])) <= MAX_BASE_HEIGHT_M
                    and float(np.max(np.abs(motion_attitudes[:, 0]))) <= MAX_ABS_ROLL_PITCH_RAD
                    and float(np.max(np.abs(motion_attitudes[:, 1]))) <= MAX_ABS_ROLL_PITCH_RAD
                ),
                "all_state_values_finite": bool(all(_state_is_finite(state) for state in motion_states)),
                "release_stops": bool(
                    float(np.linalg.norm(release_end.position[:2] - release_start.position[:2]))
                    <= MAX_RELEASE_DISPLACEMENT_M
                    and float(np.linalg.norm(release_end.linear_velocity[:2])) <= MAX_RELEASE_FINAL_SPEED_MPS
                ),
            },
        }
        report["thresholds"] = {
            "max_warmup_displacement_m": MAX_WARMUP_DISPLACEMENT_M,
            "warmup_end_base_height_m": {
                "min": MIN_WARMUP_END_HEIGHT_M,
                "max": MAX_WARMUP_END_HEIGHT_M,
            },
            "min_forward_displacement_m": MIN_FORWARD_DISPLACEMENT_M,
            "min_average_forward_speed_mps": MIN_AVERAGE_FORWARD_SPEED_MPS,
            "base_height_m": {"min": MIN_BASE_HEIGHT_M, "max": MAX_BASE_HEIGHT_M},
            "max_abs_roll_pitch_rad": MAX_ABS_ROLL_PITCH_RAD,
            "max_release_displacement_m": MAX_RELEASE_DISPLACEMENT_M,
            "max_release_final_horizontal_speed_mps": MAX_RELEASE_FINAL_SPEED_MPS,
        }
        report["qualified"] = bool(all(report["checks"].values()))
        return report
    finally:
        try:
            if engine is not None:
                engine.close()
        finally:
            np.random.set_state(random_state)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--warmup-s", type=float, default=0.5)
    parser.add_argument("--walk-s", type=float, default=6.0)
    parser.add_argument("--release-s", type=float, default=2.0)
    parser.add_argument("--vx", type=float, default=FORWARD_COMMAND_X_MPS)
    parser.add_argument(
        "--robot-xml",
        type=Path,
        default=ROBOT_XML,
        help="Repository-contained ThunderV4 MJCF to qualify.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=None,
        help="Optional path for the JSON qualification evidence.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the qualification and return a process exit status."""

    args = _parser().parse_args(argv)
    try:
        report = run_qualification(
            seed=args.seed,
            warmup_s=args.warmup_s,
            walk_s=args.walk_s,
            release_s=args.release_s,
            forward_x_mps=args.vx,
            robot_xml=args.robot_xml,
        )
    except Exception as exc:
        message = str(exc) if isinstance(exc, QualificationInputError) else "qualification failed"
        report = {
            "schema": "lingtu.sim.thunderv4-continuous-walk-qualification.v1",
            "seed": int(args.seed),
            "qualified": False,
            "error": {"type": type(exc).__name__, "message": message},
        }

    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("qualified") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
