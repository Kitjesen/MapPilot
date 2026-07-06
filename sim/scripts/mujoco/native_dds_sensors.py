#!/usr/bin/env python3
"""Publish MuJoCo MID-360/IMU raw sensors to the native DDS SLAM boundary.

This script is a simulator sensor bridge, not SLAM. It never estimates pose,
builds a map, or relocalizes. The expected SLAM/runtime consumer is the native
C++ ``lingtu_slam_cyclone_runtime`` process.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drivers.sim.mujoco.driver import _xyzi_to_livox_frame
from drivers.sim.mujoco.runtime import (
    DEFAULT_MID360_PATTERN,
    DEFAULT_MID360_SAMPLES_PER_FRAME,
    build_engine,
    parse_start,
    resolve_world,
)
from drivers.sim.mujoco.sensors import (
    angle_delta_rad,
    projected_gravity_body,
    specific_force_body,
    world_xyzi_to_sensor_xyzi,
    yaw_from_quat_xyzw,
)
from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from sim.scripts.mujoco_live.motion import (
    _physical_rolling_scan_from_samples,
    _relative_times_for_scan,
)


NATIVE_SLAM_RUNTIME = "lingtu_slam_cyclone_runtime"
NATIVE_SENSOR_PUBLISHER = "livox_sdk2_stream --stdin-records --dds"
LIDAR_FRAME_ID = topic_default_frame_id(TOPICS.lidar_scan)
IMU_FRAME_ID = topic_default_frame_id(TOPICS.imu)
_MAGIC = b"LTU1"
_RECORD_CLOUD = 1
_RECORD_IMU = 2
_RECORD_ODOM_PRIOR = 3
_HEADER = struct.Struct("<4sB3xQIII")
_IMU_PAYLOAD = struct.Struct("<ffffff")
_ODOM_PRIOR_PAYLOAD = struct.Struct("<ddddddddddB7x")
_MID360_ACCEL_MPS2_PER_G = 9.80665
KINEMATIC_LEGACY_IMU_ACC_AXIS_SCALE = (-0.43, 1.0, 1.0)
# MuJoCo kinematic drive sets base velocity directly. Its finite-difference X
# acceleration is a control artifact, not a physical IMU force for Fast-LIO.
KINEMATIC_SIM_HARDWARE_IMU_ACC_AXIS_SCALE = (0.0, 1.0, 1.0)
_THUNDERV4_POLICY_DIR = ROOT / "sim" / "robots" / "thunderv4" / "policy"
DEFAULT_THUNDERV4_ONNX_POLICY = _THUNDERV4_POLICY_DIR / "pose_flat_low_kpkd_microterrain_model29600_policy.onnx"
DEFAULT_THUNDERV4_TORCHSCRIPT_POLICY = _THUNDERV4_POLICY_DIR / "pose_flat_low_kpkd_microterrain_model29600_policy.pt"
REQUIRED_SLAM_OUTPUT_TOPICS = (
    TOPICS.odometry,
    TOPICS.map_cloud,
    TOPICS.localization_health,
)
SIM_HARDWARE_CLOCK = "sim_hardware"
DEFAULT_IMU_ACC_CONDITIONING = "realistic"
DEFAULT_IMU_ACC_LOWPASS_HZ = 30.0
DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2 = 6.0
DEFAULT_IMU_ACC_MAX_SLEW_MPS3 = 400.0


def _gravity_specific_force_body(state: Any) -> np.ndarray:
    return -projected_gravity_body(state) * _MID360_ACCEL_MPS2_PER_G


class SimImuSignalConditioner:
    """Sensor-side conditioning for MuJoCo accelerometer samples.

    This keeps Fast-LIO on the normal accelerometer route while modeling the
    finite bandwidth and saturation envelope that a real IMU/driver chain has.
    """

    def __init__(
        self,
        *,
        lowpass_hz: float = DEFAULT_IMU_ACC_LOWPASS_HZ,
        max_dynamic_accel_mps2: float = DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2,
        max_slew_rate_mps3: float = DEFAULT_IMU_ACC_MAX_SLEW_MPS3,
    ) -> None:
        self.lowpass_hz = max(0.0, float(lowpass_hz))
        self.max_dynamic_accel_mps2 = max(0.0, float(max_dynamic_accel_mps2))
        self.max_slew_rate_mps3 = max(0.0, float(max_slew_rate_mps3))
        self._prev_accel: np.ndarray | None = None
        self._count = 0
        self._dynamic_clipped_count = 0
        self._slew_limited_count = 0
        self._raw_dynamic_max_mps2 = 0.0
        self._conditioned_dynamic_max_mps2 = 0.0

    def condition(self, accel: Any, *, state: Any, dt_s: float) -> np.ndarray:
        raw = np.asarray(accel, dtype=np.float64).reshape(3)
        gravity = _gravity_specific_force_body(state)
        dynamic = raw - gravity
        raw_dynamic_norm = float(np.linalg.norm(dynamic))
        self._raw_dynamic_max_mps2 = max(self._raw_dynamic_max_mps2, raw_dynamic_norm)

        if self.max_dynamic_accel_mps2 > 0.0 and raw_dynamic_norm > self.max_dynamic_accel_mps2:
            dynamic = dynamic * (self.max_dynamic_accel_mps2 / max(raw_dynamic_norm, 1e-12))
            self._dynamic_clipped_count += 1
        target = gravity + dynamic

        dt = max(0.0, float(dt_s))
        if self._prev_accel is not None and self.max_slew_rate_mps3 > 0.0 and dt > 0.0:
            delta = target - self._prev_accel
            delta_norm = float(np.linalg.norm(delta))
            max_delta = self.max_slew_rate_mps3 * dt
            if delta_norm > max_delta:
                target = self._prev_accel + delta * (max_delta / max(delta_norm, 1e-12))
                self._slew_limited_count += 1

        if self._prev_accel is not None and self.lowpass_hz > 0.0 and dt > 0.0:
            tau = 1.0 / (2.0 * math.pi * self.lowpass_hz)
            alpha = dt / (tau + dt)
            target = self._prev_accel + alpha * (target - self._prev_accel)

        self._prev_accel = target.astype(np.float64, copy=True)
        conditioned_dynamic_norm = float(np.linalg.norm(target - gravity))
        self._conditioned_dynamic_max_mps2 = max(
            self._conditioned_dynamic_max_mps2,
            conditioned_dynamic_norm,
        )
        self._count += 1
        return target.astype(np.float64, copy=False)

    def stats(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "lowpass_hz": self.lowpass_hz,
            "max_dynamic_accel_mps2": self.max_dynamic_accel_mps2,
            "max_slew_rate_mps3": self.max_slew_rate_mps3,
            "sample_count": self._count,
            "dynamic_clipped_count": self._dynamic_clipped_count,
            "slew_limited_count": self._slew_limited_count,
            "raw_dynamic_max_mps2": self._raw_dynamic_max_mps2,
            "conditioned_dynamic_max_mps2": self._conditioned_dynamic_max_mps2,
        }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="industrial_park")
    parser.add_argument("--start", default="", help="Optional start pose x,y,z")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--settle-s",
        type=float,
        default=3.0,
        help=(
            "Advance MuJoCo with a zero command before starting the simulated "
            "hardware clock or publishing sensors, keeping Fast-LIO IMU "
            "initialization away from model drop/settle transients."
        ),
    )
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=2.0,
        help="Publish stationary IMU/LiDAR before driving so Fast-LIO can initialize gyro bias at rest.",
    )
    parser.add_argument(
        "--drive-ramp-s",
        type=float,
        default=5.0,
        help="Ramp kinematic cmd_vel after warmup to avoid an unrealistic IMU acceleration step.",
    )
    parser.add_argument("--publish-hz", type=float, default=10.0, help="LiDAR frame rate.")
    parser.add_argument("--imu-hz", type=float, default=200.0)
    parser.add_argument(
        "--publish-odom-prior",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Publish MuJoCo odometry to /slam/odom_prior for simulation-only "
            "diagnostics. Real hardware configs keep this path off, and "
            "Fast-LIO simulation acceptance must not depend on it."
        ),
    )
    parser.add_argument(
        "--imu-acc-mode",
        choices=["sensor", "finite_difference", "gravity_only"],
        default="sensor",
        help=(
            "IMU specific-force source. The native DDS MuJoCo default is "
            "sensor so Fast-LIO receives the same raw accelerometer route as "
            "real hardware. Use gravity_only only as a diagnostic fallback; "
            "finite_difference is legacy."
        ),
    )
    parser.add_argument(
        "--imu-acc-conditioning",
        choices=["realistic", "off"],
        default=DEFAULT_IMU_ACC_CONDITIONING,
        help=(
            "Condition MuJoCo accelerometer samples before publishing raw IMU. "
            "The default keeps sensor mode on the real Fast-LIO route while "
            "adding sensor-bandwidth low-pass, dynamic acceleration clipping, "
            "and slew limiting to suppress non-physical contact impulses."
        ),
    )
    parser.add_argument(
        "--imu-acc-lowpass-hz",
        type=float,
        default=DEFAULT_IMU_ACC_LOWPASS_HZ,
        help="First-order accelerometer low-pass cutoff used by realistic conditioning.",
    )
    parser.add_argument(
        "--imu-acc-max-dynamic-mps2",
        type=float,
        default=DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2,
        help="Maximum body-frame dynamic acceleration magnitude before clipping in realistic conditioning.",
    )
    parser.add_argument(
        "--imu-acc-max-slew-mps3",
        type=float,
        default=DEFAULT_IMU_ACC_MAX_SLEW_MPS3,
        help="Maximum accelerometer slew rate before clipping in realistic conditioning.",
    )
    parser.add_argument(
        "--imu-acc-axis-scale",
        default="auto",
        help=(
            "Comma-separated XYZ scale applied to simulated IMU specific force before DDS publication. "
            "'auto' uses the calibrated MuJoCo kinematic profile."
        ),
    )
    parser.add_argument(
        "--imu-gyro-axis-scale",
        default="1,1,1",
        help="Comma-separated XYZ scale applied to simulated IMU angular velocity before DDS publication.",
    )
    parser.add_argument(
        "--scan-time-profile",
        choices=["instantaneous", "synthetic_rolling", "physical_rolling"],
        default="physical_rolling",
    )
    parser.add_argument(
        "--physical-rolling-sample-mode",
        choices=["full_frame", "subscan"],
        default="subscan",
        help=(
            "How physical_rolling collects MuJoCo LiDAR samples. subscan allocates "
            "the MID-360 scan pattern across IMU samples and matches rolling-scan "
            "timing. full_frame is a legacy diagnostic compatibility mode and must "
            "not be used as saved-map acceptance evidence."
        ),
    )
    parser.add_argument(
        "--timestamp-clock",
        choices=[SIM_HARDWARE_CLOCK, "sim", "wall"],
        default=SIM_HARDWARE_CLOCK,
        help=(
            "Clock profile used for DDS LiDAR/IMU timestamps. sim_hardware is the "
            "product MuJoCo bridge mode: LiDAR scan timing, IMU sampling, and "
            "publish pacing share one simulated hardware clock. wall/sim are "
            "legacy diagnostics."
        ),
    )
    parser.add_argument(
        "--sim-hardware-realtime-factor",
        type=float,
        default=1.0,
        help="Wall pacing factor for sim_hardware mode. 1.0 means one simulated second per wall second.",
    )
    parser.add_argument(
        "--imu-timestamp-clock",
        choices=["", SIM_HARDWARE_CLOCK, "sim", "wall"],
        default="",
        help="Optional IMU timestamp clock override. Empty inherits --timestamp-clock.",
    )
    parser.add_argument(
        "--lidar-timestamp-clock",
        choices=["", SIM_HARDWARE_CLOCK, "sim", "wall"],
        default="",
        help="Optional LiDAR scan timestamp clock override. Empty inherits --timestamp-clock.",
    )
    parser.add_argument("--min-localization-quality", type=float, default=0.5)
    parser.add_argument("--max-odom-abs-m", type=float, default=100.0)
    parser.add_argument("--max-odom-z-abs-m", type=float, default=20.0)
    parser.add_argument(
        "--min-sim-motion-for-odom-check-m",
        type=float,
        default=0.5,
        help="When requiring SLAM output, fail if MuJoCo moves at least this far but SLAM odometry barely moves.",
    )
    parser.add_argument(
        "--min-slam-motion-ratio",
        type=float,
        default=0.50,
        help="Minimum SLAM odom XY motion divided by MuJoCo XY motion for the motion-consistency gate.",
    )
    parser.add_argument(
        "--max-slam-motion-ratio",
        type=float,
        default=1.6,
        help="Maximum SLAM odom XY motion divided by MuJoCo XY motion for the motion-consistency gate.",
    )
    parser.add_argument(
        "--min-sim-yaw-for-odom-check-rad",
        type=float,
        default=0.20,
        help="When MuJoCo yaw changes by at least this much, compare SLAM odom yaw against it.",
    )
    parser.add_argument(
        "--max-slam-yaw-error-rad",
        type=float,
        default=0.15,
        help="Maximum absolute SLAM odom yaw error relative to MuJoCo yaw delta.",
    )
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--publisher-bin", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN", ""))
    parser.add_argument("--slam-status-json", default=os.environ.get("LINGTU_SLAM_STATUS_JSON", ""))
    parser.add_argument("--drive-mode", choices=["kinematic", "policy"], default="policy")
    parser.add_argument(
        "--allow-kinematic-fastlio-acceptance",
        action="store_true",
        help=(
            "Allow require-slam-output gates to pass with kinematic drive. "
            "Default false because kinematic cmd_vel pose injection is not a "
            "hardware-equivalent LiDAR/IMU source for Fast-LIO saved-map claims."
        ),
    )
    parser.add_argument(
        "--drive-profile",
        choices=["arc", "box_explore", "box_explore_gentle"],
        default="arc",
        help=(
            "Kinematic command profile. arc preserves the historical constant "
            "cmd_vel path; box_explore alternates forward runs and turns to "
            "exercise long-map accumulation."
        ),
    )
    parser.add_argument("--drive-vx", type=float, default=0.10)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.04)
    parser.add_argument("--policy-path", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_POLICY_PATH", ""))
    parser.add_argument("--n-rays", type=int, default=6400)
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--mid360-pattern", type=Path, default=DEFAULT_MID360_PATTERN)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument("--lidar-backend", choices=["mujoco_lidar", "ray_caster_lidar"], default="mujoco_lidar")
    parser.add_argument("--mujoco-lidar-backend", choices=["cpu", "taichi", "warp", "jax"], default="cpu")
    parser.add_argument("--allow-legacy-lidar-fallback", action="store_true")
    parser.add_argument("--max-points", type=int, default=15000)
    parser.add_argument(
        "--require-slam-output",
        action="store_true",
        help="Fail unless native C++ SLAM publishes odometry, map cloud, and health.",
    )
    parser.add_argument(
        "--motion-log",
        default="",
        help=(
            "Optional JSONL path for periodic MuJoCo ground-truth pose samples. "
            "Each line carries the sensor-clock timestamp so continuous gates "
            "can join simulator truth against the native SLAM trajectory."
        ),
    )
    parser.add_argument("--json-out", default="")
    return parser


def _runtime_imu_from_state(
    state: Any,
    ts: float,
    prev_velocity: Any = None,
    dt: float = 0.0,
    acc_mode: str = "finite_difference",
    acc_conditioner: SimImuSignalConditioner | None = None,
) -> Imu:
    quat = np.asarray(getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)), dtype=float)
    gyro = np.asarray(getattr(state, "imu_gyro", (0.0, 0.0, 0.0)), dtype=float)
    accel = specific_force_body(state, prev_velocity, dt, mode=acc_mode)
    if acc_conditioner is not None:
        accel = acc_conditioner.condition(accel, state=state, dt_s=dt)
    return Imu(
        orientation=Quaternion(
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
            float(quat[3]),
        ),
        angular_velocity=Vector3(float(gyro[0]), float(gyro[1]), float(gyro[2])),
        linear_acceleration=Vector3(float(accel[0]), float(accel[1]), float(accel[2])),
        ts=float(ts),
        frame_id=IMU_FRAME_ID,
    )


def _resolve_imu_acc_axis_scale(
    value: str,
    *,
    drive_mode: str,
    acc_mode: str,
    timestamp_clock: str = "",
) -> tuple[tuple[float, float, float], str]:
    text = str(value or "auto").strip().lower()
    if text == "auto":
        if (
            str(drive_mode).strip().lower() == "kinematic"
            and str(acc_mode).strip().lower() == "finite_difference"
        ):
            if str(timestamp_clock).strip().lower() == SIM_HARDWARE_CLOCK:
                return KINEMATIC_SIM_HARDWARE_IMU_ACC_AXIS_SCALE, "auto_kinematic_sim_hardware"
            return KINEMATIC_LEGACY_IMU_ACC_AXIS_SCALE, "auto_kinematic_legacy_split"
        return (1.0, 1.0, 1.0), "auto_identity"

    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        raise ValueError("--imu-acc-axis-scale must be 'auto' or three comma-separated numbers")
    try:
        x, y, z = (float(part) for part in parts)
    except ValueError as exc:
        raise ValueError("--imu-acc-axis-scale contains a non-numeric value") from exc
    return (x, y, z), "explicit"


def _apply_imu_acc_axis_scale(imu: Imu, scale: tuple[float, float, float]) -> None:
    imu.linear_acceleration.x *= float(scale[0])
    imu.linear_acceleration.y *= float(scale[1])
    imu.linear_acceleration.z *= float(scale[2])


def _resolve_axis_scale(value: str, name: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in str(value or "").split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} must contain three comma-separated numbers")
    try:
        values = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise ValueError(f"{name} contains a non-numeric value") from exc
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{name} values must be finite")
    return values  # type: ignore[return-value]


def _apply_imu_gyro_axis_scale(imu: Imu, scale: tuple[float, float, float]) -> None:
    imu.angular_velocity.x *= float(scale[0])
    imu.angular_velocity.y *= float(scale[1])
    imu.angular_velocity.z *= float(scale[2])


def _drive_command_for_profile(
    profile: str,
    elapsed_s: float,
    *,
    drive_vx: float,
    drive_vy: float,
    drive_wz: float,
) -> tuple[float, float, float]:
    name = str(profile or "arc").strip().lower()
    if name == "arc":
        return float(drive_vx), float(drive_vy), float(drive_wz)
    if name == "box_explore_gentle":
        forward_vx = max(abs(float(drive_vx)), 0.10)
        turn_wz = math.copysign(max(abs(float(drive_wz)), 0.20), float(drive_wz) if drive_wz else 1.0)
        phase_s = max(0.0, float(elapsed_s)) % 28.0
        if phase_s < 9.0:
            return forward_vx, 0.0, 0.0
        if phase_s < 13.0:
            return 0.0, 0.0, turn_wz
        if phase_s < 22.0:
            return forward_vx, 0.0, 0.0
        return 0.0, 0.0, turn_wz
    if name == "box_explore":
        forward_vx = max(abs(float(drive_vx)), 0.10)
        turn_wz = math.copysign(max(abs(float(drive_wz)), 0.35), float(drive_wz) if drive_wz else 1.0)
        phase_s = max(0.0, float(elapsed_s)) % 24.0
        if phase_s < 7.5:
            return forward_vx, 0.0, 0.0
        if phase_s < 12.0:
            return 0.0, 0.0, turn_wz
        if phase_s < 19.5:
            return forward_vx, 0.0, 0.0
        return 0.0, 0.0, turn_wz
    raise ValueError(f"unknown drive profile: {profile}")


class SimulatedHardwareClock:
    """One simulated sensor clock with optional wall pacing.

    Timestamps advance in simulated seconds. Wall time only throttles publication
    so native consumers see a realistic sensor stream instead of a bursty replay.
    """

    def __init__(
        self,
        *,
        sim_start_s: float,
        wall_epoch_s: float,
        monotonic_start_s: float,
        realtime_factor: float = 1.0,
    ) -> None:
        if not math.isfinite(float(realtime_factor)) or float(realtime_factor) <= 0.0:
            raise ValueError("--sim-hardware-realtime-factor must be positive and finite")
        self.sim_start_s = float(sim_start_s)
        self.wall_epoch_s = float(wall_epoch_s) - self.sim_start_s
        self.monotonic_start_s = float(monotonic_start_s)
        self.realtime_factor = float(realtime_factor)

    def timestamp_s(self, sim_time_s: float) -> float:
        return self.wall_epoch_s + float(sim_time_s)

    def monotonic_deadline_s(self, sim_time_s: float) -> float:
        sim_elapsed_s = max(0.0, float(sim_time_s) - self.sim_start_s)
        return self.monotonic_start_s + sim_elapsed_s / self.realtime_factor

    def sleep_until(self, sim_time_s: float) -> None:
        sleep_s = self.monotonic_deadline_s(sim_time_s) - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)


def _sensor_timestamp_s(
    *,
    clock: str,
    sim_clock_epoch_s: float,
    sim_time_s: float,
    wall_time_s: float,
) -> float:
    if str(clock or "sim") == "wall":
        return float(wall_time_s)
    return float(sim_clock_epoch_s) + float(sim_time_s)


def _scan_start_timestamp_s(
    *,
    clock: str,
    sim_clock_epoch_s: float,
    scan_start_sim_s: float,
    wall_scan_start_s: float,
) -> float:
    if str(clock or "sim") == "wall":
        return float(wall_scan_start_s)
    return float(sim_clock_epoch_s) + float(scan_start_sim_s)


def _uses_unified_sim_hardware_clock(*, timestamp_clock: str, imu_clock: str, lidar_clock: str) -> bool:
    return (
        str(timestamp_clock) == SIM_HARDWARE_CLOCK
        and str(imu_clock) == SIM_HARDWARE_CLOCK
        and str(lidar_clock) == SIM_HARDWARE_CLOCK
    )


def _scan_stamp_sim_time_s(*, scan_time_profile: str, scan_start_sim_s: float, scan_end_sim_s: float) -> float:
    if str(scan_time_profile or "").strip().lower() == "instantaneous":
        return float(scan_end_sim_s)
    return float(scan_start_sim_s)


def _scan_stamp_wall_time_s(*, scan_time_profile: str, wall_scan_start_s: float, wall_scan_end_s: float) -> float:
    if str(scan_time_profile or "").strip().lower() == "instantaneous":
        return float(wall_scan_end_s)
    return float(wall_scan_start_s)


def _bounded_points(points: Any, max_points: int) -> Any:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.shape[1] == 3:
        intensity = np.full((pts.shape[0], 1), 100.0, dtype=np.float32)
        pts = np.hstack([pts, intensity])
    elif pts.shape[1] > 4:
        pts = pts[:, :4]
    if max_points > 0 and pts.shape[0] > max_points:
        stride = int(np.ceil(pts.shape[0] / float(max_points)))
        pts = pts[::stride][:max_points]
    return pts.astype(np.float32, copy=False)


def _rolling_subscan_sample_count(
    *,
    frame_samples: int,
    imu_period_s: float,
    lidar_period_s: float,
) -> int:
    if lidar_period_s <= 0.0:
        return max(1, int(frame_samples))
    fraction = max(0.0, float(imu_period_s)) / float(lidar_period_s)
    return max(1, int(math.ceil(max(1, int(frame_samples)) * fraction)))


def _step_engine_for_sensor_tick(engine: Any, cmd: VelocityCommand, imu_period_s: float) -> Any:
    step_sensor_tick = getattr(engine, "step_sensor_tick", None)
    if callable(step_sensor_tick):
        return step_sensor_tick(cmd, dt_s=float(imu_period_s))
    return engine.step(cmd)


def _resolve_policy_path_for_drive(drive_mode: str, value: str) -> Path | None:
    text = str(value or "").strip()
    if text:
        candidate = Path(text).expanduser()
        if not candidate.is_absolute():
            candidate = (ROOT / candidate).resolve()
        return candidate
    if str(drive_mode or "").strip().lower() != "policy":
        return None
    for candidate in (DEFAULT_THUNDERV4_ONNX_POLICY, DEFAULT_THUNDERV4_TORCHSCRIPT_POLICY):
        if candidate.exists():
            return candidate.resolve()
    return None


def _make_report(
    *,
    ok: bool,
    duration_s: float,
    domain_id: int,
    sensor_counts: Counter[str],
    slam_counts: Counter[str],
    require_slam_output: bool,
    remaining_gaps: list[str],
    publisher: str = "",
    slam_status: dict[str, Any] | None = None,
    motion: dict[str, Any] | None = None,
    scan_time_profile: str = "",
    timestamp_clock: str = "",
    error: str = "",
) -> dict[str, Any]:
    return {
        "schema_version": "lingtu.mujoco_native_dds_sensors.v1",
        "ok": bool(ok),
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "no_python_slam": True,
        "python_role": "mujoco_sensor_dds_adapter_only",
        "native_sensor_publisher": publisher,
        "localization_runtime_expected": NATIVE_SLAM_RUNTIME,
        "domain_id": int(domain_id),
        "duration_s": float(duration_s),
        "published": dict(sensor_counts),
        "observed_slam_outputs": dict(slam_counts),
        "slam_status": slam_status or {},
        "motion": motion or {},
        "scan_time_profile": scan_time_profile,
        "timestamp_clock": timestamp_clock,
        "require_slam_output": bool(require_slam_output),
        "required_slam_output_topics": list(REQUIRED_SLAM_OUTPUT_TOPICS),
        "sensor_topics": {
            TOPICS.lidar_scan: {"dds_topic": "rt/lidar/raw_frame", "frame_id": LIDAR_FRAME_ID},
            TOPICS.imu: {"dds_topic": "rt/imu/raw", "frame_id": IMU_FRAME_ID},
            TOPICS.odom_prior: {"dds_topic": "rt/slam/odom_prior", "frame_id": "odom"},
        },
        "remaining_gaps": list(remaining_gaps),
        "error": error,
    }


def _remaining_gaps(
    *,
    sensor_counts: Counter[str],
    slam_counts: Counter[str],
    require_slam_output: bool,
    require_odom_prior: bool = True,
    drive_mode: str = "",
    allow_kinematic_fastlio_acceptance: bool = False,
) -> list[str]:
    gaps: list[str] = []
    for topic in (TOPICS.lidar_scan, TOPICS.imu):
        if sensor_counts.get(topic, 0) <= 0:
            gaps.append(f"sensor_not_published:{topic}")
    if require_odom_prior and sensor_counts.get(TOPICS.odom_prior, 0) <= 0:
        gaps.append(f"sensor_not_published:{TOPICS.odom_prior}")
    if require_slam_output:
        if (
            str(drive_mode or "").strip().lower() == "kinematic"
            and not bool(allow_kinematic_fastlio_acceptance)
        ):
            gaps.append("kinematic_fastlio_acceptance_disabled")
        for topic in REQUIRED_SLAM_OUTPUT_TOPICS:
            if slam_counts.get(topic, 0) <= 0:
                gaps.append(f"native_slam_output_missing:{topic}")
    return gaps


def _slam_health_gaps(
    status: dict[str, Any],
    *,
    min_quality: float,
    max_odom_abs_m: float,
    max_odom_z_abs_m: float,
) -> list[str]:
    if not status:
        return []
    gaps: list[str] = []
    state = str(status.get("state") or "").upper()
    if state != "TRACKING":
        gaps.append(f"native_slam_not_tracking:{state or 'unknown'}")
    quality = float(status.get("localization_quality") or 0.0)
    if quality < float(min_quality):
        gaps.append(f"native_slam_quality_low:{quality:.3f}")
    pose = ((status.get("odometry") or {}).get("pose") or {})
    coords = {
        axis: float(pose.get(axis) or 0.0)
        for axis in ("x", "y", "z")
    }
    if any(not math.isfinite(value) for value in coords.values()):
        gaps.append("native_slam_odom_not_finite")
    elif any(abs(value) > float(max_odom_abs_m) for value in coords.values()):
        gaps.append(
            "native_slam_odom_out_of_bounds:"
            f"x={coords['x']:.3f},y={coords['y']:.3f},z={coords['z']:.3f}"
        )
    if math.isfinite(coords["z"]) and abs(coords["z"]) > float(max_odom_z_abs_m):
        gaps.append(f"native_slam_odom_z_out_of_bounds:{coords['z']:.3f}")
    return gaps


def _slam_odom_pose(status: dict[str, Any]) -> dict[str, Any]:
    return ((status.get("odometry") or {}).get("pose") or {})


def _slam_odom_xy_m(status: dict[str, Any], *, reference_xy: Any = None) -> float:
    pose = _slam_odom_pose(status)
    try:
        x = float(pose.get("x") or 0.0)
        y = float(pose.get("y") or 0.0)
    except (TypeError, ValueError):
        return float("nan")
    if reference_xy is not None and bool(status.get("odom_prior_enabled")):
        try:
            ref = np.asarray(reference_xy, dtype=np.float64)
            if ref.size >= 2:
                x -= float(ref[0])
                y -= float(ref[1])
        except (TypeError, ValueError):
            pass
    return float(math.hypot(x, y))


def _slam_odom_yaw_rad(status: dict[str, Any], *, reference_yaw: float | None = None) -> float:
    pose = _slam_odom_pose(status)
    try:
        qx = float(pose.get("qx") or 0.0)
        qy = float(pose.get("qy") or 0.0)
        qz = float(pose.get("qz") or 0.0)
        qw = float(pose.get("qw") if pose.get("qw") is not None else 1.0)
    except (TypeError, ValueError):
        return float("nan")
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = float(math.atan2(siny_cosp, cosy_cosp))
    if reference_yaw is not None and bool(status.get("odom_prior_enabled")):
        return float(angle_delta_rad(yaw, float(reference_yaw)))
    return yaw


def _motion_report(
    *,
    sim_start_position: Any,
    sim_end_position: Any,
    sim_start_yaw: float | None,
    sim_end_yaw: float | None,
    sim_path_length_xy_m: float = 0.0,
    sim_abs_yaw_delta_rad: float = 0.0,
    slam_status: dict[str, Any],
    min_sim_motion_for_odom_check_m: float,
    min_slam_motion_ratio: float,
    max_slam_motion_ratio: float,
    min_sim_yaw_for_odom_check_rad: float,
    max_slam_yaw_error_rad: float,
) -> dict[str, Any]:
    if sim_start_position is None or sim_end_position is None:
        return {
            "available": False,
            "reason": "mujoco_state_not_sampled",
        }
    start = np.asarray(sim_start_position, dtype=np.float64)
    end = np.asarray(sim_end_position, dtype=np.float64)
    delta = end[:3] - start[:3]
    sim_xy_m = float(math.hypot(float(delta[0]), float(delta[1])))
    slam_xy_m = _slam_odom_xy_m(slam_status, reference_xy=start[:2])
    slam_yaw_rad = _slam_odom_yaw_rad(slam_status, reference_yaw=sim_start_yaw)
    ratio = (slam_xy_m / sim_xy_m) if sim_xy_m > 1e-9 and math.isfinite(slam_xy_m) else None
    yaw_delta = None
    yaw_error = None
    if sim_start_yaw is not None and sim_end_yaw is not None:
        yaw_delta = float(angle_delta_rad(float(sim_end_yaw), float(sim_start_yaw)))
        if math.isfinite(slam_yaw_rad):
            yaw_error = float(angle_delta_rad(float(slam_yaw_rad), yaw_delta))
    return {
        "available": True,
        "sim_start_xyz": [float(v) for v in start[:3]],
        "sim_end_xyz": [float(v) for v in end[:3]],
        "sim_delta_xyz": [float(v) for v in delta[:3]],
        "sim_xy_m": sim_xy_m,
        "sim_path_length_xy_m": float(sim_path_length_xy_m),
        "sim_yaw_delta_rad": yaw_delta,
        "sim_abs_yaw_delta_rad": float(sim_abs_yaw_delta_rad),
        "slam_odom_xy_m": slam_xy_m,
        "slam_to_sim_xy_ratio": ratio,
        "slam_odom_yaw_rad": slam_yaw_rad,
        "slam_to_sim_yaw_error_rad": yaw_error,
        "min_sim_motion_for_odom_check_m": float(min_sim_motion_for_odom_check_m),
        "min_slam_motion_ratio": float(min_slam_motion_ratio),
        "max_slam_motion_ratio": float(max_slam_motion_ratio),
        "min_sim_yaw_for_odom_check_rad": float(min_sim_yaw_for_odom_check_rad),
        "max_slam_yaw_error_rad": float(max_slam_yaw_error_rad),
    }


def _slam_motion_gaps(
    motion: dict[str, Any],
    *,
    min_sim_motion_for_odom_check_m: float,
    min_slam_motion_ratio: float,
    max_slam_motion_ratio: float,
    min_sim_yaw_for_odom_check_rad: float = 0.20,
    max_slam_yaw_error_rad: float = 0.15,
) -> list[str]:
    if not motion.get("available"):
        return []
    gaps: list[str] = []
    min_sim_motion = max(0.0, float(min_sim_motion_for_odom_check_m))
    sim_xy_m = float(motion.get("sim_xy_m") or 0.0)
    if min_sim_motion > 0.0 and sim_xy_m >= min_sim_motion:
        slam_xy_m = float(motion.get("slam_odom_xy_m") or 0.0)
        min_slam_xy_m = sim_xy_m * max(0.0, float(min_slam_motion_ratio))
        if not math.isfinite(slam_xy_m):
            gaps.append("native_slam_motion_not_finite")
        elif slam_xy_m < min_slam_xy_m:
            gaps.append(
                "native_slam_motion_mismatch:"
                f"sim_xy={sim_xy_m:.3f},slam_xy={slam_xy_m:.3f},min_slam_xy={min_slam_xy_m:.3f}"
            )
        else:
            max_ratio = max(0.0, float(max_slam_motion_ratio))
            if max_ratio > 0.0:
                max_slam_xy_m = sim_xy_m * max_ratio
                if slam_xy_m > max_slam_xy_m:
                    gaps.append(
                        "native_slam_motion_overshoot:"
                        f"sim_xy={sim_xy_m:.3f},slam_xy={slam_xy_m:.3f},max_slam_xy={max_slam_xy_m:.3f}"
                    )
    min_sim_yaw = max(0.0, float(min_sim_yaw_for_odom_check_rad))
    sim_yaw = motion.get("sim_yaw_delta_rad")
    yaw_error = motion.get("slam_to_sim_yaw_error_rad")
    if sim_yaw is not None and abs(float(sim_yaw)) >= min_sim_yaw:
        if yaw_error is None or not math.isfinite(float(yaw_error)):
            gaps.append("native_slam_yaw_not_finite")
        elif abs(float(yaw_error)) > max(0.0, float(max_slam_yaw_error_rad)):
            gaps.append(
                "native_slam_yaw_mismatch:"
                f"sim_yaw={float(sim_yaw):.3f},slam_yaw={float(motion.get('slam_odom_yaw_rad') or 0.0):.3f},"
                f"yaw_error={float(yaw_error):.3f},max_error={float(max_slam_yaw_error_rad):.3f}"
            )
    return gaps


def _publisher_candidates(value: str) -> list[Path]:
    if value:
        return [Path(value).expanduser()]
    if os.name == "nt":
        return [
            ROOT / "build" / "livox_sdk2_stream" / "Debug" / "livox_sdk2_stream.exe",
            ROOT / "build" / "livox_sdk2_stream" / "Release" / "livox_sdk2_stream.exe",
        ]
    return [
        ROOT / "build" / "livox_sdk2_stream" / "livox_sdk2_stream",
    ]


def _resolve_publisher_bin(value: str) -> Path:
    for candidate in _publisher_candidates(value):
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(
        "native DDS sensor publisher missing. Build it with: "
        "LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON bash scripts/build/build_livox_sdk2_stream.sh"
    )


def _start_native_publisher(args: argparse.Namespace) -> subprocess.Popen[bytes]:
    publisher = _resolve_publisher_bin(str(args.publisher_bin or ""))
    command = [
        str(publisher),
        "--stdin-records",
        "--dds",
        "--domain-id",
        str(int(args.domain_id)),
        "--lidar-frame",
        LIDAR_FRAME_ID,
        "--imu-frame",
        IMU_FRAME_ID,
    ]
    return subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
    )


def _write_record(stream: Any, record_type: int, timestamp_ns: int, sequence: int, payload: bytes, count: int) -> None:
    stream.write(_HEADER.pack(_MAGIC, int(record_type), int(timestamp_ns), int(sequence), int(count), len(payload)))
    stream.write(payload)


def _write_native_scan(stream: Any, scan: Any) -> None:
    payload = np.asarray(scan.points).tobytes()
    _write_record(
        stream,
        _RECORD_CLOUD,
        int(scan.timestamp_ns),
        int(scan.sequence),
        payload,
        int(scan.point_count),
    )


def _write_native_imu(stream: Any, imu: Imu, sequence: int) -> None:
    acc_scale = _MID360_ACCEL_MPS2_PER_G
    payload = _IMU_PAYLOAD.pack(
        float(imu.angular_velocity.x),
        float(imu.angular_velocity.y),
        float(imu.angular_velocity.z),
        float(imu.linear_acceleration.x) / acc_scale,
        float(imu.linear_acceleration.y) / acc_scale,
        float(imu.linear_acceleration.z) / acc_scale,
    )
    _write_record(stream, _RECORD_IMU, int(float(imu.ts) * 1_000_000_000), sequence, payload, 1)


def _write_native_odom_prior(stream: Any, state: Any, timestamp_s: float, sequence: int) -> None:
    position = np.asarray(getattr(state, "position", (0.0, 0.0, 0.0)), dtype=np.float64)
    orientation = np.asarray(getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)), dtype=np.float64)
    velocity = np.asarray(getattr(state, "linear_velocity", (0.0, 0.0, 0.0)), dtype=np.float64)
    if position.size < 3:
        position = np.pad(position, (0, 3 - position.size))
    if orientation.size < 4:
        orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    if velocity.size < 3:
        velocity = np.pad(velocity, (0, 3 - velocity.size))
    payload = _ODOM_PRIOR_PAYLOAD.pack(
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(orientation[0]),
        float(orientation[1]),
        float(orientation[2]),
        float(orientation[3]),
        float(velocity[0]),
        float(velocity[1]),
        float(velocity[2]),
        1,
    )
    _write_record(
        stream,
        _RECORD_ODOM_PRIOR,
        int(float(timestamp_s) * 1_000_000_000),
        sequence,
        payload,
        1,
    )


def _slam_status_counts(path: str) -> tuple[Counter[str], dict[str, Any]]:
    if not path:
        return Counter(), {}
    status_path = Path(path)
    if not status_path.exists():
        return Counter(), {}
    status = json.loads(status_path.read_text(encoding="utf-8"))
    counts: Counter[str] = Counter()
    if bool(status.get("has_odom")):
        counts[TOPICS.odometry] += 1
    if int(status.get("registered_points") or 0) > 0:
        counts[TOPICS.registered_cloud] += 1
    if int(status.get("map_points") or 0) > 0:
        counts[TOPICS.map_cloud] += 1
    if status.get("state"):
        counts[TOPICS.localization_health] += 1
    if float(status.get("localization_quality") or 0.0) > 0.0:
        counts[TOPICS.localization_quality] += 1
    return counts, status


def run(args: argparse.Namespace) -> dict[str, Any]:
    from sim.engine.core.engine import VelocityCommand

    duration_s = max(0.0, float(args.duration))
    settle_s = max(0.0, float(args.settle_s))
    warmup_s = max(0.0, float(args.warmup_s))
    drive_ramp_s = max(0.0, float(args.drive_ramp_s))
    imu_acc_axis_scale, imu_acc_axis_scale_source = _resolve_imu_acc_axis_scale(
        str(args.imu_acc_axis_scale),
        drive_mode=str(args.drive_mode),
        acc_mode=str(args.imu_acc_mode),
        timestamp_clock=str(args.timestamp_clock),
    )
    imu_gyro_axis_scale = _resolve_axis_scale(str(args.imu_gyro_axis_scale), "--imu-gyro-axis-scale")
    lidar_hz = max(0.1, float(args.publish_hz))
    imu_hz = max(lidar_hz, float(args.imu_hz))
    lidar_period_s = 1.0 / lidar_hz
    imu_period_s = 1.0 / imu_hz
    scan_duration_ns = int(1_000_000_000 * lidar_period_s)
    imu_acc_conditioner: SimImuSignalConditioner | None = None
    if str(args.imu_acc_mode) == "sensor" and str(args.imu_acc_conditioning) == "realistic":
        imu_acc_conditioner = SimImuSignalConditioner(
            lowpass_hz=float(args.imu_acc_lowpass_hz),
            max_dynamic_accel_mps2=float(args.imu_acc_max_dynamic_mps2),
            max_slew_rate_mps3=float(args.imu_acc_max_slew_mps3),
        )
    sensor_counts: Counter[str] = Counter()
    slam_counts: Counter[str] = Counter()
    publisher_path = _resolve_publisher_bin(str(args.publisher_bin or ""))
    publisher = _start_native_publisher(args)

    motion_log_path = str(getattr(args, "motion_log", "") or "")
    motion_log_stream = None
    motion_log_samples = 0
    if motion_log_path:
        log_path = Path(motion_log_path)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        motion_log_stream = log_path.open("w", encoding="utf-8")

    engine = None
    try:
        policy_path = _resolve_policy_path_for_drive(str(args.drive_mode), str(args.policy_path or ""))
        engine = build_engine(
            world=resolve_world(str(args.world)),
            drive_mode=str(args.drive_mode),
            n_rays=int(args.n_rays),
            start=parse_start(str(args.start or "")),
            mujoco_memory=str(args.mujoco_memory),
            mid360_pattern=args.mid360_pattern,
            mid360_samples_per_frame=int(args.mid360_samples_per_frame),
            lidar_backend=str(args.lidar_backend),
            mujoco_lidar_backend=str(args.mujoco_lidar_backend),
            require_product_lidar_backend=True,
            allow_legacy_lidar_fallback=bool(args.allow_legacy_lidar_fallback),
            policy_path=policy_path,
        )
        hold_cmd = VelocityCommand()
        if settle_s > 0.0:
            settle_end_s = float(getattr(engine, "sim_time", 0.0)) + settle_s
            while float(getattr(engine, "sim_time", 0.0)) + 1e-9 < settle_end_s:
                engine.step(hold_cmd)

        timestamp_clock = str(args.timestamp_clock)
        imu_timestamp_clock = str(args.imu_timestamp_clock or timestamp_clock)
        lidar_timestamp_clock = str(args.lidar_timestamp_clock or timestamp_clock)
        sim_start_s = float(getattr(engine, "sim_time", 0.0))
        hardware_clock = SimulatedHardwareClock(
            sim_start_s=sim_start_s,
            wall_epoch_s=time.time(),
            monotonic_start_s=time.monotonic(),
            realtime_factor=float(args.sim_hardware_realtime_factor),
        )
        sim_clock_epoch_s = hardware_clock.wall_epoch_s
        unified_sim_hardware_clock = _uses_unified_sim_hardware_clock(
            timestamp_clock=timestamp_clock,
            imu_clock=imu_timestamp_clock,
            lidar_clock=lidar_timestamp_clock,
        )
        drive_vx = float(args.drive_vx)
        drive_vy = float(args.drive_vy)
        drive_wz = float(args.drive_wz)
        drive_profile = str(args.drive_profile)
        run_start_s = time.monotonic()
        drive_start_s = run_start_s + warmup_s
        deadline = drive_start_s + duration_s
        sequence = 0
        imu_sequence = 0
        odom_prior_sequence = 0
        prev_imu_s = None
        prev_velocity = None
        next_lidar_s = 0.0
        sim_start_position = None
        sim_end_position = None
        sim_start_yaw = None
        sim_end_yaw = None
        sim_path_length_xy_m = 0.0
        sim_abs_yaw_delta_rad = 0.0
        prev_drive_position = None
        prev_drive_yaw = None
        subscan_samples: list[tuple[float, Any, Any, int]] = []
        rolling_subscan_samples: int | None = None
        if str(args.physical_rolling_sample_mode) == "subscan":
            rolling_subscan_samples = _rolling_subscan_sample_count(
                frame_samples=int(args.mid360_samples_per_frame),
                imu_period_s=imu_period_s,
                lidar_period_s=lidar_period_s,
            )
        next_lidar_sim_s = sim_start_s
        while True:
            loop_start = time.monotonic()
            if unified_sim_hardware_clock:
                sim_elapsed_before_step_s = max(0.0, float(getattr(engine, "sim_time", 0.0)) - sim_start_s)
                if sim_elapsed_before_step_s >= warmup_s + duration_s:
                    break
                driving = sim_elapsed_before_step_s >= warmup_s
                drive_elapsed_s = max(0.0, sim_elapsed_before_step_s - warmup_s)
            else:
                if loop_start >= deadline:
                    break
                driving = loop_start >= drive_start_s
                drive_elapsed_s = max(0.0, loop_start - drive_start_s) if driving else 0.0
            if driving:
                ramp = 1.0 if drive_ramp_s <= 0.0 else min(1.0, drive_elapsed_s / drive_ramp_s)
                profile_vx, profile_vy, profile_wz = _drive_command_for_profile(
                    drive_profile,
                    drive_elapsed_s,
                    drive_vx=drive_vx,
                    drive_vy=drive_vy,
                    drive_wz=drive_wz,
                )
                cmd = VelocityCommand(
                    linear_x=profile_vx * ramp,
                    linear_y=profile_vy * ramp,
                    angular_z=profile_wz * ramp,
                )
            else:
                cmd = hold_cmd
            state = _step_engine_for_sensor_tick(engine, cmd, imu_period_s)
            sim_time_s = float(getattr(engine, "sim_time", 0.0))
            position = np.asarray(state.position, dtype=np.float64).copy()
            yaw = yaw_from_quat_xyzw(state.orientation)
            if driving and sim_start_position is None:
                sim_start_position = position.copy()
                sim_start_yaw = float(yaw)
                prev_drive_position = position.copy()
                prev_drive_yaw = float(yaw)
            if driving:
                if prev_drive_position is not None:
                    drive_delta = position[:2] - prev_drive_position[:2]
                    sim_path_length_xy_m += float(math.hypot(float(drive_delta[0]), float(drive_delta[1])))
                if prev_drive_yaw is not None:
                    sim_abs_yaw_delta_rad += abs(float(angle_delta_rad(float(yaw), float(prev_drive_yaw))))
                prev_drive_position = position.copy()
                prev_drive_yaw = float(yaw)
                sim_end_position = position.copy()
                sim_end_yaw = float(yaw)
            wall_ts_s = time.time()
            sensor_ts_s = _sensor_timestamp_s(
                clock=imu_timestamp_clock,
                sim_clock_epoch_s=sim_clock_epoch_s,
                sim_time_s=sim_time_s,
                wall_time_s=wall_ts_s,
            )
            imu_dt = 0.0 if prev_imu_s is None else max(0.0, sensor_ts_s - prev_imu_s)
            imu = _runtime_imu_from_state(
                state,
                sensor_ts_s,
                prev_velocity,
                imu_dt,
                str(args.imu_acc_mode),
                acc_conditioner=imu_acc_conditioner,
            )
            _apply_imu_acc_axis_scale(imu, imu_acc_axis_scale)
            _apply_imu_gyro_axis_scale(imu, imu_gyro_axis_scale)
            if publisher.stdin is None:
                raise RuntimeError("native publisher stdin closed")
            _write_native_imu(publisher.stdin, imu, imu_sequence)
            sensor_counts[TOPICS.imu] += 1
            imu_sequence += 1
            if bool(args.publish_odom_prior):
                _write_native_odom_prior(publisher.stdin, state, sensor_ts_s, odom_prior_sequence)
                sensor_counts[TOPICS.odom_prior] += 1
                odom_prior_sequence += 1
            prev_imu_s = sensor_ts_s
            prev_velocity = np.asarray(state.linear_velocity, dtype=np.float64).copy()
            if str(args.scan_time_profile) == "physical_rolling":
                sample_world_points = _bounded_points(
                    engine.get_lidar_points(sample_count=rolling_subscan_samples),
                    int(args.max_points),
                )
                sample_sensor_points = world_xyzi_to_sensor_xyzi(engine, sample_world_points)
                subscan_samples.append((sim_time_s, sample_sensor_points, sample_world_points, 0))
                min_sample_time = sim_time_s - lidar_period_s * 1.5
                subscan_samples = [sample for sample in subscan_samples if sample[0] >= min_sample_time]
            lidar_due = (
                sim_time_s + 1e-9 >= next_lidar_sim_s
                if unified_sim_hardware_clock
                else loop_start >= next_lidar_s
            )
            if lidar_due:
                scan_start_sim_s = max(0.0, sim_time_s - lidar_period_s)
                scan_start_wall_s = wall_ts_s - min(lidar_period_s, max(0.0, sim_time_s - scan_start_sim_s))
                scan_stamp_sim_s = _scan_stamp_sim_time_s(
                    scan_time_profile=str(args.scan_time_profile),
                    scan_start_sim_s=scan_start_sim_s,
                    scan_end_sim_s=sim_time_s,
                )
                scan_stamp_wall_s = _scan_stamp_wall_time_s(
                    scan_time_profile=str(args.scan_time_profile),
                    wall_scan_start_s=scan_start_wall_s,
                    wall_scan_end_s=wall_ts_s,
                )
                scan_start_timestamp_s = _scan_start_timestamp_s(
                    clock=lidar_timestamp_clock,
                    sim_clock_epoch_s=sim_clock_epoch_s,
                    scan_start_sim_s=scan_stamp_sim_s,
                    wall_scan_start_s=scan_stamp_wall_s,
                )
                if str(args.scan_time_profile) == "physical_rolling":
                    sensor_points, _world_points, relative_times_s, _moving_count, _subscan_count = (
                        _physical_rolling_scan_from_samples(
                            subscan_samples,
                            scan_start_s=scan_start_sim_s,
                            scan_end_s=sim_time_s,
                        )
                    )
                    if int(args.max_points) > 0 and len(sensor_points) > int(args.max_points):
                        stride = int(np.ceil(len(sensor_points) / float(args.max_points)))
                        sensor_points = sensor_points[::stride][: int(args.max_points)]
                        relative_times_s = relative_times_s[::stride][: int(args.max_points)]
                else:
                    world_points = _bounded_points(engine.get_lidar_points(), int(args.max_points))
                    sensor_points = world_xyzi_to_sensor_xyzi(engine, world_points)
                    relative_times_s = _relative_times_for_scan(
                        len(sensor_points),
                        lidar_period_s,
                        scan_time_profile=str(args.scan_time_profile),
                    )
                scan = _xyzi_to_livox_frame(
                    sensor_points,
                    timestamp_ns=int(scan_start_timestamp_s * 1_000_000_000),
                    sequence=sequence,
                    frame_id=LIDAR_FRAME_ID,
                    scan_duration_ns=scan_duration_ns,
                    offset_time_ns=(relative_times_s * 1_000_000_000).astype(np.uint64),
                )
                _write_native_scan(publisher.stdin, scan)
                sensor_counts[TOPICS.lidar_scan] += 1
                sequence += 1
                if motion_log_stream is not None:
                    motion_log_stream.write(
                        json.dumps(
                            {
                                "t": float(sensor_ts_s),
                                "sim_time_s": float(sim_time_s),
                                "x": float(position[0]),
                                "y": float(position[1]),
                                "z": float(position[2]),
                                "yaw": float(yaw),
                                "driving": bool(driving),
                            },
                            ensure_ascii=True,
                        )
                        + "\n"
                    )
                    motion_log_samples += 1
                    if motion_log_samples % 20 == 0:
                        motion_log_stream.flush()
                if unified_sim_hardware_clock:
                    next_lidar_sim_s += lidar_period_s
                    while next_lidar_sim_s <= sim_time_s - 1e-9:
                        next_lidar_sim_s += lidar_period_s
                else:
                    next_lidar_s = loop_start + lidar_period_s
            publisher.stdin.flush()
            if publisher.poll() is not None:
                raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
            if unified_sim_hardware_clock:
                hardware_clock.sleep_until(sim_time_s)
            else:
                time.sleep(max(0.0, imu_period_s - (time.monotonic() - loop_start)))
    finally:
        if motion_log_stream is not None:
            motion_log_stream.close()
        if publisher.stdin is not None:
            publisher.stdin.close()
        try:
            publisher.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            publisher.terminate()
            publisher.wait(timeout=3.0)
        if engine is not None:
            engine.close()

    status_counts, slam_status = _slam_status_counts(str(args.slam_status_json or ""))
    slam_counts.update(status_counts)
    motion = _motion_report(
        sim_start_position=sim_start_position,
        sim_end_position=sim_end_position,
        sim_start_yaw=sim_start_yaw,
        sim_end_yaw=sim_end_yaw,
        sim_path_length_xy_m=sim_path_length_xy_m,
        sim_abs_yaw_delta_rad=sim_abs_yaw_delta_rad,
        slam_status=slam_status,
        min_sim_motion_for_odom_check_m=float(args.min_sim_motion_for_odom_check_m),
        min_slam_motion_ratio=float(args.min_slam_motion_ratio),
        max_slam_motion_ratio=float(args.max_slam_motion_ratio),
        min_sim_yaw_for_odom_check_rad=float(args.min_sim_yaw_for_odom_check_rad),
        max_slam_yaw_error_rad=float(args.max_slam_yaw_error_rad),
    )
    gaps = _remaining_gaps(
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=bool(args.require_slam_output),
        require_odom_prior=bool(args.publish_odom_prior),
        drive_mode=str(args.drive_mode),
        allow_kinematic_fastlio_acceptance=bool(args.allow_kinematic_fastlio_acceptance),
    )
    if args.require_slam_output and not str(args.slam_status_json or ""):
        gaps.append("slam_status_json_not_configured")
    if args.require_slam_output:
        gaps.extend(
            _slam_health_gaps(
                slam_status,
                min_quality=float(args.min_localization_quality),
                max_odom_abs_m=float(args.max_odom_abs_m),
                max_odom_z_abs_m=float(args.max_odom_z_abs_m),
            )
        )
        gaps.extend(
            _slam_motion_gaps(
                motion,
                min_sim_motion_for_odom_check_m=float(args.min_sim_motion_for_odom_check_m),
                min_slam_motion_ratio=float(args.min_slam_motion_ratio),
                max_slam_motion_ratio=float(args.max_slam_motion_ratio),
                min_sim_yaw_for_odom_check_rad=float(args.min_sim_yaw_for_odom_check_rad),
                max_slam_yaw_error_rad=float(args.max_slam_yaw_error_rad),
            )
        )
    report = _make_report(
        ok=not gaps,
        duration_s=duration_s,
        domain_id=int(args.domain_id),
        sensor_counts=sensor_counts,
        slam_counts=slam_counts,
        require_slam_output=bool(args.require_slam_output),
        remaining_gaps=gaps,
        publisher=str(publisher_path),
        slam_status=slam_status,
        motion=motion,
        scan_time_profile=str(args.scan_time_profile),
        timestamp_clock=str(args.timestamp_clock),
    )
    report["warmup_s"] = warmup_s
    report["settle_s"] = settle_s
    report["drive_ramp_s"] = drive_ramp_s
    report["drive_duration_s"] = duration_s
    report["drive_mode"] = str(args.drive_mode)
    report["drive_profile"] = str(args.drive_profile)
    report["policy_path"] = str(policy_path) if policy_path is not None else ""
    report["imu_acc_mode"] = str(args.imu_acc_mode)
    if str(args.imu_acc_mode) == "gravity_only":
        sensor_model = "fastlio_gravity_only_imu"
    elif str(args.imu_acc_mode) == "sensor" and imu_acc_conditioner is not None:
        sensor_model = "mujoco_accelerometer_conditioned_imu"
    elif str(args.imu_acc_mode) == "sensor":
        sensor_model = "mujoco_accelerometer_raw_imu"
    else:
        sensor_model = "legacy_finite_difference_imu"
    report["sim_hardware_sensor_model"] = sensor_model
    report["imu_acc_conditioning"] = str(args.imu_acc_conditioning)
    if imu_acc_conditioner is not None:
        report["imu_acc_conditioner"] = imu_acc_conditioner.stats()
    else:
        report["imu_acc_conditioner"] = {"enabled": False}
    report["allow_kinematic_fastlio_acceptance"] = bool(args.allow_kinematic_fastlio_acceptance)
    report["publish_odom_prior"] = bool(args.publish_odom_prior)
    report["physical_rolling_sample_mode"] = str(args.physical_rolling_sample_mode)
    report["imu_acc_axis_scale"] = [float(v) for v in imu_acc_axis_scale]
    report["imu_acc_axis_scale_source"] = imu_acc_axis_scale_source
    report["imu_gyro_axis_scale"] = [float(v) for v in imu_gyro_axis_scale]
    report["imu_timestamp_clock"] = str(args.imu_timestamp_clock or args.timestamp_clock)
    report["lidar_timestamp_clock"] = str(args.lidar_timestamp_clock or args.timestamp_clock)
    report["clock_profile"] = "sim_hardware" if unified_sim_hardware_clock else "legacy_split_or_wall"
    report["sim_hardware_realtime_factor"] = float(args.sim_hardware_realtime_factor)
    report["motion_log"] = motion_log_path
    report["motion_log_samples"] = int(motion_log_samples)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        report = run(args)
    except Exception as exc:
        report = _make_report(
            ok=False,
            duration_s=float(getattr(args, "duration", 0.0) or 0.0),
            domain_id=int(getattr(args, "domain_id", 0) or 0),
            sensor_counts=Counter(),
            slam_counts=Counter(),
            require_slam_output=bool(getattr(args, "require_slam_output", False)),
            remaining_gaps=[f"mujoco_native_dds_sensor_bridge_failed:{type(exc).__name__}"],
            publisher=str(getattr(args, "publisher_bin", "") or NATIVE_SENSOR_PUBLISHER),
            scan_time_profile=str(getattr(args, "scan_time_profile", "") or ""),
            timestamp_clock=str(getattr(args, "timestamp_clock", "") or ""),
            error=str(exc),
        )
    text = json.dumps(report, ensure_ascii=True, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
