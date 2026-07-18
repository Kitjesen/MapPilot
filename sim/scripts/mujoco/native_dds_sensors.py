#!/usr/bin/env python3
"""Publish MuJoCo MID-360/IMU raw sensors to the native DDS SLAM boundary.

This script is a simulator sensor bridge, not SLAM. It never estimates pose,
builds a map, or relocalizes. The optional navigation fixture publishes MuJoCo
ground truth as native DDS navigation inputs so navigation can be accepted
independently from SLAM accuracy and rolling-scan performance.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import struct
import subprocess
import sys
import threading
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

from sim.scripts.mujoco_live.motion import (
    _physical_rolling_scan_from_samples,
    _relative_times_for_scan,
)
from sim.scripts.mujoco.async_jsonl_writer import AsyncJsonlWriter

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
    quat_xyzw_to_matrix,
    specific_force_body,
    world_xyzi_to_sensor_xyzi,
    yaw_from_quat_xyzw,
)
from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from message.livox_frame import POINT_DTYPE

NATIVE_SLAM_RUNTIME = "lingtu_slam_cyclone_runtime"
NATIVE_SENSOR_PUBLISHER = "livox_sdk2_stream --stdin-records --dds"
LIDAR_FRAME_ID = topic_default_frame_id(TOPICS.lidar_scan)
IMU_FRAME_ID = topic_default_frame_id(TOPICS.imu)
_MAGIC = b"LTU1"
_RECORD_CLOUD = 1
_RECORD_IMU = 2
_RECORD_ODOM_PRIOR = 3
_RECORD_REGISTERED_CLOUD = 4
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
# MuJoCo contact impulses are substantially sharper than the mechanically
# filtered signal observed by a lidar-mounted IMU. These values retain body
# acceleration while rejecting solver impulses that destabilize deskew/LIO.
DEFAULT_IMU_ACC_LOWPASS_HZ = 5.0
DEFAULT_IMU_ACC_MAX_DYNAMIC_MPS2 = 1.5
DEFAULT_IMU_ACC_MAX_SLEW_MPS3 = 30.0
DEFAULT_SIM_HARDWARE_MAX_LAG_S = 0.05
DEFAULT_SIM_HARDWARE_CATCH_UP_YIELD_STEPS = 40
DEFAULT_ODOM_PRIOR_VELOCITY_WINDOW_S = 0.10


class OdomPriorVelocityEstimator:
    """Estimate map-frame velocity from a short pose window.

    MuJoCo generalized velocity can contain solver-scale contact impulses that
    are not present in the base pose trajectory.  The simulation-only odometry
    prior therefore uses the same observable quantity as odometry: position
    change over time.  Values are intentionally not clipped so real pose jumps
    still exercise the native SLAM prior safety gates.
    """

    def __init__(self, *, window_s: float = DEFAULT_ODOM_PRIOR_VELOCITY_WINDOW_S) -> None:
        self.window_s = max(0.02, float(window_s))
        self._samples: list[tuple[float, np.ndarray]] = []
        self._last_velocity = np.zeros(3, dtype=np.float64)
        self._last_valid = False
        self._max_speed_mps = 0.0
        self._max_velocity = np.zeros(3, dtype=np.float64)
        self._max_velocity_stamp_s = 0.0

    def update(self, position: Any, timestamp_s: float) -> tuple[np.ndarray, bool]:
        stamp = float(timestamp_s)
        pose = np.asarray(position, dtype=np.float64).reshape(-1)
        if pose.size < 3 or not math.isfinite(stamp) or not np.all(np.isfinite(pose[:3])):
            self._samples.clear()
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False

        if self._samples and stamp <= self._samples[-1][0]:
            self._samples.clear()
        self._samples.append((stamp, pose[:3].copy()))

        cutoff = stamp - self.window_s
        first_inside = next(
            (index for index, (sample_stamp, _) in enumerate(self._samples) if sample_stamp >= cutoff),
            len(self._samples) - 1,
        )
        keep_from = max(0, first_inside - 1)
        if keep_from:
            self._samples = self._samples[keep_from:]

        first_stamp, first_pose = self._samples[0]
        span_s = stamp - first_stamp
        min_span_s = max(0.02, 0.5 * self.window_s)
        if span_s + 1e-12 < min_span_s:
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False

        # Use robust centres of the older and newer half-windows. A MuJoCo
        # contact solve can displace the reported base pose for one 5 ms tick;
        # endpoint differencing turns that single sample into a false multi-m/s
        # velocity. Medians reject that impulse while a persistent teleport or
        # genuinely fast trajectory still moves the newer half-window centre.
        split = max(1, len(self._samples) // 2)
        older = self._samples[:split]
        newer = self._samples[split:]
        if not newer:
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False
        older_stamp = float(np.median([sample_stamp for sample_stamp, _ in older]))
        newer_stamp = float(np.median([sample_stamp for sample_stamp, _ in newer]))
        robust_span_s = newer_stamp - older_stamp
        if robust_span_s <= 1e-12:
            self._last_velocity.fill(0.0)
            self._last_valid = False
            return np.zeros(3, dtype=np.float64), False
        older_pose = np.median(np.stack([sample_pose for _, sample_pose in older]), axis=0)
        newer_pose = np.median(np.stack([sample_pose for _, sample_pose in newer]), axis=0)
        velocity = (newer_pose - older_pose) / robust_span_s
        self._last_velocity = velocity.astype(np.float64, copy=True)
        self._last_valid = True
        speed_mps = float(np.linalg.norm(velocity))
        if speed_mps > self._max_speed_mps:
            self._max_speed_mps = speed_mps
            self._max_velocity = self._last_velocity.copy()
            self._max_velocity_stamp_s = stamp
        return self._last_velocity.copy(), True

    def stats(self) -> dict[str, Any]:
        return {
            "source": "robust_pose_window",
            "window_s": self.window_s,
            "sample_count": len(self._samples),
            "valid": self._last_valid,
            "velocity_mps": self._last_velocity.astype(float).tolist(),
            "max_speed_mps": self._max_speed_mps,
            "max_velocity_mps": self._max_velocity.astype(float).tolist(),
            "max_velocity_stamp_s": self._max_velocity_stamp_s,
        }


class RuntimeStageProfiler:
    """Low-overhead wall-time profiler for the simulation sensor loop."""

    def __init__(self, *, slow_threshold_s: float = 0.05, max_slow_events: int = 32) -> None:
        self.slow_threshold_s = max(0.0, float(slow_threshold_s))
        self.max_slow_events = max(1, int(max_slow_events))
        self._stages: dict[str, dict[str, float | int]] = {}
        self._slow_events: list[dict[str, float | str]] = []

    def record(self, stage: str, elapsed_s: float, sim_time_s: float) -> None:
        elapsed = max(0.0, float(elapsed_s))
        name = str(stage)
        stats = self._stages.setdefault(
            name,
            {
                "count": 0,
                "total_s": 0.0,
                "max_s": 0.0,
                "max_sim_time_s": 0.0,
            },
        )
        stats["count"] = int(stats["count"]) + 1
        stats["total_s"] = float(stats["total_s"]) + elapsed
        if elapsed > float(stats["max_s"]):
            stats["max_s"] = elapsed
            stats["max_sim_time_s"] = float(sim_time_s)
        if elapsed >= self.slow_threshold_s:
            self._slow_events.append({"stage": name, "elapsed_s": elapsed, "sim_time_s": float(sim_time_s)})
            self._slow_events.sort(key=lambda event: float(event["elapsed_s"]), reverse=True)
            del self._slow_events[self.max_slow_events :]

    def stats(self) -> dict[str, Any]:
        stages: dict[str, Any] = {}
        for name, values in sorted(self._stages.items()):
            count = int(values["count"])
            total_s = float(values["total_s"])
            stages[name] = {
                **values,
                "mean_s": total_s / float(count) if count > 0 else 0.0,
            }
        return {
            "slow_threshold_s": self.slow_threshold_s,
            "stages": stages,
            "slow_events": list(self._slow_events),
        }


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
    parser.add_argument(
        "--start-anchor",
        choices=["off", "warmup", "run"],
        default="off",
        help=(
            "Keep the MuJoCo base at the configured start pose during SLAM "
            "initialization, or for the complete no-motion run. Motion "
            "acceptance must use warmup so the base is released before driving."
        ),
    )
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
        "--navigation-fixture",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Publish MuJoCo truth odometry, identity map-to-odom TF, healthy "
            "localization state, and a body-frame registered cloud through the "
            "native DDS publisher. This is only for navigation functional acceptance."
        ),
    )
    parser.add_argument(
        "--navigation-fixture-cloud-points",
        type=int,
        default=4000,
        help="Maximum body-frame registered-cloud points per navigation fixture frame.",
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
        "--sim-hardware-max-lag-s",
        type=float,
        default=DEFAULT_SIM_HARDWARE_MAX_LAG_S,
        help=(
            "Maximum simulated-hardware lag behind its wall pacing target before "
            "intermediate sensor ticks are dropped while MuJoCo dynamics continue."
        ),
    )
    parser.add_argument(
        "--odom-prior-velocity-window-s",
        type=float,
        default=DEFAULT_ODOM_PRIOR_VELOCITY_WINDOW_S,
        help=(
            "Pose-difference window used for the simulation-only odometry prior velocity. "
            "This rejects MuJoCo contact-solver qvel impulses without clipping real pose jumps."
        ),
    )
    parser.add_argument(
        "--sim-hardware-catch-up-yield-steps",
        type=int,
        default=DEFAULT_SIM_HARDWARE_CATCH_UP_YIELD_STEPS,
        help=(
            "Yield to peer DDS processes after this many consecutive catch-up "
            "physics steps. Physics is never time-jumped."
        ),
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
        "--max-slam-map-xy-error-m",
        type=float,
        default=0.35,
        help="Maximum map-frame localization XY error when map tracking is active.",
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
        "--viewer",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Open a passive MuJoCo viewer. The viewer is presentation-only; "
            "motion still comes from --command-source and the physics loop."
        ),
    )
    parser.add_argument(
        "--viewer-hz",
        type=float,
        default=30.0,
        help="Maximum passive-viewer refresh rate.",
    )
    parser.add_argument(
        "--command-source",
        choices=["profile", "dds"],
        default="profile",
        help=(
            "Velocity source for the MuJoCo engine. profile uses the deterministic "
            "diagnostic command below; dds consumes the final typed-DDS /nav/cmd_vel "
            "through the Linux C++ tap and is required for native navigation acceptance."
        ),
    )
    parser.add_argument(
        "--cmd-vel-tap-bin",
        default=os.environ.get("LINGTU_MUJOCO_CMD_VEL_TAP_BIN", ""),
        help="Linux C++ typed-DDS /nav/cmd_vel tap used by --command-source dds.",
    )
    parser.add_argument("--cmd-vel-pid-file", default="")
    parser.add_argument("--publisher-pid-file", default="")
    parser.add_argument(
        "--cmd-vel-timeout-s",
        type=float,
        default=0.25,
        help="Fail-safe age after which the last DDS command is replaced by zero velocity.",
    )
    parser.add_argument(
        "--require-cmd-vel",
        action="store_true",
        help="Fail the sensor/locomotion report unless a non-zero DDS command was consumed.",
    )
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
    parser.add_argument("--max-points", type=int, default=DEFAULT_MID360_SAMPLES_PER_FRAME)
    parser.add_argument(
        "--require-slam-output",
        action="store_true",
        help="Fail unless native C++ SLAM publishes odometry, map cloud, and health.",
    )
    parser.add_argument(
        "--motion-log",
        default="",
        help=(
            "Optional JSONL path for periodic MuJoCo ground-truth, native nav, and live LiDAR visualization samples."
        ),
    )
    parser.add_argument(
        "--motion-log-hz",
        type=float,
        default=24.0,
        help="Simulation-time sampling rate for --motion-log.",
    )
    parser.add_argument(
        "--motion-log-lidar-points",
        type=int,
        default=640,
        help="Maximum live world-frame LiDAR points stored in each motion-log sample.",
    )
    parser.add_argument(
        "--nav-status-json",
        default="",
        help="Native nav endpoint status snapshot used only for acceptance visualization logging.",
    )
    parser.add_argument(
        "--stop-on-nav-goal-reached",
        action="store_true",
        help="End the simulation cleanly once native navigation reports goal_reached.",
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
        if str(drive_mode).strip().lower() == "kinematic" and str(acc_mode).strip().lower() == "finite_difference":
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

    def expected_sim_time_s(self, monotonic_now_s: float | None = None) -> float:
        now_s = time.monotonic() if monotonic_now_s is None else float(monotonic_now_s)
        wall_elapsed_s = max(0.0, now_s - self.monotonic_start_s)
        return self.sim_start_s + wall_elapsed_s * self.realtime_factor

    def lag_s(self, sim_time_s: float, monotonic_now_s: float | None = None) -> float:
        return self.expected_sim_time_s(monotonic_now_s) - float(sim_time_s)

    def sleep_until(self, sim_time_s: float) -> None:
        sleep_s = self.monotonic_deadline_s(sim_time_s) - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)


class SimHardwareCatchUpController:
    """Drop only sensor observations while advancing every MuJoCo physics tick.

    A fixed simulated-hardware epoch is useful only while simulated time stays
    close to the wall pacing target. Under host load, publishing every overdue
    observation preserves sample count but makes every source timestamp stale.
    This controller instead accounts for omitted observations and lets the
    normal small-step dynamics catch back up without changing timestamp rules.
    """

    def __init__(
        self,
        *,
        clock: SimulatedHardwareClock,
        max_lag_s: float = DEFAULT_SIM_HARDWARE_MAX_LAG_S,
        yield_every_steps: int = DEFAULT_SIM_HARDWARE_CATCH_UP_YIELD_STEPS,
    ) -> None:
        if not math.isfinite(float(max_lag_s)) or float(max_lag_s) < 0.0:
            raise ValueError("--sim-hardware-max-lag-s must be non-negative and finite")
        if int(yield_every_steps) <= 0:
            raise ValueError("--sim-hardware-catch-up-yield-steps must be positive")
        self.clock = clock
        self.max_lag_s = float(max_lag_s)
        self.yield_every_steps = int(yield_every_steps)
        self.catch_up_events = 0
        self.dropped_imu_ticks = 0
        self.dropped_lidar_subscan_ticks = 0
        self.dropped_lidar_frames = 0
        self.fast_static_clock_ticks = 0
        self.dynamic_catch_up_ticks = 0
        self.catch_up_yields = 0
        self.max_lag_observed_s = 0.0
        self.max_consecutive_steps = 0
        self._consecutive_steps = 0
        self._catching_up = False

    def should_drop_sensor_tick(
        self,
        sim_time_s: float,
        *,
        monotonic_now_s: float | None = None,
    ) -> bool:
        lag_s = self.clock.lag_s(sim_time_s, monotonic_now_s)
        self.max_lag_observed_s = max(self.max_lag_observed_s, lag_s)
        if lag_s <= self.max_lag_s:
            self._catching_up = False
            self._consecutive_steps = 0
            return False
        if not self._catching_up:
            self.catch_up_events += 1
        self._catching_up = True
        self._consecutive_steps += 1
        self.max_consecutive_steps = max(self.max_consecutive_steps, self._consecutive_steps)
        self.dropped_imu_ticks += 1
        return True

    def record_lidar_subscan_drop(self) -> None:
        self.dropped_lidar_subscan_ticks += 1

    def record_lidar_frame_drops(self, count: int) -> None:
        self.dropped_lidar_frames += max(0, int(count))

    def record_catch_up_step(self, *, fast_static_clock: bool) -> None:
        if fast_static_clock:
            self.fast_static_clock_ticks += 1
        else:
            self.dynamic_catch_up_ticks += 1

    def yield_if_due(self) -> None:
        if self._consecutive_steps > 0 and self._consecutive_steps % self.yield_every_steps == 0:
            self.catch_up_yields += 1
            time.sleep(0.0)

    def stats(self, sim_time_s: float, *, monotonic_now_s: float | None = None) -> dict[str, Any]:
        final_lag_s = self.clock.lag_s(sim_time_s, monotonic_now_s)
        return {
            "enabled": True,
            "strategy": "small_step_dynamics_drop_intermediate_sensor_ticks",
            "max_lag_s": self.max_lag_s,
            "yield_every_steps": self.yield_every_steps,
            "catch_up_events": self.catch_up_events,
            "catch_up_yields": self.catch_up_yields,
            "dropped_imu_ticks": self.dropped_imu_ticks,
            "dropped_lidar_subscan_ticks": self.dropped_lidar_subscan_ticks,
            "dropped_lidar_frames": self.dropped_lidar_frames,
            "fast_static_clock_ticks": self.fast_static_clock_ticks,
            "dynamic_catch_up_ticks": self.dynamic_catch_up_ticks,
            "max_lag_observed_s": self.max_lag_observed_s,
            "max_consecutive_steps": self.max_consecutive_steps,
            "final_lag_s": final_lag_s,
        }


def _advance_skipped_lidar_deadlines(
    *,
    next_lidar_sim_s: float,
    lidar_period_s: float,
    sim_time_s: float,
) -> tuple[float, int]:
    """Advance scheduled LiDAR frames crossed by a sensor-drop physics step."""

    next_due_s = float(next_lidar_sim_s)
    period_s = float(lidar_period_s)
    if not math.isfinite(period_s) or period_s <= 0.0:
        raise ValueError("lidar period must be positive and finite")
    dropped = 0
    while sim_time_s + 1e-9 >= next_due_s:
        next_due_s += period_s
        dropped += 1
    return next_due_s, dropped


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


_MOTION_LOG_LIDAR_MAX_Z_ABOVE_ROBOT_M = 1.8
_MOTION_LOG_LIDAR_LOCAL_RADIUS_M = 4.0
_MOTION_LOG_LIDAR_LOCAL_SHARE = 0.75


def _motion_log_lidar_sample(
    points: Any,
    *,
    max_points: int,
    robot_xyz: Any,
) -> np.ndarray:
    """Keep a dense local scan while retaining some wider scene context."""

    pts = _bounded_points(points, 0)
    if pts.shape[0] <= 0 or max_points <= 0:
        return np.zeros((0, 4), dtype=np.float32)
    robot = np.asarray(robot_xyz, dtype=np.float64).reshape(-1)
    if robot.size < 3 or not np.isfinite(robot[:3]).all():
        robot = np.zeros(3, dtype=np.float64)
    finite = np.isfinite(pts[:, :3]).all(axis=1)
    below_presentation_ceiling = (
        pts[:, 2]
        <= float(robot[2]) + _MOTION_LOG_LIDAR_MAX_Z_ABOVE_ROBOT_M
    )
    visible = pts[finite & below_presentation_ceiling]
    if visible.shape[0] <= max_points:
        return visible.astype(np.float32, copy=False)

    delta_xy = visible[:, :2].astype(np.float64) - robot[:2]
    local_mask = np.einsum("ij,ij->i", delta_xy, delta_xy) <= (
        _MOTION_LOG_LIDAR_LOCAL_RADIUS_M**2
    )
    local = visible[local_mask]
    context = visible[~local_mask]
    target_local = min(
        len(local),
        int(math.ceil(max_points * _MOTION_LOG_LIDAR_LOCAL_SHARE)),
    )
    target_context = min(len(context), max_points - target_local)
    target_local = min(len(local), max_points - target_context)
    sampled_local = _bounded_points(local, target_local)
    sampled_context = _bounded_points(context, target_context)
    if sampled_local.shape[0] == 0:
        return sampled_context
    if sampled_context.shape[0] == 0:
        return sampled_local
    return np.vstack([sampled_local, sampled_context]).astype(np.float32, copy=False)


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


def _step_static_engine_for_sensor_tick(engine: Any, imu_period_s: float) -> Any:
    step_static = getattr(engine, "step_static_sensor_tick", None)
    if not callable(step_static):
        raise RuntimeError("MuJoCo engine does not implement static sensor stepping")
    return step_static(dt_s=float(imu_period_s))


def _advance_static_engine_clock_for_dropped_tick(engine: Any, imu_period_s: float) -> float:
    advance_clock = getattr(engine, "advance_static_sensor_clock", None)
    if not callable(advance_clock):
        raise RuntimeError("MuJoCo engine does not implement fast anchored clock advance")
    return float(advance_clock(dt_s=float(imu_period_s)))


def _start_anchor_active(mode: str, *, motion_started: bool) -> bool:
    return mode == "run" or (mode == "warmup" and not motion_started)


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


def _wsl_path(path: Path) -> str:
    resolved = path.resolve()
    if os.name != "nt":
        return str(resolved)
    drive = resolved.drive.rstrip(":").lower()
    if not drive:
        raise ValueError(f"cannot convert path to WSL form: {resolved}")
    relative = resolved.as_posix().split(":", 1)[1].lstrip("/")
    return f"/mnt/{drive}/{relative}"


def _linux_binary_command(binary: Path, *args: str) -> list[str]:
    if os.name != "nt" or binary.suffix.lower() == ".exe":
        return [str(binary), *args]
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        raise FileNotFoundError("wsl.exe is required to run Linux native DDS binaries from Windows")
    return [wsl, "-e", _wsl_path(binary), *args]


def _managed_wsl_command(command: list[str], pid_file: Path) -> list[str]:
    """Wrap a WSL launch so the Linux exec PID is externally owned."""

    if os.name != "nt" or len(command) < 3 or command[1] != "-e":
        return command
    pid_file.parent.mkdir(parents=True, exist_ok=True)
    pid_file.unlink(missing_ok=True)
    script = 'pid_file="$1"; shift; echo "$$" > "$pid_file"; exec "$@"'
    return [
        command[0],
        "-e",
        "bash",
        "-lc",
        script,
        "lingtu-managed",
        _wsl_path(pid_file),
        *command[2:],
    ]


def _read_linux_pid(pid_file: Path, timeout_s: float = 2.0) -> int | None:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() <= deadline:
        try:
            pid = int(pid_file.read_text(encoding="ascii").strip())
            if pid > 0:
                return pid
        except (OSError, ValueError):
            pass
        time.sleep(0.02)
    return None


def _wsl_pid_alive(pid: int | None) -> bool:
    if os.name != "nt" or pid is None or pid <= 0:
        return False
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        return False
    try:
        proc = subprocess.run(
            [wsl, "-e", "kill", "-0", str(pid)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        # Cleanup verification must fail closed. An unavailable WSL control
        # path is not evidence that the Linux process exited.
        return True
    return proc.returncode == 0


def _signal_wsl_pid(pid: int | None, signal_name: str) -> bool:
    if os.name != "nt" or pid is None or pid <= 0:
        return False
    wsl = shutil.which("wsl.exe") or shutil.which("wsl")
    if not wsl:
        return False
    try:
        proc = subprocess.run(
            [wsl, "-e", "kill", f"-{signal_name}", str(pid)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0


def _wait_wsl_pid_exit(pid: int | None, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() <= deadline:
        if not _wsl_pid_alive(pid):
            return True
        time.sleep(0.05)
    return not _wsl_pid_alive(pid)


def _terminate_wsl_pid(
    pid: int | None,
    *,
    term_timeout_s: float = 3.0,
    kill_timeout_s: float = 2.0,
) -> dict[str, Any]:
    """Terminate one explicitly owned Linux PID and return auditable evidence."""

    if os.name != "nt" or pid is None or pid <= 0:
        return {
            "linux_pid": pid,
            "owned_pid": False,
            "alive_before_cleanup": False,
            "term_sent": False,
            "kill_sent": False,
            "alive_after_cleanup": False,
            "clean": os.name != "nt",
            "errors": ["linux_pid_handshake_missing"] if os.name == "nt" else [],
        }

    errors: list[str] = []
    alive_before = _wsl_pid_alive(pid)
    term_sent = False
    kill_sent = False
    if alive_before:
        term_sent = _signal_wsl_pid(pid, "TERM")
        if not term_sent:
            errors.append("term_signal_failed")
        if not _wait_wsl_pid_exit(pid, term_timeout_s):
            kill_sent = _signal_wsl_pid(pid, "KILL")
            if not kill_sent:
                errors.append("kill_signal_failed")
            _wait_wsl_pid_exit(pid, kill_timeout_s)
    alive_after = _wsl_pid_alive(pid)
    if alive_after:
        errors.append("linux_process_still_alive")
    return {
        "linux_pid": pid,
        "owned_pid": True,
        "alive_before_cleanup": alive_before,
        "term_sent": term_sent,
        "kill_sent": kill_sent,
        "alive_after_cleanup": alive_after,
        "clean": not alive_after,
        "errors": errors,
    }


def _stop_relay(process: subprocess.Popen[Any], timeout_s: float = 2.0) -> list[str]:
    """Stop the Windows WSL relay without assuming it owns the Linux child."""

    errors: list[str] = []
    if process.poll() is not None:
        return errors
    try:
        process.terminate()
        process.wait(timeout=max(0.1, timeout_s))
    except subprocess.TimeoutExpired:
        try:
            process.kill()
            process.wait(timeout=max(0.1, timeout_s))
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"relay_kill_failed:{type(exc).__name__}:{exc}")
    except OSError as exc:
        errors.append(f"relay_terminate_failed:{type(exc).__name__}:{exc}")
    return errors


def _cmd_vel_tap_candidates(value: str) -> list[Path]:
    if value:
        return [Path(value).expanduser()]
    return [ROOT / "build" / "mujoco_native_dds" / "lingtu_mujoco_cmd_vel_tap"]


def _resolve_cmd_vel_tap_bin(value: str) -> Path:
    for candidate in _cmd_vel_tap_candidates(value):
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(
        "native /nav/cmd_vel tap missing. Build it with: "
        "cmake -S sim/native_dds -B build/mujoco_native_dds && "
        "cmake --build build/mujoco_native_dds -j"
    )


def _linear_command_direction(
    vx: float,
    vy: float,
    *,
    epsilon: float = 1e-4,
) -> str:
    """Classify translational body-frame intent for acceptance diagnostics."""

    if math.hypot(float(vx), float(vy)) <= float(epsilon):
        return "idle"
    if float(vx) > float(epsilon):
        return "forward"
    if float(vx) < -float(epsilon):
        return "reverse"
    return "lateral"


class NativeCmdVelSource:
    """Receive final navigation commands through a C++ typed-DDS reader.

    Python only consumes a line-oriented process boundary and applies the
    already-arbitrated command to MuJoCo. It never imports a DDS binding and
    never computes a global path, local path, or follower command.
    """

    _PREFIX = "LT_CMD_V1"
    _PID_PREFIX = "LT_PID_V1"

    def __init__(
        self,
        *,
        binary: Path,
        domain_id: int,
        timeout_s: float,
        pid_file: Path | None = None,
    ) -> None:
        self.binary = binary.resolve()
        self.domain_id = int(domain_id)
        self.timeout_s = max(0.01, float(timeout_s))
        self._lock = threading.Lock()
        self._latest: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._latest_arrival_s = 0.0
        self._latest_source_stamp_s = 0.0
        self._samples = 0
        self._nonzero_samples = 0
        self._forward_linear_samples = 0
        self._reverse_linear_samples = 0
        self._lateral_linear_samples = 0
        self._stale_reads = 0
        self._parse_errors = 0
        self._stderr_tail: list[str] = []
        self._failed_before_close = False
        self._terminated_by_parent = False
        self._linux_pid: int | None = None
        self._cleanup: dict[str, Any] = {}
        self._pid_file = pid_file
        if os.name == "nt" and self._pid_file is None:
            self._pid_file = (
                ROOT / "artifacts" / "mujoco_native_dds" / f"cmd_vel_tap_{os.getpid()}_{id(self)}.pid"
            ).resolve()
        if self._pid_file is not None:
            self._pid_file.parent.mkdir(parents=True, exist_ok=True)
            self._pid_file.unlink(missing_ok=True)
        command = _linux_binary_command(
            self.binary,
            "--domain-id",
            str(self.domain_id),
        )
        launch_command = (
            _managed_wsl_command(command, self._pid_file) if os.name == "nt" and self._pid_file is not None else command
        )
        self.process = subprocess.Popen(
            launch_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
        )
        self._stdout_thread = threading.Thread(target=self._read_stdout, daemon=True)
        self._stderr_thread = threading.Thread(target=self._read_stderr, daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()
        if os.name == "nt" and self._pid_file is not None:
            linux_pid = _read_linux_pid(self._pid_file, timeout_s=10.0)
            if linux_pid is None:
                relay_errors = _stop_relay(self.process)
                self._stdout_thread.join(timeout=1.0)
                self._stderr_thread.join(timeout=1.0)
                detail = ";".join(relay_errors) if relay_errors else "pid_file_not_written"
                raise RuntimeError(f"cmd_vel tap Linux PID handshake failed: {detail}")
            with self._lock:
                self._linux_pid = linux_pid

    @classmethod
    def parse_line(cls, line: str) -> tuple[float, float, float, float] | None:
        parts = line.strip().split("\t")
        if len(parts) != 6 or parts[0] != cls._PREFIX:
            return None
        try:
            return float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])
        except ValueError:
            return None

    @classmethod
    def parse_pid_line(cls, line: str) -> int | None:
        parts = line.strip().split("\t")
        if len(parts) != 2 or parts[0] != cls._PID_PREFIX:
            return None
        try:
            pid = int(parts[1])
        except ValueError:
            return None
        return pid if pid > 0 else None

    def _read_stdout(self) -> None:
        stream = self.process.stdout
        if stream is None:
            return
        for line in stream:
            linux_pid = self.parse_pid_line(line)
            if linux_pid is not None:
                with self._lock:
                    self._linux_pid = linux_pid
                if self._pid_file is not None:
                    self._pid_file.write_text(f"{linux_pid}\n", encoding="ascii")
                continue
            parsed = self.parse_line(line)
            if parsed is None:
                with self._lock:
                    self._parse_errors += 1
                continue
            source_stamp_s, vx, vy, wz = parsed
            now_s = time.monotonic()
            with self._lock:
                self._latest = (vx, vy, wz)
                self._latest_arrival_s = now_s
                self._latest_source_stamp_s = source_stamp_s
                self._samples += 1
                if math.hypot(vx, vy) > 1e-4 or abs(wz) > 1e-4:
                    self._nonzero_samples += 1
                linear_direction = _linear_command_direction(vx, vy)
                if linear_direction == "forward":
                    self._forward_linear_samples += 1
                elif linear_direction == "reverse":
                    self._reverse_linear_samples += 1
                elif linear_direction == "lateral":
                    self._lateral_linear_samples += 1

    def _read_stderr(self) -> None:
        stream = self.process.stderr
        if stream is None:
            return
        for line in stream:
            with self._lock:
                self._stderr_tail.append(line.rstrip())
                del self._stderr_tail[:-20]

    def command(self) -> tuple[float, float, float]:
        now_s = time.monotonic()
        with self._lock:
            age_s = now_s - self._latest_arrival_s if self._latest_arrival_s > 0.0 else math.inf
            if age_s > self.timeout_s:
                self._stale_reads += 1
                return 0.0, 0.0, 0.0
            return self._latest

    def stats(self) -> dict[str, Any]:
        with self._lock:
            age_s = time.monotonic() - self._latest_arrival_s if self._latest_arrival_s > 0.0 else None
            return {
                "transport": "cpp_typed_dds_tap",
                "topic": "/nav/cmd_vel",
                "dds_topic": "rt/nav/cmd_vel",
                "binary": str(self.binary),
                "samples": self._samples,
                "nonzero_samples": self._nonzero_samples,
                "forward_linear_samples": self._forward_linear_samples,
                "reverse_linear_samples": self._reverse_linear_samples,
                "lateral_linear_samples": self._lateral_linear_samples,
                "stale_reads": self._stale_reads,
                "parse_errors": self._parse_errors,
                "last_source_stamp_s": self._latest_source_stamp_s,
                "last_arrival_age_s": age_s,
                "last_command": {
                    "vx": self._latest[0],
                    "vy": self._latest[1],
                    "wz": self._latest[2],
                },
                "process_returncode": self.process.poll(),
                "linux_pid": self._linux_pid,
                "failed_before_close": self._failed_before_close,
                "terminated_by_parent": self._terminated_by_parent,
                "process_cleanup": dict(self._cleanup),
                "stderr_tail": list(self._stderr_tail),
            }

    def close(self) -> None:
        returncode = self.process.poll()
        if returncode is None:
            self._terminated_by_parent = True
            with self._lock:
                linux_pid = self._linux_pid
            self._cleanup = _terminate_wsl_pid(linux_pid)
            relay_errors = _stop_relay(self.process, timeout_s=3.0)
            if relay_errors:
                self._cleanup.setdefault("errors", []).extend(relay_errors)
                self._cleanup["clean"] = False
        elif returncode != 0:
            self._failed_before_close = True
            with self._lock:
                linux_pid = self._linux_pid
            self._cleanup = _terminate_wsl_pid(linux_pid)
        else:
            with self._lock:
                linux_pid = self._linux_pid
            self._cleanup = _terminate_wsl_pid(linux_pid)
        self._stdout_thread.join(timeout=1.0)
        self._stderr_thread.join(timeout=1.0)


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
        if str(drive_mode or "").strip().lower() == "kinematic" and not bool(allow_kinematic_fastlio_acceptance):
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
    pose = (status.get("odometry") or {}).get("pose") or {}
    coords = {axis: float(pose.get(axis) or 0.0) for axis in ("x", "y", "z")}
    if any(not math.isfinite(value) for value in coords.values()):
        gaps.append("native_slam_odom_not_finite")
    elif any(abs(value) > float(max_odom_abs_m) for value in coords.values()):
        gaps.append(f"native_slam_odom_out_of_bounds:x={coords['x']:.3f},y={coords['y']:.3f},z={coords['z']:.3f}")
    if math.isfinite(coords["z"]) and abs(coords["z"]) > float(max_odom_z_abs_m):
        gaps.append(f"native_slam_odom_z_out_of_bounds:{coords['z']:.3f}")
    return gaps


def _slam_odom_pose(status: dict[str, Any]) -> dict[str, Any]:
    return (status.get("odometry") or {}).get("pose") or {}


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


def _multiply_quaternions_xyzw(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = [float(value) for value in lhs]
    rx, ry, rz, rw = [float(value) for value in rhs]
    result = np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(result))
    return result / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def _slam_map_pose(status: dict[str, Any]) -> dict[str, float]:
    transform = status.get("map_odom_tf") or {}
    odom_pose = _slam_odom_pose(status)
    if transform.get("valid") is not True or not odom_pose:
        return {}
    try:
        map_translation = np.array(
            [transform["tx"], transform["ty"], transform["tz"]],
            dtype=np.float64,
        )
        map_quaternion = np.array(
            [transform["qx"], transform["qy"], transform["qz"], transform["qw"]],
            dtype=np.float64,
        )
        odom_translation = np.array(
            [odom_pose["x"], odom_pose["y"], odom_pose["z"]],
            dtype=np.float64,
        )
        odom_quaternion = np.array(
            [odom_pose["qx"], odom_pose["qy"], odom_pose["qz"], odom_pose["qw"]],
            dtype=np.float64,
        )
    except (KeyError, TypeError, ValueError):
        return {}
    if not all(
        np.isfinite(values).all() for values in (map_translation, map_quaternion, odom_translation, odom_quaternion)
    ):
        return {}
    position = map_translation + quat_xyzw_to_matrix(map_quaternion) @ odom_translation
    quaternion = _multiply_quaternions_xyzw(map_quaternion, odom_quaternion)
    return {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "qx": float(quaternion[0]),
        "qy": float(quaternion[1]),
        "qz": float(quaternion[2]),
        "qw": float(quaternion[3]),
    }


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
    slam_map_pose = _slam_map_pose(slam_status)
    slam_map_xy_error_m = None
    slam_map_yaw_error_rad = None
    if slam_map_pose:
        slam_map_xy_error_m = float(
            math.hypot(
                float(slam_map_pose["x"]) - float(end[0]),
                float(slam_map_pose["y"]) - float(end[1]),
            )
        )
        if sim_end_yaw is not None:
            map_yaw = yaw_from_quat_xyzw(
                np.array(
                    [
                        slam_map_pose["qx"],
                        slam_map_pose["qy"],
                        slam_map_pose["qz"],
                        slam_map_pose["qw"],
                    ],
                    dtype=np.float64,
                )
            )
            slam_map_yaw_error_rad = float(angle_delta_rad(map_yaw, float(sim_end_yaw)))
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
        "slam_map_pose": slam_map_pose or None,
        "slam_map_xy_error_m": slam_map_xy_error_m,
        "slam_map_yaw_error_rad": slam_map_yaw_error_rad,
        "track_against_map": slam_status.get("track_against_map") or {},
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
    max_slam_map_xy_error_m: float = 0.35,
) -> list[str]:
    if not motion.get("available"):
        return []
    gaps: list[str] = []
    min_sim_motion = max(0.0, float(min_sim_motion_for_odom_check_m))
    sim_xy_m = float(motion.get("sim_xy_m") or 0.0)
    map_error = motion.get("slam_map_xy_error_m")
    tracking = motion.get("track_against_map") or {}
    map_localization_valid = (
        map_error is not None
        and math.isfinite(float(map_error))
        and int(tracking.get("successes") or 0) > 0
        and bool(tracking.get("enabled"))
        and int(tracking.get("consecutive_failures") or 0) == 0
    )
    if map_localization_valid:
        if float(map_error) > max(0.0, float(max_slam_map_xy_error_m)):
            gaps.append(
                "native_slam_map_pose_mismatch:"
                f"xy_error={float(map_error):.3f},max_error={float(max_slam_map_xy_error_m):.3f}"
            )
        map_yaw_error = motion.get("slam_map_yaw_error_rad")
        if map_yaw_error is not None and math.isfinite(float(map_yaw_error)):
            if abs(float(map_yaw_error)) > max(0.0, float(max_slam_yaw_error_rad)):
                gaps.append(
                    "native_slam_map_yaw_mismatch:"
                    f"yaw_error={abs(float(map_yaw_error)):.3f},"
                    f"max_error={float(max_slam_yaw_error_rad):.3f}"
                )
    elif min_sim_motion > 0.0 and sim_xy_m >= min_sim_motion:
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
    yaw_error = (
        motion.get("slam_map_yaw_error_rad") if map_localization_valid else motion.get("slam_to_sim_yaw_error_rad")
    )
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
    command = _linux_binary_command(
        publisher,
        "--stdin-records",
        "--restamp-stdin-records",
        "--dds",
        "--domain-id",
        str(int(args.domain_id)),
        "--lidar-frame",
        LIDAR_FRAME_ID,
        "--imu-frame",
        IMU_FRAME_ID,
    )
    if bool(getattr(args, "navigation_fixture", False)):
        command.append("--navigation-fixture")
    pid_file_value = str(getattr(args, "publisher_pid_file", "") or "")
    if pid_file_value:
        pid_file = Path(pid_file_value).expanduser().resolve()
    else:
        report_path = Path(str(getattr(args, "json_out", "") or "")).expanduser()
        base = report_path.parent if str(report_path) else ROOT / "artifacts"
        pid_file = (base / f"mujoco_sensor_publisher_{os.getpid()}.pid").resolve()
    process = subprocess.Popen(
        _managed_wsl_command(command, pid_file),
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
    )
    linux_pid = _read_linux_pid(pid_file, timeout_s=10.0)
    if linux_pid is None:
        relay_errors = _stop_relay(process)
        detail = ";".join(relay_errors) if relay_errors else "pid_file_not_written"
        raise RuntimeError(f"sensor publisher Linux PID handshake failed: {detail}")
    process._lingtu_linux_pid_file = pid_file
    process._lingtu_linux_pid = linux_pid
    return process


def _finish_native_publisher(process: subprocess.Popen[bytes]) -> dict[str, Any]:
    pid = getattr(process, "_lingtu_linux_pid", None)
    pid_file = getattr(process, "_lingtu_linux_pid_file", None)
    errors: list[str] = []
    try:
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
    except OSError as exc:
        errors.append(f"publisher_stdin_close_failed:{type(exc).__name__}:{exc}")
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        pass
    except OSError as exc:
        errors.append(f"publisher_wait_failed:{type(exc).__name__}:{exc}")

    cleanup = _terminate_wsl_pid(pid)
    relay_errors = _stop_relay(process)
    if relay_errors:
        errors.extend(relay_errors)
    if errors:
        cleanup.setdefault("errors", []).extend(errors)
        cleanup["clean"] = False
    cleanup.update(
        {
            "pid_file": str(pid_file or ""),
            "returncode": process.poll(),
        }
    )
    return cleanup


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


def _write_native_odom_prior(
    stream: Any,
    state: Any,
    timestamp_s: float,
    sequence: int,
    *,
    velocity: Any | None = None,
    has_velocity: bool = True,
) -> None:
    position = np.asarray(getattr(state, "position", (0.0, 0.0, 0.0)), dtype=np.float64)
    orientation = np.asarray(getattr(state, "orientation", (0.0, 0.0, 0.0, 1.0)), dtype=np.float64)
    if velocity is None:
        velocity = getattr(state, "linear_velocity", (0.0, 0.0, 0.0))
    velocity = np.asarray(velocity, dtype=np.float64)
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
        int(bool(has_velocity)),
    )
    _write_record(
        stream,
        _RECORD_ODOM_PRIOR,
        int(float(timestamp_s) * 1_000_000_000),
        sequence,
        payload,
        1,
    )


def _write_native_registered_cloud(
    stream: Any,
    points_xyzi_body: Any,
    *,
    timestamp_ns: int,
    sequence: int,
) -> None:
    points = np.asarray(points_xyzi_body, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"expected body-frame XYZI cloud shape (N, >=3), got {points.shape}")
    frame = _xyzi_to_livox_frame(
        points,
        timestamp_ns=int(timestamp_ns),
        sequence=int(sequence),
        frame_id="body",
        scan_duration_ns=0,
    )
    payload = np.asarray(frame.points, dtype=POINT_DTYPE).tobytes()
    _write_record(
        stream,
        _RECORD_REGISTERED_CLOUD,
        int(timestamp_ns),
        int(sequence),
        payload,
        int(frame.point_count),
    )


def _world_xyzi_to_body_xyzi(points_xyzi_world: Any, state: Any) -> Any:
    points = np.asarray(points_xyzi_world, dtype=np.float32)
    if points.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"expected world-frame XYZI cloud shape (N, >=3), got {points.shape}")
    position = np.asarray(state.position, dtype=np.float64).reshape(3)
    rotation_body_to_world = quat_xyzw_to_matrix(
        np.asarray(state.orientation, dtype=np.float64)
    )
    xyz_body = (points[:, :3].astype(np.float64) - position) @ rotation_body_to_world
    intensity = (
        points[:, 3:4]
        if points.shape[1] >= 4
        else np.zeros((len(points), 1), dtype=np.float32)
    )
    return np.hstack((xyz_body.astype(np.float32), intensity.astype(np.float32))).astype(
        np.float32,
        copy=False,
    )


def _read_json_object(path: str | Path) -> dict[str, Any]:
    if not path:
        return {}
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _native_nav_goal_reached(status: dict[str, Any]) -> bool:
    return bool((status.get("last_local") or {}).get("goal_reached"))


def _slam_status_counts(path: str) -> tuple[Counter[str], dict[str, Any]]:
    if not path:
        return Counter(), {}
    status_path = Path(path)
    if not status_path.exists():
        return Counter(), {}
    status = _read_json_object(status_path)
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
    requested_start = parse_start(str(args.start or ""))
    start_anchor = str(args.start_anchor)
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
    navigation_fixture = bool(getattr(args, "navigation_fixture", False))
    if navigation_fixture and not bool(args.publish_odom_prior):
        raise ValueError("--navigation-fixture requires --publish-odom-prior")
    if navigation_fixture and str(args.scan_time_profile) != "instantaneous":
        raise ValueError("--navigation-fixture requires --scan-time-profile instantaneous")
    navigation_fixture_cloud_points = max(
        1,
        int(getattr(args, "navigation_fixture_cloud_points", 4000) or 4000),
    )
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
    publisher_cleanup: dict[str, Any] = {}
    cleanup_errors: list[str] = []
    cmd_vel_source: NativeCmdVelSource | None = None
    cmd_vel_stats: dict[str, Any] = {
        "transport": "deterministic_profile",
        "samples": 0,
        "nonzero_samples": 0,
    }
    try:
        if str(args.command_source) == "dds":
            cmd_vel_source = NativeCmdVelSource(
                binary=_resolve_cmd_vel_tap_bin(str(args.cmd_vel_tap_bin or "")),
                domain_id=int(args.domain_id),
                timeout_s=float(args.cmd_vel_timeout_s),
                pid_file=(
                    Path(str(args.cmd_vel_pid_file)).expanduser().resolve()
                    if str(args.cmd_vel_pid_file or "")
                    else None
                ),
            )
    except Exception:
        _finish_native_publisher(publisher)
        raise

    motion_log_path = str(getattr(args, "motion_log", "") or "")
    motion_log_writer: AsyncJsonlWriter | None = None
    motion_log_samples = 0
    motion_log_writer_diagnostics = {
        "submitted": 0,
        "written": 0,
        "dropped": 0,
        "failures": 0,
        "max_pending": 0,
    }
    planner_debug_path = ""
    planner_debug_writer: AsyncJsonlWriter | None = None
    planner_debug_samples = 0
    planner_debug_writer_diagnostics = {
        "submitted": 0,
        "written": 0,
        "dropped": 0,
        "failures": 0,
        "max_pending": 0,
    }
    planner_debug_id = 0
    last_planner_debug_key: tuple[float, float] | None = None
    motion_log_hz = max(1.0, float(getattr(args, "motion_log_hz", 24.0) or 24.0))
    motion_log_period_s = 1.0 / motion_log_hz
    motion_log_lidar_points = max(
        0,
        int(getattr(args, "motion_log_lidar_points", 640) or 0),
    )
    if motion_log_path:
        log_path = Path(motion_log_path)
        motion_log_writer = AsyncJsonlWriter(
            log_path,
            max_pending=2048,
            flush_every=64,
        )
        planner_debug_log_path = log_path.with_name(f"{log_path.stem}_planner_debug.jsonl")
        planner_debug_path = str(planner_debug_log_path)
        planner_debug_writer = AsyncJsonlWriter(
            planner_debug_log_path,
            max_pending=1024,
            flush_every=16,
        )

    engine = None
    viewer = None
    viewer_closed_early = False
    policy_loaded = False
    pacing_controller: SimHardwareCatchUpController | None = None
    pacing_stats: dict[str, Any] = {
        "enabled": False,
        "strategy": "legacy_split_or_wall_clock",
    }
    last_sim_time_s = 0.0
    sim_start_s = 0.0
    goal_reached_early = False
    lidar_backend_report: dict[str, Any] = {}
    runtime_stage_profiler = RuntimeStageProfiler()
    try:
        policy_path = _resolve_policy_path_for_drive(str(args.drive_mode), str(args.policy_path or ""))
        engine = build_engine(
            world=resolve_world(str(args.world)),
            drive_mode=str(args.drive_mode),
            n_rays=int(args.n_rays),
            start=requested_start,
            mujoco_memory=str(args.mujoco_memory),
            mid360_pattern=args.mid360_pattern,
            mid360_samples_per_frame=int(args.mid360_samples_per_frame),
            lidar_backend=str(args.lidar_backend),
            mujoco_lidar_backend=str(args.mujoco_lidar_backend),
            require_product_lidar_backend=True,
            allow_legacy_lidar_fallback=bool(args.allow_legacy_lidar_fallback),
            policy_path=policy_path,
        )
        if bool(getattr(args, "viewer", False)):
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(engine.model, engine.data)
        lidar_backend_report = engine.get_lidar_backend_report()
        policy_loaded = bool(getattr(engine, "has_policy", False))
        hold_cmd = VelocityCommand()
        initial_state = engine.get_robot_state()
        anchor_position = np.asarray(
            requested_start if requested_start is not None else initial_state.position,
            dtype=np.float64,
        )
        anchor_orientation = np.asarray(initial_state.orientation, dtype=np.float64)

        def apply_start_anchor() -> Any:
            engine.set_robot_pose(anchor_position, anchor_orientation)
            return engine.get_robot_state()

        motion_started = False
        if _start_anchor_active(start_anchor, motion_started=motion_started):
            apply_start_anchor()
        if settle_s > 0.0:
            settle_end_s = float(getattr(engine, "sim_time", 0.0)) + settle_s
            while float(getattr(engine, "sim_time", 0.0)) + 1e-9 < settle_end_s:
                if _start_anchor_active(start_anchor, motion_started=motion_started):
                    _step_static_engine_for_sensor_tick(engine, imu_period_s)
                else:
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
        if unified_sim_hardware_clock:
            pacing_controller = SimHardwareCatchUpController(
                clock=hardware_clock,
                max_lag_s=float(args.sim_hardware_max_lag_s),
                yield_every_steps=int(args.sim_hardware_catch_up_yield_steps),
            )
        last_sim_time_s = sim_start_s
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
        registered_cloud_sequence = 0
        odom_prior_velocity_estimator = OdomPriorVelocityEstimator(window_s=float(args.odom_prior_velocity_window_s))
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
        latest_world_points = np.zeros((0, 4), dtype=np.float32)
        next_motion_log_sim_s = sim_start_s
        next_nav_goal_check_sim_s = sim_start_s
        rolling_subscan_samples: int | None = None
        if str(args.physical_rolling_sample_mode) == "subscan":
            rolling_subscan_samples = _rolling_subscan_sample_count(
                frame_samples=int(args.mid360_samples_per_frame),
                imu_period_s=imu_period_s,
                lidar_period_s=lidar_period_s,
            )
        next_lidar_sim_s = sim_start_s
        viewer_period_s = 1.0 / max(1.0, float(getattr(args, "viewer_hz", 30.0) or 30.0))
        next_viewer_sim_s = sim_start_s
        previous_loop_end_wall_s: float | None = None
        while True:
            loop_start = time.monotonic()
            if previous_loop_end_wall_s is not None:
                runtime_stage_profiler.record(
                    "inter_loop_gap",
                    loop_start - previous_loop_end_wall_s,
                    float(getattr(engine, "sim_time", 0.0)),
                )
            command_stage_start = time.monotonic()
            drop_sensor_tick = False
            if unified_sim_hardware_clock:
                sim_elapsed_before_step_s = max(0.0, float(getattr(engine, "sim_time", 0.0)) - sim_start_s)
                if sim_elapsed_before_step_s >= warmup_s + duration_s:
                    break
                driving = sim_elapsed_before_step_s >= warmup_s
                drive_elapsed_s = max(0.0, sim_elapsed_before_step_s - warmup_s)
                drop_sensor_tick = bool(
                    pacing_controller
                    and pacing_controller.should_drop_sensor_tick(
                        float(getattr(engine, "sim_time", 0.0)),
                        monotonic_now_s=loop_start,
                    )
                )
            else:
                if loop_start >= deadline:
                    break
                driving = loop_start >= drive_start_s
                drive_elapsed_s = max(0.0, loop_start - drive_start_s) if driving else 0.0
            if driving and cmd_vel_source is not None:
                command_vx, command_vy, command_wz = cmd_vel_source.command()
                cmd = VelocityCommand(
                    linear_x=command_vx,
                    linear_y=command_vy,
                    angular_z=command_wz,
                )
            elif driving:
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
            command_norm = math.sqrt(float(cmd.linear_x) ** 2 + float(cmd.linear_y) ** 2 + float(cmd.angular_z) ** 2)
            if driving and (cmd_vel_source is None or command_norm > 1e-4):
                motion_started = True
            anchor_active = _start_anchor_active(
                start_anchor,
                motion_started=motion_started,
            )
            runtime_stage_profiler.record(
                "command_input",
                time.monotonic() - command_stage_start,
                float(getattr(engine, "sim_time", 0.0)),
            )
            physics_stage_start = time.monotonic()
            fast_static_clock = anchor_active and drop_sensor_tick
            if fast_static_clock:
                sim_time_s = _advance_static_engine_clock_for_dropped_tick(
                    engine,
                    imu_period_s,
                )
                state = None
            elif anchor_active:
                state = _step_static_engine_for_sensor_tick(engine, imu_period_s)
            else:
                state = _step_engine_for_sensor_tick(engine, cmd, imu_period_s)
            sim_time_s = float(getattr(engine, "sim_time", sim_time_s if state is None else 0.0))
            runtime_stage_profiler.record(
                "physics_step",
                time.monotonic() - physics_stage_start,
                sim_time_s,
            )
            last_sim_time_s = sim_time_s
            if viewer is not None and sim_time_s + 1e-9 >= next_viewer_sim_s:
                if not viewer.is_running():
                    viewer_closed_early = True
                    break
                viewer.sync()
                next_viewer_sim_s = sim_time_s + viewer_period_s
            if state is None:
                position = anchor_position.copy()
                yaw = yaw_from_quat_xyzw(anchor_orientation)
            else:
                position = np.asarray(state.position, dtype=np.float64).copy()
                yaw = yaw_from_quat_xyzw(state.orientation)
            physical_drive_active = driving and (cmd_vel_source is None or motion_started)
            if physical_drive_active and sim_start_position is None:
                sim_start_position = position.copy()
                sim_start_yaw = float(yaw)
                prev_drive_position = position.copy()
                prev_drive_yaw = float(yaw)
            if physical_drive_active:
                if prev_drive_position is not None:
                    drive_delta = position[:2] - prev_drive_position[:2]
                    sim_path_length_xy_m += float(math.hypot(float(drive_delta[0]), float(drive_delta[1])))
                if prev_drive_yaw is not None:
                    sim_abs_yaw_delta_rad += abs(float(angle_delta_rad(float(yaw), float(prev_drive_yaw))))
                prev_drive_position = position.copy()
                prev_drive_yaw = float(yaw)
                sim_end_position = position.copy()
                sim_end_yaw = float(yaw)
            if drop_sensor_tick:
                catch_up_stage_start = time.monotonic()
                if pacing_controller is not None:
                    pacing_controller.record_catch_up_step(
                        fast_static_clock=fast_static_clock,
                    )
                imu_sequence += 1
                if bool(args.publish_odom_prior):
                    odom_prior_sequence += 1
                if str(args.scan_time_profile) == "physical_rolling" and pacing_controller is not None:
                    pacing_controller.record_lidar_subscan_drop()
                next_lidar_sim_s, dropped_lidar_frames = _advance_skipped_lidar_deadlines(
                    next_lidar_sim_s=next_lidar_sim_s,
                    lidar_period_s=lidar_period_s,
                    sim_time_s=sim_time_s,
                )
                sequence += dropped_lidar_frames
                if pacing_controller is not None:
                    pacing_controller.record_lidar_frame_drops(dropped_lidar_frames)
                    pacing_controller.yield_if_due()
                if publisher.poll() is not None:
                    raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
                loop_end = time.monotonic()
                runtime_stage_profiler.record(
                    "catch_up_bookkeeping",
                    loop_end - catch_up_stage_start,
                    sim_time_s,
                )
                runtime_stage_profiler.record("loop_total", loop_end - loop_start, sim_time_s)
                previous_loop_end_wall_s = loop_end
                continue
            imu_publish_stage_start = time.monotonic()
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
                odom_prior_velocity, odom_prior_velocity_valid = odom_prior_velocity_estimator.update(
                    state.position,
                    sensor_ts_s,
                )
                _write_native_odom_prior(
                    publisher.stdin,
                    state,
                    sensor_ts_s,
                    odom_prior_sequence,
                    velocity=odom_prior_velocity,
                    has_velocity=odom_prior_velocity_valid,
                )
                sensor_counts[TOPICS.odom_prior] += 1
                odom_prior_sequence += 1
            prev_imu_s = sensor_ts_s
            prev_velocity = np.asarray(state.linear_velocity, dtype=np.float64).copy()
            runtime_stage_profiler.record(
                "imu_prior_publish",
                time.monotonic() - imu_publish_stage_start,
                sim_time_s,
            )
            if str(args.scan_time_profile) == "physical_rolling":
                lidar_subscan_stage_start = time.monotonic()
                sample_world_points = _bounded_points(
                    engine.get_lidar_points(sample_count=rolling_subscan_samples),
                    int(args.max_points),
                )
                sample_sensor_points = world_xyzi_to_sensor_xyzi(engine, sample_world_points)
                subscan_samples.append((sim_time_s, sample_sensor_points, sample_world_points, 0))
                min_sample_time = sim_time_s - lidar_period_s * 1.5
                subscan_samples = [sample for sample in subscan_samples if sample[0] >= min_sample_time]
                runtime_stage_profiler.record(
                    "lidar_subscan",
                    time.monotonic() - lidar_subscan_stage_start,
                    sim_time_s,
                )
            lidar_due = (
                sim_time_s + 1e-9 >= next_lidar_sim_s if unified_sim_hardware_clock else loop_start >= next_lidar_s
            )
            if lidar_due:
                lidar_frame_stage_start = time.monotonic()
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
                    sensor_points, world_points, relative_times_s, _moving_count, _subscan_count = (
                        _physical_rolling_scan_from_samples(
                            subscan_samples,
                            scan_start_s=scan_start_sim_s,
                            scan_end_s=sim_time_s,
                        )
                    )
                    if int(args.max_points) > 0 and len(sensor_points) > int(args.max_points):
                        stride = int(np.ceil(len(sensor_points) / float(args.max_points)))
                        sensor_points = sensor_points[::stride][: int(args.max_points)]
                        world_points = world_points[::stride][: int(args.max_points)]
                        relative_times_s = relative_times_s[::stride][: int(args.max_points)]
                else:
                    world_points = _bounded_points(engine.get_lidar_points(), int(args.max_points))
                    sensor_points = world_xyzi_to_sensor_xyzi(engine, world_points)
                    relative_times_s = _relative_times_for_scan(
                        len(sensor_points),
                        lidar_period_s,
                        scan_time_profile=str(args.scan_time_profile),
                    )
                latest_world_points = np.asarray(world_points, dtype=np.float32)
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
                if navigation_fixture:
                    body_points = _bounded_points(
                        _world_xyzi_to_body_xyzi(world_points, state),
                        navigation_fixture_cloud_points,
                    )
                    _write_native_registered_cloud(
                        publisher.stdin,
                        body_points,
                        timestamp_ns=int(scan_start_timestamp_s * 1_000_000_000),
                        sequence=registered_cloud_sequence,
                    )
                    sensor_counts[TOPICS.registered_cloud] += 1
                    registered_cloud_sequence += 1
                if unified_sim_hardware_clock:
                    next_lidar_sim_s += lidar_period_s
                    while next_lidar_sim_s <= sim_time_s - 1e-9:
                        next_lidar_sim_s += lidar_period_s
                else:
                    next_lidar_s = loop_start + lidar_period_s
                runtime_stage_profiler.record(
                    "lidar_frame_publish",
                    time.monotonic() - lidar_frame_stage_start,
                    sim_time_s,
                )
            nav_status_snapshot: dict[str, Any] | None = None
            goal_reached_this_tick = False
            if (
                bool(getattr(args, "stop_on_nav_goal_reached", False))
                and driving
                and sim_time_s + 1e-9 >= next_nav_goal_check_sim_s
            ):
                goal_check_stage_start = time.monotonic()
                nav_status_snapshot = _read_json_object(str(getattr(args, "nav_status_json", "") or ""))
                goal_reached_this_tick = _native_nav_goal_reached(nav_status_snapshot)
                runtime_stage_profiler.record(
                    "goal_status_check",
                    time.monotonic() - goal_check_stage_start,
                    sim_time_s,
                )
                next_nav_goal_check_sim_s = sim_time_s + 0.1

            motion_log_due = sim_time_s + 1e-9 >= next_motion_log_sim_s
            if motion_log_writer is not None and (motion_log_due or goal_reached_this_tick):
                motion_log_stage_start = time.monotonic()
                nav_status = nav_status_snapshot
                if nav_status is None:
                    nav_status = _read_json_object(str(getattr(args, "nav_status_json", "") or ""))
                local_planner_debug = nav_status.get("local_candidates") or {}
                local_map_debug = nav_status.get("local_map") or {}
                try:
                    nav_status_stamp_s = float(nav_status.get("stamp_s") or 0.0)
                except (TypeError, ValueError):
                    nav_status_stamp_s = 0.0
                try:
                    planner_snapshot_stamp_s = float(local_planner_debug.get("timestamp_s") or 0.0)
                except (TypeError, ValueError):
                    planner_snapshot_stamp_s = 0.0
                planner_debug_key = (nav_status_stamp_s, planner_snapshot_stamp_s)
                if planner_debug_writer is not None and planner_debug_key != last_planner_debug_key:
                    next_planner_debug_id = planner_debug_id + 1
                    if planner_debug_writer.submit(
                        {
                            "id": next_planner_debug_id,
                            "nav_status_stamp_s": nav_status_stamp_s,
                            "local_planner_debug": local_planner_debug,
                            "local_map": local_map_debug,
                            "global_path": nav_status.get("global_path") or [],
                            "local_path": nav_status.get("local_path") or [],
                            "last_local": nav_status.get("last_local") or {},
                            "input_gate": nav_status.get("input_gate") or {},
                            "dynamic_objects": nav_status.get("dynamic_objects") or [],
                            "timing_ms": nav_status.get("timing_ms") or {},
                            "status_snapshot_writer": nav_status.get("status_snapshot_writer") or {},
                        }
                    ):
                        planner_debug_id = next_planner_debug_id
                        planner_debug_samples += 1
                        last_planner_debug_key = planner_debug_key
                display_points = (
                    _motion_log_lidar_sample(
                        latest_world_points,
                        max_points=motion_log_lidar_points,
                        robot_xyz=position,
                    )
                    if motion_log_lidar_points > 0
                    else np.zeros((0, 4), dtype=np.float32)
                )
                if motion_log_writer.submit(
                    {
                        "t": float(sensor_ts_s),
                        "sim_time_s": float(sim_time_s),
                        "x": float(position[0]),
                        "y": float(position[1]),
                        "z": float(position[2]),
                        "yaw": float(yaw),
                        "driving": bool(driving),
                        "start_xyz": [float(value) for value in parse_start(str(args.start or ""))],
                        "qpos": np.asarray(engine.data.qpos, dtype=np.float64).tolist(),
                        "cmd": [float(cmd.linear_x), float(cmd.linear_y), float(cmd.angular_z)],
                        "global_path": nav_status.get("global_path") or [],
                        "local_path": nav_status.get("local_path") or [],
                        "planner_debug_id": planner_debug_id or None,
                        "nav_status_stamp_s": nav_status_stamp_s,
                        "local_reason": str((nav_status.get("last_local") or {}).get("reason") or ""),
                        "local_diagnostics": nav_status.get("last_local") or {},
                        "input_gate": nav_status.get("input_gate") or {},
                        "odom_prior_velocity": odom_prior_velocity_estimator.stats(),
                        "lidar_world": display_points[:, :3].astype(float).tolist(),
                    }
                ):
                    motion_log_samples += 1
                if motion_log_due:
                    next_motion_log_sim_s += motion_log_period_s
                    while next_motion_log_sim_s <= sim_time_s - 1e-9:
                        next_motion_log_sim_s += motion_log_period_s
                runtime_stage_profiler.record(
                    "motion_log",
                    time.monotonic() - motion_log_stage_start,
                    sim_time_s,
                )
            if goal_reached_this_tick:
                goal_reached_early = True
                loop_end = time.monotonic()
                runtime_stage_profiler.record("loop_total", loop_end - loop_start, sim_time_s)
                previous_loop_end_wall_s = loop_end
                break
            publisher_flush_stage_start = time.monotonic()
            publisher.stdin.flush()
            if publisher.poll() is not None:
                raise RuntimeError(f"native DDS sensor publisher exited: {publisher.returncode}")
            runtime_stage_profiler.record(
                "publisher_flush",
                time.monotonic() - publisher_flush_stage_start,
                sim_time_s,
            )
            pacing_sleep_stage_start = time.monotonic()
            if unified_sim_hardware_clock:
                hardware_clock.sleep_until(sim_time_s)
            else:
                time.sleep(max(0.0, imu_period_s - (time.monotonic() - loop_start)))
            loop_end = time.monotonic()
            runtime_stage_profiler.record(
                "pacing_sleep",
                loop_end - pacing_sleep_stage_start,
                sim_time_s,
            )
            runtime_stage_profiler.record("loop_total", loop_end - loop_start, sim_time_s)
            previous_loop_end_wall_s = loop_end
    finally:
        if pacing_controller is not None:
            pacing_stats = pacing_controller.stats(
                last_sim_time_s,
                monotonic_now_s=time.monotonic(),
            )
        if cmd_vel_source is not None:
            try:
                cmd_vel_source.close()
            except Exception as exc:
                cleanup_errors.append(f"cmd_vel_tap_cleanup_failed:{type(exc).__name__}:{exc}")
            finally:
                cmd_vel_stats = cmd_vel_source.stats()
        if motion_log_writer is not None:
            try:
                motion_log_writer_diagnostics = motion_log_writer.close()
            except Exception as exc:
                motion_log_writer_diagnostics = motion_log_writer.diagnostics()
                cleanup_errors.append(f"motion_log_cleanup_failed:{type(exc).__name__}:{exc}")
        if planner_debug_writer is not None:
            try:
                planner_debug_writer_diagnostics = planner_debug_writer.close()
            except Exception as exc:
                planner_debug_writer_diagnostics = planner_debug_writer.diagnostics()
                cleanup_errors.append(f"planner_debug_log_cleanup_failed:{type(exc).__name__}:{exc}")
        try:
            publisher_cleanup = _finish_native_publisher(publisher)
        except Exception as exc:
            cleanup_errors.append(f"publisher_cleanup_failed:{type(exc).__name__}:{exc}")
            publisher_cleanup = {
                "linux_pid": getattr(publisher, "_lingtu_linux_pid", None),
                "clean": False,
                "errors": [cleanup_errors[-1]],
            }
        if viewer is not None:
            try:
                viewer.close()
            except Exception as exc:
                cleanup_errors.append(f"mujoco_viewer_cleanup_failed:{type(exc).__name__}:{exc}")
        if engine is not None:
            try:
                engine.close()
            except Exception as exc:
                cleanup_errors.append(f"mujoco_cleanup_failed:{type(exc).__name__}:{exc}")

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
    if str(args.command_source) == "dds":
        if bool(cmd_vel_stats.get("failed_before_close")):
            gaps.append("native_cmd_vel_tap_failed")
        if (cmd_vel_stats.get("process_cleanup") or {}).get("clean") is not True:
            gaps.append("native_cmd_vel_tap_cleanup_failed")
        if bool(args.require_cmd_vel) and int(cmd_vel_stats.get("nonzero_samples") or 0) <= 0:
            gaps.append("native_cmd_vel_nonzero_missing")
    if publisher_cleanup.get("clean") is not True:
        gaps.append("native_sensor_publisher_cleanup_failed")
    gaps.extend(cleanup_errors)
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
                max_slam_map_xy_error_m=float(args.max_slam_map_xy_error_m),
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
    report["start_anchor"] = start_anchor
    report["start_anchor_xyz"] = [float(value) for value in anchor_position[:3]]
    report["settle_s"] = settle_s
    report["drive_ramp_s"] = drive_ramp_s
    report["drive_duration_s"] = duration_s
    report["drive_mode"] = str(args.drive_mode)
    report["command_source"] = str(args.command_source)
    report["viewer"] = {
        "enabled": bool(getattr(args, "viewer", False)),
        "refresh_hz": float(getattr(args, "viewer_hz", 30.0) or 30.0),
        "closed_early": viewer_closed_early,
        "control_authority": "none_presentation_only",
    }
    report["cmd_vel"] = cmd_vel_stats
    report["native_sensor_publisher_process"] = publisher_cleanup
    report["policy_loaded"] = policy_loaded
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
    report["navigation_fixture"] = navigation_fixture
    report["navigation_fixture_cloud_points"] = navigation_fixture_cloud_points
    report["odom_prior_velocity_source"] = "robust_pose_window"
    report["odom_prior_velocity_window_s"] = float(args.odom_prior_velocity_window_s)
    report["odom_prior_velocity"] = odom_prior_velocity_estimator.stats()
    report["physical_rolling_sample_mode"] = str(args.physical_rolling_sample_mode)
    report["imu_acc_axis_scale"] = [float(v) for v in imu_acc_axis_scale]
    report["imu_acc_axis_scale_source"] = imu_acc_axis_scale_source
    report["imu_gyro_axis_scale"] = [float(v) for v in imu_gyro_axis_scale]
    report["imu_timestamp_clock"] = str(args.imu_timestamp_clock or args.timestamp_clock)
    report["lidar_timestamp_clock"] = str(args.lidar_timestamp_clock or args.timestamp_clock)
    report["clock_profile"] = "sim_hardware" if unified_sim_hardware_clock else "legacy_split_or_wall"
    report["sim_hardware_realtime_factor"] = float(args.sim_hardware_realtime_factor)
    report["sim_hardware_pacing"] = pacing_stats
    report["sim_elapsed_s"] = max(0.0, float(last_sim_time_s) - float(sim_start_s))
    report["effective_publish_hz"] = {
        str(topic): (float(count) / report["sim_elapsed_s"] if report["sim_elapsed_s"] > 0.0 else 0.0)
        for topic, count in sensor_counts.items()
    }
    report["lidar_backend"] = lidar_backend_report
    report["motion_log"] = motion_log_path
    report["motion_log_samples"] = int(motion_log_samples)
    report["motion_log_writer"] = motion_log_writer_diagnostics
    report["motion_log_hz"] = float(motion_log_hz)
    report["motion_log_lidar_points"] = int(motion_log_lidar_points)
    report["motion_log_lidar_presentation"] = {
        "sample_after_vertical_filter": True,
        "max_z_above_robot_m": _MOTION_LOG_LIDAR_MAX_Z_ABOVE_ROBOT_M,
        "local_radius_m": _MOTION_LOG_LIDAR_LOCAL_RADIUS_M,
        "local_share": _MOTION_LOG_LIDAR_LOCAL_SHARE,
    }
    report["planner_debug_log"] = planner_debug_path
    report["planner_debug_samples"] = int(planner_debug_samples)
    report["planner_debug_writer"] = planner_debug_writer_diagnostics
    report["goal_reached_early"] = goal_reached_early
    report["runtime_stage_timing"] = runtime_stage_profiler.stats()
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
