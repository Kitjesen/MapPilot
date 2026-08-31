"""Typed DDS process adapter for simulation truth odometry."""

from __future__ import annotations

import json
import os
import re
import struct
import subprocess
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, BinaryIO

from sim.runtime.process_owner import ProcessTreeOwner

from .samples import ImuSample, Mid360FrameSample, TruthOdometrySample

# Compatibility-boundary assertion: from .samples import TruthOdometrySample

if TYPE_CHECKING:
    from sim.runtime.coordinator.run_allocation import RunAllocation

_PROTOCOL_VERSION = 1
_HAS_LINEAR_VELOCITY = 1 << 0
_HAS_ANGULAR_VELOCITY = 1 << 1
_HAS_POSE_COVARIANCE = 1 << 2
_HAS_TWIST_COVARIANCE = 1 << 3
_HEADER = struct.Struct("<4sHHII64sQQQQ")
_PAYLOAD = struct.Struct("<85d")
_SESSION_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,62}\Z")
_UINT64_MAX = (1 << 64) - 1
_MAXIMUM_IDL_TIME_NS = ((1 << 31) - 1) * 1_000_000_000 + 999_999_999
_READY_SCHEMA = "lingtu.sim.truth-odom-publisher.ready.v1"
_TRUTH_ODOMETRY_TOPIC = "rt/sim/truth/odom"

_IMU_PROTOCOL_VERSION = 1
_IMU_HAS_ORIENTATION = 1 << 0
_IMU_HAS_GYRO = 1 << 1
_IMU_HAS_ACCELERATION = 1 << 2
_IMU_UNITS_SI = 1 << 3
_IMU_ORIENTATION_WXYZ = 1 << 4
_IMU_ALL_FLAGS = (
    _IMU_HAS_ORIENTATION
    | _IMU_HAS_GYRO
    | _IMU_HAS_ACCELERATION
    | _IMU_UNITS_SI
    | _IMU_ORIENTATION_WXYZ
)
_IMU_HEADER = struct.Struct("<4sHHII64sQQQQ64s")
_IMU_PAYLOAD = struct.Struct("<10d")
_IMU_READY_SCHEMA = "lingtu.sim.imu-publisher.ready.v1"
_IMU_TOPIC = "rt/sim/imu"
_MID360_TOPIC = "rt/sim/lidar/raw_frame"
_LTU1_HEADER = struct.Struct("<4sB3xQIII")
_LTU1_CLOUD_RECORD = 1
_LIVOX_POINT_PAYLOAD = struct.Struct("<4fIBBH")


class TruthOdometryAdapterError(RuntimeError):
    """Raised when the truth odometry DDS process cannot operate safely."""


class ImuAdapterError(RuntimeError):
    """Raised when the simulation IMU DDS process cannot operate safely."""


class Mid360AdapterError(RuntimeError):
    """Raised when the simulation MID-360 DDS process cannot operate safely."""


def _uint64(value: int, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{field} must fit an unsigned 64-bit integer")
    return value


def _session_id(value: object, field: str = "session_id") -> str:
    if not isinstance(value, str) or _SESSION_ID_RE.fullmatch(value) is None:
        raise ValueError(f"{field} must be an ASCII slug of at most 63 characters")
    return value


def encode_truth_odometry_sample(sample: TruthOdometrySample) -> bytes:
    """Encode one transport-neutral sample as one canonical LTOD v1 record."""

    if not isinstance(sample, TruthOdometrySample):
        raise TypeError("sample must be a TruthOdometrySample")
    stamp = sample.stamp
    session_id = _session_id(stamp.session_id)
    model_generation = _uint64(stamp.model_generation, "model_generation")
    reset_generation = _uint64(stamp.reset_generation, "reset_generation")
    sequence = _uint64(stamp.sequence, "sequence")
    timestamp_ns = _uint64(stamp.sim_time_ns, "sim_time_ns")
    if timestamp_ns > _MAXIMUM_IDL_TIME_NS:
        raise ValueError("sim_time_ns is outside the canonical IDL Time range")

    flags = 0
    linear_velocity = sample.linear_velocity_mps or (0.0, 0.0, 0.0)
    if sample.linear_velocity_mps is not None:
        flags |= _HAS_LINEAR_VELOCITY
    angular_velocity = sample.angular_velocity_rps or (0.0, 0.0, 0.0)
    if sample.angular_velocity_rps is not None:
        flags |= _HAS_ANGULAR_VELOCITY
    pose_covariance = sample.pose_covariance or (0.0,) * 36
    if sample.pose_covariance is not None:
        flags |= _HAS_POSE_COVARIANCE
    twist_covariance = sample.twist_covariance or (0.0,) * 36
    if sample.twist_covariance is not None:
        flags |= _HAS_TWIST_COVARIANCE

    header = _HEADER.pack(
        b"LTOD",
        _PROTOCOL_VERSION,
        flags,
        _HEADER.size,
        _PAYLOAD.size,
        session_id.encode("ascii"),
        model_generation,
        reset_generation,
        sequence,
        timestamp_ns,
    )
    payload = _PAYLOAD.pack(
        *sample.position_m,
        *sample.orientation_wxyz,
        *linear_velocity,
        *angular_velocity,
        *pose_covariance,
        *twist_covariance,
    )
    return header + payload


def encode_imu_sample(sample: ImuSample) -> bytes:
    """Encode a complete SI-unit IMU sample as an LTIM v1 record."""

    if not isinstance(sample, ImuSample):
        raise TypeError("sample must be an ImuSample")
    stamp = sample.stamp
    session_id = _session_id(stamp.session_id)
    if (
        sample.orientation_wxyz is None
        or sample.angular_velocity_rps is None
        or sample.linear_acceleration_mps2 is None
    ):
        raise ValueError("simulation IMU records require orientation, gyro, and acceleration")
    frame = _frame(stamp.frame_id, "frame_id")
    frame_bytes = frame.encode("utf-8")
    if len(frame_bytes) > 63:
        raise ValueError("frame_id must fit the 64-byte IMU protocol field")
    timestamp_ns = _uint64(stamp.sim_time_ns, "sim_time_ns")
    if timestamp_ns > _MAXIMUM_IDL_TIME_NS:
        raise ValueError("sim_time_ns is outside the canonical IDL Time range")
    frame_field = frame_bytes + b"\0" * (64 - len(frame_bytes))
    header = _IMU_HEADER.pack(
        b"LTIM",
        _IMU_PROTOCOL_VERSION,
        _IMU_ALL_FLAGS,
        _IMU_HEADER.size,
        _IMU_PAYLOAD.size,
        session_id.encode("ascii"),
        _uint64(stamp.model_generation, "model_generation"),
        _uint64(stamp.reset_generation, "reset_generation"),
        _uint64(stamp.sequence, "sequence"),
        timestamp_ns,
        frame_field,
    )
    return header + _IMU_PAYLOAD.pack(
        *sample.orientation_wxyz,
        *sample.angular_velocity_rps,
        *sample.linear_acceleration_mps2,
    )


def encode_mid360_frame_sample(sample: Mid360FrameSample) -> bytes:
    """Encode one validated MID-360 frame as a bounded LTU1 binary cloud record."""

    if not isinstance(sample, Mid360FrameSample):
        raise TypeError("sample must be a Mid360FrameSample")
    stamp = sample.stamp
    _session_id(stamp.session_id)
    timestamp_ns = _uint64(stamp.sim_time_ns, "sim_time_ns")
    if timestamp_ns > _MAXIMUM_IDL_TIME_NS:
        raise ValueError("sim_time_ns is outside the canonical IDL Time range")
    if len(sample.points) > 1_000_000:
        raise ValueError("MID-360 frame exceeds the bounded LTU1 point limit")

    payload = bytearray(len(sample.points) * _LIVOX_POINT_PAYLOAD.size)
    previous_offset = -1
    for index, point in enumerate(sample.points):
        if point.offset_time_ns < previous_offset:
            raise ValueError("MID-360 point offsets must be non-decreasing")
        previous_offset = point.offset_time_ns
        _LIVOX_POINT_PAYLOAD.pack_into(
            payload,
            index * _LIVOX_POINT_PAYLOAD.size,
            float(point.x),
            float(point.y),
            float(point.z),
            float(point.reflectivity),
            int(point.offset_time_ns),
            int(point.tag),
            int(point.line),
            0,
        )
    return _LTU1_HEADER.pack(
        b"LTU1",
        _LTU1_CLOUD_RECORD,
        timestamp_ns,
        int(stamp.sequence) & 0xFFFFFFFF,
        len(sample.points),
        len(payload),
    ) + bytes(payload)


def _frame(value: str, field: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or any(character.isspace() for character in value)
    ):
        raise ValueError(f"{field} must be one non-empty frame token")
    return value


class TruthOdometryDdsAdapter:
    """Own one native publisher process for a single simulation run."""

    def __init__(
        self,
        executable: Path,
        *,
        allocation: RunAllocation,
        parent_frame: str,
        child_frame: str,
        readiness_timeout_s: float = 10.0,
        shutdown_timeout_s: float = 5.0,
    ) -> None:
        self._executable = Path(executable).resolve()
        self._dds_domain = allocation.dds_domain
        self._session_id = _session_id(allocation.session_id, "allocation.session_id")
        self._child_environment = allocation.child_environment()
        self._ready_file = Path(allocation.run_dir) / "truth-odom-publisher.ready.json"
        self._stderr_path = Path(allocation.log_dir) / "truth-odom-publisher.stderr.log"
        self._parent_frame = _frame(parent_frame, "parent_frame")
        self._child_frame = _frame(child_frame, "child_frame")
        self._readiness_timeout_s = readiness_timeout_s
        self._shutdown_timeout_s = shutdown_timeout_s
        self._process: subprocess.Popen[bytes] | None = None
        self._process_owner: ProcessTreeOwner | None = None
        self._stderr: BinaryIO | None = None

        if (
            isinstance(self._dds_domain, bool)
            or not isinstance(self._dds_domain, int)
            or not 0 <= self._dds_domain <= 232
        ):
            raise ValueError("allocation.dds_domain must be in the range 0..232")
        if (
            isinstance(readiness_timeout_s, bool)
            or not isinstance(readiness_timeout_s, (int, float))
            or readiness_timeout_s <= 0
        ):
            raise ValueError("readiness_timeout_s must be positive")
        if (
            isinstance(shutdown_timeout_s, bool)
            or not isinstance(shutdown_timeout_s, (int, float))
            or shutdown_timeout_s <= 0
        ):
            raise ValueError("shutdown_timeout_s must be positive")

    @property
    def pid(self) -> int | None:
        """Return the owned child PID after startup."""

        return self._process.pid if self._process is not None else None

    def start(self) -> dict[str, Any]:
        """Launch the publisher and wait until its DDS writer exists."""

        if self._process is not None:
            raise TruthOdometryAdapterError("truth odometry publisher is already started")
        if not self._executable.is_file():
            raise TruthOdometryAdapterError(f"truth odometry publisher does not exist: {self._executable}")
        self._ready_file.parent.mkdir(parents=True, exist_ok=True)
        self._ready_file.unlink(missing_ok=True)
        self._stderr_path.parent.mkdir(parents=True, exist_ok=True)
        self._stderr = self._stderr_path.open("wb")

        command = [
            str(self._executable),
            "--dds-domain",
            str(self._dds_domain),
            "--session-id",
            self._session_id,
            "--parent-frame",
            self._parent_frame,
            "--child-frame",
            self._child_frame,
            "--ready-file",
            str(self._ready_file),
        ]
        creationflags = (
            subprocess.CREATE_NO_WINDOW
            if os.name == "nt" and hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        self._process_owner = ProcessTreeOwner()
        try:
            self._process = subprocess.Popen(  # noqa: S603 - fixed executable and argv
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
                bufsize=0,
                env=self._child_environment,
                **self._process_owner.popen_options(creationflags=creationflags),
            )
            self._process_owner.attach(self._process)
            return self._wait_for_readiness()
        except Exception:
            self._terminate()
            raise

    def publish(self, sample: TruthOdometrySample) -> None:
        """Write one sample to the strict native protocol."""

        process = self._process
        if process is None or process.stdin is None:
            raise TruthOdometryAdapterError("truth odometry publisher is not started")
        if process.poll() is not None:
            raise TruthOdometryAdapterError(f"truth odometry publisher exited with code {process.returncode}")
        if sample.stamp.session_id != self._session_id:
            raise TruthOdometryAdapterError("truth odometry sample session does not match the run allocation")
        try:
            process.stdin.write(encode_truth_odometry_sample(sample))
            process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise TruthOdometryAdapterError("cannot write truth odometry sample to publisher") from exc

    def close(self) -> None:
        """Close stdin and require a clean publisher exit."""

        process = self._process
        if process is None:
            return
        try:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
            returncode = process.wait(timeout=self._shutdown_timeout_s)
            if returncode != 0:
                raise TruthOdometryAdapterError(f"truth odometry publisher exited with code {returncode}")
        except subprocess.TimeoutExpired as exc:
            self._terminate()
            raise TruthOdometryAdapterError("truth odometry publisher did not stop after stdin EOF") from exc
        finally:
            self._process = None
            self._ready_file.unlink(missing_ok=True)
            if self._process_owner is not None:
                self._process_owner.close_after_exit()
                self._process_owner = None
            self._close_stderr()

    def _wait_for_readiness(self) -> dict[str, Any]:
        deadline = time.monotonic() + float(self._readiness_timeout_s)
        last_read_error: OSError | UnicodeError | json.JSONDecodeError | None = None
        while time.monotonic() < deadline:
            process = self._process
            if process is None:
                break
            if process.poll() is not None:
                raise TruthOdometryAdapterError(
                    f"truth odometry publisher exited before DDS readiness (code={process.returncode})"
                )
            if self._ready_file.is_file():
                try:
                    document = json.loads(self._ready_file.read_text(encoding="utf-8"))
                except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                    last_read_error = exc
                    time.sleep(0.01)
                    continue
                expected = {
                    "schema": _READY_SCHEMA,
                    "ready": True,
                    "dds_domain": self._dds_domain,
                    "session_id": self._session_id,
                    "topic": _TRUTH_ODOMETRY_TOPIC,
                }
                if type(document) is not dict or document != expected:
                    raise TruthOdometryAdapterError("truth odometry publisher readiness does not match this run")
                return document
            time.sleep(0.01)
        if last_read_error is not None:
            raise TruthOdometryAdapterError("truth odometry publisher emitted invalid readiness") from last_read_error
        raise TruthOdometryAdapterError("truth odometry DDS readiness timed out")

    def _terminate(self) -> None:
        process = self._process
        if process is not None and self._process_owner is not None:
            self._process_owner.terminate(process, timeout_s=2.0)
        elif process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process is not None and process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        self._process = None
        self._ready_file.unlink(missing_ok=True)
        if self._process_owner is not None:
            self._process_owner.close()
            self._process_owner = None

        self._close_stderr()

    def _close_stderr(self) -> None:
        if self._stderr is not None:
            self._stderr.close()
            self._stderr = None


class ImuDdsAdapter:
    """Own one native LTIM-to-typed-DDS publisher for a simulation run."""

    def __init__(
        self,
        executable: Path,
        *,
        allocation: RunAllocation,
        readiness_timeout_s: float = 10.0,
        shutdown_timeout_s: float = 5.0,
    ) -> None:
        self._executable = Path(executable).resolve()
        self._dds_domain = allocation.dds_domain
        self._session_id = _session_id(allocation.session_id, "allocation.session_id")
        self._child_environment = allocation.child_environment()
        self._ready_file = Path(allocation.run_dir) / "imu-publisher.ready.json"
        self._stderr_path = Path(allocation.log_dir) / "imu-publisher.stderr.log"
        self._readiness_timeout_s = readiness_timeout_s
        self._shutdown_timeout_s = shutdown_timeout_s
        self._process: subprocess.Popen[bytes] | None = None
        self._process_owner: ProcessTreeOwner | None = None
        self._stderr: BinaryIO | None = None
        if (
            isinstance(self._dds_domain, bool)
            or not isinstance(self._dds_domain, int)
            or not 0 <= self._dds_domain <= 232
        ):
            raise ValueError("allocation.dds_domain must be in the range 0..232")
        if readiness_timeout_s <= 0 or shutdown_timeout_s <= 0:
            raise ValueError("IMU adapter timeouts must be positive")

    def start(self) -> dict[str, Any]:
        """Launch the native IMU publisher and wait for its DDS writer."""
        if self._process is not None:
            raise ImuAdapterError("IMU publisher is already started")
        if not self._executable.is_file():
            raise ImuAdapterError(f"IMU publisher does not exist: {self._executable}")
        self._ready_file.parent.mkdir(parents=True, exist_ok=True)
        self._ready_file.unlink(missing_ok=True)
        self._stderr_path.parent.mkdir(parents=True, exist_ok=True)
        self._stderr = self._stderr_path.open("wb")
        command = [
            str(self._executable),
            "--dds-domain", str(self._dds_domain),
            "--session-id", self._session_id,
            "--ready-file", str(self._ready_file),
        ]
        creationflags = (
            subprocess.CREATE_NO_WINDOW
            if os.name == "nt" and hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        self._process_owner = ProcessTreeOwner()
        try:
            self._process = subprocess.Popen(  # noqa: S603 - fixed executable and argv
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
                bufsize=0,
                env=self._child_environment,
                **self._process_owner.popen_options(creationflags=creationflags),
            )
            self._process_owner.attach(self._process)
            return self._wait_for_readiness()
        except Exception:
            self._terminate()
            raise

    def publish(self, sample: ImuSample) -> None:
        """Write one complete SI-unit IMU sample to the native process."""
        process = self._process
        if process is None or process.stdin is None:
            raise ImuAdapterError("IMU publisher is not started")
        if process.poll() is not None:
            raise ImuAdapterError(f"IMU publisher exited with code {process.returncode}")
        if sample.stamp.session_id != self._session_id:
            raise ImuAdapterError("IMU sample session does not match the run allocation")
        try:
            process.stdin.write(encode_imu_sample(sample))
            process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise ImuAdapterError("cannot write IMU sample to publisher") from exc

    def close(self) -> None:
        """Close stdin and require the publisher to exit cleanly."""
        process = self._process
        if process is None:
            return
        try:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
            returncode = process.wait(timeout=self._shutdown_timeout_s)
            if returncode != 0:
                raise ImuAdapterError(f"IMU publisher exited with code {returncode}")
        except subprocess.TimeoutExpired as exc:
            self._terminate()
            raise ImuAdapterError("IMU publisher did not stop after stdin EOF") from exc
        finally:
            self._process = None
            self._ready_file.unlink(missing_ok=True)
            if self._process_owner is not None:
                self._process_owner.close_after_exit()
                self._process_owner = None
            self._close_stderr()

    def _wait_for_readiness(self) -> dict[str, Any]:
        deadline = time.monotonic() + float(self._readiness_timeout_s)
        while time.monotonic() < deadline:
            process = self._process
            if process is None:
                break
            if process.poll() is not None:
                raise ImuAdapterError(
                    f"IMU publisher exited before DDS readiness (code={process.returncode})"
                )
            if self._ready_file.is_file():
                try:
                    document = json.loads(self._ready_file.read_text(encoding="utf-8"))
                except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                    raise ImuAdapterError("IMU publisher emitted invalid readiness") from exc
                expected = {
                    "schema": _IMU_READY_SCHEMA,
                    "ready": True,
                    "dds_domain": self._dds_domain,
                    "session_id": self._session_id,
                    "topic": _IMU_TOPIC,
                }
                if type(document) is not dict or document != expected:
                    raise ImuAdapterError("IMU publisher readiness does not match this run")
                return document
            time.sleep(0.01)
        raise ImuAdapterError("IMU DDS readiness timed out")

    def _terminate(self) -> None:
        process = self._process
        if process is not None and self._process_owner is not None:
            self._process_owner.terminate(process, timeout_s=2.0)
        elif process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process is not None and process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        self._process = None
        if self._process_owner is not None:
            self._process_owner.close()
            self._process_owner = None

        self._close_stderr()

    def _close_stderr(self) -> None:
        if self._stderr is not None:
            self._stderr.close()
            self._stderr = None


class Mid360DdsAdapter:
    """Own one native LTU1-to-sim-LivoxFrame publisher for MID-360 frames."""

    def __init__(
        self,
        executable: Path,
        *,
        allocation: RunAllocation,
        frame_id: str,
        readiness_timeout_s: float = 10.0,
        shutdown_timeout_s: float = 5.0,
    ) -> None:
        self._executable = Path(executable).resolve()
        self._dds_domain = allocation.dds_domain
        self._session_id = _session_id(allocation.session_id, "allocation.session_id")
        self._child_environment = allocation.child_environment()
        self._ready_file = Path(allocation.run_dir) / "mid360-publisher.ready.json"
        self._stderr_path = Path(allocation.log_dir) / "mid360-publisher.stderr.log"
        self._frame_id = _frame(frame_id, "frame_id")
        self._readiness_timeout_s = readiness_timeout_s
        self._shutdown_timeout_s = shutdown_timeout_s
        self._process: subprocess.Popen[bytes] | None = None
        self._process_owner: ProcessTreeOwner | None = None
        self._stderr: BinaryIO | None = None
        self._last_generation: tuple[int, int] | None = None
        self._last_sequence: int | None = None
        self._last_time_ns: int | None = None
        self._active = False
        if (
            isinstance(self._dds_domain, bool)
            or not isinstance(self._dds_domain, int)
            or not 0 <= self._dds_domain <= 232
        ):
            raise ValueError("allocation.dds_domain must be in the range 0..232")
        if readiness_timeout_s <= 0 or shutdown_timeout_s <= 0:
            raise ValueError("MID-360 adapter timeouts must be positive")

    @property
    def active(self) -> bool:
        """Return true only after the first non-empty frame is accepted by the writer."""

        return self._active

    def start(self) -> dict[str, Any]:
        """Launch the native publisher and wait until its sim DDS writer exists."""

        if self._process is not None:
            raise Mid360AdapterError("MID-360 publisher is already started")
        if not self._executable.is_file():
            raise Mid360AdapterError(f"MID-360 publisher does not exist: {self._executable}")
        self._ready_file.parent.mkdir(parents=True, exist_ok=True)
        self._ready_file.unlink(missing_ok=True)
        self._stderr_path.parent.mkdir(parents=True, exist_ok=True)
        self._stderr = self._stderr_path.open("wb")
        command = [
            str(self._executable),
            "--stdin-records",
            "--dds",
            "--domain-id",
            str(self._dds_domain),
            "--lidar-frame",
            self._frame_id,
            "--scan-window",
            "0",
            "--sim-lidar-raw-frame",
            "--ready-file",
            str(self._ready_file),
        ]
        creationflags = (
            subprocess.CREATE_NO_WINDOW
            if os.name == "nt" and hasattr(subprocess, "CREATE_NO_WINDOW")
            else 0
        )
        self._process_owner = ProcessTreeOwner()
        try:
            self._process = subprocess.Popen(  # noqa: S603 - fixed executable and argv
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
                bufsize=0,
                env=self._child_environment,
                **self._process_owner.popen_options(creationflags=creationflags),
            )
            self._process_owner.attach(self._process)
            return self._wait_for_readiness()
        except Exception:
            self._terminate()
            raise

    def publish(self, sample: Mid360FrameSample) -> None:
        """Write one generation-checked MID-360 frame to the native process."""

        process = self._process
        if process is None or process.stdin is None:
            raise Mid360AdapterError("MID-360 publisher is not started")
        if process.poll() is not None:
            raise Mid360AdapterError(f"MID-360 publisher exited with code {process.returncode}")
        stamp = sample.stamp
        if stamp.session_id != self._session_id:
            raise Mid360AdapterError("MID-360 sample session does not match the run allocation")
        generation = (stamp.model_generation, stamp.reset_generation)
        if self._last_generation is not None:
            if generation < self._last_generation:
                raise Mid360AdapterError("MID-360 sample generation moved backward")
            same_generation = generation == self._last_generation
            if same_generation and (
                self._last_sequence is None
                or stamp.sequence <= self._last_sequence
                or self._last_time_ns is None
                or stamp.sim_time_ns <= self._last_time_ns
            ):
                raise Mid360AdapterError("MID-360 sample sequence/time is not increasing")
        try:
            process.stdin.write(encode_mid360_frame_sample(sample))
            process.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise Mid360AdapterError("cannot write MID-360 sample to publisher") from exc
        self._last_generation = generation
        self._last_sequence = stamp.sequence
        self._last_time_ns = stamp.sim_time_ns
        self._active = True

    def close(self) -> None:
        """Close stdin and require a clean publisher exit."""

        process = self._process
        if process is None:
            return
        try:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
            returncode = process.wait(timeout=self._shutdown_timeout_s)
            if returncode != 0:
                raise Mid360AdapterError(f"MID-360 publisher exited with code {returncode}")
        except subprocess.TimeoutExpired as exc:
            self._terminate()
            raise Mid360AdapterError("MID-360 publisher did not stop after stdin EOF") from exc
        finally:
            self._process = None
            self._ready_file.unlink(missing_ok=True)
            if self._process_owner is not None:
                self._process_owner.close_after_exit()
                self._process_owner = None
            self._close_stderr()

    def _wait_for_readiness(self) -> dict[str, Any]:
        deadline = time.monotonic() + float(self._readiness_timeout_s)
        while time.monotonic() < deadline:
            process = self._process
            if process is None:
                break
            if process.poll() is not None:
                raise Mid360AdapterError(
                    f"MID-360 publisher exited before DDS readiness (code={process.returncode})"
                )
            if self._ready_file.is_file():
                try:
                    document = json.loads(self._ready_file.read_text(encoding="utf-8"))
                except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                    raise Mid360AdapterError("MID-360 publisher emitted invalid readiness") from exc
                expected = {
                    "schema": "lingtu.mujoco_sensor_publisher.ready.v1",
                    "ready": True,
                    "topic": _MID360_TOPIC,
                }
                if (
                    type(document) is not dict
                    or document.get("schema") != expected["schema"]
                    or document.get("ready") is not True
                    or document.get("topic") != expected["topic"]
                ):
                    raise Mid360AdapterError("MID-360 publisher readiness does not match this run")
                return document
            time.sleep(0.01)
        raise Mid360AdapterError("MID-360 DDS readiness timed out")

    def _terminate(self) -> None:
        process = self._process
        if process is not None and self._process_owner is not None:
            self._process_owner.terminate(process, timeout_s=2.0)
        elif process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        if process is not None and process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        self._process = None
        self._ready_file.unlink(missing_ok=True)
        if self._process_owner is not None:
            self._process_owner.close()
            self._process_owner = None

        self._close_stderr()

    def _close_stderr(self) -> None:
        if self._stderr is not None:
            self._stderr.close()
            self._stderr = None
__all__ = [
    "ImuAdapterError",
    "ImuDdsAdapter",
    "Mid360AdapterError",
    "Mid360DdsAdapter",
    "TruthOdometryAdapterError",
    "TruthOdometryDdsAdapter",
    "encode_imu_sample",
    "encode_mid360_frame_sample",
    "encode_truth_odometry_sample",
]
