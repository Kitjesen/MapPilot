"""Native GNSS DDS boundary using the existing LingTu fix/status IDL."""

from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, BinaryIO
from urllib.parse import quote

from sim.runtime.process_owner import ProcessTreeOwner

from .dds_adapter import _frame, _session_id, _uint64
from .gnss import GnssSample

if TYPE_CHECKING:
    from sim.runtime.coordinator.run_allocation import RunAllocation

_HEADER = struct.Struct("<4sHHII64sQQQQ64s")
_PAYLOAD = struct.Struct("<iII14d")
_MAX_IDL_TIME_NS = ((1 << 31) - 1) * 1_000_000_000 + 999_999_999


def encode_gnss_sample(sample: GnssSample) -> bytes:
    """Encode the local LTGN input record; the native writer owns DDS serialization."""
    if not isinstance(sample, GnssSample):
        raise TypeError("sample must be a GnssSample")
    stamp, fix = sample.stamp, sample.fix
    session = _session_id(stamp.session_id).encode("ascii")
    frame = _frame(stamp.frame_id, "frame_id").encode("ascii")
    if len(frame) > 63:
        raise ValueError("GNSS frame_id must fit 63 ASCII bytes")
    for field in ("model_generation", "reset_generation", "sequence", "sim_time_ns"):
        _uint64(getattr(stamp, field), field)
    if stamp.sim_time_ns > _MAX_IDL_TIME_NS:
        raise ValueError("GNSS timestamp is outside IDL Time range")
    if fix.frame_id != stamp.frame_id or fix.seq != stamp.sequence or fix.ts != stamp.sim_time_ns / 1_000_000_000:
        raise ValueError("GNSS payload identity/time disagrees with its stamp")
    if fix.fix_type not in (0, 8) or int(fix.fix_type) != (8 if sample.configuration.fix_available else 0):
        raise ValueError("GNSS simulation supports only explicit NO_FIX or SIMULATION")
    values = (fix.lat, fix.lon, fix.alt, *fix.covariance)
    if len(values) != 12:
        raise ValueError("GNSS covariance must contain nine values")
    if fix.fix_type == 0:
        if not all(math.isnan(value) for value in values):
            raise ValueError("GNSS NO_FIX must not carry position or covariance")
    elif (
        not all(math.isfinite(value) for value in values)
        or not -90 <= fix.lat <= 90
        or not -180 <= fix.lon <= 180
        or any(fix.covariance[index] < 0 for index in (0, 4, 8))
    ):
        raise ValueError("GNSS fix requires valid WGS84 coordinates and covariance")
    if fix.num_sat != 0 or fix.num_sat_used != 0:
        raise ValueError("GNSS geometric model has no simulated constellation")
    return _HEADER.pack(
        b"LTGN",
        1,
        7,
        _HEADER.size,
        _PAYLOAD.size,
        session,
        stamp.model_generation,
        stamp.reset_generation,
        stamp.sequence,
        stamp.sim_time_ns,
        frame,
    ) + _PAYLOAD.pack(int(fix.fix_type), 0, 0, *values, math.nan, math.nan)


class GnssDdsAdapter:
    """Own the native GNSS writer, scoped to one sensor and run allocation."""

    def __init__(
        self,
        executable: Path,
        *,
        allocation: RunAllocation,
        sensor_id: str,
        frame_id: str,
        readiness_timeout_s: float = 10.0,
        shutdown_timeout_s: float = 5.0,
    ) -> None:
        self._executable = Path(executable).resolve()
        self._session_id = _session_id(allocation.session_id)
        self._domain = allocation.dds_domain
        if type(self._domain) is not int or not 0 <= self._domain <= 232:
            raise ValueError("GNSS DDS domain must be in 0..232")
        self._sensor_id = _frame(sensor_id, "sensor_id")
        self._frame_id = _frame(frame_id, "frame_id")
        self._environment = allocation.child_environment()
        basename = "gnss-" + quote(sensor_id, safe="")
        self._ready_file = Path(allocation.run_dir) / f"{basename}.ready.json"
        self._stderr_path = Path(allocation.log_dir) / f"{basename}.stderr.log"
        for value in (readiness_timeout_s, shutdown_timeout_s):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("GNSS timeouts must be positive finite seconds")
        self._readiness_timeout_s = readiness_timeout_s
        self._shutdown_timeout_s = shutdown_timeout_s
        self._process: subprocess.Popen[bytes] | None = None
        self._owner: ProcessTreeOwner | None = None
        self._stderr: BinaryIO | None = None

    def start(self) -> dict[str, Any]:
        """Wait until the native fix/status DDS writers are created."""
        if self._process is not None:
            raise RuntimeError("GNSS publisher is already started")
        if not self._executable.is_file():
            raise RuntimeError(f"GNSS publisher does not exist: {self._executable}")
        self._ready_file.parent.mkdir(parents=True, exist_ok=True)
        self._ready_file.unlink(missing_ok=True)
        self._stderr_path.parent.mkdir(parents=True, exist_ok=True)
        self._stderr = self._stderr_path.open("wb")
        self._owner = ProcessTreeOwner()
        try:
            self._process = subprocess.Popen(
                [
                    str(self._executable),
                    "--dds-domain",
                    str(self._domain),
                    "--session-id",
                    self._session_id,
                    "--ready-file",
                    str(self._ready_file),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._stderr,
                bufsize=0,
                env=self._environment,
                **self._owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0),
            )
            self._owner.attach(self._process)
            expected = {
                "schema": "lingtu.sim.gnss-publisher.ready.v1",
                "ready": True,
                "dds_domain": self._domain,
                "session_id": self._session_id,
                "topic": "rt/gnss/fix",
                "status_topic": "rt/gnss/status",
            }
            deadline = time.monotonic() + self._readiness_timeout_s
            while time.monotonic() < deadline:
                if self._process.poll() is not None:
                    raise RuntimeError(f"GNSS publisher exited before readiness: {self._process.returncode}")
                if self._ready_file.is_file():
                    try:
                        ready = json.loads(self._ready_file.read_text(encoding="utf-8"))
                    except json.JSONDecodeError:
                        # Readiness may still be in the native writer's buffered write.
                        time.sleep(0.01)
                        continue
                    if ready != expected:
                        raise RuntimeError("GNSS publisher readiness does not match this run")
                    return expected
                time.sleep(0.01)
            raise RuntimeError("GNSS DDS readiness timed out")
        except BaseException:
            self._dispose(terminate=True)
            raise

    def publish(self, sample: GnssSample) -> None:
        """Send one exact stamped sample; native validation owns stream ordering."""
        process = self._process
        if process is None or process.stdin is None:
            raise RuntimeError("GNSS publisher is not started")
        if process.poll() is not None:
            raise RuntimeError(f"GNSS publisher exited with code {process.returncode}")
        if (sample.stamp.session_id, sample.stamp.sensor_id, sample.stamp.frame_id) != (
            self._session_id,
            self._sensor_id,
            self._frame_id,
        ):
            raise ValueError("GNSS sample does not belong to this run/sensor/frame")
        payload = encode_gnss_sample(sample)
        try:
            if process.stdin.write(payload) != len(payload):
                raise RuntimeError("incomplete GNSS input write")
            process.stdin.flush()
        except OSError as exc:
            raise RuntimeError("cannot write GNSS sample to publisher") from exc

    def close(self) -> None:
        """Close stdin, let the writer drain, and surface a rejected record."""
        process = self._process
        if process is None:
            return
        try:
            if process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
            result = process.wait(timeout=self._shutdown_timeout_s)
            if result != 0:
                raise RuntimeError(f"GNSS publisher exited with code {result}")
        finally:
            self._dispose(terminate=process.poll() is None)

    def _dispose(self, *, terminate: bool) -> None:
        process, owner = self._process, self._owner
        try:
            if process is not None and owner is not None:
                if terminate:
                    owner.terminate(process, timeout_s=2.0)
                else:
                    owner.close_after_exit()
        finally:
            if process is not None and process.stdin is not None and not process.stdin.closed:
                process.stdin.close()
            if owner is not None:
                owner.close()
            self._process = None
            self._owner = None
            self._ready_file.unlink(missing_ok=True)
            if self._stderr is not None:
                self._stderr.close()
                self._stderr = None
