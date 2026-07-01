#!/usr/bin/env python3
"""Emit Livox SDK2 stream records without hardware."""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import struct
import sys
import time

HEADER = struct.Struct("<4sB3xQIII")
POINT = struct.Struct("<ffffIBBH")
IMU = struct.Struct("<ffffff")
MAGIC = b"LTU1"
RECORD_CLOUD = 1
RECORD_IMU = 2

_stop = False


def _signal_stop(_signum, _frame) -> None:
    global _stop
    _stop = True


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def _write_record(record_type: int, timestamp_ns: int, sequence: int, count: int, payload: bytes) -> None:
    out = sys.stdout.buffer
    out.write(HEADER.pack(MAGIC, record_type, timestamp_ns, sequence, count, len(payload)))
    out.write(payload)
    out.flush()


def _cloud_payload(frame: int, point_count: int) -> bytes:
    chunks = []
    for i in range(point_count):
        angle = (i / max(point_count, 1)) * math.tau
        radius = 3.0 + 0.02 * frame
        chunks.append(
            POINT.pack(
                radius * math.cos(angle),
                radius * math.sin(angle),
                -0.2 + 0.01 * (i % 40),
                20.0 + float(i % 10),
                i * 1_000_000,
                0,
                i % 4,
                0,
            )
        )
    return b"".join(chunks)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: livox_stream_sim.py <MID360_config.json>", file=sys.stderr)
        return 2
    if not Path(argv[1]).exists():
        print(f"livox_stream_sim config not found: {argv[1]}", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, _signal_stop)
    signal.signal(signal.SIGTERM, _signal_stop)

    frames = _env_int("LINGTU_LIVOX_SIM_FRAMES", 0)
    point_count = max(1, _env_int("LINGTU_LIVOX_SIM_POINTS", 240))
    imu_per_frame = max(1, _env_int("LINGTU_LIVOX_SIM_IMU_PER_FRAME", 2))
    hz = max(0.1, _env_float("LINGTU_LIVOX_SIM_HZ", 10.0))
    period_s = 1.0 / hz
    period_ns = int(period_s * 1_000_000_000)
    start_ns = time.time_ns()
    seq = 0
    frame = 0

    print("livox_stream_sim started", file=sys.stderr, flush=True)
    try:
        while not _stop and (frames <= 0 or frame < frames):
            stamp_ns = start_ns + frame * period_ns
            for j in range(imu_per_frame):
                imu_stamp = stamp_ns + j * max(1, period_ns // imu_per_frame)
                imu = IMU.pack(0.0, 0.0, 0.02, 0.0, 0.0, 9.81)
                _write_record(RECORD_IMU, imu_stamp, seq, 1, imu)
                seq += 1
            cloud = _cloud_payload(frame, point_count)
            _write_record(RECORD_CLOUD, stamp_ns, seq, point_count, cloud)
            seq += 1
            frame += 1
            if frames <= 0 or frame < frames:
                time.sleep(period_s)
    except BrokenPipeError:
        return 0
    finally:
        print("livox_stream_sim stopped", file=sys.stderr, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
