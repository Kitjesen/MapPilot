#!/usr/bin/env python3
"""Emit Livox SDK2 stream records without hardware."""

from __future__ import annotations

import argparse
import math
import os
import signal
import struct
import sys
import time
from pathlib import Path

import yaml

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
    settings_path = Path(__file__).resolve().parents[2] / "config/system/settings/lidar.yaml"
    defaults = yaml.safe_load(settings_path.read_text(encoding="utf-8"))["lidar"]
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    parser.add_argument("--publish-freq", type=float, default=defaults["scan_hz"])
    parser.add_argument("--imu-publish-freq", type=float, default=defaults["imu_hz"])
    # LTU1 records have no frame-name field; the receiving adapter supplies it.
    parser.add_argument("--lidar-frame")
    parser.add_argument("--imu-frame")
    args = parser.parse_args(argv[1:])
    if not math.isfinite(args.publish_freq) or args.publish_freq <= 0:
        parser.error("--publish-freq must be finite and positive")
    if not math.isfinite(args.imu_publish_freq) or args.imu_publish_freq < 0:
        parser.error("--imu-publish-freq must be finite and non-negative")
    if not Path(args.config).exists():
        print(f"livox_stream_sim config not found: {args.config}", file=sys.stderr)
        return 1

    signal.signal(signal.SIGINT, _signal_stop)
    signal.signal(signal.SIGTERM, _signal_stop)

    frames = _env_int("LINGTU_LIVOX_SIM_FRAMES", 0)
    point_count = max(1, _env_int("LINGTU_LIVOX_SIM_POINTS", 240))
    imu_per_frame = max(1, _env_int("LINGTU_LIVOX_SIM_IMU_PER_FRAME", 2))
    hz = max(0.1, _env_float("LINGTU_LIVOX_SIM_HZ", args.publish_freq))
    imu_period_ns = max(1, int(1_000_000_000 / args.imu_publish_freq)) if args.imu_publish_freq else 0
    period_s = 1.0 / hz
    period_ns = int(period_s * 1_000_000_000)
    start_ns = time.time_ns()
    next_imu_ns = start_ns
    seq = 0
    frame = 0

    print("livox_stream_sim started", file=sys.stderr, flush=True)
    try:
        while not _stop and (frames <= 0 or frame < frames):
            stamp_ns = start_ns + frame * period_ns
            if imu_period_ns:
                imu_times = range(next_imu_ns, stamp_ns + period_ns, imu_period_ns)
                next_imu_ns += len(imu_times) * imu_period_ns
            else:
                step_ns = max(1, period_ns // imu_per_frame)
                imu_times = range(stamp_ns, stamp_ns + imu_per_frame * step_ns, step_ns)
            for imu_stamp in imu_times:
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
