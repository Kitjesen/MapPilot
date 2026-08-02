#!/usr/bin/env python3
"""Record bounded MapScene point counts inside one acceptance-test ROI.

This process is a read-only test observer. It consumes the product MapScene
contract through the native client C ABI and never publishes control or map
data. Only counts are persisted, so long-running acceptance logs stay bounded
by the configured sampling rate rather than point-cloud size.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import struct
import sys
import time
from pathlib import Path
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
for candidate in (ROOT, SRC):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from nav.adapters.native.abi import NativeCommandSession

_POINT = struct.Struct("<ffff")
_STOP = False


def _request_stop(_signum: int, _frame: object) -> None:
    global _STOP
    _STOP = True


def _finite_positive(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive and finite")
    return parsed


def count_points_in_roi(
    payload: bytes,
    *,
    center: tuple[float, float, float],
    half_extent: tuple[float, float, float],
) -> int:
    """Count finite XYZI records inside one closed axis-aligned map ROI."""

    if len(payload) % _POINT.size != 0:
        raise ValueError("MapScene point payload is not an XYZI float32 array")
    cx, cy, cz = center
    hx, hy, hz = half_extent
    count = 0
    for x, y, z, _intensity in _POINT.iter_unpack(payload):
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            raise ValueError("MapScene point payload contains a non-finite coordinate")
        if abs(x - cx) <= hx and abs(y - cy) <= hy and abs(z - cz) <= hz:
            count += 1
    return count


def scene_roi_record(
    scene: Mapping[str, Any],
    *,
    center: tuple[float, float, float],
    half_extent: tuple[float, float, float],
    wall_s: float | None = None,
) -> dict[str, Any]:
    """Build one compact ROI-count record from a bounded native MapScene sample."""
    clouds = scene.get("clouds")
    if not isinstance(clouds, Mapping):
        raise ValueError("MapScene payload has no clouds object")
    counts: dict[str, int] = {}
    totals: dict[str, int] = {}
    for layer in ("live", "voxel", "accumulated"):
        cloud = clouds.get(layer)
        if not isinstance(cloud, Mapping):
            raise ValueError(f"MapScene payload has no {layer} cloud")
        payload = cloud.get("points_xyzi_f32")
        if not isinstance(payload, bytes):
            raise ValueError(f"MapScene {layer} cloud is not a bounded byte payload")
        counts[layer] = count_points_in_roi(
            payload,
            center=center,
            half_extent=half_extent,
        )
        totals[layer] = int(cloud.get("point_count") or 0)
    return {
        "type": "scene",
        "wall_s": time.time() if wall_s is None else float(wall_s),
        "scene_timestamp_s": float(scene.get("timestamp_s") or 0.0),
        "frame_id": str(scene.get("frame_id") or ""),
        "producer_boot_id": str(scene.get("producer_boot_id") or ""),
        "reset_epoch": int(scene.get("reset_epoch") or 0),
        "observation_sequence": int(scene.get("observation_sequence") or 0),
        "generation": int(scene.get("generation") or 0),
        "live": bool(scene.get("live")),
        "roi_center": list(center),
        "roi_half_extent": list(half_extent),
        "roi_counts": counts,
        "layer_point_counts": totals,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True)
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--center-x", type=float, required=True)
    parser.add_argument("--center-y", type=float, required=True)
    parser.add_argument("--center-z", type=float, required=True)
    parser.add_argument("--half-x", type=_finite_positive, required=True)
    parser.add_argument("--half-y", type=_finite_positive, required=True)
    parser.add_argument("--half-z", type=_finite_positive, required=True)
    parser.add_argument("--poll-hz", type=_finite_positive, default=8.0)
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Zero runs until SIGINT/SIGTERM.",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    """Poll MapScene until the requested deadline and persist ROI count evidence."""
    center = (float(args.center_x), float(args.center_y), float(args.center_z))
    half_extent = (float(args.half_x), float(args.half_y), float(args.half_z))
    if not all(math.isfinite(value) for value in (*center, *half_extent)):
        raise ValueError("ROI values must be finite")
    duration_s = float(args.duration_s)
    if not math.isfinite(duration_s) or duration_s < 0.0:
        raise ValueError("--duration-s must be finite and non-negative")
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.unlink(missing_ok=True)
    period_s = 1.0 / float(args.poll_hz)
    deadline = time.monotonic() + duration_s if duration_s > 0.0 else None
    samples = 0
    session = NativeCommandSession(
        Path(args.library),
        domain_id=int(args.domain_id),
        timeout_ms=1000,
    )
    try:
        with output.open("a", encoding="utf-8", buffering=1) as stream:
            while not _STOP and (deadline is None or time.monotonic() < deadline):
                started = time.monotonic()
                scene = session.take_map_scene()
                if scene is not None:
                    record = scene_roi_record(
                        scene,
                        center=center,
                        half_extent=half_extent,
                    )
                    stream.write(
                        json.dumps(record, allow_nan=False, sort_keys=True) + "\n"
                    )
                    samples += 1
                remaining = period_s - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        session.close()
    print(
        json.dumps(
            {
                "type": "summary",
                "samples": samples,
                "output": str(output),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0 if samples > 0 else 3


def main() -> int:
    """Run the command-line MapScene ROI monitor."""
    signal.signal(signal.SIGINT, _request_stop)
