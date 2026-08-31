#!/usr/bin/env python3
"""Capture unique native SLAM status snapshots as evaluation JSONL evidence."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path
from typing import Any, Callable, Sequence

EXPECTED_SCHEMA = "lingtu.slam.status_snapshot.v1"
CAPTURE_SCHEMA = "lingtu.slam.status_capture.v1"


class SnapshotUnavailable(RuntimeError):
    """A transient snapshot read failure that may resolve on the next poll."""

    def __init__(self, kind: str) -> None:
        super().__init__(kind)
        self.kind = kind


class SnapshotContractError(ValueError):
    """A valid JSON document that is not a native SLAM status snapshot."""


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        text=True,
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise


def _read_snapshot(path: Path) -> dict[str, Any]:
    try:
        raw = path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise SnapshotUnavailable("missing") from exc
    except OSError as exc:
        raise SnapshotUnavailable("io_error") from exc
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise SnapshotUnavailable("invalid_json") from exc
    if not isinstance(payload, dict):
        raise SnapshotContractError("snapshot root must be a JSON object")
    return payload


def _snapshot_identity(payload: dict[str, Any]) -> tuple[str, int]:
    schema = payload.get("schema_version")
    if schema != EXPECTED_SCHEMA:
        raise SnapshotContractError(f"schema_version must be {EXPECTED_SCHEMA!r}, got {schema!r}")
    runtime_id = payload.get("runtime_instance_id")
    if not isinstance(runtime_id, str) or not runtime_id.strip():
        raise SnapshotContractError("runtime_instance_id must be a non-empty string")
    sequence = payload.get("observation_sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise SnapshotContractError("observation_sequence must be a non-negative integer")
    stamp_s = payload.get("stamp_s")
    if isinstance(stamp_s, bool) or not isinstance(stamp_s, (int, float)):
        raise SnapshotContractError("stamp_s must be numeric")
    if not math.isfinite(float(stamp_s)):
        raise SnapshotContractError("stamp_s must be finite")
    return runtime_id, sequence


def capture_snapshot_history(
    source: Path,
    output: Path,
    *,
    duration_s: float,
    poll_hz: float,
    min_samples: int,
    min_observed_duration_s: float | None = None,
    overwrite: bool = False,
    monotonic: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
    reader: Callable[[], dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Capture unique observations; duplicate frozen snapshots never count as evidence."""

    if duration_s < 0.0:
        raise ValueError("duration_s must be non-negative")
    if not math.isfinite(duration_s):
        raise ValueError("duration_s must be finite")
    if poll_hz <= 0.0 or not math.isfinite(poll_hz):
        raise ValueError("poll_hz must be finite and positive")
    if min_samples < 1:
        raise ValueError("min_samples must be at least 1")
    if min_observed_duration_s is None:
        min_observed_duration_s = max(0.0, duration_s - max(1.0, 2.0 / poll_hz))
    if min_observed_duration_s < 0.0 or not math.isfinite(min_observed_duration_s):
        raise ValueError("min_observed_duration_s must be finite and non-negative")
    if output.exists() and not overwrite:
        raise FileExistsError(f"refusing to overwrite existing evidence: {output}")

    output.parent.mkdir(parents=True, exist_ok=True)
    read = reader or (lambda: _read_snapshot(source))
    unavailable_reads: Counter[str] = Counter()
    seen: set[tuple[str, int]] = set()
    runtime_ids: set[str] = set()
    duplicate_reads = 0
    poll_count = 0
    first_stamp_s: float | None = None
    last_stamp_s: float | None = None
    start_s = monotonic()
    period_s = 1.0 / poll_hz

    mode = "w" if overwrite else "x"
    try:
        with output.open(mode, encoding="utf-8", newline="\n") as handle:
            while True:
                poll_count += 1
                try:
                    payload = read()
                except SnapshotUnavailable as exc:
                    unavailable_reads[exc.kind] += 1
                else:
                    identity = _snapshot_identity(payload)
                    if identity in seen:
                        duplicate_reads += 1
                    else:
                        serialized = json.dumps(
                            payload,
                            sort_keys=True,
                            separators=(",", ":"),
                            allow_nan=False,
                        )
                        handle.write(serialized + "\n")
                        handle.flush()
                        seen.add(identity)
                        runtime_ids.add(identity[0])
                        stamp_s = float(payload["stamp_s"])
                        if first_stamp_s is None:
                            first_stamp_s = stamp_s
                        last_stamp_s = stamp_s

                elapsed_s = monotonic() - start_s
                if elapsed_s >= duration_s:
                    break
                sleeper(min(period_s, duration_s - elapsed_s))
            os.fsync(handle.fileno())
    except BaseException:
        if not seen:
            output.unlink(missing_ok=True)
        raise

    unique_samples = len(seen)
    observed_duration_s = round(
        max(0.0, last_stamp_s - first_stamp_s) if first_stamp_s is not None and last_stamp_s is not None else 0.0,
        9,
    )
    blockers: list[str] = []
    if unique_samples < min_samples:
        blockers.append("insufficient_unique_samples")
    if observed_duration_s < min_observed_duration_s:
        blockers.append("insufficient_observed_duration")
    return {
        "schema_version": CAPTURE_SCHEMA,
        "status": "COMPLETE" if not blockers else "INCOMPLETE",
        "motion_authorization": False,
        "blockers": blockers,
        "source": str(source),
        "output": str(output),
        "requested_duration_s": duration_s,
        "poll_hz": poll_hz,
        "min_samples": min_samples,
        "min_observed_duration_s": min_observed_duration_s,
        "poll_count": poll_count,
        "unique_samples": unique_samples,
        "duplicate_reads": duplicate_reads,
        "unavailable_reads": dict(sorted(unavailable_reads.items())),
        "runtime_instance_ids": sorted(runtime_ids),
        "first_stamp_s": first_stamp_s,
        "last_stamp_s": last_stamp_s,
        "observed_duration_s": observed_duration_s,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture unique C++ SLAM status snapshots as JSONL. Frozen duplicate "
            "snapshots do not count, and this command never authorizes motion."
        )
    )
    parser.add_argument("source", type=Path, help="atomic SLAM status snapshot JSON")
    parser.add_argument("output", type=Path, help="destination JSONL evidence file")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--poll-hz", type=float, default=20.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument(
        "--min-observed-duration-s",
        type=float,
        help="required sensor-time span; defaults to requested duration minus 1 second",
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--summary", type=Path, help="write capture summary atomically")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        summary = capture_snapshot_history(
            args.source,
            args.output,
            duration_s=args.duration_s,
            poll_hz=args.poll_hz,
            min_samples=args.min_samples,
            min_observed_duration_s=args.min_observed_duration_s,
            overwrite=args.overwrite,
        )
    except (OSError, ValueError) as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 1
    if args.summary is not None:
        _atomic_write_json(args.summary, summary)
    else:
        print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
