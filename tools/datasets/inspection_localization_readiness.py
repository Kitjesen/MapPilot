#!/usr/bin/env python3
"""Evaluate patrol localization readiness from native SLAM status JSONL."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from capture_slam_status_jsonl import capture_snapshot_history  # noqa: E402
from sim.evaluation.slam.inspection_readiness import (  # noqa: E402
    FAIL,
    INCOMPLETE,
    PASS,
    EvidenceWindow,
    InspectionLocalizationReadinessConfig,
    RelocalizationRecoveryWindow,
    evaluate_inspection_localization_readiness,
)

_STATUS_TO_RC = {PASS: 0, FAIL: 1, INCOMPLETE: 2}


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
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            if not raw_line.strip():
                continue
            try:
                sample = json.loads(raw_line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number} is invalid JSON: {exc.msg}") from exc
            if not isinstance(sample, dict):
                raise ValueError(f"{path}:{line_number} must be a JSON object")
            samples.append(sample)
    return samples


def _window(values: Sequence[float], label: str) -> EvidenceWindow:
    if len(values) != 2:
        raise ValueError(f"{label} requires START END")
    return EvidenceWindow(float(values[0]), float(values[1]))


def _full_window(samples: Sequence[dict[str, Any]]) -> EvidenceWindow:
    if not samples:
        raise ValueError("snapshot JSONL is empty")
    stamps: list[float] = []
    for index, sample in enumerate(samples):
        raw = sample.get("stamp_s", sample.get("timestamp_s"))
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"snapshot {index} has no numeric stamp_s")
        stamp_s = float(raw)
        if not math.isfinite(stamp_s):
            raise ValueError(f"snapshot {index} has non-finite stamp_s")
        stamps.append(stamp_s)
    return EvidenceWindow(min(stamps), max(stamps))


def _relocalization_window(raw: dict[str, Any]) -> RelocalizationRecoveryWindow:
    return RelocalizationRecoveryWindow(
        start_s=float(raw["start_s"]),
        end_s=float(raw["end_s"]),
        min_stable_s=float(raw["min_stable_s"]),
        initial_offset_m=float(raw.get("initial_offset_m", 0.0)),
        initial_yaw_offset_rad=float(raw.get("initial_yaw_offset_rad", 0.0)),
        expected_map_sha256=str(raw.get("expected_map_sha256", "")),
        observed_map_sha256=str(raw.get("observed_map_sha256", "")),
    )


def _load_relocalization_case(path: Path | None) -> list[RelocalizationRecoveryWindow]:
    if path is None:
        return []
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if isinstance(payload, list):
        raw_windows = payload
    elif isinstance(payload, dict):
        raw_windows = payload.get("relocalization_windows", [])
    else:
        raise ValueError("relocalization case must be a JSON object or array")
    if not isinstance(raw_windows, list):
        raise ValueError("relocalization_windows must be a JSON array")
    windows: list[RelocalizationRecoveryWindow] = []
    for index, raw in enumerate(raw_windows):
        if not isinstance(raw, dict):
            raise ValueError(f"relocalization_windows[{index}] must be an object")
        try:
            windows.append(_relocalization_window(raw))
        except KeyError as exc:
            raise ValueError(f"relocalization_windows[{index}] missing {exc.args[0]}") from exc
    return windows


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate localization-only patrol readiness from C++ SLAM status "
            "snapshot JSONL. This never authorizes robot motion."
        )
    )
    parser.add_argument(
        "snapshots",
        type=Path,
        help="SLAM status JSONL input, or capture destination with --capture-source",
    )
    parser.add_argument(
        "--capture-source",
        type=Path,
        help="capture the atomic native SLAM status JSON before evaluation",
    )
    parser.add_argument("--capture-duration-s", type=float, default=60.0)
    parser.add_argument("--capture-poll-hz", type=float, default=20.0)
    parser.add_argument("--capture-min-samples", type=int, default=20)
    parser.add_argument("--capture-min-observed-duration-s", type=float)
    parser.add_argument("--capture-overwrite", action="store_true")
    parser.add_argument(
        "--capture-summary",
        type=Path,
        help="write the capture-only summary in addition to the readiness report",
    )
    evidence = parser.add_mutually_exclusive_group(required=True)
    evidence.add_argument(
        "--evidence",
        nargs=2,
        type=float,
        metavar=("START_S", "END_S"),
        help="evidence time window",
    )
    evidence.add_argument(
        "--full-evidence-window",
        action="store_true",
        help="explicitly use the full timestamp range in snapshots as evidence",
    )
    stationary = parser.add_mutually_exclusive_group()
    stationary.add_argument(
        "--stationary",
        nargs=2,
        type=float,
        metavar=("START_S", "END_S"),
        help="known stationary window used to estimate yaw drift",
    )
    stationary.add_argument(
        "--stationary-full-window",
        action="store_true",
        help="assert the complete captured file is a known stationary interval",
    )
    parser.add_argument(
        "--degeneracy-window",
        nargs=2,
        type=float,
        action="append",
        default=[],
        metavar=("START_S", "END_S"),
        help="repeatable annotated degeneracy interval",
    )
    parser.add_argument(
        "--relocalization-case",
        type=Path,
        help=(
            "JSON file with relocalization_windows containing start_s, end_s, "
            "min_stable_s, initial offsets, and expected/observed map hashes"
        ),
    )
    parser.add_argument("--min-status-samples", type=int, default=20)
    parser.add_argument("--min-stationary-duration", type=float, default=60.0)
    parser.add_argument("--max-sequence-gap", type=int, default=1)
    parser.add_argument("--min-degeneracy-coverage", type=float, default=0.95)
    parser.add_argument(
        "--min-degeneracy-detection-rate",
        type=float,
        default=0.80,
    )
    parser.add_argument(
        "--max-yaw-drift-deg-per-min",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--write",
        type=Path,
        help="write report atomically instead of printing JSON to stdout",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    capture_summary: dict[str, Any] | None = None
    if args.capture_source is not None:
        try:
            capture_summary = capture_snapshot_history(
                args.capture_source,
                args.snapshots,
                duration_s=args.capture_duration_s,
                poll_hz=args.capture_poll_hz,
                min_samples=args.capture_min_samples,
                min_observed_duration_s=args.capture_min_observed_duration_s,
                overwrite=args.capture_overwrite,
            )
        except (OSError, ValueError) as exc:
            print(f"capture failed: {exc}", file=sys.stderr)
            return 1
        if args.capture_summary is not None:
            _atomic_write_json(args.capture_summary, capture_summary)
    elif args.capture_summary is not None:
        print("--capture-summary requires --capture-source", file=sys.stderr)
        return 1

    samples = _read_jsonl(args.snapshots)
    full_window = _full_window(samples)
    evidence_window = full_window if args.full_evidence_window else _window(args.evidence, "evidence_window")
    stationary_window = (
        full_window
        if args.stationary_full_window
        else (_window(args.stationary, "stationary_window") if args.stationary is not None else None)
    )
    config = InspectionLocalizationReadinessConfig(
        min_status_samples=args.min_status_samples,
        min_stationary_duration_s=args.min_stationary_duration,
        max_sequence_gap=args.max_sequence_gap,
        min_degeneracy_coverage=args.min_degeneracy_coverage,
        min_annotated_degeneracy_detection_rate=args.min_degeneracy_detection_rate,
        max_stationary_yaw_drift_deg_per_min=args.max_yaw_drift_deg_per_min,
    )
    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=evidence_window,
        stationary_window=stationary_window,
        annotated_degeneracy_windows=[
            _window(values, "annotated_degeneracy_window") for values in args.degeneracy_window
        ],
        relocalization_windows=_load_relocalization_case(args.relocalization_case),
        config=config,
    )
    if capture_summary is not None:
        capture_complete = capture_summary["status"] == "COMPLETE"
        report["capture"] = capture_summary
        report["checks"]["capture"] = {
            "status": PASS if capture_complete else INCOMPLETE,
            "reasons": list(capture_summary.get("blockers", ())),
            "metrics": capture_summary,
        }
        if not capture_complete:
            report["blockers"].extend(f"capture: {blocker}" for blocker in capture_summary.get("blockers", ()))
            if report["localization_status"] == PASS:
                report["localization_status"] = INCOMPLETE
                report["status"] = "LOCALIZATION_INCOMPLETE"
    if args.write is not None:
        _atomic_write_json(args.write, report)
    else:
        print(json.dumps(report, indent=2, sort_keys=True))
    return _STATUS_TO_RC[report["localization_status"]]


if __name__ == "__main__":
    raise SystemExit(main())
