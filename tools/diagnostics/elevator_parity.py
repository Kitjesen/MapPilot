#!/usr/bin/env python3
"""Compare native and upstream Elevator-LIO poses without trajectory alignment.

Inputs must already use the same body calibration and initial yaw/translation
anchor. This measures implementation agreement, not error against ground truth.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from statistics import fmean


@dataclass(frozen=True)
class Pose:
    stamp: Decimal
    position: tuple[float, ...]
    quaternion: tuple[float, ...]


def read_tum(path: Path) -> list[Pose]:
    poses = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{line_number}: expected timestamp, XYZ and XYZW")
        stamp = Decimal(fields[0])
        values = tuple(float(value) for value in fields[1:])
        if not stamp.is_finite() or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}:{line_number}: non-finite pose")
        if poses and stamp < poses[-1].stamp:
            raise ValueError(f"{path}:{line_number}: timestamps are not ordered")
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm == 0:
            raise ValueError(f"{path}:{line_number}: zero quaternion")
        poses.append(Pose(stamp, values[:3], tuple(value / norm for value in values[3:])))
    if not poses:
        raise ValueError(f"{path}: no poses")
    return poses


def associate(native: list[Pose], official: list[Pose], tolerance: Decimal):
    """Associate ordered poses once each, with no interpolation or timestamp shift."""
    pairs, native_only, official_only = [], [], []
    i = j = 0
    while i < len(native) and j < len(official):
        delta = native[i].stamp - official[j].stamp
        if abs(delta) <= tolerance:
            pairs.append((native[i], official[j]))
            i += 1
            j += 1
        elif delta < 0:
            native_only.append(str(native[i].stamp))
            i += 1
        else:
            official_only.append(str(official[j].stamp))
            j += 1
    native_only.extend(str(pose.stamp) for pose in native[i:])
    official_only.extend(str(pose.stamp) for pose in official[j:])
    return pairs, native_only, official_only


def orientation_difference_deg(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    # atan2 retains small-angle precision; q and -q describe the same rotation.
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    x = aw * bx - ax * bw - ay * bz + az * by
    y = aw * by + ax * bz - ay * bw - az * bx
    z = aw * bz - ax * by + ay * bx - az * bw
    w = aw * bw + ax * bx + ay * by + az * bz
    return math.degrees(2 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w)))


def statistics(values: list[float]) -> dict | None:
    if not values:
        return None
    ordered = sorted(values)
    return {
        "mean": fmean(values),
        "rms": math.sqrt(fmean(value * value for value in values)),
        "p95": ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)],
        "max": ordered[-1],
    }


def compare(
    native_path: Path, official_path: Path, tolerance: Decimal, position_tolerance: float, orientation_tolerance: float
) -> dict:
    native, official = read_tum(native_path), read_tum(official_path)
    pairs, native_only, official_only = associate(native, official, tolerance)
    errors = []
    for ours, reference in pairs:
        delta_xyz = [a - b for a, b in zip(ours.position, reference.position)]
        errors.append(
            {
                "native_stamp_s": str(ours.stamp),
                "official_stamp_s": str(reference.stamp),
                "stamp_delta_s": float(ours.stamp - reference.stamp),
                "position_delta_xyz_m": delta_xyz,
                "position_difference_m": math.sqrt(sum(value * value for value in delta_xyz)),
                "orientation_difference_deg": orientation_difference_deg(ours.quaternion, reference.quaternion),
            }
        )
    different = [
        error
        for error in errors
        if error["position_difference_m"] > position_tolerance
        or error["orientation_difference_deg"] > orientation_tolerance
    ]
    complete = bool(pairs) and not native_only and not official_only
    return {
        "native": str(native_path.resolve()),
        "official": str(official_path.resolve()),
        "pose_count": {"native": len(native), "official": len(official)},
        "duplicate_timestamps": {
            "native": len(native) - len({pose.stamp for pose in native}),
            "official": len(official) - len({pose.stamp for pose in official}),
        },
        "matched_poses": len(pairs),
        "exact_timestamp_matches": sum(a.stamp == b.stamp for a, b in pairs),
        "unmatched_native_timestamps_s": native_only,
        "unmatched_official_timestamps_s": official_only,
        "absolute_timestamp_difference_s": statistics([abs(error["stamp_delta_s"]) for error in errors]),
        "position_difference_m": statistics([error["position_difference_m"] for error in errors]),
        "orientation_difference_deg": statistics([error["orientation_difference_deg"] for error in errors]),
        "poses_outside_tolerance": len(different),
        "complete_sequence_match": complete,
        "parity_passed": complete and not different,
        "first_match": errors[0] if errors else None,
        "last_match": errors[-1] if errors else None,
        "first_difference": different[0] if different else None,
        "largest_position_differences": sorted(errors, key=lambda error: error["position_difference_m"], reverse=True)[
            :5
        ],
        "largest_orientation_differences": sorted(
            errors, key=lambda error: error["orientation_difference_deg"], reverse=True
        )[:5],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pair", nargs=3, action="append", required=True, metavar=("NAME", "NATIVE_TUM", "OFFICIAL_BODY_ANCHORED_TUM")
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timestamp-tolerance-s", type=Decimal, default=Decimal("0.00001"))
    parser.add_argument("--position-tolerance-m", type=float, default=1e-6)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=1e-4)
    parser.add_argument(
        "--require-parity",
        action="store_true",
        help="Exit with 1 when any full pose sequence differs beyond the stated tolerances.",
    )
    args = parser.parse_args()
    if (
        not args.timestamp_tolerance_s.is_finite()
        or args.timestamp_tolerance_s < 0
        or not math.isfinite(args.position_tolerance_m)
        or args.position_tolerance_m < 0
        or not math.isfinite(args.orientation_tolerance_deg)
        or args.orientation_tolerance_deg < 0
    ):
        parser.error("comparison tolerances must be finite and non-negative")
    names = [pair[0] for pair in args.pair]
    if len(set(names)) != len(names):
        parser.error("each pair needs a distinct name")
    results = {
        name: compare(
            Path(native),
            Path(official),
            args.timestamp_tolerance_s,
            args.position_tolerance_m,
            args.orientation_tolerance_deg,
        )
        for name, native, official in args.pair
    }
    report = {
        "comparison": "Implementation agreement, not ground-truth trajectory accuracy.",
        "coordinates": "Inputs already use identical body calibration and initial yaw/translation anchoring.",
        "alignment": "None: no fitted transform, scale, interpolation, smoothing or time shift.",
        "timestamp_association": "Ordered one-to-one association within the explicit tolerance; missing samples are retained.",
        "timestamp_tolerance_s": str(args.timestamp_tolerance_s),
        "position_tolerance_m": args.position_tolerance_m,
        "orientation_tolerance_deg": args.orientation_tolerance_deg,
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for name, result in results.items():
        counts = result["pose_count"]
        position, orientation = result["position_difference_m"], result["orientation_difference_deg"]
        print(
            f"{name}: native={counts['native']} official={counts['official']} "
            f"matched={result['matched_poses']} exact_time={result['exact_timestamp_matches']} "
            f"missing_native={len(result['unmatched_official_timestamps_s'])} "
            f"missing_official={len(result['unmatched_native_timestamps_s'])}"
        )
        if position is not None:
            print(
                f"  position rms/max={position['rms']:.9g}/{position['max']:.9g} m; "
                f"orientation rms/max={orientation['rms']:.9g}/{orientation['max']:.9g} deg; "
                f"parity={'PASS' if result['parity_passed'] else 'DIFF'}"
            )
    print(f"Report: {args.output.resolve()}")
    return int(args.require_parity and not all(result["parity_passed"] for result in results.values()))


if __name__ == "__main__":
    raise SystemExit(main())
