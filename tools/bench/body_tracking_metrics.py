"""Measure saved body-route execution against its actual finite global path."""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np


def path_distances(xy: np.ndarray, route: np.ndarray) -> np.ndarray:
    """Include endpoint overshoot; an infinite supporting line is not a route."""
    edges = np.diff(route, axis=0)
    squared_lengths = np.sum(edges * edges, axis=1)
    valid = squared_lengths > 1e-12
    if not np.any(valid):
        return np.linalg.norm(xy - route[0], axis=1)
    starts, vectors, lengths = route[:-1][valid], edges[valid], squared_lengths[valid]
    fraction = np.clip(np.sum((xy[:, None, :] - starts) * vectors, axis=2) / lengths, 0, 1)
    closest = starts + fraction[:, :, None] * vectors
    return np.sqrt(np.min(np.sum((xy[:, None, :] - closest) ** 2, axis=2), axis=1))


def summarize(run: Path) -> dict:
    states = [json.loads(line) for line in (run / "states.jsonl").read_text().splitlines()]
    summary = json.loads((run / "summary.json").read_text())
    invocation = json.loads((run / "invocation.json").read_text())
    with (run / "native/global.csv").open(newline="") as source:
        route = np.array([[float(row[key]) for key in ("x", "y", "z")]
                          for row in csv.DictReader(source)])
    with (run / "native/states.csv").open(newline="") as source:
        native_states = list(csv.DictReader(source))
    xyz = np.array([row["position"] for row in states])
    # Include the measured initialization and all recorded terminal-hold samples.
    movement = np.diff(np.vstack((route[0], xyz)), axis=0)
    distance = path_distances(xyz[:, :2], route[:, :2])
    tracking = np.array([row["command"]["status"] == "spline_control_ready" for row in states])
    phases = Counter(row["command"]["status"] for row in states)
    dt = invocation["dt_s"]
    result = {
        "run": str(run),
        "result": summary["result"],
        "global_xy_length_m": float(np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1).sum()),
        "measured_xy_displacement_m": float(np.linalg.norm(xyz[-1, :2] - route[0, :2])),
        "measured_xy_trace_length_m": float(np.linalg.norm(movement[:, :2], axis=1).sum()),
        "measured_3d_trace_length_m": float(np.linalg.norm(movement, axis=1).sum()),
        "nearest_global_xy_distance_rmse_m": float(np.sqrt(np.mean(distance ** 2))),
        "nearest_global_xy_distance_max_m": float(distance.max()),
        "tracking_phase_global_xy_distance_rmse_m": (
            float(np.sqrt(np.mean(distance[tracking] ** 2))) if np.any(tracking) else None),
        "phase_duration_s": {key: round(count * dt, 6) for key, count in phases.items()},
        "goal_xy_error_m": summary["goal_xy_error_m"],
        "goal_yaw_error_rad": summary["goal_yaw_error_rad"],
        "execution_duration_s": summary["execution_duration_s"],
        "sample_interval_s": dt,
        "native_travel_definition": "Sum of 3D position changes before terminal feedback, including base heave.",
        "distance_definition": "Nearest distance to the finite global XY polyline, including endpoint overshoot; all-phase RMSE includes waits and final orientation. This does not measure time-parametrized SCAN spline tracking.",
        "follower_error_definition": "Native tracking_error during spline_control_ready: XY distance from measured pose to the spline reference evaluated at the controller's current execution time.",
    }
    active_errors = [float(row["tracking_error"]) for row in native_states
                     if row["reason"] == "spline_control_ready"]
    result["reported_follower_position_error_rmse_m"] = (
        float(np.sqrt(np.mean(np.square(active_errors)))) if active_errors else None)
    result["reported_follower_position_error_max_m"] = max(active_errors, default=None)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = {run.name: summarize(run) for run in args.runs}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
