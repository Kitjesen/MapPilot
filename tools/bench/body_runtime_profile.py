"""Summarize measured wall timers without double-counting retained debug data."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def summarize(run: Path) -> dict:
    summary = json.loads((run / "summary.json").read_text())
    native = json.loads((run / "native/summary.json").read_text())
    with (run / "native/states.csv").open(newline="") as source:
        rows = list(csv.DictReader(source))
    phases = {}
    for name in ("tick_ms", "executor_ms", "final_grid_ms", "final_body_ms",
                 "final_measured_braking_ms", "final_command_braking_ms", "final_support_ms"):
        if name not in rows[0]:
            continue
        values = np.array([float(row[name]) for row in rows])
        phases[name] = {"sum_s": float(values.sum() / 1000),
                        "mean_ms": float(values.mean()), "p50_ms": float(np.median(values)),
                        "p95_ms": float(np.percentile(values, 95)), "max_ms": float(values.max())}
    snapshot_s = summary["samples"] * summary["snapshot_cost_ms"]["mean"] / 1000
    result = {
        "run": str(run), "result": summary["result"],
        "wall_time_s": summary["wall_time_s"], "simulation_execution_s": summary["execution_duration_s"],
        "native_global_s": native["global_ms"] / 1000,
        "native_tick_count": len(rows),
        "native_tick_over_20ms": sum(float(row["tick_ms"]) > 20 for row in rows),
        "native_phases": phases, "snapshot_total_s": snapshot_s,
        "accounting_note": "Native phase timers are nested inside tick_ms and Python protocol waits/writes. Never add these levels together. plan_ms is retained debug data and is not summed. These are wall timers, not thread CPU time.",
    }
    if "wall_timing_s" in summary:
        result["python_wall_partition_s"] = summary["wall_timing_s"]
        result["partition_error_s"] = sum(summary["wall_timing_s"].values()) - summary["wall_time_s"]
        result["python_process_cpu_s"] = summary["python_process_cpu_s"]
    else:
        known = native["global_ms"] / 1000 + phases["tick_ms"]["sum_s"] + snapshot_s
        result["unpartitioned_wall_s"] = summary["wall_time_s"] - known
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = {run.name: summarize(run) for run in args.runs}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(args.out)


if __name__ == "__main__":
    main()
