#!/usr/bin/env python3
"""Non-motion localization probe for robot-side SLAM backend checks."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
import time
from typing import Any


def _curl_json(gateway: str, path: str) -> dict[str, Any]:
    url = gateway.rstrip("/") + path
    try:
        raw = subprocess.check_output(
            ["curl", "-s", "--max-time", "4", url],
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except Exception:
        return {}
    try:
        data = json.loads(raw)
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _process_rows() -> list[dict[str, Any]]:
    try:
        raw = subprocess.check_output(
            [
                "ps",
                "-eo",
                "comm,pcpu,pmem,rss,args",
            ],
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except Exception:
        return []

    rows: list[dict[str, Any]] = []
    needles = (
        "lingtu.py",
        "fastlio",
        "localizer",
        "super_lio",
        "super-lio",
        "lio_node",
        "relocation_node",
    )
    for line in raw.splitlines()[1:]:
        if not any(needle in line for needle in needles):
            continue
        parts = line.split(None, 4)
        if len(parts) < 5:
            continue
        comm, pcpu, pmem, rss, cmdline = parts
        label = comm
        normalized = cmdline.lower().replace("-", "_")
        if "lingtu.py" in cmdline:
            label = "lingtu"
        elif "super_lio_relocation" in normalized or "relocation_node" in normalized:
            label = "super_lio_relocation"
        elif "super_lio" in normalized:
            label = "super_lio"
        elif "localizer_node" in cmdline:
            label = "localizer"
        elif "lio_node" in cmdline or "fastlio" in normalized:
            label = "fastlio2"
        try:
            rows.append(
                {
                    "label": label,
                    "pcpu": float(pcpu),
                    "pmem": float(pmem),
                    "rss_kb": int(rss),
                }
            )
        except ValueError:
            continue
    return rows


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def _mean(values: list[float]) -> float | None:
    return round(statistics.fmean(values), 4) if values else None


def _range(values: list[float]) -> list[float] | None:
    return [round(min(values), 4), round(max(values), 4)] if values else None


def _source_name(control: dict[str, Any]) -> str:
    source = control.get("active_source")
    if isinstance(source, dict):
        return str(source.get("name") or source.get("source") or source.get("owner") or "none")
    return str(control.get("active_cmd_source") or "none")


def _angle_delta(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def collect(gateway: str, phase: str, duration: int, interval: int) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    processes: list[list[dict[str, Any]]] = []
    sample_count = int(duration / interval) + 1 if interval > 0 else 1
    sample_count = max(1, sample_count)

    for index in range(sample_count):
        now = time.time()
        state = _curl_json(gateway, "/api/v1/state")
        loc = state.get("localization") if isinstance(state.get("localization"), dict) else {}
        sess = state.get("session") if isinstance(state.get("session"), dict) else {}
        odom = state.get("odometry") if isinstance(state.get("odometry"), dict) else {}
        nav = state.get("navigation") if isinstance(state.get("navigation"), dict) else {}
        control = nav.get("control") if isinstance(nav.get("control"), dict) else {}
        map_info = state.get("map") if isinstance(state.get("map"), dict) else {}
        samples.append(
            {
                "t": now,
                "x": _number(odom.get("x")),
                "y": _number(odom.get("y")),
                "z": _number(odom.get("z")),
                "yaw": _number(odom.get("yaw")),
                "backend": loc.get("backend") or sess.get("localization_backend"),
                "health_source": loc.get("health_source") or sess.get("health_source"),
                "state": loc.get("state"),
                "ready": bool(loc.get("ready")),
                "confidence": _number(loc.get("confidence")),
                "icp_quality": _number(sess.get("icp_quality")) or _number(loc.get("icp_quality")),
                "odom_age_ms": _number(loc.get("odom_age_ms")),
                "cloud_age_ms": _number(loc.get("cloud_age_ms")),
                "live_points": _number(map_info.get("live_points")),
                "live_frames": _number(map_info.get("live_cloud_frames")),
                "map_save_source": loc.get("map_save_source") or sess.get("map_save_source"),
                "recovery_signal": loc.get("recovery_signal") or sess.get("recovery_signal"),
                "recovery_action": loc.get("recovery_action") or sess.get("recovery_action"),
                "active_cmd_source": _source_name(control),
            }
        )
        processes.append(_process_rows())
        if index != sample_count - 1 and interval > 0:
            time.sleep(interval)

    first = samples[0]
    last = samples[-1]
    xy_drift = None
    if first.get("x") is not None and first.get("y") is not None and last.get("x") is not None and last.get("y") is not None:
        xy_drift = math.hypot(float(last["x"]) - float(first["x"]), float(last["y"]) - float(first["y"]))
    yaw_drift = None
    if first.get("yaw") is not None and last.get("yaw") is not None:
        yaw_drift = _angle_delta(float(last["yaw"]), float(first["yaw"]))

    frames = [float(s["live_frames"]) for s in samples if s.get("live_frames") is not None]
    fps = None
    elapsed = samples[-1]["t"] - samples[0]["t"]
    if len(frames) >= 2 and elapsed > 0:
        fps = (frames[-1] - frames[0]) / elapsed

    process_by_label: dict[str, dict[str, list[float]]] = {}
    for snapshot in processes:
        for row in snapshot:
            bucket = process_by_label.setdefault(
                str(row["label"]),
                {"pcpu": [], "pmem": [], "rss_kb": []},
            )
            bucket["pcpu"].append(float(row["pcpu"]))
            bucket["pmem"].append(float(row["pmem"]))
            bucket["rss_kb"].append(float(row["rss_kb"]))

    process_summary = {
        label: {
            "avg_pcpu": round(statistics.fmean(values["pcpu"]), 2),
            "max_pcpu": round(max(values["pcpu"]), 2),
            "avg_rss_mb": round(statistics.fmean(values["rss_kb"]) / 1024, 1),
            "max_rss_mb": round(max(values["rss_kb"]) / 1024, 1),
            "avg_pmem": round(statistics.fmean(values["pmem"]), 2),
        }
        for label, values in process_by_label.items()
        if values["pcpu"]
    }

    def nums(key: str) -> list[float]:
        return [float(s[key]) for s in samples if s.get(key) is not None]

    return {
        "schema_version": 1,
        "mode": phase,
        "duration_s": round(elapsed, 1),
        "samples": len(samples),
        "backend_set": sorted({str(s["backend"]) for s in samples if s.get("backend")}),
        "health_source_set": sorted({str(s["health_source"]) for s in samples if s.get("health_source")}),
        "ready_all": all(bool(s.get("ready")) for s in samples),
        "cmd_source_set": sorted({str(s.get("active_cmd_source") or "none") for s in samples}),
        "map_save_source_set": sorted({str(s["map_save_source"]) for s in samples if s.get("map_save_source")}),
        "recovery_signal_set": sorted({str(s["recovery_signal"]) for s in samples if s.get("recovery_signal")}),
        "recovery_action_set": sorted({str(s["recovery_action"]) for s in samples if s.get("recovery_action")}),
        "xy_drift_m": round(xy_drift, 4) if xy_drift is not None else None,
        "yaw_drift_rad": round(yaw_drift, 5) if yaw_drift is not None else None,
        "confidence_range": _range(nums("confidence")),
        "confidence_avg": _mean(nums("confidence")),
        "icp_quality_range": _range(nums("icp_quality")),
        "icp_quality_avg": _mean(nums("icp_quality")),
        "odom_age_ms_avg": _mean(nums("odom_age_ms")),
        "cloud_age_ms_avg": _mean(nums("cloud_age_ms")),
        "live_points_avg": _mean(nums("live_points")),
        "map_cloud_fps_est": round(fps, 2) if fps is not None else None,
        "processes": process_summary,
        "first_pose": {key: first.get(key) for key in ("x", "y", "z", "yaw")},
        "last_pose": {key: last.get(key) for key in ("x", "y", "z", "yaw")},
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway", default="http://localhost:5050")
    parser.add_argument("--phase", required=True)
    parser.add_argument("--duration", type=int, default=60)
    parser.add_argument("--interval", type=int, default=5)
    args = parser.parse_args()

    if args.duration < 0:
        parser.error("--duration must be >= 0")
    if args.interval <= 0:
        parser.error("--interval must be > 0")
    json.dump(
        collect(args.gateway, args.phase, args.duration, args.interval),
        sys.stdout,
        ensure_ascii=False,
        indent=2,
    )
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
