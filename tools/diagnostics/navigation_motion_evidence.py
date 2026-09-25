"""Read-only capture of native navigation decisions, driver ACKs and odometry.

This reports observed facts, not a collision-free navigation certification.
No goals, leases or movement commands are sent.
"""

from __future__ import annotations

import argparse
import json
import math
import time
import urllib.request
from pathlib import Path


def summarize(samples: list[dict]) -> dict:
    result = {
        "unique_navigation_samples": 0,
        "unavailable_navigation_samples": 0,
        "prediction_samples": 0,
        "decisions": {},
        "driver_matched_samples": 0,
        "moving_command_samples": 0,
        "acknowledged_command_with_observed_motion_samples": 0,
        "wait_with_nonzero_command_samples": 0,
        "wait_with_fresh_quiet_odometry_samples": 0,
        "capture_errors": 0,
    }
    last_nav_stamp = None
    last_odom_stamp = None
    waiting_states = {"waiting", "stale", "waiting_for_detour", "timeout"}
    for sample in samples:
        if "error" in sample:
            result["capture_errors"] += 1
            continue
        nav = sample["dds"]["nav_endpoint"]
        if not isinstance(nav, dict) or not isinstance(nav.get("stamp_s"), (float, int)):
            result["unavailable_navigation_samples"] += 1
            continue
        if nav.get("stamp_s") == last_nav_stamp:
            continue
        last_nav_stamp = nav.get("stamp_s")
        result["unique_navigation_samples"] += 1
        local = nav.get("last_local", {})
        decision = local.get("dynamic_avoidance", "unavailable")
        result["decisions"][decision] = result["decisions"].get(decision, 0) + 1
        result["prediction_samples"] += int(local.get("prediction_count", 0) > 0)
        command = nav.get("final_cmd_vel", {})
        moving = any(abs(command.get(axis, 0)) > 1e-6 for axis in ("vx", "vy", "wz"))
        result["moving_command_samples"] += int(moving)
        output, driver = nav.get("final_output", {}), nav.get("driver_control", {})
        matched = bool(
            output.get("published") and output.get("output_sequence", 0) > 0
            and driver.get("fresh") and driver.get("last_command_accepted")
            and output.get("producer_boot_id")
            and driver.get("accepted_producer_boot_id") == output.get("producer_boot_id")
            and driver.get("accepted_output_sequence") == output.get("output_sequence")
        )
        result["driver_matched_samples"] += int(matched)
        odom = sample["state"].get("localization", {}).get("odometry", {})
        gate = nav.get("input_gate", {})
        age, limit = gate.get("odom_age_s", -1), gate.get("odom_max_age_s", 0)
        stamp, speed, yaw_rate = odom.get("ts"), odom.get("vx"), odom.get("wz")
        fresh = (
            all(isinstance(v, (float, int)) and math.isfinite(v)
                for v in (stamp, speed, yaw_rate, age, limit))
            and stamp > 0 and (last_odom_stamp is None or stamp > last_odom_stamp)
            and limit > 0 and 0 <= age <= limit
        )
        if fresh:
            last_odom_stamp = stamp
            quiet = abs(speed) <= .03 and abs(yaw_rate) <= .08
            result["acknowledged_command_with_observed_motion_samples"] += int(
                matched and moving and not quiet)
            result["wait_with_fresh_quiet_odometry_samples"] += int(
                decision in waiting_states and not moving and quiet)
        result["wait_with_nonzero_command_samples"] += int(decision in waiting_states and moving)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:15052")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--platform", required=True, choices=("go2-nx", "s100p", "native-sim"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    args.output.mkdir(parents=True, exist_ok=True)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    rows = []
    started = time.monotonic()
    with (args.output / "samples.jsonl").open("x", encoding="utf-8") as log:
        while time.monotonic() - started < args.seconds:
            tick = time.monotonic()
            row = {"elapsed_s": tick - started, "sampled_at": time.time()}
            try:
                for key, endpoint in (("dds", "navigation/dds_snapshot"), ("state", "state")):
                    with opener.open(f"{args.url.rstrip('/')}/api/v1/{endpoint}", timeout=3) as response:
                        row[key] = json.load(response)
            except Exception as error:
                row["error"] = str(error)
            rows.append(row)
            log.write(json.dumps(row, ensure_ascii=False) + "\n")
            log.flush()
            time.sleep(max(0, .1 - (time.monotonic() - tick)))
    report = {"platform": args.platform, "url": args.url, "facts": summarize(rows),
              "scope": "Sampled status evidence; requires scene/video and distance review. "
                       "Driver ACK is not proof of physical movement or stopping distance."}
    (args.output / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
