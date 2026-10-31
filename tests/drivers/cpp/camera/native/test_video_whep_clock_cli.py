#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from test_video_gst_cli import (
    Fixture, events, free_port, has_whepserversink, matching_status,
    publish_until_running, wait_for,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--artifacts", required=True)
    args = parser.parse_args()
    if not has_whepserversink():
        print("SKIP: requires whepserversink")
        return 77

    root = Path(args.artifacts)
    root.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="whep-clock-", dir=root))
    status_path = directory / "status.json"
    metrics_path = directory / "metrics.jsonl"
    name = f"/lingtu_whep_clock_{os.getpid()}_{time.time_ns()}"
    fixture = Fixture(args.fixture, name)
    port = free_port()
    endpoint = f"http://127.0.0.1:{port}/whep/endpoint"
    with (directory / "stderr.log").open("wb") as stderr:
        process = subprocess.Popen(
            [args.video, "--color-shm", name, "--listen", f"http://127.0.0.1:{port}",
             "--encoder", "x264enc", "--fps", "30", "--max-age-ms", "1000",
             "--idle-timeout-ms", "0", "--metrics-interval-ms", "20",
             "--metrics", str(metrics_path), "--status-file", str(status_path)],
            stdout=subprocess.DEVNULL, stderr=stderr,
        )
        try:
            publish_until_running(fixture, status_path, 811, endpoint=endpoint)
            stream_count = sum(event.get("event") == "stream" for event in events(metrics_path))
            # A higher SHM sequence has a lower Unix timestamp, without changing host time.
            regressed_sequence = fixture.publish(811, age_ms=500)
            wait_for(
                lambda: sum(event.get("event") == "stream" for event in events(metrics_path))
                == stream_count + 1,
                timeout=0.4,
            )
            restart_index = max(
                index for index, event in enumerate(events(metrics_path))
                if event.get("event") == "stream"
            )
            recovered = wait_for(
                lambda: next(
                    (event for event in events(metrics_path)[restart_index + 1:]
                     if event.get("event") == "metrics" and event.get("accepted_frames", 0) >= 1),
                    None,
                ),
                timeout=0.4,
            )
            assert recovered["rejected_timestamps"] == 0, recovered
            stale = wait_for(
                lambda: matching_status(
                    status_path,
                    lambda current: current.get("status") == "waiting_for_source"
                    and current.get("source_sequence") == regressed_sequence,
                )
            )
            assert stale["source_generation"] == 811, stale
            assert stale["ready"] is False and stale["source_fresh"] is False, stale
            assert stale["frame_age_ms"] > 1000, stale
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=8) == 0
        finally:
            fixture.close()
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)
    print(f"whep_clock same_generation_timestamp_regression=ok source_expiry=ok artifacts={directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
