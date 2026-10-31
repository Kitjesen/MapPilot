#!/usr/bin/env python3
import argparse
import json
import os
import signal
import shutil
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

FRAME_BYTES = 16 * 16 * 3


class Fixture:
    def __init__(self, executable, name):
        self.process = subprocess.Popen(
            [executable, name],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        if self.process.stdout.readline().strip() != f"ready {name}":
            raise AssertionError("fixture did not become ready")

    def publish(self, generation, age_ms=0, encoding="rgb8"):
        self.process.stdin.write(f"publish {generation} 1 {age_ms} {encoding}\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline().strip()
        if not response.startswith(f"published {generation} "):
            raise AssertionError(f"fixture publish failed: {response}")
        return int(response.split()[3])

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write("quit\n")
            self.process.stdin.flush()
            if self.process.stdout.readline().strip() != "bye":
                raise AssertionError("fixture shutdown failed")
            self.process.wait(timeout=3)


def events(path):
    if not path.exists():
        return []
    result = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return result


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("timed out waiting for GStreamer video evidence")


def status(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def matching_status(path, predicate):
    current = status(path)
    return current if current and predicate(current) else None


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def has_whepserversink():
    inspect = shutil.which("gst-inspect-1.0")
    if inspect is None:
        return False
    return subprocess.run(
        [inspect, "whepserversink"], capture_output=True, check=False
    ).returncode == 0


def endpoint_status(url):
    try:
        request = urllib.request.Request(url, method="OPTIONS")
        with urllib.request.urlopen(request, timeout=0.2) as response:
            return response.status
    except urllib.error.HTTPError as error:
        return error.code
    except (urllib.error.URLError, TimeoutError):
        return None


def publish_until_running(fixture, status_path, generation, endpoint=None, timeout=8.0):
    deadline = time.monotonic() + timeout
    latest_sequence = 0
    while time.monotonic() < deadline:
        latest_sequence = fixture.publish(generation)
        time.sleep(0.05)
        current = status(status_path)
        if (
            current
            and current.get("status") == "running"
            and current.get("source_generation") == generation
            and (endpoint is None or endpoint_status(endpoint) is not None)
        ):
            return current.get("source_sequence", latest_sequence), current
    raise AssertionError(f"WHEP did not accept a fresh frame: {status(status_path)}")


def accepted(path, generation):
    values = [
        event.get("accepted_frames", 0)
        for event in events(path)
        if event.get("event") == "metrics"
        and event.get("source_generation") == generation
    ]
    return max(values, default=0)


def decode(ffmpeg, stream):
    # Preserve decoded pictures; FFmpeg 4.2 auto-vsync otherwise duplicates frames.
    result = subprocess.run(
        [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "h264",
            "-i",
            "pipe:0",
            "-vsync",
            "0",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "pipe:1",
        ],
        input=stream,
        capture_output=True,
        timeout=15,
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(result.stderr.decode("utf-8", errors="replace"))
    return result.stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--artifacts")
    args = parser.parse_args()

    cleanup = None
    if args.artifacts:
        root = Path(args.artifacts)
        root.mkdir(parents=True, exist_ok=True)
        directory = Path(tempfile.mkdtemp(prefix="video-gst-cli-", dir=root))
    else:
        cleanup = tempfile.TemporaryDirectory(prefix="lingtu-video-gst-cli-")
        directory = Path(cleanup.name)

    name = f"/lingtu_video_gst_{os.getpid()}_{time.time_ns()}"
    output = directory / "continuous.h264"
    metadata = directory / "continuous.jsonl"
    fixture = Fixture(args.fixture, name)
    command = [
        args.video,
        "--color-shm",
        name,
        "--output",
        str(output),
        "--encoder",
        "x264enc",
        "--fps",
        "30",
        "--max-age-ms",
        "2000",
        "--idle-timeout-ms",
        "2000",
        "--metrics-interval-ms",
        "20",
        "--metrics",
        str(metadata),
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        fixture.publish(401, age_ms=5000, encoding="rgb8")
        time.sleep(0.15)
        if any(event.get("event") == "stream" for event in events(metadata)):
            raise AssertionError("stale color frame started an encoder stream")

        for index in range(16):
            fixture.publish(401, encoding="rgb8")
            wait_for(lambda index=index: accepted(metadata, 401) >= index + 1)
            time.sleep(0.04)
        for index in range(16):
            fixture.publish(402, encoding="bgr8")
            wait_for(lambda index=index: accepted(metadata, 402) >= index + 1)
            time.sleep(0.04)

        process.send_signal(signal.SIGTERM)
        _, stderr = process.communicate(timeout=8)
        if process.returncode != 0:
            raise AssertionError(stderr.decode("utf-8", errors="replace"))
        recorded = events(metadata)
        streams = [event for event in recorded if event.get("event") == "stream"]
        if [event.get("source_generation") for event in streams] != [401, 402]:
            raise AssertionError(f"generation restart evidence is wrong: {streams}")
        measured = [event for event in recorded if event.get("event") == "metrics"]
        if not measured or any(event.get("selected_encoder") != "x264enc" for event in measured):
            raise AssertionError("runtime did not report the exact x264 encoder")
        if any(event.get("queued_raw_frames", 2) > 1 for event in measured):
            raise AssertionError("raw frame queue exceeded its one-frame bound")

        raw = decode(args.ffmpeg, output.read_bytes())
        if len(raw) != 32 * FRAME_BYTES:
            raise AssertionError(f"decoded {len(raw) // FRAME_BYTES} frames instead of 32")
        for frame_index in range(32):
            current = raw[frame_index * FRAME_BYTES : (frame_index + 1) * FRAME_BYTES]
            red = sum(current[0::3]) / (len(current) / 3)
            green = sum(current[1::3]) / (len(current) / 3)
            blue = sum(current[2::3]) / (len(current) / 3)
            if red < 180 or green > 70 or blue > 70:
                raise AssertionError(
                    f"decoded frame {frame_index} color is wrong: {red:.1f}/{green:.1f}/{blue:.1f}"
                )
    finally:
        fixture.close()
        if process.poll() is None:
            process.kill()

    missing_output = directory / "idle.h264"
    missing_metadata = directory / "idle.jsonl"
    missing_status = directory / "idle.status.json"
    missing = subprocess.run(
        [
            args.video,
            "--color-shm",
            f"/lingtu_missing_{os.getpid()}_{time.time_ns()}",
            "--output",
            str(missing_output),
            "--encoder",
            "x264enc",
            "--idle-timeout-ms",
            "100",
            "--metrics",
            str(missing_metadata),
            "--status-file",
            str(missing_status),
        ],
        capture_output=True,
        timeout=4,
        check=False,
    )
    if missing.returncode == 0 or b"timed out" not in missing.stderr:
        raise AssertionError("missing fresh camera input did not fail cleanly")
    failed = status(missing_status)
    if not failed or failed.get("status") != "error" or failed.get("ready") is not False:
        raise AssertionError(f"H.264 timeout status is not failed: {failed}")

    whep_result = "whep_unavailable"
    if has_whepserversink():
        whep_status = directory / "whep.status.json"
        whep_metadata = directory / "whep.jsonl"
        port = free_port()
        endpoint = f"http://127.0.0.1:{port}/whep/endpoint"
        whep = subprocess.Popen(
            [
                args.video,
                "--color-shm",
                name,
                "--listen",
                f"http://127.0.0.1:{port}",
                "--encoder",
                "x264enc",
                "--fps",
                "30",
                "--max-age-ms",
                "1000",
                "--idle-timeout-ms",
                "100",
                "--metrics-interval-ms",
                "20",
                "--metrics",
                str(whep_metadata),
                "--status-file",
                str(whep_status),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        fixture = Fixture(args.fixture, name)
        try:
            waiting = wait_for(
                lambda: matching_status(
                    whep_status, lambda current: current.get("status") == "waiting_for_source"
                )
            )
            if waiting.get("ready") is not False or waiting.get("source_fresh") is not False:
                raise AssertionError(f"initial WHEP status is incorrectly ready: {waiting}")

            _first_sequence, running = publish_until_running(
                fixture, whep_status, 501, endpoint=endpoint
            )
            if running.get("ready") is not True or running.get("source_fresh") is not True:
                raise AssertionError(f"fresh WHEP status is not ready: {running}")

            stale = wait_for(
                lambda: matching_status(
                    whep_status,
                    lambda current: current.get("status") == "waiting_for_source"
                    and current.get("source_generation") == 501,
                )
            )
            if stale.get("ready") is not False or stale.get("source_fresh") is not False:
                raise AssertionError(f"stale WHEP status is incorrectly ready: {stale}")
            if whep.poll() is not None:
                raise AssertionError("WHEP process exited after its source exceeded idle timeout")
            wait_for(lambda: endpoint_status(endpoint) is None)
            stale_sequence = stale.get("source_sequence")
            time.sleep(0.2)
            stale_again = status(whep_status)
            if not stale_again or stale_again.get("source_sequence") != stale_sequence:
                raise AssertionError(f"stale WHEP status advanced without a new frame: {stale_again}")

            _recovered_sequence, recovered = publish_until_running(
                fixture, whep_status, 502, endpoint=endpoint
            )
            if (
                recovered.get("source_generation") != 502
                or recovered.get("ready") is not True
            ):
                raise AssertionError(f"WHEP did not recover on a newer source frame: {recovered}")

            whep.send_signal(signal.SIGTERM)
            _, stderr = whep.communicate(timeout=8)
            if whep.returncode != 0:
                raise AssertionError(stderr.decode("utf-8", errors="replace"))
            stopped = status(whep_status)
            if not stopped or stopped.get("status") != "stopped" or stopped.get("ready") is not False:
                raise AssertionError(f"stopped WHEP status is incorrect: {stopped}")
            whep_result = "whep_gap_recovery=ok"
        finally:
            fixture.close()
            if whep.poll() is None:
                whep.kill()

        failed_name = f"/lingtu_video_gst_failed_{os.getpid()}_{time.time_ns()}"
        failed_status = directory / "encoder-failed.status.json"
        failed_fixture = Fixture(args.fixture, failed_name)
        failed_process = subprocess.Popen(
            [
                args.video,
                "--color-shm",
                failed_name,
                "--listen",
                f"http://127.0.0.1:{free_port()}",
                "--encoder",
                "lingtu_missing_h264_encoder",
                "--max-age-ms",
                "2000",
                "--status-file",
                str(failed_status),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            failed_fixture.publish(701)
            failed_process.wait(timeout=5)
            failed_snapshot = status(failed_status)
            if (
                failed_process.returncode == 0
                or not failed_snapshot
                or failed_snapshot.get("status") != "error"
                or failed_snapshot.get("ready") is not False
                or "unavailable" not in failed_snapshot.get("last_error", "")
            ):
                raise AssertionError(
                    f"fatal encoder failure status is incorrect: {failed_snapshot}"
                )
        finally:
            failed_fixture.close()
            if failed_process.poll() is None:
                failed_process.kill()

    print(
        "video_gst_cli h264=32_decoded generation_restart=401,402 "
        f"encoder=x264enc stale=ignored idle_timeout=ok {whep_result} artifacts={directory}"
    )
    if cleanup is not None:
        cleanup.cleanup()


if __name__ == "__main__":
    main()
