#!/usr/bin/env python3
import argparse
import json
import os
import signal
import subprocess
import tempfile
import time
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
        ready = self.process.stdout.readline().strip()
        if ready != f"ready {name}":
            raise AssertionError(f"fixture did not become ready: {ready!r}")

    def publish(self, generation, count=1, age_ms=0, encoding="rgb8"):
        self.process.stdin.write(
            f"publish {generation} {count} {age_ms} {encoding}\n"
        )
        self.process.stdin.flush()
        result = self.process.stdout.readline().strip()
        expected = f"published {generation} "
        if not result.startswith(expected):
            raise AssertionError(f"fixture publish failed: {result!r}")
        return result

    def close(self):
        if self.process.poll() is not None:
            return
        self.process.stdin.write("quit\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline().strip()
        if response != "bye":
            raise AssertionError(f"fixture shutdown failed: {response!r}")
        self.process.wait(timeout=3)

    def terminate(self):
        if self.process.poll() is None:
            self.process.terminate()
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


def wait_for(predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("timed out waiting for video test event")


def submitted(path):
    return [event for event in events(path) if event.get("event") == "submitted"]


def video_command(
    video, source, output, metadata, encoder, frames=0, idle_ms=1200, fps=120, gop=1
):
    return [
        video,
        "--color-shm",
        source,
        "--output",
        str(output),
        "--metadata",
        str(metadata),
        "--ffmpeg",
        encoder,
        "--fps",
        str(fps),
        "--gop",
        str(gop),
        "--max-age-ms",
        "2000",
        "--write-timeout-ms",
        "2000",
        "--idle-timeout-ms",
        str(idle_ms),
        "--frames",
        str(frames),
    ]


def run_process(command, env=None):
    return subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=False, env=env
    )


def finish(process, timeout=6):
    stdout, stderr = process.communicate(timeout=timeout)
    return process.returncode, stdout, stderr.decode("utf-8", errors="replace")


def make_fake_encoder(directory):
    path = directory / "fake_ffmpeg.py"
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "while True:\n"
        "    data = os.read(0, 65536)\n"
        "    if not data:\n"
        "        break\n"
        "    view = memoryview(data)\n"
        "    while view:\n"
        "        count = os.write(1, view)\n"
        "        view = view[count:]\n",
        encoding="utf-8",
    )
    path.chmod(0o755)
    return str(path)


def make_blocked_encoder(directory):
    path = directory / "blocked_ffmpeg.py"
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "import time\n"
        "with open(os.environ['LINGTU_BLOCK_ENCODER_PIDFILE'], 'w', encoding='ascii') as stream:\n"
        "    stream.write(str(os.getpid()))\n"
        "    stream.flush()\n"
        "time.sleep(60)\n",
        encoding="utf-8",
    )
    path.chmod(0o755)
    return str(path)


def scenario_latest(video, fixture_exe, encoder, directory):
    name = f"/lingtu_video_latest_{os.getpid()}_{time.time_ns()}"
    output = directory / "latest.h264"
    metadata = directory / "latest.jsonl"
    fixture = Fixture(fixture_exe, name)
    fixture.publish(101, count=5, encoding="rgb8")
    process = run_process(
        video_command(video, name, output, metadata, encoder, frames=2, fps=10)
    )
    try:
        first = wait_for(lambda: submitted(metadata))
        if first[0]["sequence"] != 5:
            raise AssertionError(f"backlog was not skipped to latest frame: {first}")
        fixture.publish(101, count=3, encoding="rgb8")
        wait_for(lambda: len(submitted(metadata)) == 2)
        code, _, stderr = finish(process)
        if code != 0:
            raise AssertionError(f"latest-frame run failed: {stderr}")
        sent = submitted(metadata)
        if sent[1]["sequence"] != 8 or sent[1]["skipped"] != 2:
            raise AssertionError(f"latest-frame skip metadata is wrong: {sent}")
        if output.stat().st_size != 2 * FRAME_BYTES:
            raise AssertionError("fake encoder did not receive two visible 16x16 frames")
        summary = events(metadata)[-1]
        if summary.get("event") != "finished" or summary.get("submitted") != 2:
            raise AssertionError(f"missing completion metadata: {summary}")
    finally:
        fixture.close()
        if process.poll() is None:
            process.kill()


def scenario_restart(video, fixture_exe, encoder, directory, real_h264=False):
    name = f"/lingtu_video_restart_{os.getpid()}_{time.time_ns()}"
    output = directory / ("restart-real.h264" if real_h264 else "restart.h264")
    metadata = directory / ("restart-real.jsonl" if real_h264 else "restart.jsonl")
    fixture = Fixture(fixture_exe, name)
    process = run_process(
        video_command(video, name, output, metadata, encoder, frames=2)
    )
    try:
        fixture.publish(201, age_ms=5000, encoding="rgb8")
        time.sleep(0.15)
        if submitted(metadata):
            raise AssertionError("stale frame was submitted")
        fixture.publish(201, encoding="rgb8")
        wait_for(lambda: len(submitted(metadata)) == 1)
        fixture.publish(202, encoding="bgr8")
        wait_for(lambda: len(submitted(metadata)) == 2)
        code, _, stderr = finish(process, timeout=8)
        if code != 0:
            raise AssertionError(f"generation restart run failed: {stderr}")
        sent = submitted(metadata)
        if [(item["source_generation"], item["sequence"]) for item in sent] != [
            (201, 2),
            (202, 1),
        ]:
            raise AssertionError(f"generation/sequence restart is wrong: {sent}")
        streams = [event for event in events(metadata) if event.get("event") == "stream"]
        if len(streams) != 2:
            raise AssertionError(f"encoder stream was not restarted: {streams}")
        if not real_h264 and output.stat().st_size != 2 * FRAME_BYTES:
            raise AssertionError("fake restart output has the wrong frame count")
        return output, metadata
    finally:
        fixture.close()
        if process.poll() is None:
            process.kill()


def scenario_failures(video, fixture_exe, fake, directory):
    missing_name = f"/lingtu_video_missing_{os.getpid()}_{time.time_ns()}"
    idle_output = directory / "idle.h264"
    idle_metadata = directory / "idle.jsonl"
    idle = run_process(
        video_command(
            video, missing_name, idle_output, idle_metadata, fake, idle_ms=100
        )
    )
    code, _, stderr = finish(idle)
    if code == 0 or "timed out" not in stderr:
        raise AssertionError(f"missing camera did not fail cleanly: {code}, {stderr}")

    existing = directory / "existing.h264"
    existing.write_bytes(b"keep")
    existing_metadata = directory / "existing.jsonl"
    collision = run_process(
        video_command(
            video, missing_name, existing, existing_metadata, fake, idle_ms=100
        )
    )
    code, _, _ = finish(collision)
    if code == 0 or existing.read_bytes() != b"keep":
        raise AssertionError("existing output was overwritten")

    name = f"/lingtu_video_stop_{os.getpid()}_{time.time_ns()}"
    stop_output = directory / "stop.h264"
    stop_metadata = directory / "stop.jsonl"
    fixture = Fixture(fixture_exe, name)
    stopped = run_process(
        video_command(video, name, stop_output, stop_metadata, fake, frames=0)
    )
    try:
        fixture.publish(301)
        wait_for(lambda: submitted(stop_metadata))
        stopped.send_signal(signal.SIGTERM)
        code, _, stderr = finish(stopped)
        if code != 0:
            raise AssertionError(f"SIGTERM did not drain encoder cleanly: {stderr}")
        if events(stop_metadata)[-1].get("event") != "finished":
            raise AssertionError("SIGTERM run did not emit completion metadata")
    finally:
        fixture.close()
        if stopped.poll() is None:
            stopped.kill()


def process_exists(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def scenario_blocked_stop(video, fixture_exe, directory):
    name = f"/lingtu_video_blocked_{os.getpid()}_{time.time_ns()}"
    output = directory / "blocked.h264"
    metadata = directory / "blocked.jsonl"
    pidfile = directory / "blocked.pid"
    blocked_encoder = make_blocked_encoder(directory)
    environment = os.environ.copy()
    environment["LINGTU_BLOCK_ENCODER_PIDFILE"] = str(pidfile)
    fixture = Fixture(fixture_exe, name)
    process = run_process(
        video_command(video, name, output, metadata, blocked_encoder, frames=0),
        env=environment,
    )
    child_pid = None
    try:
        fixture.publish(501)
        wait_for(lambda: pidfile.exists(), timeout=3)
        child_pid = int(pidfile.read_text(encoding="ascii"))
        wait_for(lambda: len(submitted(metadata)) == 1, timeout=3)
        completed = 1
        blocked = False
        for _ in range(255):
            fixture.publish(501)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                current = len(submitted(metadata))
                if current > completed:
                    completed = current
                    break
                if process.poll() is not None:
                    _, stderr = process.communicate()
                    raise AssertionError(
                        "video process exited before blocked-stop signal: "
                        + stderr.decode("utf-8", errors="replace")
                    )
                time.sleep(0.005)
            else:
                blocked = True
                break
        if not blocked or completed == 0:
            raise AssertionError("could not fill the encoder input pipe")

        process.send_signal(signal.SIGTERM)
        code, _, stderr = finish(process, timeout=4)
        if code != 0:
            raise AssertionError(f"blocked SIGTERM returned {code}: {stderr}")
        recorded = events(metadata)
        stopped = [event for event in recorded if event.get("event") == "stopped"]
        if len(stopped) != 1:
            raise AssertionError(f"blocked stop metadata is missing: {recorded}")
        if stopped[0].get("encoder_aborted") is not True:
            raise AssertionError(f"blocked encoder was not marked aborted: {stopped[0]}")
        if stopped[0].get("submitted") != completed:
            raise AssertionError(
                f"blocked stop submitted count is wrong: {stopped[0]}, observed={completed}"
            )
        if any(event.get("event") == "finished" for event in recorded):
            raise AssertionError("cancelled blocked stream emitted finished metadata")
        wait_for(lambda: not process_exists(child_pid), timeout=3)
    finally:
        fixture.close()
        if process.poll() is None:
            process.kill()
        if child_pid is not None and process_exists(child_pid):
            os.kill(child_pid, signal.SIGKILL)


def decode_h264(ffmpeg, stream):
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
        check=False,
        timeout=10,
    )
    if result.returncode != 0:
        raise AssertionError(
            "FFmpeg could not decode produced H.264: "
            + result.stderr.decode("utf-8", errors="replace")
        )
    return result.stdout


def annexb_nal_types(stream):
    starts = []
    index = 0
    while index + 3 < len(stream):
        if stream[index : index + 4] == b"\x00\x00\x00\x01":
            starts.append(index + 4)
            index += 4
        elif stream[index : index + 3] == b"\x00\x00\x01":
            starts.append(index + 3)
            index += 3
        else:
            index += 1
    return [stream[start] & 0x1F for start in starts if start < len(stream)]


def validate_real_h264(ffmpeg, output, metadata):
    encoded = output.read_bytes()
    nal_types = annexb_nal_types(encoded)
    if 1 not in nal_types or 5 not in nal_types:
        raise AssertionError(f"H.264 lacks predictive or IDR pictures: {nal_types}")
    if nal_types.count(7) < 2 or nal_types.count(8) < 2:
        raise AssertionError(f"H.264 restart lacks repeated SPS/PPS: {nal_types}")
    raw = decode_h264(ffmpeg, encoded)
    successful = len(submitted(metadata))
    if len(raw) != successful * FRAME_BYTES:
        raise AssertionError(
            f"decoded {len(raw) // FRAME_BYTES} frames for {successful} submissions"
        )
    for offset in range(0, len(raw), FRAME_BYTES):
        frame = raw[offset : offset + FRAME_BYTES]
        red = sum(frame[0::3]) / (len(frame) / 3)
        green = sum(frame[1::3]) / (len(frame) / 3)
        blue = sum(frame[2::3]) / (len(frame) / 3)
        if red < 180 or green > 70 or blue > 70:
            raise AssertionError(
                f"decoded color is wrong: rgb means {red:.1f}, {green:.1f}, {blue:.1f}"
            )


def scenario_real_h264(video, fixture_exe, ffmpeg, directory):
    name = f"/lingtu_video_real_{os.getpid()}_{time.time_ns()}"
    output = directory / "continuous.h264"
    metadata = directory / "continuous.jsonl"
    fixture = Fixture(fixture_exe, name)
    process = run_process(
        video_command(
            video,
            name,
            output,
            metadata,
            ffmpeg,
            frames=32,
            idle_ms=2000,
            fps=30,
            gop=15,
        )
    )
    try:
        for index in range(16):
            fixture.publish(401, encoding="rgb8")
            wait_for(lambda index=index: len(submitted(metadata)) == index + 1)
        for index in range(16):
            fixture.publish(402, encoding="bgr8")
            wait_for(lambda index=index: len(submitted(metadata)) == 17 + index)
        code, _, stderr = finish(process, timeout=12)
        if code != 0:
            raise AssertionError(f"continuous H.264 run failed: {stderr}")
        streams = [event for event in events(metadata) if event.get("event") == "stream"]
        if len(streams) != 2:
            raise AssertionError(f"continuous H.264 did not restart stream: {streams}")
        validate_real_h264(ffmpeg, output, metadata)
        return output, metadata
    finally:
        fixture.close()
        if process.poll() is None:
            process.kill()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--ffmpeg")
    parser.add_argument("--artifacts")
    args = parser.parse_args()

    if args.artifacts:
        root = Path(args.artifacts)
        root.mkdir(parents=True, exist_ok=True)
        directory = Path(tempfile.mkdtemp(prefix="video-cli-", dir=root))
        cleanup = None
    else:
        cleanup = tempfile.TemporaryDirectory(prefix="lingtu-video-cli-")
        directory = Path(cleanup.name)

    fake = make_fake_encoder(directory)
    scenario_latest(args.video, args.fixture, fake, directory)
    scenario_restart(args.video, args.fixture, fake, directory)
    scenario_failures(args.video, args.fixture, fake, directory)
    scenario_blocked_stop(args.video, args.fixture, directory)
    facts = [
        "latest=5,8",
        "restart=201:2,202:1",
        "idle_failure=ok",
        "stop=drained",
        "blocked_stop=cancelled",
    ]
    if args.ffmpeg:
        scenario_real_h264(args.video, args.fixture, args.ffmpeg, directory)
        facts.append("h264=32_frames_idr_and_p_restart")
    print("video_cli " + " ".join(facts) + f" artifacts={directory}")
    if cleanup is not None:
        cleanup.cleanup()


if __name__ == "__main__":
    main()
