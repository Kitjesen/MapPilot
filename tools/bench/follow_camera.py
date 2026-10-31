"""Compare real-size synthetic RGB-D processing with and without native encoding.

Linux-only local component evidence. Detection/features, depth and odometry are
controlled inputs; this does not test recognition, calibration or robot motion.
"""

from __future__ import annotations

import argparse
import faulthandler
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from decision.modules.visual_servo import VisualServoModule
from drivers.real.camera.dds_module import DdsCameraModule
from drivers.real.camera.shm import ShmFrameWriter, StreamKind
from perception.backends import RgbdObservationSource
from perception.module import PerceptionModule
from perception.pipeline import PerceptionSettings
from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import NavigationState, Odometry


class ControlledDetector:
    """Fixed identity evidence and explicit latency, never a trained model."""

    def __init__(self, width: int, height: int, delay_ms: float):
        self.bbox = np.array([width * 0.35, height * 0.15, width * 0.65, height * 0.9])
        self.feature = np.eye(1, 384, dtype=np.float32)[0]
        self.delay_s = delay_ms / 1000

    def detect(self, _image, _prompt):
        time.sleep(self.delay_s)
        return [SimpleNamespace(
            bbox=self.bbox, label="person", score=0.95,
            track_id=7, dino_feature=self.feature,
        )]


def distribution(values):
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    return {
        "count": len(values), "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)), "max": float(max(values)),
    }


def rss_mib(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024
    except FileNotFoundError:
        pass
    return 0.0


def cpu_seconds(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def run_phase(args, encoded):
    name = "with_video" if encoded else "without_video"
    output = args.output / name
    output.mkdir(parents=True, exist_ok=False)
    paths = {key: Path(f"/dev/shm/lingtu_follow_check_{os.getpid()}_{key}")
             for key in ("color", "depth", "info")}
    writers = {}
    native = None
    log = (output / "native.log").open("w")
    camera = DdsCameraModule(**{f"{key}_shm_path": str(path) for key, path in paths.items()})
    transform = np.eye(4)
    transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    perception = PerceptionModule(
        settings=PerceptionSettings(laplacian_threshold=0),
        detector=RgbdObservationSource(
            ControlledDetector(args.width, args.height, args.detector_delay_ms),
            min_depth=0.3, max_depth=6, u16_depth_scale=0.001,
        ),
        camera_to_body=transform, skip_frames=1, max_rgbd_skew_s=0.02,
        max_odom_age_s=0.1, max_map_odom_age_s=0.5,
        product_session_id="local-follow-camera-check",
    )
    follow = VisualServoModule(product_session_id="local-follow-camera-check")
    camera.color_image.subscribe(perception.color_image._deliver)
    camera.depth_image.subscribe(perception.depth_image._deliver)
    camera.camera_info.subscribe(perception.camera_info._deliver)
    perception.observation.subscribe(follow.observation._deliver)
    observations, goals, ages, elapsed_ages, identities = [], [], [], [], set()
    capture_times = {}
    host_rss, video_rss = [], []
    follow.goal_pose.subscribe(goals.append)

    def observed(item):
        if item.status == "positive":
            age = (time.time() - item.source_ts) * 1000
            elapsed_age = (time.monotonic() - capture_times[item.image.source_sequence]) * 1000
            ages.append(age)
            elapsed_ages.append(elapsed_age)
            observations.append({
                "source_sequence": item.image.source_sequence,
                "source_ts": item.source_ts, "age_ms": age,
                "processing_elapsed_ms": elapsed_age,
                "wall_clock_step_ms": age - elapsed_age,
                "position": [item.detections[0].position.x,
                             item.detections[0].position.y,
                             item.detections[0].position.z],
            })
            identity = follow.get_servo_status().get("target_identity_id")
            if identity:
                identities.add(identity)

    perception.observation.subscribe(observed)
    receipt = None
    frames = 0
    elapsed = 0.0
    host_cpu_percent = 0.0
    video_cpu_percent = 0.0
    try:
        for key, kind, capacity in (("color", StreamKind.COLOR, args.width * args.height * 3),
                                    ("depth", StreamKind.DEPTH, args.width * args.height * 2),
                                    ("info", StreamKind.INFO, 64)):
            writers[key] = ShmFrameWriter(paths[key], stream_kind=kind,
                                         slot_capacity=capacity, source_generation=101)
        follow.setup()
        perception.setup()
        perception.start()
        camera.setup()
        camera.start()
        perception.navigation_state._deliver(NavigationState(
            boot_id="controlled-nav", sequence=1, map_id="controlled-map", map_content_epoch=1,
        ))
        if encoded:
            native = subprocess.Popen([
                "bash", "-c", 'source "$1"; shift; exec "$@"', "follow-camera-check",
                str(args.runtime_env), str(args.native_bin),
                "--color-shm", "/" + paths["color"].name,
                "--encoder", args.encoder, "--fps", "20",
                "--output", str(output / "stream.h264"),
                "--metrics", str(output / "native.jsonl"),
                "--metrics-interval-ms", "1000",
                "--status-file", str(output / "native-status.json"),
            ], stdout=log, stderr=subprocess.STDOUT)
        rows, columns = np.indices((args.height, args.width))
        checker = (((rows // 24 + columns // 24) % 2) * 150 + 40).astype(np.uint8)
        color = np.repeat(checker[:, :, None], 3, axis=2)
        depth = np.full((args.height, args.width), 3000, dtype=np.uint16).tobytes()
        host_cpu = cpu_seconds(os.getpid())
        video_cpu = cpu_seconds(native.pid) if native else 0.0
        started = time.monotonic()
        due = started
        sample_due = started
        while time.monotonic() - started < args.duration_s:
            if native is not None and native.poll() is not None:
                raise RuntimeError(f"native video exited early: {native.returncode}; see {log.name}")
            capture_times[frames + 1] = time.monotonic()
            stamp = time.time_ns()
            perception.odometry._deliver(Odometry(pose=Pose(position=Vector3()),
                                                  ts=stamp * 1e-9, frame_id="map"))
            metadata = dict(timestamp_ns=stamp, width=args.width, height=args.height,
                            frame_id="controlled_camera_optical")
            writers["info"].publish(**metadata, stride=0, encoding="camera_info", payload=b"",
                                    fx=600, fy=600, cx=args.width / 2, cy=args.height / 2,
                                    depth_scale=0.001)
            writers["depth"].publish(**metadata, stride=args.width * 2,
                                     encoding="16UC1", payload=depth)
            color[:, :, 2] = np.roll(checker, frames % 48, axis=1)
            writers["color"].publish(**metadata, stride=args.width * 3,
                                     encoding="bgr8", payload=color.tobytes())
            frames += 1
            if receipt is None and observations:
                receipt = follow.start_follow(target_id="track_7")
                if receipt.get("accepted") is not True:
                    raise RuntimeError(f"controlled Follow admission failed: {receipt}")
            now = time.monotonic()
            if now >= sample_due:
                host_rss.append(rss_mib(os.getpid()))
                if native:
                    video_rss.append(rss_mib(native.pid))
                sample_due = now + 0.5
            due += 1 / args.fps
            if due < now:
                due = now
            time.sleep(max(0, due - time.monotonic()))
        elapsed = time.monotonic() - started
        host_cpu_percent = (cpu_seconds(os.getpid()) - host_cpu) / elapsed * 100
        if native:
            video_cpu_percent = (cpu_seconds(native.pid) - video_cpu) / elapsed * 100
        assert receipt and goals and len(observations) >= 10, "no complete RGB-D/Follow flow"
    finally:
        if not observations:
            with (output / "threads.log").open("w") as threads:
                faulthandler.dump_traceback(file=threads)
        (output / "diagnostics.json").write_text(json.dumps({
            "source_frames": frames, "observations": len(observations),
            "follow_receipt": receipt, "goals": len(goals),
            "camera": camera.health(), "perception": perception.health(),
            "follow": follow.get_servo_status(),
        }, indent=2) + "\n")
        camera.stop()
        perception.stop()
        if receipt:
            follow.cancel_follow_task(receipt["follow_task_id"], receipt["product_session_id"])
        follow.stop()
        if native and native.poll() is None:
            native.send_signal(signal.SIGINT)
            try:
                native.wait(timeout=10)
            except subprocess.TimeoutExpired:
                native.kill()
                native.wait()
        for writer in writers.values():
            writer.close()
        for path in paths.values():
            path.unlink(missing_ok=True)
        log.close()
        with (output / "observations.jsonl").open("w") as handle:
            for observation in observations:
                handle.write(json.dumps(observation) + "\n")
    if native and native.returncode != 0:
        raise RuntimeError(f"native video stop failed: {native.returncode}")
    if encoded and not (output / "stream.h264").stat().st_size:
        raise RuntimeError("no encoded H.264 output")
    return {
        "phase": name, "elapsed_s": elapsed, "source_frames": frames,
        "source_fps": frames / elapsed, "positive_observations": len(observations),
        "observation_age_ms": distribution(ages),
        "processing_elapsed_ms": distribution(elapsed_ages), "goal_requests": len(goals),
        "host_cpu_percent_one_core": host_cpu_percent,
        "video_cpu_percent_one_core": video_cpu_percent,
        "host_rss_mib": distribution(host_rss), "video_rss_mib": distribution(video_rss),
        "perception": perception.health(), "camera": camera.health(),
        "follow_task": follow.get_follow_task(receipt["follow_task_id"], receipt["product_session_id"]),
        "observed_identity_count": len(identities),
        "h264_bytes": (output / "stream.h264").stat().st_size if encoded else 0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native-bin", type=Path, required=True)
    parser.add_argument("--runtime-env", type=Path, required=True)
    parser.add_argument("--encoder", default="x264enc")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration-s", type=float, default=30)
    parser.add_argument("--detector-delay-ms", type=float, default=40)
    args = parser.parse_args()
    if sys.platform != "linux":
        parser.error("native POSIX SHM and encoding check requires Linux/WSL")
    args.output = args.output.resolve()
    args.native_bin = args.native_bin.resolve()
    args.runtime_env = args.runtime_env.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    warmup_started = time.monotonic()
    # Keep the first lazy SciPy filesystem import outside the steady-load window.
    from scipy.cluster.hierarchy import fclusterdata

    result = {"level": "local_synthetic_parallel_load", "robot_product_started": False,
              "recognition_tested": False, "motion_tested": False,
              "dependency_warmup_s": time.monotonic() - warmup_started,
              "input": {"width": args.width, "height": args.height, "fps": args.fps,
                        "detector": "controlled", "detector_delay_ms": args.detector_delay_ms},
              "phases": []}
    try:
        for encoded in (False, True):
            print(f"starting {'with' if encoded else 'without'} video", flush=True)
            result["phases"].append(run_phase(args, encoded))
        result["passed"] = True
    except Exception as error:
        result.update(passed=False, error=str(error))
        raise
    finally:
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({
            "passed": result.get("passed", False), "error": result.get("error"),
            "summary": str(args.output / "summary.json"),
            "phases": [{key: phase[key] for key in (
                "phase", "source_fps", "positive_observations", "observation_age_ms",
                "processing_elapsed_ms",
                "host_cpu_percent_one_core", "video_cpu_percent_one_core", "h264_bytes",
            )} for phase in result["phases"]],
        }, indent=2), flush=True)


if __name__ == "__main__":
    main()
