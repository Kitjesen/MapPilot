"""Launch MuJoCo headless plus RobotSimUE live visual streaming."""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from sim.runtime.control.fake import zero_output_components
from sim.runtime.sensors import (
    ImuEndpointFactory,
    Mid360EndpointFactory,
    SensorEndpointRouter,
    TruthOdometryEndpointFactory,
)

from .coordinator import CoordinatorError, RuntimeCoordinator
from .external_evidence import ExternalEvidenceWatcher
from .live_snapshot import UdpLoopbackSnapshotPublisher
from .mujoco_process import MujocoProcess
from .run_allocation import RunAllocationError, load_resolved_session_bundle
from .session_host import SessionHost, VisualProcess
from .unreal_process import (
    PackagedUnrealProcess,
    UnrealLaunchProfile,
    UnrealProcess,
)

GateMode = Literal["visual-applied", "session-ready"]
DEFAULT_SNAPSHOT_PORT = 25123
_UNREAL_LEVEL_RE = re.compile(r"^/Game/\S(?:.*\S)?$")


def _validated_unreal_level(value: object, context: str) -> str:
    if not isinstance(value, str) or _UNREAL_LEVEL_RE.fullmatch(value) is None:
        raise ValueError(f"{context} must be a valid /Game/... level")
    return value


def _native_sensor_endpoint_router(
    *,
    gate: GateMode,
    physics_host: MujocoProcess,
    imu_publisher: Path | None,
    truth_odom_publisher: Path | None,
    mid360_publisher: Path | None,
    truth_odom_parent_frame: str,
) -> SensorEndpointRouter | None:
    """Bind all native sensor publishers atomically around one Physics host."""

    if gate not in {"visual-applied", "session-ready"}:
        raise ValueError(f"unsupported gate: {gate}")
    publishers = (
        ("--imu-publisher", imu_publisher),
        ("--truth-odom-publisher", truth_odom_publisher),
        ("--mid360-publisher", mid360_publisher),
    )
    missing = tuple(name for name, value in publishers if value is None)
    if len(missing) == len(publishers):
        if gate == "session-ready":
            raise ValueError(
                "session-ready requires --imu-publisher, "
                "--truth-odom-publisher, and --mid360-publisher"
            )
        return None
    if missing:
        raise ValueError(
            "native sensor publishers must be provided together; missing: "
            + ", ".join(missing)
        )

    if imu_publisher is None or truth_odom_publisher is None or mid360_publisher is None:
        raise ValueError("native sensor publisher validation invariant failed")
    return SensorEndpointRouter(
        (
            ImuEndpointFactory(imu_publisher),
            TruthOdometryEndpointFactory(
                truth_odom_publisher,
                parent_frame=truth_odom_parent_frame,
            ),
            Mid360EndpointFactory(
                mid360_publisher,
                process=physics_host,
            ),
        )
    )




@dataclass(frozen=True)
class LiveVisualLaunch:
    """Concrete objects and paths for one live visual run."""

    host: SessionHost
    coordinator: RuntimeCoordinator
    run_id: str
    gate: GateMode
    snapshot_port: int
    visual_evidence_path: Path
    first_frame_screenshot_path: Path
    screenshot_timeout_s: float
    sensor_evidence_path: Path
    run_allocation_path: Path
    unreal_log_path: Path


def default_uproject(repo_root: Path) -> Path:
    """Return the repo-local RobotSimUE project path."""

    return (
        Path(repo_root)
        / "sim"
        / "runtime"
        / "visual"
        / "RobotSimUE"
        / "RobotSimUE.uproject"
    )


def create_live_visual_launch(
    *,
    bundle: Path,
    repo_root: Path,
    run_root: Path,
    mujoco_host: Path,
    unreal_editor: Path | None = None,
    uproject: Path | None = None,
    packaged_executable: Path | None = None,
    map_name: str | None = None,
    gate: GateMode = "visual-applied",
    imu_publisher: Path | None = None,
    truth_odom_publisher: Path | None = None,
    mid360_publisher: Path | None = None,
    truth_odom_parent_frame: str = "world",
    snapshot_port: int = DEFAULT_SNAPSHOT_PORT,
    dds_domain: int = 0,
    run_id: str | None = None,
    motion_camera_stable_id: str | None = None,
    frame_capture_dir: Path | None = None,
    frame_capture_every: int = 1,
    frame_capture_max: int = 1,
    ready_timeout_s: float = 45.0,
    warmup_steps: int = 8,
    sleep_s: float = 0.01,
    screenshot_timeout_s: float = 30.0,
) -> LiveVisualLaunch:
    """Assemble the existing coordinator, MuJoCo, UDP, UE, and evidence seams."""

    if isinstance(snapshot_port, bool) or not isinstance(snapshot_port, int):
        raise ValueError("snapshot_port must be an integer in [1, 65535]")
    if snapshot_port < 1 or snapshot_port > 65535:
        raise ValueError("snapshot_port must be an integer in [1, 65535]")
    resolved_repo = Path(repo_root).resolve()
    resolved_run_root = Path(run_root).resolve()
    resolved_bundle = load_resolved_session_bundle(bundle, repo_root=resolved_repo)
    level_source = (
        map_name
        if map_name is not None
        else resolved_bundle.plans["visual.plan.json"]["world"]["level"]
    )
    resolved_map_name = _validated_unreal_level(
        level_source,
        "map_name" if map_name is not None else "visual.plan.world.level",
    )
    editor_requested = unreal_editor is not None or uproject is not None
    packaged_requested = packaged_executable is not None
    if editor_requested == packaged_requested:
        raise ValueError(
            "exactly one Unreal runtime must be selected: "
            "unreal_editor + uproject, or packaged_executable"
        )
    if editor_requested and (unreal_editor is None or uproject is None):
        raise ValueError("Unreal Editor runtime requires both unreal_editor and uproject")
    resolved_run_id = run_id or f"live-visual-{uuid.uuid4().hex[:12]}"
    run_dir = resolved_run_root / resolved_run_id
    log_dir = run_dir / "logs"
    visual_evidence_path = log_dir / "visual-readiness.json"
    first_frame_screenshot_path = log_dir / "visual-first-frame.png"
    sensor_evidence_path = log_dir / "sensor-readiness.json"

    physics_host = MujocoProcess(mujoco_host)
    sensor_endpoint_factory = _native_sensor_endpoint_router(
        gate=gate,
        physics_host=physics_host,
        imu_publisher=imu_publisher,
        truth_odom_publisher=truth_odom_publisher,
        mid360_publisher=mid360_publisher,
        truth_odom_parent_frame=truth_odom_parent_frame,
    )
    coordinator = RuntimeCoordinator(
        bundle_dir=resolved_bundle.bundle_dir,
        repo_root=resolved_repo,
        run_root=resolved_run_root,
        physics_host=physics_host,
        controller_factory=zero_output_components,
        sensor_endpoint_factory=sensor_endpoint_factory,
        run_id=resolved_run_id,
        dds_domain=dds_domain,
        ports={"visual_snapshot_udp": snapshot_port},
    )
    visual_watcher = ExternalEvidenceWatcher(
        visual_evidence_path,
        session_id=resolved_bundle.session_id,
        model_generation=0,
        reset_generation=0,
        expected_source_id="robotsimue-visual",
    )
    sensor_watcher = ExternalEvidenceWatcher(
        sensor_evidence_path,
        session_id=resolved_bundle.session_id,
        model_generation=0,
        reset_generation=0,
        expected_source_id="robotsimue-camera",
    )
    unreal_process: VisualProcess
    if packaged_executable is not None:
        unreal_process = PackagedUnrealProcess(
            packaged_executable,
            resolved_map_name,
            frame_capture_dir=frame_capture_dir,
            frame_capture_every=frame_capture_every,
            frame_capture_max=frame_capture_max,
            motion_camera_stable_id=motion_camera_stable_id,
        )
        unreal_log_path = log_dir / "RobotSimUE.log"
    else:
        if unreal_editor is None or uproject is None:
            raise ValueError("Unreal Editor runtime selection invariant failed")
        unreal_process = UnrealProcess(
            unreal_editor,
            uproject,
            resolved_map_name,
            launch_profile=UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
            frame_capture_dir=frame_capture_dir,
            frame_capture_every=frame_capture_every,
            frame_capture_max=frame_capture_max,
            motion_camera_stable_id=motion_camera_stable_id,
        )
        unreal_log_path = log_dir / "Unreal.log"
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=unreal_process,
        publisher=UdpLoopbackSnapshotPublisher(snapshot_port),
        evidence_watchers=(visual_watcher, sensor_watcher),
        snapshot_port=snapshot_port,
        warmup_steps=warmup_steps,
        ready_timeout_s=ready_timeout_s,
        sleep_s=sleep_s,
    )
    return LiveVisualLaunch(
        host=host,
        coordinator=coordinator,
        run_id=resolved_run_id,
        gate=gate,
        snapshot_port=snapshot_port,
        visual_evidence_path=visual_evidence_path,
        first_frame_screenshot_path=first_frame_screenshot_path,
        screenshot_timeout_s=screenshot_timeout_s,
        run_allocation_path=run_dir / "run-allocation.json",
        sensor_evidence_path=sensor_evidence_path,
        unreal_log_path=unreal_log_path,
    )


def _remove_stale_screenshot(path: Path) -> None:
    """Ensure a reused run directory cannot satisfy current-run evidence."""

    try:
        path.unlink(missing_ok=True)
    except OSError as exc:
        raise CoordinatorError(f"cannot remove stale visual screenshot: {path} ({exc})") from exc


def _wait_for_nonempty_png(path: Path, *, timeout_s: float = 5.0, sleep_s: float = 0.05) -> None:
    """Fail closed unless UE wrote a bounded, real PNG screenshot artifact."""

    deadline = time.monotonic() + timeout_s
    last_error = "file is missing"
    last_valid_size: int | None = None
    while True:
        try:
            if path.is_file():
                size = path.stat().st_size
                if size > 8:
                    with path.open("rb") as handle:
                        signature = handle.read(8)
                    if signature == b"\x89PNG\r\n\x1a\n":
                        if last_valid_size == size:
                            return
                        last_valid_size = size
                        last_error = "file is still being written"
                    else:
                        last_valid_size = None
                        last_error = "file is not a PNG"
                else:
                    last_valid_size = None
                    last_error = "file is empty"
        except OSError as exc:
            last_valid_size = None
            last_error = str(exc)
        if time.monotonic() >= deadline:
            raise CoordinatorError(f"visual first-frame screenshot was not produced: {path} ({last_error})")
        time.sleep(sleep_s)


def run_live_visual(
    launch: LiveVisualLaunch,
    *,
    gate: GateMode,
    frames: int,
    steps_per_frame: int,
) -> dict[str, object]:
    """Run one bounded live visual session and return machine-readable evidence."""

    if gate != launch.gate:
        raise ValueError("run gate must match the gate validated during launch assembly")
    try:
        _remove_stale_screenshot(launch.first_frame_screenshot_path)
        if gate == "visual-applied":
            launch.host.prepare_until_visual_applied()
            _wait_for_nonempty_png(
                launch.first_frame_screenshot_path,
                timeout_s=launch.screenshot_timeout_s,
            )
            streamed_frames = launch.host.run_visual_only(
                frame_limit=frames,
                steps_per_frame=steps_per_frame,
                close_on_finish=False,
            )
            stop_condition = (
                "visual ACTIVE evidence observed; continued via PREPARING-safe "
                "warmup if session READY was blocked"
            )
        elif gate == "session-ready":
            launch.host.prepare()
            _wait_for_nonempty_png(
                launch.first_frame_screenshot_path,
                timeout_s=launch.screenshot_timeout_s,
            )
            streamed_frames = launch.host.run(
                frame_limit=frames,
                steps_per_frame=steps_per_frame,
                close_on_finish=False,
            )
            stop_condition = "all required session bindings reached READY before RUNNING"
        else:
            raise ValueError(f"unsupported gate: {gate}")

        manifest_path = launch.coordinator.manifest_path
        result: dict[str, object] = {
            "ok": True,
            "gate": gate,
            "frames": streamed_frames,
            "snapshot_port": launch.snapshot_port,
            "run_id": launch.run_id,
            "session_runtime_manifest": str(manifest_path),
            "run_allocation": str(launch.run_allocation_path),
            "visual_evidence": str(launch.visual_evidence_path),
            "first_frame_screenshot": str(launch.first_frame_screenshot_path),
            "unreal_log": str(launch.unreal_log_path),
            "sensor_evidence": str(launch.sensor_evidence_path),
            "snapshot_source": "MuJoCo headless via RuntimeCoordinator.advance/warmup -> UDP truth-snapshot.v1",
            "stop_condition": stop_condition,
        }
    except BaseException as exc:
        try:
            launch.host.close()
        except Exception as close_error:
            if hasattr(exc, "add_note"):
                exc.add_note(f"live visual cleanup failed: {close_error}")
        raise

    launch.host.close()
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Launch a resolved LingTu SessionBundle with C++ MuJoCo headless "
            "and RobotSimUE live truth-snapshot visualization."
        )
    )
    parser.add_argument("bundle", type=Path, help="ResolvedSessionBundle directory")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, default=Path("runs/simulation"))
    parser.add_argument("--mujoco-host", type=Path, required=True)
    unreal_runtime = parser.add_mutually_exclusive_group(required=True)
    unreal_runtime.add_argument("--unreal-editor", type=Path)
    unreal_runtime.add_argument("--packaged-executable", type=Path)
    parser.add_argument("--uproject", type=Path)
    parser.add_argument("--map", dest="map_name")
    parser.add_argument("--snapshot-port", type=int, default=DEFAULT_SNAPSHOT_PORT)
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument(
        "--imu-publisher",
        type=Path,
        help="native Windows CycloneDDS IMU publisher executable",
    )
    parser.add_argument(
        "--truth-odom-publisher",
        type=Path,
        help="native Windows CycloneDDS truth-odometry publisher executable",
    )
    parser.add_argument(
        "--mid360-publisher",
        type=Path,
        help="native Windows CycloneDDS Mid360 publisher executable",
    )
    parser.add_argument(
        "--truth-odom-parent-frame",
        default="world",
        help="parent frame for the simulation-only truth odometry stream",
    )
    parser.add_argument("--run-id")
    parser.add_argument(
        "--motion-camera-stable-id",
        help="stable body ID used to anchor the generated third-person camera",
    )
    parser.add_argument(
        "--frame-capture-dir",
        type=Path,
        help="optional directory for bounded live UE frame capture",
    )
    parser.add_argument("--frame-capture-every", type=int, default=1)
    parser.add_argument("--frame-capture-max", type=int, default=1)
    parser.add_argument("--gate", choices=("visual-applied", "session-ready"), default="visual-applied")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--steps-per-frame", type=int, default=8)
    parser.add_argument("--ready-timeout-s", type=float, default=45.0)
    parser.add_argument("--screenshot-timeout-s", type=float, default=30.0)
    parser.add_argument("--warmup-steps", type=int, default=8)
    parser.add_argument("--sleep-s", type=float, default=0.01)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the live visual coordinator command-line entry point."""
    parser = _parser()
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    uproject = (
        args.uproject or default_uproject(repo_root)
        if args.unreal_editor is not None
        else None
    )
    try:
        launch = create_live_visual_launch(
            bundle=args.bundle,
            repo_root=repo_root,
            run_root=args.run_root,
            mujoco_host=args.mujoco_host,
            unreal_editor=args.unreal_editor,
            uproject=uproject,
            packaged_executable=args.packaged_executable,
            map_name=args.map_name,
            gate=args.gate,
            imu_publisher=args.imu_publisher,
            truth_odom_publisher=args.truth_odom_publisher,
            mid360_publisher=args.mid360_publisher,
            truth_odom_parent_frame=args.truth_odom_parent_frame,
            snapshot_port=args.snapshot_port,
            dds_domain=args.dds_domain,
            run_id=args.run_id,
            motion_camera_stable_id=args.motion_camera_stable_id,
            frame_capture_dir=args.frame_capture_dir,
            frame_capture_every=args.frame_capture_every,
            frame_capture_max=args.frame_capture_max,
            ready_timeout_s=args.ready_timeout_s,
            screenshot_timeout_s=args.screenshot_timeout_s,
            warmup_steps=args.warmup_steps,
            sleep_s=args.sleep_s,
        )
        result = run_live_visual(
            launch,
            gate=args.gate,
            frames=args.frames,
            steps_per_frame=args.steps_per_frame,
        )
    except (CoordinatorError, RunAllocationError, OSError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
