"""Trusted local command for a RobotSimUE visual replay session."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from sim.runtime.coordinator import CoordinatorError, RunAllocationError

from .visual import VisualReplayConfig, VisualReplayError, run_visual_replay


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Launch RobotSimUE and present one validated simulation recording "
            "without launching MuJoCo."
        )
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, default=Path("runs/simulation-replay"))
    parser.add_argument("--unreal-editor", type=Path, required=True)
    parser.add_argument("--uproject", type=Path)
    parser.add_argument("--map", dest="map_name")
    parser.add_argument("--run-id")
    parser.add_argument("--snapshot-port", type=int, default=25124)
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--no-pace", action="store_true")
    parser.add_argument("--ready-timeout-s", type=float, default=180.0)
    parser.add_argument("--screenshot-timeout-s", type=float, default=30.0)
    parser.add_argument("--frame-timeout-s", type=float, default=30.0)
    parser.add_argument("--capture-every", type=int, default=10)
    parser.add_argument("--maximum-frames", type=int, default=12)
    parser.add_argument("--minimum-frames", type=int, default=2)
    parser.add_argument("--session-camera-tag")
    parser.add_argument("--motion-camera-stable-id")
    return parser


def _default_uproject(repo_root: Path) -> Path:
    return (
        repo_root
        / "sim"
        / "runtime"
        / "visual"
        / "RobotSimUE"
        / "RobotSimUE.uproject"
    )


def main(argv: list[str] | None = None) -> int:
    """Run one bounded visual replay and print its committed result."""

    args = _parser().parse_args(argv)
    repo_root = args.repo_root.resolve()
    try:
        result = run_visual_replay(
            VisualReplayConfig(
                bundle_dir=args.bundle,
                recording_dir=args.recording,
                repo_root=repo_root,
                run_root=args.run_root,
                unreal_editor=args.unreal_editor,
                uproject=args.uproject or _default_uproject(repo_root),
                run_id=args.run_id,
                map_name=args.map_name,
                snapshot_port=args.snapshot_port,
                dds_domain=args.dds_domain,
                rate=args.rate,
                pace=not args.no_pace,
                ready_timeout_s=args.ready_timeout_s,
                screenshot_timeout_s=args.screenshot_timeout_s,
                frame_timeout_s=args.frame_timeout_s,
                capture_every=args.capture_every,
                maximum_frames=args.maximum_frames,
                minimum_frames=args.minimum_frames,
                session_camera_tag=args.session_camera_tag,
                motion_camera_stable_id=args.motion_camera_stable_id,
            )
        )
    except (
        CoordinatorError,
        OSError,
        RunAllocationError,
        ValueError,
        VisualReplayError,
    ) as exc:
        print(
            json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False),
            file=sys.stderr,
        )
        return 1
    print(json.dumps({"ok": True, **result.to_dict()}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
