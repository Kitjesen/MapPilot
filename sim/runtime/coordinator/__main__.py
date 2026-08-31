"""Run a bounded headless simulation session from a resolved bundle."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from sim.runtime.control.fake import zero_output_components

from .coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from .mujoco_process import MujocoProcess


def main() -> int:
    """Execute one bounded Physics Runtime smoke session."""

    parser = argparse.ArgumentParser(
        description="Run a resolved LingTu simulation bundle with the C++ MuJoCo host."
    )
    parser.add_argument("bundle", type=Path, help="ResolvedSessionBundle directory")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, default=Path("runs/simulation"))
    parser.add_argument("--mujoco-host", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--run-id")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument(
        "--snapshot-out",
        type=Path,
        help="write the advanced SimulationSnapshot as truth-snapshot JSON",
    )
    args = parser.parse_args()

    coordinator = RuntimeCoordinator(
        bundle_dir=args.bundle,
        repo_root=args.repo_root,
        run_root=args.run_root,
        physics_host=MujocoProcess(args.mujoco_host),
        controller_factory=zero_output_components,
        run_id=args.run_id,
    )
    try:
        ready = coordinator.prepare()
        coordinator.start()
        snapshot = coordinator.advance(args.steps)
        coordinator.pause()
        reset = coordinator.reset() if args.reset else None
        coordinator.stop()
    except (CoordinatorError, OSError, ValueError) as exc:
        if coordinator.state in {
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }:
            try:
                coordinator.stop()
            except CoordinatorError:
                pass
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 1

    bodies = snapshot.get("bodies", [])
    if args.snapshot_out is not None:
        snapshot_path = args.snapshot_out.resolve()
        snapshot_path.parent.mkdir(parents=True, exist_ok=True)
        truth_snapshot = {
            "schema": "lingtu.sim.truth-snapshot.v1",
            **{key: value for key, value in snapshot.items() if key != "event"},
        }
        temporary = snapshot_path.with_suffix(snapshot_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(
                truth_snapshot,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(snapshot_path)
    else:
        snapshot_path = None
    result = {
        "ok": True,
        "session_id": ready["session_id"],
        "model_generation": ready["model_generation"],
        "physics_step": snapshot["physics_step"],
        "sim_time_ns": snapshot["sim_time_ns"],
        "body_count": len(bodies),
        "stable_ids": [body["stable_id"] for body in bodies],
        "reset_generation": (
            reset["reset_generation"] if reset is not None else snapshot["reset_generation"]
        ),
        "manifest": str(coordinator.manifest_path),
        "snapshot": str(snapshot_path) if snapshot_path is not None else None,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
