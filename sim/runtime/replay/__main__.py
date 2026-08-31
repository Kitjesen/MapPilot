"""Local-only CLI for presenting a validated recording to RobotSimUE."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from sim.runtime.coordinator import CoordinatorError
from sim.runtime.coordinator.live_snapshot import UdpLoopbackSnapshotPublisher

from .timeline import SimulationReplay, SimulationReplayError, replay_snapshots


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay validated simulation truth to RobotSimUE over localhost."
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--no-pace", action="store_true")
    return parser


def _coordinator_event(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    if snapshot.get("schema") != "lingtu.sim.truth-snapshot.v1":
        raise SimulationReplayError("replay snapshot schema is invalid")
    return {
        "event": "snapshot",
        **{key: value for key, value in snapshot.items() if key != "schema"},
    }


def main(argv: list[str] | None = None) -> int:
    """Validate and present one recording without advancing simulation time."""

    args = _parser().parse_args(argv)
    try:
        replay = SimulationReplay.open(args.recording)
        with UdpLoopbackSnapshotPublisher(args.port) as publisher:
            report = replay_snapshots(
                replay,
                lambda snapshot: publisher.publish(_coordinator_event(snapshot)),
                pace=not args.no_pace,
                rate=args.rate,
            )
    except (CoordinatorError, OSError, SimulationReplayError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "ok": True,
                "run_id": replay.run_id,
                "session_id": report.session_id,
                "frames_presented": report.frames_presented,
                "frames_dropped": report.frames_dropped,
                "duration_ns": report.duration_ns,
                "clock_authority": "recorded_mujoco",
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
