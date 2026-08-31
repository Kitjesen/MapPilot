"""Bounded loopback delivery for the first live Physics-to-Visual slice.

The transport is deliberately presentation-only.  It does not replace typed
DDS contracts or camera shared memory, and it never becomes a physics clock
authority.  RuntimeCoordinator advances MuJoCo; this module only serializes
validated immutable snapshots and drops them onto localhost with latest-wins
semantics.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from sim.runtime.control.fake import zero_output_components

from .coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from .mujoco_process import MujocoProcess

TRUTH_SNAPSHOT_SCHEMA = "lingtu.sim.truth-snapshot.v1"
MAX_LOOPBACK_DATAGRAM_BYTES = 60_000


class SnapshotPublisher(Protocol):
    """One-way latest-value snapshot publisher."""

    def publish(self, event: Mapping[str, Any]) -> int:
        """Publish one coordinator-validated snapshot and return its byte size."""


def truth_snapshot_document(event: Mapping[str, Any]) -> dict[str, Any]:
    """Project one coordinator event onto the immutable snapshot wire document."""

    if event.get("event") != "snapshot":
        raise CoordinatorError("live visual transport accepts snapshot events only")
    required = (
        "session_id",
        "model_generation",
        "reset_generation",
        "sequence",
        "physics_step",
        "sim_time_ns",
        "bodies",
    )
    missing = [field for field in required if field not in event]
    if missing:
        raise CoordinatorError(
            "snapshot event is missing required field(s): " + ", ".join(missing)
        )
    return {
        "schema": TRUTH_SNAPSHOT_SCHEMA,
        **{key: value for key, value in event.items() if key != "event"},
    }


def encode_truth_snapshot(event: Mapping[str, Any]) -> bytes:
    """Encode one snapshot as strict compact UTF-8 JSON."""

    try:
        payload = json.dumps(
            truth_snapshot_document(event),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise CoordinatorError(f"snapshot is not JSON-serializable: {exc}") from exc
    if len(payload) > MAX_LOOPBACK_DATAGRAM_BYTES:
        raise CoordinatorError(
            "truth snapshot exceeds the first-slice loopback datagram limit: "
            f"{len(payload)} > {MAX_LOOPBACK_DATAGRAM_BYTES}"
        )
    return payload


class UdpLoopbackSnapshotPublisher:
    """Non-blocking localhost UDP publisher for RobotSimUE presentation state."""

    def __init__(
        self,
        port: int,
        *,
        host: str = "127.0.0.1",
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        if host != "127.0.0.1":
            raise ValueError("the first live visual slice is restricted to 127.0.0.1")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("snapshot port must be an integer from 1 to 65535")
        self._destination = (host, port)
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)

    def publish(self, event: Mapping[str, Any]) -> int:
        """Send one whole immutable snapshot or explicitly drop it when busy."""

        payload = encode_truth_snapshot(event)
        try:
            sent = self._socket.sendto(payload, self._destination)
        except BlockingIOError:
            return 0
        if sent != len(payload):
            raise CoordinatorError(
                f"loopback snapshot datagram was truncated: {sent} of {len(payload)} bytes"
            )
        return sent

    def close(self) -> None:
        """Release the UDP socket."""

        self._socket.close()

    def __enter__(self) -> UdpLoopbackSnapshotPublisher:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def stream_live_snapshots(
    coordinator: RuntimeCoordinator,
    publisher: SnapshotPublisher,
    *,
    steps_per_frame: int = 8,
    frame_limit: int | None = None,
    pace: bool = True,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Run one bounded or interruptible live presentation stream.

    Pacing follows MuJoCo simulation time.  Snapshot loss never slows physics;
    UDP send pressure is treated as a dropped presentation frame.
    """

    if isinstance(steps_per_frame, bool) or not isinstance(steps_per_frame, int):
        raise ValueError("steps_per_frame must be a positive integer")
    if steps_per_frame <= 0:
        raise ValueError("steps_per_frame must be a positive integer")
    if frame_limit is not None and (
        isinstance(frame_limit, bool) or not isinstance(frame_limit, int) or frame_limit <= 0
    ):
        raise ValueError("frame_limit must be a positive integer when provided")

    coordinator.prepare()
    coordinator.start()
    frames = 0
    wall_origin: float | None = None
    sim_origin_ns: int | None = None
    try:
        while frame_limit is None or frames < frame_limit:
            snapshot = coordinator.advance(steps_per_frame)
            publisher.publish(snapshot)
            frames += 1

            if pace:
                sim_time_ns = snapshot.get("sim_time_ns")
                if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
                    raise CoordinatorError("snapshot sim_time_ns must be an integer")
                if wall_origin is None:
                    wall_origin = monotonic()
                    sim_origin_ns = sim_time_ns
                else:
                    if sim_origin_ns is None:
                        raise CoordinatorError("live pacing origin is unavailable")
                    deadline = wall_origin + (sim_time_ns - sim_origin_ns) / 1_000_000_000
                    remaining = deadline - monotonic()
                    if remaining > 0:
                        sleep(remaining)
    finally:
        if coordinator.state is RuntimeState.RUNNING:
            coordinator.pause()
        if coordinator.state in {
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }:
            coordinator.stop()
    return frames


def main() -> int:
    """Run the first localhost Physics-to-RobotSimUE live stream."""

    parser = argparse.ArgumentParser(
        description="Stream MuJoCo truth snapshots to RobotSimUE over localhost UDP."
    )
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, default=Path("runs/simulation"))
    parser.add_argument("--mujoco-host", type=Path, required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--run-id")
    parser.add_argument("--steps-per-frame", type=int, default=8)
    parser.add_argument("--frames", type=int)
    parser.add_argument("--no-pace", action="store_true")
    args = parser.parse_args()

    coordinator = RuntimeCoordinator(
        bundle_dir=args.bundle,
        repo_root=args.repo_root,
        run_root=args.run_root,
        physics_host=MujocoProcess(args.mujoco_host),
        controller_factory=zero_output_components,
        run_id=args.run_id,
        ports={"visual_snapshot_udp": args.port},
    )
    try:
        with UdpLoopbackSnapshotPublisher(args.port) as publisher:
            frames = stream_live_snapshots(
                coordinator,
                publisher,
                steps_per_frame=args.steps_per_frame,
                frame_limit=args.frames,
                pace=not args.no_pace,
            )
    except KeyboardInterrupt:
        return 130
    except (CoordinatorError, OSError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 1

    print(
        json.dumps(
            {
                "ok": True,
                "frames": frames,
                "transport": "udp_loopback_json_v1",
                "port": args.port,
                "manifest": str(coordinator.manifest_path),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
