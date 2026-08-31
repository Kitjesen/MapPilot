"""Reusable bounded runner for generation-stamped base-twist control."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Protocol

from sim.runtime.control import (
    CommandSubmitResult,
    ControllerCommand,
    GenerationStamp,
    create_production_components,
    load_control_plan,
)
from sim.runtime.control.fake import zero_output_components
from sim.runtime.sensors import (
    SensorEndpointFactory,
    SensorReadiness,
    TruthOdometryEndpointFactory,
)

from .coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from .mujoco_process import MujocoProcess


def _identifier(value: str, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field} must be a non-empty trimmed string")
    return value


def _finite(value: float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


@dataclass(frozen=True)
class BaseTwistTarget:
    """The exact compiled command endpoint for one controller instance."""

    controller_id: str
    instance_id: str
    channel_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "controller_id", _identifier(self.controller_id, "controller_id"))
        object.__setattr__(self, "instance_id", _identifier(self.instance_id, "instance_id"))
        object.__setattr__(self, "channel_id", _identifier(self.channel_id, "channel_id"))


@dataclass(frozen=True)
class BaseTwist:
    """Finite planar velocity intent consumed by a base-twist channel."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "linear_x", _finite(self.linear_x, "linear_x"))
        object.__setattr__(self, "linear_y", _finite(self.linear_y, "linear_y"))
        object.__setattr__(self, "angular_z", _finite(self.angular_z, "angular_z"))

    def payload(self) -> dict[str, float]:
        """Return the typed command payload expected by controller adapters."""

        return {
            "linear_x": self.linear_x,
            "linear_y": self.linear_y,
            "angular_z": self.angular_z,
        }


class ControlledCoordinator(Protocol):
    @property
    def state(self) -> RuntimeState: ...

    def prepare(self) -> dict[str, Any]: ...

    def start(self) -> dict[str, Any]: ...

    def submit_controller_command(self, controller_id: str, command: ControllerCommand) -> CommandSubmitResult: ...

    def advance(self, steps: int = 1) -> dict[str, Any]: ...

    def pause(self) -> dict[str, Any]: ...

    def stop(self) -> dict[str, Any]: ...


def _optional_truth_odometry_factory(
    executable: Path | None,
    *,
    parent_frame: str,
) -> SensorEndpointFactory | None:
    """Create the exact native truth endpoint only when explicitly enabled."""

    if executable is None:
        return None
    return TruthOdometryEndpointFactory(
        executable=executable,
        parent_frame=parent_frame,
    )


def _truth_odometry_states(readiness: SensorReadiness) -> dict[str, str]:
    """Project observed truth-stream qualification without inferring activity."""

    return {
        stream_id: qualification.state.value
        for stream_id, qualification in sorted(readiness.streams.items())
        if qualification.stream_kind == "truth_odom"
    }


def _event_integer(event: Mapping[str, Any], field: str) -> int:
    value = event.get(field)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CoordinatorError(f"runtime event {field} must be a non-negative integer")
    return value


def run_base_twist_session(
    coordinator: ControlledCoordinator,
    target: BaseTwistTarget,
    twist: BaseTwist,
    *,
    steps: int,
    refresh_steps: int,
) -> dict[str, Any]:
    """Run a bounded closed loop and refresh its command before it can go stale."""

    if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
        raise ValueError("steps must be a positive integer")
    if isinstance(refresh_steps, bool) or not isinstance(refresh_steps, int) or refresh_steps <= 0:
        raise ValueError("refresh_steps must be a positive integer")

    current = coordinator.prepare()
    if coordinator.state is not RuntimeState.READY:
        try:
            coordinator.stop()
        finally:
            raise CoordinatorError(f"controlled session is not READY after prepare (state={coordinator.state.value})")

    remaining = steps
    command_sequence = 0
    snapshot: dict[str, Any] | None = None
    try:
        coordinator.start()
        while remaining > 0:
            command_sequence += 1
            generation = GenerationStamp(
                _event_integer(current, "model_generation"),
                _event_integer(current, "reset_generation"),
            )
            result = coordinator.submit_controller_command(
                target.controller_id,
                ControllerCommand(
                    channel_id=target.channel_id,
                    instance_id=target.instance_id,
                    generation=generation,
                    sequence=command_sequence,
                    apply_time_ns=_event_integer(current, "sim_time_ns"),
                    payload=twist.payload(),
                ),
            )
            if result is not CommandSubmitResult.ACCEPTED:
                raise CoordinatorError(f"controller rejected refreshed base-twist command: {result.value}")
            chunk = min(remaining, refresh_steps)
            snapshot = coordinator.advance(chunk)
            current = snapshot
            remaining -= chunk
    finally:
        if coordinator.state is RuntimeState.RUNNING:
            coordinator.pause()
        if coordinator.state in {
            RuntimeState.PREPARING,
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }:
            coordinator.stop()

    if snapshot is None:
        raise CoordinatorError("controlled session did not produce a snapshot")
    return snapshot


def resolve_base_twist_target(bundle_dir: Path, controller_id: str | None) -> BaseTwistTarget:
    """Resolve exactly one subscribed base-twist endpoint from a compiled bundle."""

    plan = load_control_plan(Path(bundle_dir) / "control.plan.json")
    controllers = (
        tuple(item for item in plan.controllers if item.controller_id == controller_id)
        if controller_id is not None
        else plan.controllers
    )
    if len(controllers) != 1:
        raise CoordinatorError(f"controlled runner requires exactly one selected controller; found {len(controllers)}")
    controller = controllers[0]
    channels = tuple(
        plan.command_channel(channel_id)
        for channel_id in controller.command_channels
        if plan.command_channel(channel_id).command_type == "base_twist"
        and plan.command_channel(channel_id).direction == "subscribe"
    )
    if len(channels) != 1:
        raise CoordinatorError(
            f"controller {controller.controller_id!r} must declare exactly one subscribed base_twist channel"
        )
    return BaseTwistTarget(
        controller_id=controller.controller_id,
        instance_id=controller.instance_id,
        channel_id=channels[0].channel_id,
    )


_target_from_bundle = resolve_base_twist_target


def _write_snapshot(path: Path, snapshot: Mapping[str, Any]) -> Path:
    destination = Path(path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "schema": "lingtu.sim.truth-snapshot.v1",
        **{key: value for key, value in snapshot.items() if key != "event"},
    }
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    return destination


def main() -> int:
    """Run a bounded production or qualification-only controlled session."""

    parser = argparse.ArgumentParser(description="Run a generation-safe base-twist Controller→MuJoCo session.")
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, default=Path("runs/simulation"))
    parser.add_argument("--mujoco-host", type=Path, required=True)
    parser.add_argument("--run-id")
    parser.add_argument("--dds-domain", type=int, default=0)
    parser.add_argument("--controller-id")
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--refresh-steps", type=int, default=25)
    parser.add_argument("--linear-x", type=float, default=0.0)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--snapshot-out", type=Path, required=True)
    parser.add_argument(
        "--truth-odom-publisher",
        type=Path,
        help="native CycloneDDS truth-odometry publisher executable",
    )
    parser.add_argument(
        "--truth-odom-parent-frame",
        default="world",
        help="parent frame for the simulation-only truth odometry stream",
    )
    parser.add_argument(
        "--components",
        choices=("production", "zero-output"),
        default="production",
        help="production loads the package policy; zero-output is qualification-only",
    )
    args = parser.parse_args()

    component_factory = create_production_components if args.components == "production" else zero_output_components
    coordinator = RuntimeCoordinator(
        bundle_dir=args.bundle,
        repo_root=args.repo_root,
        run_root=args.run_root,
        physics_host=MujocoProcess(args.mujoco_host),
        controller_factory=component_factory,
        sensor_endpoint_factory=_optional_truth_odometry_factory(
            args.truth_odom_publisher,
            parent_frame=args.truth_odom_parent_frame,
        ),
        run_id=args.run_id,
        dds_domain=args.dds_domain,
    )
    try:
        target = resolve_base_twist_target(args.bundle, args.controller_id)
        snapshot = run_base_twist_session(
            coordinator,
            target,
            BaseTwist(args.linear_x, args.linear_y, args.angular_z),
            steps=args.steps,
            refresh_steps=args.refresh_steps,
        )
        snapshot_path = _write_snapshot(args.snapshot_out, snapshot)
    except (CoordinatorError, OSError, RuntimeError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 1

    try:
        truth_odometry = _truth_odometry_states(coordinator.sensor_readiness)
    except CoordinatorError:
        truth_odometry = {}

    print(
        json.dumps(
            {
                "ok": True,
                "session_id": snapshot["session_id"],
                "model_generation": snapshot["model_generation"],
                "reset_generation": snapshot["reset_generation"],
                "physics_step": snapshot["physics_step"],
                "sim_time_ns": snapshot["sim_time_ns"],
                "snapshot": str(snapshot_path),
                "manifest": str(coordinator.manifest_path),
                "components": args.components,
                "truth_odometry": truth_odometry,
                "dds_domain": args.dds_domain,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


__all__ = [
    "BaseTwist",
    "BaseTwistTarget",
    "resolve_base_twist_target",
    "run_base_twist_session",
]


if __name__ == "__main__":
    raise SystemExit(main())
