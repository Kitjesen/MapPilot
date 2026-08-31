"""Run and qualify the package-local OmniCart controller against MuJoCo."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import runpy
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

_SCRIPT_PATH = Path(__file__).resolve()
_SCRIPT_DIR = _SCRIPT_PATH.parent
_REPO_ROOT = _SCRIPT_PATH.parents[4]
sys.path = [
    entry
    for entry in sys.path
    if Path(entry or Path.cwd()).resolve() != _SCRIPT_DIR
]
sys.path.insert(0, str(_REPO_ROOT / "src"))
sys.path.insert(0, str(_REPO_ROOT))
_CONTROLLER_RUNTIME = runpy.run_path(str(_SCRIPT_PATH.with_name("runtime.py")))
create_components = _CONTROLLER_RUNTIME["create_components"]

from sim.runtime.control import ControllerRuntimeError, load_control_plan
from sim.runtime.coordinator import CoordinatorError, RuntimeCoordinator
from sim.runtime.coordinator.controlled_run import (
    BaseTwist,
    resolve_base_twist_target,
    run_base_twist_session,
)
from sim.runtime.coordinator.mujoco_process import MujocoProcess

EVIDENCE_FILENAME = "omni-cart-control-evidence.json"
EVIDENCE_SCHEMA = "lingtu.sim.omni-cart-control-evidence.v1"


def _entity(values: Any, stable_id: str, field: str) -> Mapping[str, Any]:
    if not isinstance(values, list):
        raise ControllerRuntimeError(f"snapshot {field} must be a list")
    matches = [
        value
        for value in values
        if isinstance(value, Mapping) and value.get("stable_id") == stable_id
    ]
    if len(matches) != 1:
        raise ControllerRuntimeError(
            f"snapshot must contain exactly one {field} entry for {stable_id!r}"
        )
    return matches[0]


def _finite_vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise ControllerRuntimeError(f"{field} must contain exactly {size} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ControllerRuntimeError(f"{field} must contain finite values")
    return result


def _canonical_bytes(document: Mapping[str, Any]) -> bytes:
    return json.dumps(
        document,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _write_evidence(path: Path, document: Mapping[str, Any]) -> Path:
    destination = Path(path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    body = dict(document)
    body["evidence_sha256"] = hashlib.sha256(_canonical_bytes(document)).hexdigest()
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            body,
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


def run_headless_evidence(
    *,
    bundle_dir: Path,
    repo_root: Path,
    run_root: Path,
    mujoco_host: Path,
    run_id: str,
    twist: BaseTwist,
    steps: int,
    refresh_steps: int,
    minimum_displacement_m: float,
    minimum_speed_mps: float,
) -> tuple[Path, dict[str, Any]]:
    """Execute one bounded run and persist content-bound motion evidence."""

    if minimum_displacement_m < 0.0 or not math.isfinite(minimum_displacement_m):
        raise ValueError("minimum_displacement_m must be finite and non-negative")
    if minimum_speed_mps < 0.0 or not math.isfinite(minimum_speed_mps):
        raise ValueError("minimum_speed_mps must be finite and non-negative")
    bundle = Path(bundle_dir).resolve()
    root = Path(repo_root).resolve()
    plan = load_control_plan(bundle / "control.plan.json")
    if len(plan.controllers) != 1:
        raise ControllerRuntimeError(
            f"OmniCart evidence runner requires one controller, found {len(plan.controllers)}"
        )
    controller = plan.controllers[0]
    if (
        controller.adapter.plugin != "differential_drive_wheel_torque"
        or controller.policy.runtime != "analytic"
    ):
        raise ControllerRuntimeError("bundle does not select the OmniCart differential-drive controller")
    output_channels = [
        plan.command_channel(channel_id)
        for channel_id in controller.command_channels
        if plan.command_channel(channel_id).direction == "publish"
        and plan.command_channel(channel_id).target == "actuators"
    ]
    if len(output_channels) != 1:
        raise ControllerRuntimeError("OmniCart controller must have one actuator output channel")

    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=root,
        run_root=run_root,
        physics_host=MujocoProcess(Path(mujoco_host).resolve()),
        controller_factory=create_components,
        run_id=run_id,
    )
    target = resolve_base_twist_target(bundle, controller.controller_id)
    snapshot = run_base_twist_session(
        coordinator,
        target,
        twist,
        steps=steps,
        refresh_steps=refresh_steps,
    )

    robot = next(
        (item for item in coordinator.plan.robots if item.instance_id == controller.instance_id),
        None,
    )
    if robot is None:
        raise ControllerRuntimeError("PhysicsPlan does not contain the controlled OmniCart instance")
    initial_position = tuple(float(item) for item in robot.position_m)
    base_id = f"{controller.instance_id}/{robot.attach_root}"
    base = _entity(snapshot.get("bodies"), base_id, "bodies")
    final_position = _finite_vector(base.get("position_m"), 3, f"{base_id}.position_m")
    base_velocity = _finite_vector(
        base.get("linear_velocity_mps"),
        3,
        f"{base_id}.linear_velocity_mps",
    )
    displacement_m = math.dist(initial_position, final_position)
    base_speed_mps = math.sqrt(sum(value * value for value in base_velocity))

    wheel_velocity: dict[str, float] = {}
    wheel_torque: dict[str, float] = {}
    for channel in controller.actuators.channels:
        stable_id = f"{controller.instance_id}/{channel}"
        joint = _entity(snapshot.get("joints"), stable_id, "joints")
        velocity = _finite_vector(
            joint.get("velocity_rps"),
            1,
            f"{stable_id}.velocity_rps",
        )
        actuator = _entity(snapshot.get("actuators"), stable_id, "actuators")
        torque = float(actuator.get("control"))
        if not math.isfinite(torque):
            raise ControllerRuntimeError(f"{stable_id}.control must be finite")
        wheel_velocity[stable_id] = velocity[0]
        wheel_torque[stable_id] = torque

    qualified = (
        displacement_m >= minimum_displacement_m
        and base_speed_mps >= minimum_speed_mps
        and any(abs(value) > 0.01 for value in wheel_torque.values())
    )
    document: dict[str, Any] = {
        "schema": EVIDENCE_SCHEMA,
        "session_id": plan.session_id,
        "run_id": run_id,
        "controller": {
            "controller_id": controller.controller_id,
            "adapter_plugin": controller.adapter.plugin,
            "policy_runtime": controller.policy.runtime,
            "output_command_type": output_channels[0].command_type,
            "controller_model": "two_wheel_differential_drive",
            "holonomic": False,
            "linear_y_supported": False,
        },
        "command": twist.payload(),
        "thresholds": {
            "minimum_displacement_m": minimum_displacement_m,
            "minimum_speed_mps": minimum_speed_mps,
        },
        "result": {
            "physics_step": snapshot["physics_step"],
            "sim_time_ns": snapshot["sim_time_ns"],
            "initial_position_m": list(initial_position),
            "final_position_m": list(final_position),
            "displacement_m": displacement_m,
            "base_linear_velocity_mps": list(base_velocity),
            "base_speed_mps": base_speed_mps,
            "wheel_velocity_rps": wheel_velocity,
            "wheel_torque_nm": wheel_torque,
        },
        "artifact_references": {
            "runtime_manifest": "session.runtime.json",
            "episode_result": "episode_result.json",
        },
        "qualified": qualified,
    }
    evidence_path = _write_evidence(
        coordinator.allocation.run_dir / EVIDENCE_FILENAME,
        document,
    )
    return evidence_path, {**document, "evidence_sha256": json.loads(evidence_path.read_text(encoding="utf-8"))["evidence_sha256"]}


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the OmniCart differential-drive ControllerPackage against native MuJoCo."
    )
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--mujoco-host", type=Path, required=True)
    parser.add_argument("--run-id", default="omni-cart-control")
    parser.add_argument("--steps", type=int, default=1_000)
    parser.add_argument("--refresh-steps", type=int, default=20)
    parser.add_argument("--linear-x", type=float, default=0.4)
    parser.add_argument("--linear-y", type=float, default=0.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument("--minimum-displacement-m", type=float, default=0.1)
    parser.add_argument("--minimum-speed-mps", type=float, default=0.05)
    return parser.parse_args(list(argv))


def main(argv: Sequence[str] | None = None) -> int:
    """Run the reproducible OmniCart headless qualification command."""

    args = _parse_args(sys.argv[1:] if argv is None else argv)
    try:
        evidence_path, evidence = run_headless_evidence(
            bundle_dir=args.bundle,
            repo_root=args.repo_root,
            run_root=args.run_root,
            mujoco_host=args.mujoco_host,
            run_id=args.run_id,
            twist=BaseTwist(
                linear_x=args.linear_x,
                linear_y=args.linear_y,
                angular_z=args.angular_z,
            ),
            steps=args.steps,
            refresh_steps=args.refresh_steps,
            minimum_displacement_m=args.minimum_displacement_m,
            minimum_speed_mps=args.minimum_speed_mps,
        )
    except (CoordinatorError, ControllerRuntimeError, OSError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "ok": evidence["qualified"],
                "evidence": str(evidence_path),
                "session_id": evidence["session_id"],
                "displacement_m": evidence["result"]["displacement_m"],
                "base_speed_mps": evidence["result"]["base_speed_mps"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if evidence["qualified"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
