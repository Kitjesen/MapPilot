"""Validated, immutable inputs for the simulation Controller Runtime."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence

_TOP_LEVEL_KEYS = {
    "schema",
    "session_id",
    "env",
    "backends",
    "controllers",
    "command_channels",
    "stale_stop_authority",
}
_CONTROLLER_KEYS = {
    "instance_id",
    "controller_id",
    "package",
    "adapter",
    "policy",
    "timing",
    "state_channels",
    "command_channels",
    "actuator_channels",
}
_COMMAND_CHANNEL_KEYS = {
    "channel_id",
    "direction",
    "owner",
    "source",
    "transport",
    "message_type",
    "command_type",
    "target",
}


class ControllerPlanError(ValueError):
    """Raised when a compiled control plan is not safe to execute."""


@dataclass(frozen=True)
class ActuatorLayout:
    """Stable actuator order declared by one controller plan."""

    channels: tuple[str, ...]
    _indices: Mapping[str, int] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not self.channels:
            raise ControllerPlanError("actuator channels must not be empty")
        indices = {channel: index for index, channel in enumerate(self.channels)}
        if len(indices) != len(self.channels):
            raise ControllerPlanError("actuator channels must be unique")
        object.__setattr__(self, "_indices", MappingProxyType(indices))

    def index_of(self, channel: str) -> int:
        """Return the plan-stable index for one actuator channel."""

        return self._indices[channel]

    def order(self, values: Mapping[str, Any] | Sequence[Any]) -> tuple[Any, ...]:
        """Bind named or already ordered values to the immutable plan layout."""

        if isinstance(values, Mapping):
            missing = set(self.channels) - set(values)
            extra = set(values) - set(self.channels)
            if missing or extra:
                raise ValueError(
                    f"actuator values do not match the plan; missing={sorted(missing)!r}, extra={sorted(extra)!r}"
                )
            return tuple(values[channel] for channel in self.channels)
        if isinstance(values, (str, bytes)):
            raise ValueError("actuator values must be a mapping or numeric sequence")
        ordered = tuple(values)
        if len(ordered) != len(self.channels):
            raise ValueError(f"expected {len(self.channels)} actuator values, got {len(ordered)}")
        return ordered


@dataclass(frozen=True)
class CommandChannelSpec:
    """One command channel referenced by controller declarations."""

    channel_id: str
    direction: str
    owner: str
    source: str
    transport: str
    message_type: str
    command_type: str
    target: str


@dataclass(frozen=True)
class AdapterSpec:
    """Adapter plugin identity selected by the compiled plan."""

    plugin: str
    abi: str


@dataclass(frozen=True)
class PolicySpec:
    """Policy artifact identity retained without loading its runtime."""

    runtime: str
    artifact: str
    manifest: str


@dataclass(frozen=True)
class ControllerSpec:
    """The runtime-facing portion of one compiled controller declaration."""

    controller_id: str
    instance_id: str
    adapter: AdapterSpec
    policy: PolicySpec
    inference_hz: float
    low_level_hz: float
    state_channels: tuple[str, ...]
    command_channels: tuple[str, ...]
    actuators: ActuatorLayout


@dataclass(frozen=True)
class ControlPlan:
    """Immutable controller declarations for one resolved session."""

    session_id: str
    controllers: tuple[ControllerSpec, ...]
    command_channels: tuple[CommandChannelSpec, ...]
    stale_timeout_ns: int

    def controller(self, controller_id: str) -> ControllerSpec:
        """Select exactly one controller by its stable plan ID."""

        for controller in self.controllers:
            if controller.controller_id == controller_id:
                return controller
        raise KeyError(controller_id)

    def command_channel(self, channel_id: str) -> CommandChannelSpec:
        """Select one command-channel declaration by its stable ID."""

        for channel in self.command_channels:
            if channel.channel_id == channel_id:
                return channel
        raise KeyError(channel_id)


def _object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise ControllerPlanError(f"control plan contains duplicate key {key!r}")
        value[key] = item
    return value


def _object(value: Any, field_name: str, keys: set[str]) -> dict[str, Any]:
    if type(value) is not dict:
        raise ControllerPlanError(f"{field_name} must be an object")
    actual = set(value)
    if actual != keys:
        missing = sorted(keys - actual)
        extra = sorted(actual - keys)
        raise ControllerPlanError(f"{field_name} has invalid keys; missing={missing!r}, extra={extra!r}")
    return value


def _text(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ControllerPlanError(f"{field_name} must be a non-empty trimmed string")
    return value


def _strings(value: Any, field_name: str, *, nonempty: bool) -> tuple[str, ...]:
    if not isinstance(value, list) or (nonempty and not value):
        qualifier = "a non-empty" if nonempty else "a"
        raise ControllerPlanError(f"{field_name} must be {qualifier} list")
    result = tuple(_text(item, f"{field_name}[{index}]") for index, item in enumerate(value))
    if len(set(result)) != len(result):
        raise ControllerPlanError(f"{field_name} must contain unique values")
    return result


def _rate(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ControllerPlanError(f"{field_name} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ControllerPlanError(f"{field_name} must be finite and positive")
    return result


def _literal(value: Any, expected: Any, field_name: str) -> None:
    if type(value) is not type(expected) or value != expected:
        raise ControllerPlanError(f"{field_name} must be {expected!r}")


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_object_pairs,
            parse_constant=lambda constant: (_ for _ in ()).throw(
                ControllerPlanError(f"control plan contains non-finite value {constant}")
            ),
        )
    except ControllerPlanError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ControllerPlanError(f"cannot read {path}: {exc}") from exc
    if type(value) is not dict:
        raise ControllerPlanError("control plan must contain a JSON object")
    return value


def _parse_command_channels(raw: Any) -> tuple[CommandChannelSpec, ...]:
    if not isinstance(raw, list):
        raise ControllerPlanError("command_channels must be a list")
    channels: list[CommandChannelSpec] = []
    channel_ids: set[str] = set()
    for index, value in enumerate(raw):
        field = f"command_channels[{index}]"
        item = _object(value, field, _COMMAND_CHANNEL_KEYS)
        channel_id = _text(item["channel_id"], f"{field}.channel_id")
        if channel_id in channel_ids:
            raise ControllerPlanError(f"duplicate command channel {channel_id!r}")
        channel_ids.add(channel_id)
        direction = _text(item["direction"], f"{field}.direction")
        owner = _text(item["owner"], f"{field}.owner")
        transport = _text(item["transport"], f"{field}.transport")
        command_type = _text(item["command_type"], f"{field}.command_type")
        target = _text(item["target"], f"{field}.target")
        if direction not in {"publish", "subscribe"}:
            raise ControllerPlanError(f"{field}.direction is unsupported")
        if owner not in {"physics", "simulation", "native_service"}:
            raise ControllerPlanError(f"{field}.owner is unsupported")
        if transport not in {"typed_dds", "in_process"}:
            raise ControllerPlanError(f"{field}.transport is unsupported")
        if command_type not in {
            "joint_position",
            "joint_velocity",
            "joint_torque",
            "base_twist",
            "stop",
        }:
            raise ControllerPlanError(f"{field}.command_type is unsupported")
        if target not in {"actuators", "base", "safety"}:
            raise ControllerPlanError(f"{field}.target is unsupported")
        channels.append(
            CommandChannelSpec(
                channel_id=channel_id,
                direction=direction,
                owner=owner,
                source=_text(item["source"], f"{field}.source"),
                transport=transport,
                message_type=_text(item["message_type"], f"{field}.message_type"),
                command_type=command_type,
                target=target,
            )
        )
    return tuple(channels)


def _parse_controller(value: Any, index: int) -> ControllerSpec:
    field = f"controllers[{index}]"
    item = _object(value, field, _CONTROLLER_KEYS)
    package = _object(
        item["package"],
        f"{field}.package",
        {"id", "version", "kind", "manifest"},
    )
    _literal(package["kind"], "controller", f"{field}.package.kind")
    for key in ("id", "version", "manifest"):
        _text(package[key], f"{field}.package.{key}")

    adapter = _object(item["adapter"], f"{field}.adapter", {"plugin", "abi"})
    policy = _object(
        item["policy"],
        f"{field}.policy",
        {
            "runtime",
            "artifact",
            "manifest",
        },
    )
    policy_runtime = _text(policy["runtime"], f"{field}.policy.runtime")
    policy_artifact = _text(policy["artifact"], f"{field}.policy.artifact")
    policy_manifest = _text(policy["manifest"], f"{field}.policy.manifest")
    timing = _object(item["timing"], f"{field}.timing", {"inference_hz", "low_level_hz"})
    return ControllerSpec(
        controller_id=_text(item["controller_id"], f"{field}.controller_id"),
        instance_id=_text(item["instance_id"], f"{field}.instance_id"),
        adapter=AdapterSpec(
            plugin=_text(adapter["plugin"], f"{field}.adapter.plugin"),
            abi=_text(adapter["abi"], f"{field}.adapter.abi"),
        ),
        policy=PolicySpec(
            runtime=policy_runtime,
            artifact=policy_artifact,
            manifest=policy_manifest,
        ),
        inference_hz=_rate(timing["inference_hz"], f"{field}.timing.inference_hz"),
        low_level_hz=_rate(timing["low_level_hz"], f"{field}.timing.low_level_hz"),
        state_channels=_strings(item["state_channels"], f"{field}.state_channels", nonempty=False),
        command_channels=_strings(item["command_channels"], f"{field}.command_channels", nonempty=True),
        actuators=ActuatorLayout(
            _strings(
                item["actuator_channels"],
                f"{field}.actuator_channels",
                nonempty=True,
            )
        ),
    )


def load_control_plan(path: Path) -> ControlPlan:
    """Load and strictly validate one compiled control plan."""

    plan_path = Path(path)
    raw = _object(_load_json(plan_path), "control plan", _TOP_LEVEL_KEYS)
    _literal(raw["schema"], "lingtu.sim.control-plan.v1", "control plan.schema")
    _literal(raw["env"], "sim", "control plan.env")
    backends = _object(raw["backends"], "control plan.backends", {"physics", "visual"})
    _literal(backends["physics"], "mujoco", "control plan.backends.physics")
    if backends["visual"] not in {"unreal", None}:
        raise ControllerPlanError("control plan.backends.visual is unsupported")

    command_channels = _parse_command_channels(raw["command_channels"])
    known_channels = {channel.channel_id for channel in command_channels}
    raw_controllers = raw["controllers"]
    if not isinstance(raw_controllers, list):
        raise ControllerPlanError("controllers must be a list")
    controllers = tuple(_parse_controller(value, index) for index, value in enumerate(raw_controllers))
    controller_ids = [controller.controller_id for controller in controllers]
    if len(controller_ids) != len(set(controller_ids)):
        raise ControllerPlanError("controller_id values must be unique")
    for controller in controllers:
        unknown = set(controller.command_channels) - known_channels
        if unknown:
            raise ControllerPlanError(
                f"controller {controller.controller_id!r} references unknown command channels {sorted(unknown)!r}"
            )

    stale = _object(
        raw["stale_stop_authority"],
        "control plan.stale_stop_authority",
        {
            "owner",
            "hardware_forwarding",
            "safe_stop_on_stale",
            "stale_timeout_ms",
        },
    )
    _literal(stale["owner"], "simulation", "stale_stop_authority.owner")
    _literal(
        stale["hardware_forwarding"],
        False,
        "stale_stop_authority.hardware_forwarding",
    )
    _literal(
        stale["safe_stop_on_stale"],
        True,
        "stale_stop_authority.safe_stop_on_stale",
    )
    timeout_ms = stale["stale_timeout_ms"]
    if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or timeout_ms < 1:
        raise ControllerPlanError("stale_stop_authority.stale_timeout_ms must be a positive integer")
    return ControlPlan(
        session_id=_text(raw["session_id"], "control plan.session_id"),
        controllers=controllers,
        command_channels=command_channels,
        stale_timeout_ns=timeout_ms * 1_000_000,
    )
