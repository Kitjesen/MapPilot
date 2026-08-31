"""Analytic two-wheel differential-drive adapter for the generic Controller Runtime."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sim.runtime.control import (
    ActuatorLayout,
    ControllerCommand,
    ControllerRuntimeError,
    ControllerSpec,
    ControllerState,
    GenerationStamp,
)

_ACTUATORS = ("left_wheel_joint", "right_wheel_joint")
_CONFIG_KEYS = {
    "schema",
    "wheel_radius_m",
    "track_width_m",
    "speed_error_gain_nm_per_rps",
    "torque_limit_nm",
    "linear_velocity_limit_mps",
    "angular_velocity_limit_rps",
    "wheel_speed_limit_rps",
    "lateral_velocity_supported",
}


@dataclass(frozen=True)
class _DriveConfig:
    wheel_radius_m: float
    track_width_m: float
    speed_error_gain_nm_per_rps: float
    torque_limit_nm: float
    linear_velocity_limit_mps: float
    angular_velocity_limit_rps: float
    wheel_speed_limit_rps: float


@dataclass(frozen=True)
class _DriveObservation:
    target_velocity_rps: tuple[float, float]


def _finite_positive(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ControllerRuntimeError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ControllerRuntimeError(f"{field} must be finite and positive")
    return result


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ControllerRuntimeError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ControllerRuntimeError(f"{field} must be finite")
    return result


def _finite_pair(value: Any, field: str) -> tuple[float, float]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != 2:
        raise ControllerRuntimeError(f"{field} must contain exactly two values")
    return (_finite(value[0], f"{field}[0]"), _finite(value[1], f"{field}[1]"))


def _verified_path(repo_root: Path, relative: str, field: str) -> Path:
    root = Path(repo_root).resolve()
    candidate = Path(relative)
    if candidate.is_absolute():
        raise ControllerRuntimeError(f"{field} must be relative to repo_root")
    path = (root / candidate).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ControllerRuntimeError(f"{field} escapes repo_root") from exc
    if not path.is_file():
        raise ControllerRuntimeError(f"{field} is not a file: {path}")
    return path


def _load_config(controller: ControllerSpec, repo_root: Path) -> _DriveConfig:
    artifact = _verified_path(
        repo_root,
        controller.policy.artifact,
        "differential-drive policy artifact",
    )
    _verified_path(
        repo_root,
        controller.policy.manifest,
        "differential-drive policy manifest",
    )
    try:
        raw = json.loads(
            artifact.read_text(encoding="utf-8"),
            parse_constant=lambda value: (_ for _ in ()).throw(
                ControllerRuntimeError(f"policy artifact contains non-finite value {value}")
            ),
        )
    except ControllerRuntimeError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ControllerRuntimeError(f"cannot read differential-drive policy artifact: {exc}") from exc
    if not isinstance(raw, dict) or set(raw) != _CONFIG_KEYS:
        raise ControllerRuntimeError("differential-drive policy artifact has invalid keys")
    if raw["schema"] != "lingtu.sim.differential-drive-analytic-policy.v1":
        raise ControllerRuntimeError("differential-drive policy artifact schema is unsupported")
    if raw["lateral_velocity_supported"] is not False:
        raise ControllerRuntimeError("differential-drive policy must reject lateral velocity")
    return _DriveConfig(
        wheel_radius_m=_finite_positive(raw["wheel_radius_m"], "wheel_radius_m"),
        track_width_m=_finite_positive(raw["track_width_m"], "track_width_m"),
        speed_error_gain_nm_per_rps=_finite_positive(
            raw["speed_error_gain_nm_per_rps"],
            "speed_error_gain_nm_per_rps",
        ),
        torque_limit_nm=_finite_positive(raw["torque_limit_nm"], "torque_limit_nm"),
        linear_velocity_limit_mps=_finite_positive(
            raw["linear_velocity_limit_mps"],
            "linear_velocity_limit_mps",
        ),
        angular_velocity_limit_rps=_finite_positive(
            raw["angular_velocity_limit_rps"],
            "angular_velocity_limit_rps",
        ),
        wheel_speed_limit_rps=_finite_positive(
            raw["wheel_speed_limit_rps"],
            "wheel_speed_limit_rps",
        ),
    )


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class _DifferentialDriveAdapter:
    def __init__(self, controller: ControllerSpec, config: _DriveConfig) -> None:
        if controller.adapter.plugin != "differential_drive_wheel_torque":
            raise ControllerRuntimeError("OmniCart controller adapter plugin is unsupported")
        if controller.adapter.abi != "lingtu.sim.controller-adapter.v1":
            raise ControllerRuntimeError("OmniCart controller adapter ABI is unsupported")
        if controller.policy.runtime != "analytic":
            raise ControllerRuntimeError("OmniCart controller policy runtime must be analytic")
        if controller.actuators.channels != _ACTUATORS:
            raise ControllerRuntimeError(
                "OmniCart differential-drive actuator order must be left wheel then right wheel"
            )
        required_state = {"joint_position", "joint_velocity"}
        if not required_state.issubset(controller.state_channels):
            raise ControllerRuntimeError("OmniCart controller state contract is incomplete")
        self._actuators = controller.actuators.channels
        self._config = config

    def observe(
        self,
        state: ControllerState,
        command: ControllerCommand,
        actuators: ActuatorLayout,
    ) -> _DriveObservation:
        self._validate_actuators(actuators)
        if not isinstance(command.payload, Mapping):
            raise ControllerRuntimeError("base_twist command payload must be a mapping")
        linear_x = _clamp(
            _finite(command.payload.get("linear_x", 0.0), "base_twist.linear_x"),
            self._config.linear_velocity_limit_mps,
        )
        linear_y = _finite(command.payload.get("linear_y", 0.0), "base_twist.linear_y")
        if abs(linear_y) > 1e-9:
            raise ControllerRuntimeError(
                "OmniCart differential drive does not support non-zero linear_y"
            )
        angular_z = _clamp(
            _finite(command.payload.get("angular_z", 0.0), "base_twist.angular_z"),
            self._config.angular_velocity_limit_rps,
        )
        half_track = self._config.track_width_m * 0.5
        target_left = (linear_x - angular_z * half_track) / self._config.wheel_radius_m
        target_right = (linear_x + angular_z * half_track) / self._config.wheel_radius_m
        return _DriveObservation(
            (
                _clamp(target_left, self._config.wheel_speed_limit_rps),
                _clamp(target_right, self._config.wheel_speed_limit_rps),
            )
        )

    def actuate(
        self,
        state: ControllerState,
        action: Any,
        actuators: ActuatorLayout,
    ) -> Mapping[str, float]:
        self._validate_actuators(actuators)
        target = _finite_pair(action, "differential-drive target wheel velocity")
        velocity = _finite_pair(state.channels.get("joint_velocity"), "joint_velocity")
        torques = tuple(
            _clamp(
                self._config.speed_error_gain_nm_per_rps * (target_rps - actual_rps),
                self._config.torque_limit_nm,
            )
            for target_rps, actual_rps in zip(target, velocity)
        )
        return dict(zip(self._actuators, torques))

    def safe_stop(
        self,
        state: ControllerState,
        actuators: ActuatorLayout,
    ) -> Mapping[str, float]:
        del state
        self._validate_actuators(actuators)
        return {channel: 0.0 for channel in self._actuators}

    def reset(self, generation: GenerationStamp) -> None:
        del generation

    def _validate_actuators(self, actuators: ActuatorLayout) -> None:
        if not isinstance(actuators, ActuatorLayout) or actuators.channels != self._actuators:
            raise ControllerRuntimeError(
                "runtime actuator layout does not match the compiled OmniCart controller"
            )


class _AnalyticDifferentialDrivePolicy:
    def infer(self, observation: Any) -> tuple[float, float]:
        if not isinstance(observation, _DriveObservation):
            raise ControllerRuntimeError("analytic differential-drive observation is invalid")
        return observation.target_velocity_rps

    def reset(self, generation: GenerationStamp) -> None:
        del generation


def create_components(controller: ControllerSpec, repo_root: Path) -> tuple[Any, Any]:
    """Create package-local components for the generic Controller Runtime seam."""

    if not isinstance(controller, ControllerSpec):
        raise TypeError("controller must be a compiled ControllerSpec")
    config = _load_config(controller, repo_root)
    return _DifferentialDriveAdapter(controller, config), _AnalyticDifferentialDrivePolicy()
