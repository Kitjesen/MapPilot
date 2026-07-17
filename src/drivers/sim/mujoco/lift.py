"""ROS-free MuJoCo lift adapter for building navigation tests and demos."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

from runtime.contracts.building import LiftState

FloorSpec = tuple[str, float]


@dataclass(frozen=True)
class MuJoCoLiftConfig:
    """Immutable MuJoCo lift joint and floor configuration."""

    lift_id: str = "lift-a"
    floors: tuple[FloorSpec, ...] = (("floor-1", 0.0), ("floor-2", 3.5))
    cabin_joint_name: str = "lift_cabin_z"
    left_door_joint_name: str = "lift_door_left_slide"
    right_door_joint_name: str = "lift_door_right_slide"
    left_door_closed_qpos: float = 0.0
    right_door_closed_qpos: float = 0.0
    left_door_open_qpos: float = -0.65
    right_door_open_qpos: float = 0.65
    cabin_speed_mps: float = 0.8
    door_speed_mps: float = 0.65
    position_tolerance_m: float = 1e-6
    _floor_positions: MappingProxyType[str, float] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        """Validate config and cache floor positions."""

        lift_id = str(self.lift_id).strip()
        if not lift_id:
            raise ValueError("lift_id is required")
        floors = tuple((str(floor_id).strip(), float(position)) for floor_id, position in self.floors)
        if not floors:
            raise ValueError("at least one floor is required")
        floor_positions: dict[str, float] = {}
        for floor_id, position in floors:
            if not floor_id:
                raise ValueError("floor_id is required")
            if floor_id in floor_positions:
                raise ValueError(f"duplicate floor_id: {floor_id}")
            if not math.isfinite(position):
                raise ValueError(f"invalid floor position for {floor_id}")
            floor_positions[floor_id] = position
        if float(self.cabin_speed_mps) <= 0.0 or not math.isfinite(float(self.cabin_speed_mps)):
            raise ValueError("cabin_speed_mps must be finite and positive")
        if float(self.door_speed_mps) <= 0.0 or not math.isfinite(float(self.door_speed_mps)):
            raise ValueError("door_speed_mps must be finite and positive")
        if float(self.position_tolerance_m) < 0.0 or not math.isfinite(float(self.position_tolerance_m)):
            raise ValueError("position_tolerance_m must be finite and non-negative")

        object.__setattr__(self, "lift_id", lift_id)
        object.__setattr__(self, "floors", floors)
        object.__setattr__(self, "_floor_positions", MappingProxyType(floor_positions))

    @property
    def floor_positions(self) -> dict[str, float]:
        """Return a copy of configured floor z positions by floor id."""

        return dict(self._floor_positions)


@dataclass(frozen=True)
class _JointBinding:
    name: str
    qpos_addr: int
    qvel_addr: int


class MuJoCoLiftCommandAdapter:
    """Deterministic MuJoCo implementation of ``LiftCommandPort``."""

    def __init__(
        self,
        engine: Any,
        *,
        config: MuJoCoLiftConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._engine = engine
        self.config = config or MuJoCoLiftConfig()
        self._clock = clock
        self._lock = threading.RLock()
        self._model = self._require_engine_attr("_model")
        self._data = self._require_engine_attr("_data")
        self._data_lock = getattr(engine, "_data_lock", threading.RLock())
        self._mujoco = self._import_mujoco()
        self._cabin = self._bind_slide_joint(self.config.cabin_joint_name)
        self._left_door = self._bind_slide_joint(self.config.left_door_joint_name)
        self._right_door = self._bind_slide_joint(self.config.right_door_joint_name)
        self._available = True
        self._session_id = ""
        self._destination_floor_id = self._nearest_floor_id(self._read_qpos(self._cabin))
        self._current_floor_id = self._destination_floor_id
        self._door_state = "closed"
        self._motion_state = "stopped"
        self._phase = "idle"
        self._write_joint_positions(
            cabin=self.config._floor_positions[self._current_floor_id],
            left_door=self.config.left_door_closed_qpos,
            right_door=self.config.right_door_closed_qpos,
        )

    def request(
        self,
        *,
        lift_id: str,
        destination_floor_id: str,
        session_id: str,
    ) -> tuple[bool, str]:
        """Claim or renew one session and request a destination floor."""

        with self._lock:
            reason = self._validate_request(lift_id, destination_floor_id, session_id)
            if reason:
                return False, reason
            if self._session_id and self._session_id != session_id:
                return False, "lift_session_busy"
            if self._motion_state == "moving" and destination_floor_id != self._destination_floor_id:
                return False, "lift_moving"

            self._session_id = session_id
            self._destination_floor_id = destination_floor_id
            if self._current_floor_id == destination_floor_id and self._motion_state == "stopped":
                self._phase = "opening"
                self._door_state = "opening"
            elif self._motion_state != "moving":
                self._phase = "closing"
                self._door_state = "closing"
            return True, "lift_request_accepted"

    def release(self, *, lift_id: str, session_id: str) -> tuple[bool, str]:
        """Release the claimed lift session only from a stable idle state."""

        with self._lock:
            if lift_id != self.config.lift_id:
                return False, "unknown_lift_id"
            if not self._session_id:
                return True, "lift_not_claimed"
            if session_id != self._session_id:
                return False, "lift_session_mismatch"
            if self._motion_state == "moving":
                return False, "lift_moving"
            if self._phase != "idle":
                return False, "lift_transition_in_progress"
            self._session_id = ""
            self._destination_floor_id = self._current_floor_id
            return True, "lift_released"

    def snapshot(self, lift_id: str) -> LiftState | None:
        """Return a fresh transport-neutral lift state."""

        with self._lock:
            if lift_id != self.config.lift_id:
                return None
            return LiftState(
                lift_id=self.config.lift_id,
                available=self._available,
                current_floor_id=self._current_floor_id,
                destination_floor_id=self._destination_floor_id,
                door_state=self._door_state,
                motion_state=self._motion_state,
                session_id=self._session_id,
                stamp_s=float(self._clock()),
            )

    def step(self, dt_s: float) -> LiftState:
        """Advance deterministic door/cabin kinematics by ``dt_s`` seconds."""

        if not math.isfinite(float(dt_s)) or float(dt_s) < 0.0:
            raise ValueError("dt_s must be finite and non-negative")
        with self._lock:
            if not self._available or dt_s == 0.0:
                state = self.snapshot(self.config.lift_id)
                if state is None:
                    raise RuntimeError("configured lift disappeared")
                return state
            remaining_s = float(dt_s)
            while remaining_s > 0.0:
                phase_before = self._phase
                consumed_s = self._step_phase(remaining_s)
                if consumed_s <= 0.0:
                    if self._phase != phase_before:
                        continue
                    break
                remaining_s = max(0.0, remaining_s - consumed_s)
            state = self.snapshot(self.config.lift_id)
            if state is None:
                raise RuntimeError("configured lift disappeared")
            return state

    def set_available(self, available: bool) -> None:
        """Inject or clear an availability fault for deterministic tests."""

        with self._lock:
            self._available = bool(available)

    def _step_phase(self, dt_s: float) -> float:
        if self._phase == "opening":
            return self._advance_doors(
                dt_s,
                left_target=self.config.left_door_open_qpos,
                right_target=self.config.right_door_open_qpos,
                next_phase="idle",
                final_door_state="open",
            )
        if self._phase == "closing":
            return self._advance_doors(
                dt_s,
                left_target=self.config.left_door_closed_qpos,
                right_target=self.config.right_door_closed_qpos,
                next_phase="moving",
                final_door_state="closed",
            )
        if self._phase == "moving":
            return self._advance_cabin(dt_s)
        return 0.0

    def _advance_doors(
        self,
        dt_s: float,
        *,
        left_target: float,
        right_target: float,
        next_phase: str,
        final_door_state: str,
    ) -> float:
        left = self._read_qpos(self._left_door)
        right = self._read_qpos(self._right_door)
        left_next, left_done = self._move_toward(left, left_target, self.config.door_speed_mps * dt_s)
        right_next, right_done = self._move_toward(right, right_target, self.config.door_speed_mps * dt_s)
        self._write_joint_positions(left_door=left_next, right_door=right_next)
        if left_done and right_done:
            self._door_state = final_door_state
            self._phase = next_phase
            if next_phase == "moving":
                self._motion_state = "moving"
            if next_phase == "idle":
                self._motion_state = "stopped"
            return self._max_joint_time(left, left_target, right, right_target, self.config.door_speed_mps)
        return dt_s

    def _advance_cabin(self, dt_s: float) -> float:
        target = self.config._floor_positions[self._destination_floor_id]
        current = self._read_qpos(self._cabin)
        next_qpos, done = self._move_toward(current, target, self.config.cabin_speed_mps * dt_s)
        self._write_joint_positions(cabin=next_qpos)
        if done:
            self._current_floor_id = self._destination_floor_id
            self._motion_state = "stopped"
            self._phase = "opening"
            self._door_state = "opening"
            return abs(target - current) / float(self.config.cabin_speed_mps)
        return dt_s

    def _validate_request(self, lift_id: str, destination_floor_id: str, session_id: str) -> str:
        if lift_id != self.config.lift_id:
            return "unknown_lift_id"
        if destination_floor_id not in self.config._floor_positions:
            return "unknown_floor_id"
        if not str(session_id).strip():
            return "lift_session_required"
        if not self._available:
            return "lift_unavailable"
        return ""

    def _write_joint_positions(
        self,
        *,
        cabin: float | None = None,
        left_door: float | None = None,
        right_door: float | None = None,
    ) -> None:
        changed = False
        with self._data_lock:
            for binding, value in (
                (self._cabin, cabin),
                (self._left_door, left_door),
                (self._right_door, right_door),
            ):
                if value is None:
                    continue
                self._data.qpos[binding.qpos_addr] = float(value)
                if 0 <= binding.qvel_addr < len(self._data.qvel):
                    self._data.qvel[binding.qvel_addr] = 0.0
                changed = True
            if changed:
                self._mujoco.mj_forward(self._model, self._data)

    def _read_qpos(self, binding: _JointBinding) -> float:
        with self._data_lock:
            return float(self._data.qpos[binding.qpos_addr])

    def _bind_slide_joint(self, name: str) -> _JointBinding:
        joint_id = self._mujoco.mj_name2id(self._model, self._mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"MuJoCo lift joint not found: {name}")
        joint_type = int(self._model.jnt_type[joint_id])
        if joint_type != int(self._mujoco.mjtJoint.mjJNT_SLIDE):
            raise ValueError(f"MuJoCo lift joint must be a slide joint: {name}")
        return _JointBinding(
            name=name,
            qpos_addr=int(self._model.jnt_qposadr[joint_id]),
            qvel_addr=int(self._model.jnt_dofadr[joint_id]),
        )

    def _nearest_floor_id(self, cabin_qpos: float) -> str:
        return min(
            self.config._floor_positions,
            key=lambda floor_id: abs(self.config._floor_positions[floor_id] - cabin_qpos),
        )

    def _require_engine_attr(self, name: str) -> Any:
        value = getattr(self._engine, name, None)
        if value is None:
            raise ValueError(f"MuJoCo engine missing {name}")
        return value

    @staticmethod
    def _import_mujoco() -> Any:
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("mujoco is required for MuJoCoLiftCommandAdapter") from exc
        return mujoco

    @staticmethod
    def _move_toward(current: float, target: float, max_delta: float) -> tuple[float, bool]:
        if abs(target - current) <= max_delta:
            return float(target), True
        direction = 1.0 if target > current else -1.0
        return float(current + direction * max_delta), False

    @staticmethod
    def _max_joint_time(
        left: float,
        left_target: float,
        right: float,
        right_target: float,
        speed: float,
    ) -> float:
        distances: Sequence[float] = (abs(left_target - left), abs(right_target - right))
        return max(distances) / float(speed)
