"""Generation-safe routing for deterministic scenario effects."""

from __future__ import annotations

import threading
from collections.abc import Mapping
from dataclasses import replace
from typing import Any, Protocol

from .runtime import (
    DISPATCH_AUTHORITIES,
    EntitySnapshot,
    GenerationStamp,
    ScenarioSnapshot,
)


class ScenarioDispatchError(RuntimeError):
    """Raised when a scenario effect cannot be routed without ambiguity."""


class ScenarioPhysicsSink(Protocol):
    """Process-safe sink for scenario-owned MuJoCo kinematic proxies."""

    def apply_kinematic_poses(
        self,
        snapshot: ScenarioSnapshot,
    ) -> Mapping[str, Any] | None:
        """Apply one complete physical batch without exposing MuJoCo internals."""


class ScenarioVisualSink(Protocol):
    """Presentation sink for scenario and UE-animation entity transforms."""

    def apply_visual_entities(
        self,
        snapshot: ScenarioSnapshot,
    ) -> Mapping[str, Any] | None:
        """Apply one complete visual batch using the original scenario stamp."""


class ScenarioSensorSink(Protocol):
    """Optional sink for generation-stamped dynamic sensor-scene input."""

    def apply_sensor_entities(
        self,
        snapshot: ScenarioSnapshot,
    ) -> Mapping[str, Any] | None:
        """Apply physical scenario entities without becoming readiness authority."""


class CompositeScenarioDispatcher:
    """Validate one scenario stream and route immutable authority-specific views.

    The dispatcher commits its generation/sequence cursor only after every
    configured sink accepts the batch. Sink processes still own their local
    transactional application and must reject stale generations independently.
    """

    def __init__(
        self,
        *,
        session_id: str,
        initial_generation: GenerationStamp,
        physics_sink: ScenarioPhysicsSink | None = None,
        visual_sink: ScenarioVisualSink | None = None,
        sensor_sink: ScenarioSensorSink | None = None,
    ) -> None:
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError("session_id must be non-empty")
        self._session_id = session_id
        self._initial_generation = initial_generation
        self._physics_sink = physics_sink
        self._visual_sink = visual_sink
        self._sensor_sink = sensor_sink
        self._active_generation: GenerationStamp | None = None
        self._last_sequence: int | None = None
        self._lock = threading.Lock()

    @property
    def updates_physics(self) -> bool:
        """Return whether this dispatcher applies MuJoCo kinematic proxies."""

        return self._physics_sink is not None

    @property
    def active_generation(self) -> GenerationStamp | None:
        """Return the generation accepted by every configured sink."""

        with self._lock:
            return self._active_generation

    @property
    def last_sequence(self) -> int | None:
        """Return the last sequence accepted by every configured sink."""

        with self._lock:
            return self._last_sequence

    def dispatch(self, snapshot: ScenarioSnapshot) -> None:
        """Validate and apply one complete, ordered scenario snapshot."""

        with self._lock:
            generation = self._validate_stamp(snapshot)
            physical, visual, sensor = self._route_entities(snapshot)
            calls: list[
                tuple[
                    str,
                    ScenarioPhysicsSink | ScenarioVisualSink | ScenarioSensorSink,
                    str,
                    tuple[EntitySnapshot, ...],
                ]
            ] = []
            if self._physics_sink is not None and physical:
                calls.append(
                    ("physics", self._physics_sink, "apply_kinematic_poses", physical)
                )
            if self._visual_sink is not None and visual:
                calls.append(
                    ("visual", self._visual_sink, "apply_visual_entities", visual)
                )
            if self._sensor_sink is not None and sensor:
                calls.append(
                    ("sensor", self._sensor_sink, "apply_sensor_entities", sensor)
                )
            if not calls:
                raise ScenarioDispatchError("scenario snapshot has no routed sink effects")

            for sink_name, sink, method_name, entities in calls:
                routed = replace(snapshot, entities=entities)
                self._apply_sink(
                    sink_name=sink_name,
                    sink=sink,
                    method_name=method_name,
                    snapshot=routed,
                )

            self._active_generation = generation
            self._last_sequence = snapshot.sequence

    def _validate_stamp(self, snapshot: ScenarioSnapshot) -> GenerationStamp:
        if snapshot.session_id != self._session_id:
            raise ScenarioDispatchError("scenario snapshot session_id does not match the dispatcher session")
        if isinstance(snapshot.sequence, bool) or not isinstance(snapshot.sequence, int):
            raise ScenarioDispatchError("scenario snapshot sequence must be an integer")
        if snapshot.sequence < 0:
            raise ScenarioDispatchError("scenario snapshot sequence must be non-negative")
        if isinstance(snapshot.sim_time_ns, bool) or not isinstance(snapshot.sim_time_ns, int):
            raise ScenarioDispatchError("scenario snapshot sim_time_ns must be an integer")
        if snapshot.sim_time_ns < 0:
            raise ScenarioDispatchError("scenario snapshot sim_time_ns must be non-negative")
        try:
            generation = GenerationStamp(
                snapshot.model_generation,
                snapshot.reset_generation,
            )
        except ValueError as exc:
            raise ScenarioDispatchError(f"scenario snapshot generation is invalid: {exc}") from exc

        active = self._active_generation
        if active is None:
            if generation != self._initial_generation:
                raise ScenarioDispatchError("scenario initial generation does not match the dispatcher generation")
            if snapshot.sequence != 0:
                raise ScenarioDispatchError(
                    f"scenario sequence is {snapshot.sequence}, expected 0 for initial generation"
                )
            return generation

        if generation.model_generation != active.model_generation:
            direction = "backward" if generation.model_generation < active.model_generation else "unexpected"
            raise ScenarioDispatchError(f"scenario model generation moved {direction}")
        if generation.reset_generation < active.reset_generation:
            raise ScenarioDispatchError("scenario generation moved backward")
        if generation.reset_generation > active.reset_generation:
            if generation.reset_generation != active.reset_generation + 1:
                raise ScenarioDispatchError("scenario reset generation skipped an epoch")
            if snapshot.sequence != 0:
                raise ScenarioDispatchError(
                    f"scenario sequence is {snapshot.sequence}, expected 0 for reset generation"
                )
            return generation

        expected_sequence = (self._last_sequence if self._last_sequence is not None else -1) + 1
        if snapshot.sequence != expected_sequence:
            raise ScenarioDispatchError(f"scenario sequence is {snapshot.sequence}, expected {expected_sequence}")
        return generation

    def _route_entities(
        self,
        snapshot: ScenarioSnapshot,
    ) -> tuple[
        tuple[EntitySnapshot, ...],
        tuple[EntitySnapshot, ...],
        tuple[EntitySnapshot, ...],
    ]:
        if not snapshot.entities:
            raise ScenarioDispatchError("scenario snapshot contains no entities")
        seen: set[str] = set()
        physical: list[EntitySnapshot] = []
        visual: list[EntitySnapshot] = []
        sensor: list[EntitySnapshot] = []
        for entity in snapshot.entities:
            if entity.entity_id in seen:
                raise ScenarioDispatchError(f"scenario snapshot contains duplicate entity_id {entity.entity_id!r}")
            seen.add(entity.entity_id)
            if entity.authority not in DISPATCH_AUTHORITIES:
                raise ScenarioDispatchError(f"{entity.entity_id}: unsupported dispatch authority {entity.authority!r}")
            if entity.authority == "ue_animation":
                if self._visual_sink is None:
                    raise ScenarioDispatchError(f"visual sink is required for {entity.entity_id!r}")
                visual.append(entity)
                continue

            has_kinematic_proxy = entity.physics_proxy_mode == "kinematic"
            if has_kinematic_proxy:
                if (
                    entity.body_stable_id is None
                    or not entity.body_stable_id.startswith(f"{entity.entity_id}/")
                    or entity.body_stable_id == f"{entity.entity_id}/"
                ):
                    raise ScenarioDispatchError(
                        f"{entity.entity_id}: body_stable_id must belong to the scenario entity"
                    )
                if self._physics_sink is None:
                    raise ScenarioDispatchError(f"physics sink is required for {entity.entity_id!r}")
                physical.append(entity)
                if self._sensor_sink is not None:
                    sensor.append(entity)
            elif self._visual_sink is None:
                raise ScenarioDispatchError(
                    f"visual sink is required for nonphysical scenario entity {entity.entity_id!r}"
                )

            if self._visual_sink is not None:
                visual.append(entity)

        return tuple(physical), tuple(visual), tuple(sensor)

    @staticmethod
    def _apply_sink(
        *,
        sink_name: str,
        sink: ScenarioPhysicsSink | ScenarioVisualSink | ScenarioSensorSink,
        method_name: str,
        snapshot: ScenarioSnapshot,
    ) -> None:
        method = getattr(sink, method_name, None)
        if not callable(method):
            raise ScenarioDispatchError(f"{sink_name} sink does not implement {method_name}")
        try:
            result = method(snapshot)
        except Exception as exc:
            message = str(exc).strip()
            reason = f"{type(exc).__name__}: {message}" if message else type(exc).__name__
            raise ScenarioDispatchError(
                f"{sink_name} sink failed at model_generation={snapshot.model_generation} "
                f"reset_generation={snapshot.reset_generation} sequence={snapshot.sequence}: {reason}"
            ) from exc
        if result is not None and not isinstance(result, Mapping):
            raise ScenarioDispatchError(
                f"{sink_name} sink returned a non-mapping acknowledgement at "
                f"reset_generation={snapshot.reset_generation} sequence={snapshot.sequence}"
            )
