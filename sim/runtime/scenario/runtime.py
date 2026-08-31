"""Deterministic runtime view for compiled scenario plans."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Protocol, cast

SCENARIO_PLAN_SCHEMA = "lingtu.sim.scenario-plan.v1"
MUJOCO_CLOCK_SOURCE = "mujoco_sim_time"
SUPPORTED_AUTHORITIES = frozenset({"mujoco", "scenario", "ue_animation"})
DISPATCH_AUTHORITIES = frozenset({"scenario", "ue_animation"})
SUPPORTED_BEHAVIOR_PROFILES = frozenset({"linear_crossing"})
PLAN_FIELDS = frozenset(
    {
        "schema",
        "session_id",
        "env",
        "backend",
        "package",
        "model_generation",
        "reset_generation",
        "seed",
        "clock",
        "authority_policy",
        "entities",
    }
)
PACKAGE_FIELDS = frozenset({"id", "version", "kind", "manifest"})
PACKAGE_REQUIRED_FIELDS = frozenset({"id", "version"})
CLOCK_FIELDS = frozenset({"unit", "source", "sim_time_ns"})
AUTHORITY_POLICY_FIELDS = frozenset({"robot_physics_owner", "dynamic_behavior_owner", "visual_animation_owner"})
ENTITY_FIELDS = frozenset(
    {
        "entity_id",
        "entity_type",
        "authority",
        "source_epoch",
        "initial_transform",
        "physics_proxy",
        "semantic_class",
        "behavior",
    }
)
TRANSFORM_FIELDS = frozenset({"position_m", "quaternion_wxyz"})
BEHAVIOR_FIELDS = frozenset({"profile", "seed", "parameters"})
LINEAR_CROSSING_FIELDS = frozenset({"start_time_s", "duration_s", "speed_mps", "end_position_m"})
PHYSICS_PROXY_FIELDS = frozenset({"mode", "body_stable_id"})


class ScenarioPlanError(ValueError):
    """Raised when a scenario plan or runtime clock cannot be accepted."""


class ScenarioDispatcher(Protocol):
    """Side-effect seam that applies generation-stamped scenario state."""

    def dispatch(self, snapshot: ScenarioSnapshot) -> None:
        """Apply one ordered scenario snapshot."""


@dataclass(frozen=True)
class GenerationStamp:
    """Model/reset generation identity for scenario snapshots."""

    model_generation: int
    reset_generation: int

    def __post_init__(self) -> None:
        _nonnegative_int(self.model_generation, "model_generation")
        _nonnegative_int(self.reset_generation, "reset_generation")


@dataclass(frozen=True)
class ScenarioClock:
    """Authoritative MuJoCo simulation clock sample."""

    session_id: str
    model_generation: int
    reset_generation: int
    sim_time_ns: int
    source: str = MUJOCO_CLOCK_SOURCE

    def __post_init__(self) -> None:
        _identifier(self.session_id, "session_id")
        _nonnegative_int(self.model_generation, "model_generation")
        _nonnegative_int(self.reset_generation, "reset_generation")
        _nonnegative_int(self.sim_time_ns, "sim_time_ns")
        if self.source != MUJOCO_CLOCK_SOURCE:
            raise ScenarioPlanError("clock source must be mujoco_sim_time")


@dataclass(frozen=True)
class Transform:
    """One immutable pose in world coordinates."""

    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-compatible transform view."""

        return {
            "position_m": list(self.position_m),
            "quaternion_wxyz": list(self.quaternion_wxyz),
        }


@dataclass(frozen=True)
class LinearCrossing:
    """Deterministic straight-line behavior for scenario-authority entities."""

    start_time_s: float
    duration_s: float
    speed_mps: float
    end_position_m: tuple[float, float, float]


@dataclass(frozen=True)
class ScenarioEntity:
    """Runtime-owned immutable entity declaration."""

    entity_id: str
    entity_type: str
    authority: str
    source_epoch: int
    initial_transform: Transform
    physics_proxy_mode: str
    body_stable_id: str | None
    semantic_class: str
    behavior: LinearCrossing | None

    @property
    def physics_proxy(self) -> str:
        """Return the compatibility view of the physical proxy mode."""

        return self.physics_proxy_mode


@dataclass(frozen=True)
class EntitySnapshot:
    """Stable public state for one scenario entity at one sim-time sample."""

    entity_id: str
    transform: Transform
    authority: str
    source_epoch: int
    semantic_class: str
    motion_state: str
    physics_proxy_mode: str
    body_stable_id: str | None

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-compatible entity snapshot view."""

        return {
            "entity_id": self.entity_id,
            "transform": self.transform.to_dict(),
            "authority": self.authority,
            "source_epoch": self.source_epoch,
            "semantic_class": self.semantic_class,
            "motion_state": self.motion_state,
            "physics_proxy_mode": self.physics_proxy_mode,
            "body_stable_id": self.body_stable_id,
        }


@dataclass(frozen=True)
class ScenarioSnapshot:
    """Immutable generation-stamped scenario state at MuJoCo sim time."""

    session_id: str
    model_generation: int
    reset_generation: int
    sequence: int
    sim_time_ns: int
    entities: tuple[EntitySnapshot, ...]

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-compatible scenario snapshot view."""

        return {
            "schema": "lingtu.sim.scenario-snapshot.v1",
            "session_id": self.session_id,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "sequence": self.sequence,
            "sim_time_ns": self.sim_time_ns,
            "entities": [entity.to_dict() for entity in self.entities],
        }

    def for_dispatch(self) -> ScenarioSnapshot:
        """Return the ordered externally applicable subset of this snapshot."""

        return ScenarioSnapshot(
            session_id=self.session_id,
            model_generation=self.model_generation,
            reset_generation=self.reset_generation,
            sequence=self.sequence,
            sim_time_ns=self.sim_time_ns,
            entities=tuple(entity for entity in self.entities if entity.authority in DISPATCH_AUTHORITIES),
        )


@dataclass(frozen=True)
class ScenarioPlan:
    """Strict runtime view of a compiled scenario plan."""

    session_id: str
    generation: GenerationStamp
    initial_sim_time_ns: int
    entities: tuple[ScenarioEntity, ...]

    @property
    def requires_dispatch(self) -> bool:
        """Return whether the plan contains externally applied entities."""

        return any(entity.authority in DISPATCH_AUTHORITIES for entity in self.entities)

    @property
    def supports_mujoco_dispatch(self) -> bool:
        """Return whether every dispatched entity has an explicit MuJoCo proxy."""

        dispatched = tuple(entity for entity in self.entities if entity.authority in DISPATCH_AUTHORITIES)
        return bool(dispatched) and all(
            entity.authority == "scenario"
            and entity.physics_proxy_mode == "kinematic"
            and entity.body_stable_id is not None
            for entity in dispatched
        )

    @property
    def mujoco_kinematic_bindings(self) -> tuple[tuple[str, str], ...]:
        """Return stable entity/body bindings declared for MuJoCo dispatch."""

        return tuple(
            (entity.entity_id, entity.body_stable_id)
            for entity in self.entities
            if entity.physics_proxy_mode == "kinematic" and entity.body_stable_id is not None
        )


def load_scenario_plan(path: str | Path | None) -> ScenarioPlan | None:
    """Load an optional compiled scenario plan from strict JSON."""

    if path is None:
        return None
    plan_path = Path(path)
    try:
        data = json.loads(
            plan_path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_object_pairs,
            parse_constant=_reject_json_constant,
        )
    except ScenarioPlanError:
        raise
    except json.JSONDecodeError as exc:
        raise ScenarioPlanError(f"scenario plan JSON is invalid: {exc}") from exc
    except OSError as exc:
        raise ScenarioPlanError(f"scenario plan could not be read: {exc}") from exc
    return parse_scenario_plan(data)


class ScenarioRuntime:
    """Pure deterministic evaluator for scenario-authority dynamic entities."""

    def __init__(self, plan: ScenarioPlan | None) -> None:
        self._plan = plan
        self._last_generation: GenerationStamp | None = None
        self._last_sim_time_ns: int | None = None
        self._sequence = 0

    @property
    def requires_dispatch(self) -> bool:
        """Return whether evaluation results must cross the dispatcher seam."""

        return self._plan is not None and self._plan.requires_dispatch

    @property
    def supports_mujoco_dispatch(self) -> bool:
        """Return whether the runtime can use the built-in MuJoCo dispatcher."""

        return self._plan is not None and self._plan.supports_mujoco_dispatch

    @property
    def mujoco_kinematic_bindings(self) -> tuple[tuple[str, str], ...]:
        """Return the compiled MuJoCo entity/body routing contract."""

        return () if self._plan is None else self._plan.mujoco_kinematic_bindings

    @classmethod
    def from_plan(cls, data: Mapping[str, Any] | ScenarioPlan | None) -> ScenarioRuntime:
        """Build a runtime from an already-loaded strict plan mapping."""

        if isinstance(data, ScenarioPlan) or data is None:
            return cls(data)
        return cls(parse_scenario_plan(data))

    @classmethod
    def from_plan_file(cls, path: str | Path | None) -> ScenarioRuntime:
        """Build a runtime from an optional compiled scenario plan path."""

        return cls(load_scenario_plan(path))

    def snapshot(self, clock: ScenarioClock) -> ScenarioSnapshot:
        """Evaluate one immutable scenario snapshot at MuJoCo simulation time."""

        if self._plan is None:
            return self._empty_snapshot(clock)
        if clock.session_id != self._plan.session_id:
            raise ScenarioPlanError("clock session_id does not match scenario plan")
        generation = GenerationStamp(clock.model_generation, clock.reset_generation)
        self._accept_clock(generation, clock.sim_time_ns)
        entities = tuple(_snapshot_entity(entity, clock.sim_time_ns) for entity in self._plan.entities)
        snapshot = ScenarioSnapshot(
            session_id=clock.session_id,
            model_generation=generation.model_generation,
            reset_generation=generation.reset_generation,
            sequence=self._sequence,
            sim_time_ns=clock.sim_time_ns,
            entities=entities,
        )
        self._sequence += 1
        return snapshot

    def _empty_snapshot(self, clock: ScenarioClock) -> ScenarioSnapshot:
        generation = GenerationStamp(clock.model_generation, clock.reset_generation)
        self._accept_clock(generation, clock.sim_time_ns)
        snapshot = ScenarioSnapshot(
            session_id=clock.session_id,
            model_generation=generation.model_generation,
            reset_generation=generation.reset_generation,
            sequence=self._sequence,
            sim_time_ns=clock.sim_time_ns,
            entities=(),
        )
        self._sequence += 1
        return snapshot

    def _accept_clock(self, generation: GenerationStamp, sim_time_ns: int) -> None:
        if self._last_generation is None:
            self._last_generation = generation
            self._last_sim_time_ns = sim_time_ns
            self._sequence = 0
            return

        previous = self._last_generation
        if previous is None:
            raise ScenarioPlanError("scenario clock generation history is unavailable")
        if generation.model_generation < previous.model_generation:
            raise ScenarioPlanError("model generation moved backward")
        if (
            generation.model_generation == previous.model_generation
            and generation.reset_generation < previous.reset_generation
        ):
            raise ScenarioPlanError("reset generation moved backward")

        generation_changed = generation != previous
        if generation_changed:
            initial_sim_time_ns = 0 if self._plan is None else self._plan.initial_sim_time_ns
            if sim_time_ns != initial_sim_time_ns:
                raise ScenarioPlanError(
                    "scenario reset clock must return to the plan initial sim_time_ns"
                )
            self._last_generation = generation
            self._last_sim_time_ns = sim_time_ns
            self._sequence = 0
            return

        if self._last_sim_time_ns is None:
            raise ScenarioPlanError("scenario clock history is unavailable")
        if sim_time_ns < self._last_sim_time_ns:
            raise ScenarioPlanError("scenario clock moved backward")
        if sim_time_ns == self._last_sim_time_ns:
            raise ScenarioPlanError("scenario clock is stale")
        self._last_sim_time_ns = sim_time_ns


def parse_scenario_plan(data: Mapping[str, Any]) -> ScenarioPlan:
    """Parse a strict runtime view from a compiled scenario plan mapping."""

    if not isinstance(data, Mapping):
        raise ScenarioPlanError("scenario plan must be a JSON object")
    _reject_unknown_fields(data, PLAN_FIELDS, "scenario plan")
    _require_fields(data, PLAN_FIELDS, "scenario plan")
    if data["schema"] != SCENARIO_PLAN_SCHEMA:
        raise ScenarioPlanError("scenario plan schema must be lingtu.sim.scenario-plan.v1")
    if data["env"] != "sim":
        raise ScenarioPlanError("scenario plan env must be sim")
    if data["backend"] != "mujoco":
        raise ScenarioPlanError("scenario plan backend must be mujoco")
    _identifier(data["session_id"], "session_id")
    generation = GenerationStamp(
        _nonnegative_int(data["model_generation"], "model_generation"),
        _nonnegative_int(data["reset_generation"], "reset_generation"),
    )
    if generation != GenerationStamp(0, 0):
        raise ScenarioPlanError("scenario plan model/reset generation must start at zero")
    _nonnegative_int(data["seed"], "seed")
    _parse_package(data["package"])
    initial_sim_time_ns = _parse_clock(data["clock"])
    _parse_authority_policy(data["authority_policy"])
    entities = _parse_entities(data["entities"])
    return ScenarioPlan(
        session_id=data["session_id"],
        generation=generation,
        initial_sim_time_ns=initial_sim_time_ns,
        entities=entities,
    )


def _parse_package(value: Any) -> Mapping[str, str]:
    if not isinstance(value, Mapping):
        raise ScenarioPlanError("package must be an object")
    _reject_unknown_fields(value, PACKAGE_FIELDS, "package")
    _require_fields(value, PACKAGE_REQUIRED_FIELDS, "package")
    _identifier(value["id"], "package.id")
    _identifier(value["version"], "package.version")
    if "kind" in value and value["kind"] != "scenario":
        raise ScenarioPlanError("package.kind must be scenario")
    if "manifest" in value:
        _identifier(value["manifest"], "package.manifest")
    return MappingProxyType(dict(value))


def _parse_clock(value: Any) -> int:
    if not isinstance(value, Mapping):
        raise ScenarioPlanError("clock must be an object")
    _reject_unknown_fields(value, CLOCK_FIELDS, "clock")
    _require_fields(value, CLOCK_FIELDS, "clock")
    if value["unit"] != "ns":
        raise ScenarioPlanError("clock unit must be ns")
    if value["source"] != MUJOCO_CLOCK_SOURCE:
        raise ScenarioPlanError("clock source must be mujoco_sim_time")
    return _nonnegative_int(value["sim_time_ns"], "clock.sim_time_ns")


def _parse_authority_policy(value: Any) -> None:
    if not isinstance(value, Mapping):
        raise ScenarioPlanError("authority_policy must be an object")
    _reject_unknown_fields(value, AUTHORITY_POLICY_FIELDS, "authority_policy")
    _require_fields(value, AUTHORITY_POLICY_FIELDS, "authority_policy")
    expected = {
        "robot_physics_owner": "mujoco",
        "dynamic_behavior_owner": "scenario",
        "visual_animation_owner": "ue_animation",
    }
    for key, expected_value in expected.items():
        if value[key] != expected_value:
            raise ScenarioPlanError(f"authority_policy.{key} must be {expected_value}")


def _parse_entities(value: Any) -> tuple[ScenarioEntity, ...]:
    if not isinstance(value, list):
        raise ScenarioPlanError("entities must be a list")
    entities: list[ScenarioEntity] = []
    seen: set[str] = set()
    for index, item in enumerate(value):
        if isinstance(item, Mapping) and "entity_id" in item:
            candidate_id = _identifier(item["entity_id"], f"entities[{index}].entity_id")
            if candidate_id in seen:
                raise ScenarioPlanError(f"duplicate entity_id {candidate_id!r}")
        entity = _parse_entity(item, index)
        if entity.entity_id in seen:
            raise ScenarioPlanError(f"duplicate entity_id {entity.entity_id!r}")
        seen.add(entity.entity_id)
        entities.append(entity)
    return tuple(entities)


def _parse_entity(value: Any, index: int) -> ScenarioEntity:
    context = f"entities[{index}]"
    if not isinstance(value, Mapping):
        raise ScenarioPlanError(f"{context} must be an object")
    _reject_unknown_fields(value, ENTITY_FIELDS, context)
    required = ENTITY_FIELDS - {"behavior"}
    _require_fields(value, required, context)
    entity_id = _identifier(value["entity_id"], f"{context}.entity_id")
    entity_type = _identifier(value["entity_type"], f"{context}.entity_type")
    authority = _identifier(value["authority"], f"{context}.authority")
    if authority not in SUPPORTED_AUTHORITIES:
        raise ScenarioPlanError(f"{context}.authority is unsupported: {authority!r}")
    source_epoch = _nonnegative_int(value["source_epoch"], f"{context}.source_epoch")
    transform = _parse_transform(value["initial_transform"], f"{context}.initial_transform")
    physics_proxy_mode, body_stable_id = _parse_physics_proxy(
        value["physics_proxy"],
        entity_id,
        f"{context}.physics_proxy",
    )
    semantic_class = _identifier(value["semantic_class"], f"{context}.semantic_class")
    behavior = _parse_behavior(value["behavior"], context) if "behavior" in value else None

    if entity_type == "robot" and authority != "mujoco":
        raise ScenarioPlanError(f"{context}: robot entities must use mujoco authority")
    if authority == "mujoco" and entity_type != "robot":
        raise ScenarioPlanError(f"{context}: mujoco authority is supported only for robot entities")
    if authority != "scenario" and behavior is not None:
        raise ScenarioPlanError(f"{context}: behavior is compatible only with scenario authority")
    if authority == "scenario" and physics_proxy_mode == "mujoco":
        raise ScenarioPlanError(f"{context}: scenario authority must not claim mujoco physics_proxy")

    return ScenarioEntity(
        entity_id=entity_id,
        entity_type=entity_type,
        authority=authority,
        source_epoch=source_epoch,
        initial_transform=transform,
        physics_proxy_mode=physics_proxy_mode,
        body_stable_id=body_stable_id,
        semantic_class=semantic_class,
        behavior=behavior,
    )


def _parse_physics_proxy(value: Any, entity_id: str, context: str) -> tuple[str, str | None]:
    if isinstance(value, str):
        return _identifier(value, context), None
    if not isinstance(value, Mapping):
        raise ScenarioPlanError(f"{context} must be a string or object")
    _reject_unknown_fields(value, PHYSICS_PROXY_FIELDS, context)
    _require_fields(value, PHYSICS_PROXY_FIELDS, context)
    mode = _identifier(value["mode"], f"{context}.mode")
    if mode != "kinematic":
        raise ScenarioPlanError(f"{context}.mode must be kinematic")
    body_stable_id = _identifier(value["body_stable_id"], f"{context}.body_stable_id")
    if not body_stable_id.startswith(f"{entity_id}/") or body_stable_id == f"{entity_id}/":
        raise ScenarioPlanError(f"{context}.body_stable_id must belong to {entity_id!r}")
    return mode, body_stable_id


def _parse_transform(value: Any, context: str) -> Transform:
    if not isinstance(value, Mapping):
        raise ScenarioPlanError(f"{context} must be an object")
    _reject_unknown_fields(value, TRANSFORM_FIELDS, context)
    _require_fields(value, TRANSFORM_FIELDS, context)
    position_m = cast(
        tuple[float, float, float],
        _finite_tuple(value["position_m"], 3, f"{context}.position_m"),
    )
    quaternion_wxyz = cast(
        tuple[float, float, float, float],
        _finite_tuple(value["quaternion_wxyz"], 4, f"{context}.quaternion_wxyz"),
    )
    return Transform(
        position_m=position_m,
        quaternion_wxyz=quaternion_wxyz,
    )


def _parse_behavior(value: Any, context: str) -> LinearCrossing:
    if not isinstance(value, Mapping):
        raise ScenarioPlanError(f"{context}.behavior must be an object")
    _reject_unknown_fields(value, BEHAVIOR_FIELDS, f"{context}.behavior")
    _require_fields(value, BEHAVIOR_FIELDS, f"{context}.behavior")
    profile = _identifier(value["profile"], f"{context}.behavior.profile")
    if profile not in SUPPORTED_BEHAVIOR_PROFILES:
        raise ScenarioPlanError(f"{context}.behavior.profile is unsupported: {profile!r}")
    _nonnegative_int(value["seed"], f"{context}.behavior.seed")
    parameters = value["parameters"]
    if not isinstance(parameters, Mapping):
        raise ScenarioPlanError(f"{context}.behavior.parameters must be an object")
    _reject_unknown_fields(parameters, LINEAR_CROSSING_FIELDS, f"{context}.behavior.parameters")
    _require_fields(parameters, LINEAR_CROSSING_FIELDS, f"{context}.behavior.parameters")
    start_time_s = _finite_float(parameters["start_time_s"], f"{context}.behavior.parameters.start_time_s")
    duration_s = _positive_float(parameters["duration_s"], f"{context}.behavior.parameters.duration_s")
    speed_mps = _positive_float(parameters["speed_mps"], f"{context}.behavior.parameters.speed_mps")
    end_position_m = cast(
        tuple[float, float, float],
        _finite_tuple(
            parameters["end_position_m"],
            3,
            f"{context}.behavior.parameters.end_position_m",
        ),
    )
    return LinearCrossing(
        start_time_s=start_time_s,
        duration_s=duration_s,
        speed_mps=speed_mps,
        end_position_m=end_position_m,
    )


def _snapshot_entity(entity: ScenarioEntity, sim_time_ns: int) -> EntitySnapshot:
    if entity.authority == "mujoco":
        return EntitySnapshot(
            entity_id=entity.entity_id,
            transform=entity.initial_transform,
            authority=entity.authority,
            source_epoch=entity.source_epoch,
            semantic_class=entity.semantic_class,
            motion_state="mujoco_authority",
            physics_proxy_mode=entity.physics_proxy_mode,
            body_stable_id=entity.body_stable_id,
        )
    if entity.authority == "ue_animation":
        return EntitySnapshot(
            entity_id=entity.entity_id,
            transform=entity.initial_transform,
            authority=entity.authority,
            source_epoch=entity.source_epoch,
            semantic_class=entity.semantic_class,
            motion_state="intent_only",
            physics_proxy_mode=entity.physics_proxy_mode,
            body_stable_id=entity.body_stable_id,
        )
    if entity.behavior is None:
        return EntitySnapshot(
            entity_id=entity.entity_id,
            transform=entity.initial_transform,
            authority=entity.authority,
            source_epoch=entity.source_epoch,
            semantic_class=entity.semantic_class,
            motion_state="stationary",
            physics_proxy_mode=entity.physics_proxy_mode,
            body_stable_id=entity.body_stable_id,
        )
    transform, motion_state = _linear_crossing_transform(
        entity.initial_transform,
        entity.behavior,
        sim_time_ns,
    )
    return EntitySnapshot(
        entity_id=entity.entity_id,
        transform=transform,
        authority=entity.authority,
        source_epoch=entity.source_epoch,
        semantic_class=entity.semantic_class,
        motion_state=motion_state,
        physics_proxy_mode=entity.physics_proxy_mode,
        body_stable_id=entity.body_stable_id,
    )


def _linear_crossing_transform(
    initial: Transform,
    behavior: LinearCrossing,
    sim_time_ns: int,
) -> tuple[Transform, str]:
    sim_time_s = sim_time_ns / 1_000_000_000
    if sim_time_s < behavior.start_time_s:
        return initial, "pending"
    elapsed_s = sim_time_s - behavior.start_time_s
    fraction = min(1.0, elapsed_s / behavior.duration_s)
    start = initial.position_m
    end = behavior.end_position_m
    delta = tuple(end[index] - start[index] for index in range(3))
    distance = math.sqrt(sum(component * component for component in delta))
    max_distance = behavior.speed_mps * elapsed_s
    if distance > 0.0 and elapsed_s < behavior.duration_s:
        speed_fraction = min(1.0, max_distance / distance)
        fraction = min(fraction, speed_fraction)
    position = cast(
        tuple[float, float, float],
        tuple(start[index] + delta[index] * fraction for index in range(3)),
    )
    motion_state = "complete" if elapsed_s >= behavior.duration_s else "active"
    if motion_state == "complete":
        position = end
    return Transform(position_m=position, quaternion_wxyz=initial.quaternion_wxyz), motion_state


def _reject_duplicate_object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ScenarioPlanError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise ScenarioPlanError(f"non-finite JSON value is not allowed: {value}")


def _reject_unknown_fields(value: Mapping[str, Any], allowed: frozenset[str], context: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ScenarioPlanError(f"{context} has unknown field(s): {', '.join(unknown)}")


def _require_fields(value: Mapping[str, Any], required: frozenset[str], context: str) -> None:
    missing = sorted(required - set(value))
    if missing:
        raise ScenarioPlanError(f"{context} is missing required field(s): {', '.join(missing)}")


def _identifier(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ScenarioPlanError(f"{field_name} must be a non-empty trimmed string")
    return value


def _nonnegative_int(value: Any, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ScenarioPlanError(f"{field_name} must be a non-negative integer")
    return value


def _finite_float(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ScenarioPlanError(f"{field_name} must be finite")
    return float(value)


def _positive_float(value: Any, field_name: str) -> float:
    result = _finite_float(value, field_name)
    if result <= 0.0:
        raise ScenarioPlanError(f"{field_name} must be positive")
    return result


def _finite_tuple(value: Any, length: int, field_name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ScenarioPlanError(f"{field_name} must be a list of {length} finite numbers")
    return tuple(_finite_float(item, f"{field_name}[{index}]") for index, item in enumerate(value))
