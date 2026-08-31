"""Runner-side MuJoCo proof for applied scenario kinematic proxies."""

from __future__ import annotations

import json
import math
import os
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, cast

from sim.runtime.scenario import (
    CompositeScenarioDispatcher,
    EntitySnapshot,
    GenerationStamp,
    ScenarioSnapshot,
    ScenarioVisualSink,
)

SCENARIO_PHYSICS_EVIDENCE_FILENAME = "scenario-physics-evidence.json"
SCENARIO_PHYSICS_EVIDENCE_SCHEMA = "lingtu.sim.scenario-physics-evidence.v1"
SCENARIO_PHYSICS_EVIDENCE_SOURCE = "mujoco_applied_kinematic_proxy"
SCENARIO_EVIDENCE_INPUT_SOURCE = "canonical_scenario_snapshot"
SCENARIO_PHYSICS_POSITION_TOLERANCE_M = 0.02
DEFAULT_KINEMATIC_PROXY_RAYCAST_TARGET_Z_OFFSET_M = 0.9


class ScenarioPhysicsEvidenceError(RuntimeError):
    """Raised when MuJoCo scenario evidence cannot be produced honestly."""


class ScenarioPhysicsEvidenceHost(Protocol):
    """Minimum authoritative Physics Runtime seam required for evidence."""

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> Mapping[str, Any]:
        """Apply one canonical scenario snapshot to MuJoCo kinematic proxies."""

    def snapshot(self) -> Mapping[str, Any]:
        """Read back the same authoritative MuJoCo state."""

    def raycast(
        self,
        *,
        sensor_frame_id: str,
        directions_sensor: tuple[tuple[float, float, float], ...],
        session_id: str,
        model_generation: int,
        reset_generation: int,
        sequence: int,
        sim_time_ns: int,
        offsets_time_ns: tuple[int, ...] | None = None,
        range_min_m: float = 0.1,
        range_max_m: float = 40.0,
        reflectivity_proxy: int = 15,
        unknown_line: int = 0,
    ) -> Mapping[str, Any]:
        """Return a MuJoCo raycast frame with hit body identity."""


@dataclass(frozen=True)
class ScenarioPhysicsRaycastProbe:
    """One finite raycast query used to prove proxy observability."""

    sensor_frame_id: str
    direction_sensor: tuple[float, float, float]
    expected_entity_id: str
    expected_body_stable_id: str
    range_min_m: float = 0.01
    range_max_m: float = 40.0
    offset_time_ns: int = 0

    def __post_init__(self) -> None:
        _token(self.sensor_frame_id, "sensor_frame_id")
        _token(self.expected_entity_id, "expected_entity_id")
        _token(self.expected_body_stable_id, "expected_body_stable_id")
        _finite_vector(self.direction_sensor, "direction_sensor")
        if all(value == 0.0 for value in self.direction_sensor):
            raise ValueError("direction_sensor must be non-zero")
        _finite_float(self.range_min_m, "range_min_m")
        _finite_float(self.range_max_m, "range_max_m")
        if self.range_min_m < 0.0 or self.range_max_m <= self.range_min_m:
            raise ValueError("raycast range must be finite and increasing")
        _non_negative_int(self.offset_time_ns, "offset_time_ns")


class ScenarioPhysicsEvidenceSink:
    """Apply scenario poses through MuJoCo and commit readback evidence."""

    def __init__(
        self,
        *,
        host: ScenarioPhysicsEvidenceHost,
        run_dir: Path | Callable[[], Path],
        raycast_probes: Sequence[ScenarioPhysicsRaycastProbe],
        run_id: str | Callable[[], str] | None = None,
        register_episode_artifact: Callable[[str, str], None] | None = None,
    ) -> None:
        if not callable(getattr(host, "apply_kinematic_poses", None)):
            raise ValueError("host must provide apply_kinematic_poses()")
        if not callable(getattr(host, "snapshot", None)):
            raise ValueError("host must provide snapshot()")
        if not callable(getattr(host, "raycast", None)):
            raise ValueError("host must provide raycast()")
        if not raycast_probes:
            raise ValueError("raycast_probes must not be empty")
        self._host = host
        self._run_dir = run_dir
        self._run_id = run_id
        self._raycast_probes = tuple(raycast_probes)
        self._register_episode_artifact = register_episode_artifact
        self._registered = False

    def set_episode_artifact_registrar(
        self,
        register_episode_artifact: Callable[[str, str], None],
    ) -> None:
        """Attach RuntimeCoordinator.register_episode_artifact after construction."""

        self._register_episode_artifact = register_episode_artifact

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> Mapping[str, Any]:
        """Apply one batch, verify same-stamp readback/raycast, and write evidence."""

        _scenario_snapshot(snapshot)
        apply_event = self._host.apply_kinematic_poses(snapshot)
        if apply_event.get("result") != "applied":
            raise ScenarioPhysicsEvidenceError("MuJoCo did not apply kinematic poses")
        _same_stamp(apply_event, snapshot, "kinematic pose application")
        if self._registered:
            return {
                "result": "applied",
                "artifact": SCENARIO_PHYSICS_EVIDENCE_FILENAME,
                "source": SCENARIO_PHYSICS_EVIDENCE_SOURCE,
                "input_source": SCENARIO_EVIDENCE_INPUT_SOURCE,
                "session_id": snapshot.session_id,
                "model_generation": snapshot.model_generation,
                "reset_generation": snapshot.reset_generation,
                "sequence": snapshot.sequence,
                "sim_time_ns": snapshot.sim_time_ns,
            }
        readback = self._host.snapshot()
        _same_stamp(readback, snapshot, "physics readback snapshot")
        expected = _expected_entities(snapshot)
        proxies = _proxy_readback_records(expected, readback)
        max_position_error = max(record["position_error_m"] for record in proxies)
        max_quaternion_error = max(record["quaternion_error"] for record in proxies)
        raycast_observations = [
            self._raycast_observation(probe, snapshot, readback, proxies)
            for probe in self._raycast_probes
        ]
        document = {
            "schema": SCENARIO_PHYSICS_EVIDENCE_SCHEMA,
            "source": SCENARIO_PHYSICS_EVIDENCE_SOURCE,
            "input_source": SCENARIO_EVIDENCE_INPUT_SOURCE,
            "run_id": self._resolved_run_id(),
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "position_tolerance_m": SCENARIO_PHYSICS_POSITION_TOLERANCE_M,
            "maximum_position_error_m": max_position_error,
            "maximum_quaternion_error": max_quaternion_error,
            "within_tolerance": max_position_error <= SCENARIO_PHYSICS_POSITION_TOLERANCE_M,
            "complete_proxy_set": True,
            "pose_applied": True,
            "expected_proxy_count": len(proxies),
            "proxy_count": len(proxies),
            "proxies": proxies,
            "raycast_observations": raycast_observations,
            "contact_observations": [],
        }
        path = self._artifact_path()
        _atomic_write_json(path, document)
        self._register_once()
        return {
            "result": "applied",
            "artifact": SCENARIO_PHYSICS_EVIDENCE_FILENAME,
            "source": SCENARIO_PHYSICS_EVIDENCE_SOURCE,
            "input_source": SCENARIO_EVIDENCE_INPUT_SOURCE,
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "proxy_count": len(proxies),
        }

    def _raycast_observation(
        self,
        probe: ScenarioPhysicsRaycastProbe,
        snapshot: ScenarioSnapshot,
        readback: Mapping[str, Any],
        proxies: Sequence[Mapping[str, Any]],
    ) -> dict[str, Any]:
        direction_sensor, range_max_m = _readback_raycast_query(
            probe,
            readback,
            proxies,
        )
        frame = self._host.raycast(
            sensor_frame_id=probe.sensor_frame_id,
            directions_sensor=(direction_sensor,),
            session_id=snapshot.session_id,
            model_generation=snapshot.model_generation,
            reset_generation=snapshot.reset_generation,
            sequence=snapshot.sequence,
            sim_time_ns=snapshot.sim_time_ns,
            offsets_time_ns=(probe.offset_time_ns,),
            range_min_m=probe.range_min_m,
            range_max_m=range_max_m,
        )
        if frame.get("event") != "raycast":
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast event is invalid")
        if frame.get("session_id") != snapshot.session_id:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast session_id mismatch")
        if frame.get("model_generation") != snapshot.model_generation:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast model_generation mismatch")
        if frame.get("reset_generation") != snapshot.reset_generation:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast reset_generation mismatch")
        if frame.get("sequence") != snapshot.sequence:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast sequence mismatch")
        sim_time_ns = frame.get("sim_time_ns")
        if sim_time_ns != snapshot.sim_time_ns:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast sim_time_ns mismatch")
        hits = frame.get("hits")
        if type(hits) is not list or not hits:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast did not hit a proxy")
        hit = hits[0]
        if type(hit) is not dict:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast hit is invalid")
        if hit.get("entity_id") != probe.expected_entity_id:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast hit the wrong entity")
        if hit.get("body_stable_id") != probe.expected_body_stable_id:
            raise ScenarioPhysicsEvidenceError("MuJoCo raycast hit the wrong body")
        return {
            "source": "mujoco_readback",
            "applied": True,
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "query": {
                "sensor_frame_id": probe.sensor_frame_id,
                "direction_sensor": list(direction_sensor),
                "range_min_m": probe.range_min_m,
                "range_max_m": range_max_m,
                "offset_time_ns": probe.offset_time_ns,
                "origin_world_m": _finite_list(hit.get("origin_world_m"), "origin_world_m", 3),
                "direction_world": _finite_list(hit.get("direction_world"), "direction_world", 3),
            },
            "result": {
                "hit": True,
                "entity_id": hit["entity_id"],
                "body_stable_id": hit["body_stable_id"],
                "distance_m": _finite_float(hit.get("distance_m"), "distance_m"),
                "position_world_m": _finite_list(hit.get("position_world_m"), "position_world_m", 3),
            },
        }

    def _artifact_path(self) -> Path:
        run_dir = self._run_dir() if callable(self._run_dir) else self._run_dir
        path = Path(run_dir).resolve() / SCENARIO_PHYSICS_EVIDENCE_FILENAME
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _resolved_run_id(self) -> str | None:
        run_id = self._run_id() if callable(self._run_id) else self._run_id
        if run_id is None:
            return None
        return _token(run_id, "run_id")

    def _register_once(self) -> None:
        if self._registered:
            return
        if self._register_episode_artifact is None:
            return
        self._register_episode_artifact(
            "scenario_physics_evidence",
            SCENARIO_PHYSICS_EVIDENCE_FILENAME,
        )
        self._registered = True


def build_scenario_physics_evidence_dispatcher(
    *,
    session_id: str,
    initial_generation: GenerationStamp,
    physics_sink: ScenarioPhysicsEvidenceSink,
    visual_sink: ScenarioVisualSink | None = None,
) -> CompositeScenarioDispatcher:
    """Create the production dispatcher without creating a second lifecycle owner."""

    return CompositeScenarioDispatcher(
        session_id=session_id,
        initial_generation=initial_generation,
        physics_sink=physics_sink,
        visual_sink=visual_sink,
    )


def _readback_raycast_query(
    probe: ScenarioPhysicsRaycastProbe,
    readback: Mapping[str, Any],
    proxies: Sequence[Mapping[str, Any]],
) -> tuple[tuple[float, float, float], float]:
    proxy = next(
        (
            candidate
            for candidate in proxies
            if candidate.get("body_stable_id") == probe.expected_body_stable_id
        ),
        None,
    )
    if proxy is None:
        raise ScenarioPhysicsEvidenceError(
            f"physics readback is missing proxy {probe.expected_body_stable_id!r}"
        )
    bodies = _body_readback_by_stable_id(readback)
    sensor = bodies.get(probe.sensor_frame_id)
    if sensor is None:
        return probe.direction_sensor, probe.range_max_m

    observed = proxy.get("observed")
    if type(observed) is not dict:
        raise ScenarioPhysicsEvidenceError("proxy readback observed pose is invalid")
    target_position = _finite_list(observed.get("position_m"), "proxy observed position_m", 3)
    target_position[2] += DEFAULT_KINEMATIC_PROXY_RAYCAST_TARGET_Z_OFFSET_M
    sensor_position = _finite_list(sensor.get("position_m"), "sensor frame position_m", 3)
    sensor_quaternion = _finite_list(sensor.get("quaternion_wxyz"), "sensor frame quaternion_wxyz", 4)
    world_direction = cast(
        tuple[float, float, float],
        tuple(
            float(target_position[axis]) - float(sensor_position[axis])
            for axis in range(3)
        ),
    )
    distance = math.sqrt(sum(component * component for component in world_direction))
    if not math.isfinite(distance) or distance <= 0.0:
        raise ScenarioPhysicsEvidenceError("sensor frame and proxy positions overlap")
    world_unit = cast(
        tuple[float, float, float],
        tuple(component / distance for component in world_direction),
    )
    sensor_direction = _rotate_world_vector_to_local(sensor_quaternion, world_unit)
    return sensor_direction, max(probe.range_max_m, distance + 10.0)


def build_default_scenario_physics_raycast_probes(
    *,
    physics_plan: Any,
) -> tuple[ScenarioPhysicsRaycastProbe, ...]:
    """Build deterministic MuJoCo observability probes from the compiled plan."""

    robots = tuple(getattr(physics_plan, "robots", ()))
    entities = tuple(getattr(physics_plan, "kinematic_entities", ()))
    if not robots:
        raise ScenarioPhysicsEvidenceError("physics plan has no robot frame for raycast probes")
    if not entities:
        raise ScenarioPhysicsEvidenceError("physics plan has no kinematic entities for raycast probes")
    robot = robots[0]
    origin = tuple(float(value) for value in robot.position_m)
    sensor_frame_id = f"{_token(robot.instance_id, 'robot instance_id')}/"
    sensor_frame_id += _token(robot.attach_root, "robot attach_root")
    probes: list[ScenarioPhysicsRaycastProbe] = []
    for entity in entities:
        entity_id = _token(entity.entity_id, "kinematic entity_id")
        attach_root = _token(entity.attach_root, "kinematic attach_root")
        target = tuple(float(value) for value in entity.position_m)
        direction = cast(
            tuple[float, float, float],
            tuple(target[axis] - origin[axis] for axis in range(3)),
        )
        length = math.sqrt(sum(value * value for value in direction))
        if not math.isfinite(length) or length <= 0.0:
            direction = (1.0, 0.0, 0.0)
            length = 1.0
        probes.append(
            ScenarioPhysicsRaycastProbe(
                sensor_frame_id=sensor_frame_id,
                direction_sensor=direction,  # robot root is initially aligned to world in compiled plan
                expected_entity_id=entity_id,
                expected_body_stable_id=f"{entity_id}/{attach_root}",
                range_min_m=0.0,
                range_max_m=max(1.0, length + 10.0),
            )
        )
    return tuple(probes)


def _body_readback_by_stable_id(readback: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    bodies = readback.get("bodies")
    if type(bodies) is not list:
        raise ScenarioPhysicsEvidenceError("physics readback bodies must be an array")
    result: dict[str, Mapping[str, Any]] = {}
    for body in bodies:
        if type(body) is dict and isinstance(body.get("stable_id"), str):
            result[body["stable_id"]] = body
    return result


def _expected_entities(snapshot: ScenarioSnapshot) -> dict[str, EntitySnapshot]:
    expected: dict[str, EntitySnapshot] = {}
    for entity in snapshot.entities:
        if entity.authority != "scenario" or entity.physics_proxy_mode != "kinematic":
            continue
        if entity.body_stable_id is None:
            raise ScenarioPhysicsEvidenceError("scenario entity is missing body_stable_id")
        if entity.entity_id in expected:
            raise ScenarioPhysicsEvidenceError("scenario snapshot contains duplicate entity")
        expected[entity.entity_id] = entity
    if not expected:
        raise ScenarioPhysicsEvidenceError("scenario snapshot has no kinematic proxies")
    return expected


def _proxy_readback_records(
    expected: Mapping[str, EntitySnapshot],
    readback: Mapping[str, Any],
) -> list[dict[str, Any]]:
    by_stable_id = _body_readback_by_stable_id(readback)
    records: list[dict[str, Any]] = []
    for entity in expected.values():
        if entity.body_stable_id is None:
            raise ScenarioPhysicsEvidenceError("scenario entity is missing body_stable_id")
        body = by_stable_id.get(entity.body_stable_id)
        if body is None:
            raise ScenarioPhysicsEvidenceError(
                f"physics readback is missing body {entity.body_stable_id!r}"
            )
        observed_position = tuple(_finite_list(body.get("position_m"), "position_m", 3))
        observed_quaternion = tuple(_finite_list(body.get("quaternion_wxyz"), "quaternion_wxyz", 4))
        position_error = _distance(entity.transform.position_m, observed_position)
        quaternion_error = _quaternion_error(
            entity.transform.quaternion_wxyz,
            observed_quaternion,
        )
        records.append(
            {
                "entity_id": entity.entity_id,
                "body_stable_id": entity.body_stable_id,
                "expected": entity.transform.to_dict(),
                "observed": {
                    "position_m": list(observed_position),
                    "quaternion_wxyz": list(observed_quaternion),
                },
                "position_error_m": position_error,
                "quaternion_error": quaternion_error,
            }
        )
    return records


def _rotate_world_vector_to_local(
    quaternion_wxyz: Sequence[float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = (float(component) for component in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ScenarioPhysicsEvidenceError("sensor frame quaternion is invalid")
    w /= norm
    x = -x / norm
    y = -y / norm
    z = -z / norm
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    result = (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )
    _finite_vector(result, "direction_sensor")
    return cast(tuple[float, float, float], result)


def _same_stamp(
    value: Mapping[str, Any],
    snapshot: ScenarioSnapshot,
    label: str,
) -> None:
    for field in ("session_id", "model_generation", "reset_generation", "sequence", "sim_time_ns"):
        if value.get(field) != getattr(snapshot, field):
            raise ScenarioPhysicsEvidenceError(f"{label} {field} mismatch")


def _scenario_snapshot(snapshot: ScenarioSnapshot) -> None:
    if not isinstance(snapshot, ScenarioSnapshot):
        raise ScenarioPhysicsEvidenceError("snapshot must be a ScenarioSnapshot")
    _token(snapshot.session_id, "session_id")
    _non_negative_int(snapshot.model_generation, "model_generation")
    _non_negative_int(snapshot.reset_generation, "reset_generation")
    _non_negative_int(snapshot.sequence, "sequence")
    _non_negative_int(snapshot.sim_time_ns, "sim_time_ns")


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def _quaternion_error(left: Sequence[float], right: Sequence[float]) -> float:
    direct = _distance(left, right)
    flipped = _distance(left, [-float(value) for value in right])
    return min(direct, flipped)


def _finite_list(value: object, label: str, size: int) -> list[float]:
    if type(value) is not list or len(value) != size:
        raise ScenarioPhysicsEvidenceError(f"{label} must contain {size} values")
    return [_finite_float(item, f"{label}[{index}]") for index, item in enumerate(value)]


def _finite_vector(value: object, label: str) -> None:
    if not isinstance(value, tuple) or len(value) != 3:
        raise ValueError(f"{label} must contain exactly 3 values")
    for index, item in enumerate(value):
        _finite_float(item, f"{label}[{index}]")


def _finite_float(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioPhysicsEvidenceError(f"{label} must be finite numeric data")
    number = float(value)
    if not math.isfinite(number):
        raise ScenarioPhysicsEvidenceError(f"{label} must be finite numeric data")
    return number


def _non_negative_int(value: object, label: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ScenarioPhysicsEvidenceError(f"{label} must be a non-negative integer")


def _token(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ScenarioPhysicsEvidenceError(f"{label} must be non-empty trimmed text")
    return value


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    from sim.runtime.coordinator.atomic_file import replace_file_with_retry

    payload = (
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    descriptor, name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        replace_file_with_retry(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


__all__ = [
    "SCENARIO_PHYSICS_EVIDENCE_FILENAME",
    "SCENARIO_PHYSICS_EVIDENCE_SCHEMA",
    "ScenarioPhysicsEvidenceError",
    "ScenarioPhysicsEvidenceSink",
    "ScenarioPhysicsRaycastProbe",
    "build_default_scenario_physics_raycast_probes",
    "build_scenario_physics_evidence_dispatcher",
]
