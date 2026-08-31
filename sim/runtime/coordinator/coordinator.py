"""Own one simulation session lifecycle and its run-specific allocation."""

from __future__ import annotations

import copy
import json
import logging
import math
import os
import re
import threading
import time
import uuid
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Mapping, Protocol, TypeGuard

import yaml
from sim.runtime.control.contracts import (
    ActuatorCommand,
    CommandSubmitResult,
    ControllerCommand,
    ControllerRuntimeError,
)
from sim.runtime.control.plan import ControllerPlanError, load_control_plan
from sim.runtime.control.session import (
    ControllerComponentFactory,
    SessionControlRuntime,
)
from sim.runtime.recording import EpisodeRecorder, EpisodeResult, EpisodeStatus
from sim.runtime.scenario import (
    CompositeScenarioDispatcher,
    GenerationStamp,
    MujocoScenarioDispatcher,
    ScenarioClock,
    ScenarioDispatcher,
    ScenarioPlanError,
    ScenarioRuntime,
    ScenarioSensorSink,
    ScenarioSnapshot,
    ScenarioVisualSink,
    UdpScenarioVisualSink,
    load_scenario_plan,
)
from sim.runtime.sensors.evidence import (
    THUNDERV4_NAVIGATION_STREAM_IDS,
    SensorEvidenceError,
    build_sensor_stream_summary,
    build_thunderv4_navigation_stream_summary,
    sensor_stream_binding_identity,
)
from sim.runtime.sensors.readiness import (
    SensorReadiness,
    SensorReadinessError,
    SensorStreamState,
)
from sim.runtime.sensors.runtime import SensorPlanError, SensorRuntime
from sim.runtime.sensors.session import (
    SensorEndpointFactory,
    SensorSessionError,
    SessionSensorRuntime,
)

from .atomic_file import replace_file_with_retry
from .readiness import (
    BindingFacet,
    BindingReadiness,
    BindingReadinessError,
    BindingState,
)
from .run_allocation import RunAllocation, RunAllocationError, create_run_allocation

_REALTIME_MANIFEST_PERIOD_NS = 100_000_000

if TYPE_CHECKING:
    from sim.runtime.qualification.scenario_physics_evidence import (
        ScenarioPhysicsEvidenceHost,
    )

_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_ARTIFACT_NAME_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
_RESERVED_EPISODE_ARTIFACT_NAMES = frozenset(
    {"run_allocation", "runtime_manifest"}
)
_LOGGER = logging.getLogger(__name__)


class CoordinatorError(RuntimeError):
    """Raised when a session cannot be prepared or driven safely."""


class RuntimeState(Enum):
    """Externally visible state of one prepared simulation session."""

    NEW = "NEW"
    PREPARING = "PREPARING"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class _ExternalSensorPublication:
    """Generation-bound camera publication accepted from its external owner."""

    source_id: str
    model_generation: int
    reset_generation: int
    sample_count: int
    last_sample_truth_sequence: int
    last_sample_sim_time_ns: int


@dataclass(frozen=True)
class PhysicsRobotPlan:
    """Validated runtime input for one robot instance."""

    instance_id: str
    model_path: Path
    attach_root: str
    initial_keyframe: str | None
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class PhysicsKinematicEntityPlan:
    """Validated runtime input for one scenario-owned MuJoCo mocap proxy."""

    entity_id: str
    model_path: Path
    attach_root: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class PhysicsPayloadPlan:
    """Validated runtime input for one robot-mounted MuJoCo payload."""

    instance_id: str
    namespace: str
    robot_instance_id: str
    package: Mapping[str, str]
    parent_frame: str
    parent_body: str
    model_path: Path
    attach_root: str
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    authority: Literal["mujoco"]
    collision_representation: Literal["primitive_proxy", "convex_proxy"]
    frames: tuple[Mapping[str, Any], ...]


@dataclass(frozen=True)
class PhysicsGlobalPolicy:
    """Session-global MuJoCo options resolved from the owning WorldPackage."""

    timestep_s: float
    integrator: str
    solver: str
    iterations: int
    gravity_mps2: tuple[float, float, float]


@dataclass(frozen=True)
class PhysicsPlan:
    """Validated, repository-resolved input to the Physics Runtime."""

    session_id: str
    model_generation: int
    reset_generation: int
    repo_root: Path
    world_model_path: Path
    global_policy: PhysicsGlobalPolicy
    robots: tuple[PhysicsRobotPlan, ...]
    kinematic_entities: tuple[PhysicsKinematicEntityPlan, ...] = ()
    payloads: tuple[PhysicsPayloadPlan, ...] = ()


class PhysicsHost(Protocol):
    """Process seam used by the coordinator to drive a Physics Runtime."""

    @property
    def pid(self) -> int | None:
        """Return the current host process ID, when available."""

    def prepare(self, plan: PhysicsPlan, allocation: RunAllocation) -> Mapping[str, Any]:
        """Create the physics runtime and return its READY event."""

    def start(self) -> Mapping[str, Any]:
        """Enter RUNNING."""

    def advance(self, steps: int) -> Mapping[str, Any]:
        """Advance the shared physics clock and return a snapshot."""

    def snapshot(self) -> Mapping[str, Any]:
        """Return the current immutable physics snapshot without advancing."""

    def bind_actuators(
        self,
        *,
        source_id: str,
        instance_id: str,
        command_type: str,
        stale_timeout_ns: int,
        channels: tuple[str, ...],
    ) -> Mapping[str, Any]:
        """Resolve one controller's stable actuator layout."""

    def apply_actuator_command(self, command: ActuatorCommand) -> Mapping[str, Any]:
        """Apply one generation-stamped dense actuator command."""

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> Mapping[str, Any]:
        """Apply one atomic scenario-owned kinematic pose batch."""

    def pause(self) -> Mapping[str, Any]:
        """Enter PAUSED."""

    def reset(self) -> Mapping[str, Any]:
        """Reset physics state and return a new-generation snapshot."""

    def stop(self) -> Mapping[str, Any]:
        """Stop and release the host process."""


def _is_scenario_physics_evidence_host(
    host: PhysicsHost,
) -> TypeGuard[ScenarioPhysicsEvidenceHost]:
    """Return whether a PhysicsHost exposes the stricter scenario evidence seam."""

    return (
        callable(getattr(host, "apply_kinematic_poses", None))
        and callable(getattr(host, "snapshot", None))
        and callable(getattr(host, "raycast", None))
    )


def _json_object(path: Path) -> dict[str, Any]:
    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise CoordinatorError(f"{path.name} contains duplicate key {key!r}")
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=object_from_pairs,
            parse_constant=lambda value: (_ for _ in ()).throw(
                CoordinatorError(f"{path.name} contains non-finite value {value}")
            ),
        )
    except CoordinatorError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise CoordinatorError(f"cannot read {path.name}: {exc}") from exc
    if type(value) is not dict:
        raise CoordinatorError(f"{path.name} must contain a JSON object")
    return value


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise CoordinatorError(f"{field} must be an object")
    return value


def _text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise CoordinatorError(f"{field} must be a non-empty trimmed string")
    return value


def _generation(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CoordinatorError(f"{field} must be a non-negative integer")
    return value


def _validate_episode_artifact_name(name: object) -> str:
    if not isinstance(name, str) or _ARTIFACT_NAME_RE.fullmatch(name) is None:
        raise CoordinatorError("episode artifact name contains unsupported characters")
    if name in _RESERVED_EPISODE_ARTIFACT_NAMES:
        raise CoordinatorError(
            f"episode artifact name {name!r} is reserved by the runtime"
        )
    return name


def _vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise CoordinatorError(f"{field} must contain exactly {size} values")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise CoordinatorError(f"{field}[{index}] must be numeric")
        number = float(item)
        if not math.isfinite(number):
            raise CoordinatorError(f"{field}[{index}] must be finite")
        result.append(number)
    return tuple(result)


def _resolved_asset(repo_root: Path, value: Any, field: str) -> Path:
    relative = Path(_text(value, field))
    if relative.is_absolute():
        raise CoordinatorError(f"{field} must be repository-relative")
    candidate = (repo_root / relative).resolve()
    try:
        candidate.relative_to(repo_root)
    except ValueError as exc:
        raise CoordinatorError(f"{field} escapes the repository root") from exc
    if not candidate.is_file():
        raise CoordinatorError(f"{field} does not exist: {candidate}")
    return candidate


def _physics_payloads(
    robot: Mapping[str, Any],
    *,
    robot_index: int,
    robot_instance_id: str,
    repo_root: Path,
    instance_ids: set[str],
) -> tuple[PhysicsPayloadPlan, ...]:
    raw_payloads = robot.get("payloads", [])
    if not isinstance(raw_payloads, list):
        raise CoordinatorError(
            f"physics_plan.robots[{robot_index}].payloads must be a list"
        )
    robot_frames = robot.get("frames")
    if not isinstance(robot_frames, list):
        raise CoordinatorError(
            f"physics_plan.robots[{robot_index}].frames must be a list"
        )
    frame_by_name: dict[str, Mapping[str, Any]] = {}
    for frame_index, frame_value in enumerate(robot_frames):
        context = f"physics_plan.robots[{robot_index}].frames[{frame_index}]"
        frame = _mapping(frame_value, context)
        name = _text(frame.get("name"), f"{context}.name")
        if name in frame_by_name:
            raise CoordinatorError(f"duplicate robot frame {name!r}")
        frame_by_name[name] = frame

    result: list[PhysicsPayloadPlan] = []
    expected_keys = {
        "instance_id",
        "namespace",
        "robot_instance_id",
        "package",
        "parent_frame",
        "parent_body",
        "mount_transform",
        "model",
        "authority",
        "collision_representation",
        "frames",
    }
    for payload_index, payload_value in enumerate(raw_payloads):
        context = f"physics_plan.robots[{robot_index}].payloads[{payload_index}]"
        payload = _mapping(payload_value, context)
        if set(payload) != expected_keys:
            raise CoordinatorError(f"{context} has invalid fields")
        instance_id = _text(payload.get("instance_id"), f"{context}.instance_id")
        namespace = _text(payload.get("namespace"), f"{context}.namespace")
        if namespace != instance_id:
            raise CoordinatorError(f"{context}.namespace must equal instance_id")
        if instance_id in instance_ids:
            raise CoordinatorError(f"duplicate physics attachment namespace {instance_id!r}")
        instance_ids.add(instance_id)
        if payload.get("robot_instance_id") != robot_instance_id:
            raise CoordinatorError(
                f"{context}.robot_instance_id does not match its containing robot"
            )

        package = _mapping(payload.get("package"), f"{context}.package")
        package_keys = {"id", "version", "kind", "manifest"}
        if set(package) != package_keys or package.get("kind") != "payload":
            raise CoordinatorError(f"{context}.package must identify a payload package")
        typed_package = {
            key: _text(package.get(key), f"{context}.package.{key}")
            for key in ("id", "version", "kind", "manifest")
        }
        parent_frame = _text(payload.get("parent_frame"), f"{context}.parent_frame")
        parent_body = _text(payload.get("parent_body"), f"{context}.parent_body")
        mount_frame = frame_by_name.get(parent_frame)
        if mount_frame is None or mount_frame.get("role") != "payload_mount":
            raise CoordinatorError(
                f"{context}.parent_frame must identify a robot payload_mount frame"
            )
        expected_parent_body = mount_frame.get("parent_frame", parent_frame)
        if parent_body != expected_parent_body:
            raise CoordinatorError(f"{context}.parent_body does not match parent_frame")
        transform = _mapping(payload.get("mount_transform"), f"{context}.mount_transform")
        if set(transform) != {"position_m", "quaternion_wxyz"}:
            raise CoordinatorError(f"{context}.mount_transform has invalid fields")
        position = _vector(
            transform.get("position_m"), 3, f"{context}.mount_transform.position_m"
        )
        quaternion = _vector(
            transform.get("quaternion_wxyz"),
            4,
            f"{context}.mount_transform.quaternion_wxyz",
        )
        if sum(component * component for component in quaternion) < 1e-24:
            raise CoordinatorError(f"{context}.mount_transform.quaternion_wxyz is zero")
        expected_transform = mount_frame.get(
            "extrinsic",
            {"position_m": [0.0, 0.0, 0.0], "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]},
        )
        if transform != expected_transform:
            raise CoordinatorError(f"{context}.mount_transform does not match parent_frame")

        model = _mapping(payload.get("model"), f"{context}.model")
        if set(model) != {"mjcf", "attach_root"}:
            raise CoordinatorError(f"{context}.model has invalid fields")
        model_path = _resolved_asset(repo_root, model.get("mjcf"), f"{context}.model.mjcf")
        attach_root = _text(model.get("attach_root"), f"{context}.model.attach_root")

        frames_value = payload.get("frames")
        if not isinstance(frames_value, list) or not frames_value:
            raise CoordinatorError(f"{context}.frames must be a non-empty list")
        frames: list[Mapping[str, Any]] = []
        payload_frame_names: set[str] = set()
        for frame_index, frame_value in enumerate(frames_value):
            frame_context = f"{context}.frames[{frame_index}]"
            frame = _mapping(frame_value, frame_context)
            frame_name = _text(frame.get("name"), f"{frame_context}.name")
            _text(frame.get("role"), f"{frame_context}.role")
            if "parent_frame" in frame:
                _text(frame.get("parent_frame"), f"{frame_context}.parent_frame")
            if frame_name in payload_frame_names:
                raise CoordinatorError(f"duplicate payload frame {frame_name!r}")
            payload_frame_names.add(frame_name)
            frames.append(copy.deepcopy(frame))
        if attach_root not in payload_frame_names:
            raise CoordinatorError(f"{context}.model.attach_root is absent from payload frames")
        for frame_index, frame in enumerate(frames):
            parent = frame.get("parent_frame")
            if parent is not None and parent not in payload_frame_names:
                raise CoordinatorError(
                    f"{context}.frames[{frame_index}].parent_frame is unknown"
                )

        authority = payload.get("authority")
        if authority != "mujoco":
            raise CoordinatorError(f"{context}.authority must be mujoco")
        collision = payload.get("collision_representation")
        if collision not in {"primitive_proxy", "convex_proxy"}:
            raise CoordinatorError(f"{context}.collision_representation is unsupported")
        result.append(
            PhysicsPayloadPlan(
                instance_id=instance_id,
                namespace=namespace,
                robot_instance_id=robot_instance_id,
                package=typed_package,
                parent_frame=parent_frame,
                parent_body=parent_body,
                model_path=model_path,
                attach_root=attach_root,
                position_m=position,  # type: ignore[arg-type]
                quaternion_wxyz=quaternion,  # type: ignore[arg-type]
                authority=authority,
                collision_representation=collision,
                frames=tuple(frames),
            )
        )
    return tuple(result)


def load_physics_plan(bundle_dir: Path, repo_root: Path) -> PhysicsPlan:
    """Validate one SessionBundle and return absolute Physics Runtime inputs."""

    bundle_dir = Path(bundle_dir).resolve()
    repo_root = Path(repo_root).resolve()
    if not bundle_dir.is_dir():
        raise CoordinatorError(f"session bundle does not exist: {bundle_dir}")
    if not repo_root.is_dir():
        raise CoordinatorError(f"repository root does not exist: {repo_root}")

    raw = _json_object(bundle_dir / "physics.plan.json")
    plan_schema = raw.get("schema")
    if plan_schema not in {"lingtu.sim.physics-plan.v1", "lingtu.sim.physics-plan.v2"}:
        raise CoordinatorError("physics.plan.json has an unsupported schema")
    plan_digest = _text(raw.get("session_id"), "physics_plan.session_id")

    composition = _mapping(raw.get("composition"), "physics_plan.composition")
    required_composition = {
        "model_kind": "single_mjmodel",
        "composer": "mjs_attach_v1",
        "namespace_separator": "__",
        "state_authority": "mujoco",
    }
    if composition != required_composition:
        raise CoordinatorError("physics_plan.composition is unsupported")

    global_policy = _mapping(raw.get("global_policy"), "physics_plan.global_policy")
    required_policy_keys = {
        "owner",
        "timestep_s",
        "integrator",
        "solver",
        "iterations",
        "gravity_mps2",
    }
    if set(global_policy) != required_policy_keys or global_policy.get("owner") != "world":
        raise CoordinatorError("physics_plan.global_policy must be an explicit world-owned policy")
    timestep = global_policy.get("timestep_s")
    if (
        isinstance(timestep, bool)
        or not isinstance(timestep, (int, float))
        or not math.isfinite(float(timestep))
        or float(timestep) <= 0
    ):
        raise CoordinatorError("physics_plan.global_policy.timestep_s must be positive and finite")
    integrator = global_policy.get("integrator")
    if integrator not in {"euler", "rk4", "implicit", "implicitfast"}:
        raise CoordinatorError("physics_plan.global_policy.integrator is unsupported")
    solver = global_policy.get("solver")
    if solver not in {"pgs", "cg", "newton"}:
        raise CoordinatorError("physics_plan.global_policy.solver is unsupported")
    iterations = global_policy.get("iterations")
    if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations <= 0:
        raise CoordinatorError("physics_plan.global_policy.iterations must be a positive integer")
    gravity = _vector(global_policy.get("gravity_mps2"), 3, "physics_plan.global_policy.gravity_mps2")

    world = _mapping(raw.get("world"), "physics_plan.world")
    world_path = _resolved_asset(repo_root, world.get("mjcf"), "physics_plan.world.mjcf")

    raw_robots = raw.get("robots")
    if not isinstance(raw_robots, list) or not raw_robots:
        raise CoordinatorError("physics_plan.robots must contain at least one robot")
    robots: list[PhysicsRobotPlan] = []
    payloads: list[PhysicsPayloadPlan] = []
    instance_ids: set[str] = set()
    for index, value in enumerate(raw_robots):
        item = _mapping(value, f"physics_plan.robots[{index}]")
        instance_id = _text(item.get("instance_id"), f"physics_plan.robots[{index}].instance_id")
        if instance_id in instance_ids:
            raise CoordinatorError(f"duplicate robot instance_id {instance_id!r}")
        instance_ids.add(instance_id)
        if item.get("namespace") != instance_id:
            raise CoordinatorError(f"physics_plan.robots[{index}].namespace must equal instance_id")
        model = _mapping(item.get("model"), f"physics_plan.robots[{index}].model")
        model_path = _resolved_asset(
            repo_root,
            model.get("mjcf"),
            f"physics_plan.robots[{index}].model.mjcf",
        )
        spawn = _mapping(item.get("spawn"), f"physics_plan.robots[{index}].spawn")
        position = _vector(
            spawn.get("position_m"),
            3,
            f"physics_plan.robots[{index}].spawn.position_m",
        )
        quaternion = _vector(
            spawn.get("quaternion_wxyz"),
            4,
            f"physics_plan.robots[{index}].spawn.quaternion_wxyz",
        )
        if sum(value * value for value in quaternion) < 1e-24:
            raise CoordinatorError(f"physics_plan.robots[{index}].spawn.quaternion_wxyz is zero")
        robots.append(
            PhysicsRobotPlan(
                instance_id=instance_id,
                model_path=model_path,
                attach_root=_text(
                    model.get("attach_root"),
                    f"physics_plan.robots[{index}].model.attach_root",
                ),
                initial_keyframe=(
                    None
                    if model.get("initial_keyframe") is None
                    else _text(
                        model.get("initial_keyframe"),
                        f"physics_plan.robots[{index}].model.initial_keyframe",
                    )
                ),
                position_m=position,  # type: ignore[arg-type]
                quaternion_wxyz=quaternion,  # type: ignore[arg-type]
            )
        )
        if plan_schema == "lingtu.sim.physics-plan.v2":
            payloads.extend(
                _physics_payloads(
                    item,
                    robot_index=index,
                    robot_instance_id=instance_id,
                    repo_root=repo_root,
                    instance_ids=instance_ids,
                )
            )

    raw_kinematic_entities = raw.get("kinematic_entities", [])
    if not isinstance(raw_kinematic_entities, list):
        raise CoordinatorError("physics_plan.kinematic_entities must be a list")
    kinematic_entities: list[PhysicsKinematicEntityPlan] = []
    for index, value in enumerate(raw_kinematic_entities):
        context = f"physics_plan.kinematic_entities[{index}]"
        item = _mapping(value, context)
        entity_id = _text(item.get("entity_id"), f"{context}.entity_id")
        if "__" in entity_id:
            raise CoordinatorError(f"{context}.entity_id must not contain '__'")
        if entity_id in instance_ids:
            raise CoordinatorError(f"duplicate physics attachment namespace {entity_id!r}")
        instance_ids.add(entity_id)
        if item.get("namespace") != entity_id:
            raise CoordinatorError(f"{context}.namespace must equal entity_id")
        _mapping(item.get("package"), f"{context}.package")
        model = _mapping(item.get("model"), f"{context}.model")
        model_path = _resolved_asset(repo_root, model.get("mjcf"), f"{context}.model.mjcf")
        transform = _mapping(item.get("initial_transform"), f"{context}.initial_transform")
        position = _vector(
            transform.get("position_m"), 3, f"{context}.initial_transform.position_m"
        )
        quaternion = _vector(
            transform.get("quaternion_wxyz"),
            4,
            f"{context}.initial_transform.quaternion_wxyz",
        )
        if sum(component * component for component in quaternion) < 1e-24:
            raise CoordinatorError(f"{context}.initial_transform.quaternion_wxyz is zero")
        kinematic_entities.append(
            PhysicsKinematicEntityPlan(
                entity_id=entity_id,
                model_path=model_path,
                attach_root=_text(model.get("attach_root"), f"{context}.model.attach_root"),
                position_m=position,  # type: ignore[arg-type]
                quaternion_wxyz=quaternion,  # type: ignore[arg-type]
            )
        )

    if "model_generation" in raw or "reset_generation" in raw:
        raise CoordinatorError("physics plan must not contain runtime generation fields")
    return PhysicsPlan(
        session_id=plan_digest,
        model_generation=0,
        reset_generation=0,
        repo_root=repo_root,
        world_model_path=world_path,
        global_policy=PhysicsGlobalPolicy(
            timestep_s=float(timestep),
            integrator=integrator,
            solver=solver,
            iterations=iterations,
            gravity_mps2=gravity,  # type: ignore[arg-type]
        ),
        robots=tuple(robots),
        kinematic_entities=tuple(kinematic_entities),
        payloads=tuple(payloads),
    )


class RuntimeCoordinator:
    """Drive one resolved simulation session and own only run-time facts."""

    def __init__(
        self,
        *,
        bundle_dir: Path,
        repo_root: Path,
        run_root: Path,
        physics_host: PhysicsHost,
        run_id: str | None = None,
        boot_id: str | None = None,
        adopt_existing_empty_run_dir: bool = False,
        trusted_root: Path | None = None,
        artifact_root_mode: Literal["repository", "run"] = "repository",
        dds_domain: int = 0,
        ports: Mapping[str, int] | None = None,
        shm: Mapping[str, str] | None = None,
        controller_factory: ControllerComponentFactory | None = None,
        sensor_endpoint_factory: SensorEndpointFactory | None = None,
        scenario_dispatcher: ScenarioDispatcher | None = None,
        scenario_visual_sink: ScenarioVisualSink | None = None,
        scenario_sensor_sink: ScenarioSensorSink | None = None,
    ) -> None:
        self._bundle_dir = Path(bundle_dir).resolve()
        self._repo_root = Path(repo_root).resolve()
        self._run_root = Path(os.path.abspath(os.fspath(run_root)))
        self._trusted_root = (
            None
            if trusted_root is None
            else Path(os.path.abspath(os.fspath(trusted_root)))
        )
        self._host = physics_host
        self._run_id = run_id or f"run-{uuid.uuid4().hex}"
        self._boot_id = boot_id
        self._adopt_existing_empty_run_dir = adopt_existing_empty_run_dir
        self._artifact_root_mode = artifact_root_mode
        self._dds_domain = dds_domain
        self._ports = dict(ports or {})
        self._shm = dict(shm or {})
        self._controller_factory = controller_factory
        self._sensor_endpoint_factory = sensor_endpoint_factory
        self._scenario_dispatcher = scenario_dispatcher
        self._scenario_visual_sink = scenario_visual_sink
        self._scenario_sensor_sink = scenario_sensor_sink
        self._owned_scenario_visual_sink: UdpScenarioVisualSink | None = None
        if scenario_dispatcher is not None and (
            scenario_visual_sink is not None or scenario_sensor_sink is not None
        ):
            raise CoordinatorError(
                "scenario_dispatcher cannot be combined with individual scenario sinks"
            )
        if _RUN_ID_RE.fullmatch(self._run_id) is None:
            raise CoordinatorError("run_id contains unsupported characters")
        self._run_dir = self._run_root / self._run_id
        self._allocation: RunAllocation | None = None
        self._state = RuntimeState.NEW
        self._plan: PhysicsPlan | None = None
        self._sensor_plan: SensorRuntime | None = None
        self._last_event: dict[str, Any] | None = None
        self._physics_pid: int | None = None
        self._control: SessionControlRuntime | None = None
        self._sensors: SessionSensorRuntime | None = None
        self._scenario: ScenarioRuntime | None = None
        self._scenario_updates_physics = False
        self._last_scenario_snapshot: ScenarioSnapshot | None = None
        self._readiness: BindingReadiness | None = None
        self._sensor_readiness: SensorReadiness | None = None
        self._binding_sources: dict[BindingFacet, str] = {}
        self._sensor_binding_sources: dict[str, str] = {}
        self._native_sensor_ids: frozenset[str] = frozenset()
        self._external_sensor_publications: dict[
            str, _ExternalSensorPublication
        ] = {}
        self._start_sim_time_ns: int | None = None
        self._episode_result: EpisodeResult | None = None
        self._episode_artifacts: dict[str, str] = {}
        self._terminal_failure_reason: str | None = None
        self._manifest_write_lock = threading.Lock()
        self._next_realtime_manifest_write_ns = 0
        self._latest_runtime_manifest: dict[str, Any] | None = None

    @property
    def state(self) -> RuntimeState:
        """Return the current session runtime state."""

        return self._state

    @property
    def plan(self) -> PhysicsPlan:
        """Return the validated plan after preparation."""

        if self._plan is None:
            raise CoordinatorError("session has not been prepared")
        return self._plan

    @property
    def bundle_dir(self) -> Path:
        """Return the resolved immutable session bundle directory."""

        return self._bundle_dir

    @property
    def manifest_path(self) -> Path:
        """Return the run-specific mutable manifest path."""

        return self._run_dir / "session.runtime.json"

    def runtime_manifest_snapshot(self) -> dict[str, Any]:
        """Return the latest atomically committed manifest without disk I/O."""

        with self._manifest_write_lock:
            if self._latest_runtime_manifest is None:
                raise CoordinatorError("runtime manifest is not available")
            return copy.deepcopy(self._latest_runtime_manifest)

    def register_episode_artifact(self, name: str, path: str) -> None:
        """Bind one existing run-local artifact into terminal episode evidence."""

        _validate_episode_artifact_name(name)
        if not isinstance(path, str) or not path or path != path.strip():
            raise CoordinatorError("episode artifact path must be non-empty text")
        if self._state in {RuntimeState.STOPPED, RuntimeState.FAILED}:
            raise CoordinatorError("episode artifacts cannot be registered after closure")
        allocation = self._allocation
        if allocation is None:
            raise CoordinatorError("episode artifacts require a prepared run allocation")
        relative = Path(path)
        if relative.is_absolute() or relative.drive or any(part in {"", ".", ".."} for part in relative.parts):
            raise CoordinatorError("episode artifact must be a run-local relative path")
        try:
            target = (allocation.run_dir / relative).resolve(strict=True)
            target.relative_to(allocation.run_dir.resolve(strict=True))
        except (FileNotFoundError, OSError, ValueError) as exc:
            raise CoordinatorError("episode artifact must resolve to an existing run-local file") from exc
        if not target.is_file():
            raise CoordinatorError("episode artifact must resolve to an existing run-local file")
        normalized = relative.as_posix()
        previous = self._episode_artifacts.get(name)
        if previous is not None and previous != normalized:
            raise CoordinatorError(f"episode artifact {name!r} is already registered")
        self._episode_artifacts[name] = normalized

    def declare_episode_artifact(self, name: str, relative_path: str) -> None:
        """Declare a run-local artifact that will be written during shutdown."""

        _validate_episode_artifact_name(name)
        if self._allocation is None or self._state not in {
            RuntimeState.PREPARING,
            RuntimeState.READY,
        }:
            raise CoordinatorError(
                "future episode artifacts may be declared only for a prepared, not-yet-running session"
            )
        relative = Path(relative_path)
        if (
            not relative_path
            or relative_path != relative_path.strip()
            or relative.is_absolute()
            or relative.drive
            or relative_path != relative.as_posix()
            or any(part in {"", ".", ".."} for part in relative.parts)
        ):
            raise CoordinatorError("episode artifact must be a normalized run-local relative path")
        previous = self._episode_artifacts.get(name)
        if previous is not None and previous != relative_path:
            raise CoordinatorError(f"episode artifact {name!r} is already registered")
        self._episode_artifacts[name] = relative_path

    @property
    def allocation(self) -> RunAllocation:
        """Return the run allocation after preparation."""

        if self._allocation is None:
            raise CoordinatorError("session has not been prepared")
        return self._allocation

    @property
    def readiness(self) -> BindingReadiness:
        """Return immutable per-facet qualification for the current generation."""

        if self._readiness is None:
            raise CoordinatorError("session binding readiness is not initialized")
        return self._readiness

    @property
    def sensor_readiness(self) -> SensorReadiness:
        """Return immutable per-stream sensor qualification for this session."""

        if self._sensor_readiness is None:
            raise CoordinatorError("session does not declare sensor streams")
        return self._sensor_readiness

    def prepare(self) -> dict[str, Any]:
        """Validate the bundle, allocate the run, and reach READY."""

        self._require_state(RuntimeState.NEW)
        plan = load_physics_plan(self._bundle_dir, self._repo_root)
        try:
            control_plan = load_control_plan(self._bundle_dir / "control.plan.json")
        except ControllerPlanError as exc:
            raise CoordinatorError(str(exc)) from exc
        if control_plan.session_id != plan.session_id:
            raise CoordinatorError("control plan session_id does not match the physics plan")
        try:
            sensor_plan = SensorRuntime.from_path(self._bundle_dir / "sensor.plan.json")
        except SensorPlanError as exc:
            raise CoordinatorError(str(exc)) from exc
        if sensor_plan.session_id != plan.session_id:
            raise CoordinatorError("sensor plan session_id does not match the physics plan")
        scenario_path = self._bundle_dir / "scenario.plan.json"
        try:
            scenario_plan = load_scenario_plan(scenario_path if scenario_path.exists() else None)
        except ScenarioPlanError as exc:
            raise CoordinatorError(str(exc)) from exc
        if scenario_plan is not None and scenario_plan.session_id != plan.session_id:
            raise CoordinatorError("scenario plan session_id does not match the physics plan")
        scenario_runtime = ScenarioRuntime(scenario_plan) if scenario_plan is not None else None
        scenario_dispatcher = self._scenario_dispatcher
        scenario_updates_physics = isinstance(scenario_dispatcher, MujocoScenarioDispatcher) or bool(
            getattr(scenario_dispatcher, "updates_physics", False)
        )
        if scenario_runtime is not None:
            if scenario_plan is None:
                raise CoordinatorError(
                    "scenario runtime exists without a loaded scenario plan"
                )
            expected_kinematic_bindings = {
                (entity.entity_id, f"{entity.entity_id}/{entity.attach_root}")
                for entity in plan.kinematic_entities
            }
            actual_kinematic_bindings = set(scenario_runtime.mujoco_kinematic_bindings)
            if actual_kinematic_bindings != expected_kinematic_bindings:
                raise CoordinatorError(
                    "scenario and physics plans disagree on MuJoCo kinematic entity bindings"
                )
            if scenario_runtime.requires_dispatch and scenario_dispatcher is None:
                apply_kinematic_poses = getattr(self._host, "apply_kinematic_poses", None)
                for entity in scenario_plan.entities:
                    if entity.authority not in {"scenario", "ue_animation"}:
                        continue
                    if entity.authority == "scenario" and entity.physics_proxy_mode == "kinematic":
                        if not callable(apply_kinematic_poses):
                            raise CoordinatorError("scenario plan requires a scenario dispatcher")
                        if entity.body_stable_id is None:
                            raise CoordinatorError(
                                f"scenario entity {entity.entity_id!r} is missing body_stable_id"
                            )
                    elif (
                        self._scenario_visual_sink is None
                        and "visual_snapshot_udp" not in self._ports
                    ):
                        raise CoordinatorError(
                            f"scenario entity {entity.entity_id!r} requires a visual sink"
                        )
        try:
            session_declaration = _mapping(
                yaml.safe_load((self._bundle_dir / "session.yaml").read_text(encoding="utf-8")),
                "session.yaml",
            )
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise CoordinatorError(f"cannot read session.yaml: {exc}") from exc
        try:
            readiness = BindingReadiness.from_session(
                session_declaration,
                model_generation=plan.model_generation,
                reset_generation=plan.reset_generation,
            )
        except ValueError as exc:
            raise CoordinatorError(f"invalid session binding requirements: {exc}") from exc
        if BindingFacet.CONTROL in readiness.required_facets and not control_plan.controllers:
            raise CoordinatorError("session requires control binding but control.plan declares no controllers")
        if control_plan.controllers and self._controller_factory is None:
            raise CoordinatorError("resolved session declares controllers but no controller_factory was provided")
        if BindingFacet.SENSORS in readiness.required_facets and not sensor_plan.streams:
            raise CoordinatorError("session requires sensor binding but sensor.plan declares no streams")
        sensor_readiness = (
            SensorReadiness.from_runtime(
                sensor_plan,
                sensors_required=True,
                model_generation=plan.model_generation,
                reset_generation=plan.reset_generation,
            )
            if sensor_plan.streams
            else None
        )
        try:
            allocation = create_run_allocation(
                self._bundle_dir,
                self._run_root,
                run_id=self._run_id,
                boot_id=self._boot_id,
                dds_domain=self._dds_domain,
                ports=self._ports,
                shm=self._shm,
                repo_root=self._repo_root,
                adopt_existing_empty_run_dir=self._adopt_existing_empty_run_dir,
                trusted_root=self._trusted_root,
                artifact_root_mode=self._artifact_root_mode,
            )
        except RunAllocationError as exc:
            raise CoordinatorError(str(exc)) from exc
        self._allocation = allocation
        self._plan = plan
        self._sensor_plan = sensor_plan
        self._readiness = readiness
        self._sensor_readiness = sensor_readiness
        self._state = RuntimeState.PREPARING
        self._scenario = scenario_runtime
        try:
            if (
                scenario_runtime is not None
                and scenario_runtime.requires_dispatch
                and scenario_dispatcher is None
            ):
                visual_sink = self._scenario_visual_sink
                snapshot_port = self._ports.get("visual_snapshot_udp")
                if visual_sink is None and snapshot_port is not None:
                    try:
                        self._owned_scenario_visual_sink = UdpScenarioVisualSink(snapshot_port)
                    except (OSError, ValueError) as exc:
                        raise CoordinatorError(
                            f"scenario visual sink could not be created: {exc}"
                        ) from exc
                    visual_sink = self._owned_scenario_visual_sink
                physics_sink = None
                if scenario_runtime.mujoco_kinematic_bindings:
                    from sim.runtime.qualification.scenario_physics_evidence import (
                        ScenarioPhysicsEvidenceSink,
                        build_default_scenario_physics_raycast_probes,
                    )

                    if not _is_scenario_physics_evidence_host(self._host):
                        raise CoordinatorError(
                            "scenario plan requires MuJoCo readback/raycast evidence"
                        )
                    physics_sink = ScenarioPhysicsEvidenceSink(
                        host=self._host,
                        run_dir=allocation.run_dir,
                        run_id=self._run_id,
                        raycast_probes=build_default_scenario_physics_raycast_probes(
                            physics_plan=plan,
                        ),
                        register_episode_artifact=self.register_episode_artifact,
                    )
                scenario_dispatcher = CompositeScenarioDispatcher(
                    session_id=plan.session_id,
                    initial_generation=GenerationStamp(
                        plan.model_generation,
                        plan.reset_generation,
                    ),
                    physics_sink=physics_sink,
                    visual_sink=visual_sink,
                    sensor_sink=self._scenario_sensor_sink,
                )
                scenario_updates_physics = physics_sink is not None
            self._scenario_dispatcher = scenario_dispatcher
            self._scenario_updates_physics = scenario_updates_physics
            host_event = self._host.prepare(plan, allocation)
            self._physics_pid = self._host.pid
            event = self._accept_event(host_event, expected="ready")
            self._readiness = self.readiness.mark_prepared(
                BindingFacet.PHYSICS,
                model_generation=event["model_generation"],
                reset_generation=event["reset_generation"],
            ).mark_active(
                BindingFacet.PHYSICS,
                model_generation=event["model_generation"],
                reset_generation=event["reset_generation"],
            )
            self._binding_sources[BindingFacet.PHYSICS] = "mujoco-runtime"
            if control_plan.controllers:
                factory = self._controller_factory
                if factory is None:
                    raise CoordinatorError("controller_factory invariant failed")
                self._control = SessionControlRuntime(
                    plan=control_plan,
                    attach_roots={robot.instance_id: robot.attach_root for robot in plan.robots},
                    physics_host=self._host,
                    component_factory=factory,
                    repo_root=self._repo_root,
                )
                self._control.prepare(event)
                self._readiness = self.readiness.mark_prepared(
                    BindingFacet.CONTROL,
                    model_generation=event["model_generation"],
                    reset_generation=event["reset_generation"],
                ).mark_active(
                    BindingFacet.CONTROL,
                    model_generation=event["model_generation"],
                    reset_generation=event["reset_generation"],
                )
                self._binding_sources[BindingFacet.CONTROL] = "controller-runtime"
            initial_snapshot: dict[str, Any] | None = None
            sensor_snapshot_required = False
            if sensor_plan.streams and self._sensor_endpoint_factory is not None:
                self._sensors = SessionSensorRuntime(
                    plan=sensor_plan,
                    allocation=allocation,
                    endpoint_factory=self._sensor_endpoint_factory,
                )
                sensor_preparation = self._sensors.prepare()
                self._native_sensor_ids = frozenset(
                    sensor_preparation.prepared_sensor_ids
                )
                for sensor_id in sensor_preparation.prepared_sensor_ids:
                    self._sensor_readiness = self.sensor_readiness.mark_prepared(
                        sensor_id,
                        source=self.sensor_readiness.streams[sensor_id].source,
                        model_generation=event["model_generation"],
                        reset_generation=event["reset_generation"],
                    )
                    self._sensor_binding_sources[sensor_id] = sensor_preparation.sources[sensor_id]
                for sensor_id in sensor_preparation.active_sensor_ids:
                    self._sensor_readiness = self.sensor_readiness.mark_active(
                        sensor_id,
                        source=self.sensor_readiness.streams[sensor_id].source,
                        model_generation=event["model_generation"],
                        reset_generation=event["reset_generation"],
                    )
                sensor_snapshot_required = bool(sensor_preparation.active_sensor_ids)
            if self._scenario is not None and self._scenario.requires_dispatch:
                if initial_snapshot is None:
                    initial_snapshot = self._accept_event(self._host.snapshot(), expected="snapshot")
                self._dispatch_scenario_event(initial_snapshot)
                if self._scenario_updates_physics:
                    initial_snapshot = self._accept_event(self._host.snapshot(), expected="snapshot")
            if self._sensors is not None and sensor_snapshot_required:
                if initial_snapshot is None:
                    initial_snapshot = self._accept_event(self._host.snapshot(), expected="snapshot")
                self._sensors.process_snapshot(initial_snapshot)
                self._sync_sensor_runtime_activity(initial_snapshot)
            self._sync_sensor_facet()
        except Exception as exc:
            self._physics_pid = self._host.pid
            self._remember_terminal_failure(exc)
            if isinstance(exc, SensorSessionError):
                self._record_sensor_runtime_failure(exc)
            self._close_sensor_runtime(log_failure=True)
            self._close_owned_scenario_visual_sink(log_failure=True)
            try:
                self._host.stop()
            except Exception:
                _LOGGER.exception("failed to stop physics host after prepare error")
            self._state = RuntimeState.FAILED
            self._write_manifest()
            self._write_episode_result()
            if isinstance(
                exc,
                (ControllerRuntimeError, SensorReadinessError, SensorSessionError),
            ):
                raise CoordinatorError(str(exc)) from exc
            raise
        self._refresh_prepared_state()
        self._write_manifest()
        return event

    def start(self) -> dict[str, Any]:
        """Start or resume physics execution."""

        if self._state not in {RuntimeState.READY, RuntimeState.PAUSED}:
            self._require_state(RuntimeState.READY)
        event = self._accept_event(self._host.start(), expected="running")
        self._state = RuntimeState.RUNNING
        self._write_manifest()
        return event

    def snapshot(self) -> dict[str, Any]:
        """Return the current immutable physics snapshot without advancing."""

        if self._state not in {
            RuntimeState.PREPARING,
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }:
            raise CoordinatorError(f"snapshot is invalid in state {self._state.value}")
        event = self._accept_event(self._host.snapshot(), expected="snapshot")
        self._write_manifest()
        return event

    def warmup(self, steps: int = 1) -> dict[str, Any]:
        """Advance during PREPARING without exposing the session as RUNNING."""

        if self._state not in {RuntimeState.PREPARING, RuntimeState.READY}:
            raise CoordinatorError(f"warmup is invalid in state {self._state.value}")
        if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
            raise CoordinatorError("steps must be a positive integer")

        visible_state = self._state
        self._accept_event(self._host.start(), expected="running")
        try:
            event = self._advance_running(steps)
            self._accept_event(self._host.pause(), expected="paused")
        except Exception:
            if self._state is not RuntimeState.FAILED:
                self._state = visible_state
                self._write_manifest()
            raise
        self._state = visible_state
        self._refresh_prepared_state()
        self._write_manifest()
        return event

    def advance(self, steps: int = 1) -> dict[str, Any]:
        """Advance a running session by a positive number of fixed steps."""

        self._require_state(RuntimeState.RUNNING)
        if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
            raise CoordinatorError("steps must be a positive integer")
        event = self._advance_running(steps)
        self._write_manifest()
        return event

    def advance_realtime(self, steps: int = 1) -> dict[str, Any]:
        """Advance one interactive interval with a single control boundary.

        The latest actuator output is held by Physics across the fixed MuJoCo
        substeps.  Sensor schedules still catch up against the final truth
        snapshot, avoiding one text-protocol round trip per physics substep.
        """

        self._require_state(RuntimeState.RUNNING)
        if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
            raise CoordinatorError("steps must be a positive integer")
        event = self._advance_realtime_running(steps)
        self._write_realtime_manifest_if_due()
        return event

    def _advance_realtime_running(self, steps: int) -> dict[str, Any]:
        scenario_active = self._scenario is not None and self._scenario.requires_dispatch
        if self._control is None and self._sensors is None and not scenario_active:
            return self._accept_event(self._host.advance(steps), expected="snapshot")

        current = self._last_event
        if (
            current is None
            or current.get("event") != "snapshot"
            or "bodies" not in current
            or "joints" not in current
        ):
            current = self._accept_event(self._host.snapshot(), expected="snapshot")
        try:
            sampled_advance = getattr(self._host, "advance_sampled", None)
            sampled_snapshots: tuple[Mapping[str, Any], ...] | None = None
            if (
                self._sensors is not None
                and not scenario_active
                and callable(sampled_advance)
            ):
                fused_control_advance = (
                    getattr(self._control, "step_and_advance_sampled", None)
                    if self._control is not None
                    else None
                )
                raw_sampled = (
                    fused_control_advance(current, steps)
                    if callable(fused_control_advance)
                    else sampled_advance(steps)
                )
                if (
                    not isinstance(raw_sampled, tuple)
                    or not raw_sampled
                    or len(raw_sampled) > steps
                    or any(not isinstance(item, Mapping) for item in raw_sampled)
                ):
                    raise CoordinatorError(
                        "physics sampled advance returned an invalid snapshot batch"
                    )
                first_physics_step = _generation(
                    current.get("physics_step"),
                    "physics snapshot physics_step",
                )
                expected_final_physics_step = first_physics_step + steps
                previous_physics_step = first_physics_step
                for sampled in raw_sampled:
                    sampled_physics_step = _generation(
                        sampled.get("physics_step"),
                        "sampled physics snapshot physics_step",
                    )
                    if not (
                        previous_physics_step
                        < sampled_physics_step
                        <= expected_final_physics_step
                    ):
                        raise CoordinatorError(
                            "physics sampled advance returned an invalid snapshot batch"
                        )
                    previous_physics_step = sampled_physics_step
                if previous_physics_step != expected_final_physics_step:
                    raise CoordinatorError(
                        "physics sampled advance returned an invalid snapshot batch"
                    )
                sampled_snapshots = raw_sampled
                for sampled in sampled_snapshots:
                    current = self._accept_event(
                        sampled,
                        expected="snapshot",
                    )
                    self._sensors.process_snapshot(current)
            else:
                if self._control is not None:
                    self._control.step(current)
                current = self._accept_event(
                    self._host.advance(steps),
                    expected="snapshot",
                )
            if scenario_active:
                self._dispatch_scenario_event(current)
                if self._scenario_updates_physics:
                    current = self._accept_event(
                        self._host.snapshot(),
                        expected="snapshot",
                    )
            if self._sensors is not None:
                if sampled_snapshots is None:
                    self._sensors.process_snapshot(current)
                self._sync_sensor_runtime_activity(current)
        except (ControllerRuntimeError, SensorSessionError) as exc:
            if isinstance(exc, SensorSessionError):
                self._record_sensor_runtime_failure(exc)
            self._abort_active_runtime(exc)
            raise CoordinatorError(str(exc)) from exc
        except Exception as exc:
            self._abort_active_runtime(exc)
            raise
        return current

    def _advance_running(self, steps: int) -> dict[str, Any]:
        scenario_active = self._scenario is not None and self._scenario.requires_dispatch
        if self._control is None and self._sensors is None and not scenario_active:
            event = self._accept_event(self._host.advance(steps), expected="snapshot")
        else:
            current = self._last_event
            if (
                current is None
                or current.get("event") != "snapshot"
                or "bodies" not in current
                or "joints" not in current
            ):
                current = self._accept_event(self._host.snapshot(), expected="snapshot")
            try:
                for _ in range(steps):
                    if self._control is not None:
                        self._control.step(current)
                    current = self._accept_event(self._host.advance(1), expected="snapshot")
                    if scenario_active:
                        self._dispatch_scenario_event(current)
                        if self._scenario_updates_physics:
                            current = self._accept_event(self._host.snapshot(), expected="snapshot")
                    if self._sensors is not None:
                        self._sensors.process_snapshot(current)
                        self._sync_sensor_runtime_activity(current)
            except (ControllerRuntimeError, SensorSessionError) as exc:
                if isinstance(exc, SensorSessionError):
                    self._record_sensor_runtime_failure(exc)
                self._abort_active_runtime(exc)
                raise CoordinatorError(str(exc)) from exc
            except Exception as exc:
                self._abort_active_runtime(exc)
                raise
            event = current
        return event

    def _dispatch_scenario_event(
        self,
        event: Mapping[str, Any],
        *,
        allow_generation_change: bool = False,
    ) -> ScenarioSnapshot | None:
        runtime = self._scenario
        if runtime is None or not runtime.requires_dispatch:
            return None
        dispatcher = self._scenario_dispatcher
        if dispatcher is None:
            raise CoordinatorError("scenario dispatcher invariant failed")

        previous = self._last_scenario_snapshot
        if previous is None:
            if (
                event["model_generation"] != self.plan.model_generation
                or event["reset_generation"] != self.plan.reset_generation
            ):
                raise CoordinatorError("scenario initial generation does not match the physics plan")
        else:
            event_generation_changed = (
                event["model_generation"] != previous.model_generation
                or event["reset_generation"] != previous.reset_generation
            )
            if event_generation_changed and not allow_generation_change:
                raise CoordinatorError("scenario generation changed outside reset")

        try:
            snapshot = runtime.snapshot(
                ScenarioClock(
                    session_id=self.plan.session_id,
                    model_generation=event["model_generation"],
                    reset_generation=event["reset_generation"],
                    sim_time_ns=event["sim_time_ns"],
                )
            ).for_dispatch()
        except ScenarioPlanError as exc:
            raise CoordinatorError(f"scenario evaluation failed: {exc}") from exc

        if event["sequence"] != snapshot.sequence:
            raise CoordinatorError(f"physics scenario sequence is {event['sequence']}, expected {snapshot.sequence}")
        if previous is None:
            expected_sequence = 0
        else:
            generation_changed = (
                snapshot.model_generation != previous.model_generation
                or snapshot.reset_generation != previous.reset_generation
            )
            expected_sequence = 0 if generation_changed else previous.sequence + 1
        if snapshot.sequence != expected_sequence:
            raise CoordinatorError(f"scenario sequence is {snapshot.sequence}, expected {expected_sequence}")
        if not snapshot.entities:
            raise CoordinatorError("scenario dispatch snapshot contains no applicable entities")

        try:
            dispatcher.dispatch(snapshot)
        except Exception as exc:
            message = str(exc).strip()
            reason = f"{type(exc).__name__}: {message}" if message else type(exc).__name__
            raise CoordinatorError(f"scenario dispatcher failed: {reason}") from exc
        self._last_scenario_snapshot = snapshot
        return snapshot

    def pause(self) -> dict[str, Any]:
        """Pause physics execution without changing simulation state."""

        self._require_state(RuntimeState.RUNNING)
        event = self._accept_event(self._host.pause(), expected="paused")
        self._state = RuntimeState.PAUSED
        self._write_manifest()
        return event

    def reset(self) -> dict[str, Any]:
        """Reset physics and require a new reset generation."""

        if self._state not in {
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }:
            raise CoordinatorError(f"reset is invalid in state {self._state.value}")
        previous_generation = int((self._last_event or {}).get("reset_generation", 0))
        try:
            event = self._accept_event(self._host.reset(), expected="snapshot")
        except Exception as exc:
            self._abort_active_runtime(exc)
            raise
        if event["reset_generation"] != previous_generation + 1:
            reason = "physics host did not increment reset_generation"
            self._abort_active_runtime(reason)
            raise CoordinatorError(reason)
        try:
            self._dispatch_scenario_event(event, allow_generation_change=True)
            if self._scenario_updates_physics:
                event = self._accept_event(self._host.snapshot(), expected="snapshot")
        except Exception as exc:
            self._abort_active_runtime(exc)
            raise
        if self._control is not None:
            try:
                self._control.set_generation(event)
            except ControllerRuntimeError as exc:
                self._abort_active_runtime(exc)
                raise CoordinatorError(str(exc)) from exc
        if self._readiness is not None:
            self._readiness = self._readiness.with_generations(
                model_generation=event["model_generation"],
                reset_generation=event["reset_generation"],
            )
        if self._sensor_readiness is not None:
            self._sensor_readiness = self._sensor_readiness.with_generations(
                model_generation=event["model_generation"],
                reset_generation=event["reset_generation"],
            )
            self._retract_external_camera_streams_after_reset(event)
        # External camera evidence is generation-bound.  A reset invalidates
        # every prior frame count and truth stamp until the UE producer emits
        # a fresh, matching readiness document.
        self._external_sensor_publications.clear()
        if self._sensors is not None:
            try:
                self._sensors.process_snapshot(event)
                self._sync_sensor_runtime_activity(event)
            except SensorSessionError as exc:
                self._record_sensor_runtime_failure(exc)
                self._abort_active_runtime(exc)
                raise CoordinatorError(str(exc)) from exc
        self._sync_sensor_facet()
        self._state = RuntimeState.READY if self.readiness.is_ready else RuntimeState.PREPARING
        self._write_manifest()
        return event

    def _retract_external_camera_streams_after_reset(
        self,
        event: Mapping[str, Any],
    ) -> None:
        """Move externally published render streams out of READY after reset.

        Same-model resets keep native SensorRuntime endpoints alive, but UE
        camera_shm samples are generation-bound external evidence.  Their
        previous ACTIVE state must not keep the aggregate sensors facet READY
        until the UE producer emits fresh evidence for the new reset generation.
        """

        readiness = self._sensor_readiness
        sensor_plan = self._sensor_plan
        if readiness is None or sensor_plan is None:
            return
        next_readiness = readiness
        for stream in sensor_plan.streams:
            sensor_id = stream.sensor_id
            if stream.route.transport != "camera_shm":
                continue
            if sensor_id not in next_readiness.streams:
                continue
            state = next_readiness.state(sensor_id)
            if state is not SensorStreamState.ACTIVE:
                continue
            try:
                next_readiness = next_readiness.mark_prepared(
                    sensor_id,
                    source=next_readiness.streams[sensor_id].source,
                    model_generation=event["model_generation"],
                    reset_generation=event["reset_generation"],
                )
            except SensorReadinessError as exc:
                raise CoordinatorError(str(exc)) from exc
        self._sensor_readiness = next_readiness

    def report_sensor_stream_prepared(
        self,
        sensor_id: str,
        **evidence: Any,
    ) -> None:
        """Accept PREPARED evidence for one compiled SensorPlan stream."""

        self._report_sensor_stream(sensor_id, SensorStreamState.PREPARED, **evidence)

    def report_sensor_stream_active(
        self,
        sensor_id: str,
        **evidence: Any,
    ) -> None:
        """Accept ACTIVE evidence for one compiled SensorPlan stream."""

        self._report_sensor_stream(sensor_id, SensorStreamState.ACTIVE, **evidence)

    def report_sensor_stream_retracted(
        self,
        sensor_id: str,
        **evidence: Any,
    ) -> None:
        """Retract stale ACTIVE stream evidence back to non-ready PREPARED."""

        self._report_sensor_stream(sensor_id, SensorStreamState.PREPARED, **evidence)

    def report_sensor_stream_failed(
        self,
        sensor_id: str,
        **evidence: Any,
    ) -> None:
        """Record a stream failure and abort when the Sensor facet is required."""

        self._report_sensor_stream(sensor_id, SensorStreamState.FAILED, **evidence)

    def report_binding_prepared(
        self,
        facet: BindingFacet | str,
        **evidence: Any,
    ) -> None:
        """Accept generation-stamped PREPARED evidence from an external runtime."""

        self._report_binding(facet, BindingState.PREPARED, **evidence)

    def report_binding_active(
        self,
        facet: BindingFacet | str,
        **evidence: Any,
    ) -> None:
        """Accept generation-stamped ACTIVE evidence from an external runtime."""

        self._report_binding(facet, BindingState.ACTIVE, **evidence)

    def report_binding_failed(
        self,
        facet: BindingFacet | str,
        **evidence: Any,
    ) -> None:
        """Record an external binding failure and abort when it is required."""

        self._report_binding(facet, BindingState.FAILED, **evidence)

    def _report_sensor_stream(
        self,
        sensor_id: str,
        state: SensorStreamState,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        reason: str | None = None,
        published_frames: int = 0,
        last_sample_truth_sequence: int = 0,
        last_sample_sim_time_ns: int = 0,
    ) -> None:
        runtime_failed = self._state is RuntimeState.FAILED
        stream_id = self._validate_sensor_stream_evidence(
            sensor_id,
            source_id=source_id,
            session_id=session_id,
            model_generation=model_generation,
            reset_generation=reset_generation,
            allow_failed_runtime=runtime_failed and state is SensorStreamState.FAILED,
        )
        publication = self._external_sensor_publication(
            stream_id,
            source_id=source_id,
            model_generation=model_generation,
            reset_generation=reset_generation,
            published_frames=published_frames,
            last_sample_truth_sequence=last_sample_truth_sequence,
            last_sample_sim_time_ns=last_sample_sim_time_ns,
        )
        readiness = self.sensor_readiness
        stream_source = readiness.streams[stream_id].source
        try:
            if state is SensorStreamState.FAILED:
                readiness = readiness.mark_failed(
                    stream_id,
                    reason,
                    source=stream_source,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
            elif state is SensorStreamState.ACTIVE:
                readiness = readiness.mark_active(
                    stream_id,
                    source=stream_source,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
            else:
                readiness = readiness.mark_prepared(
                    stream_id,
                    source=stream_source,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
        except (SensorReadinessError, ValueError) as exc:
            raise CoordinatorError(str(exc)) from exc
        self._sensor_readiness = readiness
        self._sensor_binding_sources[stream_id] = source_id
        if publication is not None:
            self._external_sensor_publications[stream_id] = publication
        self._sync_sensor_facet()
        if state is not SensorStreamState.FAILED:
            self._refresh_prepared_state()
        if (
            state is SensorStreamState.FAILED
            and not runtime_failed
            and BindingFacet.SENSORS in self.readiness.required_facets
            and stream_id in readiness.required_stream_ids
        ):
            self._abort_active_runtime(f"{stream_id}: {reason}")
        else:
            self._write_manifest()

    def _report_binding(
        self,
        facet: BindingFacet | str,
        state: BindingState,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        reason: str | None = None,
    ) -> None:
        key = self._validate_external_binding_evidence(
            facet,
            source_id=source_id,
            session_id=session_id,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )
        readiness = self.readiness
        try:
            if state is BindingState.FAILED:
                readiness = readiness.mark_failed(
                    key,
                    reason,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
            elif state is BindingState.ACTIVE:
                readiness = readiness.mark_active(
                    key,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
            else:
                readiness = readiness.mark_prepared(
                    key,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                )
        except (BindingReadinessError, ValueError) as exc:
            raise CoordinatorError(str(exc)) from exc
        self._readiness = readiness
        self._binding_sources[key] = source_id
        if state is BindingState.FAILED and key in readiness.required_facets:
            self._abort_active_runtime(f"{key.value}: {reason}")
        else:
            self._refresh_prepared_state()
            self._write_manifest()

    def submit_controller_command(self, controller_id: str, command: ControllerCommand) -> CommandSubmitResult:
        """Submit one command to a prepared plan-declared Controller Runtime."""

        if self._state not in {
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }:
            raise CoordinatorError(f"controller command is invalid in state {self._state.value}")
        if self._control is None:
            raise CoordinatorError("session does not have a prepared Controller Runtime")
        try:
            return self._control.submit_command(controller_id, command)
        except ControllerRuntimeError as exc:
            raise CoordinatorError(str(exc)) from exc

    def hold_controller_commands(self) -> None:
        """Clear active Controller Runtime state while preserving command watermarks."""

        if self._state not in {
            RuntimeState.PREPARING,
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
            RuntimeState.FAILED,
        }:
            raise CoordinatorError(f"controller hold is invalid in state {self._state.value}")
        if self._control is None:
            return
        try:
            self._control.hold()
        except ControllerRuntimeError as exc:
            raise CoordinatorError(str(exc)) from exc

    def stop(self, *, failure_reason: str | None = None) -> dict[str, Any]:
        """Stop the physics host and commit exactly one terminal result."""

        if self._state in {RuntimeState.STOPPED, RuntimeState.FAILED}:
            self._write_episode_result()
            return dict(self._last_event or {})
        if failure_reason is not None:
            self._remember_terminal_failure(failure_reason)
        if self._state is RuntimeState.NEW:
            self._state = RuntimeState.FAILED if self._terminal_failure_reason is not None else RuntimeState.STOPPED
            return {}

        sensor_error = self._close_sensor_runtime(log_failure=False)
        if sensor_error is not None:
            self._remember_terminal_failure(sensor_error)
        scenario_sink_error = self._close_owned_scenario_visual_sink(log_failure=False)
        if scenario_sink_error is not None:
            self._remember_terminal_failure(scenario_sink_error)
        try:
            event = self._accept_event(self._host.stop(), expected="stopped")
        except Exception as exc:
            self._remember_terminal_failure(exc)
            self._state = RuntimeState.FAILED
            self._write_manifest()
            self._write_episode_result()
            raise

        self._state = RuntimeState.FAILED if self._terminal_failure_reason is not None else RuntimeState.STOPPED
        self._write_manifest()
        self._write_episode_result()
        if sensor_error is not None:
            raise CoordinatorError(str(sensor_error)) from sensor_error
        if scenario_sink_error is not None:
            raise CoordinatorError(str(scenario_sink_error)) from scenario_sink_error
        return event

    def finalize_terminal_failure(self, failure_reason: str) -> dict[str, Any]:
        """Promote a stopped terminal result to FAILED after post-stop evidence fails."""

        self._remember_terminal_failure(failure_reason)
        if self._state is RuntimeState.NEW:
            self._state = RuntimeState.FAILED
            return {}
        if self._state not in {RuntimeState.STOPPED, RuntimeState.FAILED}:
            raise CoordinatorError(
                f"terminal failure finalization is invalid in state {self._state.value}"
            )
        self._state = RuntimeState.FAILED
        self._write_manifest()
        self._episode_result = None
        self._write_episode_result()
        return dict(self._last_event or {})

    def _sync_sensor_runtime_activity(self, event: Mapping[str, Any]) -> None:
        """Promote concrete Sensor Runtime endpoints after real snapshot output."""

        runtime = self._sensors
        readiness = self._sensor_readiness
        if runtime is None or readiness is None:
            return
        for sensor_id in runtime.active_sensor_ids:
            if sensor_id not in readiness.streams:
                continue
            source_id = runtime.sources.get(sensor_id)
            if source_id is None:
                continue
            stream_source = readiness.streams[sensor_id].source
            try:
                if readiness.state(sensor_id) is SensorStreamState.UNBOUND:
                    readiness = readiness.mark_prepared(
                        sensor_id,
                        source=stream_source,
                        model_generation=event["model_generation"],
                        reset_generation=event["reset_generation"],
                    )
                if readiness.state(sensor_id) is SensorStreamState.PREPARED:
                    readiness = readiness.mark_active(
                        sensor_id,
                        source=stream_source,
                        model_generation=event["model_generation"],
                        reset_generation=event["reset_generation"],
                    )
                    self._sensor_binding_sources[sensor_id] = source_id
            except SensorReadinessError as exc:
                raise CoordinatorError(str(exc)) from exc
        self._sensor_readiness = readiness
        self._sync_sensor_facet()
        self._refresh_prepared_state()

    def _validate_sensor_stream_evidence(
        self,
        sensor_id: str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
        allow_failed_runtime: bool = False,
    ) -> str:
        allowed_states = {
            RuntimeState.PREPARING,
            RuntimeState.READY,
            RuntimeState.RUNNING,
            RuntimeState.PAUSED,
        }
        if allow_failed_runtime:
            allowed_states.add(RuntimeState.FAILED)
        if self._state not in allowed_states:
            raise CoordinatorError(f"sensor stream evidence is invalid in state {self._state.value}")
        if not isinstance(sensor_id, str) or not sensor_id or sensor_id != sensor_id.strip():
            raise CoordinatorError("sensor_id must be a non-empty trimmed string")
        if sensor_id not in self.sensor_readiness.streams:
            raise CoordinatorError(f"unknown SensorPlan stream {sensor_id!r}")
        if not isinstance(source_id, str) or not source_id or source_id != source_id.strip():
            raise CoordinatorError("sensor source_id must be a non-empty trimmed string")
        if session_id != self.plan.session_id:
            raise CoordinatorError("sensor session_id does not match the session")
        if model_generation != self.sensor_readiness.model_generation:
            raise CoordinatorError("sensor model_generation does not match the current model")
        if reset_generation != self.sensor_readiness.reset_generation:
            raise CoordinatorError("sensor reset_generation does not match the current reset")
        previous_source = self._sensor_binding_sources.get(sensor_id)
        if previous_source is not None and previous_source != source_id:
            raise CoordinatorError(f"sensor stream {sensor_id!r} is already owned by {previous_source!r}")
        return sensor_id

    def _validate_external_binding_evidence(
        self,
        facet: BindingFacet | str,
        *,
        source_id: str,
        session_id: str,
        model_generation: int,
        reset_generation: int,
    ) -> BindingFacet:
        if self._state not in {
            RuntimeState.PREPARING,
            RuntimeState.READY,
            RuntimeState.PAUSED,
        }:
            raise CoordinatorError(f"binding evidence is invalid in state {self._state.value}")
        try:
            key = facet if isinstance(facet, BindingFacet) else BindingFacet(facet)
        except (TypeError, ValueError) as exc:
            raise CoordinatorError(f"unsupported binding facet {facet!r}") from exc
        if key in {BindingFacet.PHYSICS, BindingFacet.CONTROL}:
            raise CoordinatorError(f"{key.value} binding is owned internally by RuntimeCoordinator")
        if key is BindingFacet.SENSORS and self._sensor_readiness is not None:
            raise CoordinatorError("sensor binding requires per-stream SensorPlan evidence")
        if not isinstance(source_id, str) or not source_id or source_id != source_id.strip():
            raise CoordinatorError("binding source_id must be a non-empty trimmed string")
        if session_id != self.plan.session_id:
            raise CoordinatorError("binding session_id does not match the session")
        if model_generation != self.readiness.model_generation:
            raise CoordinatorError("binding model_generation does not match the current model")
        if reset_generation != self.readiness.reset_generation:
            raise CoordinatorError("binding reset_generation does not match the current reset")
        previous_source = self._binding_sources.get(key)
        if previous_source is not None and previous_source != source_id:
            raise CoordinatorError(f"{key.value} binding is already owned by {previous_source!r}")
        return key

    def _external_sensor_publication(
        self,
        sensor_id: str,
        *,
        source_id: str,
        model_generation: int,
        reset_generation: int,
        published_frames: int,
        last_sample_truth_sequence: int,
        last_sample_sim_time_ns: int,
    ) -> _ExternalSensorPublication | None:
        count = _generation(published_frames, "sensor published_frames")
        truth_sequence = _generation(
            last_sample_truth_sequence,
            "sensor last_sample_truth_sequence",
        )
        sim_time_ns = _generation(
            last_sample_sim_time_ns,
            "sensor last_sample_sim_time_ns",
        )
        plan = self._sensor_plan
        if plan is None:
            raise CoordinatorError("sensor plan is not initialized")
        stream = next(
            (item for item in plan.streams if item.sensor_id == sensor_id),
            None,
        )
        if stream is None:
            raise CoordinatorError(f"unknown SensorPlan stream {sensor_id!r}")
        if stream.route.transport != "camera_shm":
            if count or truth_sequence or sim_time_ns:
                raise CoordinatorError(
                    "external publication evidence is valid only for camera_shm streams"
                )
            return None
        if count == 0 and (truth_sequence != 0 or sim_time_ns != 0):
            raise CoordinatorError(
                "camera sample stamps require at least one published frame"
            )
        publication = _ExternalSensorPublication(
            source_id=source_id,
            model_generation=model_generation,
            reset_generation=reset_generation,
            sample_count=count,
            last_sample_truth_sequence=truth_sequence,
            last_sample_sim_time_ns=sim_time_ns,
        )
        return publication

    def _refresh_prepared_state(self) -> None:
        if self._state not in {RuntimeState.PREPARING, RuntimeState.READY}:
            return
        self._state = RuntimeState.READY if self.readiness.is_ready else RuntimeState.PREPARING

    def _sync_sensor_facet(self) -> None:
        """Project per-stream qualification onto the coarse session facet."""

        stream_readiness = self._sensor_readiness
        if stream_readiness is None or self._readiness is None:
            return
        facet_state = self.readiness.state(BindingFacet.SENSORS)
        if stream_readiness.failures:
            reason = "; ".join(
                f"{sensor_id}: {failure}" for sensor_id, failure in sorted(stream_readiness.failures.items())
            )
            if facet_state.value != "FAILED":
                self._readiness = self.readiness.mark_failed(
                    BindingFacet.SENSORS,
                    reason,
                    model_generation=stream_readiness.model_generation,
                    reset_generation=stream_readiness.reset_generation,
                )
            return

        required_states = tuple(
            stream_readiness.state(sensor_id) for sensor_id in sorted(stream_readiness.required_stream_ids)
        )
        all_prepared = bool(required_states) and all(
            state in {SensorStreamState.PREPARED, SensorStreamState.ACTIVE} for state in required_states
        )
        if all_prepared and not stream_readiness.is_ready and facet_state.value == "ACTIVE":
            self._readiness = self.readiness.mark_prepared(
                BindingFacet.SENSORS,
                model_generation=stream_readiness.model_generation,
                reset_generation=stream_readiness.reset_generation,
            )
            self._binding_sources[BindingFacet.SENSORS] = "sensor-stream-aggregate"
            facet_state = self.readiness.state(BindingFacet.SENSORS)
        if all_prepared and facet_state.value == "UNBOUND":
            self._readiness = self.readiness.mark_prepared(
                BindingFacet.SENSORS,
                model_generation=stream_readiness.model_generation,
                reset_generation=stream_readiness.reset_generation,
            )
            self._binding_sources[BindingFacet.SENSORS] = "sensor-stream-aggregate"
            facet_state = self.readiness.state(BindingFacet.SENSORS)
        if stream_readiness.is_ready and facet_state.value == "PREPARED":
            self._readiness = self.readiness.mark_active(
                BindingFacet.SENSORS,
                model_generation=stream_readiness.model_generation,
                reset_generation=stream_readiness.reset_generation,
            )
            self._binding_sources[BindingFacet.SENSORS] = "sensor-stream-aggregate"

    def _require_state(self, expected: RuntimeState) -> None:
        if self._state is not expected:
            raise CoordinatorError(f"operation requires {expected.value}, current state is {self._state.value}")

    def _remember_terminal_failure(self, failure: BaseException | str) -> None:
        if isinstance(failure, str):
            reason = failure.strip()
        else:
            message = str(failure).strip()
            reason = f"{type(failure).__name__}: {message}" if message else type(failure).__name__
        if not reason:
            raise CoordinatorError("failure reason must be non-empty")
        if self._terminal_failure_reason is None:
            self._terminal_failure_reason = reason

    def _abort_active_runtime(self, failure: BaseException | str) -> None:
        """Quiesce actuator output, stop the host, and persist a terminal failure."""

        self._remember_terminal_failure(failure)
        self._close_sensor_runtime(log_failure=True)
        self._close_owned_scenario_visual_sink(log_failure=True)
        try:
            self._host.pause()
        except Exception:
            _LOGGER.exception("failed to pause physics host during runtime abort")
        try:
            self._host.stop()
        except Exception:
            _LOGGER.exception("failed to stop physics host during runtime abort")
        self._physics_pid = self._host.pid
        self._state = RuntimeState.FAILED
        self._write_manifest()
        self._write_episode_result()

    def _close_sensor_runtime(self, *, log_failure: bool) -> Exception | None:
        runtime = self._sensors
        if runtime is None:
            return None
        try:
            runtime.close()
            return None
        except Exception as exc:
            if log_failure:
                _LOGGER.exception("failed to close Sensor Runtime")
            return exc

    def _close_owned_scenario_visual_sink(self, *, log_failure: bool) -> Exception | None:
        sink = self._owned_scenario_visual_sink
        self._owned_scenario_visual_sink = None
        if sink is None:
            return None
        try:
            sink.close()
            return None
        except Exception as exc:
            if log_failure:
                _LOGGER.exception("failed to close scenario visual sink")
            return exc

    def _record_sensor_runtime_failure(self, error: SensorSessionError) -> None:
        sensor_id = error.sensor_id
        readiness = self._sensor_readiness
        if (
            sensor_id is None
            or readiness is None
            or sensor_id not in readiness.streams
            or readiness.state(sensor_id) is SensorStreamState.FAILED
        ):
            return
        self._sensor_readiness = readiness.mark_failed(
            sensor_id,
            str(error),
            source=readiness.streams[sensor_id].source,
            model_generation=readiness.model_generation,
            reset_generation=readiness.reset_generation,
        )
        self._sync_sensor_facet()

    def _accept_event(self, value: Mapping[str, Any], *, expected: str) -> dict[str, Any]:
        if not isinstance(value, Mapping):
            raise CoordinatorError("physics host event must be an object")
        event = dict(value)
        if event.get("event") != expected:
            raise CoordinatorError(f"physics host returned {event.get('event')!r}, expected {expected!r}")
        if event.get("session_id") != self.plan.session_id:
            raise CoordinatorError("physics host session_id does not match the plan")
        if event.get("model_generation") != self.plan.model_generation:
            raise CoordinatorError("physics host model_generation does not match the plan")
        for field in ("reset_generation", "sequence", "physics_step", "sim_time_ns"):
            _generation(event.get(field), f"physics host {field}")
        if self._start_sim_time_ns is None:
            self._start_sim_time_ns = int(event["sim_time_ns"])
        self._last_event = event
        return event

    def _write_episode_result(self) -> None:
        allocation = self._allocation
        plan = self._plan
        if (
            self._episode_result is not None
            or allocation is None
            or plan is None
            or self._state not in {RuntimeState.STOPPED, RuntimeState.FAILED}
        ):
            return
        last = self._last_event or {}
        failed = self._state is RuntimeState.FAILED
        failure_reason = (
            self._terminal_failure_reason or "runtime entered FAILED without a reported cause" if failed else None
        )
        result = EpisodeResult(
            run_id=self._run_id,
            session_id=plan.session_id,
            model_generation=plan.model_generation,
            reset_generation=int(last.get("reset_generation", plan.reset_generation)),
            start_sim_time_ns=self._start_sim_time_ns or 0,
            end_sim_time_ns=int(last.get("sim_time_ns", self._start_sim_time_ns or 0)),
            status=EpisodeStatus.FAILED if failed else EpisodeStatus.SUCCEEDED,
            failure_reason=failure_reason,
            artifact_references={
                "run_allocation": allocation.path.name,
                "runtime_manifest": self.manifest_path.name,
                **self._episode_artifacts,
            },
        )
        EpisodeRecorder(allocation.run_dir).write(result)
        self._episode_result = result

    def _write_manifest(self) -> None:
        with self._manifest_write_lock:
            self._write_manifest_unlocked()
            self._next_realtime_manifest_write_ns = (
                time.monotonic_ns() + _REALTIME_MANIFEST_PERIOD_NS
            )

    def _write_realtime_manifest_if_due(self) -> None:
        with self._manifest_write_lock:
            now_ns = time.monotonic_ns()
            if now_ns < self._next_realtime_manifest_write_ns:
                return
            self._write_manifest_unlocked()
            self._next_realtime_manifest_write_ns = (
                now_ns + _REALTIME_MANIFEST_PERIOD_NS
            )

    def _write_manifest_unlocked(self) -> None:
        allocation = self._allocation
        if self._plan is None or allocation is None or not allocation.run_dir.is_dir():
            return
        last = self._last_event or {}
        manifest_model_generation = self._plan.model_generation
        manifest_reset_generation = _generation(
            last.get("reset_generation", self._plan.reset_generation),
            "runtime manifest reset_generation",
        )
        sensor_streams: dict[str, Any] | None = None
        if self._sensor_readiness is not None:
            sensor_streams = self._sensor_readiness.to_manifest()
            for sensor_id, stream in sensor_streams["streams"].items():
                stream["runtime_source_id"] = self._sensor_binding_sources.get(sensor_id)
            sensor_streams["summary"] = self._sensor_summary_for_manifest(
                allocation,
                model_generation=manifest_model_generation,
                reset_generation=manifest_reset_generation,
            )
        manifest = {
            "schema": "lingtu.sim.session-runtime.v1",
            "run_id": self._run_id,
            "session_id": self._plan.session_id,
            "model_generation": manifest_model_generation,
            "reset_generation": manifest_reset_generation,
            "state": self._state.value,
            "bindings": {
                facet.value: {
                    "required": facet in self.readiness.required_facets,
                    "state": qualification.state.value,
                    "source_id": self._binding_sources.get(facet),
                    "failure_reason": qualification.failure_reason,
                    "model_generation": qualification.model_generation,
                    "reset_generation": qualification.reset_generation,
                }
                for facet, qualification in self.readiness.bindings.items()
            },
            "sensor_streams": sensor_streams,
            "bundle_dir": str(self._bundle_dir),
            "allocation": {
                "run_dir": str(allocation.run_dir),
                "log_dir": str(allocation.log_dir),
                "boot_id": allocation.boot_id,
                "physics_pid": self._physics_pid,
                "dds_domain": allocation.dds_domain,
                "ports": dict(allocation.ports),
                "shm": dict(allocation.shm),
                "shared_memory": dict(allocation.shared_memory),
            },
            "clock": {
                "sequence": last.get("sequence", 0),
                "physics_step": last.get("physics_step", 0),
                "sim_time_ns": last.get("sim_time_ns", 0),
            },
        }
        temporary = self.manifest_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(
                manifest,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        replace_file_with_retry(temporary, self.manifest_path)
        self._latest_runtime_manifest = manifest

    def _sensor_summary_for_manifest(
        self,
        allocation: RunAllocation,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> dict[str, Any]:
        plan = self._sensor_plan
        readiness = self._sensor_readiness
        if plan is None or readiness is None:
            raise CoordinatorError("sensor summary requires an initialized SensorPlan")
        if (
            BindingFacet.SENSORS in self.readiness.required_facets
            and not readiness.required_stream_ids
        ):
            raise CoordinatorError("required sensors have no required_stream_ids")

        observations: list[Mapping[str, Any]] = []
        if self._sensors is not None and self._native_sensor_ids:
            observations.extend(
                observation
                for observation in self._sensors.evidence_observations()
                if observation.get("session_id") == plan.session_id
                and observation.get("model_generation") == model_generation
                and observation.get("reset_generation") == reset_generation
            )

        streams = {stream.sensor_id: stream for stream in plan.streams}
        camera_shm_allocations = {
            sensor_id: allocation.shm[sensor_id]
            for sensor_id, stream in streams.items()
            if stream.route.transport == "camera_shm" and sensor_id in allocation.shm
        }
        for sensor_id, publication in sorted(
            self._external_sensor_publications.items()
        ):
            if (
                publication.model_generation != model_generation
                or publication.reset_generation != reset_generation
                or readiness.model_generation != model_generation
                or readiness.reset_generation != reset_generation
            ):
                continue
            stream = streams.get(sensor_id)
            if stream is None or stream.route.transport != "camera_shm":
                continue
            state = readiness.state(sensor_id)
            observation: dict[str, Any] = {
                "sensor_id": sensor_id,
                "state": state.value,
                "session_id": plan.session_id,
                "model_generation": model_generation,
                "reset_generation": reset_generation,
                "owner": stream.route.owner,
                "source": stream.route.source,
                "transport": stream.route.transport,
                "message_type": stream.message_type,
                "runtime_source_id": publication.source_id,
                "binding_identity": sensor_stream_binding_identity(
                    stream,
                    session_id=plan.session_id,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                    runtime_source_id=publication.source_id,
                ),
                "sample_count": publication.sample_count,
                "last_sample_truth_sequence": (
                    publication.last_sample_truth_sequence
                    if publication.sample_count > 0
                    else None
                ),
                "last_sample_sim_time_ns": (
                    publication.last_sample_sim_time_ns
                    if publication.sample_count > 0
                    else None
                ),
                "shm_name": camera_shm_allocations.get(sensor_id),
            }
            if state is SensorStreamState.FAILED:
                observation["failure_reason"] = readiness.failure_reason(sensor_id)
            observations.append(observation)

        try:
            actual_stream_ids = frozenset(streams)
            if actual_stream_ids == frozenset(THUNDERV4_NAVIGATION_STREAM_IDS):
                raw_summary: object = build_thunderv4_navigation_stream_summary(
                    plan,
                    observations,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                    shm_allocations=camera_shm_allocations,
                )
            else:
                raw_summary = build_sensor_stream_summary(
                    plan,
                    observations,
                    model_generation=model_generation,
                    reset_generation=reset_generation,
                    required_stream_ids=readiness.required_stream_ids,
                    shm_allocations=camera_shm_allocations,
                )
        except (SensorEvidenceError, SensorSessionError) as exc:
            raise CoordinatorError(f"cannot build sensor stream summary: {exc}") from exc
        if type(raw_summary) is not dict:
            raise CoordinatorError("sensor stream summary builder returned a non-object")
        summary: dict[str, Any] = {}
        for key, value in raw_summary.items():
            if not isinstance(key, str) or not key:
                raise CoordinatorError("sensor stream summary contains an invalid key")
            summary[key] = value
        return summary
