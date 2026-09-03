"""Resolve simulation packages into deterministic session artifacts.

The catalog is intentionally local and exact-version only.  It is a build-time
boundary: it does not start processes, allocate ports, load Unreal assets, or
mutate the legacy Python simulation runtime.
"""

from __future__ import annotations

import copy
import json
import math
import re
import struct
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from sim.contracts import physics_sensor_timebase_violation

from .diagnostics import CatalogDiagnostic, DiagnosticCode

_IDENTITY_EXTRINSIC = {
    "position_m": [0.0, 0.0, 0.0],
    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
}
_QUATERNION_NORM_TOLERANCE = 1e-6
_MJCF_INTEGRATORS = {
    "Euler": "euler",
    "RK4": "rk4",
    "implicit": "implicit",
    "implicitfast": "implicitfast",
}
_MJCF_SOLVERS = {"PGS": "pgs", "CG": "cg", "Newton": "newton"}


class CatalogError(ValueError):
    """Raised when a package catalog or session contract is invalid."""

    def __init__(
        self,
        message: str,
        *,
        code: DiagnosticCode | str = DiagnosticCode.CATALOG_INVALID,
        context: str | None = None,
        details: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.context = context
        self.details = dict(details or {})

    def to_diagnostic(self) -> CatalogDiagnostic:
        return CatalogDiagnostic(
            code=self.code,
            message=str(self),
            context=self.context,
            details=self.details,
        )


_SCHEMA_KIND = {
    "lingtu.sim.robot-package.v1": "robot",
    "lingtu.sim.controller-package.v1": "controller",
    "lingtu.sim.sensor-package.v1": "sensor",
    "lingtu.sim.sensor-rig-package.v1": "sensor_rig",
    "lingtu.sim.world-package.v1": "world",
    "lingtu.sim.scenario-package.v1": "scenario",
    "lingtu.sim.payload-package.v1": "payload",
}
_PACKAGE_SUFFIXES = (".package.yaml", ".package.yml")
_REF_RE = re.compile(r"^(?P<id>[A-Za-z0-9][A-Za-z0-9_.-]*)@(?P<version>[A-Za-z0-9][A-Za-z0-9+_.-]*)$")
_SESSION_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,62}$", re.ASCII)


def _pretty_json(value: Any) -> str:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            separators=(",", ": "),
            allow_nan=False,
        )
        + "\n"
    )


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise CatalogError(f"{context} must be a mapping")
    return dict(value)


def _list(value: Any, context: str) -> list[Any]:
    if not isinstance(value, list):
        raise CatalogError(f"{context} must be a list")
    return list(value)


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CatalogError(f"{context} must be a non-empty string")
    return value.strip()


def _session_id(value: Any, context: str) -> str:
    session_id = _string(value, context)
    if value != session_id or _SESSION_ID_RE.fullmatch(session_id) is None:
        raise CatalogError(f"{context} must match {_SESSION_ID_RE.pattern}")
    return session_id


def _keys(data: Mapping[str, Any], required: Iterable[str], optional: Iterable[str], context: str) -> None:
    required_set = set(required)
    optional_set = set(optional)
    missing = sorted(required_set - set(data))
    unknown = sorted(set(data) - required_set - optional_set)
    if missing:
        raise CatalogError(f"{context} is missing required key(s): {', '.join(missing)}")
    if unknown:
        raise CatalogError(f"{context} has unknown key(s): {', '.join(unknown)}")


def _finite_vector(value: Any, size: int, context: str) -> list[float]:
    values = _list(value, context)
    if len(values) != size:
        raise CatalogError(f"{context} must contain exactly {size} values")
    result: list[float] = []
    for index, item in enumerate(values):
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
            raise CatalogError(f"{context}[{index}] must be finite numeric data")
        result.append(float(item))
    return result


def _validate_physics_sensor_timebase(
    world: PackageRecord,
    global_policy: Mapping[str, Any],
    sensor_plan: Mapping[str, Any],
) -> None:
    """Reject periodic physics streams that cannot land on an exact snapshot."""

    timestep_s = global_policy["timestep_s"]
    stream_groups = _mapping(sensor_plan["streams"], "sensor_plan.streams")
    streams = [
        _mapping(stream, "sensor_plan stream")
        for declarations in stream_groups.values()
        for stream in _list(declarations, "sensor_plan stream declarations")
    ]
    violation = physics_sensor_timebase_violation(
        timestep_s=timestep_s,
        streams=streams,
    )
    if violation is None:
        return
    raise CatalogError(
        f"physics-owned sensor {violation.sensor_id!r} at "
        f"{float(violation.rate_hz):g} Hz cannot sample exactly on world "
        f"timestep {float(violation.timestep_s):g} s",
        code=DiagnosticCode.SENSOR_TIMEBASE_INCOMPATIBLE,
        context=violation.sensor_id,
        details={**violation.details(), "world": world.ref},
    )


def _world_global_policy(value: Any, mjcf: Path, context: str) -> dict[str, Any]:
    policy = _mapping(value, context)
    _keys(
        policy,
        ("timestep_s", "integrator", "solver", "iterations", "gravity_mps2"),
        (),
        context,
    )
    timestep = policy["timestep_s"]
    if (
        isinstance(timestep, bool)
        or not isinstance(timestep, (int, float))
        or not math.isfinite(float(timestep))
        or float(timestep) <= 0
    ):
        raise CatalogError(f"{context}.timestep_s must be positive finite numeric data")
    integrator = _string(policy["integrator"], f"{context}.integrator")
    if integrator not in set(_MJCF_INTEGRATORS.values()):
        raise CatalogError(f"{context}.integrator is unsupported: {integrator!r}")
    solver = _string(policy["solver"], f"{context}.solver")
    if solver not in set(_MJCF_SOLVERS.values()):
        raise CatalogError(f"{context}.solver is unsupported: {solver!r}")
    iterations = policy["iterations"]
    if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations <= 0:
        raise CatalogError(f"{context}.iterations must be a positive integer")
    normalized = {
        "timestep_s": float(timestep),
        "integrator": integrator,
        "solver": solver,
        "iterations": iterations,
        "gravity_mps2": _finite_vector(policy["gravity_mps2"], 3, f"{context}.gravity_mps2"),
    }

    option = ET.parse(mjcf).getroot().find("option")
    if option is None:
        raise CatalogError(f"{context} requires the world MJCF to author an option element")
    required_attributes = {"timestep", "integrator", "solver", "iterations", "gravity"}
    missing = sorted(required_attributes - set(option.attrib))
    if missing:
        raise CatalogError(
            f"{context} is not explicit in the world MJCF",
            context=f"{context}.{missing[0]}",
            details={"missing_mjcf_fields": missing},
        )
    try:
        authored = {
            "timestep_s": float(option.attrib["timestep"]),
            "integrator": _MJCF_INTEGRATORS[option.attrib["integrator"]],
            "solver": _MJCF_SOLVERS[option.attrib["solver"]],
            "iterations": int(option.attrib["iterations"]),
            "gravity_mps2": [float(item) for item in option.attrib["gravity"].split()],
        }
    except (KeyError, ValueError) as exc:
        raise CatalogError(f"{context} cannot be reconciled with the world MJCF option") from exc
    for field in ("timestep_s", "integrator", "solver", "iterations", "gravity_mps2"):
        if authored[field] != normalized[field]:
            raise CatalogError(
                f"{context}.{field} does not match the world MJCF option",
                context=f"{context}.{field}",
                details={"manifest": normalized[field], "mjcf": authored[field]},
            )
    return normalized


def _normalized_quaternion(value: Any, context: str) -> list[float]:
    quaternion = _finite_vector(value, 4, context)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
        raise CatalogError(f"{context} must be normalized within {_QUATERNION_NORM_TOLERANCE:g}")
    return quaternion


def _unreal_game_package_path(value: Any, context: str) -> str:
    level = _string(value, context)
    if not level.startswith("/Game/"):
        raise CatalogError(f"{context} must start with /Game/")
    relative = level.removeprefix("/Game/")
    parts = relative.split("/")
    if (
        value != level
        or not relative
        or level.endswith("/")
        or "\\" in level
        or "//" in level
        or any(char.isspace() for char in level)
        or any(part in ("", ".", "..") for part in parts)
    ):
        raise CatalogError(f"{context} must be a valid /Game/... level")
    return level


def _extrinsic(value: Any, context: str) -> dict[str, list[float]]:
    transform = _mapping(value, context)
    has_canonical = "position_m" in transform or "quaternion_wxyz" in transform
    has_legacy = "pos" in transform or "quat" in transform
    if has_canonical and has_legacy:
        raise CatalogError(f"{context} must not mix canonical and legacy extrinsic keys")
    if has_legacy:
        _keys(transform, ("pos", "quat"), (), context)
        position_key = "pos"
        quaternion_key = "quat"
    else:
        _keys(transform, ("position_m", "quaternion_wxyz"), (), context)
        position_key = "position_m"
        quaternion_key = "quaternion_wxyz"
    return {
        "position_m": _finite_vector(transform[position_key], 3, f"{context}.{position_key}"),
        "quaternion_wxyz": _normalized_quaternion(transform[quaternion_key], f"{context}.{quaternion_key}"),
    }


def _quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> list[float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def _rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> list[float]:
    conjugate = [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, [0.0, *vector]),
        conjugate,
    )
    return rotated[1:]


def _compose_extrinsics(
    parent: Mapping[str, Sequence[float]],
    child: Mapping[str, Sequence[float]],
) -> dict[str, list[float]]:
    parent_position = parent["position_m"]
    parent_quaternion = parent["quaternion_wxyz"]
    child_position = child["position_m"]
    rotated_child = _rotate_vector(parent_quaternion, child_position)
    quaternion = _quaternion_multiply(
        parent_quaternion,
        child["quaternion_wxyz"],
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    return {
        "position_m": [
            float(parent_position[index]) + rotated_child[index]
            for index in range(3)
        ],
        "quaternion_wxyz": [component / norm for component in quaternion],
    }


def _configuration_value(value: Any, context: str) -> Any:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise CatalogError(f"{context} must be finite numeric data")
        return float(value) if isinstance(value, float) else value
    if isinstance(value, str):
        if not value:
            raise CatalogError(f"{context} must not be empty")
        return value
    if isinstance(value, list):
        return [_configuration_value(item, f"{context}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, Mapping):
        return {
            _string(key, f"{context} key"): _configuration_value(item, f"{context}.{key}")
            for key, item in sorted(value.items())
        }
    raise CatalogError(f"{context} must be finite configuration data")


def _configuration(value: Any, context: str) -> dict[str, Any]:
    return {
        _string(key, f"{context} key"): _configuration_value(item, f"{context}.{key}")
        for key, item in sorted(_mapping(value, context).items())
    }


def _finite_behavior_value(value: Any, context: str) -> Any:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise CatalogError(f"{context} must be finite numeric data")
        return float(value)
    if isinstance(value, str):
        if not value:
            raise CatalogError(f"{context} must not be empty")
        return value
    if isinstance(value, list):
        return [_finite_behavior_value(item, f"{context}[{index}]") for index, item in enumerate(value)]
    raise CatalogError(f"{context} must be finite numeric, string, boolean, or a list of those values")


def _relative_path(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError as exc:
        raise CatalogError(f"path escapes repository root: {path}") from exc


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise CatalogError(f"cannot read YAML {path}: {exc}") from exc
    return _mapping(value, str(path))


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CatalogError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, Mapping):
        raise CatalogError(f"{path} must contain a JSON object")
    return value


@dataclass(frozen=True)
class PackageRecord:
    """One immutable package manifest discovered by the local catalog."""

    id: str
    version: str
    kind: str
    manifest_path: Path
    data: dict[str, Any]

    @property
    def ref(self) -> str:
        return f"{self.id}@{self.version}"


@dataclass(frozen=True)
class ResolvedSession:
    """One resolved simulation session."""

    session_id: str
    session: dict[str, Any]
    physics_plan: dict[str, Any]
    visual_plan: dict[str, Any]
    sensor_plan: dict[str, Any]
    control_plan: dict[str, Any]
    transport_intent: dict[str, Any]
    physics_json: str
    visual_json: str
    sensor_json: str
    control_json: str
    transport_json: str
    scenario_plan: dict[str, Any] | None = None
    scenario_json: str | None = None

    def write_bundle(self, bundle_dir: Path) -> Path:
        """Write session.yaml and the backend plans."""

        bundle_dir = Path(bundle_dir)
        bundle_dir.mkdir(parents=True, exist_ok=True)
        (bundle_dir / "session.yaml").write_text(
            yaml.safe_dump(self.session, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        (bundle_dir / "physics.plan.json").write_text(self.physics_json, encoding="utf-8")
        (bundle_dir / "visual.plan.json").write_text(self.visual_json, encoding="utf-8")
        (bundle_dir / "sensor.plan.json").write_text(self.sensor_json, encoding="utf-8")
        (bundle_dir / "control.plan.json").write_text(self.control_json, encoding="utf-8")
        (bundle_dir / "transport.intent.json").write_text(self.transport_json, encoding="utf-8")
        if self.scenario_json is not None:
            (bundle_dir / "scenario.plan.json").write_text(self.scenario_json, encoding="utf-8")
        return bundle_dir


class CatalogResolver:
    """Discover and resolve exact-version local simulation packages."""

    def __init__(self, repo_root: Path, catalog_roots: Sequence[Path]) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.catalog_roots = tuple(Path(root).resolve() for root in catalog_roots)
        self._records_by_path: dict[Path, PackageRecord] = {}
        self._records_by_ref: dict[tuple[str, str, str], PackageRecord] = {}
        self._scan_catalog()

    @classmethod
    def from_repository(cls, repo_root: Path) -> CatalogResolver:
        root = Path(repo_root).resolve()
        return cls(root, (root / "sim" / "packages",))

    def _scan_catalog(self) -> None:
        candidates: set[Path] = set()
        for root in self.catalog_roots:
            if root.is_dir():
                for suffix in _PACKAGE_SUFFIXES:
                    candidates.update(path.resolve() for path in root.rglob(f"*{suffix}"))
        for path in sorted(candidates):
            record = self._load_record(path)
            key = (record.kind, record.id, record.version)
            if key in self._records_by_ref:
                other = self._records_by_ref[key]
                raise CatalogError(
                    f"duplicate package {record.ref} ({record.kind}): {other.manifest_path} and {path}",
                    code=DiagnosticCode.DUPLICATE_IDENTITY,
                    context=record.ref,
                    details={
                        "kind": record.kind,
                        "first_manifest": str(other.manifest_path),
                        "second_manifest": str(path),
                    },
                )
            self._records_by_path[path] = record
            self._records_by_ref[key] = record

    def _load_record(self, path: Path) -> PackageRecord:
        if not path.is_file():
            raise CatalogError(f"package manifest does not exist: {path}")
        data = _load_yaml(path)
        missing = sorted({"schema", "id", "version", "kind"} - set(data))
        if missing:
            raise CatalogError(f"{path} is missing required key(s): {', '.join(missing)}")
        schema = _string(data["schema"], f"{path}.schema")
        kind = _SCHEMA_KIND.get(schema)
        if kind is None:
            raise CatalogError(f"{path}.schema is unsupported: {schema}")
        declared_kind = _string(data["kind"], f"{path}.kind")
        if declared_kind != kind:
            raise CatalogError(f"{path}.kind={declared_kind!r} does not match {schema}")
        record = PackageRecord(
            id=_string(data["id"], f"{path}.id"),
            version=_string(data["version"], f"{path}.version"),
            kind=kind,
            manifest_path=path.resolve(),
            data=data,
        )
        self._validate_package(record)
        return record

    def _validate_package(self, record: PackageRecord) -> None:
        data = record.data
        context = str(record.manifest_path)
        if record.kind == "robot":
            _keys(
                data,
                (
                    "schema",
                    "id",
                    "version",
                    "kind",
                    "physics",
                    "visual",
                    "semantic",
                    "frames",
                    "interfaces",
                    "defaults",
                    "declared_capabilities",
                ),
                ("description", "compatibility", "qualification"),
                context,
            )
            physics = _mapping(data["physics"], f"{context}.physics")
            _keys(
                physics,
                ("mjcf", "attach_root", "root_joint", "global_options"),
                ("initial_keyframe",),
                f"{context}.physics",
            )
            if physics["global_options"] != "inherit_session":
                raise CatalogError(
                    f"{context}.physics.global_options must be 'inherit_session'",
                    code=DiagnosticCode.GLOBAL_PHYSICS_OWNERSHIP,
                    context=f"{context}.physics.global_options",
                )
            mjcf = self._asset_path(record, physics["mjcf"], "physics.mjcf")
            mjcf_root = ET.parse(mjcf).getroot()
            option = mjcf_root.find("option")
            if option is not None:
                fields = sorted(option.attrib)
                if fields:
                    option_context = f"{context}.physics.mjcf.option"
                    raise CatalogError(
                        "attachable RobotPackage MJCF must inherit session-owned MuJoCo options",
                        code=DiagnosticCode.GLOBAL_PHYSICS_OWNERSHIP,
                        context=option_context,
                        details={"fields": fields, "required_policy": "inherit_session"},
                    )
            symbols = _xml_symbols(mjcf)
            for field in ("attach_root", "root_joint"):
                if physics[field] not in symbols:
                    raise CatalogError(f"{context}.physics.{field}={physics[field]!r} is absent from {mjcf}")
            if "initial_keyframe" in physics:
                initial_keyframe = _string(
                    physics["initial_keyframe"],
                    f"{context}.physics.initial_keyframe",
                )
                if initial_keyframe not in symbols:
                    raise CatalogError(f"{context}.physics.initial_keyframe={initial_keyframe!r} is absent from {mjcf}")
            semantic = _mapping(data["semantic"], f"{context}.semantic")
            _keys(semantic, ("class",), (), f"{context}.semantic")
            data["semantic"] = {"class": _string(semantic["class"], f"{context}.semantic.class")}
            visual = _mapping(data["visual"], f"{context}.visual")
            _keys(
                visual,
                ("binding", "projection"),
                ("source_mesh_root",),
                f"{context}.visual",
            )
            _string(visual["binding"], f"{context}.visual.binding")
            if "source_mesh_root" in visual:
                visual_mesh_root = self._require_safe_package_path(
                    visual["source_mesh_root"],
                    f"{context}.visual.source_mesh_root",
                )
                visual_mesh_root_path = (record.manifest_path.parent / visual_mesh_root).resolve()
                try:
                    visual_mesh_root_path.relative_to(record.manifest_path.parent.resolve())
                except ValueError as exc:
                    raise CatalogError(
                        f"{context}.visual.source_mesh_root escapes robot package",
                        code=DiagnosticCode.PATH_TRAVERSAL,
                        context=f"{context}.visual.source_mesh_root",
                    ) from exc
                if not visual_mesh_root_path.is_dir():
                    raise CatalogError(
                        f"{context}.visual.source_mesh_root does not exist: {visual_mesh_root}"
                    )
            data["frames"] = self._validate_frames(record, data["frames"], mjcf)
            interfaces = _mapping(data["interfaces"], f"{context}.interfaces")
            _keys(interfaces, ("state", "command"), (), f"{context}.interfaces")
            for name in ("state", "command"):
                values = _list(interfaces[name], f"{context}.interfaces.{name}")
                for index, value in enumerate(values):
                    _string(value, f"{context}.interfaces.{name}[{index}]")
            defaults = _mapping(data["defaults"], f"{context}.defaults")
            _keys(defaults, ("controller", "sensor_rig"), (), f"{context}.defaults")
            for name in ("controller", "sensor_rig"):
                if defaults[name] is not None:
                    _string(defaults[name], f"{context}.defaults.{name}")
            _mapping(data["declared_capabilities"], f"{context}.declared_capabilities")
            self._validate_robot_visual_projection(record)
            return

        if record.kind == "payload":
            _keys(
                data,
                (
                    "schema",
                    "id",
                    "version",
                    "kind",
                    "description",
                    "safety",
                    "compatibility",
                    "mount",
                    "physics",
                    "visual",
                    "frames",
                    "interfaces",
                    "declared_capabilities",
                    "provenance",
                ),
                ("qualification",),
                context,
            )
            _string(data["description"], f"{context}.description")

            safety = _mapping(data["safety"], f"{context}.safety")
            _keys(
                safety,
                ("mode", "fictional", "real_world_calibration"),
                (),
                f"{context}.safety",
            )
            if safety["mode"] != "simulation_only":
                raise CatalogError(f"{context}.safety.mode must be 'simulation_only'")
            if not isinstance(safety["fictional"], bool):
                raise CatalogError(f"{context}.safety.fictional must be boolean")
            if safety["real_world_calibration"] not in {"prohibited", "not_applicable"}:
                raise CatalogError(
                    f"{context}.safety.real_world_calibration is unsupported"
                )

            compatibility = _mapping(data["compatibility"], f"{context}.compatibility")
            _keys(
                compatibility,
                ("runtime_abi", "mujoco"),
                (),
                f"{context}.compatibility",
            )
            if compatibility["runtime_abi"] != "lingtu.sim.payload.v1":
                raise CatalogError(f"{context}.compatibility.runtime_abi is unsupported")
            _string(compatibility["mujoco"], f"{context}.compatibility.mujoco")

            mount = _mapping(data["mount"], f"{context}.mount")
            _keys(
                mount,
                ("required_parent_role", "root_frame"),
                (),
                f"{context}.mount",
            )
            if mount["required_parent_role"] != "payload_mount":
                raise CatalogError(
                    f"{context}.mount.required_parent_role must be 'payload_mount'"
                )
            root_frame = _string(mount["root_frame"], f"{context}.mount.root_frame")

            physics = _mapping(data["physics"], f"{context}.physics")
            _keys(
                physics,
                (
                    "mjcf",
                    "attach_root",
                    "global_options",
                    "authority",
                    "collision_representation",
                ),
                (),
                f"{context}.physics",
            )
            if physics["global_options"] != "inherit_session":
                raise CatalogError(
                    f"{context}.physics.global_options must be 'inherit_session'",
                    code=DiagnosticCode.GLOBAL_PHYSICS_OWNERSHIP,
                    context=f"{context}.physics.global_options",
                )
            if physics["authority"] != "mujoco":
                raise CatalogError(f"{context}.physics.authority must be 'mujoco'")
            if physics["collision_representation"] not in {
                "primitive_proxy",
                "convex_proxy",
            }:
                raise CatalogError(
                    f"{context}.physics.collision_representation is unsupported"
                )
            _, mjcf = self._package_content_reference(
                record,
                physics["mjcf"],
                f"{context}.physics.mjcf",
            )
            mjcf_root = ET.parse(mjcf).getroot()
            option = mjcf_root.find("option")
            if option is not None and option.attrib:
                raise CatalogError(
                    "attachable PayloadPackage MJCF must inherit session-owned MuJoCo options",
                    code=DiagnosticCode.GLOBAL_PHYSICS_OWNERSHIP,
                    context=f"{context}.physics.mjcf.option",
                    details={"fields": sorted(option.attrib)},
                )
            symbols = _xml_symbols(mjcf)
            attach_root = _string(
                physics["attach_root"],
                f"{context}.physics.attach_root",
            )
            if attach_root not in _xml_body_names(mjcf):
                raise CatalogError(
                    f"{context}.physics.attach_root={attach_root!r} is not an MJCF body"
                )
            if root_frame != attach_root:
                raise CatalogError(
                    f"{context}.mount.root_frame must equal physics.attach_root"
                )

            visual = _mapping(data["visual"], f"{context}.visual")
            _keys(
                visual,
                ("binding", "projection", "authority", "ue_collision"),
                (),
                f"{context}.visual",
            )
            _string(visual["binding"], f"{context}.visual.binding")
            if visual["authority"] != "mujoco":
                raise CatalogError(f"{context}.visual.authority must be 'mujoco'")
            if visual["ue_collision"] != "disabled":
                raise CatalogError(f"{context}.visual.ue_collision must be 'disabled'")

            data["frames"] = self._validate_frames(record, data["frames"], mjcf)
            if root_frame not in {frame["name"] for frame in data["frames"]}:
                raise CatalogError(f"{context}.mount.root_frame is absent from payload frames")
            if root_frame not in symbols:
                raise CatalogError(f"{context}.mount.root_frame is absent from payload MJCF")

            interfaces = _mapping(data["interfaces"], f"{context}.interfaces")
            _keys(interfaces, ("state", "command"), (), f"{context}.interfaces")
            for name in ("state", "command"):
                values = _list(interfaces[name], f"{context}.interfaces.{name}")
                for index, value in enumerate(values):
                    _string(value, f"{context}.interfaces.{name}[{index}]")

            capabilities = _list(
                data["declared_capabilities"],
                f"{context}.declared_capabilities",
            )
            if not capabilities:
                raise CatalogError(f"{context}.declared_capabilities must not be empty")
            for index, value in enumerate(capabilities):
                _string(value, f"{context}.declared_capabilities[{index}]")

            provenance = _mapping(data["provenance"], f"{context}.provenance")
            _keys(
                provenance,
                ("generator",),
                (
                    "source_task_id",
                    "conversion_task_id",
                    "segmentation_task_id",
                    "source_asset",
                    "conditioning_report",
                    "source_uri",
                    "license",
                ),
                f"{context}.provenance",
            )
            _string(provenance["generator"], f"{context}.provenance.generator")
            for name in (
                "source_task_id",
                "conversion_task_id",
                "segmentation_task_id",
                "source_uri",
                "license",
            ):
                if name in provenance:
                    _string(provenance[name], f"{context}.provenance.{name}")
            for name in ("source_asset", "conditioning_report"):
                if name in provenance:
                    self._package_content_reference(
                        record,
                        provenance[name],
                        f"{context}.provenance.{name}",
                    )
            self._validate_payload_visual_projection(record)
            return

        if record.kind == "controller":
            _keys(
                data,
                ("schema", "id", "version", "kind", "adapter", "policy", "timing", "robot_interface", "actuators"),
                ("description",),
                context,
            )
            adapter = _mapping(data["adapter"], f"{context}.adapter")
            _keys(adapter, ("plugin", "abi"), (), f"{context}.adapter")
            policy = _mapping(data["policy"], f"{context}.policy")
            _keys(policy, ("runtime", "artifact", "manifest"), (), f"{context}.policy")
            self._asset_path(record, policy["artifact"], "policy.artifact")
            self._asset_path(record, policy["manifest"], "policy.manifest")
            timing = _mapping(data["timing"], f"{context}.timing")
            _keys(timing, ("inference_hz", "low_level_hz"), (), f"{context}.timing")
            robot_interface = _mapping(data["robot_interface"], f"{context}.robot_interface")
            _keys(
                robot_interface,
                ("requires_state", "produces_command"),
                ("accepts_command",),
                f"{context}.robot_interface",
            )
            for value in _list(robot_interface["requires_state"], f"{context}.robot_interface.requires_state"):
                _string(value, f"{context}.robot_interface.requires_state item")
            if "accepts_command" in robot_interface:
                accepts = _mapping(robot_interface["accepts_command"], f"{context}.robot_interface.accepts_command")
                _keys(accepts, ("type",), ("message_type",), f"{context}.robot_interface.accepts_command")
                _string(accepts["type"], f"{context}.robot_interface.accepts_command.type")
                if "message_type" in accepts:
                    _string(accepts["message_type"], f"{context}.robot_interface.accepts_command.message_type")
            elif adapter["plugin"] == "quadruped_him":
                raise CatalogError(
                    f"{context}.robot_interface.accepts_command is required for quadruped_him"
                )
            produces = _mapping(robot_interface["produces_command"], f"{context}.robot_interface.produces_command")
            _keys(produces, ("type",), ("message_type",), f"{context}.robot_interface.produces_command")
            _string(produces["type"], f"{context}.robot_interface.produces_command.type")
            if "message_type" in produces:
                _string(produces["message_type"], f"{context}.robot_interface.produces_command.message_type")
            if adapter["plugin"] == "quadruped_him":
                accepts = _mapping(robot_interface["accepts_command"], f"{context}.robot_interface.accepts_command")
                if accepts["type"] != "base_twist":
                    raise CatalogError(
                        f"{context}.robot_interface.accepts_command.type must be base_twist for quadruped_him"
                    )
                if accepts.get("message_type") != "lingtu.dds.FinalVelocityCommand":
                    raise CatalogError(
                        f"{context}.robot_interface.accepts_command.message_type must be "
                        "lingtu.dds.FinalVelocityCommand for quadruped_him"
                    )
                if produces["type"] != "joint_torque":
                    raise CatalogError(
                        f"{context}.robot_interface.produces_command.type must be joint_torque "
                        "for quadruped_him"
                    )
            actuators = _mapping(data["actuators"], f"{context}.actuators")
            _keys(actuators, ("channels",), (), f"{context}.actuators")
            channels = _list(actuators["channels"], f"{context}.actuators.channels")
            if not channels:
                raise CatalogError(f"{context}.actuators.channels must not be empty")
            for index, value in enumerate(channels):
                _string(value, f"{context}.actuators.channels[{index}]")
            return

        if record.kind == "sensor":
            _keys(
                data,
                (
                    "schema",
                    "id",
                    "version",
                    "kind",
                    "sensor_type",
                    "outputs",
                    "timing",
                    "interface",
                    "configuration",
                ),
                ("description", "noise"),
                context,
            )
            _string(data["sensor_type"], f"{context}.sensor_type")
            outputs = _list(data["outputs"], f"{context}.outputs")
            if not outputs:
                raise CatalogError(f"{context}.outputs must not be empty")
            for index, value in enumerate(outputs):
                _string(value, f"{context}.outputs[{index}]")
            timing = _mapping(data["timing"], f"{context}.timing")
            _keys(timing, ("rate_hz",), (), f"{context}.timing")
            interface = _mapping(data["interface"], f"{context}.interface")
            _keys(interface, ("transport", "message_type"), (), f"{context}.interface")
            _mapping(data["configuration"], f"{context}.configuration")
            return

        if record.kind == "sensor_rig":
            _keys(data, ("schema", "id", "version", "kind", "sensors"), ("description",), context)
            sensors = _list(data["sensors"], f"{context}.sensors")
            seen: set[str] = set()
            for index, item in enumerate(sensors):
                sensor = _mapping(item, f"{context}.sensors[{index}]")
                _keys(
                    sensor,
                    ("id", "package", "parent_frame", "runtime"),
                    ("frequency_hz", "extrinsic", "configuration"),
                    f"{context}.sensors[{index}]",
                )
                sensor_id = _string(sensor["id"], f"{context}.sensors[{index}].id")
                if sensor_id in seen:
                    raise CatalogError(f"{context}.sensors contains duplicate id {sensor_id!r}")
                seen.add(sensor_id)
                _string(sensor["package"], f"{context}.sensors[{index}].package")
                _string(sensor["parent_frame"], f"{context}.sensors[{index}].parent_frame")
                runtime = _mapping(sensor["runtime"], f"{context}.sensors[{index}].runtime")
                _keys(runtime, ("owner", "source"), (), f"{context}.sensors[{index}].runtime")
                _string(runtime["owner"], f"{context}.sensors[{index}].runtime.owner")
                _string(runtime["source"], f"{context}.sensors[{index}].runtime.source")
                if "frequency_hz" in sensor:
                    frequency = sensor["frequency_hz"]
                    if (
                        isinstance(frequency, bool)
                        or not isinstance(frequency, (int, float))
                        or not math.isfinite(float(frequency))
                        or float(frequency) <= 0
                    ):
                        raise CatalogError(
                            f"{context}.sensors[{index}].frequency_hz must be positive finite numeric data"
                        )
                if "extrinsic" in sensor:
                    _extrinsic(sensor["extrinsic"], f"{context}.sensors[{index}].extrinsic")
                if "configuration" in sensor:
                    _configuration(sensor["configuration"], f"{context}.sensors[{index}].configuration")
            return

        if record.kind == "world":
            canonical = self._world_manifest_is_canonical(data)
            _keys(
                data,
                (
                    "schema",
                    "id",
                    "version",
                    "kind",
                    "physics",
                    "visual",
                    "entities",
                    *(("provenance", "content") if canonical else ()),
                ),
                ("description",),
                context,
            )
            if canonical and _REF_RE.fullmatch(f"{record.id}@{record.version}") is None:
                raise CatalogError(f"{context} has an unsafe canonical package identity")
            physics = _mapping(data["physics"], f"{context}.physics")
            _keys(physics, ("mjcf", "global_policy"), (), f"{context}.physics")
            self._reject_colon_in_path(physics["mjcf"], f"{context}.physics.mjcf")
            mjcf = self._asset_path(record, physics["mjcf"], "physics.mjcf")
            _xml_symbols(mjcf)
            data["physics"]["global_policy"] = _world_global_policy(
                physics["global_policy"],
                mjcf,
                f"{context}.physics.global_policy",
            )
            visual = _mapping(data["visual"], f"{context}.visual")
            _keys(
                visual,
                ("binding", "level", *(("projection",) if canonical else ())),
                (),
                f"{context}.visual",
            )
            _string(visual["binding"], f"{context}.visual.binding")
            data["visual"]["level"] = _unreal_game_package_path(
                visual["level"],
                f"{context}.visual.level",
            )
            entities = _list(data["entities"], f"{context}.entities")
            for index, entity in enumerate(entities):
                if canonical:
                    self._validate_world_entity(entity, f"{context}.entities[{index}]")
                else:
                    legacy_entity = _mapping(entity, f"{context}.entities[{index}]")
                    if "source" in legacy_entity:
                        self._reject_colon_in_path(
                            legacy_entity["source"],
                            f"{context}.entities[{index}].source",
                        )
            if canonical:
                self._validate_world_package_content(record)
            return

        if record.kind == "scenario":
            _keys(data, ("schema", "id", "version", "kind", "world", "entities"), ("description",), context)
            _string(data["world"], f"{context}.world")
            entities = _list(data["entities"], f"{context}.entities")
            if not entities:
                raise CatalogError(f"{context}.entities must not be empty")
            seen_entity_ids: set[str] = set()
            for index, item in enumerate(entities):
                entity = _mapping(item, f"{context}.entities[{index}]")
                entity_context = f"{context}.entities[{index}]"
                _keys(
                    entity,
                    ("entity_id", "entity_type", "authority", "initial_transform", "physics_proxy", "semantic_class"),
                    ("behavior", "visual"),
                    entity_context,
                )
                entity_id = _string(entity["entity_id"], f"{entity_context}.entity_id")
                if entity_id in seen_entity_ids:
                    raise CatalogError(f"{context}.entities contains duplicate entity_id {entity_id!r}")
                seen_entity_ids.add(entity_id)
                entity_type = _string(entity["entity_type"], f"{entity_context}.entity_type")
                if entity_type not in {"pedestrian", "vehicle", "door", "elevator", "obstacle", "terrain", "prop"}:
                    raise CatalogError(f"{entity_context}.entity_type is unsupported: {entity_type!r}")
                authority = _string(entity["authority"], f"{entity_context}.authority")
                if authority not in {"scenario", "ue_animation", "mujoco"}:
                    raise CatalogError(f"{entity_context}.authority is unsupported: {authority!r}")
                transform = _mapping(entity["initial_transform"], f"{entity_context}.initial_transform")
                _keys(transform, ("position_m", "quaternion_wxyz"), (), f"{entity_context}.initial_transform")
                _finite_vector(transform["position_m"], 3, f"{entity_context}.initial_transform.position_m")
                _finite_vector(
                    transform["quaternion_wxyz"],
                    4,
                    f"{entity_context}.initial_transform.quaternion_wxyz",
                )
                physics_proxy = self._scenario_physics_proxy(
                    record,
                    entity["physics_proxy"],
                    f"{entity_context}.physics_proxy",
                )
                if physics_proxy["mode"] == "kinematic" and authority != "scenario":
                    raise CatalogError(
                        f"{entity_context}.physics_proxy kinematic mode requires scenario authority"
                    )
                if "visual" in entity:
                    self._validate_scenario_entity_visual_projection(
                        record,
                        entity,
                        physics_proxy,
                        entity_context,
                    )
                _string(entity["semantic_class"], f"{entity_context}.semantic_class")
                if "behavior" in entity:
                    behavior = _mapping(entity["behavior"], f"{entity_context}.behavior")
                    _keys(behavior, ("profile", "seed", "parameters"), (), f"{entity_context}.behavior")
                    _string(behavior["profile"], f"{entity_context}.behavior.profile")
                    seed = behavior["seed"]
                    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
                        raise CatalogError(f"{entity_context}.behavior.seed must be a non-negative integer")
                    parameters = _mapping(behavior["parameters"], f"{entity_context}.behavior.parameters")
                    if not parameters:
                        raise CatalogError(f"{entity_context}.behavior.parameters must not be empty")
                    for key, value in parameters.items():
                        _string(key, f"{entity_context}.behavior.parameters key")
                        _finite_behavior_value(value, f"{entity_context}.behavior.parameters.{key}")
            return

        raise CatalogError(f"unsupported package kind: {record.kind}")

    def _world_manifest_is_canonical(self, data: Mapping[str, Any]) -> bool:
        visual = data.get("visual")
        return (
            "provenance" in data
            or "content" in data
            or (isinstance(visual, Mapping) and "projection" in visual)
        )

    def _require_safe_package_path(self, value: Any, context: str) -> str:
        path = _string(value, context)
        parts = path.split("/")
        if (
            not parts
            or any(not part or part in {".", ".."} for part in parts)
            or "\\" in path
            or path.startswith("/")
            or re.match(r"^[A-Za-z]:", path) is not None
            or ":" in path
            or any(character.isspace() for character in path)
        ):
            raise CatalogError(
                f"{context} must be a safe package-relative POSIX path",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=context,
            )
        return path

    @staticmethod
    def _reject_colon_in_path(value: Any, context: str) -> str:
        """Reject NTFS drive/alternate-stream syntax without changing legacy roots."""

        path = _string(value, context)
        if ":" in path:
            raise CatalogError(
                f"{context} must not contain ':'",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=context,
            )
        return path

    def _package_content_reference(
        self,
        record: PackageRecord,
        value: Any,
        context: str,
    ) -> tuple[str, Path]:
        reference = _mapping(value, context)
        _keys(reference, ("path",), (), context)
        relative = self._require_safe_package_path(reference["path"], f"{context}.path")
        path = self._package_relative_file(record, relative, f"{context}.path")
        return relative, path

    def _validate_world_entity(self, value: Any, context: str) -> None:
        entity = _mapping(value, context)
        _keys(
            entity,
            (
                "entity_id",
                "entity_type",
                "authority",
                "initial_transform",
                "physics_proxy",
                "semantic_class",
                "collision",
            ),
            ("geometry", "visual"),
            context,
        )
        _string(entity["entity_id"], f"{context}.entity_id")
        _string(entity["entity_type"], f"{context}.entity_type")
        if entity["authority"] not in {"scenario", "ue_animation", "mujoco"}:
            raise CatalogError(f"{context}.authority is unsupported: {entity['authority']!r}")
        transform = _mapping(entity["initial_transform"], f"{context}.initial_transform")
        _keys(transform, ("position_m", "quaternion_wxyz"), (), f"{context}.initial_transform")
        _finite_vector(transform["position_m"], 3, f"{context}.initial_transform.position_m")
        _normalized_quaternion(transform["quaternion_wxyz"], f"{context}.initial_transform.quaternion_wxyz")
        if entity["physics_proxy"] not in {"mujoco", "kinematic", "none"}:
            raise CatalogError(f"{context}.physics_proxy is unsupported: {entity['physics_proxy']!r}")
        _string(entity["semantic_class"], f"{context}.semantic_class")
        if not isinstance(entity["collision"], bool):
            raise CatalogError(f"{context}.collision must be boolean")
        if "visual" in entity:
            self._validate_world_entity_visual(entity["visual"], f"{context}.visual")
        geometry_value = entity.get("geometry")
        if geometry_value is None:
            return
        geometry = _mapping(geometry_value, f"{context}.geometry")
        shape = _string(geometry.get("shape"), f"{context}.geometry.shape")
        if shape == "box":
            _keys(geometry, ("shape", "size_m"), (), f"{context}.geometry")
            size = _finite_vector(geometry["size_m"], 3, f"{context}.geometry.size_m")
            if any(component <= 0.0 for component in size):
                raise CatalogError(f"{context}.geometry.size_m must contain positive values")
            return
        if shape == "cylinder":
            _keys(
                geometry,
                ("shape", "radius_m", "half_height_m"),
                (),
                f"{context}.geometry",
            )
            for field in ("radius_m", "half_height_m"):
                value = geometry[field]
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    or float(value) <= 0.0
                ):
                    raise CatalogError(f"{context}.geometry.{field} must be positive finite numeric data")
            return
        raise CatalogError(f"{context}.geometry.shape is unsupported: {shape!r}")

    @staticmethod
    def _validate_world_entity_visual(value: Any, context: str) -> None:
        visual = _mapping(value, context)
        mode = _string(visual.get("mode"), f"{context}.mode")
        if mode == "level":
            _keys(visual, ("mode",), (), context)
            return
        if mode != "runtime":
            raise CatalogError(f"{context}.mode is unsupported: {mode!r}")
        _keys(visual, ("mode", "material"), (), context)
        material = _mapping(visual["material"], f"{context}.material")
        _keys(
            material,
            ("key", "base_color_rgba", "metallic", "roughness"),
            (),
            f"{context}.material",
        )
        _string(material["key"], f"{context}.material.key")
        rgba = _finite_vector(
            material["base_color_rgba"],
            4,
            f"{context}.material.base_color_rgba",
        )
        if any(component < 0.0 or component > 1.0 for component in rgba):
            raise CatalogError(f"{context}.material.base_color_rgba must stay in [0, 1]")
        for field in ("metallic", "roughness"):
            numeric = material[field]
            if (
                isinstance(numeric, bool)
                or not isinstance(numeric, (int, float))
                or not math.isfinite(float(numeric))
                or float(numeric) < 0.0
                or float(numeric) > 1.0
            ):
                raise CatalogError(f"{context}.material.{field} must stay in [0, 1]")

    def _validate_world_package_content(self, record: PackageRecord) -> dict[str, str]:
        context = str(record.manifest_path)
        data = record.data
        visual = _mapping(data["visual"], f"{context}.visual")
        provenance_relative, _provenance_path = self._package_content_reference(
            record,
            data["provenance"],
            f"{context}.provenance",
        )
        projection_relative = self._require_safe_package_path(
            visual["projection"],
            f"{context}.visual.projection",
        )
        projection_path = self._package_relative_file(
            record,
            projection_relative,
            f"{context}.visual.projection",
        )
        content = _mapping(data["content"], f"{context}.content")
        _keys(
            content,
            ("files", "provenance", "visual_projection"),
            (),
            f"{context}.content",
        )
        declared_provenance = _mapping(content["provenance"], f"{context}.content.provenance")
        if declared_provenance.get("path") != provenance_relative:
            raise CatalogError(f"{context}.content.provenance does not match package provenance")
        for index, value in enumerate(_list(content["files"], f"{context}.content.files")):
            item = _mapping(value, f"{context}.content.files[{index}]")
            self._package_relative_file(record, item.get("path"), f"{context}.content.files[{index}].path")
        projection_reference = _mapping(content["visual_projection"], f"{context}.content.visual_projection")
        _keys(
            projection_reference,
            ("path",),
            (),
            f"{context}.content.visual_projection",
        )
        if projection_reference["path"] != projection_relative:
            raise CatalogError(f"{context}.content.visual_projection.path does not match visual.projection")
        return self._validate_world_visual_projection(
            record,
            projection_path=projection_path,
            projection_relative=projection_relative,
            provenance_relative=provenance_relative,
        )

    def _validate_world_visual_projection(
        self,
        record: PackageRecord,
        *,
        projection_path: Path,
        projection_relative: str,
        provenance_relative: str,
    ) -> dict[str, str]:
        context = f"{record.manifest_path}.visual.projection"
        projection = _load_json(projection_path)
        _keys(
            projection,
            (
                "schema",
                "package",
                "binding",
                "level",
                "units",
                "terrain",
                "entities",
                "spawn_alignment",
            ),
            (),
            context,
        )
        if projection["schema"] != "lingtu.sim.world-visual-projection.v1":
            raise CatalogError(f"{context}.schema is unsupported: {projection['schema']!r}")
        package = _mapping(projection["package"], f"{context}.package")
        _keys(package, ("id", "version", "manifest", "provenance"), (), f"{context}.package")
        manifest_relative = record.manifest_path.relative_to(record.manifest_path.parent).as_posix()
        if package != {
            "id": record.id,
            "version": record.version,
            "manifest": manifest_relative,
            "provenance": provenance_relative,
        }:
            raise CatalogError(f"{context}.package does not match the world manifest")
        binding = _string(projection["binding"], f"{context}.binding")
        visual = _mapping(record.data["visual"], f"{record.manifest_path}.visual")
        if binding != visual["binding"]:
            raise CatalogError(f"{context}.binding does not match visual.binding")
        level = _unreal_game_package_path(projection["level"], f"{context}.level")
        if level != visual["level"]:
            raise CatalogError(f"{context}.level does not match visual.level")
        units = _mapping(projection["units"], f"{context}.units")
        if units != {"length": "m", "up_axis": "Z", "handedness": "RH"}:
            raise CatalogError(f"{context}.units is unsupported")
        terrain = _mapping(projection["terrain"], f"{context}.terrain")
        _keys(
            terrain,
            ("grid_px", "extent_m", "sample_spacing_m", "physics_bounds_m", "visual_bounds_m", "assets"),
            (),
            f"{context}.terrain",
        )
        grid = _finite_vector(terrain["grid_px"], 2, f"{context}.terrain.grid_px")
        if any(value < 2 or value != int(value) for value in grid):
            raise CatalogError(f"{context}.terrain.grid_px must contain integers >= 2")
        _finite_vector(terrain["extent_m"], 2, f"{context}.terrain.extent_m")
        _finite_vector(terrain["sample_spacing_m"], 2, f"{context}.terrain.sample_spacing_m")
        for bounds_name in ("physics_bounds_m", "visual_bounds_m"):
            bounds = _mapping(terrain[bounds_name], f"{context}.terrain.{bounds_name}")
            _keys(bounds, ("min_m", "max_m"), (), f"{context}.terrain.{bounds_name}")
            _finite_vector(bounds["min_m"], 3, f"{context}.terrain.{bounds_name}.min_m")
            _finite_vector(bounds["max_m"], 3, f"{context}.terrain.{bounds_name}.max_m")
        assets = _list(terrain["assets"], f"{context}.terrain.assets")
        if len(assets) < 3:
            raise CatalogError(f"{context}.terrain.assets must contain at least three assets")
        for index, item in enumerate(assets):
            asset_context = f"{context}.terrain.assets[{index}]"
            asset = _mapping(item, asset_context)
            _keys(asset, ("role", "path", "bytes"), ("collision",), asset_context)
            _string(asset["role"], f"{asset_context}.role")
            self._package_relative_file(record, asset["path"], f"{asset_context}.path")
            size = asset["bytes"]
            if isinstance(size, bool) or not isinstance(size, int) or size < 1:
                raise CatalogError(f"{asset_context}.bytes must be a positive integer")
            if "collision" in asset and not isinstance(asset["collision"], bool):
                raise CatalogError(f"{asset_context}.collision must be boolean")
        self._validate_world_projection_entities(
            record,
            projection["entities"],
            f"{context}.entities",
        )
        spawn = _mapping(projection["spawn_alignment"], f"{context}.spawn_alignment")
        _keys(spawn, ("position_m", "aligned_to_heightmap"), (), f"{context}.spawn_alignment")
        _finite_vector(spawn["position_m"], 3, f"{context}.spawn_alignment.position_m")
        if spawn["aligned_to_heightmap"] is not True:
            raise CatalogError(f"{context}.spawn_alignment.aligned_to_heightmap must be true")
        return {
            "schema": projection["schema"],
            "path": _relative_path(projection_path, self.repo_root),
        }

    def _validate_world_projection_entities(
        self,
        record: PackageRecord,
        value: Any,
        context: str,
    ) -> None:
        projected = _list(value, context)
        source_entities = [
            _mapping(item, f"{record.manifest_path}.entities")
            for item in record.data["entities"]
            if _mapping(item, f"{record.manifest_path}.entities").get("visual", {}).get("mode")
            == "runtime"
        ]
        expected_by_id = {entity["entity_id"]: entity for entity in source_entities}
        projected_ids: list[str] = []
        for index, item in enumerate(projected):
            entity_context = f"{context}[{index}]"
            entity = _mapping(item, entity_context)
            _keys(
                entity,
                (
                    "entity_id",
                    "semantic_class",
                    "authority",
                    "transform",
                    "geometry",
                    "unreal",
                    "material",
                ),
                (),
                entity_context,
            )
            entity_id = _string(entity["entity_id"], f"{entity_context}.entity_id")
            if entity_id in projected_ids:
                raise CatalogError(f"{context} contains duplicate entity_id {entity_id!r}")
            projected_ids.append(entity_id)
            source = expected_by_id.get(entity_id)
            if source is None:
                raise CatalogError(f"{entity_context} does not belong to a runtime WorldPackage entity")
            if entity["semantic_class"] != source["semantic_class"] or entity["authority"] != source["authority"]:
                raise CatalogError(f"{entity_context} semantic identity does not match the WorldPackage")
            if entity["transform"] != source["initial_transform"]:
                raise CatalogError(f"{entity_context}.transform does not match the WorldPackage")
            geometry = source.get("geometry") or {
                "shape": "box",
                "size_m": [1.0, 1.0, 1.0],
            }
            if entity["geometry"] != geometry:
                raise CatalogError(f"{entity_context}.geometry does not match the WorldPackage")
            if geometry["shape"] == "box":
                dimensions = [float(component) for component in geometry["size_m"]]
                asset_path = "/Engine/BasicShapes/Cube.Cube"
            else:
                diameter = 2.0 * float(geometry["radius_m"])
                dimensions = [diameter, diameter, 2.0 * float(geometry["half_height_m"])]
                asset_path = "/Engine/BasicShapes/Cylinder.Cylinder"
            expected_unreal = {
                "representation": "static_mesh",
                "component_class": "/Script/Engine.StaticMeshComponent",
                "asset_path": asset_path,
                "dimensions_m": dimensions,
            }
            if entity["unreal"] != expected_unreal:
                raise CatalogError(f"{entity_context}.unreal does not match primitive geometry")
            source_material = _mapping(source["visual"]["material"], f"{entity_context}.source_material")
            expected_material = {
                "source": "world_package",
                "key": source_material["key"],
                "pbr": {
                    "base_color_rgba": source_material["base_color_rgba"],
                    "metallic": source_material["metallic"],
                    "roughness": source_material["roughness"],
                },
            }
            if entity["material"] != expected_material:
                raise CatalogError(f"{entity_context}.material does not match the WorldPackage")
        expected_ids = sorted(expected_by_id)
        if projected_ids != expected_ids:
            raise CatalogError(
                f"{context} must contain every runtime entity in deterministic identity order"
            )

    def _validate_frames(self, record: PackageRecord, value: Any, mjcf: Path) -> list[dict[str, Any]]:
        context = f"{record.manifest_path}.frames"
        frames = _list(value, context)
        if not frames:
            raise CatalogError(f"{context} must not be empty")
        symbols = _xml_symbols(mjcf)
        parents: dict[str, str | None] = {}
        normalized_frames: list[dict[str, Any]] = []
        for index, item in enumerate(frames):
            frame = _mapping(item, f"{context}[{index}]")
            _keys(frame, ("name", "role"), ("parent", "parent_frame", "extrinsic"), f"{context}[{index}]")
            name = _string(frame["name"], f"{context}[{index}].name")
            if name in parents:
                raise CatalogError(f"{context} contains duplicate frame {name!r}")
            if name not in symbols:
                raise CatalogError(f"{context}[{index}].name={name!r} is absent from {mjcf}")
            if "parent" in frame and "parent_frame" in frame:
                raise CatalogError(f"{context}[{index}] must not mix parent and parent_frame")
            parent_frame = None
            if "parent_frame" in frame:
                parent_frame = _string(frame["parent_frame"], f"{context}[{index}].parent_frame")
            elif "parent" in frame:
                parent_frame = _string(frame["parent"], f"{context}[{index}].parent")
            normalized_frame: dict[str, Any] = {
                "name": name,
                "role": _string(frame["role"], f"{context}[{index}].role"),
            }
            if parent_frame is not None:
                normalized_frame["parent_frame"] = parent_frame
            if "extrinsic" in frame:
                normalized_frame["extrinsic"] = _extrinsic(frame["extrinsic"], f"{context}[{index}].extrinsic")
            parents[name] = parent_frame
            normalized_frames.append(normalized_frame)
        for name, parent_frame in parents.items():
            if parent_frame is not None and parent_frame not in parents:
                raise CatalogError(f"{context} frame {name!r} has unknown parent_frame {parent_frame!r}")
        for name in parents:
            visiting: set[str] = set()
            current: str | None = name
            while current is not None:
                if current in visiting:
                    raise CatalogError(f"{context} contains a parent_frame cycle involving {current!r}")
                visiting.add(current)
                current = parents[current]
        return normalized_frames

    def _asset_path(self, record: PackageRecord, value: Any, field: str) -> Path:
        relative = _string(value, f"{record.manifest_path}.{field}")
        self._reject_colon_in_path(relative, f"{record.manifest_path}.{field}")
        candidate = (record.manifest_path.parent / relative).resolve()
        try:
            candidate.relative_to(self.repo_root)
        except ValueError as exc:
            raise CatalogError(
                f"{record.manifest_path}.{field} escapes repository root",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=f"{record.manifest_path}.{field}",
            ) from exc
        if not candidate.is_file():
            raise CatalogError(f"{record.manifest_path}.{field} does not exist: {relative}")
        return candidate

    def _package_relative_file(self, record: PackageRecord, value: Any, field: str) -> Path:
        relative = _string(value, f"{record.manifest_path}.{field}")
        parts = relative.split("/")
        if (
            not parts
            or any(not part or part in {".", ".."} for part in parts)
            or "\\" in relative
            or relative.startswith("/")
            or re.match(r"^[A-Za-z]:", relative) is not None
            or ":" in relative
            or any(char.isspace() for char in relative)
        ):
            raise CatalogError(
                f"{record.manifest_path}.{field} must be a safe package-relative POSIX path",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=f"{record.manifest_path}.{field}",
            )
        package_dir = record.manifest_path.parent.resolve()
        raw_candidate = package_dir / Path(*parts)
        if raw_candidate.is_symlink():
            raise CatalogError(
                f"{record.manifest_path}.{field} must identify a regular package file",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=f"{record.manifest_path}.{field}",
            )
        candidate = raw_candidate.resolve()
        try:
            candidate.relative_to(package_dir)
        except ValueError as exc:
            raise CatalogError(
                f"{record.manifest_path}.{field} escapes robot package",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=f"{record.manifest_path}.{field}",
            ) from exc
        if not candidate.is_file():
            raise CatalogError(f"{record.manifest_path}.{field} does not exist: {relative}")
        return candidate

    def _validate_robot_visual_projection(self, record: PackageRecord) -> dict[str, str]:
        context = f"{record.manifest_path}.visual.projection"
        projection_path = self._package_relative_file(
            record,
            _mapping(record.data["visual"], f"{record.manifest_path}.visual")["projection"],
            "visual.projection",
        )
        projection = _mapping(_load_json(projection_path), context)
        _keys(
            projection,
            ("schema", "binding", "package", "mjcf", "components"),
            (),
            context,
        )
        if projection["schema"] != "lingtu.sim.robot-visual-projection.v1":
            raise CatalogError(f"{context}.schema is unsupported")
        expected_binding = _string(
            _mapping(record.data["visual"], f"{record.manifest_path}.visual")["binding"],
            f"{record.manifest_path}.visual.binding",
        )
        if projection["binding"] != expected_binding:
            raise CatalogError(f"{context}.binding does not match RobotPackage visual.binding")
        package = _mapping(projection["package"], f"{context}.package")
        if package.get("id") != record.id or package.get("version") != record.version:
            raise CatalogError(f"{context}.package does not match RobotPackage identity")
        _mapping(projection["mjcf"], f"{context}.mjcf")
        if not _list(projection["components"], f"{context}.components"):
            raise CatalogError(f"{context}.components must not be empty")
        return {
            "schema": projection["schema"],
            "path": _relative_path(projection_path, self.repo_root),
        }

    def _validate_payload_visual_projection(self, record: PackageRecord) -> dict[str, str]:
        context = f"{record.manifest_path}.visual.projection"
        visual = _mapping(record.data["visual"], f"{record.manifest_path}.visual")
        _, projection_path = self._package_content_reference(
            record,
            visual["projection"],
            context,
        )
        projection = _mapping(_load_json(projection_path), str(projection_path))
        _keys(
            projection,
            (
                "schema",
                "package",
                "binding",
                "authority",
                "visual_policy",
                "source_asset",
                "material_contract",
                "components",
            ),
            (),
            str(projection_path),
        )
        if projection["schema"] != "lingtu.sim.payload-visual-projection.v1":
            raise CatalogError(f"{context}.schema is unsupported")
        package = _mapping(projection["package"], f"{context}.package")
        _keys(package, ("id", "version", "manifest"), (), f"{context}.package")
        expected_package = {
            "id": record.id,
            "version": record.version,
            "manifest": record.manifest_path.name,
        }
        if package != expected_package:
            raise CatalogError(f"{context}.package does not match PayloadPackage identity")
        if projection["binding"] != visual["binding"]:
            raise CatalogError(f"{context}.binding does not match PayloadPackage visual.binding")
        if projection["authority"] != "mujoco":
            raise CatalogError(f"{context}.authority must be 'mujoco'")

        policy = _mapping(projection["visual_policy"], f"{context}.visual_policy")
        _keys(
            policy,
            ("collision_enabled", "simulate_physics", "generate_overlap_events"),
            (),
            f"{context}.visual_policy",
        )
        if policy != {
            "collision_enabled": "no_collision",
            "simulate_physics": False,
            "generate_overlap_events": False,
        }:
            raise CatalogError(f"{context}.visual_policy must be visual-only")

        _, source_path = self._package_content_reference(
            record,
            projection["source_asset"],
            f"{context}.source_asset",
        )
        provenance = _mapping(
            record.data["provenance"],
            f"{record.manifest_path}.provenance",
        )
        if "source_asset" in provenance and projection["source_asset"] != provenance["source_asset"]:
            raise CatalogError(f"{context}.source_asset does not match PayloadPackage provenance")

        material_contract = _mapping(
            projection["material_contract"],
            f"{context}.material_contract",
        )
        _keys(
            material_contract,
            ("source_format", "material_count", "required_channels"),
            (),
            f"{context}.material_contract",
        )
        if material_contract["source_format"] != "gltf2":
            raise CatalogError(f"{context}.material_contract.source_format must be 'gltf2'")
        material_count = material_contract["material_count"]
        if isinstance(material_count, bool) or not isinstance(material_count, int) or material_count < 1:
            raise CatalogError(f"{context}.material_contract.material_count must be positive")
        required_channels = _list(
            material_contract["required_channels"],
            f"{context}.material_contract.required_channels",
        )
        expected_channels = ["base_color", "normal", "metallic_roughness"]
        if required_channels != expected_channels:
            raise CatalogError(
                f"{context}.material_contract.required_channels must be {expected_channels!r}"
            )
        actual_materials = _glb_material_channels(source_path)
        if len(actual_materials) != material_count:
            raise CatalogError(f"{context}.material_contract.material_count does not match GLB")
        for index, channels in enumerate(actual_materials):
            missing = sorted(set(expected_channels) - channels)
            if missing:
                raise CatalogError(
                    f"{context}.source_asset material {index} is missing PBR channel(s): "
                    f"{', '.join(missing)}"
                )

        frames = {
            frame["name"]
            for frame in _list(record.data["frames"], f"{record.manifest_path}.frames")
        }
        components = _list(projection["components"], f"{context}.components")
        if not components:
            raise CatalogError(f"{context}.components must not be empty")
        seen_meshes: set[str] = set()
        for index, value in enumerate(components):
            component_context = f"{context}.components[{index}]"
            component = _mapping(value, component_context)
            _keys(
                component,
                ("mesh", "body_frame", "unreal_asset", "local_transform"),
                (),
                component_context,
            )
            mesh = _string(component["mesh"], f"{component_context}.mesh")
            if mesh in seen_meshes:
                raise CatalogError(f"{context}.components contains duplicate mesh {mesh!r}")
            seen_meshes.add(mesh)
            body_frame = _string(
                component["body_frame"],
                f"{component_context}.body_frame",
            )
            if body_frame not in frames:
                raise CatalogError(
                    f"{component_context}.body_frame is absent from PayloadPackage frames"
                )
            unreal_asset = _string(
                component["unreal_asset"],
                f"{component_context}.unreal_asset",
            )
            if (
                not unreal_asset.startswith("/Game/")
                or "." not in unreal_asset.rsplit("/", 1)[-1]
                or "\\" in unreal_asset
                or any(character.isspace() for character in unreal_asset)
            ):
                raise CatalogError(
                    f"{component_context}.unreal_asset must be a cooked /Game object path"
                )
            transform = _mapping(
                component["local_transform"],
                f"{component_context}.local_transform",
            )
            _keys(
                transform,
                ("position_m", "quaternion_wxyz", "scale"),
                (),
                f"{component_context}.local_transform",
            )
            _finite_vector(
                transform["position_m"],
                3,
                f"{component_context}.local_transform.position_m",
            )
            _normalized_quaternion(
                transform["quaternion_wxyz"],
                f"{component_context}.local_transform.quaternion_wxyz",
            )
            scale = _finite_vector(
                transform["scale"],
                3,
                f"{component_context}.local_transform.scale",
            )
            if any(component_scale <= 0.0 for component_scale in scale):
                raise CatalogError(f"{component_context}.local_transform.scale must be positive")

        return {
            "schema": projection["schema"],
            "path": _relative_path(projection_path, self.repo_root),
        }

    def _resolve_package(self, reference: Any, expected_kind: str, base_dir: Path) -> PackageRecord:
        ref = _string(reference, f"{expected_kind} package reference")
        path_candidate = (base_dir / ref).resolve()
        if path_candidate.suffix in {".yaml", ".yml"} or "/" in ref or "\\" in ref:
            try:
                path_candidate.relative_to(self.repo_root)
            except ValueError as exc:
                raise CatalogError(
                    f"{expected_kind} package path escapes repository root: {ref}",
                    code=DiagnosticCode.PATH_TRAVERSAL,
                    context=ref,
                    details={"kind": expected_kind},
                ) from exc
            record = self._records_by_path.get(path_candidate)
            if record is None:
                record = self._load_record(path_candidate)
                self._records_by_path[path_candidate] = record
                key = (record.kind, record.id, record.version)
                existing = self._records_by_ref.get(key)
                if existing is not None and existing.manifest_path != path_candidate:
                    raise CatalogError(
                        f"duplicate package {record.ref} ({record.kind})",
                        code=DiagnosticCode.DUPLICATE_IDENTITY,
                        context=record.ref,
                    )
                self._records_by_ref[key] = record
        else:
            match = _REF_RE.match(ref)
            if match is None:
                raise CatalogError(f"package reference must be exact id@version or a manifest path: {ref}")
            key = (expected_kind, match.group("id"), match.group("version"))
            record = self._records_by_ref.get(key)
            if record is None:
                raise CatalogError(
                    f"unknown {expected_kind} package: {ref}",
                    code=DiagnosticCode.PACKAGE_NOT_FOUND,
                    context=ref,
                    details={"kind": expected_kind},
                )
        if record.kind != expected_kind:
            raise CatalogError(f"package {record.ref} is {record.kind}, expected {expected_kind}")
        return record

    @property
    def records(self) -> tuple[PackageRecord, ...]:
        """Return the validated package records in deterministic identity order."""

        return tuple(
            sorted(
                self._records_by_ref.values(),
                key=lambda record: (record.kind, record.id, record.version),
            )
        )

    def find_package(self, reference: str, *, kind: str | None = None) -> PackageRecord:
        """Find one exact-version package without resolving a session."""

        match = _REF_RE.match(_string(reference, "package reference"))
        if match is None:
            raise CatalogError(
                f"package reference must be exact id@version: {reference}",
                code=DiagnosticCode.PACKAGE_NOT_FOUND,
                context=reference,
            )
        candidates = [
            record
            for record in self.records
            if record.id == match.group("id")
            and record.version == match.group("version")
            and (kind is None or record.kind == kind)
        ]
        if not candidates:
            raise CatalogError(
                f"unknown package: {reference}",
                code=DiagnosticCode.PACKAGE_NOT_FOUND,
                context=reference,
                details={"kind": kind} if kind is not None else {},
            )
        if len(candidates) != 1:
            raise CatalogError(
                f"package reference is ambiguous without kind: {reference}",
                code=DiagnosticCode.CATALOG_INVALID,
                context=reference,
                details={"kinds": [record.kind for record in candidates]},
            )
        return candidates[0]

    def resolve(self, session_path: Path) -> ResolvedSession:
        session_path = Path(session_path).resolve()
        data = _load_yaml(session_path)
        self._validate_session(data, session_path)
        payload_aware = data["schema"] == "lingtu.sim.session.v2"

        world = self._resolve_package(data["world"], "world", session_path.parent)
        package_records: dict[tuple[str, str, str], PackageRecord] = {}
        self._remember_package(package_records, world)
        world_mjcf = self._asset_path(world, world.data["physics"]["mjcf"], "physics.mjcf")
        world_projection = (
            self._validate_world_package_content(world)
            if self._world_manifest_is_canonical(world.data)
            else None
        )
        scenario: PackageRecord | None = None
        resolved_visual_scenario_entities: list[dict[str, Any]] = []
        if "scenario" in data:
            scenario = self._resolve_package(data["scenario"], "scenario", session_path.parent)
            scenario_world_ref = _string(scenario.data["world"], f"{scenario.manifest_path}.world")
            if scenario_world_ref != world.ref:
                raise CatalogError(
                    f"scenario {scenario.ref} requires world {scenario_world_ref}, but session resolves {world.ref}"
                )
            self._remember_package(package_records, scenario)
            resolved_visual_scenario_entities = self._build_scenario_visual_entities(scenario)

        resolved_robots: list[dict[str, Any]] = []
        resolved_visual_robots: list[dict[str, Any]] = []
        resolved_sensor_bindings: list[dict[str, Any]] = []
        resolved_control_bindings: list[dict[str, Any]] = []
        instance_ids: set[str] = set()
        declared_robot_ids = {
            _string(
                _mapping(item, f"{session_path}.robots")["instance_id"],
                f"{session_path}.robots.instance_id",
            )
            for item in data["robots"]
        }
        payload_instance_ids: set[str] = set()
        for index, item in enumerate(data["robots"]):
            robot_spec = _mapping(item, f"{session_path}.robots[{index}]")
            instance_id = _string(robot_spec["instance_id"], f"{session_path}.robots[{index}].instance_id")
            if instance_id in instance_ids:
                raise CatalogError(f"session has duplicate robot instance_id {instance_id!r}")
            instance_ids.add(instance_id)
            robot = self._resolve_package(robot_spec["package"], "robot", session_path.parent)
            self._remember_package(package_records, robot)
            robot_data = robot.data
            self._validate_mujoco_compatibility(robot, data["mujoco_version"])
            robot_mjcf = self._asset_path(robot, robot_data["physics"]["mjcf"], "physics.mjcf")
            projection = self._validate_robot_visual_projection(robot)
            resolved_physics_payloads: list[dict[str, Any]] = []
            resolved_visual_payloads: list[dict[str, Any]] = []
            for payload_index, payload_value in enumerate(robot_spec.get("payloads", [])):
                payload_context = f"{session_path}.robots[{index}].payloads[{payload_index}]"
                payload_spec = _mapping(payload_value, payload_context)
                payload_instance_id = _string(
                    payload_spec["instance_id"],
                    f"{payload_context}.instance_id",
                )
                if "__" in payload_instance_id:
                    raise CatalogError(
                        f"{payload_context}.instance_id must not contain '__'"
                    )
                if payload_instance_id in payload_instance_ids:
                    raise CatalogError(
                        f"session has duplicate payload instance_id {payload_instance_id!r}"
                    )
                if payload_instance_id in declared_robot_ids:
                    raise CatalogError(
                        f"payload instance_id {payload_instance_id!r} conflicts with a robot instance"
                    )
                payload_instance_ids.add(payload_instance_id)
                payload = self._resolve_package(
                    payload_spec["package"],
                    "payload",
                    session_path.parent,
                )
                self._remember_package(package_records, payload)
                self._validate_mujoco_compatibility(payload, data["mujoco_version"])
                payload_data = payload.data
                parent_frame = _string(
                    payload_spec["parent_frame"],
                    f"{payload_context}.parent_frame",
                )
                required_parent_role = _string(
                    payload_data["mount"]["required_parent_role"],
                    f"{payload.manifest_path}.mount.required_parent_role",
                )
                parent_body, package_mount_transform = self._resolve_robot_payload_mount(
                    robot,
                    robot_mjcf,
                    parent_frame,
                    required_parent_role,
                )
                instance_extrinsic = (
                    _extrinsic(
                        payload_spec["extrinsic"],
                        f"{payload_context}.extrinsic",
                    )
                    if "extrinsic" in payload_spec
                    else copy.deepcopy(_IDENTITY_EXTRINSIC)
                )
                mount_transform = _compose_extrinsics(
                    package_mount_transform,
                    instance_extrinsic,
                )
                payload_physics = _mapping(
                    payload_data["physics"],
                    f"{payload.manifest_path}.physics",
                )
                _, payload_mjcf = self._package_content_reference(
                    payload,
                    payload_physics["mjcf"],
                    f"{payload.manifest_path}.physics.mjcf",
                )
                payload_projection = self._validate_payload_visual_projection(payload)
                payload_package_ref = self._package_ref(payload)
                resolved_physics_payloads.append(
                    {
                        "instance_id": payload_instance_id,
                        "namespace": payload_instance_id,
                        "robot_instance_id": instance_id,
                        "package": payload_package_ref,
                        "parent_frame": parent_frame,
                        "parent_body": parent_body,
                        "mount_transform": mount_transform,
                        "model": {
                            "mjcf": _relative_path(payload_mjcf, self.repo_root),
                            "attach_root": payload_physics["attach_root"],
                        },
                        "authority": payload_physics["authority"],
                        "collision_representation": payload_physics[
                            "collision_representation"
                        ],
                        "frames": payload_data["frames"],
                    }
                )
                payload_visual = _mapping(
                    payload_data["visual"],
                    f"{payload.manifest_path}.visual",
                )
                resolved_visual_payloads.append(
                    {
                        "instance_id": payload_instance_id,
                        "namespace": payload_instance_id,
                        "robot_instance_id": instance_id,
                        "package": payload_package_ref,
                        "parent_frame": parent_frame,
                        "mount_transform": mount_transform,
                        "binding": payload_visual["binding"],
                        "projection": payload_projection,
                        "authority": payload_visual["authority"],
                        "ue_collision": payload_visual["ue_collision"],
                        "frames": payload_data["frames"],
                    }
                )
            controller_ref = robot_spec.get("controller", robot_data["defaults"]["controller"])
            controller: PackageRecord | None = None
            if controller_ref is not None:
                controller = self._resolve_package(controller_ref, "controller", session_path.parent)
                self._remember_package(package_records, controller)
                self._validate_controller_against_robot(controller, robot, robot_mjcf)
                resolved_control_bindings.append(
                    {
                        "instance_id": instance_id,
                        "robot": robot,
                        "controller": controller,
                    }
                )
            rig_ref = robot_spec.get("sensor_rig", robot_data["defaults"]["sensor_rig"])
            rig: PackageRecord | None = None
            sensor_records: list[PackageRecord] = []
            if rig_ref is not None:
                rig = self._resolve_package(rig_ref, "sensor_rig", session_path.parent)
                self._remember_package(package_records, rig)
                sensor_records = self._validate_rig_against_robot(rig, robot)
                for sensor in sensor_records:
                    self._remember_package(package_records, sensor)
                resolved_sensor_bindings.extend(
                    {
                        "instance_id": instance_id,
                        "robot": robot,
                        "rig": rig,
                        "rig_sensor": _mapping(rig_sensor, f"{rig.manifest_path}.sensors"),
                        "sensor": sensor,
                        "robot_frames": self._robot_frame_map(robot),
                    }
                    for rig_sensor, sensor in zip(
                        rig.data["sensors"], sensor_records, strict=True
                    )
                )
            spawn = _mapping(robot_spec["spawn"], f"{session_path}.robots[{index}].spawn")
            _keys(spawn, ("position_m", "quaternion_wxyz"), (), f"{session_path}.robots[{index}].spawn")
            position = _finite_vector(spawn["position_m"], 3, f"{session_path}.robots[{index}].spawn.position_m")
            quaternion = _finite_vector(
                spawn["quaternion_wxyz"], 4, f"{session_path}.robots[{index}].spawn.quaternion_wxyz"
            )
            resolved_robot: dict[str, Any] = {
                "instance_id": instance_id,
                "namespace": instance_id,
                "package": self._package_ref(robot),
                "controller": self._package_ref(controller) if controller else None,
                "sensor_rig": self._package_ref(rig) if rig else None,
                "spawn": {"position_m": position, "quaternion_wxyz": quaternion},
                "model": {
                    "mjcf": _relative_path(robot_mjcf, self.repo_root),
                    "attach_root": robot_data["physics"]["attach_root"],
                    "root_joint": robot_data["physics"]["root_joint"],
                    "initial_keyframe": robot_data["physics"].get("initial_keyframe"),
                },
                "semantic": {"class": robot_data["semantic"]["class"]},
                "frames": robot_data["frames"],
            }
            if resolved_physics_payloads:
                resolved_robot["payloads"] = resolved_physics_payloads
            resolved_robots.append(resolved_robot)
            resolved_visual_robot: dict[str, Any] = {
                "instance_id": instance_id,
                "namespace": instance_id,
                "package": self._package_ref(robot),
                "binding": robot_data["visual"]["binding"],
                "projection": projection,
                "spawn": {"position_m": position, "quaternion_wxyz": quaternion},
            }
            if resolved_visual_payloads:
                resolved_visual_robot["payloads"] = resolved_visual_payloads
            resolved_visual_robots.append(resolved_visual_robot)

        global_policy = {
            "owner": "world",
            **world.data["physics"]["global_policy"],
        }
        session_id = _session_id(data["session_id"], f"{session_path}.session_id")
        resolved_world = {
            "package": self._package_ref(world),
            "mjcf": _relative_path(world_mjcf, self.repo_root),
        }
        required_bindings = set(
            _list(data["runtime"]["required_bindings"], f"{session_path}.runtime.required_bindings")
        )
        visual_backend = "unreal" if "visual" in required_bindings else None
        backends = {"physics": "mujoco", "visual": visual_backend}

        physics_plan: dict[str, Any] = {
            "schema": (
                "lingtu.sim.physics-plan.v2"
                if payload_aware
                else "lingtu.sim.physics-plan.v1"
            ),
            "session_id": session_id,
            "composition": {
                "model_kind": "single_mjmodel",
                "composer": "mjs_attach_v1",
                "namespace_separator": "__",
                "state_authority": "mujoco",
            },
            "global_policy": global_policy,
            "world": resolved_world,
            "robots": resolved_robots,
        }
        if scenario is not None:
            kinematic_entities = self._build_kinematic_entities(scenario)
            if kinematic_entities:
                physics_plan["kinematic_entities"] = kinematic_entities
        visual_world: dict[str, Any] = {
            "package": self._package_ref(world),
            "binding": world.data["visual"]["binding"],
            "level": world.data["visual"]["level"],
        }
        if world_projection is not None:
            visual_world["projection"] = world_projection
        visual_plan: dict[str, Any] = {
            "schema": (
                "lingtu.sim.visual-plan.v2"
                if payload_aware
                else "lingtu.sim.visual-plan.v1"
            ),
            "session_id": session_id,
            "backends": backends,
            "coordinate_system": {
                "source": "mujoco_rh_z_up_m",
                "target": "unreal_lh_z_up_cm",
                "position_scale": 100.0,
                "axis_mapping": ["x", "-y", "z"],
                "quaternion_order": "wxyz",
            },
            "binding_policy": {
                "missing_asset": "fail",
                "data_asset_is_projection": True,
            },
            "world": visual_world,
            "robots": resolved_visual_robots,
        }
        if resolved_visual_scenario_entities:
            visual_plan["scenario_entities"] = resolved_visual_scenario_entities
        sensor_plan = self._build_sensor_plan(
            session_id, backends, resolved_sensor_bindings, "sensors" in required_bindings
        )
        _validate_physics_sensor_timebase(world, global_policy, sensor_plan)
        control_plan = self._build_control_plan(
            session_id, backends, resolved_control_bindings, "control" in required_bindings
        )
        transport_intent = self._build_transport_intent(session_id, backends, sensor_plan, control_plan)
        scenario_plan = (
            self._build_scenario_plan(session_id, int(data["seed"]), scenario, resolved_robots)
            if scenario is not None
            else None
        )
        return ResolvedSession(
            session_id=session_id,
            session=dict(data),
            physics_plan=physics_plan,
            visual_plan=visual_plan,
            sensor_plan=sensor_plan,
            control_plan=control_plan,
            transport_intent=transport_intent,
            physics_json=_pretty_json(physics_plan),
            visual_json=_pretty_json(visual_plan),
            sensor_json=_pretty_json(sensor_plan),
            control_json=_pretty_json(control_plan),
            transport_json=_pretty_json(transport_intent),
            scenario_plan=scenario_plan,
            scenario_json=_pretty_json(scenario_plan) if scenario_plan is not None else None,
        )

    def _validate_session(self, data: Mapping[str, Any], path: Path) -> None:
        context = str(path)
        _keys(
            data,
            ("schema", "session_id", "mujoco_version", "seed", "world", "robots", "runtime"),
            ("scenario",),
            context,
        )
        if data["schema"] not in {"lingtu.sim.session.v1", "lingtu.sim.session.v2"}:
            raise CatalogError(f"{context}.schema is unsupported: {data['schema']!r}")
        payload_aware = data["schema"] == "lingtu.sim.session.v2"
        _session_id(data["session_id"], f"{context}.session_id")
        _string(data["mujoco_version"], f"{context}.mujoco_version")
        if isinstance(data["seed"], bool) or not isinstance(data["seed"], int):
            raise CatalogError(f"{context}.seed must be an integer")
        _string(data["world"], f"{context}.world")
        if "scenario" in data:
            scenario_ref = _string(data["scenario"], f"{context}.scenario")
            if _REF_RE.match(scenario_ref) is None:
                raise CatalogError(f"{context}.scenario must be an exact id@version reference")
        robots = _list(data["robots"], f"{context}.robots")
        if not robots:
            raise CatalogError(f"{context}.robots must not be empty")
        for index, item in enumerate(robots):
            robot = _mapping(item, f"{context}.robots[{index}]")
            _keys(
                robot,
                ("instance_id", "package", "spawn"),
                (
                    "controller",
                    "sensor_rig",
                    *(("payloads",) if payload_aware else ()),
                ),
                f"{context}.robots[{index}]",
            )
            _string(robot["instance_id"], f"{context}.robots[{index}].instance_id")
            _string(robot["package"], f"{context}.robots[{index}].package")
            for key in ("controller", "sensor_rig"):
                if key in robot and robot[key] is not None:
                    _string(robot[key], f"{context}.robots[{index}].{key}")
            _mapping(robot["spawn"], f"{context}.robots[{index}].spawn")
            if "payloads" in robot:
                payloads = _list(
                    robot["payloads"],
                    f"{context}.robots[{index}].payloads",
                )
                if not payloads:
                    raise CatalogError(
                        f"{context}.robots[{index}].payloads must not be empty"
                    )
                for payload_index, payload_value in enumerate(payloads):
                    payload_context = (
                        f"{context}.robots[{index}].payloads[{payload_index}]"
                    )
                    payload = _mapping(payload_value, payload_context)
                    _keys(
                        payload,
                        ("instance_id", "package", "parent_frame"),
                        ("extrinsic",),
                        payload_context,
                    )
                    payload_instance_id = _string(
                        payload["instance_id"],
                        f"{payload_context}.instance_id",
                    )
                    if "__" in payload_instance_id:
                        raise CatalogError(
                            f"{payload_context}.instance_id must not contain '__'"
                        )
                    _string(payload["package"], f"{payload_context}.package")
                    _string(
                        payload["parent_frame"],
                        f"{payload_context}.parent_frame",
                    )
                    if "extrinsic" in payload:
                        _extrinsic(payload["extrinsic"], f"{payload_context}.extrinsic")
        runtime = _mapping(data["runtime"], f"{context}.runtime")
        _keys(runtime, ("backend", "mode", "required_bindings"), (), f"{context}.runtime")
        backend = _string(runtime["backend"], f"{context}.runtime.backend")
        if backend != "mujoco":
            raise CatalogError(f"{context}.runtime.backend is unsupported: {backend!r}")
        _string(runtime["mode"], f"{context}.runtime.mode")
        bindings = _list(runtime["required_bindings"], f"{context}.runtime.required_bindings")
        if not bindings:
            raise CatalogError(f"{context}.runtime.required_bindings must not be empty")
        normalized_bindings: list[str] = []
        seen_bindings: set[str] = set()
        for index, value in enumerate(bindings):
            binding = _string(value, f"{context}.runtime.required_bindings[{index}]")
            if binding not in {"physics", "visual", "sensors", "control"}:
                raise CatalogError(f"{context}.runtime.required_bindings[{index}] is unsupported: {binding!r}")
            if binding in seen_bindings:
                raise CatalogError(f"{context}.runtime.required_bindings contains duplicate value {binding!r}")
            seen_bindings.add(binding)
            normalized_bindings.append(binding)
        if "physics" not in normalized_bindings:
            raise CatalogError(f"{context}.runtime.required_bindings must include 'physics'")

    def _remember_package(self, target: dict[tuple[str, str, str], PackageRecord], record: PackageRecord) -> None:
        target[(record.kind, record.id, record.version)] = record

    def _validate_mujoco_compatibility(self, record: PackageRecord, requested_version: str) -> None:
        compatibility = record.data.get("compatibility")
        if compatibility is None:
            return
        compatibility_data = _mapping(compatibility, f"{record.manifest_path}.compatibility")
        expected = compatibility_data.get("mujoco")
        if expected is None:
            return
        expected_text = _string(expected, f"{record.manifest_path}.compatibility.mujoco")
        pattern = "^" + re.escape(expected_text).replace(r"\.x", r"\.\d+") + "$"
        if re.match(pattern, requested_version) is None:
            raise CatalogError(
                f"{record.kind} {record.ref} requires MuJoCo {expected_text}, but session requests {requested_version}"
            )

    def _package_ref(self, record: PackageRecord | None) -> dict[str, str] | None:
        if record is None:
            return None
        return {
            "id": record.id,
            "version": record.version,
            "kind": record.kind,
            "manifest": _relative_path(record.manifest_path, self.repo_root),
        }

    def _validate_controller_against_robot(self, controller: PackageRecord, robot: PackageRecord, mjcf: Path) -> None:
        controller_data = controller.data
        channels = _list(controller_data["actuators"]["channels"], f"{controller.manifest_path}.actuators.channels")
        symbols = _xml_actuator_names(mjcf)
        if channels != symbols:
            raise CatalogError(
                f"controller {controller.ref} actuator order does not match robot {robot.ref}: "
                f"controller={channels!r}, mjcf={symbols!r}"
            )

    def _validate_rig_against_robot(self, rig: PackageRecord, robot: PackageRecord) -> list[PackageRecord]:
        frames = {frame["name"] for frame in robot.data["frames"]}
        sensors: list[PackageRecord] = []
        for index, item in enumerate(rig.data["sensors"]):
            sensor_data = _mapping(item, f"{rig.manifest_path}.sensors[{index}]")
            parent_frame = sensor_data["parent_frame"]
            if parent_frame not in frames:
                raise CatalogError(
                    f"sensor rig {rig.ref} mounts sensor {sensor_data['id']!r} on unknown "
                    f"frame {parent_frame!r} of robot {robot.ref}"
                )
            sensors.append(self._resolve_package(sensor_data["package"], "sensor", rig.manifest_path.parent))
        return sensors

    def _robot_frame_map(self, robot: PackageRecord) -> dict[str, dict[str, Any]]:
        return {
            _string(frame["name"], f"{robot.manifest_path}.frames.name"): _mapping(
                frame,
                f"{robot.manifest_path}.frames[{index}]",
            )
            for index, frame in enumerate(_list(robot.data["frames"], f"{robot.manifest_path}.frames"))
        }

    def _resolve_robot_payload_mount(
        self,
        robot: PackageRecord,
        robot_mjcf: Path,
        parent_frame: str,
        required_role: str,
    ) -> tuple[str, dict[str, list[float]]]:
        frames = self._robot_frame_map(robot)
        frame = frames.get(parent_frame)
        if frame is None:
            raise CatalogError(
                f"payload references unknown parent_frame {parent_frame!r} of robot {robot.ref}"
            )
        if frame["role"] != required_role:
            raise CatalogError(
                f"payload requires parent role {required_role!r}, but robot frame "
                f"{parent_frame!r} has role {frame['role']!r}"
            )

        try:
            root = ET.parse(robot_mjcf).getroot()
        except (OSError, ET.ParseError) as exc:
            raise CatalogError(f"cannot parse MuJoCo XML {robot_mjcf}: {exc}") from exc
        parents = {child: parent for parent in root.iter() for child in parent}
        matches = [
            element
            for element in root.iter()
            if element.tag in {"body", "site"} and element.attrib.get("name") == parent_frame
        ]
        if len(matches) != 1:
            raise CatalogError(
                f"robot payload mount {parent_frame!r} must resolve to exactly one MJCF body or site"
            )
        element = matches[0]
        if element.tag == "body":
            if "parent_frame" in frame or "extrinsic" in frame:
                raise CatalogError(
                    f"body-backed payload mount {parent_frame!r} must not declare a separate extrinsic"
                )
            return parent_frame, copy.deepcopy(_IDENTITY_EXTRINSIC)

        parent = parents.get(element)
        while parent is not None and parent.tag != "body":
            parent = parents.get(parent)
        parent_body = None if parent is None else parent.attrib.get("name")
        if not parent_body:
            raise CatalogError(f"payload mount site {parent_frame!r} has no named parent body")
        if frame.get("parent_frame") != parent_body:
            raise CatalogError(
                f"payload mount frame {parent_frame!r} parent_frame does not match its MJCF body"
            )
        if "euler" in element.attrib or "axisangle" in element.attrib:
            raise CatalogError(
                f"payload mount site {parent_frame!r} must use an explicit quaternion"
            )
        site_transform = {
            "position_m": [
                float(component)
                for component in element.attrib.get("pos", "0 0 0").split()
            ],
            "quaternion_wxyz": [
                float(component)
                for component in element.attrib.get("quat", "1 0 0 0").split()
            ],
        }
        site_transform = _extrinsic(
            site_transform,
            f"{robot_mjcf}.site[{parent_frame}]",
        )
        declared_transform = frame.get("extrinsic")
        if declared_transform is None:
            raise CatalogError(
                f"payload mount frame {parent_frame!r} must declare its package-owned extrinsic"
            )
        for key in ("position_m", "quaternion_wxyz"):
            if any(
                not math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-9)
                for left, right in zip(
                    declared_transform[key],
                    site_transform[key],
                    strict=True,
                )
            ):
                raise CatalogError(
                    f"payload mount frame {parent_frame!r} extrinsic does not match the MJCF site"
                )
        return parent_body, copy.deepcopy(site_transform)

    def _build_scenario_plan(
        self,
        session_id: str,
        seed: int,
        scenario: PackageRecord,
        robots: Sequence[Mapping[str, Any]],
    ) -> dict[str, Any]:
        entities: list[dict[str, Any]] = []
        seen: set[str] = set()
        for robot in robots:
            entity_id = _string(robot["instance_id"], "scenario robot instance_id")
            if entity_id in seen:
                raise CatalogError(f"scenario plan contains duplicate entity_id {entity_id!r}")
            seen.add(entity_id)
            semantic = _mapping(robot["semantic"], f"scenario robot {entity_id}.semantic")
            semantic_class = _string(
                semantic["class"],
                f"scenario robot {entity_id}.semantic.class",
            )
            entities.append(
                {
                    "entity_id": entity_id,
                    "entity_type": "robot",
                    "authority": "mujoco",
                    "source_epoch": 0,
                    "initial_transform": robot["spawn"],
                    "physics_proxy": "mujoco",
                    "semantic_class": semantic_class,
                }
            )
        for index, item in enumerate(scenario.data["entities"]):
            entity = self._scenario_entity(scenario, item, index)
            entity_id = entity["entity_id"]
            if entity_id in seen:
                raise CatalogError(f"scenario plan contains duplicate entity_id {entity_id!r}")
            seen.add(entity_id)
            entities.append(entity)
        return {
            "schema": "lingtu.sim.scenario-plan.v1",
            "session_id": session_id,
            "env": "sim",
            "backend": "mujoco",
            "package": self._package_ref(scenario),
            "model_generation": 0,
            "reset_generation": 0,
            "seed": seed,
            "clock": {
                "unit": "ns",
                "source": "mujoco_sim_time",
                "sim_time_ns": 0,
            },
            "authority_policy": {
                "robot_physics_owner": "mujoco",
                "dynamic_behavior_owner": "scenario",
                "visual_animation_owner": "ue_animation",
            },
            "entities": entities,
        }

    def _build_kinematic_entities(self, scenario: PackageRecord) -> list[dict[str, Any]]:
        entities: list[dict[str, Any]] = []
        for index, value in enumerate(scenario.data["entities"]):
            context = f"{scenario.manifest_path}.entities[{index}]"
            source = _mapping(value, context)
            proxy = self._scenario_physics_proxy(
                scenario,
                source["physics_proxy"],
                f"{context}.physics_proxy",
            )
            model_path = proxy.get("model_path")
            if proxy["mode"] != "kinematic" or not isinstance(model_path, Path):
                continue
            entity = self._scenario_entity(scenario, source, index)
            entity_id = entity["entity_id"]
            entities.append(
                {
                    "entity_id": entity_id,
                    "namespace": entity_id,
                    "package": self._package_ref(scenario),
                    "model": {
                        "mjcf": _relative_path(model_path, self.repo_root),
                        "attach_root": proxy["attach_root"],
                    },
                    "initial_transform": entity["initial_transform"],
                }
            )
        return entities

    def _build_scenario_visual_entities(self, scenario: PackageRecord) -> list[dict[str, Any]]:
        entities: list[dict[str, Any]] = []
        for index, value in enumerate(scenario.data["entities"]):
            context = f"{scenario.manifest_path}.entities[{index}]"
            source = _mapping(value, context)
            if "visual" not in source:
                continue
            proxy = self._scenario_physics_proxy(
                scenario,
                source["physics_proxy"],
                f"{context}.physics_proxy",
            )
            projection = self._validate_scenario_entity_visual_projection(
                scenario,
                source,
                proxy,
                context,
            )
            entity = self._scenario_entity(scenario, source, index)
            visual = _mapping(source["visual"], f"{context}.visual")
            entity_id = entity["entity_id"]
            entities.append(
                {
                    "entity_id": entity_id,
                    "namespace": entity_id,
                    "package": self._package_ref(scenario),
                    "binding": _string(visual["binding"], f"{context}.visual.binding"),
                    "projection": projection,
                    "spawn": entity["initial_transform"],
                }
            )
        return entities

    def _validate_scenario_entity_visual_projection(
        self,
        scenario: PackageRecord,
        entity: Mapping[str, Any],
        physics_proxy: Mapping[str, Any],
        context: str,
    ) -> dict[str, str]:
        visual = _mapping(entity["visual"], f"{context}.visual")
        _keys(visual, ("binding", "projection"), (), f"{context}.visual")
        binding = _string(visual["binding"], f"{context}.visual.binding")
        projection_relative = self._require_safe_package_path(
            visual["projection"],
            f"{context}.visual.projection",
        )
        projection_path = self._package_relative_file(
            scenario,
            projection_relative,
            f"{context}.visual.projection",
        )
        model_path = physics_proxy.get("model_path")
        attach_root = physics_proxy.get("attach_root")
        if not isinstance(model_path, Path) or not isinstance(attach_root, str):
            raise CatalogError(
                f"{context}.visual requires an object-form kinematic physics_proxy"
            )
        projection = _mapping(_load_json(projection_path), f"{context}.visual.projection")
        _keys(
            projection,
            ("schema", "binding", "package", "mjcf", "components"),
            (),
            f"{context}.visual.projection",
        )
        if projection["schema"] != "lingtu.sim.entity-visual-projection.v1":
            raise CatalogError(f"{context}.visual.projection schema is unsupported")
        expected_package = {
            "id": scenario.id,
            "version": scenario.version,
            "manifest": scenario.manifest_path.name,
        }
        if projection["binding"] != binding:
            raise CatalogError(f"{context}.visual.projection binding does not match the entity")
        projected_mjcf = _mapping(projection["mjcf"], f"{context}.visual.projection.mjcf")
        if (
            projection["package"] != expected_package
            or projected_mjcf.get("path")
            != model_path.relative_to(scenario.manifest_path.parent).as_posix()
        ):
            raise CatalogError(f"{context}.visual.projection provenance does not match the ScenarioPackage")
        local_body_ids = {component["local_body_id"] for component in projection["components"]}
        if attach_root not in local_body_ids:
            raise CatalogError(
                f"{context}.visual.projection does not bind the physics proxy root {attach_root!r}"
            )
        return {
            "schema": projection["schema"],
            "path": _relative_path(projection_path, self.repo_root),
        }

    def _scenario_physics_proxy(
        self,
        scenario: PackageRecord,
        value: Any,
        context: str,
    ) -> dict[str, Any]:
        if isinstance(value, Mapping):
            proxy = _mapping(value, context)
            _keys(proxy, ("mode", "mjcf", "attach_root"), (), context)
            mode = _string(proxy["mode"], f"{context}.mode")
            if mode != "kinematic":
                raise CatalogError(f"{context}.mode is unsupported: {mode!r}")
            model_path = self._package_relative_file(
                scenario,
                proxy["mjcf"],
                "entities.physics_proxy.mjcf",
            )
            attach_root = _string(proxy["attach_root"], f"{context}.attach_root")
            if "__" in attach_root:
                raise CatalogError(f"{context}.attach_root must not contain '__'")
            try:
                xml_root = ET.parse(model_path).getroot()
            except (OSError, ET.ParseError) as exc:
                raise CatalogError(f"cannot parse scenario proxy MJCF {model_path}: {exc}") from exc
            body = next(
                (
                    element
                    for element in xml_root.iter("body")
                    if element.attrib.get("name") == attach_root
                ),
                None,
            )
            if body is None:
                raise CatalogError(f"{context}.attach_root {attach_root!r} is absent from the proxy MJCF")
            if body.attrib.get("mocap", "false").lower() not in {"true", "1"}:
                raise CatalogError(f"{context}.attach_root must identify a MuJoCo mocap body")
            if any(element.tag in {"joint", "freejoint"} for element in body.iter()):
                raise CatalogError(f"{context}.attach_root mocap subtree must not contain joints")
            return {
                "mode": mode,
                "model_path": model_path,
                "attach_root": attach_root,
            }

        mode = _string(value, context)
        if mode not in {"mujoco", "kinematic", "none"}:
            raise CatalogError(f"{context} is unsupported: {mode!r}")
        return {"mode": mode}

    def _scenario_entity(self, scenario: PackageRecord, value: Any, index: int) -> dict[str, Any]:
        context = f"{scenario.manifest_path}.entities[{index}]"
        entity = _mapping(value, context)
        authority = _string(entity["authority"], f"{context}.authority")
        if authority not in {"scenario", "ue_animation", "mujoco"}:
            raise CatalogError(f"{context}.authority is unsupported: {authority!r}")
        transform = _mapping(entity["initial_transform"], f"{context}.initial_transform")
        initial_transform = {
            "position_m": _finite_vector(transform["position_m"], 3, f"{context}.initial_transform.position_m"),
            "quaternion_wxyz": _finite_vector(
                transform["quaternion_wxyz"],
                4,
                f"{context}.initial_transform.quaternion_wxyz",
            ),
        }
        physics_proxy = self._scenario_physics_proxy(
            scenario,
            entity["physics_proxy"],
            f"{context}.physics_proxy",
        )
        entity_id = _string(entity["entity_id"], f"{context}.entity_id")
        compiled_proxy: str | dict[str, str]
        if isinstance(physics_proxy.get("model_path"), Path):
            compiled_proxy = {
                "mode": "kinematic",
                "body_stable_id": f"{entity_id}/{physics_proxy['attach_root']}",
            }
        else:
            compiled_proxy = physics_proxy["mode"]
        result: dict[str, Any] = {
            "entity_id": entity_id,
            "entity_type": _string(entity["entity_type"], f"{context}.entity_type"),
            "authority": authority,
            "source_epoch": 0,
            "initial_transform": initial_transform,
            "physics_proxy": compiled_proxy,
            "semantic_class": _string(entity["semantic_class"], f"{context}.semantic_class"),
        }
        if "behavior" in entity:
            behavior = _mapping(entity["behavior"], f"{context}.behavior")
            parameters = _mapping(behavior["parameters"], f"{context}.behavior.parameters")
            result["behavior"] = {
                "profile": _string(behavior["profile"], f"{context}.behavior.profile"),
                "seed": behavior["seed"],
                "parameters": {
                    _string(key, f"{context}.behavior.parameters key"): _finite_behavior_value(
                        value,
                        f"{context}.behavior.parameters.{key}",
                    )
                    for key, value in sorted(parameters.items())
                },
            }
        return result

    def _build_sensor_plan(
        self,
        session_id: str,
        backends: dict[str, str | None],
        bindings: Sequence[dict[str, Any]],
        enabled: bool,
    ) -> dict[str, Any]:
        streams: dict[str, list[dict[str, Any]]] = {
            "rgb": [],
            "depth": [],
            "imu": [],
            "mid360": [],
            "truth_odom": [],
        }
        if enabled:
            for binding in bindings:
                stream = self._sensor_stream(binding)
                stream_class = stream.pop("_stream_class")
                if stream_class not in streams:
                    raise CatalogError(f"unsupported sensor stream class: {stream_class}")
                streams[stream_class].append(stream)
        return {
            "schema": "lingtu.sim.sensor-plan.v1",
            "session_id": session_id,
            "env": "sim",
            "backends": backends,
            "streams": streams,
        }

    def _sensor_stream(self, binding: Mapping[str, Any]) -> dict[str, Any]:
        instance_id = _string(binding["instance_id"], "sensor binding instance_id")
        rig_sensor = _mapping(binding["rig_sensor"], "sensor binding rig_sensor")
        sensor = binding["sensor"]
        sensor_data = sensor.data
        base_configuration = _configuration(sensor_data["configuration"], f"{sensor.manifest_path}.configuration")
        rig_configuration = _configuration(rig_sensor.get("configuration", {}), "rig sensor configuration")
        if "raycast_frame" in base_configuration:
            raise CatalogError(
                "sensor package configuration must not own raycast_frame; declare it on the SensorRig instance"
            )
        raycast_frame = rig_configuration.pop("raycast_frame", None)
        configuration = {**base_configuration, **rig_configuration}
        stream_class = _string(
            _list(sensor_data["outputs"], f"{sensor.manifest_path}.outputs")[0], f"{sensor.manifest_path}.outputs[0]"
        )
        if stream_class == "mid360" and raycast_frame is None:
            raise CatalogError("Mid360 rig sensor requires configuration.raycast_frame")
        if stream_class != "mid360" and raycast_frame is not None:
            raise CatalogError("configuration.raycast_frame is only valid for Mid360")
        rig_sensor_id = _string(rig_sensor["id"], "rig sensor id")
        parent_frame = _string(rig_sensor["parent_frame"], "rig sensor parent_frame")
        robot_frames = {
            _string(name, "sensor binding robot frame name"): _mapping(frame, f"sensor binding robot_frames.{name}")
            for name, frame in _mapping(binding["robot_frames"], "sensor binding robot_frames").items()
        }
        robot_frame = robot_frames.get(parent_frame)
        if robot_frame is None:
            raise CatalogError(f"sensor binding parent_frame {parent_frame!r} is absent from robot frame map")
        resolved_raycast_frame: str | None = None
        if raycast_frame is not None:
            resolved_raycast_frame = _string(
                raycast_frame,
                "rig sensor configuration.raycast_frame",
            )
            if resolved_raycast_frame not in robot_frames:
                raise CatalogError(
                    f"sensor binding raycast_frame {resolved_raycast_frame!r} is absent from robot frame map"
                )
        resolved_parent_frame = _string(
            robot_frame.get("parent_frame", parent_frame), f"robot frame {parent_frame}.parent_frame"
        )
        sensor_id = f"{instance_id}.{rig_sensor_id}"
        runtime = _mapping(rig_sensor["runtime"], "rig sensor runtime")
        interface = _mapping(sensor_data["interface"], f"{sensor.manifest_path}.interface")
        timing = _mapping(sensor_data["timing"], f"{sensor.manifest_path}.timing")
        if (
            stream_class == "depth"
            and runtime.get("source") == "unreal_camera"
            and interface.get("transport") == "camera_shm"
        ):
            requested_encoding = _string(
                configuration.get("encoding"),
                f"{sensor.manifest_path}.configuration.encoding",
            )
            if requested_encoding not in {"16UC1", "32FC1"}:
                raise CatalogError(
                    "Unreal camera depth stream requires configuration.encoding "
                    "to be 16UC1 or 32FC1"
                )
            unit = _string(
                configuration.get("unit"),
                f"{sensor.manifest_path}.configuration.unit",
            )
            if unit != "m":
                raise CatalogError("Unreal camera depth stream requires configuration.unit to be 'm'")
            configuration = {
                **configuration,
                "requested_encoding": requested_encoding,
                "encoding": "16UC1",
                "depth_scale": 0.001,
            }
        rate_hz = rig_sensor.get("frequency_hz", timing["rate_hz"])
        if (
            isinstance(rate_hz, bool)
            or not isinstance(rate_hz, (int, float))
            or not math.isfinite(float(rate_hz))
            or float(rate_hz) <= 0
        ):
            raise CatalogError("sensor stream rate_hz must be positive finite numeric data")
        extrinsic_source = rig_sensor.get("extrinsic", robot_frame.get("extrinsic", _IDENTITY_EXTRINSIC))
        extrinsic = _extrinsic(extrinsic_source, "sensor stream extrinsic")
        stream: dict[str, Any] = {
            "_stream_class": stream_class,
            "instance_id": instance_id,
            "sensor_id": sensor_id,
            "owner": runtime["owner"],
            "source": runtime["source"],
            "frame_id": f"{instance_id}/{parent_frame}",
            "parent_frame_id": f"{instance_id}/{resolved_parent_frame}",
            "extrinsic": extrinsic,
            "rate_hz": rate_hz,
            "transport": interface["transport"],
            "message_type": interface["message_type"],
        }
        if resolved_raycast_frame is not None:
            stream["raycast_frame_stable_id"] = f"{instance_id}/{resolved_raycast_frame}"
        stream.update(configuration)
        return stream

    def _build_control_plan(
        self,
        session_id: str,
        backends: dict[str, str | None],
        bindings: Sequence[dict[str, Any]],
        enabled: bool,
    ) -> dict[str, Any]:
        controllers: list[dict[str, Any]] = []
        command_channels: list[dict[str, Any]] = []
        if enabled:
            for binding in bindings:
                controller = binding["controller"]
                controller_data = controller.data
                instance_id = _string(binding["instance_id"], "control binding instance_id")
                base_channel_id = f"{instance_id}.control.base_twist"
                actuator_channel_id = f"{instance_id}.control.joint_torque"
                adapter = _mapping(controller_data["adapter"], f"{controller.manifest_path}.adapter")
                policy = _mapping(controller_data["policy"], f"{controller.manifest_path}.policy")
                timing = _mapping(controller_data["timing"], f"{controller.manifest_path}.timing")
                robot_interface = _mapping(
                    controller_data["robot_interface"], f"{controller.manifest_path}.robot_interface"
                )
                accepts = _mapping(
                    robot_interface["accepts_command"], f"{controller.manifest_path}.robot_interface.accepts_command"
                )
                produces = _mapping(
                    robot_interface["produces_command"], f"{controller.manifest_path}.robot_interface.produces_command"
                )
                controllers.append(
                    {
                        "instance_id": instance_id,
                        "controller_id": f"{instance_id}.{controller.id}",
                        "package": self._package_ref(controller),
                        "adapter": adapter,
                        "policy": {
                            "runtime": policy["runtime"],
                            "artifact": _relative_path(
                                self._asset_path(controller, policy["artifact"], "policy.artifact"), self.repo_root
                            ),
                            "manifest": _relative_path(
                                self._asset_path(controller, policy["manifest"], "policy.manifest"), self.repo_root
                            ),
                        },
                        "timing": timing,
                        "state_channels": list(robot_interface["requires_state"]),
                        "command_channels": [base_channel_id, actuator_channel_id],
                        "actuator_channels": list(controller_data["actuators"]["channels"]),
                    }
                )
                command_channels.extend(
                    [
                        {
                            "channel_id": base_channel_id,
                            "direction": "subscribe",
                            "owner": "simulation",
                            "source": f"dds_base_twist/{instance_id}",
                            "transport": "typed_dds",
                            "message_type": accepts["message_type"],
                            "command_type": accepts["type"],
                            "target": "base",
                        },
                        {
                            "channel_id": actuator_channel_id,
                            "direction": "publish",
                            "owner": "physics",
                            "source": f"{instance_id}.{controller.id}",
                            "transport": "in_process",
                            "message_type": produces.get("message_type", "lingtu.sim.joint-torque.v1"),
                            "command_type": produces["type"],
                            "target": "actuators",
                        },
                    ]
                )
        return {
            "schema": "lingtu.sim.control-plan.v1",
            "session_id": session_id,
            "env": "sim",
            "backends": backends,
            "controllers": controllers,
            "command_channels": command_channels,
            "stale_stop_authority": {
                "owner": "simulation",
                "hardware_forwarding": False,
                "safe_stop_on_stale": True,
                "stale_timeout_ms": 100,
            },
        }

    def _build_transport_intent(
        self,
        session_id: str,
        backends: dict[str, str | None],
        sensor_plan: Mapping[str, Any],
        control_plan: Mapping[str, Any],
    ) -> dict[str, Any]:
        channels: list[dict[str, Any]] = []
        if backends["visual"] == "unreal":
            channels.append(
                {
                    "channel_id": "runtime.truth_snapshot",
                    "direction": "publish",
                    "owner": "physics",
                    "source": "mujoco_runtime",
                    "transport": "udp_loopback_json",
                    "delivery": "latest_wins",
                    "message_type": "lingtu.sim.truth-snapshot.v1",
                    "payload_role": "truth_snapshot",
                    "contract": "lingtu.sim.truth-snapshot.v1",
                    "frame_policy": "sim_time",
                }
            )
        for stream_class in ("rgb", "depth", "imu", "mid360", "truth_odom"):
            for stream in sensor_plan["streams"][stream_class]:
                channels.append(
                    {
                        "channel_id": stream["sensor_id"],
                        "direction": "publish",
                        "owner": stream["owner"],
                        "source": stream["source"],
                        "transport": stream["transport"],
                        "delivery": _transport_delivery(stream["transport"]),
                        "message_type": stream["message_type"],
                        "payload_role": "mid360_livox" if stream_class == "mid360" else stream_class,
                        "contract": _transport_contract(stream["message_type"]),
                        "frame_policy": "source_time",
                    }
                )
        for channel in control_plan["command_channels"]:
            channels.append(
                {
                    "channel_id": channel["channel_id"],
                    "direction": channel["direction"],
                    "owner": channel["owner"],
                    "source": channel["source"],
                    "transport": channel["transport"],
                    "delivery": _transport_delivery(channel["transport"]),
                    "message_type": channel["message_type"],
                    "payload_role": "control",
                    "contract": _transport_contract(channel["message_type"]),
                    "frame_policy": "none" if channel["transport"] == "in_process" else "source_time",
                }
            )
        return {
            "schema": "lingtu.sim.transport-intent.v1",
            "session_id": session_id,
            "env": "sim",
            "backends": backends,
            "channels": channels,
            "allocation_boundary": {
                "owner": "RunAllocation",
                "runtime_values_external": True,
            },
        }


def _xml_symbols(path: Path) -> set[str]:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise CatalogError(f"cannot parse MuJoCo XML {path}: {exc}") from exc
    symbols: set[str] = set()
    for element in root.iter():
        name = element.attrib.get("name")
        if name and element.tag in {
            "body",
            "joint",
            "site",
            "camera",
            "key",
            "actuator",
            "motor",
            "general",
            "position",
            "velocity",
            "cylinder",
            "muscle",
        }:
            symbols.add(name)
    return symbols


def _xml_body_names(path: Path) -> set[str]:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise CatalogError(f"cannot parse MuJoCo XML {path}: {exc}") from exc
    return {
        element.attrib["name"]
        for element in root.iter("body")
        if "name" in element.attrib
    }


def _glb_material_channels(path: Path) -> list[set[str]]:
    if path.suffix.lower() != ".glb":
        raise CatalogError(f"payload PBR source asset must be a GLB file: {path}")
    payload = path.read_bytes()
    if len(payload) < 20:
        raise CatalogError(f"payload GLB is truncated: {path}")
    magic, version, declared_length = struct.unpack_from("<4sII", payload, 0)
    if magic != b"glTF" or version != 2 or declared_length != len(payload):
        raise CatalogError(f"payload source is not a canonical glTF 2 GLB: {path}")
    offset = 12
    document: Mapping[str, Any] | None = None
    while offset + 8 <= len(payload):
        chunk_length, chunk_type = struct.unpack_from("<II", payload, offset)
        offset += 8
        chunk_end = offset + chunk_length
        if chunk_end > len(payload):
            raise CatalogError(f"payload GLB chunk exceeds declared length: {path}")
        if chunk_type == 0x4E4F534A:
            try:
                decoded = json.loads(payload[offset:chunk_end].rstrip(b"\x00 ").decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise CatalogError(f"payload GLB has invalid JSON metadata: {path}") from exc
            document = _mapping(decoded, f"{path}.glTF")
            break
        offset = chunk_end
    if document is None:
        raise CatalogError(f"payload GLB has no JSON chunk: {path}")
    materials = _list(document.get("materials"), f"{path}.materials")
    result: list[set[str]] = []
    for index, value in enumerate(materials):
        material = _mapping(value, f"{path}.materials[{index}]")
        channels: set[str] = set()
        pbr = material.get("pbrMetallicRoughness")
        if isinstance(pbr, Mapping):
            if isinstance(pbr.get("baseColorTexture"), Mapping):
                channels.add("base_color")
            if isinstance(pbr.get("metallicRoughnessTexture"), Mapping):
                channels.add("metallic_roughness")
        if isinstance(material.get("normalTexture"), Mapping):
            channels.add("normal")
        result.append(channels)
    return result


def _xml_actuator_names(path: Path) -> list[str]:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise CatalogError(f"cannot parse MuJoCo XML {path}: {exc}") from exc
    actuator = root.find("actuator")
    if actuator is None:
        return []
    return [
        element.attrib["name"]
        for element in actuator
        if element.tag in {"motor", "general", "position", "velocity", "cylinder", "muscle"}
        and "name" in element.attrib
    ]


def _xml_asset_paths(model_path: Path, repo_root: Path) -> set[Path]:
    """Collect declared XML assets, including MuJoCo compiler meshdir."""

    collected: set[Path] = set()
    visited_xml: set[Path] = set()

    def visit(path: Path) -> None:
        path = path.resolve()
        if path in visited_xml:
            return
        visited_xml.add(path)
        if not path.is_file():
            raise CatalogError(f"MuJoCo XML asset does not exist: {path}")
        collected.add(path)
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError) as exc:
            raise CatalogError(f"cannot parse MuJoCo XML {path}: {exc}") from exc
        compiler = root.find("compiler")
        meshdir = compiler.attrib.get("meshdir", ".") if compiler is not None else "."
        for element in root.iter():
            reference = element.attrib.get("file")
            if not reference:
                continue
            base = path.parent / meshdir if element.tag == "mesh" else path.parent
            referenced = (base / reference).resolve()
            try:
                referenced.relative_to(repo_root.resolve())
            except ValueError as exc:
                raise CatalogError(f"MuJoCo XML reference escapes repository root: {referenced}") from exc
            if not referenced.is_file():
                raise CatalogError(f"MuJoCo XML reference does not exist: {referenced}")
            collected.add(referenced)
            if referenced.suffix.lower() == ".xml":
                visit(referenced)

    visit(model_path)
    return collected


def _transport_delivery(transport: str) -> str:
    if transport in {"camera_shm", "named_file_mapping", "udp_loopback_json"}:
        return "latest_wins"
    if transport == "in_process":
        return "reliable"
    return "ordered_no_drop"


def _transport_contract(message_type: str) -> str:
    if message_type.startswith("lingtu.dds."):
        return "field_dds_v1"
    return message_type
