"""Resolve simulation packages into deterministic session artifacts.

The catalog is intentionally local and exact-version only.  It is a build-time
boundary: it does not start processes, allocate ports, load Unreal assets, or
mutate the legacy Python simulation runtime.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml


class CatalogError(ValueError):
    """Raised when a package catalog or session contract is invalid."""


_SCHEMA_KIND = {
    "lingtu.sim.robot-package.v1": "robot",
    "lingtu.sim.controller-package.v1": "controller",
    "lingtu.sim.sensor-package.v1": "sensor",
    "lingtu.sim.sensor-rig-package.v1": "sensor_rig",
    "lingtu.sim.world-package.v1": "world",
}
_PACKAGE_SUFFIXES = (".package.yaml", ".package.yml")
_REF_RE = re.compile(r"^(?P<id>[A-Za-z0-9][A-Za-z0-9_.-]*)@(?P<version>[A-Za-z0-9][A-Za-z0-9+_.-]*)$")


def _canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def _pretty_json(value: Any) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
        separators=(",", ": "),
        allow_nan=False,
    ) + "\n"


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
    """The deterministic part of one simulation session."""

    session_digest: str
    session_lock: dict[str, Any]
    physics_plan: dict[str, Any]
    lock_json: str
    physics_json: str

    def write_bundle(self, bundle_dir: Path) -> Path:
        """Materialize only deterministic plans; runtime allocation is separate."""

        bundle_dir = Path(bundle_dir)
        bundle_dir.mkdir(parents=True, exist_ok=True)
        (bundle_dir / "session.lock.json").write_text(self.lock_json, encoding="utf-8")
        (bundle_dir / "physics.plan.json").write_text(self.physics_json, encoding="utf-8")
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
    def from_repository(cls, repo_root: Path) -> "CatalogResolver":
        root = Path(repo_root).resolve()
        return cls(
            root,
            (
                root / "sim" / "robots",
                root / "sim" / "controllers",
                root / "sim" / "sensors" / "packages",
                root / "sim" / "sensor_rigs",
                root / "sim" / "worlds" / "packages",
            ),
        )

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
                raise CatalogError(f"duplicate package {record.ref} ({record.kind}): {other.manifest_path} and {path}")
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
                ("schema", "id", "version", "kind", "physics", "visual", "frames", "interfaces", "defaults", "declared_capabilities"),
                ("description", "compatibility", "qualification"),
                context,
            )
            physics = _mapping(data["physics"], f"{context}.physics")
            _keys(physics, ("mjcf", "attach_root", "root_joint"), (), f"{context}.physics")
            mjcf = self._asset_path(record, physics["mjcf"], "physics.mjcf")
            symbols = _xml_symbols(mjcf)
            for field in ("attach_root", "root_joint"):
                if physics[field] not in symbols:
                    raise CatalogError(f"{context}.physics.{field}={physics[field]!r} is absent from {mjcf}")
            visual = _mapping(data["visual"], f"{context}.visual")
            _keys(visual, ("binding",), (), f"{context}.visual")
            _string(visual["binding"], f"{context}.visual.binding")
            self._validate_frames(record, data["frames"], mjcf)
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
            _keys(robot_interface, ("requires_state", "produces_command"), (), f"{context}.robot_interface")
            for value in _list(robot_interface["requires_state"], f"{context}.robot_interface.requires_state"):
                _string(value, f"{context}.robot_interface.requires_state item")
            produces = _mapping(robot_interface["produces_command"], f"{context}.robot_interface.produces_command")
            _keys(produces, ("type",), (), f"{context}.robot_interface.produces_command")
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
                ("schema", "id", "version", "kind", "sensor_type", "outputs", "timing", "interface"),
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
            return

        if record.kind == "sensor_rig":
            _keys(data, ("schema", "id", "version", "kind", "sensors"), ("description",), context)
            sensors = _list(data["sensors"], f"{context}.sensors")
            seen: set[str] = set()
            for index, item in enumerate(sensors):
                sensor = _mapping(item, f"{context}.sensors[{index}]")
                _keys(sensor, ("id", "package", "parent_frame"), ("frequency_hz", "extrinsic"), f"{context}.sensors[{index}]")
                sensor_id = _string(sensor["id"], f"{context}.sensors[{index}].id")
                if sensor_id in seen:
                    raise CatalogError(f"{context}.sensors contains duplicate id {sensor_id!r}")
                seen.add(sensor_id)
                _string(sensor["package"], f"{context}.sensors[{index}].package")
                _string(sensor["parent_frame"], f"{context}.sensors[{index}].parent_frame")
                if "frequency_hz" in sensor:
                    frequency = sensor["frequency_hz"]
                    if isinstance(frequency, bool) or not isinstance(frequency, (int, float)) or not math.isfinite(float(frequency)) or float(frequency) <= 0:
                        raise CatalogError(f"{context}.sensors[{index}].frequency_hz must be positive finite numeric data")
            return

        if record.kind == "world":
            _keys(data, ("schema", "id", "version", "kind", "physics", "visual", "entities"), ("description",), context)
            physics = _mapping(data["physics"], f"{context}.physics")
            _keys(physics, ("mjcf",), (), f"{context}.physics")
            mjcf = self._asset_path(record, physics["mjcf"], "physics.mjcf")
            _xml_symbols(mjcf)
            visual = _mapping(data["visual"], f"{context}.visual")
            _keys(visual, ("binding",), (), f"{context}.visual")
            entities = _list(data["entities"], f"{context}.entities")
            for index, entity in enumerate(entities):
                _mapping(entity, f"{context}.entities[{index}]")
            return

        raise CatalogError(f"unsupported package kind: {record.kind}")

    def _validate_frames(self, record: PackageRecord, value: Any, mjcf: Path) -> None:
        context = f"{record.manifest_path}.frames"
        frames = _list(value, context)
        if not frames:
            raise CatalogError(f"{context} must not be empty")
        symbols = _xml_symbols(mjcf)
        seen: set[str] = set()
        for index, item in enumerate(frames):
            frame = _mapping(item, f"{context}[{index}]")
            _keys(frame, ("name", "role"), (), f"{context}[{index}]")
            name = _string(frame["name"], f"{context}[{index}].name")
            if name in seen:
                raise CatalogError(f"{context} contains duplicate frame {name!r}")
            if name not in symbols:
                raise CatalogError(f"{context}[{index}].name={name!r} is absent from {mjcf}")
            seen.add(name)
            _string(frame["role"], f"{context}[{index}].role")

    def _asset_path(self, record: PackageRecord, value: Any, field: str) -> Path:
        relative = _string(value, f"{record.manifest_path}.{field}")
        candidate = (record.manifest_path.parent / relative).resolve()
        try:
            candidate.relative_to(self.repo_root)
        except ValueError as exc:
            raise CatalogError(f"{record.manifest_path}.{field} escapes repository root") from exc
        if not candidate.is_file():
            raise CatalogError(f"{record.manifest_path}.{field} does not exist: {relative}")
        return candidate

    def _resolve_package(self, reference: Any, expected_kind: str, base_dir: Path) -> PackageRecord:
        ref = _string(reference, f"{expected_kind} package reference")
        path_candidate = (base_dir / ref).resolve()
        if path_candidate.suffix in {".yaml", ".yml"} or "/" in ref or "\\" in ref:
            record = self._records_by_path.get(path_candidate)
            if record is None:
                record = self._load_record(path_candidate)
                self._records_by_path[path_candidate] = record
                key = (record.kind, record.id, record.version)
                existing = self._records_by_ref.get(key)
                if existing is not None and existing.manifest_path != path_candidate:
                    raise CatalogError(f"duplicate package {record.ref} ({record.kind})")
                self._records_by_ref[key] = record
        else:
            match = _REF_RE.match(ref)
            if match is None:
                raise CatalogError(f"package reference must be exact id@version or a manifest path: {ref}")
            key = (expected_kind, match.group("id"), match.group("version"))
            record = self._records_by_ref.get(key)
            if record is None:
                raise CatalogError(f"unknown {expected_kind} package: {ref}")
        if record.kind != expected_kind:
            raise CatalogError(f"package {record.ref} is {record.kind}, expected {expected_kind}")
        return record

    def resolve(self, session_path: Path) -> ResolvedSession:
        session_path = Path(session_path).resolve()
        data = _load_yaml(session_path)
        self._validate_session(data, session_path)

        world = self._resolve_package(data["world"], "world", session_path.parent)
        package_records: dict[tuple[str, str, str], PackageRecord] = {}
        self._remember_package(package_records, world)
        world_mjcf = self._asset_path(world, world.data["physics"]["mjcf"], "physics.mjcf")

        resolved_robots: list[dict[str, Any]] = []
        instance_ids: set[str] = set()
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
            controller_ref = robot_spec.get("controller", robot_data["defaults"]["controller"])
            controller: PackageRecord | None = None
            if controller_ref is not None:
                controller = self._resolve_package(controller_ref, "controller", session_path.parent)
                self._remember_package(package_records, controller)
                self._validate_controller_against_robot(controller, robot, robot_mjcf)
            rig_ref = robot_spec.get("sensor_rig", robot_data["defaults"]["sensor_rig"])
            rig: PackageRecord | None = None
            sensor_records: list[PackageRecord] = []
            if rig_ref is not None:
                rig = self._resolve_package(rig_ref, "sensor_rig", session_path.parent)
                self._remember_package(package_records, rig)
                sensor_records = self._validate_rig_against_robot(rig, robot)
                for sensor in sensor_records:
                    self._remember_package(package_records, sensor)
            spawn = _mapping(robot_spec["spawn"], f"{session_path}.robots[{index}].spawn")
            _keys(spawn, ("position_m", "quaternion_wxyz"), (), f"{session_path}.robots[{index}].spawn")
            position = _finite_vector(spawn["position_m"], 3, f"{session_path}.robots[{index}].spawn.position_m")
            quaternion = _finite_vector(spawn["quaternion_wxyz"], 4, f"{session_path}.robots[{index}].spawn.quaternion_wxyz")
            resolved_robots.append(
                {
                    "instance_id": instance_id,
                    "namespace": instance_id,
                    "package": self._package_ref(robot),
                    "controller": self._package_ref(controller) if controller else None,
                    "sensor_rig": self._package_ref(rig) if rig else None,
                    "spawn": {"position_m": position, "quaternion_wxyz": quaternion},
                    "model": {
                        "mjcf": _relative_path(robot_mjcf, self.repo_root),
                        "sha256": _sha256_file(robot_mjcf),
                        "attach_root": robot_data["physics"]["attach_root"],
                        "root_joint": robot_data["physics"]["root_joint"],
                    },
                    "frames": robot_data["frames"],
                }
            )

        package_locks = [self._package_lock(record) for record in sorted(package_records.values(), key=lambda item: (item.kind, item.id, item.version))]
        lock_body: dict[str, Any] = {
            "schema": "lingtu.sim.session-lock.v1",
            "session": {
                "schema": data["schema"],
                "session_id": data["session_id"],
                "mujoco_version": data["mujoco_version"],
                "seed": data["seed"],
                "runtime": data["runtime"],
            },
            "world": {
                "package": self._package_ref(world),
                "mjcf": _relative_path(world_mjcf, self.repo_root),
                "sha256": _sha256_file(world_mjcf),
            },
            "robots": resolved_robots,
            "packages": package_locks,
        }
        session_digest = _sha256_bytes(_canonical_json(lock_body).encode("utf-8"))
        session_lock = {**lock_body, "session_digest": session_digest}

        physics_plan: dict[str, Any] = {
            "schema": "lingtu.sim.physics-plan.v1",
            "session_digest": session_digest,
            "model_generation": 0,
            "reset_generation": 0,
            "composition": {
                "model_kind": "single_mjmodel",
                "composer": "mjs_attach_v1",
                "namespace_separator": "__",
                "state_authority": "mujoco",
            },
            "world": session_lock["world"],
            "robots": resolved_robots,
        }
        return ResolvedSession(
            session_digest=session_digest,
            session_lock=session_lock,
            physics_plan=physics_plan,
            lock_json=_pretty_json(session_lock),
            physics_json=_pretty_json(physics_plan),
        )

    def _validate_session(self, data: Mapping[str, Any], path: Path) -> None:
        context = str(path)
        _keys(data, ("schema", "session_id", "mujoco_version", "seed", "world", "robots", "runtime"), (), context)
        if data["schema"] != "lingtu.sim.session.v1":
            raise CatalogError(f"{context}.schema is unsupported: {data['schema']!r}")
        _string(data["session_id"], f"{context}.session_id")
        _string(data["mujoco_version"], f"{context}.mujoco_version")
        if isinstance(data["seed"], bool) or not isinstance(data["seed"], int):
            raise CatalogError(f"{context}.seed must be an integer")
        _string(data["world"], f"{context}.world")
        robots = _list(data["robots"], f"{context}.robots")
        if not robots:
            raise CatalogError(f"{context}.robots must not be empty")
        for index, item in enumerate(robots):
            robot = _mapping(item, f"{context}.robots[{index}]")
            _keys(robot, ("instance_id", "package", "spawn"), ("controller", "sensor_rig"), f"{context}.robots[{index}]")
            _string(robot["instance_id"], f"{context}.robots[{index}].instance_id")
            _string(robot["package"], f"{context}.robots[{index}].package")
            for key in ("controller", "sensor_rig"):
                if key in robot and robot[key] is not None:
                    _string(robot[key], f"{context}.robots[{index}].{key}")
            _mapping(robot["spawn"], f"{context}.robots[{index}].spawn")
        runtime = _mapping(data["runtime"], f"{context}.runtime")
        _keys(runtime, ("backend", "mode", "required_bindings"), (), f"{context}.runtime")
        _string(runtime["backend"], f"{context}.runtime.backend")
        _string(runtime["mode"], f"{context}.runtime.mode")
        bindings = _list(runtime["required_bindings"], f"{context}.runtime.required_bindings")
        for index, value in enumerate(bindings):
            _string(value, f"{context}.runtime.required_bindings[{index}]")

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
                f"robot {record.ref} requires MuJoCo {expected_text}, "
                f"but session requests {requested_version}"
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

    def _package_lock(self, record: PackageRecord) -> dict[str, Any]:
        artifacts = self._package_artifacts(record)
        manifest_digest = _sha256_bytes(_canonical_json(record.data).encode("utf-8"))
        return {
            **self._package_ref(record),
            "manifest_sha256": manifest_digest,
            "artifacts": artifacts,
            "spec": record.data,
        }

    def _package_artifacts(self, record: PackageRecord) -> list[dict[str, str]]:
        paths: set[Path] = {record.manifest_path}
        data = record.data
        if record.kind in {"robot", "world"}:
            physics = _mapping(data["physics"], f"{record.manifest_path}.physics")
            model_path = self._asset_path(record, physics["mjcf"], "physics.mjcf")
            paths.update(_xml_asset_paths(model_path, self.repo_root))
        elif record.kind == "controller":
            policy = _mapping(data["policy"], f"{record.manifest_path}.policy")
            paths.add(self._asset_path(record, policy["artifact"], "policy.artifact"))
            paths.add(self._asset_path(record, policy["manifest"], "policy.manifest"))
        result = []
        for path in sorted(paths):
            result.append({"path": _relative_path(path, self.repo_root), "sha256": _sha256_file(path)})
        return result

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


def _xml_symbols(path: Path) -> set[str]:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise CatalogError(f"cannot parse MuJoCo XML {path}: {exc}") from exc
    symbols: set[str] = set()
    for element in root.iter():
        name = element.attrib.get("name")
        if name and element.tag in {"body", "joint", "site", "camera", "actuator", "motor", "general", "position", "velocity", "cylinder", "muscle"}:
            symbols.add(name)
    return symbols


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
        if element.tag in {"motor", "general", "position", "velocity", "cylinder", "muscle"} and "name" in element.attrib
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
