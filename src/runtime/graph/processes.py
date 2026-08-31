"""Resolve Product process roles against one deployment Env implementation."""

from __future__ import annotations

import re
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from .loader import RuntimeGraph, load_runtime_graph

PROCESS_LIFECYCLES = frozenset({"mode", "persistent"})
_PROCESS_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")
_SYSTEMD_TARGET = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.@:-]*\.service\Z")
_OPAQUE_TARGET = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.@:-]*\Z")
_PROCESS_ENV_KEY = re.compile(r"[A-Z][A-Z0-9_]*\Z")
_RESERVED_PROCESS_ENV = frozenset(
    {
        "LINGTU_RUN_PLAN",
        "LINGTU_PRODUCT",
        "LINGTU_ENV",
        "LINGTU_ENV_BACKEND",
        "LINGTU_SESSION_ROOT",
        "LINGTU_HOST_BOOT_ID",
        "LINGTU_PROCESS_LAUNCH_ID",
        "LINGTU_PRODUCT_SESSION_ID",
        "LINGTU_SYSTEMD_UNIT",
    }
)
_PYTHON_INTERPRETER = re.compile(r"python(?:3(?:\.[0-9]+)?)?\Z")
_PROCESS_BASE_FIELDS = frozenset({"target", "lifecycle", "order", "timeout_s"})
_SYSTEMD_PROCESS_OPTIONAL_FIELDS = frozenset({"provides"})
_DIRECT_PROCESS_FIELDS = frozenset({*_PROCESS_BASE_FIELDS, "provides", "command"})
_DIRECT_PLATFORM_PROCESS_FIELDS = frozenset({*_PROCESS_BASE_FIELDS, "provides", "platforms"})
_PROCESS_PLATFORMS = frozenset({"linux", "windows"})
_DIRECT_COMMAND_FIELDS = frozenset({"argv", "cwd", "env", "artifact", "readiness"})
_DIRECT_COMMAND_WITH_SHUTDOWN_FIELDS = _DIRECT_COMMAND_FIELDS | {"shutdown"}
_DIRECT_COMMAND_WITH_DEPENDENCIES_FIELDS = _DIRECT_COMMAND_FIELDS | {"dependencies"}
_DIRECT_COMMAND_WITH_DEPENDENCIES_AND_SHUTDOWN_FIELDS = _DIRECT_COMMAND_WITH_SHUTDOWN_FIELDS | {"dependencies"}
_ENV_ARTIFACT_FIELDS = frozenset({"path"})
_READINESS_PROCESS_FIELDS = frozenset({"kind"})
_READINESS_FILE_FIELDS = frozenset({"kind", "target"})
_SHUTDOWN_PROCESS_FIELDS = frozenset({"kind"})
_SHUTDOWN_FILE_FIELDS = frozenset({"kind", "target", "schema"})
_SHUTDOWN_SCHEMA = re.compile(r"[a-z][a-z0-9]*(?:\.[a-z][a-z0-9_]*)+\.v[0-9]+\Z")


def _safe_relative_path(
    value: Any,
    *,
    field: str,
    root_label: str,
    allow_current: bool = False,
) -> str:
    if (
        not isinstance(value, str)
        or not value
        or "\\" in value
        or "{run_plan_path}" in value
    ):
        raise ValueError(f"direct process {field} must be a safe {root_label}-relative POSIX path")
    if allow_current and value == ".":
        return value
    path = PurePosixPath(value)
    if (
        value == "."
        or path.is_absolute()
        or path.as_posix() != value
        or any(part in {"", ".", ".."} or ":" in part for part in path.parts)
    ):
        raise ValueError(f"direct process {field} must be a safe {root_label}-relative POSIX path")
    return value


def _normalized_process_argv(value: Any) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError("direct command argv must be a list or tuple")
    argv = tuple(value)
    if not argv or any(
        not isinstance(item, str) or not item or any(character in item for character in ("\x00", "\n", "\r"))
        for item in argv
    ):
        raise ValueError("direct command argv contains invalid values")
    return argv


def _normalized_process_env(value: Any) -> tuple[tuple[str, str], ...]:
    if isinstance(value, Mapping):
        env_items = tuple(value.items())
    elif isinstance(value, list | tuple):
        env_items = tuple(value)
    else:
        raise ValueError("direct command env must be an object or pairs")
    normalized_env: list[tuple[str, str]] = []
    for item in env_items:
        if not isinstance(item, list | tuple) or len(item) != 2:
            raise ValueError("direct command env contains invalid pairs")
        key, item_value = item
        if not isinstance(key, str) or _PROCESS_ENV_KEY.fullmatch(key) is None:
            raise ValueError("direct command env contains an invalid key")
        if key in _RESERVED_PROCESS_ENV:
            raise ValueError(f"direct command env key is reserved: {key}")
        if (
            not isinstance(item_value, str)
            or any(character in item_value for character in ("\x00", "\n", "\r"))
            or "{run_plan_path}" in item_value
        ):
            raise ValueError(f"direct command env contains an invalid value: {key}")
        normalized_env.append((key, item_value))
    if len({key for key, _ in normalized_env}) != len(normalized_env):
        raise ValueError("direct command env contains duplicate keys")
    return tuple(sorted(normalized_env))


@dataclass(frozen=True)
class ProcessArtifact:
    """One repository-owned file referenced by a direct process."""

    path: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "path",
            _safe_relative_path(
                self.path,
                field="artifact path",
                root_label="repository",
            ),
        )
    def as_dict(self) -> dict[str, str]:
        """Return the stable RunPlan representation."""

        return {"path": self.path}

    @classmethod
    def from_repository_path(
        cls,
        repository_root: str | Path,
        path: Any,
    ) -> ProcessArtifact:
        """Resolve one repository-owned spawn artifact."""

        relative_path = _safe_relative_path(
            path,
            field="artifact path",
            root_label="repository",
        )
        root = Path(repository_root).resolve()
        candidate = root / PurePosixPath(relative_path)
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as exc:
            raise ValueError(f"direct process artifact does not exist: {relative_path}") from exc
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ValueError(f"direct process artifact escapes repository: {relative_path}") from exc
        if not resolved.is_file():
            raise ValueError(f"direct process artifact is not a file: {relative_path}")
        return cls(relative_path)


@dataclass(frozen=True)
class ProcessReadiness:
    """Typed readiness policy for a direct process.

    A ``file`` target is resolved below ``LINGTU_SESSION_ROOT``.
    """

    kind: str
    target: str | None = None

    def __post_init__(self) -> None:
        if self.kind == "process":
            if self.target is not None:
                raise ValueError("process readiness must not declare a target")
            return
        if self.kind == "file":
            object.__setattr__(
                self,
                "target",
                _safe_relative_path(
                    self.target,
                    field="readiness target",
                    root_label="LINGTU_SESSION_ROOT",
                ),
            )
            return
        raise ValueError(f"invalid process readiness kind: {self.kind!r}")

    def as_dict(self) -> dict[str, str]:
        """Return the stable RunPlan representation."""

        payload = {"kind": self.kind}
        if self.kind == "file":
            if self.target is None:  # pragma: no cover - constructor invariant
                raise RuntimeError("file readiness target is missing")
            payload["target"] = self.target
        return payload


@dataclass(frozen=True)
class ProcessShutdown:
    """Typed safe-stop policy for a direct process.

    A ``file`` target is resolved below ``LINGTU_SESSION_ROOT``.
    """

    kind: str
    target: str | None = None
    schema: str | None = None

    def __post_init__(self) -> None:
        if self.kind == "process":
            if self.target is not None:
                raise ValueError("process shutdown must not declare a target")
            if self.schema is not None:
                raise ValueError("process shutdown must not declare a schema")
            return
        if self.kind == "file":
            object.__setattr__(
                self,
                "target",
                _safe_relative_path(
                    self.target,
                    field="shutdown target",
                    root_label="LINGTU_SESSION_ROOT",
                ),
            )
            if not isinstance(self.schema, str) or _SHUTDOWN_SCHEMA.fullmatch(self.schema) is None:
                raise ValueError("file shutdown schema must be a safe schema token")
            return
        raise ValueError(f"invalid process shutdown kind: {self.kind!r}")

    def as_dict(self) -> dict[str, str]:
        """Return the stable RunPlan representation."""

        payload = {"kind": self.kind}
        if self.kind == "file":
            if self.target is None:  # pragma: no cover - constructor invariant
                raise RuntimeError("file shutdown target is missing")
            if self.schema is None:  # pragma: no cover - constructor invariant
                raise RuntimeError("file shutdown schema is missing")
            payload["target"] = self.target
            payload["schema"] = self.schema
        return payload


@dataclass(frozen=True)
class ProcessCommand:
    """Shell-free command and provenance for a direct process."""

    argv: tuple[str, ...]
    cwd: str
    env: tuple[tuple[str, str], ...]
    artifact: ProcessArtifact
    readiness: ProcessReadiness
    shutdown: ProcessShutdown | None = None
    dependencies: tuple[ProcessArtifact, ...] = ()

    def __post_init__(self) -> None:
        argv = _normalized_process_argv(self.argv)
        if any(
            item == "--run-plan"
            or item.startswith("--run-plan=")
            or "{run_plan_path}" in item
            for item in argv
        ):
            raise ValueError("direct command argv must not embed runtime identity")
        object.__setattr__(self, "argv", argv)

        object.__setattr__(
            self,
            "cwd",
            _safe_relative_path(
                self.cwd,
                field="cwd",
                root_label="repository",
                allow_current=True,
            ),
        )
        if not isinstance(self.artifact, ProcessArtifact):
            raise ValueError("direct command artifact must be typed")
        if not isinstance(self.readiness, ProcessReadiness):
            raise ValueError("direct command readiness must be typed")
        if self.shutdown is not None and not isinstance(
            self.shutdown,
            ProcessShutdown,
        ):
            raise ValueError("direct command shutdown must be typed")
        if not isinstance(self.dependencies, list | tuple) or any(
            not isinstance(dependency, ProcessArtifact) for dependency in self.dependencies
        ):
            raise ValueError("direct command dependencies must contain artifacts")
        dependencies = tuple(self.dependencies)
        dependency_paths = tuple(dependency.path for dependency in dependencies)
        if len(set(dependency_paths)) != len(dependency_paths) or self.artifact.path in dependency_paths:
            raise ValueError("direct command dependencies contain duplicate artifact paths")
        object.__setattr__(self, "dependencies", dependencies)
        native_entry = argv[0] == self.artifact.path
        python_entry = (
            len(argv) >= 2 and _PYTHON_INTERPRETER.fullmatch(argv[0]) is not None and argv[1] == self.artifact.path
        )
        if not native_entry and not python_entry:
            raise ValueError(
                "direct command artifact must be the spawn entry: native argv[0] or approved Python argv[1]"
            )

        normalized_env = _normalized_process_env(self.env)
        normalized_env_map = dict(normalized_env)
        for variable in ("LINGTU_NAV_CLIENT_LIB", "LINGTU_SLAM_CONTROL"):
            dependency_path = normalized_env_map.get(variable)
            if dependency_path is not None and dependency_path not in dependency_paths:
                raise ValueError(
                    f"{variable} must be an authenticated direct command dependency"
                )
        object.__setattr__(self, "env", normalized_env)

    def as_dict(self) -> dict[str, Any]:
        """Return the stable RunPlan representation."""

        payload: dict[str, Any] = {
            "argv": list(self.argv),
            "cwd": self.cwd,
            "env": dict(self.env),
            "artifact": self.artifact.as_dict(),
            "readiness": self.readiness.as_dict(),
        }
        if self.shutdown is not None:
            payload["shutdown"] = self.shutdown.as_dict()
        if self.dependencies:
            payload["dependencies"] = [dependency.as_dict() for dependency in self.dependencies]
        return payload


@dataclass(frozen=True)
class ProcessSpec:
    """One physical process resolved to a deployment target."""

    name: str
    manager: str
    target: str
    order: int
    timeout_s: int
    lifecycle: str
    command: ProcessCommand | None = None
    provides: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not valid_process_name(self.name):
            raise ValueError(f"process name is invalid: {self.name!r}")
        if not isinstance(self.manager, str) or self.manager not in {
            "systemd",
            "direct",
            "external",
        }:
            raise ValueError(f"process manager is invalid: {self.manager!r}")
        if not isinstance(self.lifecycle, str) or self.lifecycle not in PROCESS_LIFECYCLES:
            raise ValueError(f"process lifecycle is invalid: {self.lifecycle!r}")
        if isinstance(self.order, bool) or not isinstance(self.order, int) or self.order < 0:
            raise ValueError(f"process order is invalid: {self.order!r}")
        if isinstance(self.timeout_s, bool) or not isinstance(self.timeout_s, int) or self.timeout_s <= 0:
            raise ValueError(f"process timeout is invalid: {self.timeout_s!r}")
        target_pattern = _SYSTEMD_TARGET if self.manager == "systemd" else _OPAQUE_TARGET
        if not isinstance(self.target, str) or target_pattern.fullmatch(self.target) is None:
            raise ValueError(f"{self.manager} process target is invalid: {self.target!r}")
        if self.manager == "direct" and self.command is None:
            raise ValueError("direct process requires a command")
        if self.manager == "direct" and not isinstance(self.command, ProcessCommand):
            raise ValueError("direct process requires a typed command")
        if self.manager != "direct" and self.command is not None:
            raise ValueError(f"{self.manager} process must not declare a command")
        if not isinstance(self.provides, list | tuple):
            raise ValueError("process provides must be a list or tuple")
        provides = tuple(self.provides)
        if any(not isinstance(role, str) or _PROCESS_NAME.fullmatch(role) is None for role in provides) or len(
            set(provides)
        ) != len(provides):
            raise ValueError("process provides contains invalid or duplicate roles")
        if self.manager == "systemd":
            provides = provides or (self.name,)
        elif self.manager != "direct" and provides:
            raise ValueError(f"{self.manager} process must not declare provides")
        object.__setattr__(self, "provides", provides)

    def as_dict(self) -> dict[str, Any]:
        """Return the stable representation stored in a RunPlan."""

        payload: dict[str, Any] = {
            "name": self.name,
            "manager": self.manager,
            "target": self.target,
            "order": self.order,
            "timeout_s": self.timeout_s,
            "lifecycle": self.lifecycle,
        }
        if self.manager == "direct":
            if self.command is None:  # pragma: no cover - constructor invariant
                raise RuntimeError("direct process command is missing")
            payload["command"] = self.command.as_dict()
            payload["provides"] = list(self.provides)
        elif self.manager == "systemd" and self.provides != (self.name,):
            payload["provides"] = list(self.provides)
        return payload


def resolve_processes(
    product: str,
    env: str,
    *,
    graph: RuntimeGraph | None = None,
    env_config: Mapping[str, Any] | None = None,
    implementation: Mapping[str, Any] | None = None,
    product_spec: Mapping[str, Any] | None = None,
) -> tuple[
    tuple[ProcessSpec, ...],
    tuple[ProcessSpec, ...],
    tuple[str, ...],
    tuple[str, ...],
]:
    """Return selected, available, conflict, and support process identities.

    Acceptance and external implementations own their own transaction, so
    they return no ProductControl process records. Managed implementations
    resolve physical owners and select Product role owners plus the
    implementation's explicit role-free support processes.
    """

    graph = graph or load_runtime_graph()
    if product_spec is None:
        product_spec = graph.products.get(product)
    if product_spec is None:
        raise ValueError(f"unknown Runtime Graph product: {product}")
    if implementation is None:
        implementation = resolve_env_implementation(
            env,
            graph=graph,
            env_config=env_config,
        )
    supported_products = _name_tuple(
        implementation.get("supported_products"),
        owner=f"env {env}",
        field="supported_products",
        allow_empty=True,
    )
    if product not in supported_products:
        supported = ", ".join(supported_products) or "none"
        raise ValueError(f"env {env} implementation does not support Product {product!r}; supported: {supported}")

    raw_control = implementation.get("process_control")
    if not isinstance(raw_control, str) or not raw_control.strip():
        raise ValueError(f"env {env} has invalid process control")
    process_control = raw_control.strip()
    if process_control in {"acceptance_runner", "external_runner"}:
        if "support_processes" in implementation:
            raise ValueError(f"env {env} support_processes are only valid for a managed process implementation")
        return (), (), (), ()
    expected_manager = {
        "systemd": "systemd",
        "subprocess": "direct",
    }.get(process_control)
    if expected_manager is None:
        raise ValueError(f"env {env} has unsupported process control {process_control!r}")

    names = _process_names(product_spec.get("processes"), product=product)
    definitions = implementation.get("processes")
    if not isinstance(definitions, Mapping):
        raise ValueError(f"env {env} has no process mapping")

    raw_manager = implementation.get("process_manager")
    if not isinstance(raw_manager, str) or raw_manager != expected_manager:
        raise ValueError(f"env {env} process manager must be {expected_manager!r} for {process_control!r} control")
    manager = raw_manager
    support_processes = _support_process_names(
        implementation.get("support_processes", []),
        owner=f"env {env}",
    )
    product_support_overlap = sorted(set(names) & set(support_processes))
    if product_support_overlap:
        raise ValueError(
            f"product {product} must declare logical roles, not support process names: "
            f"{', '.join(product_support_overlap)}"
        )
    provided_roles = _name_tuple(
        implementation.get("provided_roles"),
        owner=f"env {env}",
        field="provided_roles",
    )
    support_role_overlap = sorted(set(support_processes) & set(provided_roles))
    if support_role_overlap:
        raise ValueError(
            f"env {env} support process names must not be provided roles: {', '.join(support_role_overlap)}"
        )
    if not set(names).issubset(provided_roles):
        missing_declared = sorted(set(names) - set(provided_roles))
        raise ValueError(f"env {env} does not declare logical process roles: {', '.join(missing_declared)}")

    role_candidates: dict[str, list[str]] = {role: [] for role in names}
    for process_name, definition in definitions.items():
        roles = _raw_process_roles(
            process_name,
            definition,
            env=env,
            manager=manager,
        )
        for role in set(names).intersection(roles):
            role_candidates[role].append(str(process_name))
    missing_raw = sorted(role for role, owners in role_candidates.items() if not owners)
    duplicate_raw = {role: owners for role, owners in role_candidates.items() if len(owners) > 1}
    if missing_raw:
        raise ValueError(f"env {env} does not map logical process roles: {', '.join(missing_raw)}")
    if duplicate_raw:
        detail = "; ".join(f"{role}={','.join(owners)}" for role, owners in sorted(duplicate_raw.items()))
        raise ValueError(f"env {env} has duplicate logical role owners: {detail}")
    selected_names = set(support_processes)
    selected_names.update(owners[0] for owners in role_candidates.values())
    missing_support_definitions = sorted(selected_names - set(definitions))
    if missing_support_definitions:
        raise ValueError(f"env {env} references unknown support processes: {', '.join(missing_support_definitions)}")

    repository_root = _repository_root(graph.root)
    available = tuple(
        sorted(
            (
                _resolve_process(
                    name,
                    definitions[name],
                    env=env,
                    manager=manager,
                    repository_root=repository_root,
                )
                for name in selected_names
                if _process_supports_host_platform(
                    name,
                    definitions[name],
                    env=env,
                    manager=manager,
                )
            ),
            key=lambda process: (process.order, process.name),
        )
    )
    targets = [process.target for process in available]
    if len(set(targets)) != len(targets):
        raise ValueError(f"env {env} has duplicate selected process targets")

    role_owners = _role_owners(available, env=env)
    available_by_name = {process.name: process for process in available}
    missing_support = sorted(set(support_processes) - set(available_by_name))
    if missing_support:
        raise ValueError(f"env {env} references unknown support processes: {', '.join(missing_support)}")
    invalid_support = tuple(
        process_name
        for process_name in support_processes
        if available_by_name[process_name].provides or available_by_name[process_name].lifecycle != "mode"
    )
    if invalid_support:
        raise ValueError(
            f"env {env} support processes must be mode processes without logical roles: {', '.join(invalid_support)}"
        )
    if not set(role_owners).issubset(provided_roles):
        raise ValueError(f"env {env} selected process roles are not declared by provided_roles")
    missing = sorted(set(names) - set(role_owners))
    if missing:
        platform_detail = f" for host platform {_host_process_platform()!r}" if manager == "direct" else ""
        raise ValueError(f"env {env} does not map logical process roles{platform_detail}: {', '.join(missing)}")

    conflicts = _target_names(
        implementation.get("conflicts"),
        env=env,
        manager=manager,
    )
    overlap = sorted(set(targets) & set(conflicts))
    if overlap:
        raise ValueError(f"env {env} process targets also appear as conflicts: {', '.join(overlap)}")
    return available, available, conflicts, support_processes


def resolve_stop_before_start(
    implementation: Mapping[str, Any],
    processes: tuple[ProcessSpec, ...],
    conflicts: tuple[str, ...],
    *,
    owner: str,
) -> tuple[str, ...]:
    """Resolve the ordered process targets used for stop and rollback.

    ``stop_before_start`` must cover every mode-owned physical process declared
    by the Env exactly once. A Product uses the declared relative order after
    targets it did not select are filtered out; persistent processes and
    conflict tombstones are not part of that ordered list.
    """

    mode_targets = tuple(process.target for process in processes if process.lifecycle == "mode")
    raw_stop_before_start = implementation.get("stop_before_start")
    if raw_stop_before_start is None:
        raise ValueError(f"{owner} stop_before_start is required")
    if not isinstance(raw_stop_before_start, list | tuple):
        raise ValueError(f"{owner} stop_before_start must be a list")
    ordered = tuple(raw_stop_before_start)
    if any(not isinstance(target, str) or target != target.strip() for target in ordered) or len(
        set(ordered)
    ) != len(ordered):
        raise ValueError(f"{owner} stop_before_start contains invalid or duplicate targets")
    declared_mode_targets = {
        str(value.get("target"))
        for value in (
            implementation.get("processes", {}).values()
            if isinstance(implementation.get("processes"), Mapping)
            else ()
        )
        if isinstance(value, Mapping) and value.get("lifecycle", "mode") == "mode"
    }
    known_mode_targets = set(mode_targets) | declared_mode_targets
    missing = sorted(set(mode_targets) - set(ordered))
    unknown = sorted(set(ordered) - known_mode_targets)
    ordered = tuple(target for target in ordered if target in mode_targets)
    if missing or unknown:
        details: list[str] = []
        if missing:
            details.append(f"missing={missing}")
        if unknown:
            details.append(f"unknown={unknown}")
        raise ValueError(f"{owner} stop_before_start must cover every mode process exactly once ({', '.join(details)})")
    overlap = sorted(set(ordered) & set(conflicts))
    if overlap:
        raise ValueError(f"{owner} stop_before_start targets also appear as conflicts: {', '.join(overlap)}")
    return (*ordered, *conflicts)


def resolve_env_implementation(
    env: str,
    *,
    graph: RuntimeGraph | None = None,
    env_config: Mapping[str, Any] | None = None,
) -> Mapping[str, Any]:
    """Return the selected implementation for one public Env.

    ``real`` has one physical implementation. ``sim`` deliberately requires
    ``env_config.backend`` because no simulator backend implements every
    Product.
    """

    graph = graph or load_runtime_graph()
    env_spec = graph.envs.get(env)
    if env_spec is None:
        raise ValueError(f"unknown Runtime Graph env: {env}")
    if not isinstance(env_spec, Mapping):
        raise ValueError(f"Runtime Graph env {env!r} must be a mapping")
    typed_env_spec: Mapping[str, Any] = env_spec

    if env != "sim":
        selected_backend = ""
        if isinstance(env_config, Mapping):
            selected_backend = str(env_config.get("backend") or "").strip()
        if selected_backend:
            raise ValueError(f"env {env} does not accept a backend selector")
        return typed_env_spec

    if not isinstance(env_config, Mapping):
        raise ValueError("env sim requires env_config.backend")
    backend = str(env_config.get("backend") or "").strip()
    if not backend:
        raise ValueError("env sim requires env_config.backend")
    backends = typed_env_spec.get("backends")
    if not isinstance(backends, Mapping):
        raise ValueError("env sim has no backend implementations")
    implementation_value = backends.get(backend)
    if not isinstance(implementation_value, Mapping):
        available = ", ".join(sorted(str(name) for name in backends))
        raise ValueError(f"unknown backend {backend!r} for env sim; available: {available}")
    implementation: Mapping[str, Any] = implementation_value
    return implementation


def _process_names(value: Any, *, product: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"product {product} must declare a process list")
    names = tuple(value)
    invalid = tuple(name for name in names if not valid_process_name(name))
    if not names or invalid:
        raise ValueError(f"product {product} has invalid process names: {invalid}")
    if len(set(names)) != len(names):
        raise ValueError(f"product {product} has duplicate process names")
    return names


def _name_tuple(
    value: Any,
    *,
    owner: str,
    field: str,
    allow_empty: bool = False,
) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"{owner} must declare {field} as a list")
    names = tuple(value)
    if (not allow_empty and not names) or any(not valid_process_name(name) for name in names):
        raise ValueError(f"{owner} has invalid {field}")
    if len(set(names)) != len(names):
        raise ValueError(f"{owner} has duplicate {field}")
    return names


def _raw_process_roles(
    name: Any,
    value: Any,
    *,
    env: str,
    manager: str,
) -> tuple[str, ...]:
    """Read only the outer ownership declaration before artifact resolution."""

    if not valid_process_name(name) or not isinstance(value, Mapping):
        raise ValueError(f"env {env} process {name!r} must be a valid mapping")
    if manager == "systemd":
        if "provides" not in value:
            return (str(name),)
        return _name_tuple(
            value.get("provides"),
            owner=f"env {env} process {name!r}",
            field="provides",
        )
    return _name_tuple(
        value.get("provides"),
        owner=f"env {env} process {name!r}",
        field="provides",
        allow_empty=True,
    )


def _support_process_names(value: Any, *, owner: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise ValueError(f"{owner} must declare support_processes as a list")
    names = tuple(value)
    if any(not valid_process_name(name) for name in names):
        raise ValueError(f"{owner} has invalid support_processes")
    if len(set(names)) != len(names):
        raise ValueError(f"{owner} has duplicate support_processes")
    return names


def _target_names(value: Any, *, env: str, manager: str) -> tuple[str, ...]:
    return _parse_conflict_targets(
        value,
        owner=f"env {env}",
        manager=manager,
    )


def _resolve_process(
    name: Any,
    value: Any,
    *,
    env: str,
    manager: str,
    repository_root: Path | None,
) -> ProcessSpec:
    value, platform = _select_process_platform(
        name,
        value,
        owner=f"env {env}",
        manager=manager,
    )
    return _parse_process_definition(
        name,
        value,
        owner=f"env {env}",
        manager=manager,
        repository_root=repository_root,
        platform=platform,
    )


def _host_process_platform() -> str:
    """Return the private process artifact platform for this compiler host."""

    if sys.platform == "win32":
        return "windows"
    if sys.platform.startswith("linux"):
        return "linux"
    raise ValueError(f"unsupported direct-process host platform: {sys.platform!r}")


def _process_supports_host_platform(
    name: Any,
    value: Any,
    *,
    env: str,
    manager: str,
) -> bool:
    if manager != "direct" or not isinstance(value, Mapping) or "platforms" not in value:
        return True
    _validate_platform_process_shape(name, value, owner=f"env {env}")
    platforms = value["platforms"]
    return _host_process_platform() in platforms


def _select_process_platform(
    name: Any,
    value: Any,
    *,
    owner: str,
    manager: str,
) -> tuple[Any, str | None]:
    if manager != "direct" or not isinstance(value, Mapping) or "platforms" not in value:
        return value, None
    _validate_platform_process_shape(name, value, owner=owner)
    platform = _host_process_platform()
    command = value["platforms"].get(platform)
    if command is None:
        raise ValueError(f"{owner} process {name!r} has no {platform} implementation")
    selected = {key: item for key, item in value.items() if key != "platforms"}
    selected["command"] = command
    return selected, platform


def _validate_platform_process_shape(
    name: Any,
    value: Mapping[Any, Any],
    *,
    owner: str,
) -> None:
    label = f"{owner} process {name!r}"
    _require_exact_fields(value, _DIRECT_PLATFORM_PROCESS_FIELDS, label=label)
    platforms = value.get("platforms")
    if not isinstance(platforms, Mapping) or not platforms:
        raise ValueError(f"{label} platforms must be a non-empty mapping")
    unknown = sorted(repr(item) for item in set(platforms) - _PROCESS_PLATFORMS)
    if unknown:
        raise ValueError(f"{label} platforms contain unknown keys: {', '.join(unknown)}")
    for platform, command in platforms.items():
        _parse_direct_command(
            command,
            owner=f"{label} platform {platform!r}",
            repository_root=None,
            platform=str(platform),
        )


def _parse_process_platform_variants(
    name: Any,
    value: Any,
    *,
    owner: str,
    manager: str,
    repository_root: Path | None,
) -> dict[str | None, ProcessSpec]:
    """Parse every declared platform variant for schema validation."""

    if manager != "direct" or not isinstance(value, Mapping) or "platforms" not in value:
        return {
            None: _parse_process_definition(
                name,
                value,
                owner=owner,
                manager=manager,
                repository_root=repository_root,
                platform=None,
            )
        }
    _validate_platform_process_shape(name, value, owner=owner)
    variants: dict[str | None, ProcessSpec] = {}
    for platform, command in value["platforms"].items():
        selected = {key: item for key, item in value.items() if key != "platforms"}
        selected["command"] = command
        variants[str(platform)] = _parse_process_definition(
            name,
            selected,
            owner=owner,
            manager=manager,
            repository_root=repository_root,
            platform=str(platform),
        )
    return variants


def _parse_process_definition(
    name: Any,
    value: Any,
    *,
    owner: str,
    manager: str,
    repository_root: Path | None,
    platform: str | None,
) -> ProcessSpec:
    """Parse one strict Env process definition.

    ``repository_root=None`` performs schema-only validation and deliberately
    does not require a build artifact to exist. RunPlan compilation supplies a
    root and verifies that each referenced artifact exists.
    """

    if not isinstance(value, Mapping):
        raise ValueError(f"{owner} process {name!r} must be a mapping")
    expected_fields = _DIRECT_PROCESS_FIELDS if manager == "direct" else _PROCESS_BASE_FIELDS
    if manager == "systemd":
        expected_fields = frozenset(
            {*expected_fields, *(_SYSTEMD_PROCESS_OPTIONAL_FIELDS.intersection(value))}
        )
    _require_exact_fields(
        value,
        expected_fields,
        label=f"{owner} process {name!r}",
    )
    target = _required_mapping_text(
        value,
        "target",
        label=f"{owner} process {name!r}",
    )
    lifecycle = _required_mapping_text(
        value,
        "lifecycle",
        label=f"{owner} process {name!r}",
    )
    command: ProcessCommand | None = None
    provides: Any = ()
    if manager == "direct":
        provides = value.get("provides")
        command = _parse_direct_command(
            value.get("command"),
            owner=f"{owner} process {name!r}",
            repository_root=repository_root,
            platform=platform,
        )
    elif manager == "systemd":
        provides = value.get("provides", ())
    order_value = value.get("order")
    if isinstance(order_value, bool) or not isinstance(order_value, int) or order_value < 0:
        raise ValueError(f"process order is invalid: {order_value!r}")
    timeout_value = value.get("timeout_s")
    if isinstance(timeout_value, bool) or not isinstance(timeout_value, int) or timeout_value <= 0:
        raise ValueError(f"process timeout is invalid: {timeout_value!r}")
    return ProcessSpec(
        name=name,
        manager=manager,
        target=target,
        order=order_value,
        timeout_s=timeout_value,
        lifecycle=lifecycle,
        command=command,
        provides=provides,
    )


def _parse_direct_command(
    value: Any,
    *,
    owner: str,
    repository_root: Path | None,
    platform: str | None,
) -> ProcessCommand:
    label = f"{owner} command"
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    expected_fields = set(_DIRECT_COMMAND_FIELDS)
    if "shutdown" in value:
        expected_fields.add("shutdown")
    if "dependencies" in value:
        expected_fields.add("dependencies")
    _require_exact_fields(value, frozenset(expected_fields), label=label)

    artifact_value = value.get("artifact")
    if not isinstance(artifact_value, Mapping):
        raise ValueError(f"{label} artifact must be a mapping")
    _require_exact_fields(
        artifact_value,
        _ENV_ARTIFACT_FIELDS,
        label=f"{label} artifact",
    )
    artifact_path = artifact_value.get("path")
    artifact = (
        ProcessArtifact.from_repository_path(repository_root, artifact_path)
        if repository_root is not None
        else ProcessArtifact(
            _safe_relative_path(
                artifact_path,
                field="artifact path",
                root_label="repository",
            )
        )
    )
    dependencies_value = value.get("dependencies", [])
    if not isinstance(dependencies_value, list):
        raise ValueError(f"{label} dependencies must be a list")
    dependencies: list[ProcessArtifact] = []
    for index, dependency_value in enumerate(dependencies_value):
        dependency_label = f"{label} dependencies[{index}]"
        if not isinstance(dependency_value, Mapping):
            raise ValueError(f"{dependency_label} must be a mapping")
        _require_exact_fields(
            dependency_value,
            _ENV_ARTIFACT_FIELDS,
            label=dependency_label,
        )
        dependency_path = dependency_value.get("path")
        dependencies.append(
            ProcessArtifact.from_repository_path(
                repository_root,
                dependency_path,
            )
            if repository_root is not None
            else ProcessArtifact(
                _safe_relative_path(
                    dependency_path,
                    field="dependency artifact path",
                    root_label="repository",
                )
            )
        )

    readiness_value = value.get("readiness")
    if not isinstance(readiness_value, Mapping):
        raise ValueError(f"{label} readiness must be a mapping")
    readiness_kind = _required_mapping_text(
        readiness_value,
        "kind",
        label=f"{label} readiness",
    )
    readiness_fields = _READINESS_FILE_FIELDS if readiness_kind == "file" else _READINESS_PROCESS_FIELDS
    _require_exact_fields(
        readiness_value,
        readiness_fields,
        label=f"{label} readiness",
    )
    readiness = ProcessReadiness(
        readiness_kind,
        target=readiness_value.get("target"),
    )
    shutdown: ProcessShutdown | None = None
    if "shutdown" in value:
        shutdown_value = value.get("shutdown")
        if not isinstance(shutdown_value, Mapping):
            raise ValueError(f"{label} shutdown must be a mapping")
        shutdown_kind = _required_mapping_text(
            shutdown_value,
            "kind",
            label=f"{label} shutdown",
        )
        shutdown_fields = _SHUTDOWN_FILE_FIELDS if shutdown_kind == "file" else _SHUTDOWN_PROCESS_FIELDS
        _require_exact_fields(
            shutdown_value,
            shutdown_fields,
            label=f"{label} shutdown",
        )
        shutdown = ProcessShutdown(
            shutdown_kind,
            target=shutdown_value.get("target"),
            schema=shutdown_value.get("schema"),
        )
    command_argv = _normalized_process_argv(value.get("argv"))
    command_cwd = _safe_relative_path(
        value.get("cwd"),
        field="cwd",
        root_label="repository",
        allow_current=True,
    )
    command_env = _normalized_process_env(value.get("env"))
    command = ProcessCommand(
        argv=command_argv,
        cwd=command_cwd,
        env=command_env,
        artifact=artifact,
        readiness=readiness,
        shutdown=shutdown,
        dependencies=tuple(dependencies),
    )
    return command


def _require_exact_fields(
    value: Mapping[Any, Any],
    expected: frozenset[str],
    *,
    label: str,
) -> None:
    actual = set(value)
    if actual == set(expected):
        return
    missing = sorted(set(expected) - actual)
    unknown = sorted(repr(item) for item in actual - set(expected))
    detail: list[str] = []
    if missing:
        detail.append(f"missing={missing}")
    if unknown:
        detail.append(f"unknown={unknown}")
    raise ValueError(f"{label} fields are invalid ({', '.join(detail)})")


def _required_mapping_text(
    value: Mapping[Any, Any],
    field: str,
    *,
    label: str,
) -> str:
    item = value.get(field)
    if not isinstance(item, str) or not item or item != item.strip():
        raise ValueError(f"{label} {field} must be a non-empty trimmed string")
    return item


def _repository_root(graph_root: Path) -> Path:
    resolved = graph_root.resolve()
    if resolved.name == "runtime_graph" and resolved.parent.name == "config":
        return resolved.parent.parent
    return resolved


def _role_owners(
    processes: tuple[ProcessSpec, ...],
    *,
    env: str,
) -> dict[str, ProcessSpec]:
    owners: dict[str, ProcessSpec] = {}
    for process in processes:
        for role in process.provides:
            existing = owners.setdefault(role, process)
            if existing.name != process.name:
                raise ValueError(
                    f"env {env} has duplicate logical role owner for {role}: {existing.name}, {process.name}"
                )
    return owners


def _parse_conflict_targets(
    value: Any,
    *,
    owner: str,
    manager: str,
) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list | tuple):
        raise ValueError(f"{owner} conflicts must be a list")
    targets = tuple(value)
    if any(
        not isinstance(target, str) or target != target.strip() or not _valid_target(target, manager=manager)
        for target in targets
    ) or len(set(targets)) != len(targets):
        raise ValueError(f"{owner} has invalid or duplicate conflict targets")
    return targets


def valid_process_name(value: Any) -> bool:
    """Return whether *value* is a valid physical process or logical role token."""

    return isinstance(value, str) and value == value.strip() and _PROCESS_NAME.fullmatch(value) is not None


def _valid_target(value: str, *, manager: str) -> bool:
    if manager == "systemd":
        return _SYSTEMD_TARGET.fullmatch(value) is not None
    if manager in {"direct", "external"}:
        return _OPAQUE_TARGET.fullmatch(value) is not None
    return False
