"""Resolved Product execution plan."""

from __future__ import annotations

import json
import math
import os
import re
from copy import deepcopy
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, cast

from runtime.contracts.product_runtime import resolve_product_runtime_contracts
from runtime.graph import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessShutdown,
    ProcessSpec,
)

RUN_PLAN_SCHEMA = "lingtu.run_plan.v7"
CURRENT_RUN_SCHEMA = "lingtu.current.v1"
SIMULATION_SCHEMA = "lingtu.run_plan.simulation.v1"

_TOP_LEVEL_FIELDS = frozenset({"identity", "launch", "host", "checks"})
_IDENTITY_FIELDS = frozenset({"schema", "product", "product_variant", "env", "robot"})
_LAUNCH_FIELDS = frozenset(
    {
        "controller",
        "process_catalog",
        "stop_before_start",
        "native_process_environment",
        "session",
        "parameter_profile",
        "parameter_overrides",
        "simulation",
    }
)
_PROCESS_CATALOG_FIELDS = frozenset({"selected", "available", "support_processes"})
_HOST_FIELDS = frozenset({"config", "expected_modules", "route_contract"})
_CHECK_FIELDS = frozenset({"contracts", "critical_modules"})
_PROCESS_FIELDS = frozenset({"name", "manager", "target", "order", "timeout_s", "lifecycle"})
_SIMULATION_FIELDS = frozenset({"schema", "session_source", "session", "physics_plan"})
_SIMULATION_BUNDLE_SCHEMAS = {
    "visual_plan": "lingtu.sim.visual-plan.v1",
    "sensor_plan": "lingtu.sim.sensor-plan.v1",
    "control_plan": "lingtu.sim.control-plan.v1",
    "transport_intent": "lingtu.sim.transport-intent.v1",
}
_SIMULATION_BUNDLE_FIELDS = frozenset({*_SIMULATION_BUNDLE_SCHEMAS, "scenario_plan"})
_ENVIRONMENT_KEY = re.compile(r"[A-Z][A-Z0-9_]*\Z")
_NATIVE_NAV_ENVIRONMENT_KEYS = {
    "control_mode": "LINGTU_NAV_CONTROL_MODE",
    "global_planner": "NAV_GLOBAL_PLANNER",
    "local_planner": "LINGTU_NAV_LOCAL_PLANNER_BACKEND",
    "publish_cmd_vel": "LINGTU_NAV_PUBLISH_CMD_VEL",
    "check_obstacle": "LINGTU_NAV_CHECK_OBSTACLE",
    "use_traversability_cost": "LINGTU_NAV_USE_TRAVERSABILITY_COST",
    "allow_teleop_takeover": "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER",
    "teleop_local_planner": "LINGTU_TELEOP_LOCAL_PLANNER",
}
_NATIVE_NAV_TEXT_FIELDS = frozenset({"control_mode", "global_planner", "local_planner"})


@dataclass(frozen=True)
class RunPlan:
    """Concrete processes and Host configuration for one Product and env."""

    schema_version: str
    product: str
    product_variant: str | None
    env: str
    robot: str
    process_control: str
    modules: tuple[str, ...]
    processes: tuple[ProcessSpec, ...]
    available_processes: tuple[ProcessSpec, ...]
    stop_before_start: tuple[str, ...]
    contracts: tuple[str, ...]
    critical_modules: tuple[str, ...]
    route_contract: str | None
    parameter_profile: str | None
    _native_process_environment: dict[str, str] = dataclass_field(repr=False)
    _host_config: dict[str, Any] = dataclass_field(repr=False)
    _lifecycle: dict[str, Any] = dataclass_field(repr=False)
    _parameter_overrides: dict[str, Any] = dataclass_field(repr=False)
    support_processes: tuple[str, ...] = ()
    _simulation: dict[str, Any] = dataclass_field(default_factory=dict, repr=False)

    @classmethod
    def create(
        cls,
        *,
        product: str,
        product_variant: str | None = None,
        env: str,
        robot: str,
        process_control: str,
        modules: tuple[str, ...],
        processes: tuple[ProcessSpec, ...],
        available_processes: tuple[ProcessSpec, ...],
        stop_before_start: tuple[str, ...],
        contracts: tuple[str, ...],
        critical_modules: tuple[str, ...],
        route_contract: str | None,
        host_config: Mapping[str, Any],
        lifecycle: Mapping[str, Any],
        native_process_environment: Mapping[str, str] | None = None,
        native_nav: Mapping[str, Any] | None = None,
        parameter_profile: str | None = None,
        parameter_overrides: Mapping[str, Any] | None = None,
        simulation: Mapping[str, Any] | None = None,
        support_processes: tuple[str, ...] = (),
    ) -> RunPlan:
        """Create a normalized RunPlan from resolved Product data."""
        normalized_product = _required_value_text(product, field="product")
        normalized_variant = _optional_text(product_variant)
        normalized_env = _env(env)
        normalized_robot = _required_value_text(robot, field="robot")
        controller = _required_value_text(process_control, field="controller")
        selected = _normalize_processes(processes, field="process_catalog.selected")
        available = _normalize_processes(available_processes, field="process_catalog.available")
        support = tuple(sorted(_strings(support_processes, field="process_catalog.support_processes")))
        _validate_process_catalog(selected, available, support)
        resolved_contracts = resolve_product_runtime_contracts(contracts, owner="RunPlan checks")
        normalized_host_config = _json_object(host_config, field="host.config")
        normalized_lifecycle = _json_object(lifecycle, field="launch.session")
        _validate_product_variant_identity(
            normalized_variant,
            host_config=normalized_host_config,
            lifecycle=normalized_lifecycle,
        )
        return cls(
            schema_version=RUN_PLAN_SCHEMA,
            product=normalized_product,
            product_variant=normalized_variant,
            env=normalized_env,
            robot=normalized_robot,
            process_control=controller,
            modules=_strings(modules, field="expected_modules"),
            processes=selected,
            available_processes=available,
            support_processes=support,
            stop_before_start=_strings(stop_before_start, field="stop_before_start"),
            contracts=resolved_contracts.contract_ids,
            critical_modules=_strings(critical_modules, field="critical_modules"),
            route_contract=_optional_text(route_contract),
            parameter_profile=_optional_text(parameter_profile),
            _native_process_environment=_normalize_native_environment(
                native_process_environment,
                native_nav=native_nav,
            ),
            _host_config=normalized_host_config,
            _lifecycle=normalized_lifecycle,
            _parameter_overrides=_json_object(parameter_overrides or {}, field="launch.parameter_overrides"),
            _simulation=_normalize_simulation(
                simulation or {},
                env=normalized_env,
                controller=controller,
            ),
        )

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> RunPlan:
        """Parse a RunPlan from its serialized mapping."""
        _require_fields(payload, _TOP_LEVEL_FIELDS, context="RunPlan top level")
        identity = _object(payload.get("identity"), field="identity")
        launch = _object(payload.get("launch"), field="launch")
        host = _object(payload.get("host"), field="host")
        checks = _object(payload.get("checks"), field="checks")
        _require_fields(identity, _IDENTITY_FIELDS, context="RunPlan identity")
        _require_fields(launch, _LAUNCH_FIELDS, context="RunPlan launch")
        _require_fields(host, _HOST_FIELDS, context="RunPlan host")
        _require_fields(checks, _CHECK_FIELDS, context="RunPlan checks")
        schema = _required_text(identity, "schema")
        if schema != RUN_PLAN_SCHEMA:
            raise ValueError(f"unsupported RunPlan schema: {schema!r}")
        catalog = _object(launch.get("process_catalog"), field="launch.process_catalog")
        _require_fields(catalog, _PROCESS_CATALOG_FIELDS, context="RunPlan launch.process_catalog")
        return cls.create(
            product=_required_text(identity, "product"),
            product_variant=_optional_text(identity.get("product_variant")),
            env=_required_text(identity, "env"),
            robot=_required_text(identity, "robot"),
            process_control=_required_text(launch, "controller"),
            modules=_strings(host.get("expected_modules"), field="host.expected_modules"),
            processes=_processes(catalog.get("selected"), field="launch.process_catalog.selected"),
            available_processes=_processes(catalog.get("available"), field="launch.process_catalog.available"),
            support_processes=_strings(
                catalog.get("support_processes"),
                field="launch.process_catalog.support_processes",
            ),
            stop_before_start=_strings(launch.get("stop_before_start"), field="launch.stop_before_start"),
            contracts=_strings(checks.get("contracts"), field="checks.contracts"),
            critical_modules=_strings(checks.get("critical_modules"), field="checks.critical_modules"),
            native_process_environment=_native_environment(launch.get("native_process_environment")),
            route_contract=_optional_text(host.get("route_contract")),
            host_config=_object(host.get("config"), field="host.config"),
            lifecycle=_object(launch.get("session"), field="launch.session"),
            parameter_profile=_optional_text(launch.get("parameter_profile")),
            parameter_overrides=_object(launch.get("parameter_overrides"), field="launch.parameter_overrides"),
            simulation=_object(launch.get("simulation"), field="launch.simulation"),
        )

    @classmethod
    def load(cls, path: str | Path) -> RunPlan:
        """Load a RunPlan from JSON."""
        plan_path = Path(path)
        try:
            payload = json.loads(plan_path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, ValueError) as exc:
            raise ValueError(f"RunPlan JSON is invalid: {plan_path}: {exc}") from exc
        if not isinstance(payload, Mapping):
            raise ValueError(f"RunPlan must be a JSON object: {plan_path}")
        return cls.from_dict(payload)

    @property
    def controller(self) -> str:
        """Return the process controller name."""
        return self.process_control

    @property
    def required_topics(self) -> tuple[str, ...]:
        """Return topics required by the Product contracts."""
        return cast(tuple[str, ...], resolve_product_runtime_contracts(self.contracts).topics)

    @property
    def required_capabilities(self) -> tuple[str, ...]:
        """Return capabilities required by the Product contracts."""
        return cast(tuple[str, ...], resolve_product_runtime_contracts(self.contracts).capabilities)

    @property
    def native_process_environment(self) -> dict[str, str]:
        """Return a copy of the native process environment."""
        return dict(self._native_process_environment)

    @property
    def native_nav(self) -> dict[str, Any]:
        """Return native navigation settings derived from the environment."""
        return _native_nav_from_environment(self._native_process_environment)

    @property
    def host_config(self) -> dict[str, Any]:
        """Return a copy of the Host configuration."""
        return deepcopy(self._host_config)

    @property
    def lifecycle(self) -> dict[str, Any]:
        """Return a copy of the lifecycle settings."""
        return deepcopy(self._lifecycle)

    @property
    def simulation(self) -> dict[str, Any]:
        """Return a copy of the simulation snapshot."""
        return deepcopy(self._simulation)

    @property
    def parameter_overrides(self) -> dict[str, Any]:
        """Return a copy of Product parameter overrides."""
        return deepcopy(self._parameter_overrides)

    @property
    def managed_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes owned for the lifetime of this Product run."""
        return tuple(process for process in self.processes if process.lifecycle == "mode")

    def has_process(self, name: str) -> bool:
        """Return whether a selected process provides the named role."""
        return any(name in process.provides for process in self.processes)

    def process(self, name: str) -> ProcessSpec:
        """Return the selected process that provides the named role."""
        for process in self.processes:
            if name in process.provides:
                return process
        raise KeyError(name)

    def build(self) -> Any:
        """Build the Host Blueprint described by this plan."""
        from lingtu.assembly.compiler import build_host_from_run_plan

        return build_host_from_run_plan(self)

    def as_dict(self) -> dict[str, Any]:
        """Serialize the RunPlan to its JSON-compatible structure."""
        return {
            "identity": {
                "schema": RUN_PLAN_SCHEMA,
                "product": self.product,
                "product_variant": self.product_variant,
                "env": self.env,
                "robot": self.robot,
            },
            "launch": {
                "controller": self.process_control,
                "process_catalog": {
                    "selected": [process.as_dict() for process in self.processes],
                    "available": [process.as_dict() for process in self.available_processes],
                    "support_processes": list(self.support_processes),
                },
                "stop_before_start": list(self.stop_before_start),
                "native_process_environment": self.native_process_environment,
                "session": self.lifecycle,
                "parameter_profile": self.parameter_profile,
                "parameter_overrides": self.parameter_overrides,
                "simulation": self.simulation,
            },
            "host": {
                "config": self.host_config,
                "expected_modules": list(self.modules),
                "route_contract": self.route_contract,
            },
            "checks": {
                "contracts": list(self.contracts),
                "critical_modules": list(self.critical_modules),
            },
        }

    def with_native_process_environment(self, native_process_environment: Mapping[str, str]) -> RunPlan:
        """Return a copy with a different native process environment."""
        payload = self.as_dict()
        payload["launch"]["native_process_environment"] = dict(native_process_environment)
        return RunPlan.from_dict(payload)

    def summary(self) -> dict[str, Any]:
        """Return the compact fields used by status and diagnostics."""
        return {
            "kind": "run_plan",
            "product": self.product,
            "robot": self.robot,
            "product_variant": self.product_variant,
            "env": self.env,
            "controller": self.process_control,
            "processes": [process.name for process in self.processes],
            "host_modules": list(self.modules),
            "contracts": list(self.contracts),
            "route_contract": self.route_contract,
            "parameter_profile": self.parameter_profile,
            "slam_mode": self._lifecycle.get("slam_mode"),
            "native_control_mode": self._lifecycle.get("native_control_mode"),
            "requires_map": bool(self._lifecycle.get("requires_map", False)),
        }

    def write(self, path: str | Path) -> Path:
        """Write the RunPlan as JSON and return its path."""
        plan_path = Path(path)
        plan_path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = plan_path.with_name(f".{plan_path.name}.tmp")
        try:
            temp_path.write_text(
                json.dumps(self.as_dict(), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.chmod(temp_path, 0o644)
            os.replace(temp_path, plan_path)
        finally:
            temp_path.unlink(missing_ok=True)
        return plan_path


def _required_text(payload: Mapping[str, Any], field: str) -> str:
    return _required_value_text(payload.get(field), field=field)


def _required_value_text(value: Any, *, field: str) -> str:
    normalized = _optional_text(value)
    if normalized is None:
        raise ValueError(f"RunPlan is missing {field}")
    return normalized


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise ValueError("RunPlan text values must be strings")
    return value.strip() or None


def _env(value: Any) -> str:
    env = _optional_text(value)
    if env not in {"real", "sim"}:
        raise ValueError(f"RunPlan env must be 'real' or 'sim', received {env!r}")
    return env


def _strings(value: Any, *, field: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"RunPlan {field} must be a list")
    values = tuple(value)
    if any(not isinstance(item, str) or not item or item != item.strip() for item in values):
        raise ValueError(f"RunPlan {field} contains invalid values")
    if len(set(values)) != len(values):
        raise ValueError(f"RunPlan {field} contains duplicate values")
    return values


def _object(value: Any, *, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"RunPlan {field} must be an object")
    return value


def _json_object(value: Any, *, field: str) -> dict[str, Any]:
    normalized = _json_value(value, field=field)
    if not isinstance(normalized, dict):
        raise ValueError(f"RunPlan {field} must be an object")
    return normalized


def _json_value(value: Any, *, field: str) -> Any:
    if value is None or isinstance(value, str | bool | int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"RunPlan {field} must contain finite JSON data")
        return value
    if isinstance(value, list):
        return [_json_value(item, field=f"{field}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, Mapping):
        normalized: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"RunPlan {field} keys must be strings")
            normalized[key] = _json_value(item, field=f"{field}.{key}")
        return normalized
    raise ValueError(f"RunPlan {field} must contain JSON data")


def _validate_product_variant_identity(
    product_variant: str | None,
    *,
    host_config: Mapping[str, Any],
    lifecycle: Mapping[str, Any],
) -> None:
    host_has_variant = "_product_variant" in host_config
    host_variant = _optional_text(host_config.get("_product_variant"))
    lifecycle_variant = _optional_text(lifecycle.get("product_variant"))
    if product_variant is None and host_has_variant:
        raise ValueError("RunPlan Host config declares a variant for a non-variant Product")
    if product_variant is not None and (not host_has_variant or host_variant != product_variant):
        raise ValueError("RunPlan Host config variant does not match identity.product_variant")
    if lifecycle_variant != product_variant:
        raise ValueError("RunPlan lifecycle variant does not match identity.product_variant")


def repository_relative_path(value: Any, *, field: str) -> str:
    """Return a validated repository-relative POSIX path."""
    raw_path = _required_value_text(value, field=field)
    if "\\" in raw_path:
        raise ValueError(f"RunPlan {field} must use repository-relative POSIX syntax")
    path = PurePosixPath(raw_path)
    if (
        path.is_absolute()
        or path.as_posix() != raw_path
        or any(part in {"", ".", ".."} or ":" in part for part in path.parts)
    ):
        raise ValueError(f"RunPlan {field} must be a safe repository-relative path")
    return raw_path


def _normalize_simulation(
    value: Mapping[str, Any],
    *,
    env: str,
    controller: str,
) -> dict[str, Any]:
    simulation = _json_object(value, field="launch.simulation")
    if not simulation:
        if env == "sim" and controller == "subprocess":
            raise ValueError("RunPlan env=sim subprocess requires launch.simulation")
        return {}
    if env != "sim":
        raise ValueError("RunPlan launch.simulation is only valid for env=sim")
    observed = set(simulation)
    present_bundle = observed & _SIMULATION_BUNDLE_FIELDS
    expected = _SIMULATION_FIELDS | (_SIMULATION_BUNDLE_FIELDS if present_bundle else frozenset())
    _require_fields(simulation, expected, context="RunPlan launch.simulation")
    if simulation["schema"] != SIMULATION_SCHEMA:
        raise ValueError(f"RunPlan launch.simulation.schema must be {SIMULATION_SCHEMA!r}")
    simulation["session_source"] = repository_relative_path(
        simulation["session_source"],
        field="launch.simulation.session_source",
    )
    session = _json_object(simulation["session"], field="launch.simulation.session")
    physics_plan = _json_object(simulation["physics_plan"], field="launch.simulation.physics_plan")
    session_id = _required_text(session, "session_id")
    if _required_text(physics_plan, "session_id") != session_id:
        raise ValueError("RunPlan launch.simulation.physics_plan.session_id must match the session")
    simulation["session"] = session
    simulation["physics_plan"] = physics_plan
    if not present_bundle:
        return simulation
    for field, schema in _SIMULATION_BUNDLE_SCHEMAS.items():
        simulation[field] = _session_plan(simulation[field], field=field, schema=schema, session_id=session_id)
    scenario = simulation["scenario_plan"]
    if (scenario is None) != ("scenario" not in session):
        raise ValueError("RunPlan launch.simulation.scenario_plan must match launch.simulation.session.scenario")
    if scenario is not None:
        simulation["scenario_plan"] = _session_plan(
            scenario,
            field="scenario_plan",
            schema="lingtu.sim.scenario-plan.v1",
            session_id=session_id,
        )
    return simulation


def _session_plan(value: Any, *, field: str, schema: str, session_id: str) -> dict[str, Any]:
    plan = _json_object(value, field=f"launch.simulation.{field}")
    if plan.get("schema") != schema:
        raise ValueError(f"RunPlan launch.simulation.{field}.schema must be {schema!r}")
    if plan.get("session_id") != session_id:
        raise ValueError(f"RunPlan launch.simulation.{field}.session_id must match the session")
    return plan


def _processes(value: Any, *, field: str) -> tuple[ProcessSpec, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"RunPlan {field} must be a list")
    return _normalize_processes(tuple(_process(item, field=field) for item in value), field=field)


def _normalize_processes(value: Any, *, field: str) -> tuple[ProcessSpec, ...]:
    if not isinstance(value, list | tuple) or any(not isinstance(process, ProcessSpec) for process in value):
        raise ValueError(f"RunPlan {field} must contain ProcessSpec values")
    processes = tuple(value)
    if len({process.name for process in processes}) != len(processes):
        raise ValueError(f"RunPlan {field} contains duplicate process names")
    if len({process.target for process in processes}) != len(processes):
        raise ValueError(f"RunPlan {field} contains duplicate process targets")
    owners: dict[str, str] = {}
    for process in processes:
        for role in process.provides:
            previous = owners.setdefault(role, process.name)
            if previous != process.name:
                raise ValueError(f"RunPlan {field} has duplicate role owner for {role}: {previous}, {process.name}")
    return processes


def _validate_process_catalog(
    selected: tuple[ProcessSpec, ...],
    available: tuple[ProcessSpec, ...],
    support_processes: tuple[str, ...],
) -> None:
    if not available:
        if selected or support_processes:
            raise ValueError("RunPlan selected and support processes require an available process catalog")
        return
    available_by_name = {process.name: process for process in available}
    selected_by_name = {process.name: process for process in selected}
    for process in selected:
        if available_by_name.get(process.name) != process:
            raise ValueError(f"RunPlan selected process disagrees with the available catalog: {process.name}")
    support_set = set(support_processes)
    role_free_direct = {
        process.name for process in (*available, *selected) if process.manager == "direct" and not process.provides
    }
    if role_free_direct != support_set:
        raise ValueError("RunPlan support_processes must identify the role-free direct processes")
    for name in support_processes:
        process = available_by_name.get(name)
        if process is None or selected_by_name.get(name) != process or process.lifecycle != "mode":
            raise ValueError(f"RunPlan support process is not selected as a mode process: {name}")


def _process(value: Any, *, field: str) -> ProcessSpec:
    process = _object(value, field=f"{field} entry")
    manager = _required_text(process, "manager")
    expected = set(_PROCESS_FIELDS)
    if manager == "direct":
        expected.update({"command", "provides"})
    elif manager == "systemd":
        expected.update(set(process) & {"provides"})
    _require_fields(process, frozenset(expected), context=f"RunPlan {field} entry")
    name = process.get("name")
    command = _process_command(process["command"], field=f"{field}.{name}") if manager == "direct" else None
    provides = _strings(process.get("provides", ()), field=f"{field}.{name}.provides")
    return ProcessSpec(
        name=name,
        manager=manager,
        target=process.get("target"),
        order=process.get("order"),
        timeout_s=process.get("timeout_s"),
        lifecycle=process.get("lifecycle"),
        command=command,
        provides=provides,
    )


def _process_command(value: Any, *, field: str) -> ProcessCommand:
    command = _object(value, field=f"{field}.command")
    expected = {"argv", "cwd", "env", "artifact", "readiness"}
    expected.update(set(command) & {"dependencies", "shutdown"})
    _require_fields(command, frozenset(expected), context=f"RunPlan {field}.command")
    dependencies = command.get("dependencies", [])
    if not isinstance(dependencies, list):
        raise ValueError(f"RunPlan {field}.command.dependencies must be a list")
    return ProcessCommand(
        argv=command.get("argv"),
        cwd=command.get("cwd"),
        env=_object(command.get("env"), field=f"{field}.command.env"),
        artifact=_process_artifact(command.get("artifact"), field=f"{field}.command.artifact"),
        readiness=_process_readiness(command.get("readiness"), field=f"{field}.command.readiness"),
        shutdown=(
            _process_shutdown(command.get("shutdown"), field=f"{field}.command.shutdown")
            if "shutdown" in command
            else None
        ),
        dependencies=tuple(
            _process_artifact(item, field=f"{field}.command.dependencies[{index}]")
            for index, item in enumerate(dependencies)
        ),
    )


def _process_artifact(value: Any, *, field: str) -> ProcessArtifact:
    artifact = _object(value, field=field)
    _require_fields(artifact, frozenset({"path"}), context=f"RunPlan {field}")
    return ProcessArtifact(path=artifact.get("path"))


def _process_readiness(value: Any, *, field: str) -> ProcessReadiness:
    readiness = _object(value, field=field)
    kind = _required_text(readiness, "kind")
    expected = frozenset({"kind", "target"} if kind == "file" else {"kind"})
    _require_fields(readiness, expected, context=f"RunPlan {field}")
    return ProcessReadiness(kind=kind, target=readiness.get("target"))


def _process_shutdown(value: Any, *, field: str) -> ProcessShutdown:
    shutdown = _object(value, field=field)
    kind = _required_text(shutdown, "kind")
    expected = frozenset({"kind", "target", "schema"} if kind == "file" else {"kind"})
    _require_fields(shutdown, expected, context=f"RunPlan {field}")
    return ProcessShutdown(kind=kind, target=shutdown.get("target"), schema=shutdown.get("schema"))


def _normalize_native_environment(
    value: Mapping[str, str] | None,
    *,
    native_nav: Mapping[str, Any] | None,
) -> dict[str, str]:
    if value is None:
        if native_nav is None:
            raise ValueError("RunPlan launch.native_process_environment is required")
        environment = _native_environment_from_nav(native_nav)
    else:
        environment = _native_environment(value)
    if native_nav is not None and _native_nav_from_environment(environment) != _normalized_native_nav(native_nav):
        raise ValueError("RunPlan native_nav disagrees with native_process_environment")
    return environment


def _native_environment(value: Any) -> dict[str, str]:
    if not isinstance(value, Mapping):
        raise ValueError("RunPlan launch.native_process_environment must be an object")
    environment: dict[str, str] = {}
    for key, item in value.items():
        if not isinstance(key, str) or _ENVIRONMENT_KEY.fullmatch(key) is None:
            raise ValueError(f"RunPlan has invalid environment key: {key!r}")
        if not isinstance(item, str) or any(character in item for character in ("\x00", "\n", "\r")):
            raise ValueError(f"RunPlan environment value must be a single-line string: {key}")
        environment[key] = item
    _native_nav_from_environment(environment)
    return dict(sorted(environment.items()))


def _normalized_native_nav(value: Mapping[str, Any]) -> dict[str, Any]:
    native_nav = {**value, "local_planner": value.get("local_planner", "cmu")}
    _require_fields(native_nav, frozenset(_NATIVE_NAV_ENVIRONMENT_KEYS), context="RunPlan native_nav")
    normalized = {
        field: _required_value_text(native_nav[field], field=f"native_nav.{field}")
        for field in _NATIVE_NAV_TEXT_FIELDS
    }
    for field in set(_NATIVE_NAV_ENVIRONMENT_KEYS) - _NATIVE_NAV_TEXT_FIELDS:
        item = native_nav[field]
        if not isinstance(item, bool):
            raise ValueError(f"RunPlan native_nav.{field} must be a boolean")
        normalized[field] = item
    return normalized


def _native_environment_from_nav(value: Mapping[str, Any]) -> dict[str, str]:
    native_nav = _normalized_native_nav(value)
    environment = {
        key: str(native_nav[field]) if field in _NATIVE_NAV_TEXT_FIELDS else ("1" if native_nav[field] else "0")
        for field, key in _NATIVE_NAV_ENVIRONMENT_KEYS.items()
    }
    return dict(sorted(environment.items()))


def _native_nav_from_environment(value: Mapping[str, str]) -> dict[str, Any]:
    local_planner_key = _NATIVE_NAV_ENVIRONMENT_KEYS["local_planner"]
    missing = sorted(set(_NATIVE_NAV_ENVIRONMENT_KEYS.values()) - {local_planner_key} - set(value))
    if missing:
        raise ValueError("RunPlan native process environment is missing: " + ", ".join(missing))
    native_nav: dict[str, Any] = {}
    for field, key in _NATIVE_NAV_ENVIRONMENT_KEYS.items():
        if field == "local_planner":
            native_nav[field] = value.get(key, "cmu")
        elif field in _NATIVE_NAV_TEXT_FIELDS:
            native_nav[field] = value[key]
        else:
            native_nav[field] = _environment_bool(value[key])
    return native_nav


def _environment_bool(value: str) -> bool:
    if value in {"0", "1"}:
        return value == "1"
    raise ValueError(f"RunPlan native boolean environment value must be 0 or 1: {value!r}")


def _require_fields(
    value: Mapping[str, Any],
    expected: frozenset[str],
    *,
    context: str,
) -> None:
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{context} field names must be strings")
    observed = set(value)
    missing = sorted(expected - observed)
    unknown = sorted(observed - expected)
    if not missing and not unknown:
        return
    details: list[str] = []
    if missing:
        details.append("missing fields: " + ", ".join(missing))
    if unknown:
        details.append("unsupported fields: " + ", ".join(unknown))
    raise ValueError(f"{context} has invalid fields ({'; '.join(details)})")


__all__ = [
    "CURRENT_RUN_SCHEMA",
    "RUN_PLAN_SCHEMA",
    "SIMULATION_SCHEMA",
    "RunPlan",
    "repository_relative_path",
]
