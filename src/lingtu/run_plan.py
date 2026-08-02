"""Immutable execution plan produced once by ProductControl."""

from __future__ import annotations

import hashlib
import json
import os
import re
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from runtime.contracts.product_runtime import resolve_product_runtime_contracts
from runtime.graph import ProcessSpec

RUN_PLAN_SCHEMA = "lingtu.run_plan.v1"
CURRENT_RUN_SCHEMA = "lingtu.current.v1"

_TOP_LEVEL_FIELDS = frozenset({"identity", "launch", "host", "checks"})
_IDENTITY_FIELDS = frozenset(
    {
        "schema",
        "product",
        "product_variant",
        "env",
        "fingerprint",
        "compiled_against",
    }
)
_LAUNCH_FIELDS = frozenset(
    {
        "controller",
        "process_catalog",
        "stop_plan",
        "native_process_environment",
        "session",
        "parameter_profile",
        "parameter_overrides",
    }
)
_PROCESS_CATALOG_FIELDS = frozenset({"selected", "available"})
_HOST_FIELDS = frozenset(
    {"config", "expected_modules", "route_contract", "module_transport"}
)
_CHECK_FIELDS = frozenset({"contracts", "critical_modules"})
_PROCESS_FIELDS = frozenset(
    {"name", "manager", "target", "order", "timeout_s", "lifecycle"}
)
_ENVIRONMENT_KEY = re.compile(r"[A-Z][A-Z0-9_]*\Z")
_NATIVE_NAV_ENVIRONMENT_KEYS = {
    "control_mode": "LINGTU_NAV_CONTROL_MODE",
    "publish_cmd_vel": "LINGTU_NAV_PUBLISH_CMD_VEL",
    "check_obstacle": "LINGTU_NAV_CHECK_OBSTACLE",
    "use_traversability_cost": "LINGTU_NAV_USE_TRAVERSABILITY_COST",
    "allow_teleop_takeover": "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER",
    "teleop_local_planner": "LINGTU_TELEOP_LOCAL_PLANNER",
}


@dataclass(frozen=True)
class RunPlan:
    """Resolved executable identity for one Product inside one Env."""

    schema_version: str
    product: str
    product_variant: str | None
    env: str
    process_control: str
    modules: tuple[str, ...]
    processes: tuple[ProcessSpec, ...]
    available_processes: tuple[ProcessSpec, ...]
    stop_targets: tuple[str, ...]
    contracts: tuple[str, ...]
    critical_modules: tuple[str, ...]
    route_contract: str | None
    module_transport: str
    parameter_profile: str | None
    _native_process_environment_json: str
    _host_config_json: str
    _lifecycle_json: str
    _parameter_overrides_json: str
    _compiled_against_json: str
    fingerprint: str

    @classmethod
    def create(
        cls,
        *,
        product: str,
        product_variant: str | None = None,
        env: str,
        process_control: str,
        modules: tuple[str, ...],
        processes: tuple[ProcessSpec, ...],
        available_processes: tuple[ProcessSpec, ...],
        stop_targets: tuple[str, ...],
        contracts: tuple[str, ...],
        critical_modules: tuple[str, ...],
        route_contract: str | None,
        module_transport: str,
        host_config: Mapping[str, Any],
        lifecycle: Mapping[str, Any],
        compiled_against: Mapping[str, Any],
        native_process_environment: Mapping[str, str] | None = None,
        native_nav: Mapping[str, Any] | None = None,
        parameter_profile: str | None = None,
        parameter_overrides: Mapping[str, Any] | None = None,
    ) -> RunPlan:
        """Create a run plan and compute its canonical fingerprint."""

        normalized_product = _required_value_text(product, field="product")
        normalized_product_variant = _optional_text(product_variant)
        normalized_env = _env(env)
        normalized_controller = _required_value_text(
            process_control,
            field="controller",
        )
        normalized_modules = _strings(modules, field="expected_modules")
        normalized_processes = _normalize_processes(
            processes,
            field="process_catalog.selected",
        )
        normalized_available = _normalize_processes(
            available_processes,
            field="process_catalog.available",
        )
        _validate_process_catalog(normalized_processes, normalized_available)
        normalized_stop_targets = _strings(stop_targets, field="stop_plan")
        resolved_contracts = resolve_product_runtime_contracts(
            contracts,
            owner="RunPlan checks",
        )
        normalized_critical_modules = _strings(
            critical_modules,
            field="critical_modules",
        )
        normalized_route_contract = _optional_text(route_contract)
        normalized_transport = _required_value_text(
            module_transport,
            field="module_transport",
        )
        normalized_host_config = _json_mapping(host_config, field="host.config")
        normalized_lifecycle = _json_mapping(lifecycle, field="launch.session")
        _validate_product_variant_identity(
            normalized_product_variant,
            host_config=normalized_host_config,
            lifecycle=normalized_lifecycle,
        )
        normalized_compiled_against = _json_mapping(
            compiled_against,
            field="identity.compiled_against",
        )
        normalized_parameter_profile = _optional_text(parameter_profile)
        normalized_parameter_overrides = _json_mapping(
            parameter_overrides or {},
            field="launch.parameter_overrides",
        )
        normalized_native_environment = _normalize_native_environment(
            native_process_environment,
            native_nav=native_nav,
        )
        body = _run_plan_body(
            product=normalized_product,
            product_variant=normalized_product_variant,
            env=normalized_env,
            process_control=normalized_controller,
            modules=normalized_modules,
            processes=normalized_processes,
            available_processes=normalized_available,
            stop_targets=normalized_stop_targets,
            contracts=resolved_contracts.contract_ids,
            critical_modules=normalized_critical_modules,
            native_process_environment=normalized_native_environment,
            route_contract=normalized_route_contract,
            module_transport=normalized_transport,
            host_config=normalized_host_config,
            lifecycle=normalized_lifecycle,
            parameter_profile=normalized_parameter_profile,
            parameter_overrides=normalized_parameter_overrides,
            compiled_against=normalized_compiled_against,
        )
        return cls(
            schema_version=RUN_PLAN_SCHEMA,
            product=normalized_product,
            product_variant=normalized_product_variant,
            env=normalized_env,
            process_control=normalized_controller,
            modules=normalized_modules,
            processes=normalized_processes,
            available_processes=normalized_available,
            stop_targets=normalized_stop_targets,
            contracts=resolved_contracts.contract_ids,
            critical_modules=normalized_critical_modules,
            route_contract=normalized_route_contract,
            module_transport=normalized_transport,
            parameter_profile=normalized_parameter_profile,
            _native_process_environment_json=_canonical_mapping_json(
                normalized_native_environment
            ),
            _host_config_json=_canonical_mapping_json(normalized_host_config),
            _lifecycle_json=_canonical_mapping_json(normalized_lifecycle),
            _parameter_overrides_json=_canonical_mapping_json(
                normalized_parameter_overrides
            ),
            _compiled_against_json=_canonical_mapping_json(
                normalized_compiled_against
            ),
            fingerprint=_fingerprint(body),
        )

    @property
    def schema(self) -> str:
        """Return the serialized schema identifier."""

        return self.schema_version

    @property
    def controller(self) -> str:
        """Return the launch controller selected by the Env."""

        return self.process_control

    @property
    def expected_modules(self) -> tuple[str, ...]:
        """Return the Host modules fingerprinted into the plan."""

        return self.modules

    @property
    def contract_ids(self) -> tuple[str, ...]:
        """Return the named runtime checks embedded in this plan."""

        return self.contracts

    @property
    def required_topics(self) -> tuple[str, ...]:
        """Derive required topics from the code-owned named contracts."""

        return resolve_product_runtime_contracts(self.contracts).topics

    @property
    def required_capabilities(self) -> tuple[str, ...]:
        """Derive required capabilities from the code-owned named contracts."""

        return resolve_product_runtime_contracts(self.contracts).capabilities

    @property
    def native_process_environment(self) -> dict[str, str]:
        """Return an independent copy of the resolved native process environment."""

        loaded = _mapping_from_json(self._native_process_environment_json)
        return {str(key): str(value) for key, value in loaded.items()}

    @property
    def native_nav(self) -> dict[str, Any]:
        """Project legacy native navigation settings from the process environment."""

        return _native_nav_from_environment(self.native_process_environment)

    @property
    def host_config(self) -> dict[str, Any]:
        """Return an independent copy of the Host configuration."""

        return _mapping_from_json(self._host_config_json)

    @property
    def lifecycle(self) -> dict[str, Any]:
        """Return the Product execution/session snapshot compiled into this plan."""

        return _mapping_from_json(self._lifecycle_json)

    @property
    def session(self) -> dict[str, Any]:
        """Return the launch session snapshot."""

        return self.lifecycle

    @property
    def parameter_overrides(self) -> dict[str, Any]:
        """Return the unresolved Env parameter override declaration."""

        return _mapping_from_json(self._parameter_overrides_json)

    @property
    def compiled_against(self) -> dict[str, Any]:
        """Return the code and contract identity used to compile this plan."""

        return _mapping_from_json(self._compiled_against_json)

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> RunPlan:
        """Load and verify a strict RunPlan."""

        _require_fields(payload, _TOP_LEVEL_FIELDS, context="RunPlan top level")
        identity = _object(payload.get("identity"), field="identity")
        launch = _object(payload.get("launch"), field="launch")
        host = _object(payload.get("host"), field="host")
        checks = _object(payload.get("checks"), field="checks")
        _require_fields(identity, _IDENTITY_FIELDS, context="RunPlan identity")
        _require_fields(launch, _LAUNCH_FIELDS, context="RunPlan launch")
        _require_fields(host, _HOST_FIELDS, context="RunPlan host")
        _require_fields(checks, _CHECK_FIELDS, context="RunPlan checks")

        schema_version = _required_text(identity, "schema")
        if schema_version != RUN_PLAN_SCHEMA:
            raise ValueError(f"unsupported RunPlan schema: {schema_version!r}")
        process_catalog = _object(
            launch.get("process_catalog"),
            field="launch.process_catalog",
        )
        _require_fields(
            process_catalog,
            _PROCESS_CATALOG_FIELDS,
            context="RunPlan launch.process_catalog",
        )
        plan = cls.create(
            product=_required_text(identity, "product"),
            product_variant=_optional_text(identity.get("product_variant")),
            env=_required_text(identity, "env"),
            process_control=_required_text(launch, "controller"),
            modules=_strings(
                host.get("expected_modules"),
                field="host.expected_modules",
            ),
            processes=_processes(
                process_catalog.get("selected"),
                field="launch.process_catalog.selected",
            ),
            available_processes=_processes(
                process_catalog.get("available"),
                field="launch.process_catalog.available",
            ),
            stop_targets=_strings(launch.get("stop_plan"), field="launch.stop_plan"),
            contracts=_strings(checks.get("contracts"), field="checks.contracts"),
            critical_modules=_strings(
                checks.get("critical_modules"),
                field="checks.critical_modules",
            ),
            native_process_environment=_native_environment_mapping(
                launch.get("native_process_environment")
            ),
            route_contract=_optional_text(host.get("route_contract")),
            module_transport=_required_text(host, "module_transport"),
            host_config=_object(host.get("config"), field="host.config"),
            lifecycle=_object(launch.get("session"), field="launch.session"),
            parameter_profile=_optional_text(launch.get("parameter_profile")),
            parameter_overrides=_object(
                launch.get("parameter_overrides"),
                field="launch.parameter_overrides",
            ),
            compiled_against=_object(
                identity.get("compiled_against"),
                field="identity.compiled_against",
            ),
        )
        supplied = _required_text(identity, "fingerprint")
        if supplied != plan.fingerprint:
            raise ValueError(
                "RunPlan fingerprint mismatch: "
                f"expected {plan.fingerprint}, received {supplied}"
            )
        return plan

    @classmethod
    def load(cls, path: str | Path) -> RunPlan:
        """Read one RunPlan from disk."""

        plan_path = Path(path)
        payload = json.loads(plan_path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError(f"RunPlan must be a JSON object: {plan_path}")
        return cls.from_dict(payload)

    @property
    def managed_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes replaced when the active product changes."""

        return tuple(process for process in self.processes if process.lifecycle == "mode")

    @property
    def persistent_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes preserved across product changes."""

        return tuple(
            process for process in self.processes if process.lifecycle == "persistent"
        )

    def has_process(self, name: str) -> bool:
        """Return whether the plan declares a logical process."""

        return any(process.name == name for process in self.processes)

    def process(self, name: str) -> ProcessSpec:
        """Return one declared process by logical name."""

        for process in self.processes:
            if process.name == name:
                return process
        raise KeyError(name)

    def build(self) -> Any:
        """Materialize the Host graph without recompiling the Product."""

        from lingtu.assembly.profile_builder import build_host_from_run_plan

        return build_host_from_run_plan(self)

    def assert_compatible(
        self,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        """Require current code and contracts to interpret this plan identically."""

        from lingtu.run_plan_contract import assert_run_plan_compatible

        assert_run_plan_compatible(self, environment=environment)

    def equivalent(
        self,
        other: RunPlan | Mapping[str, Any] | None = None,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> bool:
        """Return whether another or the current runtime has the same contract."""

        from lingtu.run_plan_contract import run_plan_equivalent

        return run_plan_equivalent(self, other, environment=environment)

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic, nested JSON-ready plan data."""

        return _run_plan_payload(
            product=self.product,
            product_variant=self.product_variant,
            env=self.env,
            process_control=self.process_control,
            modules=self.modules,
            processes=self.processes,
            available_processes=self.available_processes,
            stop_targets=self.stop_targets,
            contracts=self.contracts,
            critical_modules=self.critical_modules,
            native_process_environment=self.native_process_environment,
            route_contract=self.route_contract,
            module_transport=self.module_transport,
            host_config=self.host_config,
            lifecycle=self.lifecycle,
            parameter_profile=self.parameter_profile,
            parameter_overrides=self.parameter_overrides,
            compiled_against=self.compiled_against,
            fingerprint=self.fingerprint,
        )

    def summary(self) -> dict[str, Any]:
        """Return the compact ProductControl-facing view of this plan."""

        lifecycle = self.lifecycle
        return {
            "kind": "run_plan",
            "product": self.product,
            "product_variant": self.product_variant,
            "env": self.env,
            "run_plan_fingerprint": self.fingerprint,
            "controller": self.process_control,
            "processes": [process.name for process in self.processes],
            "host_modules": list(self.modules),
            "contracts": list(self.contracts),
            "route_contract": self.route_contract,
            "module_transport": self.module_transport,
            "parameter_profile": self.parameter_profile,
            "slam_mode": lifecycle.get("slam_mode"),
            "native_control_mode": lifecycle.get("native_control_mode"),
            "requires_map": bool(lifecycle.get("requires_map", False)),
        }

    def write(self, path: str | Path) -> Path:
        """Atomically publish the verified execution plan for systemd and Host."""

        plan_path = Path(path)
        plan_path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = plan_path.with_name(
            f".{plan_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
        )
        try:
            temp_path.write_text(
                json.dumps(
                    self.as_dict(),
                    ensure_ascii=False,
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            os.chmod(temp_path, 0o600)
            os.replace(temp_path, plan_path)
        finally:
            temp_path.unlink(missing_ok=True)
        return plan_path


def validate_run_plan_snapshot(plan: RunPlan) -> dict[str, Any]:
    """Verify that one compiled plan survives its strict serialized boundary."""

    try:
        restored = RunPlan.from_dict(plan.as_dict())
    except (TypeError, ValueError) as exc:
        return {
            "ok": False,
            "blockers": [f"RunPlan serialization is invalid: {exc}"],
            "warnings": [],
        }
    if restored != plan:
        return {
            "ok": False,
            "blockers": ["RunPlan serialization changed the compiled plan"],
            "warnings": [],
        }
    return {"ok": True, "blockers": [], "warnings": []}


def _run_plan_body(
    **values: Any,
) -> dict[str, Any]:
    return _run_plan_payload(**values, fingerprint=None)


def _run_plan_payload(
    *,
    product: str,
    product_variant: str | None,
    env: str,
    process_control: str,
    modules: tuple[str, ...],
    processes: tuple[ProcessSpec, ...],
    available_processes: tuple[ProcessSpec, ...],
    stop_targets: tuple[str, ...],
    contracts: tuple[str, ...],
    critical_modules: tuple[str, ...],
    native_process_environment: Mapping[str, str],
    route_contract: str | None,
    module_transport: str,
    host_config: Mapping[str, Any],
    lifecycle: Mapping[str, Any],
    parameter_profile: str | None,
    parameter_overrides: Mapping[str, Any],
    compiled_against: Mapping[str, Any],
    fingerprint: str | None,
) -> dict[str, Any]:
    identity: dict[str, Any] = {
        "schema": RUN_PLAN_SCHEMA,
        "product": product,
        "product_variant": product_variant,
        "env": env,
    }
    if fingerprint is not None:
        identity["fingerprint"] = fingerprint
    identity["compiled_against"] = dict(compiled_against)
    return {
        "identity": identity,
        "launch": {
            "controller": process_control,
            "process_catalog": {
                "selected": [process.as_dict() for process in processes],
                "available": [
                    process.as_dict() for process in available_processes
                ],
            },
            "stop_plan": list(stop_targets),
            "native_process_environment": dict(native_process_environment),
            "session": dict(lifecycle),
            "parameter_profile": parameter_profile,
            "parameter_overrides": dict(parameter_overrides),
        },
        "host": {
            "config": dict(host_config),
            "expected_modules": list(modules),
            "route_contract": route_contract,
            "module_transport": module_transport,
        },
        "checks": {
            "contracts": list(contracts),
            "critical_modules": list(critical_modules),
        },
    }


def _fingerprint(payload: Mapping[str, Any]) -> str:
    canonical = json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _required_text(payload: Mapping[str, Any], field: str) -> str:
    value = _optional_text(payload.get(field))
    if value is None:
        raise ValueError(f"RunPlan is missing {field}")
    return value


def _required_value_text(value: Any, *, field: str) -> str:
    normalized = _optional_text(value)
    if normalized is None:
        raise ValueError(f"RunPlan is missing {field}")
    return normalized


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _validate_product_variant_identity(
    product_variant: str | None,
    *,
    host_config: Mapping[str, Any],
    lifecycle: Mapping[str, Any],
) -> None:
    host_declares_variant = "_product_variant" in host_config
    host_variant = _optional_text(host_config.get("_product_variant"))
    lifecycle_variant = _optional_text(lifecycle.get("product_variant"))
    if product_variant is None:
        if host_declares_variant:
            raise ValueError(
                "RunPlan Host config declares a variant for a non-variant Product"
            )
    elif not host_declares_variant or host_variant != product_variant:
        raise ValueError(
            "RunPlan Host config variant does not match identity.product_variant"
        )
    if lifecycle_variant != product_variant:
        raise ValueError(
            "RunPlan lifecycle variant does not match identity.product_variant"
        )


def _env(value: Any) -> str:
    env = _optional_text(value)
    if env not in {"real", "sim"}:
        raise ValueError(
            f"RunPlan env must be 'real' or 'sim', received {env!r}"
        )
    return env


def _strings(value: Any, *, field: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"RunPlan {field} must be a list")
    values = tuple(str(item).strip() for item in value)
    if any(not item for item in values) or len(set(values)) != len(values):
        raise ValueError(
            f"RunPlan {field} contains invalid or duplicate values"
        )
    return values


def _object(value: Any, *, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"RunPlan {field} must be an object")
    return value


def _json_mapping(value: Any, *, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"RunPlan {field} must be an object")
    try:
        encoded = json.dumps(
            {str(key): item for key, item in value.items()},
            allow_nan=False,
            ensure_ascii=True,
            sort_keys=True,
        )
        normalized = json.loads(encoded)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"RunPlan {field} must contain JSON data") from exc
    if not isinstance(normalized, dict):
        raise ValueError(f"RunPlan {field} must be an object")
    return normalized


def _canonical_mapping_json(value: Mapping[str, Any]) -> str:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )


def _mapping_from_json(value: str) -> dict[str, Any]:
    loaded = json.loads(value)
    if not isinstance(loaded, dict):  # pragma: no cover - constructor invariant
        raise RuntimeError("RunPlan mapping storage is corrupt")
    return loaded


def _processes(value: Any, *, field: str) -> tuple[ProcessSpec, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"RunPlan {field} must be a list")
    processes = tuple(_process(item, field=field) for item in value)
    return _normalize_processes(processes, field=field)


def _normalize_processes(
    value: tuple[ProcessSpec, ...] | list[ProcessSpec],
    *,
    field: str,
) -> tuple[ProcessSpec, ...]:
    if not isinstance(value, list | tuple) or any(
        not isinstance(process, ProcessSpec) for process in value
    ):
        raise ValueError(f"RunPlan {field} must contain ProcessSpec values")
    processes = tuple(value)
    names = tuple(process.name for process in processes)
    targets = tuple(process.target for process in processes)
    if len(set(names)) != len(names) or len(set(targets)) != len(targets):
        raise ValueError(f"RunPlan {field} contains duplicate processes")
    return processes


def _validate_process_catalog(
    selected: tuple[ProcessSpec, ...],
    available: tuple[ProcessSpec, ...],
) -> None:
    if not available:
        if selected:
            raise ValueError(
                "RunPlan selected processes require an available process catalog"
            )
        return
    by_name = {process.name: process for process in available}
    for process in selected:
        if by_name.get(process.name) != process:
            raise ValueError(
                "RunPlan selected process disagrees with the available catalog: "
                f"{process.name}"
            )


def _process(value: Any, *, field: str) -> ProcessSpec:
    if not isinstance(value, Mapping):
        raise ValueError(f"RunPlan {field} entry must be an object")
    _require_fields(value, _PROCESS_FIELDS, context=f"RunPlan {field} entry")
    name = _required_text(value, "name")
    manager = _required_text(value, "manager")
    target = _required_text(value, "target")
    lifecycle = _required_text(value, "lifecycle")
    order = value.get("order")
    timeout_s = value.get("timeout_s")
    if manager not in {"systemd", "direct", "external"}:
        raise ValueError(f"RunPlan process {name} has invalid manager")
    if lifecycle not in {"mode", "persistent"}:
        raise ValueError(f"RunPlan process {name} has invalid lifecycle")
    if (
        isinstance(order, bool)
        or not isinstance(order, int)
        or order < 0
        or isinstance(timeout_s, bool)
        or not isinstance(timeout_s, int)
        or timeout_s <= 0
    ):
        raise ValueError(f"RunPlan process {name} has invalid timing")
    if any(char.isspace() for char in target) or target.startswith("-"):
        raise ValueError(f"RunPlan process {name} has invalid target")
    return ProcessSpec(
        name=name,
        manager=manager,
        target=target,
        order=order,
        timeout_s=timeout_s,
        lifecycle=lifecycle,
    )


def _normalize_native_environment(
    value: Mapping[str, str] | None,
    *,
    native_nav: Mapping[str, Any] | None,
) -> dict[str, str]:
    if value is None:
        if native_nav is None:
            raise ValueError(
                "RunPlan launch.native_process_environment is required"
            )
        environment = _native_environment_from_nav(native_nav)
    else:
        environment = _native_environment_mapping(value)
    projected = _native_nav_from_environment(environment)
    if native_nav is not None and projected != _normalized_native_nav(native_nav):
        raise ValueError(
            "RunPlan native_nav disagrees with native_process_environment"
        )
    return environment


def _native_environment_mapping(value: Any) -> dict[str, str]:
    if not isinstance(value, Mapping):
        raise ValueError(
            "RunPlan launch.native_process_environment must be an object"
        )
    environment: dict[str, str] = {}
    for raw_key, raw_value in value.items():
        key = str(raw_key).strip()
        if _ENVIRONMENT_KEY.fullmatch(key) is None:
            raise ValueError(f"RunPlan has invalid environment key: {key!r}")
        if not isinstance(raw_value, str) or any(
            character in raw_value for character in ("\x00", "\n", "\r")
        ):
            raise ValueError(
                f"RunPlan environment value must be a single-line string: {key}"
            )
        environment[key] = raw_value
    _native_nav_from_environment(environment)
    return dict(sorted(environment.items()))


def _normalized_native_nav(value: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("RunPlan native_nav must be an object")
    expected = frozenset(_NATIVE_NAV_ENVIRONMENT_KEYS)
    _require_fields(value, expected, context="RunPlan native_nav")
    control_mode = _required_text(value, "control_mode")
    booleans: dict[str, bool] = {}
    for field in expected - {"control_mode"}:
        raw = value.get(field)
        if not isinstance(raw, bool):
            raise ValueError(f"RunPlan native_nav.{field} must be a boolean")
        booleans[field] = raw
    return {"control_mode": control_mode, **booleans}


def _native_environment_from_nav(value: Mapping[str, Any]) -> dict[str, str]:
    native_nav = _normalized_native_nav(value)
    environment = {
        _NATIVE_NAV_ENVIRONMENT_KEYS["control_mode"]: str(
            native_nav["control_mode"]
        )
    }
    for field, key in _NATIVE_NAV_ENVIRONMENT_KEYS.items():
        if field == "control_mode":
            continue
        environment[key] = "1" if native_nav[field] else "0"
    return dict(sorted(environment.items()))


def _native_nav_from_environment(value: Mapping[str, str]) -> dict[str, Any]:
    missing = sorted(
        key for key in _NATIVE_NAV_ENVIRONMENT_KEYS.values() if key not in value
    )
    if missing:
        raise ValueError(
            "RunPlan native process environment is missing: "
            + ", ".join(missing)
        )
    return {
        "control_mode": str(value[_NATIVE_NAV_ENVIRONMENT_KEYS["control_mode"]]),
        "publish_cmd_vel": _environment_bool(
            value[_NATIVE_NAV_ENVIRONMENT_KEYS["publish_cmd_vel"]]
        ),
        "check_obstacle": _environment_bool(
            value[_NATIVE_NAV_ENVIRONMENT_KEYS["check_obstacle"]]
        ),
        "use_traversability_cost": _environment_bool(
            value[_NATIVE_NAV_ENVIRONMENT_KEYS["use_traversability_cost"]]
        ),
        "allow_teleop_takeover": _environment_bool(
            value[_NATIVE_NAV_ENVIRONMENT_KEYS["allow_teleop_takeover"]]
        ),
        "teleop_local_planner": _environment_bool(
            value[_NATIVE_NAV_ENVIRONMENT_KEYS["teleop_local_planner"]]
        ),
    }


def _environment_bool(value: str) -> bool:
    if value == "1":
        return True
    if value == "0":
        return False
    raise ValueError(
        f"RunPlan native boolean environment value must be 0 or 1: {value!r}"
    )


def _require_fields(
    value: Mapping[str, Any],
    expected: frozenset[str],
    *,
    context: str,
) -> None:
    observed = {str(key) for key in value}
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


__all__ = ["CURRENT_RUN_SCHEMA", "RUN_PLAN_SCHEMA", "RunPlan"]
