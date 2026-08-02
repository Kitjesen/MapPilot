"""Deterministic interpretation contract for compiled RunPlans."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import re
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from runtime.contracts.product_runtime import (
    contract_catalog_snapshot,
    resolve_product_spec_contracts,
)
from runtime.graph import (
    ProcessSpec,
    RuntimeGraph,
    load_runtime_graph,
    resolve_env_implementation,
)
from runtime.graph.loader import resolve_product_variant_spec
from runtime.service_catalogs.thunder import thunder_service_spec

if TYPE_CHECKING:
    from lingtu.run_plan import RunPlan
    from runtime.blueprint import Blueprint


COMPILED_AGAINST_SCHEMA = "lingtu.run_plan.compiled_against.v2"
_REPO_ROOT = Path(__file__).resolve().parents[2]
_RELEASE_CONTRACT = _REPO_ROOT / "config" / "release-runtime.env"
_RELEASE_NATIVE_SHA256 = _REPO_ROOT / "config" / "release-native-sha256.txt"
_VERSION_FILE = _REPO_ROOT / "VERSION"
_COMPATIBILITY_INSTRUCTION = (
    "re-run ProductControl.switch to compile and publish a current RunPlan"
)


class RunPlanCompatibilityError(RuntimeError):
    """Raised when the current runtime would reinterpret a persisted plan."""


def lifecycle_snapshot(
    product: str,
    graph: RuntimeGraph,
    *,
    product_variant: str | None = None,
) -> dict[str, Any]:
    """Return the complete ProductLifecycle data embedded in a plan."""

    product_spec = graph.products.get(product)
    if not isinstance(product_spec, Mapping):
        raise ValueError(f"Product {product!r} is not declared in the Runtime Graph")
    product_spec = resolve_product_variant_spec(
        product,
        product_spec,
        product_variant=product_variant,
    )
    if product_spec.get("operator_switchable") is not True:
        raise ValueError(f"Product is not operator-switchable: {product}")
    return {
        "product": product,
        "product_variant": product_spec.get("product_variant"),
        "label": str(product_spec.get("label") or product.replace("_", " ").title()),
        "product_mode": _required_product_text(product_spec, product, "product_mode"),
        "product_session": _required_product_text(product_spec, product, "product_session"),
        "session_mode": _required_product_text(product_spec, product, "session_mode"),
        "native_control_mode": _required_product_text(
            product_spec,
            product,
            "native_control_mode",
        ),
        "slam_mode": _required_product_text(product_spec, product, "slam_mode"),
        "requires_map": bool(product_spec.get("requires_map", False)),
        "switch_policy": _required_product_text(product_spec, product, "switch_policy"),
        "default_for_session_mode": bool(
            product_spec.get("default_for_session_mode", False)
        ),
        "hot_switch_candidates": sorted(
            _string_sequence(product_spec.get("hot_switch_candidates"))
        ),
        "online_hot_switch_supported": bool(
            product_spec.get("online_hot_switch_supported", False)
        ),
    }


def compiled_against(
    *,
    product: str,
    product_variant: str | None = None,
    env: str,
    blueprint: Blueprint,
    processes: Sequence[ProcessSpec],
    graph: RuntimeGraph,
    env_config: Mapping[str, Any] | None = None,
    environment: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Build the deterministic identity of code and contracts interpreting a plan."""

    _assert_blueprint_json(blueprint)
    module_graph = blueprint.export_graph(profile=product).to_manifest()
    host_graph_contract = {
        "module_graph": module_graph,
        "required_modules": list(blueprint.required_module_names),
    }
    product_spec = graph.products.get(product)
    if not isinstance(product_spec, Mapping):
        raise ValueError(f"Product {product!r} is not declared in the Runtime Graph")
    product_spec = resolve_product_variant_spec(
        product,
        product_spec,
        product_variant=product_variant,
    )
    runtime_contracts = resolve_product_spec_contracts(product, product_spec)
    return {
        "schema_version": COMPILED_AGAINST_SCHEMA,
        "runtime_graph_sha256": _digest(
            _runtime_graph_contract(
                product,
                product_variant,
                env,
                graph,
                env_config=env_config,
                route_contract_name=str(
                    module_graph.get("route_contract") or ""
                ).strip()
                or None,
            )
        ),
        "service_catalog_sha256": _digest(
            _service_catalog_contract(processes)
        ),
        "product_contract_catalog_sha256": _digest(
            contract_catalog_snapshot(runtime_contracts.contract_ids)
        ),
        "host_module_graph_sha256": _digest(host_graph_contract),
        "host_source_sha256": _host_source_digest(module_graph),
        "release": _release_identity(
            environment,
            require_packaged_native=env == "real",
        ),
    }


def recompute_run_plan_contract(
    plan: RunPlan,
    *,
    environment: Mapping[str, str] | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Rebuild the current interpretation contract for an embedded plan."""

    from lingtu.assembly.profile_builder import blueprint_for_resolved_product

    graph = load_runtime_graph()
    product_spec = graph.products.get(plan.product)
    if not isinstance(product_spec, Mapping):
        raise RunPlanCompatibilityError(
            f"RunPlan Product {plan.product!r} is no longer declared; "
            f"{_COMPATIBILITY_INSTRUCTION}"
        )
    current_contracts = resolve_product_spec_contracts(
        plan.product,
        product_spec,
        product_variant=plan.product_variant,
    )
    if current_contracts.contract_ids != plan.contracts:
        raise RunPlanCompatibilityError(
            "RunPlan named Product contracts have changed; "
            f"{_COMPATIBILITY_INSTRUCTION}"
        )
    env_config = _plan_env_config(plan)
    assembly_config = dict(plan.host_config)
    assembly_config["_product_required_topics"] = plan.required_topics
    assembly_config["_product_required_capabilities"] = plan.required_capabilities
    # Do not inject _run_plan or _run_plan_fingerprint here. They are
    # runtime values and would make the Host graph digest depend on itself.
    blueprint = blueprint_for_resolved_product(plan.product, assembly_config)
    if plan.route_contract:
        blueprint.route_contract(plan.route_contract)
    if plan.critical_modules:
        blueprint.require_modules(*plan.critical_modules)
    return (
        lifecycle_snapshot(
            plan.product,
            graph,
            product_variant=plan.product_variant,
        ),
        compiled_against(
            product=plan.product,
            product_variant=plan.product_variant,
            env=plan.env,
            blueprint=blueprint,
            processes=plan.processes,
            graph=graph,
            env_config=env_config,
            environment=environment,
        ),
    )


def assert_run_plan_compatible(
    plan: RunPlan,
    *,
    environment: Mapping[str, str] | None = None,
) -> None:
    """Fail closed if current code or contracts would reinterpret *plan*."""

    from lingtu.run_plan import RUN_PLAN_SCHEMA, RunPlan

    if plan.schema_version != RUN_PLAN_SCHEMA:
        raise RunPlanCompatibilityError(
            f"unsupported RunPlan schema {plan.schema_version!r}; "
            f"{_COMPATIBILITY_INSTRUCTION}"
        )
    try:
        RunPlan.from_dict(plan.as_dict())
        lifecycle, current = recompute_run_plan_contract(
            plan,
            environment=environment,
        )
    except RunPlanCompatibilityError:
        raise
    except Exception as exc:
        raise RunPlanCompatibilityError(
            "RunPlan compatibility could not be established; "
            f"{_COMPATIBILITY_INSTRUCTION}: {exc}"
        ) from exc

    changed: list[str] = []
    if not _equivalent_mapping(plan.lifecycle, lifecycle):
        changed.append("lifecycle")
    for key in sorted(set(plan.compiled_against) | set(current)):
        if plan.compiled_against.get(key) != current.get(key):
            changed.append(f"compiled_against.{key}")
    if changed:
        raise RunPlanCompatibilityError(
            "RunPlan is incompatible with the current runtime contracts "
            f"({', '.join(changed)}); {_COMPATIBILITY_INSTRUCTION}"
        )


def run_plan_equivalent(
    plan: RunPlan,
    other: RunPlan | Mapping[str, Any] | None = None,
    *,
    environment: Mapping[str, str] | None = None,
) -> bool:
    """Return whether two plans, or a plan and current contracts, agree."""

    if other is None:
        try:
            assert_run_plan_compatible(plan, environment=environment)
        except RunPlanCompatibilityError:
            return False
        return True
    if isinstance(other, Mapping):
        identity = other.get("identity")
        launch = other.get("launch")
        lifecycle = launch.get("session") if isinstance(launch, Mapping) else None
        compiled = (
            identity.get("compiled_against")
            if isinstance(identity, Mapping)
            else None
        )
    else:
        lifecycle = other.lifecycle
        compiled = other.compiled_against
    return (
        isinstance(lifecycle, Mapping)
        and isinstance(compiled, Mapping)
        and _equivalent_mapping(plan.lifecycle, lifecycle)
        and _equivalent_mapping(plan.compiled_against, compiled)
    )


def lifecycle_transition_plan(
    current: Mapping[str, Any] | None,
    target: Mapping[str, Any],
) -> dict[str, Any]:
    """Describe a transition using only fingerprinted lifecycle snapshots."""

    current_product = str((current or {}).get("product") or "").strip()
    target_product = str(target.get("product") or "").strip()
    candidates = _string_sequence((current or {}).get("hot_switch_candidates"))
    same_graph_candidate = bool(
        current_product and target_product in candidates
    )
    online_supported = bool(
        same_graph_candidate
        and (current or {}).get("online_hot_switch_supported") is True
        and target.get("online_hot_switch_supported") is True
    )
    required_lifecycle = (
        "hot_switch"
        if online_supported
        else str(target.get("switch_policy") or "").strip()
    )
    if not target_product or not required_lifecycle:
        raise ValueError("RunPlan lifecycle snapshot is incomplete")
    return {
        "current": (
            dict(current)
            if current is not None
            else {"product": None, "operator_switchable": False}
        ),
        "target": dict(target),
        "same_graph_candidate": same_graph_candidate,
        "online_hot_switch_supported": online_supported,
        "required_lifecycle": required_lifecycle,
        "reason": (
            "same graph hot switch is supported"
            if online_supported
            else f"target Product requires {required_lifecycle}"
        ),
    }


def _runtime_graph_contract(
    product: str,
    product_variant: str | None,
    env: str,
    graph: RuntimeGraph,
    *,
    env_config: Mapping[str, Any] | None,
    route_contract_name: str | None,
) -> dict[str, Any]:
    product_spec = graph.products.get(product)
    env_spec = graph.envs.get(env)
    if not isinstance(product_spec, Mapping) or not isinstance(env_spec, Mapping):
        raise ValueError(f"Runtime Graph no longer declares {product!r} in {env!r}")
    product_spec = resolve_product_variant_spec(
        product,
        product_spec,
        product_variant=product_variant,
    )
    implementation = resolve_env_implementation(
        env,
        graph=graph,
        env_config=env_config,
    )
    env_contract: dict[str, Any] = {
        "schema_version": env_spec.get("schema_version"),
        "name": env,
        "backend": str((env_config or {}).get("backend") or "") or None,
        "implementation": _without_loader_metadata(implementation),
    }
    route_contract = None
    if route_contract_name:
        from runtime.route_contract import load_route_contract

        route_contract = load_route_contract(route_contract_name).to_manifest()
    return {
        "product": _without_loader_metadata(product_spec),
        "env": env_contract,
        "topics": _without_loader_metadata(graph.topics),
        "route_contract": _without_loader_metadata(route_contract),
    }


def _service_catalog_contract(processes: Sequence[ProcessSpec]) -> dict[str, Any]:
    services: list[dict[str, Any]] = []
    for process in sorted(processes, key=lambda item: (item.order, item.name)):
        spec = thunder_service_spec(process.name)
        catalog = None
        if spec is not None:
            catalog = {
                "name": spec.name,
                "units": list(spec.units),
                "start_units": list(spec.start_units),
                **spec.metadata(),
            }
        services.append({"process": process.as_dict(), "catalog": catalog})
    return {"services": services}


def _release_identity(
    environment: Mapping[str, str] | None,
    *,
    require_packaged_native: bool,
) -> dict[str, Any]:
    values = environment if environment is not None else os.environ
    explicit_version = str(values.get("LINGTU_RELEASE_VERSION") or "").strip()
    packaged_version = _env_file_value(_RELEASE_CONTRACT, "LINGTU_RELEASE_VERSION")
    version = explicit_version
    if not version:
        version = packaged_version
    if not version and _VERSION_FILE.is_file():
        version = _VERSION_FILE.read_text(encoding="utf-8").strip()
    if not version:
        raise ValueError(
            "release identity requires LINGTU_RELEASE_VERSION, "
            "config/release-runtime.env, or VERSION"
        )
    native_kind: str
    if _RELEASE_NATIVE_SHA256.is_file():
        native_sha256 = _verified_native_release_digest(_RELEASE_NATIVE_SHA256)
        native_kind = "release_manifest"
    else:
        supplied_native = str(values.get("LINGTU_NATIVE_RELEASE_SHA256") or "").strip().lower()
        if supplied_native:
            if re.fullmatch(r"[0-9a-f]{64}", supplied_native) is None:
                raise ValueError("LINGTU_NATIVE_RELEASE_SHA256 must be 64 hexadecimal characters")
            native_sha256 = supplied_native
            native_kind = "environment"
        elif require_packaged_native and (explicit_version or packaged_version):
            raise ValueError(
                "real release identity requires config/release-native-sha256.txt "
                "or LINGTU_NATIVE_RELEASE_SHA256"
            )
        else:
            native_sha256 = _native_source_digest()
            native_kind = "source_tree"
    return {
        "version": version,
        "native_kind": native_kind,
        "native_sha256": native_sha256,
    }


def _host_source_digest(module_graph: Mapping[str, Any]) -> str:
    required_paths: set[Path] = {
        Path(__file__).resolve(),
        (_REPO_ROOT / "cli" / "main.py").resolve(),
        (_REPO_ROOT / "src" / "lingtu" / "control.py").resolve(),
        (_REPO_ROOT / "src" / "lingtu" / "product_lock.py").resolve(),
        (_REPO_ROOT / "src" / "lingtu" / "product_switch.py").resolve(),
        (_REPO_ROOT / "src" / "lingtu" / "run_plan.py").resolve(),
        (_REPO_ROOT / "src" / "lingtu" / "runtime_parameters.py").resolve(),
        (_REPO_ROOT / "src" / "lingtu" / "systemd.py").resolve(),
        (_REPO_ROOT / "src" / "runtime" / "blueprint.py").resolve(),
        (
            _REPO_ROOT
            / "src"
            / "runtime"
            / "contracts"
            / "product_runtime.py"
        ).resolve(),
        (_REPO_ROOT / "src" / "runtime" / "service_manager.py").resolve(),
        (
            _REPO_ROOT
            / "src"
            / "runtime"
            / "service_catalogs"
            / "thunder.py"
        ).resolve(),
        (_REPO_ROOT / "src" / "runtime" / "wiring.py").resolve(),
        (
            _REPO_ROOT
            / "src"
            / "runtime"
            / "introspection"
            / "module_graph.py"
        ).resolve(),
        (
            _REPO_ROOT
            / "src"
            / "runtime"
            / "profiles"
            / "native_nav_config.py"
        ).resolve(),
        (
            _REPO_ROOT
            / "src"
            / "runtime"
            / "profiles"
            / "product_lifecycle.py"
        ).resolve(),
    }
    missing = sorted(
        path.relative_to(_REPO_ROOT).as_posix()
        for path in required_paths
        if not path.is_file()
    )
    if missing:
        raise RuntimeError(
            "RunPlan source contract references missing files: "
            + ", ".join(missing)
        )
    paths = set(required_paths)
    assembly_root = _REPO_ROOT / "src" / "lingtu" / "assembly"
    paths.update(path.resolve() for path in assembly_root.rglob("*.py"))
    runtime_graph_root = _REPO_ROOT / "src" / "runtime" / "graph"
    paths.update(path.resolve() for path in runtime_graph_root.rglob("*.py"))
    route_root = _REPO_ROOT / "src" / "runtime" / "route_contract"
    paths.update(path.resolve() for path in route_root.rglob("*.py"))
    modules = module_graph.get("modules")
    if isinstance(modules, list):
        for module in modules:
            if not isinstance(module, Mapping):
                continue
            source = _module_source_path(str(module.get("module_path") or ""))
            if source is not None:
                paths.add(source)
    files: dict[str, str] = {}
    for path in sorted(paths, key=lambda item: item.as_posix()):
        if not path.is_file():
            continue
        try:
            relative = path.relative_to(_REPO_ROOT).as_posix()
        except ValueError:
            continue
        files[relative] = _file_sha256(path)
    return _digest({"files": files})


def _module_source_path(module_path: str) -> Path | None:
    if not module_path:
        return None
    loaded = sys.modules.get(module_path)
    candidate = getattr(loaded, "__file__", None)
    if not candidate:
        try:
            spec = importlib.util.find_spec(module_path)
        except (ImportError, ModuleNotFoundError, ValueError):
            return None
        candidate = getattr(spec, "origin", None)
    if not candidate or candidate in {"built-in", "frozen"}:
        return None
    path = Path(candidate).resolve()
    return path if path.is_file() else None


def _plan_env_config(plan: RunPlan) -> Mapping[str, Any] | None:
    if plan.env != "sim":
        return None
    backend = str(plan.host_config.get("_env_backend") or "").strip()
    if not backend:
        raise RunPlanCompatibilityError(
            "sim RunPlan does not preserve its backend configuration; "
            f"{_COMPATIBILITY_INSTRUCTION}"
        )
    return {"backend": backend}


def _assert_blueprint_json(blueprint: Blueprint) -> None:
    values: list[Any] = [
        dict(getattr(blueprint, "_global_cfg", {})),
        getattr(blueprint, "_swap_config", None),
    ]
    values.extend(
        dict(getattr(entry, "config", {}))
        for entry in getattr(blueprint, "_entries", ())
    )
    try:
        for value in values:
            json.dumps(
                value,
                allow_nan=False,
                ensure_ascii=True,
                sort_keys=True,
            )
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "Host ModuleGraph configuration must be deterministic JSON data"
        ) from exc


def _required_product_text(
    product_spec: Mapping[str, Any],
    product: str,
    field: str,
) -> str:
    value = str(product_spec.get(field) or "").strip()
    if not value:
        raise ValueError(f"Product {product!r} is missing lifecycle field {field}")
    return value


def _string_sequence(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,)
    if isinstance(value, list | tuple | set | frozenset):
        return tuple(str(item) for item in value)
    raise TypeError("Product lifecycle hot_switch_candidates must be a sequence")


def _without_loader_metadata(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {
            str(key): _without_loader_metadata(item)
            for key, item in value.items()
            if key != "_path"
        }
    if isinstance(value, tuple):
        return [_without_loader_metadata(item) for item in value]
    if isinstance(value, list):
        return [_without_loader_metadata(item) for item in value]
    return value


def _env_file_value(path: Path, name: str) -> str:
    if not path.is_file():
        return ""
    prefix = f"{name}="
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line.startswith(prefix):
            continue
        return line[len(prefix) :].strip().strip("\"'")
    return ""


def _verified_native_release_digest(path: Path) -> str:
    """Verify every packaged native artifact and hash its canonical identity."""

    files: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line:
            continue
        match = re.fullmatch(r"([0-9A-Fa-f]{64})\s+\*?(.+)", line)
        if match is None:
            raise ValueError(
                f"invalid native release hash entry at {path}:{line_number}"
            )
        expected = match.group(1).lower()
        relative = match.group(2).strip()
        candidate = (_REPO_ROOT / relative).resolve()
        try:
            normalized = candidate.relative_to(_REPO_ROOT).as_posix()
        except ValueError as exc:
            raise ValueError(
                f"native release hash path escapes the release root: {relative}"
            ) from exc
        if not candidate.is_file():
            raise ValueError(f"native release artifact is missing: {normalized}")
        observed = _file_sha256(candidate)
        if observed != expected:
            raise ValueError(f"native release artifact hash mismatch: {normalized}")
        if normalized in files:
            raise ValueError(f"duplicate native release artifact: {normalized}")
        files[normalized] = observed
    if not files:
        raise ValueError("native release hash plan is empty")
    return _digest({"files": files})


def _native_source_digest() -> str:
    """Bind local/dev plans to the native source used in this checkout."""

    suffixes = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".idl"}
    files: dict[str, str] = {}
    source_root = _REPO_ROOT / "src"
    for path in source_root.rglob("*"):
        if not path.is_file():
            continue
        if path.suffix.lower() not in suffixes and path.name != "CMakeLists.txt":
            continue
        relative = path.relative_to(_REPO_ROOT).as_posix()
        files[relative] = _file_sha256(path)
    if not files:
        raise ValueError("native source identity is empty")
    return _digest({"files": files})


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _digest(value: Any) -> str:
    canonical = json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _equivalent_mapping(left: Mapping[str, Any], right: Mapping[str, Any]) -> bool:
    return _digest(left) == _digest(right)


__all__ = [
    "COMPILED_AGAINST_SCHEMA",
    "RunPlanCompatibilityError",
    "assert_run_plan_compatible",
    "compiled_against",
    "lifecycle_snapshot",
    "lifecycle_transition_plan",
    "recompute_run_plan_contract",
    "run_plan_equivalent",
]
