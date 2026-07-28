"""Compiled product contract and immutable deployment manifest."""

from __future__ import annotations

import hashlib
import json
import os
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

from runtime.graph import ProcessSpec

if TYPE_CHECKING:
    from runtime.blueprint import Blueprint

PRODUCT_SCHEMA = "lingtu.product.v4"
PROCESS_CONTRACT_SCHEMA = "lingtu.product.v5"


@dataclass(frozen=True)
class ProductManifest:
    """Serializable product contract consumed by the external control plane."""

    schema_version: str
    profile: str
    endpoint: str | None
    process_control: str
    modules: tuple[str, ...]
    processes: tuple[ProcessSpec, ...]
    available_processes: tuple[ProcessSpec, ...]
    stop_targets: tuple[str, ...]
    required_topics: tuple[str, ...]
    required_capabilities: tuple[str, ...]
    critical_modules: tuple[str, ...]
    native_nav: Mapping[str, Any]
    route_contract: str | None
    module_transport: str
    host_config: Mapping[str, Any]
    fingerprint: str

    @classmethod
    def create(
        cls,
        *,
        profile: str,
        endpoint: str | None,
        process_control: str,
        modules: tuple[str, ...],
        processes: tuple[ProcessSpec, ...],
        available_processes: tuple[ProcessSpec, ...],
        stop_targets: tuple[str, ...],
        required_topics: tuple[str, ...],
        required_capabilities: tuple[str, ...],
        critical_modules: tuple[str, ...],
        native_nav: Mapping[str, Any],
        route_contract: str | None,
        module_transport: str,
        host_config: Mapping[str, Any],
    ) -> ProductManifest:
        """Create a manifest and compute its canonical fingerprint."""

        normalized_native_nav = _json_mapping(native_nav, field="native_nav")
        normalized_host_config = _json_mapping(host_config, field="host_config")
        body = _manifest_body(
            profile=profile,
            endpoint=endpoint,
            process_control=process_control,
            modules=modules,
            processes=processes,
            available_processes=available_processes,
            stop_targets=stop_targets,
            required_topics=required_topics,
            required_capabilities=required_capabilities,
            critical_modules=critical_modules,
            native_nav=normalized_native_nav,
            route_contract=route_contract,
            module_transport=module_transport,
            host_config=normalized_host_config,
        )
        return cls(
            schema_version=PRODUCT_SCHEMA,
            profile=profile,
            endpoint=endpoint,
            process_control=process_control,
            modules=tuple(modules),
            processes=tuple(processes),
            available_processes=tuple(available_processes),
            stop_targets=tuple(stop_targets),
            required_topics=tuple(required_topics),
            required_capabilities=tuple(required_capabilities),
            critical_modules=tuple(critical_modules),
            native_nav=normalized_native_nav,
            route_contract=route_contract,
            module_transport=module_transport,
            host_config=normalized_host_config,
            fingerprint=_fingerprint(body),
        )

    @classmethod
    def create_process_contract(
        cls,
        *,
        profile: str,
        endpoint: str | None,
        process_control: str,
        processes: tuple[ProcessSpec, ...],
        available_processes: tuple[ProcessSpec, ...],
        stop_targets: tuple[str, ...],
        required_topics: tuple[str, ...],
        required_capabilities: tuple[str, ...],
        native_nav: Mapping[str, Any],
        route_contract: str | None,
    ) -> ProductManifest:
        """Create a v5 process-only manifest without a Host Module graph."""

        normalized_native_nav = _json_mapping(native_nav, field="native_nav")
        body = _process_manifest_body(
            profile=profile,
            endpoint=endpoint,
            process_control=process_control,
            processes=processes,
            available_processes=available_processes,
            stop_targets=stop_targets,
            required_topics=required_topics,
            required_capabilities=required_capabilities,
            native_nav=normalized_native_nav,
            route_contract=route_contract,
        )
        return cls(
            schema_version=PROCESS_CONTRACT_SCHEMA,
            profile=profile,
            endpoint=endpoint,
            process_control=process_control,
            modules=(),
            processes=tuple(processes),
            available_processes=tuple(available_processes),
            stop_targets=tuple(stop_targets),
            required_topics=tuple(required_topics),
            required_capabilities=tuple(required_capabilities),
            critical_modules=(),
            native_nav=normalized_native_nav,
            route_contract=route_contract,
            module_transport="",
            host_config={},
            fingerprint=_fingerprint(body),
        )

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> ProductManifest:
        """Load and verify a manifest, rejecting stale or edited contracts."""

        schema_version = payload.get("schema_version")
        if schema_version == PROCESS_CONTRACT_SCHEMA:
            supplied = _required_text(payload, "fingerprint")
            public_body = dict(payload)
            public_body.pop("fingerprint", None)
            if supplied != _fingerprint(public_body):
                raise ValueError(
                    "Product manifest fingerprint mismatch: "
                    f"expected {_fingerprint(public_body)}, received {supplied}"
                )
            manifest = cls.create_process_contract(
                profile=_required_text(payload, "profile"),
                endpoint=_optional_text(payload.get("endpoint")),
                process_control=_required_text(payload, "process_control"),
                processes=_processes(payload.get("processes"), field="processes"),
                available_processes=_processes(
                    payload.get("known_processes"),
                    field="known_processes",
                ),
                stop_targets=_strings(payload.get("stop_targets"), field="stop_targets"),
                required_topics=_strings(payload.get("required_topics"), field="required_topics"),
                required_capabilities=_strings(
                    payload.get("required_capabilities"),
                    field="required_capabilities",
                ),
                native_nav=_mapping(payload.get("native_nav"), field="native_nav"),
                route_contract=_optional_text(payload.get("route_contract")),
            )
            if supplied != manifest.fingerprint:
                raise ValueError(
                    "Product manifest fingerprint mismatch: "
                    f"expected {manifest.fingerprint}, received {supplied}"
                )
            return manifest
        if schema_version != PRODUCT_SCHEMA:
            raise ValueError(f"unsupported Product manifest schema: {schema_version!r}")
        processes = _processes(payload.get("processes"), field="processes")
        available = _processes(payload.get("known_processes"), field="known_processes")
        manifest = cls.create(
            profile=_required_text(payload, "profile"),
            endpoint=_optional_text(payload.get("endpoint")),
            process_control=_required_text(payload, "process_control"),
            modules=_strings(payload.get("modules"), field="modules"),
            processes=processes,
            available_processes=available,
            stop_targets=_strings(payload.get("stop_targets"), field="stop_targets"),
            required_topics=_strings(payload.get("required_topics"), field="required_topics"),
            required_capabilities=_strings(
                payload.get("required_capabilities"),
                field="required_capabilities",
            ),
            critical_modules=_strings(payload.get("critical_modules"), field="critical_modules"),
            native_nav=_mapping(payload.get("native_nav"), field="native_nav"),
            route_contract=_optional_text(payload.get("route_contract")),
            module_transport=_required_text(payload, "module_transport"),
            host_config=_mapping(payload.get("host_config"), field="host_config"),
        )
        supplied = _required_text(payload, "fingerprint")
        if supplied != manifest.fingerprint:
            raise ValueError(
                f"Product manifest fingerprint mismatch: expected {manifest.fingerprint}, received {supplied}"
            )
        return manifest

    @classmethod
    def load(cls, path: str | Path) -> ProductManifest:
        """Read a Product manifest or a switch-plan envelope from disk."""

        manifest_path = Path(path)
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError(f"Product manifest must be a JSON object: {manifest_path}")
        switch = payload.get("product_mode_switch")
        if isinstance(switch, Mapping):
            payload = switch.get("product")
        if not isinstance(payload, Mapping):
            raise ValueError(f"Product manifest is missing from: {manifest_path}")
        return cls.from_dict(payload)

    @property
    def managed_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes replaced when the active product changes."""

        return tuple(process for process in self.processes if process.lifecycle == "mode")

    @property
    def persistent_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes preserved across product changes."""

        return tuple(process for process in self.processes if process.lifecycle == "persistent")

    def has_process(self, name: str) -> bool:
        """Return whether the manifest declares a logical process."""

        return any(process.name == name for process in self.processes)

    def process(self, name: str) -> ProcessSpec:
        """Return one declared process by logical name."""

        for process in self.processes:
            if process.name == name:
                return process
        raise KeyError(name)

    def build(self) -> Any:
        """Materialize the Host graph without recompiling the Product."""

        if self.schema_version == PROCESS_CONTRACT_SCHEMA:
            raise RuntimeError("v5 process-only Product manifests do not contain a Host Module graph")
        from lingtu.assembly.profile_builder import build_host_from_manifest

        return build_host_from_manifest(self)

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready manifest data."""

        if self.schema_version == PROCESS_CONTRACT_SCHEMA:
            body = _process_manifest_body(
                profile=self.profile,
                endpoint=self.endpoint,
                process_control=self.process_control,
                processes=self.processes,
                available_processes=self.available_processes,
                stop_targets=self.stop_targets,
                required_topics=self.required_topics,
                required_capabilities=self.required_capabilities,
                native_nav=self.native_nav,
                route_contract=self.route_contract,
            )
        else:
            body = _manifest_body(
                profile=self.profile,
                endpoint=self.endpoint,
                process_control=self.process_control,
                modules=self.modules,
                processes=self.processes,
                available_processes=self.available_processes,
                stop_targets=self.stop_targets,
                required_topics=self.required_topics,
                required_capabilities=self.required_capabilities,
                critical_modules=self.critical_modules,
                native_nav=self.native_nav,
                route_contract=self.route_contract,
                module_transport=self.module_transport,
                host_config=self.host_config,
            )
        body["fingerprint"] = self.fingerprint
        return body

    def write(self, path: str | Path) -> Path:
        """Atomically publish a read-only manifest for Launcher and the Host."""

        manifest_path = Path(path)
        manifest_path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = manifest_path.with_name(f".{manifest_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
        try:
            temp_path.write_text(
                json.dumps(self.as_dict(), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.chmod(temp_path, 0o600)
            os.replace(temp_path, manifest_path)
        finally:
            temp_path.unlink(missing_ok=True)
        return manifest_path


@dataclass(frozen=True)
class Product:
    """Side-effect-free Host graph and deployment process definition."""

    profile: str
    endpoint: str | None
    process_control: str
    config: Mapping[str, Any]
    blueprint: Blueprint | None
    processes: tuple[ProcessSpec, ...]
    available_processes: tuple[ProcessSpec, ...]
    process_conflicts: tuple[str, ...]
    required_topics: tuple[str, ...]
    required_capabilities: tuple[str, ...]
    critical_modules: tuple[str, ...]
    native_nav: Mapping[str, Any]
    route_contract: str | None
    module_transport: str
    manifest_schema: str = PRODUCT_SCHEMA

    @property
    def modules(self) -> tuple[str, ...]:
        """Return stable Host Module aliases without constructing them."""

        if self.blueprint is None:
            raise RuntimeError("process-only Products do not contain a Host Module graph")
        return self.blueprint.module_names

    @property
    def managed_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes replaced when the active product changes."""

        return tuple(process for process in self.processes if process.lifecycle == "mode")

    @property
    def persistent_processes(self) -> tuple[ProcessSpec, ...]:
        """Return processes preserved across product changes."""

        return tuple(process for process in self.processes if process.lifecycle == "persistent")

    @property
    def stop_targets(self) -> tuple[str, ...]:
        """Return endpoint targets stopped before this product starts."""

        managed = tuple(process.target for process in reversed(self.available_processes) if process.lifecycle == "mode")
        return tuple(dict.fromkeys((*managed, *self.process_conflicts)))

    @property
    def fingerprint(self) -> str:
        """Return the canonical Product contract fingerprint."""

        return self.manifest().fingerprint

    def has_process(self, name: str) -> bool:
        """Return whether this Product declares a logical process."""

        return any(process.name == name for process in self.processes)

    def process(self, name: str) -> ProcessSpec:
        """Return one declared process by logical name."""

        for process in self.processes:
            if process.name == name:
                return process
        raise KeyError(name)

    def build(self) -> Any:
        """Materialize only the Blueprint-owned Host Module graph."""

        if self.blueprint is None:
            raise RuntimeError("process-only Products do not contain a Host Module graph")
        from lingtu.assembly.profile_builder import module_transport_for_resolved_config

        transport = module_transport_for_resolved_config(self.config)
        if transport is None:
            return self.blueprint.build()
        return self.blueprint.build(transport=transport)

    def manifest(self) -> ProductManifest:
        """Freeze this compiled Product as a deployment manifest."""

        if self.manifest_schema == PROCESS_CONTRACT_SCHEMA:
            return ProductManifest.create_process_contract(
                profile=self.profile,
                endpoint=self.endpoint,
                process_control=self.process_control,
                processes=self.processes,
                available_processes=self.available_processes,
                stop_targets=self.stop_targets,
                required_topics=self.required_topics,
                required_capabilities=self.required_capabilities,
                native_nav=self.native_nav,
                route_contract=self.route_contract,
            )
        return ProductManifest.create(
            profile=self.profile,
            endpoint=self.endpoint,
            process_control=self.process_control,
            modules=self.modules,
            processes=self.processes,
            available_processes=self.available_processes,
            stop_targets=self.stop_targets,
            required_topics=self.required_topics,
            required_capabilities=self.required_capabilities,
            critical_modules=self.critical_modules,
            native_nav=self.native_nav,
            route_contract=self.route_contract,
            module_transport=self.module_transport,
            host_config=self.config,
        )

    def as_dict(self) -> dict[str, Any]:
        """Return the stable public Product contract."""

        return self.manifest().as_dict()


def _manifest_body(
    *,
    profile: str,
    endpoint: str | None,
    process_control: str,
    modules: tuple[str, ...],
    processes: tuple[ProcessSpec, ...],
    available_processes: tuple[ProcessSpec, ...],
    stop_targets: tuple[str, ...],
    required_topics: tuple[str, ...],
    required_capabilities: tuple[str, ...],
    critical_modules: tuple[str, ...],
    native_nav: Mapping[str, Any],
    route_contract: str | None,
    module_transport: str,
    host_config: Mapping[str, Any],
) -> dict[str, Any]:
    return {
        "schema_version": PRODUCT_SCHEMA,
        "profile": profile,
        "endpoint": endpoint,
        "process_control": process_control,
        "modules": list(modules),
        "processes": [process.as_dict() for process in processes],
        "known_processes": [process.as_dict() for process in available_processes],
        "known_targets": [process.target for process in available_processes],
        "stop_targets": list(stop_targets),
        "required_topics": list(required_topics),
        "required_capabilities": list(required_capabilities),
        "critical_modules": list(critical_modules),
        "native_nav": dict(native_nav),
        "route_contract": route_contract,
        "module_transport": module_transport,
        "host_config": dict(host_config),
    }


def _process_manifest_body(
    *,
    profile: str,
    endpoint: str | None,
    process_control: str,
    processes: tuple[ProcessSpec, ...],
    available_processes: tuple[ProcessSpec, ...],
    stop_targets: tuple[str, ...],
    required_topics: tuple[str, ...],
    required_capabilities: tuple[str, ...],
    native_nav: Mapping[str, Any],
    route_contract: str | None,
) -> dict[str, Any]:
    return {
        "schema_version": PROCESS_CONTRACT_SCHEMA,
        "profile": profile,
        "endpoint": endpoint,
        "process_control": process_control,
        "processes": [process.as_dict() for process in processes],
        "known_processes": [process.as_dict() for process in available_processes],
        "known_targets": [process.target for process in available_processes],
        "stop_targets": list(stop_targets),
        "required_topics": list(required_topics),
        "required_capabilities": list(required_capabilities),
        "native_nav": dict(native_nav),
        "route_contract": route_contract,
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
        raise ValueError(f"Product manifest is missing {field}")
    return value


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _strings(value: Any, *, field: str) -> tuple[str, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"Product manifest {field} must be a list")
    values = tuple(str(item).strip() for item in value)
    if any(not item for item in values) or len(set(values)) != len(values):
        raise ValueError(f"Product manifest {field} contains invalid or duplicate values")
    return values


def _mapping(value: Any, *, field: str) -> Mapping[str, Any]:
    return _json_mapping(value, field=field)


def _json_mapping(value: Any, *, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"Product manifest {field} must be an object")
    try:
        encoded = json.dumps(
            {str(key): item for key, item in value.items()},
            allow_nan=False,
            ensure_ascii=True,
            sort_keys=True,
        )
        normalized = json.loads(encoded)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Product manifest {field} must contain JSON data") from exc
    if not isinstance(normalized, dict):
        raise ValueError(f"Product manifest {field} must be an object")
    return normalized


def _processes(value: Any, *, field: str) -> tuple[ProcessSpec, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"Product manifest {field} must be a list")
    processes = tuple(_process(item, field=field) for item in value)
    names = tuple(process.name for process in processes)
    targets = tuple(process.target for process in processes)
    if len(set(names)) != len(names) or len(set(targets)) != len(targets):
        raise ValueError(f"Product manifest {field} contains duplicate processes")
    return processes


def _process(value: Any, *, field: str) -> ProcessSpec:
    if not isinstance(value, Mapping):
        raise ValueError(f"Product manifest {field} entry must be an object")
    name = _required_text(value, "name")
    manager = _required_text(value, "manager")
    target = _required_text(value, "target")
    lifecycle = _required_text(value, "lifecycle")
    application = _optional_text(value.get("application"))
    config = (
        _mapping(value.get("config"), field=f"{field}.config")
        if "config" in value
        else {}
    )
    order = value.get("order")
    timeout_s = value.get("timeout_s")
    if manager not in {"systemd", "direct", "external"}:
        raise ValueError(f"Product manifest process {name} has invalid manager")
    if lifecycle not in {"mode", "persistent"}:
        raise ValueError(f"Product manifest process {name} has invalid lifecycle")
    if (
        isinstance(order, bool)
        or not isinstance(order, int)
        or order < 0
        or isinstance(timeout_s, bool)
        or not isinstance(timeout_s, int)
        or timeout_s <= 0
    ):
        raise ValueError(f"Product manifest process {name} has invalid timing")
    if any(char.isspace() for char in target) or target.startswith("-"):
        raise ValueError(f"Product manifest process {name} has invalid target")
    return ProcessSpec(
        name=name,
        manager=manager,
        target=target,
        order=order,
        timeout_s=timeout_s,
        lifecycle=lifecycle,
        application=application,
        config=config,
    )


__all__ = ["PROCESS_CONTRACT_SCHEMA", "PRODUCT_SCHEMA", "Product", "ProductManifest"]
