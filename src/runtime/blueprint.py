"""runtime.blueprint -Declarative module orchestration.

Blueprint assembles a system from Module classes and explicit or auto-matched
wiring rules.  build() instantiates everything and returns a SystemHandle.

Typical usage::

    system = (
        Blueprint()
        .add(PerceptionModule, camera_id=0)
        .add(PlannerModule)
        .auto_wire()
        .build()
    )
    system.start()
    system.stop()

autoconnect() merges multiple blueprints and enables auto_wire in one call::

    system = autoconnect(
        driver("thunder"),
        slam("localizer"),
        perception("bpu"),
        navigation("astar"),
    ).build()
"""

from __future__ import annotations

import logging
import warnings
from collections import defaultdict
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, is_typeddict

from .module import Module
from .stream import In, Out
from .transport.local import LocalTransport, Transport
from .wiring import (
    WireSpec as _WireSpec,
)
from .wiring import (
    default_wire_topic,
    explicit_wire_topic,
    resolve_wire_delivery,
    wire_delivery_cache_key,
    wire_delivery_name,
)

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from runtime.introspection.module_graph import ModuleGraph

ConnectionKey = tuple[str, str, str, str]
ConnectionMetadata = dict[ConnectionKey, dict[str, Any]]


# ---------------------------------------------------------------------------
# _ModuleEntry
# ---------------------------------------------------------------------------


@dataclass
class _ModuleEntry:
    """One module slot in a Blueprint."""

    module_cls: type[Module]
    config: dict[str, Any]
    alias: str | None = None
    instance: Module | None = None  # pre-instantiated (e.g. NativeModule)

    @property
    def name(self) -> str:
        if self.alias:
            return self.alias
        if self.instance is not None:
            runtime_id = getattr(self.instance, "runtime_id", None)
            if runtime_id:
                return str(runtime_id)
            runtime_id = getattr(type(self.instance), "runtime_id", None)
            if runtime_id:
                return str(runtime_id)
            return type(self.instance).__name__
        runtime_id = getattr(self.module_cls, "runtime_id", None)
        if runtime_id:
            return str(runtime_id)
        return self.module_cls.__name__


# ---------------------------------------------------------------------------
# Blueprint
# ---------------------------------------------------------------------------


class Blueprint:
    """Declarative module orchestration builder.

    All configuration methods return *self* to support method chaining.
    A Blueprint should not be modified after build() is called.
    """

    def __init__(self, name: str | None = None) -> None:
        self._name = str(name).strip() if name else None
        self._entries: list[_ModuleEntry] = []
        self._wires: list[_WireSpec] = []
        self._auto_wired: bool = False
        self._global_cfg: dict[str, Any] = {}
        self._swap_config: dict[str, Any] | None = None  # set by product blueprints
        self._route_name: str | None = None
        self._route_spec: Any | None = None
        self._routed_delivery_enabled: bool = False
        # Per-module worker deployment descriptors (populated via worker()).
        from .worker_config import WorkerDeploymentRegistry

        self._worker_deployments: WorkerDeploymentRegistry = WorkerDeploymentRegistry()

    # -- registration -------------------------------------------------------

    def add(
        self,
        module_or_cls,
        alias: str | None = None,
        **config: Any,
    ) -> Blueprint:
        """Register a Module class or a pre-instantiated Module.

        Args:
            module_or_cls: Module subclass (instantiated during build()) or a
                Module instance (used as-is, e.g. NativeModule).
            alias: Optional name override for multi-instance scenarios.
            **config: Forwarded to Module.__init__ when a class is given.

        Returns:
            self
        """
        if isinstance(module_or_cls, Module):
            entry = _ModuleEntry(
                module_cls=type(module_or_cls),
                config={},
                alias=alias,
                instance=module_or_cls,
            )
        else:
            entry = _ModuleEntry(module_cls=module_or_cls, config=config, alias=alias)

        existing = {e.name for e in self._entries}
        if entry.name in existing:
            raise ValueError(
                f"Blueprint already contains '{entry.name}'. "
                "Use alias= to distinguish multiple instances of the same class."
            )
        self._entries.append(entry)
        return self

    # -- wiring -------------------------------------------------------------

    def wire(
        self,
        out_module: str,
        out_port: str,
        in_module: str,
        in_port: str,
        transport: Any = None,
        delivery: Any = None,
        topic: str | None = None,
    ) -> Blueprint:
        """Connect an Out port to an In port.

        Args:
            out_module: Source module name (class name or alias).
            out_port:   Source port name.
            in_module:  Destination module name.
            in_port:    Destination port name.
            transport:  Backward-compatible alias for delivery.
            delivery:   Per-wire delivery mode (None/callback, local, dds, shm,
                        or a Transport instance).
            topic:      Optional stable transport topic contract.

        Returns:
            self
        """
        self._wires.append(
            _WireSpec(
                out_module,
                out_port,
                in_module,
                in_port,
                transport=transport,
                topic=topic,
                delivery=delivery,
            )
        )
        return self

    def worker(
        self,
        module_name: str,
        *,
        host: str = "localhost",
        transport: str | None = None,
        domain_id: int | None = None,
        qos_profile: str | None = None,
    ) -> Blueprint:
        """Configure deployment parameters for a worker module.

        Call this *after* :meth:`add` and before :meth:`build`.  When no
        ``worker()`` call is made for a module the original SHM-only
        behaviour is preserved (full backward compatibility).

        Args:
            module_name: Name of a previously :meth:`add`-ed module.
            host: Target host.  ``"localhost"`` keeps SHM; a remote address
                triggers DDS in ``auto`` mode.
            transport: Explicit override — ``"shm"``, ``"dds"``, ``"auto"``,
                or ``None`` (same as ``"auto"``).
            domain_id: DDS domain ID override for this worker.
            qos_profile: Named QoS profile from
                ``config/qos_profiles.yaml``.

        Returns:
            self
        """
        from .worker_config import WorkerDeployment

        self._worker_deployments.register(
            WorkerDeployment(
                module_name=module_name,
                host=host,
                transport=transport,
                domain_id=domain_id,
                qos_profile=qos_profile,
            )
        )
        return self

    def global_config(self, n_workers: int = 0, **kwargs: Any) -> Blueprint:
        """Set system-level configuration.

        Supports the dimos-style chained build::

            autoconnect(...).global_config(n_workers=4).build()

        Args:
            n_workers: Worker subprocess count.  0 = single-process (default).
        """
        self._global_cfg = {"n_workers": n_workers, **kwargs}
        return self

    def route_contract(self, name: Any) -> Blueprint:
        """Attach the external route contract for this Blueprint.

        Route presets describe runtime modes such as ``robot()``, ``replay()``,
        or ``sim()``. Passing a string remains supported for compatibility.

        This records the topic/schema/port contract for introspection and
        validation only. It does not change internal ModulePort delivery.
        """

        value = str(getattr(name, "name", name)).strip()
        if not value:
            raise ValueError("route contract name must not be empty")
        self._route_name = value
        self._route_spec = _coerce_route_spec(name, value)
        return self

    def routed_delivery(self, name: Any) -> Blueprint:
        """Route explicitly-topiced Module wires through the selected backend.

        This is intentionally more explicit than ``route_contract`` because it
        can move internal Module connections onto DDS/SHM.
        """

        self.route_contract(name)
        self._routed_delivery_enabled = True
        return self

    def route(self, name: Any) -> Blueprint:
        """Deprecated alias for ``routed_delivery``.

        Older tests and tools used ``route`` to mean "make matching wire topics
        use the route backend". New code should call ``route_contract`` for
        metadata-only contracts or ``routed_delivery`` for actual transport.
        """
        warnings.warn(
            "Blueprint.route() is deprecated. Use route_contract() for metadata-only "
            "contracts, or routed_delivery() for transport-backed wires.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.routed_delivery(name)

    def auto_wire(self) -> Blueprint:
        """Enable automatic port matching by (port_name, msg_type).

        Rules:
          1. Same name + same msg_type ->connected.
          2. Ports already explicitly wired are skipped.
          3. One Out may fan out to multiple In ports.
          4. If multiple Out ports match a single In, the connection is skipped
             with a warning (ambiguity).  Add an explicit wire() to resolve.

        Returns:
            self
        """
        self._auto_wired = True
        return self

    def export_graph(self, *, profile: str | None = None) -> ModuleGraph:
        """Return a serializable declaration graph without instantiating modules."""

        from runtime.introspection.module_graph import ModuleGraph

        return ModuleGraph.from_blueprint(self, profile=profile)

    # -- namespace ----------------------------------------------------------

    def namespace(self, prefix: str) -> Blueprint:
        """Prefix all module aliases and wire module names with ``prefix/``.

        This lets multiple copies of the same module classes coexist
        in a merged Blueprint without name collisions::

            bp_a = Blueprint().add(Navigation, alias="nav.mission")
            bp_b = Blueprint().add(Navigation, alias="nav.mission")
            bp_a.namespace("robot_0").merge(bp_b.namespace("robot_1"))
        """
        pfx = prefix.rstrip("/") + "/"
        for ent in self._entries:
            ent.alias = pfx + ent.name
        self._wires = [
            _WireSpec(
                pfx + w.out_module,
                w.out_port,
                pfx + w.in_module,
                w.in_port,
                transport=w.transport,
                topic=w.topic,
                delivery=w.delivery,
            )
            for w in self._wires
        ]
        if self._swap_config is not None:
            self._swap_config = {k: (pfx + v if isinstance(v, str) else v) for k, v in self._swap_config.items()}
        return self

    # -- merge --------------------------------------------------------------

    def merge(self, other: Blueprint) -> Blueprint:
        """Merge all modules and wires from another Blueprint into this one."""
        existing = {e.name for e in self._entries}
        for entry in other._entries:
            if entry.name in existing:
                raise ValueError(f"Cannot merge: module '{entry.name}' exists in both blueprints.")
            self._entries.append(entry)
            existing.add(entry.name)
        self._wires.extend(other._wires)
        if other._auto_wired:
            self._auto_wired = True
        if other._route_name:
            if self._route_name and self._route_name != other._route_name:
                raise ValueError(f"Cannot merge: route contract mismatch {self._route_name!r} != {other._route_name!r}")
            self._route_name = other._route_name
            self._route_spec = other._route_spec
        self._routed_delivery_enabled = self._routed_delivery_enabled or other._routed_delivery_enabled
        return self

    # -- build --------------------------------------------------------------

    def build(
        self,
        transport: Transport | None = None,
        n_workers: int = 0,
    ) -> SystemHandle:
        """Instantiate all modules, apply wiring, and return a runtime handle.

        Args:
            transport: Optional shared Transport.  Defaults to LocalTransport.
            n_workers: Worker subprocess count.  0 = single-process (default).

        Returns:
            SystemHandle (n_workers=0) or WorkerSystemHandle (n_workers>0)

        Raises:
            ValueError: Unknown module or port in an explicit wire().
            TypeError:  Type mismatch in an explicit wire().
        """
        n_workers = self._global_cfg.get("n_workers", n_workers)
        # Worker mode: only activate when explicitly requested via n_workers > 0
        # or LINGTU_WORKERS env var. Auto-detect is disabled until BPU/CLIP
        # compatibility with multiprocessing fork is verified on S100P aarch64.
        if n_workers == 0:
            import os as _os

            n_workers = int(_os.environ.get("LINGTU_WORKERS", "0"))
        if n_workers > 0:
            return self._build_worker_mode(n_workers)

        if transport is None:
            transport = LocalTransport()

        # 1. Instantiate modules
        instances: dict[str, Module] = {}
        for entry in self._entries:
            inst = entry.instance if entry.instance is not None else entry.module_cls(**entry.config)
            instances[entry.name] = inst

        # 1b. Warn about declared soft dependencies that are not registered.
        self._check_soft_depends(instances)

        # 2. Collect ports
        out_ports: dict[str, dict[str, Out]] = {n: m.ports_out for n, m in instances.items()}
        in_ports: dict[str, dict[str, In]] = {n: m.ports_in for n, m in instances.items()}

        # 3. Apply explicit wires
        wired_in: set[tuple[str, str]] = set()
        connections: list[ConnectionKey] = []
        connection_metadata: ConnectionMetadata = {}
        transport_cache: dict[tuple[str, str], Transport] = {}
        for spec in self._wires:
            _do_wire(
                spec,
                instances,
                out_ports,
                in_ports,
                wired_in,
                connections,
                connection_metadata,
                transport_cache,
                self._route_spec if self._routed_delivery_enabled else None,
            )

        # 4. Auto-wire remaining ports
        if self._auto_wired:
            _do_auto_wire(
                instances,
                out_ports,
                in_ports,
                wired_in,
                connections,
                connection_metadata,
            )

        # 5. Bind the global transport to Out ports that have no per-wire transport
        for mod_name, mod in instances.items():
            for port_name, port in mod.ports_out.items():
                if port._transport is None:
                    port._bind_transport(transport, f"/{mod_name}/{port_name}")

        # 6. Topological startup order
        startup_order = _topo_sort(instances, connections)

        # 7. Notify modules of the full module set
        for mod in instances.values():
            mod.on_system_modules(dict(instances))

        handle = SystemHandle(
            modules=instances,
            transport=transport,
            connections=connections,
            connection_metadata=connection_metadata,
            startup_order=startup_order,
        )

        # 8. Post-build swap setup (opt-in, set by product blueprints)
        if self._swap_config:
            handle.enable_swap(**self._swap_config)
            # Propagate swap manager to modules that can accept it
            # (e.g. GatewayModule).  SwapManager is created AFTER
            # on_system_modules() runs, so modules cannot discover it
            # during the notification phase.
            for mod in instances.values():
                setter = getattr(mod, "_set_swap_manager", None)
                if callable(setter):
                    setter(handle.swap_manager)

        return handle

    def _check_soft_depends(self, instances: dict[str, Module | type[Module]]) -> None:
        """Warn about missing soft dependencies declared by modules."""
        registered_names = set(instances.keys())
        for mod in instances.values():
            mod_cls = mod if isinstance(mod, type) else type(mod)
            soft_deps = getattr(mod_cls, "SOFT_DEPENDS", [])
            for dep in soft_deps:
                if dep not in registered_names:
                    logger.warning(
                        "[Blueprint] %s declares SOFT_DEPENDS on '%s' which is not registered. "
                        "Related features will be degraded.",
                        mod_cls.__name__,
                        dep,
                    )

    def __repr__(self) -> str:
        modules = ", ".join(e.name for e in self._entries)
        return f"Blueprint(modules=[{modules}], wires={len(self._wires)}, auto_wire={self._auto_wired})"

    # -- worker-mode build --------------------------------------------------

    def _build_worker_mode(self, n_workers: int) -> WorkerSystemHandle:
        """Deploy modules to Worker subprocesses.

        Modules with _run_in_main=True stay in the main process and receive
        RPCClient proxies via on_system_modules().  All others are deployed
        to worker subprocesses round-robin.

        Cross-boundary wires default to SHM for the data path.  When a
        :meth:`worker` deployment specifies a remote host or explicit DDS
        transport, DDS is used instead.
        """
        from runtime.coordinator import ModuleCoordinator

        # Partition modules into: main-process locals vs worker subprocesses.
        # _run_in_worker modules are grouped by _worker_group so related heavy
        # modules share a single subprocess (and its GIL), but isolate from
        # the main process Gateway/Navigation.
        worker_entries = [e for e in self._entries if getattr(e.module_cls, "_run_in_worker", False)]
        local_entries = [e for e in self._entries if not getattr(e.module_cls, "_run_in_worker", False)]

        # Count distinct worker groups to decide n_workers
        groups: dict[str, list] = {}
        for e in worker_entries:
            grp = getattr(e.module_cls, "_worker_group", "") or "default"
            groups.setdefault(grp, []).append(e)
        actual_workers = max(n_workers, len(groups)) if groups else 0

        coord = ModuleCoordinator(n_workers=actual_workers)
        coord.start()

        try:
            # Assign each group to a dedicated worker index
            group_worker_id: dict[str, int] = {}
            for idx, grp_name in enumerate(sorted(groups.keys())):
                group_worker_id[grp_name] = idx

            proxies: dict[str, Any] = {}
            for entry in worker_entries:
                grp = getattr(entry.module_cls, "_worker_group", "") or "default"
                wid = group_worker_id[grp]
                proxies[entry.name] = coord.deploy(entry.module_cls, entry.name, kwargs=entry.config, worker_id=wid)

            local_instances: dict[str, Any] = {}
            for entry in local_entries:
                inst = entry.instance if entry.instance is not None else entry.module_cls(**entry.config)
                local_instances[entry.name] = inst

            local_out = {n: m.ports_out for n, m in local_instances.items()}
            local_in = {n: m.ports_in for n, m in local_instances.items()}
            wired_in: set[tuple[str, str]] = set()
            connections: list[tuple[str, str, str, str]] = []
            connection_metadata: ConnectionMetadata = {}
            transport_cache: dict[tuple[str, str], Transport] = {}

            for spec in self._wires:
                out_mod, out_port = spec.out_module, spec.out_port
                in_mod, in_port = spec.in_module, spec.in_port
                topic = _transport_topic(spec)
                key = (out_mod, out_port, in_mod, in_port)
                out_worker = out_mod in proxies
                in_worker = in_mod in proxies

                if out_worker and in_worker:
                    coord._mgr.bind_port(coord._assignments[out_mod], out_mod, out_port, "out", topic)
                    coord._mgr.bind_port(coord._assignments[in_mod], in_mod, in_port, "in", topic)
                    _record_connection_metadata(
                        connection_metadata,
                        key,
                        delivery="worker",
                        transport="shm",
                        topic=topic,
                    )
                elif not out_worker and not in_worker:
                    _do_wire(
                        spec,
                        local_instances,
                        local_out,
                        local_in,
                        wired_in,
                        connections,
                        connection_metadata,
                        transport_cache,
                        self._route_spec if self._routed_delivery_enabled else None,
                    )
                    continue
                elif out_worker:
                    coord._mgr.bind_port(coord._assignments[out_mod], out_mod, out_port, "out", topic)
                    xport_name = self._worker_deployments.resolve_transport(out_mod, default="shm")
                    p = local_in.get(in_mod, {}).get(in_port)
                    if p is not None and (in_mod, in_port) not in wired_in:
                        from runtime.transport.factory import create_transport_adapter

                        adapter = create_transport_adapter(xport_name)
                        adapter.subscribe(topic, p._deliver)
                        wired_in.add((in_mod, in_port))
                    _record_connection_metadata(
                        connection_metadata,
                        key,
                        delivery="worker_to_local",
                        transport=xport_name,
                        topic=topic,
                    )
                else:
                    xport_name = self._worker_deployments.resolve_transport(in_mod, default="shm")
                    p = local_out.get(out_mod, {}).get(out_port)
                    if p is not None:
                        from runtime.transport.factory import create_transport_adapter

                        adapter = create_transport_adapter(xport_name)
                        p._bind_transport(adapter, topic)
                    coord._mgr.bind_port(coord._assignments[in_mod], in_mod, in_port, "in", topic)
                    _record_connection_metadata(
                        connection_metadata,
                        key,
                        delivery="local_to_worker",
                        transport=xport_name,
                        topic=topic,
                    )

                connections.append((out_mod, out_port, in_mod, in_port))

            if self._auto_wired:
                _do_auto_wire(
                    local_instances,
                    local_out,
                    local_in,
                    wired_in,
                    connections,
                    connection_metadata,
                )

            # Warn about declared soft dependencies that are not registered.
            self._check_soft_depends({**local_instances, **{e.name: e.module_cls for e in worker_entries}})

            coord.setup_all()
            coord.start_all()
            failed: dict[str, str] = {}
            for name, inst in local_instances.items():
                try:
                    inst.setup()
                except Exception as e:
                    logger.error("WorkerMode: local module %s setup() FAILED: %s", name, e, exc_info=True)
                    failed[name] = f"setup: {e}"
            for name, inst in local_instances.items():
                if name in failed:
                    continue
                try:
                    inst.start()
                except Exception as e:
                    logger.error("WorkerMode: local module %s start() FAILED: %s", name, e, exc_info=True)
                    failed[name] = f"start: {e}"

            all_modules: dict[str, Any] = {**proxies, **local_instances}
            for inst in local_instances.values():
                inst.on_system_modules(all_modules)

            if failed:
                logger.warning(
                    "WorkerMode started with %d/%d local modules failed: %s",
                    len(failed),
                    len(local_instances),
                    list(failed.keys()),
                )
            logger.info(
                "WorkerMode: %d worker modules (%d workers), %d local, %d connections",
                len(proxies),
                n_workers,
                len(local_instances),
                len(connections),
            )
            return WorkerSystemHandle(
                coord,
                proxies,
                local_instances,
                connections,
                connection_metadata,
            )
        except Exception:
            logger.exception("WorkerMode build failed; shutting down workers")
            coord.shutdown()
            raise


# ---------------------------------------------------------------------------
# Module-level helpers (extracted from Blueprint to remove @staticmethod noise)
# ---------------------------------------------------------------------------


def _coerce_route_spec(route: Any, route_name: str) -> Any | None:
    """Return a RouteSpec-like object when the selected route is known."""
    if hasattr(route, "backend_for"):
        return route
    try:
        from runtime.route_contract.routes import route_preset

        return route_preset(route_name)
    except Exception:
        return None


def _resolve_transport(spec: Any) -> Transport | None:
    """Compatibility wrapper for wire delivery resolution."""
    return resolve_wire_delivery(spec)


def _transport_cache_key(spec: Any) -> tuple[str, str]:
    """Return a stable per-build key for a delivery specification."""
    return wire_delivery_cache_key(spec)


def _resolve_transport_cached(
    spec: Any,
    cache: dict[tuple[str, str], Transport] | None,
) -> Transport | None:
    """Resolve a transport spec, reusing one backend per build/spec."""
    if spec is None:
        return None
    if cache is None:
        return _resolve_transport(spec)
    key = _transport_cache_key(spec)
    transport = cache.get(key)
    if transport is None:
        transport = _resolve_transport(spec)
        if transport is not None:
            cache[key] = transport
    return transport


def _resolve_route_transport(backend: str) -> Transport:
    """Resolve a route-selected backend to a typed transport adapter."""
    normalized = str(backend).strip().lower()
    if normalized not in {"dds", "shm"}:
        raise ValueError(f"Runtime route backend '{normalized}' is not supported by Blueprint.build()")
    from runtime.transport.factory import create_route_transport_adapter

    return create_route_transport_adapter(normalized)


def _resolve_route_transport_cached(
    backend: str,
    cache: dict[tuple[str, str], Transport] | None,
) -> Transport:
    if cache is None:
        return _resolve_route_transport(backend)
    key = ("route", str(backend).strip().lower())
    transport = cache.get(key)
    if transport is None:
        transport = _resolve_route_transport(backend)
        cache[key] = transport
    return transport


def _transport_name(transport: Any) -> str:
    """Return a readable delivery/backend name for diagnostics."""
    return wire_delivery_name(transport)


def _explicit_topic(spec: _WireSpec) -> str | None:
    return explicit_wire_topic(spec)


def _transport_topic(spec: _WireSpec) -> str:
    return default_wire_topic(spec)


def _route_backend_for_topic(route_spec: Any | None, topic: str | None) -> str | None:
    if route_spec is None or topic is None:
        return None
    backend = str(route_spec.backend_for(topic)).strip().lower()
    if backend in {"", "callback", "local"}:
        return None
    return backend


def _record_connection_metadata(
    metadata: ConnectionMetadata | None,
    key: ConnectionKey,
    *,
    delivery: str,
    transport: str,
    topic: str | None,
) -> None:
    if metadata is None:
        return
    metadata[key] = {
        "delivery": delivery,
        "transport": transport,
        "topic": topic,
    }


def _type_name(msg_type: Any) -> str:
    return getattr(msg_type, "__name__", str(msg_type))


def _msg_types_compatible(out_type: Any, in_type: Any) -> bool:
    if out_type is Any or in_type is Any or out_type == in_type:
        return True
    return (is_typeddict(out_type) and in_type is dict) or (out_type is dict and is_typeddict(in_type))


def _do_wire(
    spec: _WireSpec,
    instances: dict[str, Any],
    out_ports: dict[str, dict[str, Out]],
    in_ports: dict[str, dict[str, In]],
    wired_in: set[tuple[str, str]],
    connections: list[ConnectionKey],
    connection_metadata: ConnectionMetadata | None = None,
    transport_cache: dict[tuple[str, str], Transport] | None = None,
    route_spec: Any | None = None,
) -> None:
    """Apply one explicit wire specification."""
    if spec.out_module not in instances:
        raise ValueError(f"wire(): unknown output module '{spec.out_module}'")
    if spec.in_module not in instances:
        raise ValueError(f"wire(): unknown input module '{spec.in_module}'")

    out = out_ports.get(spec.out_module, {}).get(spec.out_port)
    inp = in_ports.get(spec.in_module, {}).get(spec.in_port)

    if out is None:
        raise ValueError(f"wire(): '{spec.out_module}' has no Out port '{spec.out_port}'")
    if inp is None:
        raise ValueError(f"wire(): '{spec.in_module}' has no In port '{spec.in_port}'")

    if not _msg_types_compatible(out.msg_type, inp.msg_type):
        raise TypeError(
            f"wire(): type mismatch "
            f"{spec.out_module}.{spec.out_port} ({_type_name(out.msg_type)}) ->"
            f"{spec.in_module}.{spec.in_port} ({_type_name(inp.msg_type)})"
        )

    key = (spec.out_module, spec.out_port, spec.in_module, spec.in_port)
    explicit_topic = _explicit_topic(spec)
    delivery_spec = spec.delivery_spec
    route_backend = None
    if delivery_spec is None:
        route_backend = _route_backend_for_topic(route_spec, explicit_topic)

    if route_backend is not None:
        transport = _resolve_route_transport_cached(route_backend, transport_cache)
    else:
        transport = _resolve_transport_cached(delivery_spec, transport_cache)
    if transport is None:
        out._add_callback(inp._deliver)
        mode = "callback"
        _record_connection_metadata(
            connection_metadata,
            key,
            delivery="callback",
            transport="callback",
            topic=explicit_topic,
        )
    else:
        topic = explicit_topic or _transport_topic(spec)
        out._bind_transport(transport, topic)
        transport.subscribe(topic, inp._deliver)
        transport_name = _transport_name(transport)
        mode = f"transport({transport_name})"
        _record_connection_metadata(
            connection_metadata,
            key,
            delivery="transport",
            transport=transport_name,
            topic=topic,
        )

    wired_in.add((spec.in_module, spec.in_port))
    connections.append(key)
    logger.debug(
        "Wired %s.%s ->%s.%s [%s, %s]",
        spec.out_module,
        spec.out_port,
        spec.in_module,
        spec.in_port,
        _type_name(out.msg_type),
        mode,
    )


def _get_ns(mod_name: str) -> str:
    """Extract the leading namespace from a ``/``-separated module name.

    Returns:
        The namespace prefix (e.g. ``"robot_0"``) or ``""`` if the name
        has no namespace separator.
    """
    idx = mod_name.find("/")
    return mod_name[:idx] if idx > 0 else ""


def _do_auto_wire(
    instances: dict[str, Any],
    out_ports: dict[str, dict[str, Out]],
    in_ports: dict[str, dict[str, In]],
    wired_in: set[tuple[str, str]],
    connections: list[ConnectionKey],
    connection_metadata: ConnectionMetadata | None = None,
) -> None:
    """Auto-match Out->In by (port_name, msg_type).

    When multiple candidates exist, the function prefers those in the same
    namespace as the input module (determined by a leading ``namespace/``
    prefix).  This prevents false ambiguity when two separately-namespaced
    stacks (e.g. ``robot_0/`` and ``robot_1/``) are merged into a single
    Blueprint.
    """
    # Index by name; typing.Any acts as a wildcard during compatibility checks.
    out_index: dict[str, list[tuple[str, Out]]] = defaultdict(list)
    for mod_name, ports in out_ports.items():
        for port_name, port in ports.items():
            out_index[port_name].append((mod_name, port))

    for in_mod, ports in in_ports.items():
        for in_port_name, in_port in ports.items():
            if (in_mod, in_port_name) in wired_in:
                continue

            candidates = [
                (mn, op)
                for mn, op in out_index.get(in_port_name, [])
                if mn != in_mod and _msg_types_compatible(op.msg_type, in_port.msg_type)
            ]
            exact_candidates = [(mn, op) for mn, op in candidates if op.msg_type == in_port.msg_type]
            if exact_candidates:
                candidates = exact_candidates

            # ----- namespace-aware candidate filtering -----
            in_ns = _get_ns(in_mod)
            if in_ns:
                same_ns = [(mn, op) for mn, op in candidates if _get_ns(mn) == in_ns]
                if same_ns:
                    candidates = same_ns
            # -----------------------------------------------

            if len(candidates) == 1:
                out_mod, out_port = candidates[0]
                out_port._add_callback(in_port._deliver)
                wired_in.add((in_mod, in_port_name))
                key = (out_mod, in_port_name, in_mod, in_port_name)
                connections.append(key)
                _record_connection_metadata(
                    connection_metadata,
                    key,
                    delivery="callback",
                    transport="callback",
                    topic=None,
                )
                logger.debug(
                    "Auto-wired %s.%s ->%s.%s [%s]",
                    out_mod,
                    in_port_name,
                    in_mod,
                    in_port_name,
                    _type_name(in_port.msg_type),
                )
            elif len(candidates) > 1:
                logger.warning(
                    "Auto-wire ambiguity for %s.%s [%s]: %d candidates -add an explicit wire() to resolve",
                    in_mod,
                    in_port_name,
                    _type_name(in_port.msg_type),
                    len(candidates),
                )


def _find_cycle(adj: dict[str, list[str]], nodes: list[str]) -> list[str]:
    """DFS-based cycle finder. Returns one cycle path, or empty list if none found."""
    visited: set[str] = set()
    stack: list[str] = []
    stack_set: set[str] = set()

    def dfs(node: str) -> list[str]:
        visited.add(node)
        stack.append(node)
        stack_set.add(node)
        for neighbor in adj.get(node, []):
            if neighbor not in visited:
                cycle = dfs(neighbor)
                if cycle:
                    return cycle
            elif neighbor in stack_set:
                # Found cycle -extract it from current stack
                idx = stack.index(neighbor)
                return stack[idx:]
        stack.pop()
        stack_set.discard(node)
        return []

    for n in nodes:
        if n not in visited:
            cycle = dfs(n)
            if cycle:
                return cycle
    return []


def _topo_sort(
    instances: dict[str, Any],
    connections: list[tuple[str, str, str, str]],
) -> list[str]:
    """Kahn's topological sort; falls back to layer order on cycles.

    Upstream modules start first.  Within each topological level, lower-layer
    modules are sorted before higher-layer ones.

    Robotics control loops are inherently cyclic (odometry flows up while
    cmd_vel flows down). When a cycle is detected, the involved modules are
    logged at WARNING level and sorted by layer number as a sensible fallback.
    """
    in_degree: dict[str, int] = {n: 0 for n in instances}
    adj: dict[str, list[str]] = defaultdict(list)
    for out_mod, _, in_mod, _ in connections:
        if out_mod != in_mod:
            adj[out_mod].append(in_mod)
            in_degree[in_mod] = in_degree.get(in_mod, 0) + 1

    result: list[str] = []
    ready = sorted(
        (n for n, d in in_degree.items() if d == 0),
        key=lambda n: instances[n].layer or 0,
    )

    while ready:
        node = ready.pop(0)
        result.append(node)
        newly_ready = []
        for neighbor in adj.get(node, []):
            in_degree[neighbor] -= 1
            if in_degree[neighbor] == 0:
                newly_ready.append(neighbor)
        # Insert new candidates in layer order
        newly_ready.sort(key=lambda n: instances[n].layer or 0)
        ready = sorted(ready + newly_ready, key=lambda n: instances[n].layer or 0)

    if len(result) < len(instances):
        remaining = [n for n in instances if n not in set(result)]
        cycle = _find_cycle(adj, remaining)
        cycle_str = " ->".join([*cycle, cycle[0]]) if cycle else str(remaining)
        logger.warning(
            "Wire cycle detected -startup order may be imprecise for: %s. "
            "Cycle: %s. Falling back to layer-sorted order. "
            "This is expected for control loops (e.g. odometry ->/ cmd_vel ->.",
            remaining,
            cycle_str,
        )
        # Sort remaining by layer descending: higher-layer orchestrators start first
        # so they're ready to accept data when lower-layer hardware comes up.
        remaining.sort(key=lambda n: instances[n].layer or 0, reverse=True)
        result.extend(remaining)

    return result


# ---------------------------------------------------------------------------
# SystemHandle
# ---------------------------------------------------------------------------


class SystemHandle:
    """Runtime handle for a single-process module system.

    Manages the full lifecycle: setup ->start ->[running] ->stop.
    """

    def __init__(
        self,
        modules: dict[str, Module],
        transport: Transport,
        connections: list[ConnectionKey],
        connection_metadata: ConnectionMetadata | None,
        startup_order: list[str],
    ) -> None:
        self._modules = modules
        self._transport = transport
        self._connections = connections
        self._connection_metadata = connection_metadata or {}
        self._startup_order = startup_order
        self._started = False
        self._failed_modules: dict[str, str] = {}
        self.swap_manager: Any = None  # set by enable_swap()

    # -- swap manager --------------------------------------------------------

    def enable_swap(
        self,
        mux_name: str = "nav.velocity_mux",
        nav_name: str = "nav.mission",
        driver_name: str = "ThunderDriver",
    ) -> SystemHandle:
        """Create and activate a SwapManager for this system.

        The SwapManager stores references to the three core navigation modules
        so backend-swap operations can reach them by name.

        Args:
            mux_name:    Name of the VelocityMux module.
            nav_name:    Name of the nav.mission.
            driver_name: Name of the driver (robot interface) module.

        Returns:
            self (fluent interface).
        """
        from runtime.swap_manager import SwapManager

        mux = self._modules.get(mux_name)
        nav = self._modules.get(nav_name)
        self.swap_manager = SwapManager(
            system=self,
            mux=mux,
            nav=nav,
        )
        self.swap_manager.enable()
        return self

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """setup() then start() all modules in topological order.

        Module-level error isolation: if a module's setup() or start()
        fails, the module is marked as failed and skipped. Other modules
        continue starting. This prevents one broken module from taking
        down the entire system.
        """
        if self._started:
            logger.warning("SystemHandle.start() called but system is already running")
            return
        failed: dict = {}
        # Phase 0: preflight checks
        for name in self._startup_order:
            try:
                reason = self._modules[name].preflight()
                if reason:
                    logger.error("Module %s preflight FAILED: %s", name, reason)
                    failed[name] = f"preflight: {reason}"
            except Exception as e:
                logger.error("Module %s preflight FAILED: %s", name, e, exc_info=True)
                failed[name] = f"preflight: {e}"
        # Phase 1: setup
        for name in self._startup_order:
            if name in failed:
                continue
            try:
                self._modules[name].setup()
            except Exception as e:
                logger.error("Module %s setup() FAILED: %s", name, e, exc_info=True)
                failed[name] = f"setup: {e}"
        for name in self._startup_order:
            if name in failed:
                continue
            try:
                self._modules[name].start()
            except Exception as e:
                logger.error("Module %s start() FAILED: %s", name, e, exc_info=True)
                failed[name] = f"start: {e}"
        self._started = True
        self._failed_modules = failed
        if failed:
            logger.warning(
                "System started with %d/%d modules failed: %s",
                len(failed),
                len(self._modules),
                list(failed.keys()),
            )
        logger.info(
            "System started: %d modules (%d failed), %d connections",
            len(self._modules),
            len(failed),
            len(self._connections),
        )

    def stop(self, timeout_per_module: float = 5.0) -> None:
        """stop() all modules in reverse topological order, then close transport.

        Each module gets *timeout_per_module* seconds to stop gracefully.
        If a module hangs, it is skipped with a warning and shutdown continues.
        """
        if not self._started:
            return
        import threading as _th

        hung_modules: list[str] = []
        stop_errors: dict[str, str] = {}

        def _safe_stop(mod_ref, mod_name):
            try:
                mod_ref.stop()
            except Exception as exc:
                stop_errors[mod_name] = str(exc)
                logger.exception("Error stopping module %s", mod_name)

        for name in reversed(self._startup_order):
            mod = self._modules.get(name)
            if mod is None:
                continue
            t = _th.Thread(target=_safe_stop, args=(mod, name), daemon=True, name=f"stop-{name}")
            t.start()
            t.join(timeout=timeout_per_module)
            if t.is_alive():
                hung_modules.append(name)
                logger.warning("Module %s did not stop within %.1fs -skipping", name, timeout_per_module)
        if hasattr(self._transport, "close"):
            try:
                self._transport.close()
            except Exception:
                logger.exception("Error closing transport")
        if hung_modules:
            logger.warning("Hung modules during shutdown: %s", hung_modules)
        self._modules.clear()
        self._connections.clear()
        self._connection_metadata.clear()
        self._started = False
        logger.info("System stopped")

    # -- access -------------------------------------------------------------

    def get_module(self, name: str) -> Module:
        """Get a module instance by name. Raises KeyError if not found."""
        if name not in self._modules:
            raise KeyError(f"Unknown module: '{name}'")
        return self._modules[name]

    @property
    def modules(self) -> dict[str, Module]:
        """Return a snapshot of all modules keyed by name."""
        return dict(self._modules)

    @property
    def connections(self) -> list[tuple[str, str, str, str]]:
        """Return a snapshot of all wire connections."""
        return list(self._connections)

    @property
    def connection_metadata(self) -> ConnectionMetadata:
        """Return delivery metadata keyed by connection tuple."""
        return {key: dict(value) for key, value in self._connection_metadata.items()}

    @property
    def started(self) -> bool:
        """True if the system has been started."""
        return self._started

    # -- health -------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """Return a health summary dict."""
        total_in = sum(p.msg_count for m in self._modules.values() for p in m.ports_in.values())
        total_out = sum(p.msg_count for m in self._modules.values() for p in m.ports_out.values())
        return {
            "started": self._started,
            "module_count": len(self._modules),
            "connection_count": len(self._connections),
            "startup_order": self._startup_order,
            "failed_modules": dict(self._failed_modules),
            "layer_violations": [],
            "total_messages_in": total_in,
            "total_messages_out": total_out,
            "modules": {n: m.port_summary() for n, m in self._modules.items()},
        }

    def comm_health(self) -> dict[str, Any]:
        """Aggregate communication health across all modules.

        Returns per-connection stats: rate, drops, errors, latency.
        Flags unhealthy links (stale, high error rate, high latency).
        """
        links = []
        warnings = []
        total_drops = 0
        total_errors = 0

        for src_mod, src_port, dst_mod, dst_port in self._connections:
            key = (src_mod, src_port, dst_mod, dst_port)
            src_m = self._modules.get(src_mod)
            dst_m = self._modules.get(dst_mod)
            if not src_m or not dst_m:
                continue
            out_p = src_m.ports_out.get(src_port)
            in_p = dst_m.ports_in.get(dst_port)
            if not out_p or not in_p:
                continue

            drops = in_p.drop_count
            errors = in_p.callback_errors + out_p.publish_errors
            total_drops += drops
            total_errors += errors

            meta = self._connection_metadata.get(
                key,
                {"delivery": "unknown", "transport": "unknown", "topic": None},
            )
            link = {
                "src": f"{src_mod}.{src_port}",
                "dst": f"{dst_mod}.{dst_port}",
                "delivery": meta.get("delivery", "unknown"),
                "transport": meta.get("transport", "unknown"),
                "topic": meta.get("topic"),
                "out_rate_hz": round(out_p.rate_hz, 1),
                "in_rate_hz": round(in_p.rate_hz, 1),
                "out_count": out_p.msg_count,
                "in_count": in_p.msg_count,
                "drops": drops,
                "errors": errors,
                "avg_cb_ms": round(in_p.avg_callback_ms, 2),
                "max_cb_ms": round(in_p.max_callback_ms, 2),
                "stale_ms": round(in_p.stale_ms, 1),
            }
            links.append(link)

            # Flag unhealthy links
            if in_p.stale_ms > 5000 and in_p.msg_count > 0:
                warnings.append(f"{link['src']}->{link['dst']}: stale {in_p.stale_ms:.0f}ms")
            if errors > 0:
                warnings.append(f"{link['src']}->{link['dst']}: {errors} errors")
            if in_p.max_callback_ms > 500:
                warnings.append(f"{link['src']}->{link['dst']}: slow callback {in_p.max_callback_ms:.0f}ms")

        return {
            "link_count": len(links),
            "total_drops": total_drops,
            "total_errors": total_errors,
            "warnings": warnings,
            "links": links,
        }

    def __repr__(self) -> str:
        status = "running" if self._started else "stopped"
        return f"SystemHandle({status}, modules={len(self._modules)}, connections={len(self._connections)})"


# ---------------------------------------------------------------------------
# WorkerSystemHandle
# ---------------------------------------------------------------------------


class WorkerSystemHandle:
    """Runtime handle for a multi-process worker-mode system.

    Worker modules are accessible as RPCClient proxies; main-process modules
    as bare instances.  Mirrors the SystemHandle interface.
    """

    def __init__(
        self,
        coord: Any,
        proxies: dict[str, Any],
        local_instances: dict[str, Any],
        connections: list[ConnectionKey],
        connection_metadata: ConnectionMetadata | None = None,
    ) -> None:
        self._coord = coord
        self._proxies = proxies
        self._local = local_instances
        self._connections = connections
        self._connection_metadata = connection_metadata or {}
        self._started = True  # lifecycle already completed during build()

    def start(self) -> None:
        """No-op -lifecycle is handled inside build()."""

    def stop(self) -> None:
        """Stop main-process modules, then shut down all worker subprocesses."""
        for name in reversed(list(self._local)):
            try:
                self._local[name].stop()
            except Exception:
                logger.exception("Error stopping local module %s", name)
        self._coord.shutdown()
        self._local.clear()
        self._proxies.clear()
        self._connections.clear()
        self._started = False

    def get_module(self, name: str) -> Any:
        """Get a worker or local module by name. Raises KeyError if not found."""
        if name in self._proxies:
            return self._proxies[name]
        if name in self._local:
            return self._local[name]
        raise KeyError(f"Unknown module: '{name}'")

    @property
    def modules(self) -> dict[str, Any]:
        """Return all modules (worker proxies + local instances)."""
        return {**self._proxies, **self._local}

    @property
    def connections(self) -> list[tuple[str, str, str, str]]:
        """Return a snapshot of all wire connections."""
        return list(self._connections)

    @property
    def connection_metadata(self) -> ConnectionMetadata:
        """Return delivery metadata keyed by connection tuple."""
        return {key: dict(value) for key, value in self._connection_metadata.items()}

    @property
    def started(self) -> bool:
        """True if the system has been started."""
        return self._started

    def health(self) -> dict[str, Any]:
        """Return a health summary dict across all worker and local modules."""
        modules: dict[str, Any] = {}
        for name, proxy in self._proxies.items():
            try:
                modules[name] = proxy.health()
            except Exception as exc:
                modules[name] = {"error": str(exc)}
        for name, inst in self._local.items():
            modules[name] = inst.port_summary()
        return {
            "started": self._started,
            "module_count": len(self._proxies) + len(self._local),
            "worker_modules": list(self._proxies.keys()),
            "local_modules": list(self._local.keys()),
            "connection_count": len(self._connections),
            "modules": modules,
        }

    def comm_health(self) -> dict[str, Any]:
        """Aggregate communication health for local-module connections.

        Worker-proxy connections are reported as-is from the proxy's
        health() when available; cross-process port stats are limited.
        """
        links: list[dict] = []
        warnings: list[str] = []
        total_drops = 0
        total_errors = 0

        for src_mod, src_port, dst_mod, dst_port in self._connections:
            key = (src_mod, src_port, dst_mod, dst_port)
            src_m = self._local.get(src_mod) or self._proxies.get(src_mod)
            dst_m = self._local.get(dst_mod) or self._proxies.get(dst_mod)
            if not src_m or not dst_m:
                continue
            # Only local instances expose ports_out / ports_in directly
            src_ports = getattr(src_m, "ports_out", None)
            dst_ports = getattr(dst_m, "ports_in", None)
            if not src_ports or not dst_ports:
                continue
            out_p = src_ports.get(src_port)
            in_p = dst_ports.get(dst_port)
            if not out_p or not in_p:
                continue

            drops = in_p.drop_count
            errors = in_p.callback_errors + out_p.publish_errors
            total_drops += drops
            total_errors += errors

            meta = self._connection_metadata.get(
                key,
                {"delivery": "unknown", "transport": "unknown", "topic": None},
            )
            link = {
                "src": f"{src_mod}.{src_port}",
                "dst": f"{dst_mod}.{dst_port}",
                "delivery": meta.get("delivery", "unknown"),
                "transport": meta.get("transport", "unknown"),
                "topic": meta.get("topic"),
                "out_rate_hz": round(out_p.rate_hz, 1),
                "in_rate_hz": round(in_p.rate_hz, 1),
                "out_count": out_p.msg_count,
                "in_count": in_p.msg_count,
                "drops": drops,
                "errors": errors,
                "avg_cb_ms": round(in_p.avg_callback_ms, 2),
                "max_cb_ms": round(in_p.max_callback_ms, 2),
                "stale_ms": round(in_p.stale_ms, 1),
            }
            links.append(link)

            if in_p.stale_ms > 5000 and in_p.msg_count > 0:
                warnings.append(f"{link['src']}->{link['dst']}: stale {in_p.stale_ms:.0f}ms")
            if errors > 0:
                warnings.append(f"{link['src']}->{link['dst']}: {errors} errors")
            if in_p.max_callback_ms > 500:
                warnings.append(f"{link['src']}->{link['dst']}: slow callback {in_p.max_callback_ms:.0f}ms")

        return {
            "link_count": len(links),
            "total_drops": total_drops,
            "total_errors": total_errors,
            "warnings": warnings,
            "links": links,
        }

    def __repr__(self) -> str:
        status = "running" if self._started else "stopped"
        return f"WorkerSystemHandle({status}, workers={list(self._proxies.keys())}, local={list(self._local.keys())})"


# ---------------------------------------------------------------------------
# autoconnect()
# ---------------------------------------------------------------------------


def autoconnect(*blueprints: Blueprint) -> Blueprint:
    """Merge multiple Blueprints and enable auto_wire.

    Convenience factory for the composable-stack pattern::

        system = autoconnect(
            driver("thunder", dog_host="192.168.66.190"),
            slam("localizer"),
            perception("bpu"),
            navigation("astar"),
        ).build()
    """
    result = Blueprint()
    for bp in blueprints:
        result.merge(bp)
    result.auto_wire()
    return result
