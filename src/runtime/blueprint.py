"""Declarative assembly for one LingTu Python application runtime.

Blueprint is a runtime mechanism, not a product definition or native process
manager. It materializes one in-process Module graph from classes and wiring
rules.

Product declarations are compiled by :mod:`lingtu.assembly.compiler`
into a RunPlan; ProductControl and its internal SystemdRunner own their
lifecycle.

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
        driver("stub"),
        slam("localizer"),
        perception("bpu"),
        navigation("astar"),
    ).build()
"""

from __future__ import annotations

import logging
from collections import defaultdict
from collections.abc import Callable
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


class SystemStartupError(RuntimeError):
    """Raised when a critical Module cannot complete startup."""


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
    """Build one application-owned Module graph.

    A Blueprint does not install, launch, supervise, or stop native product
    services. Configuration methods return *self* to support method chaining.
    A Blueprint should not be modified after :meth:`build` is called.
    """

    def __init__(self) -> None:
        self._entries: list[_ModuleEntry] = []
        self._wires: list[_WireSpec] = []
        self._auto_wired: bool = False
        self._route_name: str | None = None
        self._build_checks: list[Callable[[], None]] = []
        self._required_modules: list[str] = []
    @property
    def module_names(self) -> tuple[str, ...]:
        """Return declared Module aliases without instantiating the graph."""

        return tuple(entry.name for entry in self._entries)

    @property
    def required_module_names(self) -> tuple[str, ...]:
        """Return aliases that must participate in fail-closed startup."""

        return tuple(self._required_modules)

    def require_modules(self, *module_aliases: str) -> Blueprint:
        """Declare Module aliases that are critical to product startup."""

        for alias in module_aliases:
            value = str(alias).strip()
            if not value:
                raise ValueError("required module alias must not be empty")
            if value not in self._required_modules:
                self._required_modules.append(value)
        return self

    def before_build(self, check: Callable[[], None]) -> Blueprint:
        """Register a side-effecting startup check deferred until ``build()``."""

        if not callable(check):
            raise TypeError("Blueprint before_build check must be callable")
        self._build_checks.append(check)
        return self

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
        delivery: Any = None,
        topic: str | None = None,
    ) -> Blueprint:
        """Connect an Out port to an In port.

        Args:
            out_module: Source module name (class name or alias).
            out_port:   Source port name.
            in_module:  Destination module name.
            in_port:    Destination port name.
            delivery:   Per-wire delivery mode (None/callback, local, shm,
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
                delivery=delivery,
                topic=topic,
            )
        )
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
        return self

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

            bp_a = Blueprint().add(CameraRelay, alias="camera.relay")
            bp_b = Blueprint().add(CameraRelay, alias="camera.relay")
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
                delivery=w.delivery,
                topic=w.topic,
            )
            for w in self._wires
        ]
        self._required_modules = [pfx + name for name in self._required_modules]
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
        self._build_checks.extend(other._build_checks)
        for name in other._required_modules:
            if name not in self._required_modules:
                self._required_modules.append(name)

        return self

    # -- build --------------------------------------------------------------

    def build(
        self,
        transport: Transport | None = None,
    ) -> SystemHandle:
        """Instantiate the Module graph and return its application handle.

        Args:
            transport: Optional shared Transport.  Defaults to LocalTransport.
        Returns:
            SystemHandle for the in-process Module graph.

        Raises:
            ValueError: Unknown module or port in an explicit wire().
            TypeError:  Type mismatch in an explicit wire().
        """
        missing_required = [name for name in self._required_modules if name not in self.module_names]
        if missing_required:
            raise ValueError(f"Blueprint required modules are missing: {missing_required}")

        for check in self._build_checks:
            check()

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
            critical_modules=tuple(self._required_modules),
        )
        for mod in instances.values():
            setter = getattr(mod, "set_system_handle", None)
            if callable(setter):
                setter(handle)

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

# ---------------------------------------------------------------------------
# Module-level helpers (extracted from Blueprint to remove @staticmethod noise)
# ---------------------------------------------------------------------------


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
        return resolve_wire_delivery(spec)
    key = _transport_cache_key(spec)
    transport = cache.get(key)
    if transport is None:
        transport = resolve_wire_delivery(spec)
        if transport is not None:
            cache[key] = transport
    return transport


def _transport_name(transport: Any) -> str:
    """Return a readable delivery/backend name for diagnostics."""
    return wire_delivery_name(transport)


def _explicit_topic(spec: _WireSpec) -> str | None:
    return explicit_wire_topic(spec)


def _transport_topic(spec: _WireSpec) -> str:
    return default_wire_topic(spec)


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
        critical_modules: tuple[str, ...] = (),
    ) -> None:
        self._modules = modules
        self._transport = transport
        self._connections = connections
        self._connection_metadata = connection_metadata or {}
        self._startup_order = startup_order
        self._started = False
        self._failed_modules: dict[str, str] = {}
        self._critical_modules = tuple(critical_modules)
        self._critical_failures: dict[str, str] = {}
        self._startup_state = "built"
        self._transport_closed = False
    # -- lifecycle ----------------------------------------------------------

    def start(
        self,
        *,
        startup_timeout_s: float = 30.0,
        readiness_poll_interval_s: float = 0.05,
    ) -> None:
        """Start the graph, failing closed when a critical Module is not ready."""

        if self._startup_state == "ready":
            logger.warning("SystemHandle.start() called but system is already ready")
            return
        if self._startup_state != "built":
            raise SystemStartupError(
                f"cannot start SystemHandle from {self._startup_state} state"
            )
        if startup_timeout_s < 0:
            raise ValueError("startup_timeout_s must be non-negative")
        if readiness_poll_interval_s <= 0:
            raise ValueError("readiness_poll_interval_s must be positive")

        import time as _time

        self._startup_state = "starting"
        failed: dict[str, str] = {}
        touched: set[str] = set()

        def _module_failure(name: str, phase: str, detail: object) -> None:
            message = f"{phase}: {detail}"
            failed[name] = message
            logger.error("Module %s %s FAILED: %s", name, phase, detail)
            if name in self._critical_modules:
                self._failed_modules = dict(failed)
                self._critical_failures[name] = message
                self._rollback_startup(touched)
                self._started = False
                self._startup_state = "failed"
                raise SystemStartupError(
                    f"critical module {name!r} failed during {phase}: {detail}"
                )
            self._stop_modules([name], timeout_per_module=5.0)
            touched.discard(name)

        for name in self._startup_order:
            try:
                reason = self._modules[name].preflight()
            except Exception as exc:
                logger.exception("Module %s preflight() raised", name)
                _module_failure(name, "preflight", exc)
                continue
            if reason:
                _module_failure(name, "preflight", reason)

        for name in self._startup_order:
            if name in failed:
                continue
            touched.add(name)
            try:
                self._modules[name].setup()
            except Exception as exc:
                logger.exception("Module %s setup() raised", name)
                _module_failure(name, "setup", exc)

        for name in self._startup_order:
            if name in failed:
                continue
            try:
                self._modules[name].start()
            except Exception as exc:
                logger.exception("Module %s start() raised", name)
                _module_failure(name, "start", exc)

        deadline = _time.monotonic() + startup_timeout_s
        while self._critical_modules:
            pending: dict[str, str] = {}
            for name in self._critical_modules:
                try:
                    reason = self._modules[name].startup_readiness()
                except Exception as exc:
                    logger.exception("Module %s startup_readiness() raised", name)
                    _module_failure(name, "startup_readiness", exc)
                    continue
                if reason:
                    pending[name] = str(reason)

            if not pending:
                break
            if _time.monotonic() >= deadline:
                for name, reason in pending.items():
                    message = f"startup_readiness: timeout ({reason})"
                    failed[name] = message
                    self._critical_failures[name] = message
                self._failed_modules = dict(failed)
                self._rollback_startup(touched)
                self._started = False
                self._startup_state = "failed"
                detail = ", ".join(
                    f"{name}={reason}" for name, reason in pending.items()
                )
                raise SystemStartupError(
                    f"critical module startup readiness timed out: {detail}"
                )
            remaining = max(0.0, deadline - _time.monotonic())
            _time.sleep(min(readiness_poll_interval_s, remaining))

        self._failed_modules = dict(failed)
        self._critical_failures = {}
        self._started = True
        self._startup_state = "ready"
        if failed:
            logger.warning(
                "System started with %d/%d optional modules failed: %s",
                len(failed),
                len(self._modules),
                list(failed),
            )
        logger.info(
            "System ready: %d modules (%d optional failures), %d connections",
            len(self._modules),
            len(failed),
            len(self._connections),
        )

    def _rollback_startup(self, touched: set[str]) -> None:
        names = [name for name in reversed(self._startup_order) if name in touched]
        self._stop_modules(names, timeout_per_module=5.0)
        self._close_transport()

    def _stop_modules(
        self,
        names: list[str],
        *,
        timeout_per_module: float,
    ) -> list[str]:
        import threading as _threading

        hung_modules: list[str] = []
        for name in names:
            module = self._modules.get(name)
            if module is None:
                continue

            def _safe_stop(mod_ref: Module = module, mod_name: str = name) -> None:
                try:
                    mod_ref.stop()
                except Exception:
                    logger.exception("Error stopping module %s", mod_name)

            thread = _threading.Thread(
                target=_safe_stop,
                daemon=True,
                name=f"stop-{name}",
            )
            thread.start()
            thread.join(timeout=timeout_per_module)
            if thread.is_alive():
                hung_modules.append(name)
                logger.warning(
                    "Module %s did not stop within %.1fs - skipping",
                    name,
                    timeout_per_module,
                )
        return hung_modules

    def _close_transport(self) -> None:
        if self._transport_closed:
            return
        self._transport_closed = True
        close = getattr(self._transport, "close", None)
        if callable(close):
            try:
                close()
            except Exception:
                logger.exception("Error closing transport")

    def stop(self, timeout_per_module: float = 5.0) -> None:
        """Stop Modules in reverse topology and permanently close this handle."""

        if timeout_per_module <= 0:
            raise ValueError("timeout_per_module must be positive")
        if self._startup_state in {"stopped", "stopping", "failed"}:
            return
        if self._startup_state == "starting":
            raise SystemStartupError("cannot stop SystemHandle while startup is in progress")

        self._startup_state = "stopping"
        hung_modules = self._stop_modules(
            list(reversed(self._startup_order)),
            timeout_per_module=timeout_per_module,
        )
        self._close_transport()
        if hung_modules:
            logger.warning("Hung modules during shutdown: %s", hung_modules)
        self._modules.clear()
        self._connections.clear()
        self._connection_metadata.clear()
        self._started = False
        self._startup_state = "stopped"
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

    @property
    def startup_state(self) -> str:
        """Return the current startup lifecycle state."""

        return self._startup_state

    @property
    def failed_modules(self) -> dict[str, str]:
        """Return all optional and critical startup failures."""

        return dict(self._failed_modules)

    @property
    def critical_failures(self) -> dict[str, str]:
        """Return failures that tripped the critical startup barrier."""

        return dict(self._critical_failures)

    # -- health -------------------------------------------------------------

    @property
    def critical_modules(self) -> tuple[str, ...]:
        """Return Module aliases governed by the startup barrier."""

        return self._critical_modules

    def health(self) -> dict[str, Any]:
        """Return a health summary dict."""
        total_in = sum(p.msg_count for m in self._modules.values() for p in m.ports_in.values())
        total_out = sum(p.msg_count for m in self._modules.values() for p in m.ports_out.values())
        return {
            "started": self._started,
            "startup_state": self._startup_state,
            "critical_modules": list(self._critical_modules),
            "critical_failures": dict(self._critical_failures),
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
# autoconnect()
# ---------------------------------------------------------------------------


def autoconnect(*blueprints: Blueprint) -> Blueprint:
    """Merge multiple Blueprints and enable auto_wire.

    Convenience factory for the composable-stack pattern::

        system = autoconnect(
            driver("stub"),
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
