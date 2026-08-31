"""lingtu.runtime.module 鈥?dimos-style Module base class with automatic In/Out port scanning.

Usage::

    class PerceptionModule(Module):
        scene_graph: Out[SceneGraph]
        detections: Out[Detection3D]
        image: In[Image]

        def setup(self):
            self.image.subscribe(self._on_image)

        def _on_image(self, img: Image):
            # process image 鈫?publish scene graph
            self.scene_graph.publish(sg)

Ports are discovered and instantiated from type hints. Module.__init__ scans all
In[T] / Out[T] annotations, creates port objects, and registers them in ports_in / ports_out.

Lifecycle: __init__ 鈫?setup() 鈫?start() 鈫?[running] 鈫?stop()
"""

from __future__ import annotations

import logging
import sys
from typing import TYPE_CHECKING, Any, Literal, Union, get_args, get_origin, get_type_hints

from .stream import In, Out

if TYPE_CHECKING:
    from .blueprint import Blueprint

logger = logging.getLogger(__name__)


def rpc(fn):
    """Mark a Module method as RPC-callable.

    Decorated methods can be discovered via Module.rpcs property
    and called across module boundaries.

    Usage:
        class MyModule(Module):
            @rpc
            def navigate_to(self, x: float, y: float) -> bool:
                ...
    """
    fn.__rpc__ = True
    return fn


def skill(fn):
    """Mark a Module method as AI-callable skill.

    A skill is an @rpc method additionally exposed to AI agents (MCP server, Agent module).
    MCPServerModule auto-discovers all @skill methods at build time via on_system_modules().

    Usage:
        class Navigation(Module):
            @skill
            def navigate_to(self, x: float, y: float) -> str:
                \"\"\"Navigate to coordinates (x, y) in map frame.\"\"\"
                ...
    """
    fn.__rpc__ = True
    fn.__skill__ = True
    return fn


import inspect as _inspect
import json as _json
import types as _types
from dataclasses import dataclass as _dataclass


@_dataclass
class SkillInfo:
    """Metadata for a single @skill method, used by MCPServerModule for tool discovery."""

    func_name: str
    class_name: str
    args_schema: str  # JSON string of MCP-compatible inputSchema


def _build_skill_schema(method) -> dict:
    """Build MCP inputSchema dict from a bound @skill method's signature + docstring."""
    sig = _inspect.signature(method)
    doc = _inspect.getdoc(method) or ""
    try:
        resolved_hints = get_type_hints(method)
    except (NameError, TypeError):
        resolved_hints = {}

    _PY_TO_JSON = {
        int: "integer",
        float: "number",
        str: "string",
        bool: "boolean",
        type(None): "null",
    }

    properties: dict[str, Any] = {}
    required: list[str] = []

    def json_schema(ann: Any) -> dict[str, Any]:
        origin = get_origin(ann)
        args = get_args(ann)
        if origin in (_types.UnionType, Union):
            non_null = [arg for arg in args if arg is not type(None)]
            if len(non_null) == 1:
                return json_schema(non_null[0])
            return {"anyOf": [json_schema(arg) for arg in args]}
        if origin is Literal:
            values = list(args)
            value_type = type(values[0]) if values else str
            return {"type": _PY_TO_JSON.get(value_type, "string"), "enum": values}
        if origin in (list, set, frozenset):
            item_type = args[0] if args else Any
            return {"type": "array", "items": json_schema(item_type)}
        if origin is tuple:
            if len(args) == 2 and args[1] is Ellipsis:
                return {"type": "array", "items": json_schema(args[0])}
            return {
                "type": "array",
                "prefixItems": [json_schema(arg) for arg in args],
                "minItems": len(args),
                "maxItems": len(args),
            }
        if origin is dict:
            value_type = args[1] if len(args) > 1 else Any
            return {
                "type": "object",
                "additionalProperties": json_schema(value_type),
            }
        if ann in (Any, _inspect.Parameter.empty):
            return {}
        return {"type": _PY_TO_JSON.get(ann, "string")}

    for pname, param in sig.parameters.items():
        if pname == "self":
            continue
        ann = resolved_hints.get(pname, param.annotation)
        prop = json_schema(ann)
        if param.default is not _inspect.Parameter.empty:
            prop["default"] = param.default
        else:
            required.append(pname)
        properties[pname] = prop

    schema: dict[str, Any] = {
        "type": "object",
        "description": doc,
        "properties": properties,
    }
    if required:
        schema["required"] = required
    return schema


class Module:
    """Module base class 鈥?core building block of the LingTu orchestration framework.

    Subclasses declare ``In[T]`` / ``Out[T]`` annotations; the base class scans and instantiates ports.
    Optional layer tag for dependency-graph level checks.

    Attributes:
        _config: config dict passed at construction
        _ports_in: input port registry {name: In[T]}
        _ports_out: output port registry {name: Out[T]}
        _running: whether the module is running
        _layer: layer (L0鈥揕6) for dependency validation
    """

    # class-level layer tag; subclasses may override
    _layer: int | None = None

    # Optional soft dependencies - Blueprint warns if missing at startup, but
    # the module still starts and related features degrade gracefully.
    SOFT_DEPENDS: list[str] = []

    # -- dimos-style __init_subclass__: set None placeholders at class definition ---

    def __init_subclass__(cls, layer: int | None = None, **kwargs: Any) -> None:
        """Subclass hook 鈥?set class-level In/Out placeholder attributes.

        dimos needs this because Dask Actor proxies look up attributes on the class.
        We keep this pattern so hasattr() finds ports at class level.
        """
        super().__init_subclass__(**kwargs)

        if layer is not None:
            cls._layer = layer

        # merge namespaces from MRO modules so forward refs resolve
        globalns: dict[str, Any] = {}
        for c in reversed(cls.__mro__):
            mod = sys.modules.get(c.__module__)
            if mod:
                globalns.update(mod.__dict__)

        try:
            hints = get_type_hints(cls, globalns=globalns, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin in (In, Out):
                if not hasattr(cls, name) or getattr(cls, name) is None:
                    setattr(cls, name, None)

    # -- instance initialisation ---------------------------------------------

    def __init__(self, **config: Any) -> None:
        self._config = config
        self._ports_in: dict[str, In[Any]] = {}
        self._ports_out: dict[str, Out[Any]] = {}
        self._running = False
        self._closed = False
        self._closed_lock = __import__("threading").Lock()

        # scan type hints and instantiate ports
        globalns: dict[str, Any] = {}
        for c in reversed(type(self).__mro__):
            mod = sys.modules.get(c.__module__)
            if mod:
                globalns.update(mod.__dict__)

        try:
            hints = get_type_hints(type(self), globalns=globalns, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin is Out:
                inner = get_args(ann)[0] if get_args(ann) else Any
                port = Out(inner, name, self)
                setattr(self, name, port)
                self._ports_out[name] = port
            elif origin is In:
                inner = get_args(ann)[0] if get_args(ann) else Any
                port = In(inner, name, self)
                setattr(self, name, port)
                self._ports_in[name] = port

    # -- lifecycle -----------------------------------------------------------

    def preflight(self) -> str | None:
        """Pre-startup check 鈥?verify required resources are available.

        Called before setup(). Return ``None`` if all checks pass, or a string
        describing what is missing/wrong.  Returning a non-None value causes
        the module to be marked as *failed* in SystemHandle.start() (same
        treatment as a setup() exception).

        Override in subclasses that need pre-flight validation::

            def preflight(self) -> Optional[str]:
                if not os.path.exists(self._model_path):
                    return f"Model file not found: {self._model_path}"
                return None
        """
        return None

    def startup_readiness(self) -> str | None:
        """Return why startup is not ready, or None when ready."""

        if self._running:
            return None
        return "not_running"

    def setup(self) -> None:
        """Configuration phase 鈥?register subscribers, load resources. Override in subclasses."""
        pass

    def start(self) -> None:
        """Start the module."""
        self._running = True
        logger.debug("Module %s started", type(self).__name__)

    def stop(self) -> None:
        """Stop module. Idempotent 鈥?safe to call multiple times.

        Breaks reference cycles (Module 鈫?Out 鈫?callback 鈫?Module) so the
        garbage collector can reclaim the entire module graph after shutdown.
        """
        with self._closed_lock:
            if self._closed:
                return
            self._closed = True
        self._running = False
        self._cleanup_refs()
        logger.debug("Module %s stopped", type(self).__name__)

    def _cleanup_refs(self) -> None:
        """Break reference cycles by clearing all port callbacks and transports."""
        for port in self._ports_out.values():
            port._clear_callbacks()
        for port in self._ports_in.values():
            port._clear_subscriber()

    # -- port access ---------------------------------------------------------

    @property
    def ports_in(self) -> dict[str, In[Any]]:
        """Read-only copy of the input port dict."""
        return dict(self._ports_in)

    @property
    def ports_out(self) -> dict[str, Out[Any]]:
        """Read-only copy of the output port dict."""
        return dict(self._ports_out)

    @property
    def all_ports(self) -> dict[str, Any]:
        """All ports (In + Out)."""
        return {**self._ports_in, **self._ports_out}

    @property
    def rpcs(self) -> dict[str, Any]:
        """Discover all @rpc-decorated methods on this module.

        Scans the class MRO __dict__ (not dir(self)) to avoid triggering
        property getters (like skills/rpcs themselves) and prevent cross-
        recursive infinite loops between the two scanning properties.
        """
        result = {}
        for cls in type(self).__mro__:
            for name, val in cls.__dict__.items():
                if name.startswith("_") or name in result:
                    continue
                if callable(val) and getattr(val, "__rpc__", False):
                    result[name] = getattr(self, name)
        return result

    @property
    def skills(self) -> dict[str, Any]:
        """Discover all @skill-decorated methods (AI-callable subset of @rpc).

        Scans the class MRO __dict__ to avoid infinite cross-recursion with rpcs.
        """
        result = {}
        for cls in type(self).__mro__:
            for name, val in cls.__dict__.items():
                if name.startswith("_") or name in result:
                    continue
                if callable(val) and getattr(val, "__skill__", False):
                    result[name] = getattr(self, name)
        return result

    def get_skill_infos(self) -> list[SkillInfo]:
        """Return SkillInfo list for all @skill methods on this module.

        Used by MCPServerModule.on_system_modules() for dynamic tool discovery.
        """
        infos = []
        for name, method in self.skills.items():
            schema = _build_skill_schema(method)
            infos.append(
                SkillInfo(
                    func_name=name,
                    class_name=type(self).__name__,
                    args_schema=_json.dumps(schema),
                )
            )
        return infos

    def call_rpc(self, method_name: str, **kwargs: Any) -> Any:
        """Call an RPC method by name. Returns the method result."""
        rpcs = self.rpcs
        if method_name not in rpcs:
            raise ValueError(f"RPC method '{method_name}' not found on {type(self).__name__}")
        return rpcs[method_name](**kwargs)

    @property
    def running(self) -> bool:
        """Return whether the Module lifecycle is running."""

        return self._running

    @property
    def layer(self) -> int | None:
        """Return the declared architecture layer."""

        return self._layer

    # -- System-level hooks -----------------------------------------------

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        """Called by Blueprint after all modules are built.

        Override to inspect other modules, discover @rpc methods,
        or establish cross-module dependencies.

        Args:
            modules: dict of {module_name: module_instance}
        """
        pass

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        """Conservative runtime switch contract.

        Modules opt in by overriding this method. The default is intentionally
        fail-closed because most robot backends are not safe to swap in place.
        """
        return {
            "ok": False,
            "category": category,
            "requested_backend": backend,
            "reason": "backend_reconfigure_unsupported",
        }

    # -- Dynamic port creation --------------------------------------------

    def io(self, name: str, direction: type, msg_type: type | None = None) -> Any:
        """Dynamically create a port at runtime.

        Args:
            name: port name
            direction: In or Out class
            msg_type: message type (optional, defaults to Any)

        Returns:
            The created port instance

        Usage::

            self.io("custom_output", Out, PoseStamped)
            self.io("custom_input", In, Odometry)
        """
        resolved_type = msg_type if msg_type is not None else Any

        if direction is Out or (isinstance(direction, type) and issubclass(direction, Out)):
            port = Out(resolved_type, name, self)
            self._ports_out[name] = port
            setattr(self, name, port)
            return port
        elif direction is In or (isinstance(direction, type) and issubclass(direction, In)):
            port = In(resolved_type, name, self)
            self._ports_in[name] = port
            setattr(self, name, port)
            return port
        else:
            raise ValueError(f"direction must be In or Out, got {direction}")

    # -- Blueprint factory ---------------------------------------------------

    @classmethod
    def blueprint(cls, **kwargs: Any) -> Blueprint:
        """Create a Blueprint containing only this module."""
        from .blueprint import Blueprint

        return Blueprint().add(cls, **kwargs)

    # -- diagnostics ---------------------------------------------------------

    def port_summary(self) -> dict[str, Any]:
        """Port summary for debugging and health checks."""
        return {
            "module": type(self).__name__,
            "layer": self._layer,
            "running": self._running,
            "ports_in": {
                name: {
                    "type": port.msg_type.__name__,
                    "msg_count": port.msg_count,
                    "connected": port.connected,
                    "rate_hz": round(port.rate_hz, 1),
                    "drop_count": port.drop_count,
                    "deliver_count": port.deliver_count,
                    "callback_errors": port.callback_errors,
                    "avg_callback_ms": round(port.avg_callback_ms, 2),
                    "max_callback_ms": round(port.max_callback_ms, 2),
                    "latency": port.latency_percentiles(),
                    "stale_ms": round(port.stale_ms, 1),
                }
                for name, port in self._ports_in.items()
            },
            "ports_out": {
                name: {
                    "type": port.msg_type.__name__,
                    "msg_count": port.msg_count,
                    "callbacks": port.callback_count,
                    "rate_hz": round(port.rate_hz, 1),
                    "publish_errors": port.publish_errors,
                    "stale_ms": round(port.stale_ms, 1),
                }
                for name, port in self._ports_out.items()
            },
        }

    def __repr__(self) -> str:
        layer_str = f", L{self._layer}" if self._layer is not None else ""
        return f"{type(self).__name__}(in={list(self._ports_in.keys())}, out={list(self._ports_out.keys())}{layer_str})"
