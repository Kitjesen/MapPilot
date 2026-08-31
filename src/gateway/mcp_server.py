"""LingTu MCP Server -Model Context Protocol for AI agent control.

Exposes robot capabilities as MCP tools so Claude/GPT can control the robot.
Standard MCP JSON-RPC 2.0 over HTTP POST /mcp.

Tools are auto-discovered from @skill methods on all system modules at
build() time via on_system_modules().  No hardcoded tool list -every
@skill automatically becomes an MCP tool with a JSON Schema derived from
the method signature and docstring.

Gateway usage::

    claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp

Module blueprint usage::

    bp.add(MCPServerModule, port=8090)
"""

from __future__ import annotations

import inspect
import json
import logging
import threading
import uuid
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import InstructionRequest
from gateway.services.command_boundary import CommandBoundaryError
from gateway.services.control_commands import ControlCommandService
from gateway.services.module_refs import navigation_state as resolve_navigation_state
from gateway.services.native_control import estop as native_estop
from runtime.module import Module, skill
from runtime.msgs.nav import NavigationGoalStatus, NavigationState, Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.registry import register
from runtime.status_provider import RuntimeStatusProvider
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

MCP_PROTOCOL_VERSION = "2025-11-25"
_BUILTIN_FALLBACK_TOOLS = frozenset({"emergency_stop", "send_instruction"})


# ---------------------------------------------------------------------------
# JSON-RPC 2.0 helpers
# ---------------------------------------------------------------------------


def _ok(req_id: Any, result: Any) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def _text(req_id: Any, value: Any) -> dict:
    text = json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else str(value)
    return _ok(req_id, {"content": [{"type": "text", "text": text}]})


def _tool_error(req_id: Any, text: str) -> dict:
    return _ok(req_id, {"content": [{"type": "text", "text": str(text)}], "isError": True})


def _error(req_id: Any, code: int, msg: str) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": msg}}


# ---------------------------------------------------------------------------
# MCPServerModule
# ---------------------------------------------------------------------------


@register("mcp", "server", description="MCP server for AI agent robot control")
class MCPServerModule(Module, layer=6):
    """MCP Server -exposes every robot @skill as an AI-callable tool.

    Tools are discovered at build() time via on_system_modules().
    No static list -modules register their own capabilities through @skill.

    Built-in system tools (get_health, list_modules, get_config) are
    implemented as @skill methods on this module so they appear automatically.
    """

    # -- receive telemetry for read-only queries ----------------------------
    odometry: In[Odometry]
    scene_graph: In[SceneGraph]
    navigation_state: In[NavigationState]
    navigation_goal_status: In[NavigationGoalStatus]

    # -- outgoing commands --------------------------------------------------
    instruction: Out[str]
    mode_cmd: Out[str]

    def __init__(
        self,
        port: int = 8090,
        host: str = "0.0.0.0",
        require_api_key: bool | None = None,
        **kw,
    ):
        super().__init__(**kw)
        self._port = port
        self._host = host
        self._require_api_key = require_api_key
        self._server_thread: threading.Thread | None = None
        self._server: Any | None = None
        self._stop_event = threading.Event()
        self._server_error: str | None = None
        self._lifecycle_lock = threading.RLock()
        self._stopping = False

        # Cached telemetry (written by subscriptions)
        self._odom: dict | None = None
        self._sg_json: str = "{}"
        self._navigation_state: dict | None = None
        self._navigation_goal_status_by_request: dict[str, dict[str, Any]] = {}

        # Injected by the Host after module startup.
        self._system_handle = None
        self._runtime_status_provider: RuntimeStatusProvider | None = None

        # Populated by on_system_modules() -all @skill across all modules
        self._tool_registry: dict[str, Any] = {}  # func_name ->bound method
        self._tool_list: list[dict] = []  # MCP tool descriptors
        self._all_modules: dict[str, Any] = {}

        # Memory / perception module references (for built-in query tools)
        self._tagged_locations_mod = None
        self._vector_memory_mod = None
        self._episodic_mod = None
        self._nav_commands = None

    # -- lifecycle ----------------------------------------------------------

    def set_system_handle(self, handle: Any) -> None:
        """Inject SystemHandle so get_health / list_modules work."""
        self._system_handle = handle

    def set_runtime_status_provider(self, provider: RuntimeStatusProvider) -> None:
        """Inject read-only runtime diagnostics without SystemHandle ownership."""
        self._runtime_status_provider = provider

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        """Auto-discover every @skill method from every running module.

        Called by Blueprint.build() after all modules are instantiated.
        Builds _tool_list (MCP descriptors) and _tool_registry (call table).
        """
        self._tool_registry = {}
        self._tool_list = []
        candidates: dict[str, dict[int, dict[str, Any]]] = {}
        self._all_modules = modules
        self._nav_commands = modules.get("nav.commands")
        # Grab module references for built-in tools
        self._tagged_locations_mod = modules.get("TaggedLocationsModule")
        self._vector_memory_mod = modules.get("VectorMemoryModule")
        self._episodic_mod = modules.get("EpisodicMemoryModule")

        # Discover @skill from every module (including self)
        for mod_name, mod in sorted(modules.items()):
            if not hasattr(mod, "get_skill_infos"):
                continue
            try:
                infos = mod.get_skill_infos()
            except Exception as e:
                logger.debug("failed to get skill infos from %s: %s", mod_name, e)
                continue
            for info in infos:
                method = getattr(mod, info.func_name, None)
                if method is None:
                    continue
                schema = json.loads(info.args_schema)
                desc = schema.pop("description", "")
                by_module = candidates.setdefault(info.func_name, {})
                existing = by_module.get(id(mod))
                if existing is not None:
                    existing["module_names"].add(mod_name)
                    continue
                by_module[id(mod)] = {
                    "module": mod,
                    "module_names": {mod_name},
                    "method": method,
                    "tool": {
                        "name": info.func_name,
                        "description": f"[{info.class_name}] {desc}".strip(),
                        "inputSchema": schema,
                    },
                }

        tool_registry: dict[str, Any] = {}
        tool_list: list[dict[str, Any]] = []
        for tool_name in sorted(candidates):
            entries = list(candidates[tool_name].values())
            peer_entries = [entry for entry in entries if entry["module"] is not self]
            self_entries = [entry for entry in entries if entry["module"] is self]

            if len(entries) == 1:
                selected = entries[0]
            elif tool_name in _BUILTIN_FALLBACK_TOOLS and len(peer_entries) == 1 and len(self_entries) == 1:
                selected = peer_entries[0]
            else:
                owners = sorted("/".join(sorted(entry["module_names"])) for entry in entries)
                raise ValueError(f"duplicate MCP tool '{tool_name}' from modules: {', '.join(owners)}")

            tool_registry[tool_name] = selected["method"]
            tool_list.append(selected["tool"])

        self._tool_registry = tool_registry
        self._tool_list = tool_list

        logger.info(
            "MCP: %d tools from %d modules",
            len(self._tool_list),
            len(modules),
        )

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_sg)
        self.navigation_state.subscribe(self._on_navigation_state)
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                raise RuntimeError("MCPServerModule cannot be restarted after stop")
            if self._stopping:
                raise RuntimeError("MCPServerModule cannot be started while stopping")

            thread = self._server_thread
            if thread is not None and thread.is_alive():
                logger.debug("MCP server thread already running")
                return

            self._stop_event.clear()
            self._server_error = None
            stop_event = self._stop_event
            thread = threading.Thread(
                target=self._run_server,
                args=(stop_event,),
                daemon=True,
                name="mcp-server",
            )
            self._server_thread = thread
            try:
                thread.start()
            except Exception:
                self._server_thread = None
                raise
            super().start()

        logger.info("MCP server thread starting on %s:%d", self._host, self._port)

    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stopping = True
            self._stop_event.set()
            server = self._server
            thread = self._server_thread

        try:
            if server is not None:
                try:
                    server.should_exit = True
                except Exception:
                    logger.debug("failed to signal MCP uvicorn shutdown", exc_info=True)

            current = threading.current_thread()
            if thread is not None and thread is not current and thread.is_alive():
                thread.join(timeout=2.0)

            with self._lifecycle_lock:
                if self._server_thread is thread:
                    if thread is None or not thread.is_alive():
                        self._server_thread = None
                    elif thread is not current:
                        logger.warning("MCP server thread did not stop within timeout")
        finally:
            try:
                super().stop()
            finally:
                with self._lifecycle_lock:
                    self._stopping = False

    # -- subscription callbacks --------------------------------------------

    def _on_odom(self, odom: Odometry) -> None:
        self._odom = {
            "x": odom.x,
            "y": odom.y,
            "z": getattr(odom, "z", 0.0),
            "yaw": odom.yaw,
            "vx": odom.vx,
            "vy": odom.vy,
            "ts": odom.ts,
        }

    def _on_sg(self, sg: SceneGraph) -> None:
        self._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)

    def _on_navigation_state(self, state: NavigationState) -> None:
        self._navigation_state = state.to_dict()

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        self._navigation_goal_status_by_request[status.request_id] = status.to_dict()
        if len(self._navigation_goal_status_by_request) > 256:
            oldest = next(iter(self._navigation_goal_status_by_request))
            self._navigation_goal_status_by_request.pop(oldest, None)

    # -- built-in @skill tools (system + perception read) ------------------

    @skill
    def get_health(self) -> str:
        """Return full system health: modules, connections, message counts."""
        if self._runtime_status_provider is not None:
            return json.dumps(self._runtime_status_provider.health(), default=str)
        if self._system_handle is None:
            return json.dumps({"error": "system handle not yet injected"})
        return json.dumps(self._system_handle.health(), default=str)

    @skill
    def list_modules(self) -> str:
        """List all deployed modules with layer, port counts, and running state."""
        if self._runtime_status_provider is not None:
            module_items = self._runtime_status_provider.modules.items()
        elif self._system_handle is not None:
            module_items = self._system_handle.modules.items()
        else:
            return json.dumps({"error": "system handle not yet injected"})
        modules = {}
        for n, m in module_items:
            modules[n] = {
                "layer": getattr(m, "layer", getattr(type(m), "_layer", None)),
                "running": bool(getattr(m, "running", False)),
                "ports_in": list(getattr(m, "ports_in", {}).keys()),
                "ports_out": list(getattr(m, "ports_out", {}).keys()),
            }
        return json.dumps({"modules": modules})

    @skill
    def get_config(self) -> str:
        """Return robot configuration: speed limits, geometry, safety thresholds."""
        try:
            from runtime.config import get_config

            cfg = get_config()
            return json.dumps(
                {
                    "speed": {
                        "max_linear": cfg.speed.max_linear,
                        "max_angular": cfg.speed.max_angular,
                        "max_speed": cfg.speed.max_speed,
                    },
                    "geometry": {
                        "height": cfg.geometry.vehicle_height,
                        "width": cfg.geometry.vehicle_width,
                        "length": cfg.geometry.vehicle_length,
                    },
                    "safety": {
                        "stop_distance": cfg.safety.stop_distance,
                        "tilt_limit_deg": cfg.safety.tilt_limit_deg,
                        "obstacle_height_thre": cfg.safety.obstacle_height_thre,
                    },
                }
            )
        except Exception as exc:
            return json.dumps({"error": str(exc)})

    @skill
    def get_robot_position(self) -> str:
        """Return the robot's current position (x, y, z) and yaw in map frame."""
        return json.dumps(self._odom or {"error": "no odometry yet"})

    @skill
    def get_scene_graph(self) -> str:
        """Return the current scene graph -detected objects with positions and labels."""
        return self._sg_json

    @skill
    def detect_objects(self, query: str) -> str:
        """Search the current scene for objects matching *query* (case-insensitive)."""
        q = query.lower()
        try:
            sg = json.loads(self._sg_json)
            matches = [o for o in sg.get("objects", []) if q in o.get("label", "").lower()]
            return json.dumps({"query": q, "matches": matches, "count": len(matches)})
        except Exception as e:
            logger.warning("scene graph JSON parse failed: %s", e)
            return json.dumps({"query": q, "matches": [], "count": 0})

    @skill
    def query_memory(self, query: str) -> str:
        """Search episodic, spatial, and vector memory for past observations."""
        results = []

        vm = self._vector_memory_mod
        if vm and hasattr(vm, "query_location"):
            try:
                r = vm.query_location(query)
                vector_navigable = (
                    r.get("navigable") is True
                    and r.get("semantic_encoder_ready") is True
                    and r.get("degraded") is False
                )
                if vector_navigable:
                    for hit in r.get("results", [])[:3]:
                        if hit.get("navigable") is not True:
                            continue
                        results.append(
                            {
                                "source": "vector",
                                "position": [hit.get("x", 0), hit.get("y", 0)],
                                "score": hit.get("score", 0),
                                "labels": hit.get("labels", ""),
                                "navigable": True,
                            }
                        )
                elif r.get("found"):
                    best = r.get("best") or {}
                    results.append(
                        {
                            "source": "vector",
                            "query_only": True,
                            "navigable": False,
                            "reason": "vector_memory_not_safe_for_navigation",
                            "encoder_type": r.get("encoder_type", "unknown"),
                            "labels": best.get("labels", ""),
                            "score": best.get("score", 0),
                        }
                    )
            except Exception as e:
                logger.debug("vector memory query failed: %s", e)

        em = self._episodic_mod
        if em and hasattr(em, "memory"):
            try:
                for r in em.memory.query_by_text(query, top_k=3):
                    results.append(
                        {
                            "source": "episodic",
                            "label": getattr(r, "label", ""),
                            "position": list(getattr(r, "position", [0, 0, 0])),
                            "ts": getattr(r, "timestamp", 0),
                        }
                    )
            except Exception as e:
                logger.debug("episodic memory query failed: %s", e)

        tl = self._tagged_locations_mod
        if tl and hasattr(tl, "store"):
            try:
                match = tl.store.query_fuzzy(query)
                if match:
                    results.append(
                        {
                            "source": "tagged",
                            "label": match["name"],
                            "position": match["position"],
                        }
                    )
            except Exception as e:
                logger.debug("tagged location query failed: %s", e)

        return json.dumps({"query": query, "results": results, "count": len(results)})

    @skill
    def list_tagged_locations(self) -> str:
        """List all named locations tagged by the robot or user."""
        tl = self._tagged_locations_mod
        locs = tl.store.list_all() if (tl and hasattr(tl, "store")) else []
        return json.dumps({"locations": locs})

    @skill
    def tag_location(self, name: str) -> str:
        """Save the robot's current position under *name* for future navigation."""
        if not self._odom:
            return json.dumps({"error": "no odometry -cannot tag location"})
        tl = self._tagged_locations_mod
        if not (tl and hasattr(tl, "store")):
            return json.dumps({"error": "TaggedLocationsModule not running"})
        tl.store.tag(name, x=self._odom["x"], y=self._odom["y"], z=self._odom.get("z", 0))
        entry = tl.store.query(name)
        return json.dumps({"tagged": name, "position": entry})

    @skill
    def navigate_to_object(self, instruction: str) -> str:
        """Navigate to a described object or place using semantic understanding."""
        return self._submit_instruction_tool("navigate_to_object", instruction)

    @skill
    def send_instruction(self, text: str) -> str:
        """Send a natural language instruction to the semantic planner."""
        return self._submit_instruction_tool("send_instruction", text)

    def _submit_instruction_tool(self, tool_name: str, text: str) -> str:
        instruction = str(text or "").strip()
        if not instruction:
            return json.dumps(
                {
                    "ok": False,
                    "accepted": False,
                    "error": "invalid_instruction",
                    "message": "instruction text must not be blank",
                    "execution_confirmed": False,
                    "motor_confirmed": False,
                }
            )

        request_id = f"mcp-{tool_name}-{uuid.uuid4().hex}"
        body = InstructionRequest(
            text=instruction,
            request_id=request_id,
            client_id="mcp",
        )
        gateway = (self._all_modules or {}).get("GatewayModule")

        if gateway is None:
            self.instruction.publish(instruction)
            return json.dumps(
                {
                    "ok": True,
                    "accepted": True,
                    "status": "submitted",
                    "stage": "host_submitted",
                    "tool": tool_name,
                    "request_id": request_id,
                    "client_id": "mcp",
                    "instruction": instruction,
                    "execution_confirmed": False,
                    "motor_confirmed": False,
                }
            )

        command_service = ControlCommandService(gateway)

        def _submit() -> dict[str, Any]:
            gateway.instruction.publish(instruction)
            return {
                "ok": True,
                "accepted": True,
                "status": "submitted",
                "stage": "submitted",
                "tool": tool_name,
                "instruction": instruction,
                "execution_confirmed": False,
                "motor_confirmed": False,
            }

        result = command_service.run_motion_guarded_command(
            "instruction",
            body,
            _submit,
        )
        if isinstance(result, JSONResponse):
            payload = json.loads(result.body.decode("utf-8"))
            payload.setdefault("execution_confirmed", False)
            payload.setdefault("motor_confirmed", False)
            payload.setdefault("tool", tool_name)
            return json.dumps(payload)
        receipt = dict(result)
        command = receipt.get("command")
        command_accepted = isinstance(command, dict) and command.get("accepted") is True
        accepted = receipt.get("ok") is True and command_accepted
        receipt["ok"] = accepted
        receipt["accepted"] = accepted
        if not accepted:
            receipt.setdefault("error", "invalid_command_ack")
            receipt.setdefault("message", "Gateway returned an invalid instruction acknowledgement.")
            receipt["status"] = "rejected"
        receipt.setdefault("execution_confirmed", False)
        receipt.setdefault("motor_confirmed", False)
        if accepted:
            receipt.setdefault("status", "submitted")
        return json.dumps(receipt)

    @skill
    def emergency_stop(self) -> str:
        """Emergency stop -immediately halts all robot motion."""
        return self._publish_estop("emergency_stopped")

    def _publish_estop(self, status: str) -> str:
        wrote_native = native_estop(self, "mcp_emergency_stop")
        if not wrote_native:
            raise CommandBoundaryError("native emergency-stop boundary is unavailable")
        return json.dumps(
            {
                "ok": True,
                "accepted": True,
                "status": status,
                "control_boundary": "native_estop",
                "stage": "native_command_ack",
                "motor_confirmed": False,
            }
        )

    @skill
    def set_mode(self, mode: str) -> str:
        """Set robot operating mode: manual | autonomous | estop."""
        if mode not in ("manual", "autonomous", "estop"):
            return json.dumps({"error": f"invalid mode: {mode!r}"})
        stage = "local_mode_published"
        if mode == "estop":
            wrote_native = native_estop(self, "mcp_mode_estop")
            if not wrote_native:
                raise CommandBoundaryError("native emergency-stop boundary is unavailable")
            stage = "native_command_ack"
        self.mode_cmd.publish(mode)
        return json.dumps(
            {
                "ok": True,
                "accepted": True,
                "mode": mode,
                "stage": stage,
                "motor_confirmed": False,
            }
        )

    # -- FastAPI + MCP JSON-RPC endpoint -----------------------------------

    def _run_server(
        self,
        stop_event: threading.Event | None = None,
    ) -> bool:
        stop_event = stop_event if stop_event is not None else self._stop_event
        server = None
        current = threading.current_thread()
        self._server_error = None
        try:
            import os

            import uvicorn
            from fastapi import FastAPI
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse
        except ImportError:
            self._server_error = "FastAPI or uvicorn is not installed"
            with self._lifecycle_lock:
                if self._server_thread is current:
                    self._server_thread = None
            logger.error("FastAPI not installed -run: pip install fastapi uvicorn")
            return False

        cors_origins = os.environ.get(
            "LINGTU_CORS_ORIGINS",
            "http://localhost:5050,http://127.0.0.1:5050",
        ).split(",")
        app = FastAPI(title="LingTu MCP Server")
        app.add_middleware(CORSMiddleware, allow_origins=cors_origins, allow_methods=["*"], allow_headers=["*"])
        from gateway.auth import APIKeyMiddleware

        require_key = (
            self._require_api_key
            if self._require_api_key is not None
            else self._host not in {"127.0.0.1", "localhost", "::1"}
        )
        app.add_middleware(APIKeyMiddleware, require_key=require_key)
        mcp = self

        @app.post("/mcp")
        async def mcp_endpoint(request: dict):
            method = request.get("method", "")
            params = request.get("params") or {}
            req_id = request.get("id")

            if req_id is None:
                return JSONResponse(status_code=204, content=None)

            if method == "initialize":
                return JSONResponse(
                    _ok(
                        req_id,
                        {
                            "protocolVersion": MCP_PROTOCOL_VERSION,
                            "capabilities": {"tools": {}},
                            "serverInfo": {"name": "lingtu", "version": "2.0.0"},
                        },
                    )
                )

            if method == "notifications/initialized":
                return JSONResponse(status_code=204, content=None)

            if method == "tools/list":
                return JSONResponse(_ok(req_id, {"tools": mcp._tool_list}))

            if method == "tools/call":
                if not isinstance(params, dict):
                    return JSONResponse(_error(req_id, -32602, "tools/call params must be an object"))
                name = params.get("name", "")
                args = params.get("arguments", {})
                if not isinstance(name, str) or not name:
                    return JSONResponse(_error(req_id, -32602, "Tool name must be a non-empty string"))
                if not isinstance(args, dict):
                    return JSONResponse(_error(req_id, -32602, "Tool arguments must be an object"))
                fn = mcp._tool_registry.get(name)
                if fn is None:
                    return JSONResponse(_error(req_id, -32602, f"Unknown tool: {name!r}"))
                try:
                    inspect.signature(fn).bind(**args)
                except (TypeError, ValueError) as exc:
                    return JSONResponse(
                        _error(
                            req_id,
                            -32602,
                            f"Invalid arguments for tool {name!r}: {exc}",
                        )
                    )
                try:
                    import asyncio as _aio

                    # Run tool in thread with timeout to prevent blocking
                    tool_timeout_s = float(os.environ.get("LINGTU_MCP_TOOL_TIMEOUT_S", "10"))
                    result = await _aio.wait_for(
                        _aio.get_running_loop().run_in_executor(None, lambda: fn(**args)),
                        timeout=tool_timeout_s,
                    )
                    return JSONResponse(_text(req_id, result if result is not None else "Done."))
                except _aio.TimeoutError:
                    logger.warning("MCP tool timeout: %s (exceeded %ss)", name, tool_timeout_s)
                    return JSONResponse(_tool_error(req_id, f"Tool {name!r} timed out after {tool_timeout_s}s."))
                except Exception:
                    logger.exception("MCP tool error: %s", name)
                    return JSONResponse(_tool_error(req_id, f"Tool {name!r} failed."))

            return JSONResponse(_error(req_id, -32601, f"Unknown method: {method}"))

        @app.get("/health")
        async def health():
            return {
                "status": "ok",
                "tools": len(mcp._tool_list),
                "port": mcp._port,
                "has_handle": mcp._system_handle is not None,
            }

        @app.get("/capabilities")
        async def capabilities():
            """Capability negotiation endpoint for AI agents.

            Returns available tools grouped by category, along with
            system state that affects tool availability.
            """
            nav_state = resolve_navigation_state(mcp._navigation_state)
            tool_names = [t["name"] for t in mcp._tool_list]
            return {
                "schema_version": 1,
                "server": "lingtu-mcp",
                "protocol_version": MCP_PROTOCOL_VERSION,
                "tools_available": len(tool_names),
                "tool_names": sorted(tool_names),
                "navigation_state": nav_state or "UNKNOWN",
                "motion_tools_blocked": nav_state != "IDLE" and nav_state != "",
                "has_system_handle": mcp._system_handle is not None,
                "tool_timeout_s": float(os.environ.get("LINGTU_MCP_TOOL_TIMEOUT_S", "10")),
            }

        try:
            config = uvicorn.Config(
                app,
                host=self._host,
                port=self._port,
                log_level="warning",
            )
            server = uvicorn.Server(config)
            with self._lifecycle_lock:
                self._server = server
            if stop_event.is_set():
                server.should_exit = True
            server.run()

            if bool(getattr(server, "should_exit", False)) or bool(getattr(server, "force_exit", False)):
                return True

            message = "uvicorn returned without shutdown signal"
            self._server_error = message
            logger.error("MCP %s", message)
            return False
        except SystemExit as exc:
            self._server_error = f"SystemExit: {exc}"
            logger.error("MCP uvicorn exited during startup: %s", exc)
            return False
        except Exception as exc:
            self._server_error = f"{type(exc).__name__}: {exc}"
            logger.exception("MCP uvicorn crashed")
            return False
        finally:
            with self._lifecycle_lock:
                if server is not None and self._server is server:
                    self._server = None
                if self._server_thread is current:
                    self._server_thread = None

    # -- Module health summary ---------------------------------------------

    def health(self) -> dict[str, Any]:
        with self._lifecycle_lock:
            thread = self._server_thread
            server = self._server
            thread_alive = bool(thread is not None and thread.is_alive())
            server_started = bool(getattr(server, "started", False))

        info = super().port_summary()
        info["mcp"] = {
            "port": self._port,
            "tools": len(self._tool_list),
            "has_handle": self._system_handle is not None,
            "thread_alive": thread_alive,
            "server_started": server_started,
            "stop_requested": self._stop_event.is_set(),
            "stopping": self._stopping,
            "last_error": self._server_error,
        }
        return info
