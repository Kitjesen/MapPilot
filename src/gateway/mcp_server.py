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

import json
import logging
import threading
from typing import Any

from runtime.module import Module, skill
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SafetyState, SceneGraph
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

MCP_PROTOCOL_VERSION = "2025-11-25"
_MOTION_BACKEND_CATEGORIES = {
    "planner",
    "local_planner",
    "path_follower",
    "terrain",
    "slam",
}
_BACKEND_RECONFIGURE_TARGETS = {
    "detector": ("PerceptionModule",),
    "encoder": ("PerceptionModule",),
    "llm": ("LLMModule",),
    "llm_client": ("LLMModule",),
    "planner": ("nav.mission",),
    "local_planner": ("nav.local_planner",),
    "path_follower": ("nav.path_follower",),
    "terrain": ("nav.terrain",),
    "slam": ("SlamAdapterModule", "SlamModule", "SlamBridgeModule"),
}


def _navigation_state(nav: Any) -> str:
    if nav is None:
        return ""
    health: dict[str, Any] = {}
    if hasattr(nav, "health"):
        try:
            raw_health = nav.health() or {}
            if isinstance(raw_health, dict):
                health = raw_health
        except Exception:
            return "UNKNOWN"
    state = health.get("state")
    nested = health.get("navigation")
    if state is None and isinstance(nested, dict):
        state = nested.get("state")
    if hasattr(state, "value"):
        state = state.value
    return str(state or "").upper()


# ---------------------------------------------------------------------------
# JSON-RPC 2.0 helpers
# ---------------------------------------------------------------------------

def _ok(req_id: Any, result: Any) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}

def _text(req_id: Any, text: str) -> dict:
    return _ok(req_id, {"content": [{"type": "text", "text": str(text)}]})

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

    _run_in_main: bool = True

    # -- receive telemetry for read-only queries ----------------------------
    odometry:       In[Odometry]
    scene_graph:    In[SceneGraph]
    safety_state:   In[SafetyState]
    mission_status: In[dict]

    # -- outgoing commands --------------------------------------------------
    goal_pose:   Out[PoseStamped]
    cmd_vel:     Out[Twist]
    stop_cmd:    Out[int]
    instruction: Out[str]
    mode_cmd:    Out[str]

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

        # Cached telemetry (written by subscriptions)
        self._odom:    dict | None = None
        self._sg_json: str = "{}"
        self._safety:  dict | None = None
        self._mission: dict | None = None

        # Injected after system.start() by cli/main.py
        self._system_handle = None

        # Populated by on_system_modules() -all @skill across all modules
        self._tool_registry: dict[str, Any] = {}   # func_name ->bound method
        self._tool_list:     list[dict] = []        # MCP tool descriptors
        self._all_modules: dict[str, Any] = {}

        # Memory / perception module references (for built-in query tools)
        self._tagged_locations_mod = None
        self._vector_memory_mod    = None
        self._episodic_mod         = None
        self._navigation = None
        self._backend_reconfigure_modules: dict[str, Any] = {}

    # -- lifecycle ----------------------------------------------------------

    def set_system_handle(self, handle: Any) -> None:
        """Inject SystemHandle so get_health / list_modules work."""
        self._system_handle = handle

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        """Auto-discover every @skill method from every running module.

        Called by Blueprint.build() after all modules are instantiated.
        Builds _tool_list (MCP descriptors) and _tool_registry (call table).
        """
        self._tool_registry = {}
        self._tool_list = []
        self._all_modules = modules
        self._navigation = modules.get("nav.mission")
        self._backend_reconfigure_modules = {
            module_name: modules.get(module_name)
            for module_names in _BACKEND_RECONFIGURE_TARGETS.values()
            for module_name in module_names
            if modules.get(module_name) is not None
        }

        # Grab module references for built-in tools
        self._tagged_locations_mod = modules.get("TaggedLocationsModule")
        self._vector_memory_mod    = modules.get("VectorMemoryModule")
        self._episodic_mod         = modules.get("EpisodicMemoryModule")

        # Discover @skill from every module (including self)
        for mod_name, mod in modules.items():
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
                self._tool_registry[info.func_name] = method
                schema = json.loads(info.args_schema)
                desc = schema.pop("description", "")
                self._tool_list.append({
                    "name": info.func_name,
                    "description": f"[{info.class_name}] {desc}".strip(),
                    "inputSchema": schema,
                })

        # Deduplicate: if a module @skill has the same name as a built-in,
        # the module-native version takes priority (last write in _tool_registry wins).
        # Rebuild _tool_list to reflect the deduplicated registry.
        seen: dict = {}
        for tool in self._tool_list:
            seen[tool["name"]] = tool  # last one wins (matches _tool_registry)
        self._tool_list = list(seen.values())
        self._install_legacy_tool_aliases()

        logger.info(
            "MCP: %d tools from %d modules",
            len(self._tool_list), len(modules),
        )

    def _install_legacy_tool_aliases(self) -> None:
        """Keep old MCP clients working without exposing lifecycle stop()."""

        if "stop" in self._tool_registry:
            return
        if "emergency_stop" not in self._tool_registry:
            return

        self._tool_registry["stop"] = self._legacy_stop_tool
        source = next(
            (tool for tool in self._tool_list if tool["name"] == "emergency_stop"),
            None,
        )
        input_schema = (
            dict(source.get("inputSchema", {}))
            if source
            else {"type": "object", "properties": {}, "required": []}
        )
        self._tool_list.append({
            "name": "stop",
            "description": "[MCPServerModule] Legacy alias for emergency_stop.",
            "inputSchema": input_schema,
        })

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_sg)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)

    def start(self) -> None:
        super().start()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True, name="mcp-server"
        )
        self._server_thread.start()
        logger.info("MCP Server at http://%s:%d/mcp", self._host, self._port)

    def stop(self) -> None:
        self._server_thread = None
        super().stop()

    # -- subscription callbacks --------------------------------------------

    def _on_odom(self, odom: Odometry) -> None:
        self._odom = {
            "x": odom.x, "y": odom.y, "z": getattr(odom, "z", 0.0),
            "yaw": odom.yaw, "vx": odom.vx, "vy": odom.vy, "ts": odom.ts,
        }

    def _on_sg(self, sg: SceneGraph) -> None:
        self._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)

    def _on_safety(self, state: SafetyState) -> None:
        import time
        self._safety = {"level": getattr(state, "level", 0), "ts": time.time()}

    def _on_mission(self, status: dict) -> None:
        self._mission = status

    # -- built-in @skill tools (system + perception read) ------------------

    @skill
    def get_health(self) -> str:
        """Return full system health: modules, connections, message counts."""
        if self._system_handle is None:
            return json.dumps({"error": "system handle not yet injected"})
        return json.dumps(self._system_handle.health(), default=str)

    @skill
    def list_modules(self) -> str:
        """List all deployed modules with layer, port counts, and running state."""
        if self._system_handle is None:
            return json.dumps({"error": "system handle not yet injected"})
        modules = {}
        for n, m in self._system_handle.modules.items():
            modules[n] = {
                "layer":     m.layer,
                "running":   m.running,
                "ports_in":  list(m.ports_in.keys()),
                "ports_out": list(m.ports_out.keys()),
            }
        return json.dumps({"modules": modules})

    @skill
    def get_config(self) -> str:
        """Return robot configuration: speed limits, geometry, safety thresholds."""
        try:
            from runtime.config import get_config
            cfg = get_config()
            return json.dumps({
                "speed":    {"max_linear": cfg.speed.max_linear,
                             "max_angular": cfg.speed.max_angular,
                             "max_speed": cfg.speed.max_speed},
                "geometry": {"height": cfg.geometry.vehicle_height,
                             "width":  cfg.geometry.vehicle_width,
                             "length": cfg.geometry.vehicle_length},
                "safety":   {"stop_distance":  cfg.safety.stop_distance,
                             "tilt_limit_deg": cfg.safety.tilt_limit_deg,
                             "obstacle_height_thre": cfg.safety.obstacle_height_thre},
            })
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
            matches = [o for o in sg.get("objects", [])
                       if q in o.get("label", "").lower()]
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
                        results.append({
                            "source":   "vector",
                            "position": [hit.get("x", 0), hit.get("y", 0)],
                            "score":    hit.get("score", 0),
                            "labels":   hit.get("labels", ""),
                            "navigable": True,
                        })
                elif r.get("found"):
                    best = r.get("best") or {}
                    results.append({
                        "source": "vector",
                        "query_only": True,
                        "navigable": False,
                        "reason": "vector_memory_not_safe_for_navigation",
                        "encoder_type": r.get("encoder_type", "unknown"),
                        "labels": best.get("labels", ""),
                        "score": best.get("score", 0),
                    })
            except Exception as e:
                logger.debug("vector memory query failed: %s", e)

        em = self._episodic_mod
        if em and hasattr(em, "memory"):
            try:
                for r in em.memory.query_by_text(query, top_k=3):
                    results.append({
                        "source":   "episodic",
                        "label":    getattr(r, "label", ""),
                        "position": list(getattr(r, "position", [0, 0, 0])),
                        "ts":       getattr(r, "timestamp", 0),
                    })
            except Exception as e:
                logger.debug("episodic memory query failed: %s", e)

        tl = self._tagged_locations_mod
        if tl and hasattr(tl, "store"):
            try:
                match = tl.store.query_fuzzy(query)
                if match:
                    results.append({
                        "source":   "tagged",
                        "label":    match["name"],
                        "position": match["position"],
                    })
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
        tl.store.tag(name, x=self._odom["x"], y=self._odom["y"],
                     z=self._odom.get("z", 0))
        entry = tl.store.query(name)
        return json.dumps({"tagged": name, "position": entry})

    @skill
    def navigate_to_object(self, instruction: str) -> str:
        """Navigate to a described object or place using semantic understanding."""
        self.instruction.publish(instruction)
        return json.dumps({"status": "processing", "instruction": instruction})

    @skill
    def send_instruction(self, text: str) -> str:
        """Send a natural language instruction to the semantic planner."""
        self.instruction.publish(text)
        return json.dumps({"status": "sent", "instruction": text})

    @skill
    def emergency_stop(self) -> str:
        """Emergency stop -immediately halts all robot motion."""
        return self._publish_stop("emergency_stopped")

    def _legacy_stop_tool(self) -> str:
        return self._publish_stop("stopped")

    def _publish_stop(self, status: str) -> str:
        self.stop_cmd.publish(2)
        self.cmd_vel.publish(Twist())
        return json.dumps({"status": status})

    @skill
    def set_mode(self, mode: str) -> str:
        """Set robot operating mode: manual | autonomous | estop."""
        if mode not in ("manual", "autonomous", "estop"):
            return json.dumps({"error": f"invalid mode: {mode!r}"})
        self.mode_cmd.publish(mode)
        if mode == "estop":
            self.stop_cmd.publish(2)
            self.cmd_vel.publish(Twist())
        return json.dumps({"mode": mode})

    @skill
    def switch_backend(self, category: str, backend: str, config_json: str = "{}") -> str:
        """Switch a low-risk runtime backend; motion backends require navigation IDLE."""
        try:
            config = json.loads(config_json or "{}")
        except json.JSONDecodeError as exc:
            return json.dumps({
                "ok": False,
                "category": category,
                "requested_backend": backend,
                "reason": "invalid_config_json",
                "error": str(exc),
            })
        if not isinstance(config, dict):
            return json.dumps({
                "ok": False,
                "category": category,
                "requested_backend": backend,
                "reason": "invalid_config_json",
            })
        gateway = self._all_modules.get("GatewayModule")
        if gateway is not None and hasattr(gateway, "reconfigure_backend"):
            result = gateway.reconfigure_backend(category, backend, **config)
            return json.dumps(result, default=str)
        result = self.reconfigure_backend(category, backend, **config)
        return json.dumps(result, default=str)

    def reconfigure_backend(
        self,
        category: str,
        backend: str,
        **config: Any,
    ) -> dict[str, Any]:
        if category in _MOTION_BACKEND_CATEGORIES:
            state = _navigation_state(self._navigation)
            if state != "IDLE":
                return {
                    "ok": False,
                    "category": category,
                    "requested_backend": backend,
                    "reason": "motion_backend_switch_requires_idle",
                    "navigation_state": state or "UNKNOWN",
                }

        for module_name in _BACKEND_RECONFIGURE_TARGETS.get(category, ()):
            module = self._backend_reconfigure_modules.get(module_name)
            reconfigure = getattr(module, "reconfigure_backend", None)
            if callable(reconfigure):
                return reconfigure(category, backend, **config)

        return super().reconfigure_backend(category, backend, **config)

    # -- FastAPI + MCP JSON-RPC endpoint -----------------------------------

    def _run_server(self) -> None:
        try:
            import uvicorn
            from fastapi import FastAPI
            import os
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse
        except ImportError:
            logger.error("FastAPI not installed -run: pip install fastapi uvicorn")
            return

        cors_origins = os.environ.get(
            "LINGTU_CORS_ORIGINS",
            "http://localhost:5050,http://127.0.0.1:5050",
        ).split(",")
        app = FastAPI(title="LingTu MCP Server")
        app.add_middleware(CORSMiddleware, allow_origins=cors_origins,
                           allow_methods=["*"], allow_headers=["*"])
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
            method  = request.get("method", "")
            params  = request.get("params") or {}
            req_id  = request.get("id")

            if req_id is None:
                return JSONResponse(status_code=204, content=None)

            if method == "initialize":
                return JSONResponse(_ok(req_id, {
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities":    {"tools": {}},
                    "serverInfo":      {"name": "lingtu", "version": "2.0.0"},
                }))

            if method == "notifications/initialized":
                return JSONResponse(status_code=204, content=None)

            if method == "tools/list":
                return JSONResponse(_ok(req_id, {"tools": mcp._tool_list}))

            if method == "tools/call":
                name   = params.get("name", "")
                args   = params.get("arguments") or {}
                fn     = mcp._tool_registry.get(name)
                if fn is None:
                    return JSONResponse(_text(req_id, f"Unknown tool: {name!r}"))
                try:
                    result = fn(**args)
                    return JSONResponse(_text(req_id, result if result is not None else "Done."))
                except Exception as exc:
                    logger.exception("MCP tool error: %s", name)
                    return JSONResponse(_text(req_id, f"Error in '{name}': {exc}"))

            return JSONResponse(_error(req_id, -32601, f"Unknown method: {method}"))

        @app.get("/health")
        async def health():
            return {
                "status":  "ok",
                "tools":   len(mcp._tool_list),
                "port":    mcp._port,
                "has_handle": mcp._system_handle is not None,
            }

        uvicorn.run(app, host=self._host, port=self._port, log_level="warning")

    # -- Module health summary ---------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["mcp"] = {
            "port":       self._port,
            "tools":      len(self._tool_list),
            "has_handle": self._system_handle is not None,
        }
        return info
