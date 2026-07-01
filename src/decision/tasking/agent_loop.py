"""AgentLoop — multi-turn observe→think→act cycle with LLM tool calling.

Upgrades SemanticPlannerModule from single-shot resolve to iterative agent:
  1. Observe: gather scene_graph + odometry + memory context + camera image
  2. Think:   LLM decides next action via tool calling
  3. Act:     execute tool → any @skill method from any Module
  4. Repeat:  until task complete, max_steps reached, or abort

Tool discovery:
  Tools are auto-discovered from all Modules with @skill-decorated methods.
  SemanticPlannerModule collects them via on_system_modules() (same mechanism
  as MCPServerModule) and passes them to AgentLoop at construction time.

  Built-in tools (always available, not from @skill):
    done(summary)                 — mark task complete
    describe_scene(context)       — VLM: describe current camera view
    assess_situation(goal)        — VLM: does current view help reach goal?

Usage (called by SemanticPlannerModule._on_instruction):
  loop = AgentLoop(llm_client, tool_registry, tool_list, context_fn)
  await loop.run("找到红色椅子然后标记位置")
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)

# Built-in tools (always available, not from @skill discovery)
_BUILTIN_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "done",
            "description": "Mark the task as complete with a summary.",
            "parameters": {
                "type": "object",
                "properties": {
                    "summary": {"type": "string", "description": "Task completion summary"},
                },
                "required": ["summary"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "describe_scene",
            "description": (
                "Ask the VLM to describe what the robot camera currently sees. "
                "Use this when you need to understand the visual environment — "
                "e.g. 'what room am I in?', 'is there a path ahead?'."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "context": {
                        "type": "string",
                        "description": "Optional hint about what the robot is trying to do",
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "assess_situation",
            "description": (
                "Ask the VLM whether what the robot currently sees is useful for "
                "reaching a specific goal. Returns whether the view is relevant, "
                "a suggestion for the next action, and visible obstacles."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "goal": {
                        "type": "string",
                        "description": "The navigation goal to assess against the current view",
                    },
                },
                "required": ["goal"],
            },
        },
    },
]

_LEGACY_HANDLER_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to",
            "description": "Navigate to map coordinates in the global frame.",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "Target x position in meters"},
                    "y": {"type": "number", "description": "Target y position in meters"},
                    "z": {
                        "type": "number",
                        "description": "Optional target z position in meters; omit to stay on the current floor",
                    },
                    "yaw": {
                        "type": "number",
                        "description": "Optional heading in radians",
                        "default": 0.0,
                    },
                },
                "required": ["x", "y"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_object",
            "description": "Resolve and navigate toward an object label from the current scene graph.",
            "parameters": {
                "type": "object",
                "properties": {
                    "label": {"type": "string", "description": "Object label to navigate toward"},
                },
                "required": ["label"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "detect_object",
            "description": "Check whether an object label is visible in the current scene graph.",
            "parameters": {
                "type": "object",
                "properties": {
                    "label": {"type": "string", "description": "Object label to search for"},
                },
                "required": ["label"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "query_memory",
            "description": "Query spatial memory for a previously seen location.",
            "parameters": {
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Natural-language memory query"},
                },
                "required": ["text"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "tag_location",
            "description": "Tag the robot's current location with a name.",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string", "description": "Tag name to assign"},
                },
                "required": ["name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "say",
            "description": "Emit a short spoken or logged message for the operator.",
            "parameters": {
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Message to say"},
                },
                "required": ["text"],
            },
        },
    },
]

# Backward-compatible public constant used by legacy tests.
AGENT_TOOLS = _BUILTIN_TOOLS + _LEGACY_HANDLER_TOOLS


def _dedupe_tools(tools: list[dict]) -> list[dict]:
    seen: dict[str, dict] = {}
    for tool in tools:
        fn = tool.get("function", {})
        name = fn.get("name")
        if name:
            seen[name] = tool
    return list(seen.values())


def skills_to_openai_tools(tool_list: list[dict]) -> list[dict]:
    """Convert MCP-style tool descriptors to OpenAI function-calling format.

    MCP format:  {"name": "...", "description": "...", "inputSchema": {...}}
    OpenAI format: {"type": "function", "function": {"name", "description", "parameters"}}
    """
    result = []
    for t in tool_list:
        result.append({
            "type": "function",
            "function": {
                "name": t["name"],
                "description": t.get("description", ""),
                "parameters": t.get("inputSchema", {"type": "object", "properties": {}}),
            },
        })
    return result

AGENT_SYSTEM_PROMPT = """You are a navigation robot assistant. You execute tasks by calling tools in sequence.

Available information:
- Current position: ({robot_x:.1f}, {robot_y:.1f})
- Visible objects: {visible_objects}
- Navigation status: {nav_status}
- Memory context: {memory_context}
- Camera available: {camera_available}

Rules:
- Break complex tasks into steps and execute them one at a time.
- Use detect_object to check if something is visible before navigating to it.
- Use query_memory for fuzzy location requests ("去上次放背包的地方").
- Use navigate_to_object for objects currently in view.
- Use navigate_to for known coordinates.
- Use follow_person(description) to continuously follow a specific person
  (e.g. "follow the person in red"); call stop_servo to cancel following.
- Use describe_scene() when you need to understand what the robot currently sees.
- Use assess_situation(goal) when you are unsure whether the current view is helpful.
- Call done() when the task is complete.
- If you cannot complete the task after several attempts, call done() with an explanation.
"""


@dataclass
class AgentState:
    """Tracks agent loop execution state."""
    instruction: str = ""
    step: int = 0
    max_steps: int = 10
    messages: list[dict] = field(default_factory=list)
    completed: bool = False
    summary: str = ""
    start_time: float = 0.0


class AgentLoop:
    """Multi-turn LLM tool-calling agent for complex navigation tasks.

    Each step: build context → LLM call with tools → parse tool call → execute → feedback.
    Loops until done() called, max_steps, or timeout.
    """

    def __init__(
        self,
        llm_client,
        tool_registry: dict[str, Callable],
        tool_list: list[dict],
        context_fn: Callable[[], dict],
        max_steps: int = 10,
        timeout: float = 120.0,
        *,
        tool_handlers: dict[str, Callable] | None = None,
    ):
        """
        Args:
            llm_client: LLM client with chat() method (supports tool calling)
            tool_registry: {tool_name: bound_method} — auto-discovered @skill methods
            tool_list: MCP-style tool descriptors from @skill auto-discovery
            context_fn: returns {robot_x, robot_y, visible_objects, nav_status,
                        memory_context, camera_image (np.ndarray or None),
                        scene_graph (dict or None), camera_available (bool)}
            max_steps: max iterations before forced stop
            timeout: max wall-clock seconds
            tool_handlers: (deprecated) legacy {name: fn} dict, merged into registry
        """
        self._llm = llm_client
        discovered_tools = skills_to_openai_tools(tool_list)
        legacy_tools = [
            tool for tool in _LEGACY_HANDLER_TOOLS
            if tool["function"]["name"] in (tool_handlers or {})
        ]

        # Merge legacy handlers + auto-discovered handlers; discovered handlers win.
        self._handlers = dict(tool_handlers or {})
        self._handlers.update(tool_registry)

        # Build OpenAI-format tools: builtins + discovered + legacy compatibility tools.
        self._tools = _dedupe_tools(_BUILTIN_TOOLS + discovered_tools + legacy_tools)
        self._context_fn = context_fn
        self._max_steps = max_steps
        self._timeout = timeout
        self._vlm_agent = None  # lazy-initialized on first VLM tool call

        # W2-12: tool-call audit log + required-argument schemas for the 7
        # canonical agent tools. Each `run()` step that reaches `_execute_tool`
        # records one audit entry with result_summary (either the short tool
        # output or `VALIDATION_ERROR: ...`).
        self._tool_call_audit: list[dict[str, Any]] = []
        # Auto-generated from _BUILTIN_TOOLS + _LEGACY_HANDLER_TOOLS to keep
        # required-field schemas in sync with tool definitions (P0.3/P2.2).
        self._tool_schemas: dict[str, dict[str, Any]] = {}
        for _tool in _BUILTIN_TOOLS + _LEGACY_HANDLER_TOOLS:
            _fn = _tool["function"]
            _params = _fn.get("parameters", {})
            self._tool_schemas[_fn["name"]] = {"required": _params.get("required", [])}
        self._known_tool_names: set[str] = {
            t["function"]["name"] for t in self._tools
        } | set(self._handlers.keys())
        logger.info(
            "AgentLoop: %d tools (%d discovered + %d legacy + %d builtin)",
            len(self._tools), len(tool_list), len(legacy_tools), len(_BUILTIN_TOOLS),
        )

    async def run(self, instruction: str) -> AgentState:
        """Execute the agent loop for a given instruction."""
        state = AgentState(
            instruction=instruction,
            max_steps=self._max_steps,
            start_time=time.time(),
        )

        # Initial system + user message
        ctx = self._context_fn()
        # Ensure new keys have defaults for backward-compat with older context_fn
        ctx.setdefault("camera_available", ctx.get("camera_image") is not None)
        c = ctx
        system_msg = (
            f"You are a navigation robot assistant. You execute tasks by calling tools in sequence.\n\n"
            f"Available information:\n"
            f"- Current position: ({c.get('robot_x', 0.0):.1f}, {c.get('robot_y', 0.0):.1f})\n"
            f"- Visible objects: {c.get('visible_objects', 'N/A')}\n"
            f"- Navigation status: {c.get('nav_status', 'N/A')}\n"
            f"- Memory context: {c.get('memory_context', 'N/A')}\n"
            f"- Camera available: {c.get('camera_available', False)}\n\n"
            f"Rules:\n"
            f"- Break complex tasks into steps and execute them one at a time.\n"
            f"- Use detect_object to check if something is visible before navigating to it.\n"
            f"- Use query_memory for fuzzy location requests (“去上次放背包的地方”).\n"
            f"- Use navigate_to_object for objects currently in view.\n"
            f"- Use navigate_to for known coordinates.\n"
            f"- Use follow_person(description) to continuously follow a specific person\n"
            f"- Use follow_person with a description (e.g. 'person in red') to follow"
            f" someone; call stop_servo to cancel following.\n"
            f"- Use describe_scene() when you need to understand what the robot currently sees.\n"
            f"- Use assess_situation(goal) when you are unsure whether the current view is helpful.\n"
            f"- Call done() when the task is complete.\n"
            f"- If you cannot complete the task after several attempts, call done() with an explanation.\n"
        )
        state.messages = [
            {"role": "system", "content": system_msg},
            {"role": "user", "content": instruction},
        ]

        while state.step < state.max_steps and not state.completed:
            if time.time() - state.start_time > self._timeout:
                state.summary = f"Timeout after {self._timeout}s"
                state.completed = True
                break

            state.step += 1

            # LLM call with tools
            try:
                response = await self._llm_call(state.messages)
            except Exception as e:
                logger.error("AgentLoop: LLM call failed at step %d: %s", state.step, e)
                state.summary = f"LLM error: {e}"
                state.completed = True
                break

            # Parse response
            if response.get("tool_calls"):
                for tc in response["tool_calls"]:
                    await self._execute_tool(tc, state)
                    if state.completed:
                        break
            elif response.get("content"):
                # LLM responded with text instead of tool call — treat as thinking
                state.messages.append({"role": "assistant", "content": response["content"]})
                logger.debug("AgentLoop step %d: LLM text: %s", state.step, response["content"][:100])
            else:
                state.summary = "LLM returned empty response"
                state.completed = True

        if not state.completed:
            state.summary = f"Max steps ({self._max_steps}) reached"
            state.completed = True

        logger.info("AgentLoop: '%s' → %d steps, summary: %s",
                     instruction, state.step, state.summary)
        return state

    async def _llm_call(self, messages: list[dict]) -> dict:
        """Call LLM with tool definitions. Returns parsed response."""
        # Try tool-calling format (OpenAI-compatible)
        if hasattr(self._llm, "chat_with_tools"):
            return await self._llm.chat_with_tools(messages, tools=self._tools)

        # Fallback: regular chat with tool descriptions in prompt
        tools_desc = "\n".join(
            (
                f"- {t['function']['name']}: {t['function'].get('description', '')} "
                f"args={json.dumps(t['function'].get('parameters', {}), ensure_ascii=False)}"
            )
            for t in self._tools
        )
        augmented = messages.copy()
        augmented[0] = {
            "role": "system",
            "content": messages[0]["content"] + f"\n\nAvailable tools:\n{tools_desc}\n\n"
                       "You must either choose exactly one tool call in JSON or call done.\n"
                       "Respond with JSON only: {\"tool\": \"name\", \"args\": {...}}"
        }

        text = await self._llm.chat(augmented)

        # Parse JSON tool call from text
        return self._parse_text_tool_call(text)

    @staticmethod
    def _parse_text_tool_call(text: str) -> dict:
        """Extract tool call from LLM text response."""
        # Try to find JSON with "tool" key — scan for balanced braces
        start = text.find('{"tool"')
        if start < 0:
            start = text.find('"tool"')
            if start > 0:
                start = text.rfind('{', 0, start)
        if start >= 0:
            depth = 0
            for i in range(start, len(text)):
                if text[i] == '{':
                    depth += 1
                elif text[i] == '}':
                    depth -= 1
                    if depth == 0:
                        try:
                            data = json.loads(text[start:i + 1])
                            tool_name = data.get("tool", "")
                            args = data.get("args", {})
                            if tool_name:
                                return {
                                    "tool_calls": [{
                                        "function": {"name": tool_name,
                                                     "arguments": json.dumps(args)},
                                        "id": f"call_{time.time_ns()}",
                                    }],
                                }
                        except json.JSONDecodeError:
                            pass
                        break
        return {"content": text}

    def _validate_tool_call(self, name: str, args: dict) -> str | None:
        """W2-12: return an error string if the call is invalid, else None.

        Catches (a) LLM hallucinated tool names, (b) missing required args.
        """
        if not name:
            return "empty tool name"
        if name not in self._known_tool_names:
            sample = sorted(self._known_tool_names)[:8]
            return (
                f"unknown tool '{name}'. Valid tools start with: {sample}"
            )
        schema = self._tool_schemas.get(name, {})
        for required_field in schema.get("required", []):
            if required_field not in args:
                return f"missing required argument '{required_field}' for tool '{name}'"
        return None

    async def _execute_tool(self, tool_call: dict, state: AgentState) -> str:
        """Execute a single tool call and append results to message history."""
        fn = tool_call.get("function", {})
        name = fn.get("name", "")
        try:
            args = json.loads(fn.get("arguments", "{}"))
        except json.JSONDecodeError:
            args = {}

        # Append assistant tool call
        state.messages.append({
            "role": "assistant",
            "content": None,
            "tool_calls": [tool_call],
        })

        # W2-12: validate BEFORE executing — feed error back to LLM for
        # self-correction instead of silently skipping.
        validation_err = self._validate_tool_call(name, args)
        if validation_err is not None:
            summary = f"VALIDATION_ERROR: {validation_err}"
            self._tool_call_audit.append({
                "ts": time.time(),
                "step": state.step,
                "tool_name": name,
                "args": args,
                "result_summary": summary,
            })
            error_msg = (
                f"VALIDATION_ERROR for tool '{name}': {validation_err}. "
                f"Please emit a corrected tool call."
            )
            state.messages.append({
                "role": "tool",
                "tool_call_id": tool_call.get("id", ""),
                "content": error_msg,
            })
            logger.warning("AgentLoop step %d: %s", state.step, summary)
            return summary

        # Handle done() specially
        if name == "done":
            state.completed = True
            state.summary = args.get("summary", "Task complete")
            result = state.summary
        elif name in ("describe_scene", "assess_situation"):
            # VLM tools — handled internally using the VLMSceneAgent
            result = await self._execute_vlm_tool(name, args)
        elif name in self._handlers:
            try:
                handler = self._handlers[name]
                if asyncio.iscoroutinefunction(handler):
                    result = await handler(**args)
                else:
                    result = handler(**args)
                result = str(result) if result is not None else "OK"
            except Exception as e:
                result = f"Error: {e}"
                logger.warning("AgentLoop: tool '%s' failed: %s", name, e)
        else:
            result = f"Unknown tool: {name}"

        # Append tool result
        state.messages.append({
            "role": "tool",
            "tool_call_id": tool_call.get("id", ""),
            "content": result,
        })

        # Update context for next step
        ctx = self._context_fn()
        state.messages.append({
            "role": "system",
            "content": f"Updated state: position=({ctx['robot_x']:.1f},{ctx['robot_y']:.1f}), "
                       f"nav_status={ctx['nav_status']}, visible={ctx['visible_objects'][:100]}",
        })

        # W2-12: audit log for every valid call that executed.
        self._tool_call_audit.append({
            "ts": time.time(),
            "step": state.step,
            "tool_name": name,
            "args": args,
            "result_summary": (result[:200] if isinstance(result, str) else str(result)[:200]),
        })

        logger.info("AgentLoop step %d: %s(%s) → %s",
                     state.step, name, args, result[:100])
        return result

    def _get_vlm_agent(self):
        """Return the VLMSceneAgent, creating it lazily on first use."""
        if self._vlm_agent is None:
            try:
                from decision.vision.vlm_scene_agent import VLMSceneAgent
                self._vlm_agent = VLMSceneAgent(self._llm)
                logger.info("AgentLoop: VLMSceneAgent initialized (backend=%s)",
                            type(self._llm).__name__)
            except Exception as e:
                logger.warning("AgentLoop: VLMSceneAgent init failed: %s", e)
        return self._vlm_agent

    async def _execute_vlm_tool(self, name: str, args: dict) -> str:
        """Execute a VLM scene-understanding tool.

        Retrieves the latest camera frame from context_fn and dispatches to
        VLMSceneAgent. Returns a plain string suitable for appending to the
        agent's message history.
        """
        vlm = self._get_vlm_agent()
        if vlm is None:
            return "VLM scene understanding not available"

        # Pull the latest context to get the current camera frame and scene graph
        ctx = self._context_fn()
        camera_image = ctx.get("camera_image")  # np.ndarray or None
        scene_graph = ctx.get("scene_graph")    # dict or None

        try:
            if name == "describe_scene":
                context_hint = args.get("context", "")
                return await vlm.describe_scene(camera_image, context=context_hint)

            elif name == "assess_situation":
                goal = args.get("goal", "")
                result = await vlm.assess_situation(camera_image, goal, scene_graph)
                # Format the structured result as a readable string for the agent
                lines = [f"Relevant to goal: {result.get('relevant', False)}"]
                suggestion = result.get("suggestion", "")
                if suggestion:
                    lines.append(f"Suggestion: {suggestion}")
                obstacles = result.get("obstacles", [])
                if obstacles:
                    lines.append(f"Obstacles: {', '.join(str(o) for o in obstacles)}")
                return "\n".join(lines)

        except Exception as e:
            logger.warning("AgentLoop: VLM tool '%s' failed: %s", name, e)
            return f"VLM tool error: {e}"

        return f"Unknown VLM tool: {name}"
