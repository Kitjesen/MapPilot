#!/usr/bin/env python3
"""Generate the single maintained LingTu API reference."""

from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path


def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)
SRC = REPO_ROOT / "src"
GATEWAY = SRC / "gateway"
DOCS_API = REPO_ROOT / "docs" / "api.md"

# ── helpers ──────────────────────────────────────────────────────────────

def _docstring_first_line(node: ast.FunctionDef) -> str:
    """Return the first line of the docstring."""
    if not node.body:
        return ""
    first = node.body[0]
    if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant):
        raw = first.value.value or ""
        return raw.strip().split("\n")[0] if raw else ""
    return ""


def _module_docstring(path: Path) -> str:
    """Return the first line of a module's docstring."""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        if tree.body and isinstance(tree.body[0], ast.Expr) and isinstance(tree.body[0].value, ast.Constant):
            raw = tree.body[0].value.value
            return raw.strip().split("\n")[0] if raw else ""
    except SyntaxError:
        pass
    return ""


# ── TASK 1: Extract @skill methods ──────────────────────────────────────

def _is_real_module(path: Path) -> bool:
    """Skip test files, legacy, and __init__.py."""
    parts = path.parts
    return (
        "tests" not in parts
        and "legacy" not in parts
        and path.name != "__init__.py"
    )


def extract_skills() -> list[dict]:
    """Scan all Module files for @skill-decorated methods.

    Returns list of dicts with: module, class_name, method_name, params, return_type, description
    """
    skills: list[dict] = []

    for py_file in sorted(SRC.rglob("*.py")):
        if not _is_real_module(py_file):
            continue

        try:
            tree = ast.parse(py_file.read_text(encoding="utf-8"))
        except (SyntaxError, UnicodeDecodeError):
            continue

        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue

            for item in node.body:
                if not isinstance(item, ast.FunctionDef):
                    continue
                decorators = [d.func if isinstance(d, ast.Call) else d for d in item.decorator_list]
                if not any(isinstance(d, ast.Name) and d.id == "skill" for d in decorators):
                    continue

                # Parse params
                params: list[dict] = []
                for arg in item.args.args:
                    if arg.arg == "self":
                        continue
                    param = {"name": arg.arg}
                    if arg.annotation:
                        param["type"] = _ast_to_str(arg.annotation)
                    params.append(param)

                # Return type
                return_type = ""
                if item.returns:
                    return_type = _ast_to_str(item.returns)

                rel_path = py_file.relative_to(REPO_ROOT)
                description = _docstring_first_line(item)
                for decorator in item.decorator_list:
                    if not isinstance(decorator, ast.Call) or not isinstance(decorator.func, ast.Name):
                        continue
                    if decorator.func.id != "skill":
                        continue
                    for keyword in decorator.keywords:
                        if keyword.arg == "description" and isinstance(keyword.value, ast.Constant):
                            description = str(keyword.value.value).strip()

                module_doc = _module_docstring(py_file)

                skills.append({
                    "file": rel_path.as_posix(),
                    "module_doc": module_doc,
                    "class_name": node.name,
                    "method_name": item.name,
                    "params": params,
                    "return_type": return_type,
                    "description": description,
                })

    return skills


def _ast_to_str(node: ast.AST) -> str:
    """Convert an AST annotation node back to a string."""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return f"{_ast_to_str(node.value)}.{node.attr}"
    if isinstance(node, ast.Subscript):
        return f"{_ast_to_str(node.value)}[{_ast_to_str(node.slice)}]"
    if isinstance(node, ast.Constant):
        return str(node.value)
    if isinstance(node, ast.Tuple):
        return f"({', '.join(_ast_to_str(e) for e in node.elts)})"
    if isinstance(node, ast.BinOp):
        if isinstance(node.op, ast.BitOr):
            return f"{_ast_to_str(node.left)} | {_ast_to_str(node.right)}"
        return f"{_ast_to_str(node.left)} {_ast_to_str(node.op)} {_ast_to_str(node.right)}"
    return ast.dump(node)


def _markdown_cell(value: str) -> str:
    """Escape extracted text for a Markdown table cell."""
    return value.replace("|", "\\|").replace("\n", " ")


def generate_mcp_tools_md(skills: list[dict]) -> str:
    """Generate the MCP section of the API reference."""
    lines = [
        "## MCP tools",
        "",
        "This inventory is generated from `@skill` decorators. The Host exposes",
        "the selected Modules' methods through MCP JSON-RPC on port 8090.",
        "Agent discovery uses the same descriptors, then applies its own tool filtering",
        "and task-owned motion handlers; the static inventory is not an availability guarantee.",
        "Parameterized declarations add parameter descriptions, optional object output schemas,",
        "and `readOnlyHint`. Input types, required arguments and defaults come from Python signatures.",
        "MCP object results include `structuredContent` and JSON text. Agent function tools receive",
        "output/behavior descriptions and JSON-encoded object results. These hints do not grant",
        "motion permission or turn an admission receipt into a completed task.",
        "See [the runtime contract](../src/runtime/README.md#skill-contract) and the",
        "[MCP tool specification](https://modelcontextprotocol.io/specification/2025-11-25/server/tools).",
        "",
    ]

    # Group by module file
    from collections import defaultdict
    by_file: dict[str, list[dict]] = defaultdict(list)
    for s in skills:
        by_file[s["file"]].append(s)

    for filepath in sorted(by_file):
        entries = by_file[filepath]
        module_doc = entries[0]["module_doc"]
        lines.append(f"## {filepath}")
        if module_doc:
            lines.append(f"_{module_doc}_")
        lines.append("")
        lines.extend(
            (
                "| Tool | Module | Parameters | Returns | Description |",
                "| --- | --- | --- | --- | --- |",
            )
        )
        for entry in entries:
            method = entry["method_name"]
            class_name = entry["class_name"]
            desc = _markdown_cell(entry["description"]) or "—"
            ret = _markdown_cell(entry["return_type"]) or "—"
            params = entry["params"]
            rendered_params = ", ".join(
                f"`{p['name']}: {_markdown_cell(p.get('type', '')) or 'Any'}`"
                for p in params
            ) or "—"
            lines.append(
                f"| `{method}` | `{class_name}` | {rendered_params} | "
                f"`{ret}` | {desc} |"
            )
        lines.append("")

    return "\n".join(lines)


# ── TASK 2: Extract Gateway REST routes ─────────────────────────────────

_HTTP_ROUTE_METHODS = frozenset({"get", "post", "put", "delete", "patch"})


def _route_keyword(call: ast.Call, name: str) -> ast.AST | None:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def _routes_from_file(py_file: Path) -> list[dict]:
    try:
        tree = ast.parse(py_file.read_text(encoding="utf-8"))
    except (SyntaxError, UnicodeDecodeError, OSError):
        return []

    routes: list[dict] = []
    rel_path = py_file.relative_to(REPO_ROOT).as_posix()
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for decorator in node.decorator_list:
            if (
                not isinstance(decorator, ast.Call)
                or not isinstance(decorator.func, ast.Attribute)
                or not isinstance(decorator.func.value, ast.Name)
                or decorator.func.value.id != "app"
                or decorator.func.attr not in _HTTP_ROUTE_METHODS
                or not decorator.args
                or not isinstance(decorator.args[0], ast.Constant)
                or not isinstance(decorator.args[0].value, str)
            ):
                continue
            summary_node = _route_keyword(decorator, "summary")
            summary = (
                summary_node.value
                if isinstance(summary_node, ast.Constant)
                and isinstance(summary_node.value, str)
                else ""
            )
            response_node = _route_keyword(decorator, "response_model")
            routes.append(
                {
                    "file": rel_path,
                    "method": decorator.func.attr.upper(),
                    "path": decorator.args[0].value,
                    "summary": summary,
                    "response_model": (
                        _ast_to_str(response_node) if response_node is not None else ""
                    ),
                    "function": node.name,
                }
            )
    return routes


def extract_gateway_routes() -> list[dict]:
    """Extract every static FastAPI route decorator, including stacked aliases."""

    files = [
        path for path in sorted(GATEWAY.rglob("*.py"))
        if path.name != "__init__.py"
    ]

    routes = [route for py_file in files for route in _routes_from_file(py_file)]
    routes.sort(key=lambda route: (route["path"], route["method"], route["file"]))
    return routes


def generate_gateway_rest_md(routes: list[dict]) -> str:
    """Generate the Gateway section of the API reference."""
    lines = [
        "## Gateway REST API",
        "",
        "The Gateway serves these generated route registrations on port 5050.",
        "FastAPI's live OpenAPI UI remains available at `/docs`.",
        "",
        "Each source file has one complete table; the rows are the route contract.",
        "",
    ]

    # Table of contents by file
    from collections import defaultdict
    by_file: dict[str, list[dict]] = defaultdict(list)
    for r in routes:
        by_file[r["file"]].append(r)

    for filepath in sorted(by_file):
        lines.append(f"### {filepath}")
        lines.append("")
        lines.extend(
            (
                "| Route | Summary | Response model | Handler |",
                "| --- | --- | --- | --- |",
            )
        )
        for entry in by_file[filepath]:
            route = _markdown_cell(f"{entry['method']} {entry['path']}")
            summary = _markdown_cell(entry["summary"]) or "—"
            response_model = _markdown_cell(entry["response_model"]) or "—"
            lines.append(
                f"| `{route}` | {summary} | `{response_model}` | "
                f"`{entry['function']}` |"
            )
        lines.append("")

    return "\n".join(lines)


def generate_api_md(skills: list[dict], routes: list[dict]) -> str:
    """Generate the complete maintained API reference."""

    header = [
        "# API",
        "",
        "**Status:** Current generated interface reference",
        "**Audience:** SDK, Web, MCP, Gateway, and integration developers",
        "**Runs on:** One LingTu Host in `env=real` or `env=sim`",
        "",
        "This file combines the maintained integration contract with inventories",
        "generated from source. Do not hand-edit the method or route tables; run",
        "`python tools/docs/extract_api_docs.py`.",
        "",
        "## Integration boundaries",
        "",
        "| Surface | Endpoint | Owner |",
        "| --- | --- | --- |",
        "| REST and SSE | `http://<host>:5050` | Gateway routes and projections |",
        "| MCP | `http://<host>:8090/mcp` | `@skill` methods discovered in the Host |",
        "| Python SDK/CLI | `lingtu-sdk` and `lingtu.sdk` | Typed client facade |",
        "| Camera Web media | Scene and Console: WHEP, with JPEG WebSocket fallback | Native media endpoint or optional go2rtc; Gateway signalling |",
        "",
        "`LingTuClient` and `AsyncLingTuClient` group operations into domain clients:",
        "`navigation`, `map`, `tasks`, `observations`, and `system`. The historical",
        "`localization` entry remains available for compatibility. Domain clients share",
        "one Gateway address, credentials and request timeout. Await asynchronous",
        "methods. Existing flat methods remain supported delegates. See the",
        "[Python SDK guide](../src/lingtu/README.md#remote-python-sdk) for classes",
        "and examples. Navigation cancellation and software emergency stop retain",
        "their separate `navigation.cancel()` and `system.emergency_stop()` meanings.",
        "",
        "Gateway submits typed intent and projects runtime facts. It does not",
        "own Product lifecycle, maps, planning, final motion, or hardware.",
        "",
        "Camera signalling uses `GET /api/v1/webrtc/status`, `POST /api/v1/webrtc/whep`",
        "and DELETE on the returned session `Location`. Status includes `backend`,",
        "`ice_servers` and `rotation_degrees` from RobotConfig. Set `LINGTU_WEBRTC_URL`",
        "to the full native WHEP URL (for example `http://127.0.0.1:9090/whep/endpoint`);",
        "otherwise Gateway uses `LINGTU_GO2RTC_URL` (default `http://127.0.0.1:1984`).",
        "`LINGTU_WEBRTC_ICE_SERVERS` is a JSON list in the browser RTCIceServer format.",
        "It configures browser traversal; the selected media server also needs its own",
        "reachable ICE candidates and TURN configuration. A signalling HTTP URL alone",
        "does not make the video transport reachable. Gateway does not start the media",
        "process, and never forwards encoded video. See [video implementation](research/video.md).",
        "",
        "Product lifecycle uses `lingtu` / `python -m lingtu.control`, not REST",
        "service orchestration assembled by a client.",
        "Web mode switches use the separate loopback ProductControl transport:",
        "`GET /api/v1/product-control` returns current session/operation;",
        "`POST /api/v1/product-control/switch` accepts `request_id`, `product`",
        "(`map` or `nav`), `map_name` (required for nav), and",
        "`expected_product_session_id` (empty only when no Product is running).",
        "`GET /api/v1/product-control/operations/{request_id}` retrieves the receipt.",
        "Gateway only forwards these requests. A 202 response is admission, not",
        "completion. Query until `succeeded`, `failed`, or `interrupted`; reconnects",
        "must query the original ID, not create another switch. A successful switch",
        "does not authorize robot motion. See [Operations](./operations.md#web-mapping-and-navigation-switches).",
        "",
        "External map integrations operate on the canonical map identity",
        "`map_id + content_epoch` and maintained artifact contracts. They must",
        "not invent version directories or bypass ProductControl activation.",
        "",
        "Live-mapping saves are durable operations. A successful admission is not a completed",
        "save: poll `GET /api/v1/maps/operations/{operation_id}` and inspect",
        "`operation.state`. A client wait timeout or a lost status response leaves the",
        "outcome unconfirmed; continue querying that operation instead of submitting",
        "another save. A `SUCCEEDED` detail includes the available",
        "`operation.processing.optimization` and `operation.processing.cleanup`",
        "summaries (`performed`, `success`, `reason_code`). Optimization can successfully",
        "skip when measurement constraints are insufficient, so save success does not",
        "imply completed optimization or navigation readiness. Lightweight operation",
        "lists omit these summaries; fetch the individual detail. A same-name save may",
        "replace an inactive map; active-map changes are rejected. Saves do not activate",
        "a map or switch Product.",
        "",
        "`GET /map/viewer?map=MAP_ID` returns a 307 redirect to `/?tab=map&map=MAP_ID`;",
        "without `map` it opens the product map library. The redirect does not activate",
        "a map or invoke ProductControl. The previous independent Gateway HTML viewer",
        "is removed.",
        "",
        "The library previews any saved map through `GET /api/v1/maps/{name}/pcd`,",
        "which streams the original native artifact and returns `Cache-Control: no-store`.",
        "`X-Lingtu-Map-Id`, `X-Lingtu-Content-Epoch`, and `X-Lingtu-Frame-Id` come from",
        "the native artifact metadata. The viewer rejects missing or mismatched identity",
        "and reloads after content-version, Product-session, or explicit refresh changes.",
        "Preview neither changes the saved data nor activates the selected map.",
        "The separate `/points` endpoint remains bound to the active live scene.",
        "",
        "`GET /api/v1/maps/{name}/octomap` streams the saved full-probability",
        "`octomap.ot` with the same artifact identity headers. Its occupied-voxel",
        "display does not classify undrawn space as free or unknown.",
        "",
        "## Active navigation task speed limit",
        "",
        "`POST /api/v1/navigation/tasks/{task_id}/speed-limit` accepts the active control",
        "owner's request. The body contains `client_id`, a distinct `request_id` and",
        "`max_speed_mps`. An optional body `task_id` must match the route.",
        "",
        "```json",
        "{\"client_id\": \"web\", \"request_id\": \"speed-1\", \"max_speed_mps\": 0.3}",
        "```",
        "",
        "The native endpoint accepts a finite positive limit up to its configured",
        "navigation maximum for the retained active task, including a paused task.",
        "A task still awaiting its first route, an unrelated task and a completed task",
        "return a rejection. A pending terminal transaction also rejects the update.",
        "",
        "A 202 response includes the correlated native ACK with kind `10` and reason",
        "`task_speed_limit_updated`. The route and task identity remain active, and an",
        "in-flight global replan retains the latest limit. SCAN uses its existing speed",
        "transition to generate the changed trajectory. The ACK confirms that the limit",
        "was updated. Actual speed and tracking still come from runtime telemetry, so",
        "`execution_confirmed` and `final_output_confirmed` remain false in this response.",
        "",
        "## Saved point-cloud subset editing",
        "",
        "`POST /api/v1/maps/{name}/save_as` forwards to native mapd's",
        "`save_edited_source`. This synchronous source-edit operation is separate",
        "from the live-mapping save-operation API above. Its JSON body contains:",
        "",
        "| Field | Contract |",
        "| --- | --- |",
        "| `name` | New target map name, different from the source. |",
        "| `expected_source_content_epoch` | Positive integer from the source artifact. |",
        "| `operations` | Ordered list of subset edits, or empty to copy the source. |",
        "| `operations[].operation` | `delete_inside` or `keep_inside`. |",
        "| `operations[].bounds` | Finite map-frame `min_x`, `min_y`, `min_z`, `max_x`, `max_y`, `max_z`; each minimum must not exceed its maximum. Units: metres. |",
        "",
        "All box faces are inclusive. Native mapd applies each operation to the",
        "remaining complete source XYZ cloud; Web display sampling does not limit",
        "the saved edit. It preserves retained coordinates without downsampling,",
        "rejects an empty result, and leaves the source unchanged. A changed source",
        "returns `source_epoch_mismatch`; reload before editing again. An existing",
        "unrelated target returns `target_exists`.",
        "",
        "Success returns the target `map_id`, new `content_epoch`, `source_map_id`,",
        "`source_content_epoch`, `frame_id`, `point_count`, `removed_points`,",
        "`saved_rays_preserved`, `build_options`, and `activation_ready=false`.",
        "A complete measured-ray bundle (poses, patches, manifest, scan origin) is",
        "retained unchanged; incomplete evidence and old derived artifacts are not",
        "copied. Source save success does not mean the navigation artifacts exist.",
        "",
        "If the save response is lost, keep the target name, source identity/epoch,",
        "and exact ordered operations. Resubmit that identical request to confirm",
        "the result. mapd returns the existing result with `replayed=true` only",
        "while the target remains invalidated with reason `edited_source` and its",
        "stored edit matches. It does not re-edit or overwrite that target. This",
        "confirmation is checked before rereading the source, so later source",
        "changes do not prevent confirmation of the saved snapshot. After rebuild",
        "replaces the invalidated metadata, this request no longer confirms it;",
        "inspect the target through the existing map queries instead.",
        "",
        "Then call `POST /api/v1/maps/{target}/build_octomap` with the returned",
        "`build_options` as its JSON body. Accepted option keys are `resolution`,",
        "`frame_id`, `source_profile`, `data_source`, `slam_source`,",
        "`localization_source`, and `mapping_source`. With no body, the builder",
        "inherits the saved target metadata. A failed build leaves the source",
        "available; retry the build rather than saving another copy. A successful",
        "build commits a new content epoch, so reload the target artifact identity.",
        "Neither operation activates the map; activation remains ProductControl's",
        "responsibility. Deleting points does not establish observed free space,",
        "ground support, route feasibility, or permission to move.",
        "",
        "For a new navigation resolution, copy the source with `operations: []`",
        "and build the target with the returned options and requested `resolution`.",
        "The Web offers 0.05, 0.06, 0.10 and 0.20 m. The build uses the copied",
        "source and available measured rays to rebuild the navigation OctoMap.",
        "The response includes `resolution` and `frame_id`; metadata retains the",
        "source map identity. Activate the target through ProductControl to use it.",
        "The separate live local collision layer retains its sensor-profile resolution.",
        "",
        "## Natural-language Agent task lifecycle",
        "",
        "`POST /api/v1/instruction` and MCP `send_instruction` prefer the loaded",
        "`AgentPlannerModule`. Admission returns an Agent `run_id`, `state` and",
        "`executor=agent`; `execution_confirmed=false` and `motor_confirmed=false`",
        "make explicit that admission is not task or motion success. A Host without",
        "Agent retains the existing semantic instruction-stream fallback.",
        "",
        "The instruction body accepts `text`, optional `run_id`, `control_only`",
        "(default `false`), `request_id`, and `client_id`. MCP `send_instruction`",
        "accepts the same text, optional run identity, and `control_only` constraint.",
        "The Agent owner recognizes complete control utterances before starting a new",
        "model task. Responses identify `operation=start/query/keep/cancel/pause/resume`.",
        "Queries and keep requests return the original instruction and a `task_status`",
        "snapshot, including `found`, without replacing the current run. An idle query",
        "returns `state=idle` and no run; it is not task admission.",
        "",
        "Use `control_only=true` when the caller may query or end an existing task but",
        "must not start a new one. The Web client uses it while its motion status is",
        "missing/stale or it still holds an active task. A non-control instruction then",
        "returns `accepted=false`, `reason=motion_start_blocked`, and does not start or",
        "publish a task. This request constraint grants no motion permission; ordinary",
        "starts retain the server's existing motion checks. Unsupported pause/resume",
        "returns `unsupported_control`. A new task while the current task is active is",
        "rejected with `task_active`; cancellation/failure without stop confirmation",
        "returns `stop_unconfirmed`. End the old task, wait for confirmation, and submit",
        "the new task explicitly. A rejected receipt may reference the current run;",
        "the presence of `run_id` must not override `accepted=false`.",
        "Rejected command receipts use HTTP 409; Web clients read `reason` from the",
        "error body instead of replacing it with a generic transport error.",
        "",
        "Read the current Host run with `GET /api/v1/agent/runs/{run_id}`. The",
        "endpoint intentionally retains only the current run: a different or expired",
        "identity returns `found=false` and `reason=not_current`. Cancel with",
        "`POST /api/v1/agent/runs/{run_id}/cancel`, retaining the original ID.",
        "Cancellation is scoped to that run, so retrying an old cancellation cannot",
        "cancel a newer Agent task. Admission of cancellation still does not confirm",
        "that native child motion has stopped. Current-run status retains the owned",
        "domain reference and reports `stop_confirmed` separately from Agent state.",
        "After cancellation, keep reading while `stop_confirmed=false`; a confirmed",
        "native/domain terminal event, or no submitted motion, can confirm task stop.",
        "This does not measure the robot's physical velocity or certify physical rest.",
        "Cancelling an already completed Agent run is rejected as `task_not_running`",
        "and preserves its completed result.",
        "Status also includes the original `instruction`, `failure_reason`, model",
        "`summary` and `navigation` details. The summary is diagnostic model text,",
        "not execution evidence. While waiting for a model before domain submission,",
        "`navigation` is null. Agent chat events carry `run_id`; clients must keep",
        "messages and query responses bound to that identity when tasks change.",
        "",
        "The Agent rejects `done(success=true)` for execution-oriented instructions",
        "unless an owned execution tool was attempted and its domain result succeeded.",
        "Explicit pure status queries are informational and do not require motion.",
        "A sustained owned Follow wait is outside the finite Agent reasoning/action",
        "budget. Model calls retain their per-call timeout, while ordinary navigation",
        "and find actions retain the Agent task deadline. Agent reads the terminal",
        "status of the same `follow_task_id`; cancellation remains explicit and scoped.",
        "`TaskPlan` remains a separate typed decomposition model; the Agent run is not",
        "yet a durable, restart-recoverable Mission executor.",
        "",
        "## Follow task lifecycle",
        "",
        "Follow is an owned execution rather than a stream of anonymous visual-servo",
        "commands. Start it with `POST /api/v1/visual_servo` using `mode=follow` and a",
        "`target` description or explicit `target_id`. A successful receipt returns both",
        "`follow_task_id` and `product_session_id`; retain both values for every later",
        "operation. The same owner methods are available to MCP as `start_follow`,",
        "`get_follow_task`, and `cancel_follow_task`.",
        "",
        "Query one execution with",
        "`GET /api/v1/follow/{follow_task_id}?product_session_id=...`. The returned",
        "`state` and `terminal` describe the Follow execution; `details` contains the",
        "current visual and native-navigation projection. A missing task or a task from",
        "another Product run returns `found=false` with `follow_task_not_found` or",
        "`product_session_mismatch`. Clients must not substitute the current Follow ID",
        "when retrying an operation for an older task.",
        "",
        "Cancel with `POST /api/v1/follow/{follow_task_id}/cancel`, passing the original",
        "`product_session_id` plus optional `reason`, `request_id`, and `client_id`.",
        "Admission acknowledges a cancellation request; query the task until",
        "`terminal=true` for the outcome backed by its native child tasks. Cancelling an",
        "old or already terminal Follow never cancels navigation owned by a newer one.",
        "The legacy `mode=stop` command remains available for non-Follow visual search.",
        "When Follow is active it requires the same explicit task and Product-session",
        "reference and never falls back to a global visual-servo stop.",
        "",
        "The synchronous `LingTuClient` and `AsyncLingTuClient` expose the same",
        "`follow.start`, `follow.task`, and `follow.cancel` operations. Await methods",
        "on the asynchronous client. The start",
        "receipt exposes `follow_task_id` and `product_session_id` directly:",
        "",
        "```python",
        'run = robot.tasks.start_follow(target_id="track_7")',
        "# Use an ID from current perception, after the Product and calibration are ready.",
        "if run.accepted:",
        "    view = robot.tasks.get_follow_task(run.follow_task_id, run.product_session_id)",
        "    receipt = robot.tasks.cancel_follow_task(run.follow_task_id, run.product_session_id)",
        "```",
        "",
        "Existing flat SDK methods remain supported. `follow_person(target_id)`",
        "delegates to `follow.start`. `stop_following`",
        "requires both original IDs and delegates to scoped cancellation; it no longer",
        "sends an anonymous stop. Cancellation admission still does not prove native",
        "children have stopped; query the returned task until its actual terminal state.",
        "",
        "For a real Follow Product with operator video, select `tracking --variant camera`.",
        "The default `tracking/standard` keeps the existing perception/Follow graph and",
        "does not start the preview encoder. Native simulation supports `standard` only.",
        "",
        "## Generation",
        "",
        "```bash",
        "python tools/docs/extract_api_docs.py",
        "python tools/docs/extract_api_docs.py --check",
        "```",
        "",
        "## Navigation status",
        "",
        "`GET /api/v1/navigation/status` and SSE `navigation_status` expose one root-level",
        "v3 contract. It answers four independent questions:",
        "Implementation lives in `src/gateway/navigation/`: `routes.py` registers the",
        "HTTP surface, `status.py` evaluates admission and publishes SSE, `projection.py`",
        "projects the public axes, and `tasks.py` handles exact task/request queries.",
        "",
        "| Axis | Meaning |",
        "| --- | --- |",
        "| `task` | Current task phase: `IDLE`, planning, execution, recovery, explicit pause, or a terminal result |",
        "| `goal_admission` | Whether a new or replacement goal can be accepted |",
        "| `control` | Whether autonomy, the operator, or nobody owns control |",
        "| `motion` | Permission, observed motion, and stop-confirmation evidence |",
        "",
        "The task lifecycle is `IDLE -> PLANNING -> EXECUTING <-> RECOVERING`, with",
        "explicit `PAUSED` and terminal `SUCCESS`, `FAILED`, or `CANCELLED` branches.",
        "E-stop and InputGate holds change control or motion; they do not fake a task",
        "pause. An operator source Claim only reserves that source. Deliberate operator",
        "motion requests a native pause that retains the ordinary navigation destination;",
        "the task becomes PAUSED only after its stop confirmation. Release keeps it",
        "paused. Its TaskResume request releases the takeover hold and continues the same",
        "task; generic motion resume releases only the hold. HTTP pause/resume receipts",
        "acknowledge the request, while native task events confirm execution.",
        "Closing a Claim that never admitted operator input releases only that source's",
        "permission. It does not insert a stop into an autonomous route. Such Hold/Release",
        "receipts have `source_accepted=true` and `final_output_sequence=0`; they do not",
        "confirm a velocity publication or a physical stop. After actual takeover, the",
        "normal zero-output barrier still applies before the source is released.",
        "`QUIET` is only a fresh odometry observation and is not equivalent",
        "to `CONFIRMED`. Missing or stale evidence projects to `UNKNOWN`.",
        "",
        "There is no `operator_state` wrapper or second rich navigation status. Full",
        "blockers live at `/api/v1/readiness`, paths at `/api/v1/path`, native evidence at",
        "`/api/v1/navigation/dds_snapshot`, and exact terminal evidence at",
        "`/api/v1/navigation/tasks/{task_id}`.",
        "",
        "## Single targets and waypoint routes",
        "",
        "Scene exposes two target-entry modes: **单点** and **连续点**. The saved-map",
        "viewer keeps the existing single-target behavior. In **单点**, clicking A, B,",
        "then C before confirmation leaves C selected; it does not enqueue A or B.",
        "Scene automatically requests a candidate/path preview after target selection or",
        "an edit to its planning parameters. Candidate/path previews do not command",
        "motion. Clicking **开始导航** submits one",
        "external goal. A later accepted external goal replaces the current single",
        "goal; while planning is busy, the native pending slot keeps only the latest",
        "accepted goal. `planning_queued` is this single pending slot, not a FIFO.",
        "An active inspection task owns its route and rejects unrelated external goals.",
        "",
        "In **连续点**, Scene keeps an ordered route draft in browser memory, bound",
        "to the active `map_id + content_epoch`. Each map click appends one numbered",
        "`x/y/z` point. The operator can move points up or down, delete a point, or",
        "clear the draft. Continuous mode requires at least two points; use **单点** for",
        "one destination. A map identity change clears the draft. The numbered connector is",
        "an editing aid, not a planned path. Scene does not preview every leg before",
        "start; the native executor plans each leg when that point becomes current.",
        "Scene has no separate save action. The first",
        "start after creating or editing a draft calls the existing route save API once,",
        "then immediately submits that saved revision once through",
        "`POST /api/v1/inspection/tasks`. There is no new queue endpoint and no Web or",
        "Python waypoint executor. The persisted route remains in the route list, and an",
        "unchanged draft may reuse that route revision after the task terminates. The",
        "running route snapshot is not editable. The single-target speed setting does not",
        "configure this route; execution uses the active runtime path-following limit.",
        "",
        "After an ordinary Scene remount or full-page reload, re-entering **连续点** queries",
        "the active map's task projection and restores controls only when exactly one nonterminal task",
        "exists. It does not guess among multiple tasks. A disconnect preserves an already",
        "known `task_id`, disables its controls, and resumes polling it after reconnect.",
        "Gateway or native endpoint restart recovery remains bounded by the task projection:",
        "clients neither synthesize task state nor restart a route automatically.",
        "",
        "Routes may also be edited and saved in the inspection workbench. The native",
        "inspection executor advances points after arrival and configured point",
        "actions. One `task_id` identifies the whole task; `request_id` identifies",
        "each start/pause/resume/cancel command. Clients display status and submit",
        "commands; they do not advance points. Pause/resume preserves the current",
        "point, and cancellation ends the task through the existing stop handshake.",
        "A successful start/control response acknowledges submission, not execution",
        "or completion. Query the identified task for the native result.",
        "",
        "Routes support finite `loop_count` rounds and `loop_interval_s` between",
        "rounds. Native `WAITING_NEXT_LOOP` emits no goal during the interval.",
        "After an endpoint restart, unfinished work returns paused and requires",
        "explicit resume. The saved map and route revision must still match.",
        "An optional final point without an action is the standby destination;",
        "reaching it does not prove charging. Pause the task for manual charging",
        "and resume when ready. There is no automatic charging interface yet.",
        "",
        "Evidence completeness and equipment condition are separate report fields.",
        "`acceptance` describes capture/history integrity; `inspection_result`",
        "describes normal, anomaly, inconclusive, pending review or not inspected.",
        "Ordinary photo capture never proves equipment normal. Visual actions",
        "`capture:indicator_on/off/green/red/yellow` and `capture:door_open/closed`",
        "compare a visible observation against the selected expectation. The action",
        "uses one exact suffix, for example `capture:indicator_green`.",
        "The existing `vision_service_url` selects the shared image-capable service;",
        "an absent service or ambiguous image leaves the finding inconclusive and",
        "does not discard the photo. Configure a viewpoint with one identifiable",
        "target and validate recognition on recorded equipment images.",
        "`POST /api/v1/inspection/evidence/{evidence_id}/review` records a human",
        "`normal`, `anomaly` or `inconclusive` finding with an optional note. It",
        "retains the original capture/model result and never changes task motion.",
        "",
        "The Python SDK's `TaskClient` groups task operations under `robot.tasks`.",
        "Inspection uses `inspection_routes`, `start_inspection`, `inspection_task`,",
        "`pause_inspection`, `resume_inspection`, `cancel_inspection`, and",
        "`inspection_report`; AsyncLingTuClient delegates",
        "the same operations. The CLI exposes the corresponding `inspection-*`",
        "commands. The former `batch_go` Python waypoint loop has been removed.",
        "Use a stored native route instead; `navigation.go` and",
        "`navigation.wait_until_arrived` remain",
        "available for single-goal navigation.",
        "",
        "```python",
        "from lingtu.sdk import LingTuClient",
        "",
        "robot = LingTuClient(\"robot-host\")",
        "routes = robot.tasks.inspection_routes()",
        "# Select an existing route from routes before starting it.",
        "result = robot.tasks.start_inspection(\"saved-route-id\")",
        "if result.ok and result.task_id:",
        "    status = robot.tasks.inspection_task(result.task_id)",
        "```",
        "",
    ]
    return "\n".join(
        (
            *header,
            generate_mcp_tools_md(skills),
            "",
            generate_gateway_rest_md(routes),
        )
    )


# ── main ─────────────────────────────────────────────────────────────────

def _publish(path: Path, content: str, *, check: bool) -> bool:
    """Write one generated document or report whether it is current."""

    expected = content.rstrip() + "\n"
    if check:
        return path.is_file() and path.read_text(encoding="utf-8") == expected
    path.write_text(expected, encoding="utf-8")
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail without writing when generated API documentation is stale",
    )
    args = parser.parse_args(argv)
    stale: list[Path] = []

    # Task 1: MCP tools
    print("Extracting @skill methods...")
    skills = extract_skills()
    print(f"  Found {len(skills)} @skill methods in {len(set(s['file'] for s in skills))} files")

    # Task 2: Gateway REST
    print("Extracting Gateway REST routes...")
    routes = extract_gateway_routes()
    print(f"  Found {len(routes)} REST endpoints")

    api_doc = generate_api_md(skills, routes)
    if not _publish(DOCS_API, api_doc, check=args.check):
        stale.append(DOCS_API)
    print(f"  -> {DOCS_API}")

    # Console summary for reviewers and CI logs.
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[union-attr]

    print(f"\nSKILL_FILES={len(set(s['file'] for s in skills))}")
    print(f"SKILL_COUNT={len(skills)}")
    print(f"ROUTE_COUNT={len(routes)}")

    # Print skill method names for verification
    print("\n=== @skill methods ===")
    for s in skills:
        print(f"  {s['class_name']}.{s['method_name']}: {s['description']}")

    print("\n=== REST endpoints ===")
    for r in routes:
        print(f"  {r['method']} {r['path']}: {r['summary']}")

    if stale:
        print("\nGenerated API documentation is stale:")
        for path in stale:
            print(f"  - {path.relative_to(REPO_ROOT).as_posix()}")
        return 1
    if args.check:
        print("\nGenerated API documentation is current.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
