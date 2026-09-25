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
                if not any(
                    isinstance(d, ast.Name) and d.id == "skill"
                    for d in item.decorator_list
                ):
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


def generate_mcp_tools_md(skills: list[dict]) -> str:
    """Generate the MCP section of the API reference."""
    lines = [
        "## MCP tools",
        "",
        "This inventory is generated from `@skill` decorators. The Host exposes",
        "these methods through MCP JSON-RPC on port 8090 and to the Agent loop.",
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

        for entry in entries:
            method = entry["method_name"]
            class_name = entry["class_name"]
            desc = entry["description"]
            ret = entry["return_type"]
            params = entry["params"]

            lines.append(f"### `{method}`")
            lines.append(f"**Module:** `{class_name}`")
            if desc:
                lines.append(f"**Description:** {desc}")
            if ret:
                lines.append(f"**Return type:** `{ret}`")
            if params:
                lines.append("**Parameters:**")
                lines.append("| Parameter | Type |")
                lines.append("|-----------|------|")
                for p in params:
                    ptype = p.get("type", "")
                    lines.append(f"| `{p['name']}` | `{ptype}` |")
            else:
                lines.append("**Parameters:** None")
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
        "### Route summary",
        "",
    ]

    # Table of contents by file
    from collections import defaultdict
    by_file: dict[str, list[dict]] = defaultdict(list)
    for r in routes:
        by_file[r["file"]].append(r)

    for filepath in sorted(by_file):
        lines.append(f"- **{filepath}**:")
        for entry in by_file[filepath]:
            summary = f" — {entry['summary']}" if entry["summary"] else ""
            lines.append(f"  - `{entry['method']} {entry['path']}`{summary}")
    lines.append("")
    # Detailed sections by file
    for filepath in sorted(by_file):
        lines.append(f"### {filepath}")
        lines.append("")

        for entry in by_file[filepath]:
            lines.append(f"#### `{entry['method']} {entry['path']}`")
            if entry["summary"]:
                lines.append(f"**Summary:** {entry['summary']}")
            if entry["response_model"]:
                lines.append(f"**Response model:** `{entry['response_model']}`")
            lines.append(f"**Handler:** `{entry['function']}`")
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
        "This file combines the small maintained integration contract with",
        "inventories generated from source. Do not hand-edit the generated",
        "method or route lists; run `python tools/docs/extract_api_docs.py`.",
        "",
        "## Integration boundaries",
        "",
        "| Surface | Endpoint | Owner |",
        "| --- | --- | --- |",
        "| REST and SSE | `http://<host>:5050` | Gateway routes and projections |",
        "| MCP | `http://<host>:8090/mcp` | `@skill` methods discovered in the Host |",
        "| Python SDK/CLI | `lingtu-sdk` and `lingtu.sdk` | Typed client facade |",
        "| Camera Web media | go2rtc WHEP with Gateway JPEG fallback | Optional media sidecar plus Gateway |",
        "",
        "Gateway submits typed intent and projects runtime facts. It does not",
        "own Product lifecycle, maps, planning, final motion, or hardware.",
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
        "Map saves are durable operations. A successful admission is not a completed",
        "save: poll `GET /api/v1/maps/operations/{operation_id}` and inspect",
        "`operation.state`. A client wait timeout or a lost status response leaves the",
        "outcome unconfirmed; continue querying that operation instead of submitting",
        "another save. A `SUCCEEDED` detail includes the available",
        "`operation.processing.optimization` and `operation.processing.cleanup`",
        "summaries (`performed`, `success`, `reason_code`). Optimization can successfully",
        "skip when measurement constraints are insufficient, so save success does not",
        "imply completed optimization or navigation readiness. Lightweight operation",
        "lists omit these summaries; fetch the individual detail. Successful same-name",
        "saves replace the old map, and saves do not activate a map or switch Product.",
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
        "E-stop, takeover, and InputGate holds change control or motion; they do not fake",
        "a task pause. `QUIET` is only a fresh odometry observation and is not equivalent",
        "to `CONFIRMED`. Missing or stale evidence projects to `UNKNOWN`.",
        "",
        "There is no `operator_state` wrapper or second rich navigation status. Full",
        "blockers live at `/api/v1/readiness`, paths at `/api/v1/path`, native evidence at",
        "`/api/v1/navigation/dds_snapshot`, and exact terminal evidence at",
        "`/api/v1/navigation/tasks/{task_id}`.",
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
