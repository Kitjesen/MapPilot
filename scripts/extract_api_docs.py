#!/usr/bin/env python3
"""Extract API documentation from LingTu source code.

Generates:
  docs/api/mcp_tools.md    — @skill methods from all Module files
  docs/api/gateway_rest.md  — REST endpoints from gateway route registrations
"""

from __future__ import annotations

import ast
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
GATEWAY = SRC / "gateway"
DOCS_API = REPO_ROOT / "docs" / "api"

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


# ── TASK 1: Extract @skill methods → mcp_tools.md ───────────────────────

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
                    "file": str(rel_path),
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
    """Generate mcp_tools.md from extracted skills."""
    lines = [
        "# MCP Tools (Auto-Discovered @skill Methods)",
        "",
        "> Auto-generated from `@skill` decorators across all Module files.",
        f"> Generated: {_now_iso()}",
        "",
        "These tools are auto-discovered by `MCPServerModule` and exposed via JSON-RPC",
        "at `http://<robot>:8090/mcp`. They are also available in the AgentLoop for",
        "multi-turn LLM tool calling.",
        "",
        "---",
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


# ── TASK 2: Extract Gateway REST routes → gateway_rest.md ───────────────

def extract_gateway_routes() -> list[dict]:
    """Extract REST route registrations from gateway/routes/ files.

    Returns list of dicts with: file, method, path, summary, response_model, action
    """
    routes: list[dict] = []
    routes_dir = GATEWAY / "routes"
    if not routes_dir.is_dir():
        return routes

    for py_file in sorted(routes_dir.rglob("*.py")):
        if py_file.name == "__init__.py":
            continue
        try:
            text = py_file.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue

        # Pattern: @app.get("/path", ...)  or @app.post("/path", ...)
        pattern = re.compile(
            r'@app\.(get|post|put|delete|patch)\s*\(\s*'
            r'"([^"]+)"'                                           # path string
            r'([\s\S]*?)(?=\n\s*(?:async\s+)?def\s+\w+\s*\()',    # up to def
            re.MULTILINE,
        )
        for match in pattern.finditer(text):
            http_method = match.group(1)
            path = match.group(2)
            decorator_body = match.group(3)

            # Extract summary
            summary_m = re.search(r'summary\s*=\s*"([^"]*)"', decorator_body)
            summary = summary_m.group(1) if summary_m else ""

            # Extract response model
            resp_m = re.search(r'response_model\s*=\s*(\w+(?:\[\w+\])?)', decorator_body)
            response_model = resp_m.group(1) if resp_m else ""

            # Find the function name after the decorator
            # Use a second pass to find the actual def
            def_match = re.search(
                rf'(?:async\s+)?def\s+(\w+)\s*\(',
                text[match.end():],
            )
            func_name = def_match.group(1) if def_match else ""

            rel_path = py_file.relative_to(REPO_ROOT)
            routes.append({
                "file": str(rel_path),
                "method": http_method.upper(),
                "path": path,
                "summary": summary,
                "response_model": response_model,
                "function": func_name,
            })

    # Also check gateway_module.py itself for inline routes
    gw_module = GATEWAY / "gateway_module.py"
    if gw_module.is_file():
        text = gw_module.read_text(encoding="utf-8")
        pattern = re.compile(
            r'@app\.(get|post|put|delete|patch)\s*\(\s*'
            r'"([^"]+)"'
            r'([\s\S]*?)(?=\n\s*(?:async\s+)?def\s+\w+\s*\()',
            re.MULTILINE,
        )
        for match in pattern.finditer(text):
            http_method = match.group(1)
            path = match.group(2)
            decorator_body = match.group(3)
            summary_m = re.search(r'summary\s*=\s*"([^"]*)"', decorator_body)
            summary = summary_m.group(1) if summary_m else ""
            resp_m = re.search(r'response_model\s*=\s*(\w+(?:\[\w+\])?)', decorator_body)
            response_model = resp_m.group(1) if resp_m else ""
            def_match = re.search(
                rf'(?:async\s+)?def\s+(\w+)\s*\(',
                text[match.end():],
            )
            func_name = def_match.group(1) if def_match else ""
            routes.append({
                "file": str(gw_module.relative_to(REPO_ROOT)),
                "method": http_method.upper(),
                "path": path,
                "summary": summary,
                "response_model": response_model,
                "function": func_name,
            })

    # Sort by path
    routes.sort(key=lambda r: r["path"])
    return routes


def generate_gateway_rest_md(routes: list[dict]) -> str:
    """Generate gateway_rest.md from extracted routes."""
    lines = [
        "# Gateway REST API",
        "",
        "> Auto-generated from route registrations in `src/gateway/routes/`.",
        f"> Generated: {_now_iso()}",
        "",
        "The GatewayModule serves these endpoints via FastAPI on port 5050.",
        "",
        "---",
        "",
        "## Summary",
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
            lines.append(
                f"  - `{entry['method']} {entry['path']}` — {entry['summary']}"
            )
    lines.append("")
    lines.append("---")
    lines.append("")

    # Detailed sections by file
    for filepath in sorted(by_file):
        lines.append(f"## {filepath}")
        lines.append("")

        for entry in by_file[filepath]:
            lines.append(f"### `{entry['method']} {entry['path']}`")
            if entry["summary"]:
                lines.append(f"**Summary:** {entry['summary']}")
            if entry["response_model"]:
                lines.append(f"**Response model:** `{entry['response_model']}`")
            lines.append(f"**Handler:** `{entry['function']}`")
            lines.append("")

    return "\n".join(lines)


# ── helpers ──────────────────────────────────────────────────────────────

def _now_iso() -> str:
    from datetime import datetime
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# ── main ─────────────────────────────────────────────────────────────────

def main():
    DOCS_API.mkdir(parents=True, exist_ok=True)

    # Task 1: MCP tools
    print("Extracting @skill methods...")
    skills = extract_skills()
    print(f"  Found {len(skills)} @skill methods in {len(set(s['file'] for s in skills))} files")

    mcp_doc = generate_mcp_tools_md(skills)
    mcp_path = DOCS_API / "mcp_tools.md"
    mcp_path.write_text(mcp_doc, encoding="utf-8")
    print(f"  → {mcp_path}")

    # Task 2: Gateway REST
    print("Extracting Gateway REST routes...")
    routes = extract_gateway_routes()
    print(f"  Found {len(routes)} REST endpoints")

    rest_doc = generate_gateway_rest_md(routes)
    rest_path = DOCS_API / "gateway_rest.md"
    rest_path.write_text(rest_doc, encoding="utf-8")
    print(f"  → {rest_path}")

    # Task 3: Output summary for ROADMAP update
    # (to be consumed by the caller)
    import sys
    sys.stdout.reconfigure(encoding='utf-8')  # type: ignore[union-attr]

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


if __name__ == "__main__":
    main()
