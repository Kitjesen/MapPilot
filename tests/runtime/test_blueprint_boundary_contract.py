from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BLUEPRINT_ROOT = ROOT / "src" / "runtime" / "blueprints"

FORBIDDEN_IMPORT_ROOTS = {
    "fastapi",
    "httpx",
    "requests",
    "rclpy",
    "socket",
    "subprocess",
    "urllib",
    "uvicorn",
}

FORBIDDEN_CALLS = {
    "os.system",
    "subprocess.Popen",
    "subprocess.call",
    "subprocess.check_call",
    "subprocess.check_output",
    "subprocess.run",
}


def _module_name(node: ast.AST) -> str:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        base = _module_name(node.value)
        return f"{base}.{node.attr}" if base else node.attr
    return ""


def _iter_python_files() -> list[Path]:
    return sorted(path for path in BLUEPRINT_ROOT.rglob("*.py") if "__pycache__" not in path.parts)


def test_blueprints_stay_composition_only() -> None:
    """Blueprints compose modules; external I/O belongs in adapters/services."""

    violations: list[str] = []
    for path in _iter_python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        rel = path.relative_to(ROOT).as_posix()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".", 1)[0]
                    if root in FORBIDDEN_IMPORT_ROOTS:
                        violations.append(f"{rel}: imports forbidden boundary module {alias.name}")
            elif isinstance(node, ast.ImportFrom):
                module = node.module or ""
                root = module.split(".", 1)[0]
                if root in FORBIDDEN_IMPORT_ROOTS:
                    violations.append(f"{rel}: imports forbidden boundary module {module}")
            elif isinstance(node, ast.Call):
                name = _module_name(node.func)
                if name in FORBIDDEN_CALLS:
                    violations.append(f"{rel}: calls forbidden boundary function {name}")

    assert violations == []
