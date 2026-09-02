from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL_ROOTS = (
    REPO_ROOT / "sim" / "catalog",
    REPO_ROOT / "sim" / "runtime",
    REPO_ROOT / "sim" / "adapters",
)
FORBIDDEN_PREFIXES = (
    "sim.compat.engine",
    "sim.scripts",
    "drivers.sim",
    "src.drivers.sim",
    "runtime.transport.dds",
    "runtime.transport.shm",
    "src.runtime.transport.dds",
    "src.runtime.transport.shm",
)


def _absolute_import(path: Path, node: ast.ImportFrom) -> str:
    if node.level == 0:
        return node.module or ""
    relative = path.relative_to(REPO_ROOT).with_suffix("")
    package = list(relative.parts[:-1])
    remove = node.level - 1
    if remove > len(package):
        return node.module or ""
    prefix = package[: len(package) - remove]
    if node.module:
        prefix.extend(node.module.split("."))
    return ".".join(prefix)


def test_canonical_simulation_layers_do_not_import_legacy_runtime_paths() -> None:
    violations: list[str] = []
    for root in CANONICAL_ROOTS:
        for path in sorted(root.rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                imports: tuple[str, ...]
                if isinstance(node, ast.Import):
                    imports = tuple(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    imports = (_absolute_import(path, node),)
                else:
                    continue
                for imported in imports:
                    if any(
                        imported == prefix or imported.startswith(prefix + ".")
                        for prefix in FORBIDDEN_PREFIXES
                    ):
                        relative = path.relative_to(REPO_ROOT)
                        violations.append(f"{relative}:{node.lineno} imports {imported}")

    assert violations == []
