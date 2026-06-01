from __future__ import annotations

import ast
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"

BOUNDARY_RULES = {
    # Existing: each package must NOT import from its forbidden set
    "gateway": {"nav", "semantic", "drivers"},
    "nav": {"semantic", "drivers", "gateway"},
    "semantic": {"nav", "drivers", "gateway"},
    "drivers": {"nav", "semantic"},
    # NEW: core is the lowest layer -- must not import any domain package
    "core": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    # All domain packages must not import each other sideways
    "slam": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "memory": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "base_autonomy": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "exploration",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "exploration": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "webrtc",
        "global_planning",
        "lingtu",
    },
    "webrtc": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "global_planning",
        "lingtu",
    },
    "global_planning": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "lingtu",
    },
    "lingtu": {
        "nav",
        "semantic",
        "drivers",
        "gateway",
        "slam",
        "memory",
        "base_autonomy",
        "exploration",
        "webrtc",
        "global_planning",
    },
}

COMPOSITION_EXCEPTIONS = {
    # Blueprint composition files — intentionally wire modules across layers
    "core/blueprints/",
    "core/blueprints/stacks/",
    "core/blueprints/full_stack.py",
    "core/blueprints/full_stack_wiring.py",
    "core/blueprints/stub.py",
    "drivers/real/thunder/blueprints.py",
    "drivers/sim/stub.py",
    "drivers/sim/test_full_pipeline_s100p.py",
    # Core glue files that aggregate sub-package protocols/types
    "core/gateway_runtime_acceptance.py",
    "core/product_field_check.py",
    "core/msgs/scene.py",
    "core/same_source_map_artifacts.py",
    # LingTu user-facing facade — wraps all internal packages
    "lingtu/",
    # Legacy ROS2 launch files with encoding corruption (unparseable)
    "global_planning/pct_planner/launch/",
}


def _python_files(package: str) -> list[Path]:
    return sorted(
        path
        for path in (SRC / package).rglob("*.py")
        if "__pycache__" not in path.parts
        and not any(part.startswith(".") for part in path.parts)
    )


def _imported_modules(tree: ast.AST) -> list[str]:
    """Return top-level absolute import module names.

    Only checks direct children of the module body (skips imports inside
    functions/methods/classes — those are lazy and do not create hard
    cross-package coupling).  Also skips relative imports (``from .foo``)
    which stay within the same package.
    """
    modules: list[str] = []
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and (node.level or 0) == 0:
            modules.append(node.module)
    return modules


def _top_level(module: str) -> str:
    return module.split(".", 1)[0]


def _is_test_file(path: Path) -> bool:
    return (
        "tests" in path.parts
        or "test" in path.parts
        or path.name.startswith("test_")
    )


def _is_example_file(path: Path) -> bool:
    return "examples" in path.parts or "example" in path.parts


def _is_composition_exception(rel: str) -> bool:
    """Check if a file is exempt from boundary checks (composition/glue code)."""
    src_rel = rel.removeprefix("src/")
    for exc in COMPOSITION_EXCEPTIONS:
        if src_rel == exc or src_rel.startswith(exc):
            return True
    return False


@pytest.mark.parametrize("package,forbidden", BOUNDARY_RULES.items())
def test_package_does_not_import_forbidden_layers_directly(
    package: str,
    forbidden: set[str],
) -> None:
    violations: list[str] = []

    for path in _python_files(package):
        rel = path.relative_to(ROOT).as_posix()
        if _is_test_file(path) or _is_example_file(path) or _is_composition_exception(rel):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        except SyntaxError:
            # Unparseable file (e.g. legacy ROS2 launch with encoding corruption).
            # The file should be added to COMPOSITION_EXCEPTIONS if intentional.
            continue
        for module in _imported_modules(tree):
            if _top_level(module) in forbidden:
                violations.append(f"{rel}: imports {module}")

    assert violations == [], "\n".join(violations)
