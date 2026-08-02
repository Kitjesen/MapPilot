from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME_LOOKUP_FILES = (
    REPO_ROOT / "src/gateway/services/map_service.py",
    REPO_ROOT / "src/decision/modules/semantic_planner.py",
)
MAP_SERVICE_MODULE = REPO_ROOT / "src/maps/modules/service.py"


def test_runtime_map_service_lookups_do_not_restore_removed_module_alias() -> None:
    violations: list[str] = []
    for path in RUNTIME_LOOKUP_FILES:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        if any(
            isinstance(node, ast.Constant) and node.value == "MapsModule"
            for node in ast.walk(tree)
        ):
            violations.append(path.relative_to(REPO_ROOT).as_posix())

    assert violations == []


def test_map_service_registry_does_not_restore_manager_alias() -> None:
    tree = ast.parse(
        MAP_SERVICE_MODULE.read_text(encoding="utf-8"),
        filename=str(MAP_SERVICE_MODULE),
    )

    removed_registrations = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "register"
        and len(node.args) >= 2
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "map"
        and isinstance(node.args[1], ast.Constant)
        and node.args[1].value == "manager"
    ]

    assert removed_registrations == []
