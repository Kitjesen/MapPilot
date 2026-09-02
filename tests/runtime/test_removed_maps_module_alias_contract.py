from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_LOOKUP_FILES = (
    REPO_ROOT / "src/gateway/services/mapd_transport.py",
    REPO_ROOT / "src/decision/modules/semantic_planner.py",
)
REMOVED_MAPS_MODULE = REPO_ROOT / "src/maps/modules/service.py"


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


def test_python_maps_module_stays_physically_removed() -> None:
    assert not REMOVED_MAPS_MODULE.exists()
