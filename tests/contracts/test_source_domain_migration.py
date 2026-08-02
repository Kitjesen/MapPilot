from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

SCAN_ROOTS = (
    ROOT / "src",
    ROOT / "cli",
    ROOT / "scripts",
    ROOT / "tools",
    ROOT / "sim",
    ROOT / "tests",
)

ACTIVE_TEXT_ROOTS = (*SCAN_ROOTS, ROOT / ".github")

REMOVED_TOP_LEVEL_PACKAGES = (
    "core",
    "base_autonomy",
    "global_planning",
    "semantic",
    "external_adapters",
    "webrtc",
)

REMOVED_IMPORT_ROOTS = (*REMOVED_TOP_LEVEL_PACKAGES,
    "nav.core",
    "kernels.nav_core",
    "_nav_core",
)

SKIPPED_PATH_PARTS = {
    "__pycache__",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "vendor",
    "legacy_ros",
}

ACTIVE_TEXT_SUFFIXES = {
    "",
    ".cfg",
    ".cmake",
    ".cmd",
    ".ini",
    ".md",
    ".ps1",
    ".py",
    ".sh",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
}

OLD_PATH_TOKENS = (
    "src/core",
    "src/base_autonomy",
    "src/global_planning",
    "src/semantic",
    "src/external_adapters",
    "src/webrtc",
    "src/nav/core",
)

OLD_NAME_TOKENS = (
    "NavigationModule",
    "GoalServiceModule",
    "MapManagerModule",
    "LocalPlannerModule",
    "PathFollowerModule",
    "TerrainModule",
    "SafetyRingModule",
    "CmdVelMux",
    "GlobalPlannerService",
    "navigation_module",
    "global_planner_service",
    "cmd_vel_mux_module",
    "safety_ring_module",
    "nav.core",
    "kernels.nav_core",
    "_nav_core",
)

TOKEN_SCAN_ALLOWLIST = {
    "tests/contracts/test_runtime_architecture_boundaries.py",
    "tests/contracts/test_source_domain_migration.py",
    # This file describes an external CMU Unity workspace layout, not LingTu src.
    "sim/scripts/launch_cmu_unity_baseline.sh",
    # Explains, by name, that src/semantic/ was retired and stays deleted -- the
    # token appears as historical/explanatory prose, not a live reference.
    "src/perception/README.md",
}

IMPORT_LINE_RE = re.compile(
    r"^\s*(?:from\s+(?P<from_module>[A-Za-z_][\w.]*)\s+import\b|"
    r"import\s+(?P<import_list>.+))"
)


def _repo_rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def _is_removed_import(module: str) -> bool:
    return any(module == root or module.startswith(f"{root}.") for root in REMOVED_IMPORT_ROOTS)


def _imported_modules_from_line(line: str) -> list[str]:
    match = IMPORT_LINE_RE.match(line)
    if not match:
        return []
    from_module = match.group("from_module")
    if from_module:
        return [from_module]
    import_list = match.group("import_list") or ""
    modules: list[str] = []
    for item in import_list.split(","):
        module = item.strip().split(" as ", 1)[0].strip()
        if module:
            modules.append(module)
    return modules


def test_removed_top_level_source_packages_stay_removed() -> None:
    present = [
        _repo_rel(ROOT / "src" / package)
        for package in REMOVED_TOP_LEVEL_PACKAGES
        if (ROOT / "src" / package).exists()
    ]

    assert present == [], "Removed source package(s) found:\n" + "\n".join(present)


def test_removed_source_domain_imports_do_not_return() -> None:
    offenders: list[str] = []
    this_file = Path(__file__).resolve()
    for root in SCAN_ROOTS:
        if not root.exists():
            continue
        for path in root.rglob("*.py"):
            if path.resolve() == this_file:
                continue
            if set(path.relative_to(ROOT).parts) & SKIPPED_PATH_PARTS:
                continue
            source = path.read_text(encoding="utf-8", errors="ignore")
            for line_number, line in enumerate(source.splitlines(), start=1):
                for module in _imported_modules_from_line(line):
                    if _is_removed_import(module):
                        offenders.append(f"{_repo_rel(path)}:{line_number}: {line.strip()}")

    assert offenders == [], "Removed source-domain import(s) found:\n" + "\n".join(offenders)


def test_removed_source_domain_tokens_do_not_return_to_active_surfaces() -> None:
    offenders: list[str] = []
    forbidden = OLD_PATH_TOKENS + OLD_NAME_TOKENS

    for root in ACTIVE_TEXT_ROOTS:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            rel = _repo_rel(path)
            if rel in TOKEN_SCAN_ALLOWLIST:
                continue
            if set(path.relative_to(ROOT).parts) & SKIPPED_PATH_PARTS:
                continue
            if path.suffix.lower() not in ACTIVE_TEXT_SUFFIXES:
                continue
            source = path.read_text(encoding="utf-8", errors="ignore")
            normalized = source.replace("\\", "/")
            for token in forbidden:
                if token in normalized:
                    offenders.append(f"{rel} contains {token}")

    assert offenders == [], "Removed source-domain token(s) found:\n" + "\n".join(offenders)
