from __future__ import annotations

from pathlib import Path

from tools.migration.catalog import (
    PACKAGE_MIGRATION_TARGETS,
    target_for_package,
    targets_by_phase,
)

REPO = Path(__file__).resolve().parents[3]
STALE_RUNTIME_PATHS = (
    "src/runtime/dimos_gap.py",
    "src/runtime/dimos_runtime_dataflow.py",
    "src/runtime/efficiency_status.py",
    "src/runtime/gateway_runtime_acceptance.py",
    "src/runtime/inspection_acceptance.py",
    "src/runtime/migration_catalog.py",
    "src/runtime/product_field_check.py",
    "src/runtime/runtime_evidence.py",
    "src/runtime/runtime_validation_gates.py",
    "src/runtime/transport/dual.py",
    "src/nav/services/plan/global_planner/direct.py",
    "src/nav/services/plan/global_planner/algorithm/direct_path.py",
)
STALE_DIAGNOSTICS_RUNTIME_PATHS = (
    "src/diagnostics/field/dimos_gap.py",
    "src/diagnostics/field/dimos_runtime_dataflow.py",
    "src/diagnostics/field/efficiency_status.py",
    "src/diagnostics/field/migration_catalog.py",
)
STALE_PATH_SCAN_ROOTS = (
    "cli",
    "config",
    "scripts",
    "sim",
    "src",
    "tools",
)
STALE_PATH_TEXT_SUFFIXES = {
    "",
    ".cfg",
    ".cmake",
    ".h",
    ".hpp",
    ".ini",
    ".json",
    ".md",
    ".py",
    ".ps1",
    ".sh",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
}
STALE_PATH_SKIP_DIRS = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    "__pycache__",
    "build",
    "build-release",
    "dist",
    "node_modules",
    "vendor",
}


def test_migration_catalog_covers_current_src_packages():
    src_packages = {
        str(path.relative_to(REPO)).replace("\\", "/")
        for path in (REPO / "src").iterdir()
        if path.is_dir() and not path.name.startswith("__")
    }
    catalog_packages = {target.package for target in PACKAGE_MIGRATION_TARGETS}

    assert src_packages <= catalog_packages


def test_non_doc_surfaces_do_not_reference_stale_runtime_paths():
    offenders: list[str] = []
    for root_name in STALE_PATH_SCAN_ROOTS:
        root = REPO / root_name
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if (
                not path.is_file()
                or any(part in STALE_PATH_SKIP_DIRS for part in path.parts)
                or "tests" in path.parts
                or path.suffix.lower() not in STALE_PATH_TEXT_SUFFIXES
            ):
                continue
            try:
                text = path.read_text(encoding="utf-8-sig")
            except UnicodeDecodeError:
                continue
            for stale in STALE_RUNTIME_PATHS:
                if stale in text:
                    rel = path.relative_to(REPO).as_posix()
                    offenders.append(f"{rel}: {stale}")
            for stale in STALE_DIAGNOSTICS_RUNTIME_PATHS:
                if stale in text:
                    rel = path.relative_to(REPO).as_posix()
                    offenders.append(f"{rel}: {stale}")

    assert offenders == []


def test_migration_catalog_covers_top_level_product_areas():
    catalog_packages = {target.package for target in PACKAGE_MIGRATION_TARGETS}

    assert {
        "tools/calibration",
        "cli",
        "config",
        "launch",
        "scripts",
        "sim",
        "web",
    } <= catalog_packages


def test_dart_is_reserved_for_client_surfaces():
    dart_targets = {
        target.package: target.target_form
        for target in PACKAGE_MIGRATION_TARGETS
        if target.target_form.startswith("dart")
    }

    assert dart_targets == {
        "src/lingtu": "dart_sdk",
        "src/lingtu/sdk": "dart_sdk",
        "web": "dart_app",
    }


def test_rust_runtime_does_not_precede_kernel_and_adapter_foundations():
    assert target_for_package("src/kernels").phase < target_for_package("src/runtime").phase
    assert target_for_package("src/*/adapters").phase < target_for_package("src/runtime").phase
    assert target_for_package("src/drivers").phase < target_for_package("src/runtime").phase


def test_heavy_model_and_native_packages_are_not_early_runtime_rewrites():
    for package in (
        "src/perception",
        "src/decision",
        "src/nav/services/plan",
        "src/nav/exploration",
        "src/gateway/media",
    ):
        target = target_for_package(package)
        assert target.phase >= 5
        assert target.target_form in {
            "native_engine_process",
            "python_compat_until_replaced",
        }


def test_config_and_launch_are_schema_or_compatibility_contracts():
    assert target_for_package("config").target_form == "schema_only"
    assert target_for_package("launch").target_form == "schema_only"
    assert targets_by_phase(0)
