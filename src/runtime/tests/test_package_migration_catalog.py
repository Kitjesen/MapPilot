from __future__ import annotations

from pathlib import Path

from runtime.migration_catalog import (
    PACKAGE_MIGRATION_TARGETS,
    target_for_package,
    targets_by_phase,
)


REPO = Path(__file__).resolve().parents[3]


def test_migration_catalog_covers_current_src_packages():
    src_packages = {
        str(path.relative_to(REPO)).replace("\\", "/")
        for path in (REPO / "src").iterdir()
        if path.is_dir() and not path.name.startswith("__")
    }
    catalog_packages = {target.package for target in PACKAGE_MIGRATION_TARGETS}

    assert src_packages <= catalog_packages


def test_migration_catalog_covers_top_level_product_areas():
    catalog_packages = {target.package for target in PACKAGE_MIGRATION_TARGETS}

    assert {
        "calibration",
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
