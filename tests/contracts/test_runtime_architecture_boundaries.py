"""Contracts for Product, Host, adapter, Module, and native-kernel boundaries."""

from __future__ import annotations

from pathlib import Path

from tools.validate.validate_architecture_boundaries import validate

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"

CORE_RUNTIME_BOUNDARY_SOURCES = {
    "runtime_interface": SRC / "runtime" / "runtime_interface.py",
    "runtime_policy": SRC / "runtime" / "runtime_policy.py",
}

BLUEPRINT_COMPAT_ADAPTER_SOURCES = {
    "driver_runtime": SRC
    / "runtime"
    / "adapters"
    / "driver_runtime.py",
}

STACK_COMPOSITION_SOURCES_WITH_ADAPTER_BOUNDARY = {
    "driver_stack": SRC / "runtime" / "blueprints" / "stacks" / "driver.py",
    "gateway_stack": SRC / "runtime" / "blueprints" / "stacks" / "gateway.py",
    "maps_stack": SRC / "runtime" / "blueprints" / "stacks" / "maps.py",
    "navigation_stack": SRC / "runtime" / "blueprints" / "stacks" / "navigation.py",
    "perception_stack": SRC / "runtime" / "blueprints" / "stacks" / "perception.py",
    "slam_stack": SRC / "runtime" / "blueprints" / "stacks" / "slam.py",
}

REMOVED_COMPATIBILITY_FACADE_PATHS = (
    SRC / "exploration" / "tare" / "ros2_bridge.py",
    SRC / "semantic" / "perception" / "perception_publishers.py",
    SRC / "semantic" / "reconstruction" / "bag_reader.py",
    SRC / "perception" / "perception_publishers.py",
    SRC / "perception" / "reconstruction" / "bag_reader.py",
    SRC / "runtime" / "blueprints" / "runtime_endpoint.py",
    SRC / "runtime" / "blueprints" / "catalog",
)

REMOVED_DRIVER_CATALOG_PATHS = (
    ROOT / "config" / "robots" / "thunder.yaml",
)

CANONICAL_DRIVER_CATALOG_PATHS = (
    ROOT / "config" / "driver_backends" / "thunder.yaml",
    SRC / "drivers" / "backends.py",
    SRC / "drivers" / "catalog.py",
)


def _repo_rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def _assert_sources_exist_under_owner(sources: dict[str, Path], owner: Path) -> None:
    missing = [
        f"{name}: {_repo_rel(path)}"
        for name, path in sources.items()
        if not path.exists()
    ]
    assert missing == [], "Missing layer boundary source(s):\n" + "\n".join(missing)

    owner_rel = _repo_rel(owner)
    for name, path in sources.items():
        rel = _repo_rel(path)
        assert path == owner or owner in path.parents, (
            f"{name} moved outside {owner_rel} ownership boundary: {rel}"
        )
        assert "/tests/" not in rel and not rel.startswith("tests/"), (
            f"{name} must be production boundary code, not test-only code: {rel}"
        )


def test_architecture_validator_is_contract_entrypoint() -> None:
    violations, scanned = validate()

    assert scanned > 0
    assert violations == [], "\n".join(violations)


def test_runtime_contract_imports_through_core_boundary() -> None:
    from runtime.runtime_interface import (
        FIELD_DATA_SOURCE,
        runtime_contract_manifest,
        runtime_data_flow_topics,
    )

    manifest = runtime_contract_manifest()

    assert manifest["schema_version"] == "lingtu.runtime_interface.v1"
    assert FIELD_DATA_SOURCE in manifest["resolved_runtime_data_flow"]
    assert tuple(manifest["runtime_data_flow_topics"][FIELD_DATA_SOURCE]) == runtime_data_flow_topics(
        FIELD_DATA_SOURCE
    )


def test_runtime_boundary_sources_remain_core_owned() -> None:
    missing = [
        f"{name}: {_repo_rel(path)}"
        for name, path in CORE_RUNTIME_BOUNDARY_SOURCES.items()
        if not path.exists()
    ]
    assert missing == [], "Missing runtime boundary source(s):\n" + "\n".join(missing)

    for name, path in CORE_RUNTIME_BOUNDARY_SOURCES.items():
        rel = _repo_rel(path)
        assert rel.startswith("src/runtime/"), f"{name} moved outside core boundary: {rel}"
        assert "/tests/" not in rel and not rel.startswith("tests/"), (
            f"{name} must be production core boundary code, not test-only code: {rel}"
        )


def test_driver_backend_catalog_replaces_retired_hardware_catalog() -> None:
    assert [
        _repo_rel(path)
        for path in REMOVED_DRIVER_CATALOG_PATHS
        if path.exists()
    ] == []
    assert [
        _repo_rel(path)
        for path in CANONICAL_DRIVER_CATALOG_PATHS
        if not path.exists()
    ] == []

    source = CANONICAL_DRIVER_CATALOG_PATHS[0].read_text(encoding="utf-8")
    assert "schema_version: lingtu.driver_catalog.v1" in source
    for key in ("backends", "protocols", "modules"):
        assert f"{key}:" in source


def test_directory_stage_keeps_lcm_adapter_layer_removed() -> None:
    assert not (SRC / "runtime" / "adapters" / ("lc" + "m")).exists()


def test_compat_adapter_helpers_live_in_blueprint_adapter_layer() -> None:
    _assert_sources_exist_under_owner(
        BLUEPRINT_COMPAT_ADAPTER_SOURCES,
        SRC / "runtime" / "adapters",
    )


def test_stack_composition_files_do_not_own_ros_fallback_strings() -> None:
    forbidden_marker = "runtime.adapters.ros2"
    offenders = [
        _repo_rel(path)
        for path in STACK_COMPOSITION_SOURCES_WITH_ADAPTER_BOUNDARY.values()
        if path.exists() and forbidden_marker in path.read_text(encoding="utf-8")
    ]

    assert offenders == [], (
        "Stack composition files must call explicit adapter helpers instead of "
        "owning ROS fallback strings:\n"
        + "\n".join(offenders)
    )


def test_old_compatibility_facades_are_removed() -> None:
    assert [
        _repo_rel(path)
        for path in REMOVED_COMPATIBILITY_FACADE_PATHS
        if path.exists()
    ] == []
