"""Behavioral contracts for the root simulation layout."""

from __future__ import annotations

import importlib
import subprocess
import sys
from pathlib import Path

from sim.catalog.resolver import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "sim"

OWNED_ROOTS = (
    "packages",
    "sessions",
    "catalog",
    "contracts",
    "runtime",
    "adapters",
    "compat",
    "diagnostics",
    "evaluation",
    "distribution",
    "tools",
    "scripts",
    "tests",
)

RETIRED_ROOTS = (
    "assets",
    "controllers",
    "datasets",
    "engine",
    "external_scenes",
    "fixtures",
    "following",
    "importers",
    "maps",
    "planning",
    "presets",
    "qualifications",
    "robots",
    "scenarios",
    "schemas",
    "sensor_rigs",
    "sensors",
    "toolchains",
    "validation",
    "worlds",
)

CANONICAL_MUJOCO_ENTRYPOINTS = (
    "native_dds_sensors.py",
    "native_navigation_acceptance.py",
    "native_control_mode_acceptance.py",
    "product_acceptance.py",
    "map_native_acceptance.py",
    "explore_native_acceptance.py",
    "inspection_native_acceptance.py",
    "teleop_native_acceptance.py",
    "teleop_avoid_native_acceptance.py",
    "record_thunderv4_mid360_policy.py",
    "record_thunderv4_stair_showcase.py",
    "continuous_mapping_quality_gate.py",
    "saved_map_relocalization.py",
    "sunrise_mapping.py",
)


def test_owned_roots_contain_real_files() -> None:
    for name in OWNED_ROOTS:
        root = SIM_ROOT / name
        assert root.is_dir(), name
        assert any(path.is_file() for path in root.rglob("*")), name


def test_owned_roots_are_self_documenting() -> None:
    index = (SIM_ROOT / "README.md").read_text(encoding="utf-8")

    for name in OWNED_ROOTS:
        readme = SIM_ROOT / name / "README.md"
        assert readme.is_file(), name
        assert f"{name}/README.md" in index, name


def test_catalog_has_one_package_root() -> None:
    resolver = CatalogResolver.from_repository(REPO_ROOT)

    assert resolver.catalog_roots == ((SIM_ROOT / "packages").resolve(),)
    assert resolver.records


def test_runtime_uses_role_named_paths() -> None:
    assert (SIM_ROOT / "runtime" / "physics" / "CMakeLists.txt").is_file()
    assert (SIM_ROOT / "runtime" / "coordinator" / "coordinator.py").is_file()
    assert (SIM_ROOT / "runtime" / "visual" / "RobotSimUE" / "RobotSimUE.uproject").is_file()
    assert (SIM_ROOT / "adapters" / "dds" / "CMakeLists.txt").is_file()


def test_public_python_surfaces_import() -> None:
    modules = (
        "sim.catalog.importers",
        "sim.compat.engine.mujoco",
        "sim.adapters.gazebo",
        "sim.diagnostics",
        "sim.evaluation.navigation_replay",
        "sim.tools.planning.octoplanner3d_route_viz",
    )

    for name in modules:
        assert importlib.import_module(name) is not None


def test_stable_mujoco_entrypoints_remain_in_place() -> None:
    scripts_root = SIM_ROOT / "scripts" / "mujoco"

    for name in CANONICAL_MUJOCO_ENTRYPOINTS:
        assert (scripts_root / name).is_file(), name

    sensor_bridge = importlib.import_module("sim.scripts.mujoco.native_dds_sensors")
    assert callable(sensor_bridge._relative_times_for_scan)
    assert callable(sensor_bridge._physical_rolling_scan_from_samples)


def test_recording_entrypoint_help_does_not_require_runtime_media_dependencies() -> None:
    script = SIM_ROOT / "scripts" / "mujoco" / "record_thunderv4_mid360_policy.py"

    completed = subprocess.run(
        [sys.executable, "-B", str(script), "--help"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr


def test_retired_roots_are_absent() -> None:
    for name in RETIRED_ROOTS:
        assert not (SIM_ROOT / name).exists(), name


def test_packages_own_manifests_and_assets() -> None:
    packages = SIM_ROOT / "packages"

    assert any((packages / "robots").rglob("robot.package.yaml"))
    assert any((packages / "controllers").rglob("controller.package.yaml"))
    assert any((packages / "sensors").rglob("sensor.package.yaml"))
    assert any((packages / "sensor_rigs").rglob("sensor-rig.package.yaml"))
    assert any((packages / "worlds").rglob("world.package.yaml"))
    assert any((packages / "scenarios").rglob("scenario.package.yaml"))
    assert (packages / "sensors" / "livox" / "mid360" / "assets" / "mid360.npy").is_file()

    outside = [
        path
        for path in SIM_ROOT.rglob("*.package.yaml")
        if packages not in path.parents
    ]
    assert outside == []
