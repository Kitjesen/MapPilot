"""Filesystem contract tests for the root sim/ layout."""

import importlib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "sim"

# Managed sim/ roots, grouped by the layer they belong to
# (SIM_RUNTIME_CONTRACT.md: Package / Runtime / Adapter / Visual / Tooling /
# compatibility / gates). Each name is asserted to exist so that a layer can
# never be silently removed.
CANONICAL_ROOTS = (
    # Package / static definition layer
    "packages",
    "scenarios",
    # Catalog / compile layer
    "catalog",
    "worlds",
    "assets",
    "robots",
    "sensors",
    "sensor_rigs",
    "controllers",
    # Runtime / engine layer
    "runtime",
    # Adapter layer
    "adapters",
    # Tooling
    "toolchains",
    "tools",
    # Compatibility, fixtures, and measurement surfaces
    "fixtures",
    "planning",
    "following",
    "datasets",
    "evaluation",
    "external_scenes",
    "maps",
    "diagnostics",
    "engine",
    "validation",
    "scripts",
    "tests",
)

CANONICAL_MUJOCO_ENTRYPOINTS = (
    "mujoco/native_dds_sensors.py",
    "mujoco/native_navigation_acceptance.py",
    "mujoco/native_control_mode_acceptance.py",
    "mujoco/product_acceptance.py",
    "mujoco/map_native_acceptance.py",
    "mujoco/explore_native_acceptance.py",
    "mujoco/inspection_native_acceptance.py",
    "mujoco/teleop_native_acceptance.py",
    "mujoco/teleop_avoid_native_acceptance.py",
    "mujoco/record_thunderv4_mid360_policy.py",
    "mujoco/record_thunderv4_stair_showcase.py",
    "mujoco/continuous_mapping_quality_gate.py",
)

PUBLIC_SCRIPT_ENTRYPOINTS = (
    "sim_diagnostics.py",
    "saved_map_relocalize_runtime_gate.py",
)

def test_sim_canonical_roots_exist():
    for name in CANONICAL_ROOTS:
        assert (SIM_ROOT / name).is_dir(), name


def test_generic_runtime_uses_role_named_paths():
    assert (SIM_ROOT / "runtime" / "physics" / "CMakeLists.txt").is_file()
    assert (SIM_ROOT / "runtime" / "coordinator" / "coordinator.py").is_file()
    assert (
        SIM_ROOT / "runtime" / "visual" / "RobotSimUE" / "RobotSimUE.uproject"
    ).is_file()
    assert (SIM_ROOT / "adapters" / "dds" / "CMakeLists.txt").is_file()

    for retired in (
        SIM_ROOT / "runtime" / "cpp",
        SIM_ROOT / "native_dds",
        SIM_ROOT / "unreal",
        SIM_ROOT / "engine" / "ue",
    ):
        assert not retired.exists(), retired


def test_sim_public_script_entrypoints_stay_in_place():
    scripts_root = SIM_ROOT / "scripts"

    for name in PUBLIC_SCRIPT_ENTRYPOINTS:
        assert (scripts_root / name).is_file(), name


def test_retired_cmu_unity_runtime_is_absent():
    retired = (
        SIM_ROOT / "scripts" / "launch_cmu_unity_baseline.sh",
        SIM_ROOT / "engine" / "bridge" / "cmu_unity_lingtu_adapter.py",
        SIM_ROOT / "planning" / "cmu_unity_lingtu_runtime.rviz",
    )

    for path in retired:
        assert not path.exists(), path


def test_sim_mujoco_scripts_have_canonical_implementation_paths():
    scripts_root = SIM_ROOT / "scripts"

    for name in CANONICAL_MUJOCO_ENTRYPOINTS:
        assert (scripts_root / name).is_file(), name


def test_native_sensor_bridge_imports():
    module = importlib.import_module("sim.scripts.mujoco.native_dds_sensors")
    assert callable(module._relative_times_for_scan)
    assert callable(module._physical_rolling_scan_from_samples)


def test_retired_ros_sim_chains_are_absent():
    retired = (
        SIM_ROOT / "scripts" / "server_sim_closure.py",
        SIM_ROOT / "engine" / "cli.py",
        SIM_ROOT / "engine" / "bridge" / "ros2_bridge.py",
        SIM_ROOT / "engine" / "scenarios" / "base.py",
        SIM_ROOT / "engine" / "scenarios" / "navigation.py",
        SIM_ROOT / "engine" / "scenarios" / "semantic_nav.py",
        SIM_ROOT / "semantic" / "factory_stub_test.py",
        SIM_ROOT / "experiments" / "eval_runner.py",
        SIM_ROOT / "scripts" / "mujoco" / "live_gate.py",
        SIM_ROOT / "scripts" / "mujoco" / "launch_fastlio2_live.sh",
        SIM_ROOT / "scripts" / "mujoco_live",
        SIM_ROOT / "scripts" / "moving_obstacle_sweep_gate.py",
        SIM_ROOT / "scripts" / "large_loop_closure_gate.py",
        SIM_ROOT / "scripts" / "render_slam_validation_screenshots.py",
    )

    for path in retired:
        assert not path.exists(), path


def test_sim_canonical_assets_are_discoverable():
    assert (SIM_ROOT / "robots" / "doso" / "thunder_v4" / "robot.package.yaml").is_file()
    assert (SIM_ROOT / "packages" / "worlds" / "open_field" / "world.package.yaml").is_file()
    assert (SIM_ROOT / "assets" / "livox" / "mid360.npy").is_file()
    assert any((SIM_ROOT / "worlds" / "mujoco").glob("*.xml"))
    assert any((SIM_ROOT / "worlds" / "gazebo").glob("*.sdf"))


def test_static_package_manifests_live_with_their_assets():
    assert any((SIM_ROOT / "packages").rglob("*.package.yaml"))
    assert any((SIM_ROOT / "robots").rglob("robot.package.yaml"))
    assert any((SIM_ROOT / "controllers").rglob("controller.package.yaml"))
    assert any((SIM_ROOT / "sensors").rglob("sensor.package.yaml"))
    assert any((SIM_ROOT / "sensor_rigs").rglob("sensor-rig.package.yaml"))
    assert not (SIM_ROOT / "packages" / "robots").exists()
    assert not (SIM_ROOT / "packages" / "controllers").exists()
    assert not (SIM_ROOT / "packages" / "sensors").exists()
    assert not (SIM_ROOT / "packages" / "sensor_rigs").exists()
    assert not any(path.name.endswith(".package.yaml") for path in (SIM_ROOT / "worlds").rglob("*.package.yaml"))
    assert not any(path.name.endswith(".package.yaml") for path in (SIM_ROOT / "scenarios").rglob("*.package.yaml"))
