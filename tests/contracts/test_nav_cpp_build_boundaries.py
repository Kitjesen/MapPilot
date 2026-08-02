from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_navigation_cpp_has_one_portable_build_entrypoint() -> None:
    root_cmake = _read("src/nav/cpp/CMakeLists.txt")
    targets = _read("src/nav/cpp/cmake/NavCoreTargets.cmake")

    assert "lingtu_nav_path_follower" in targets
    assert "lingtu_nav_plan_loop" in targets
    assert "LINGTU_NAV_CPP_BUILD_ENDPOINT" in root_cmake
    assert "add_subdirectory(endpoint)" in root_cmake
    assert "services/endpoint/cpp" not in root_cmake


def test_endpoint_links_navigation_targets_instead_of_recompiling_sources() -> None:
    endpoint = _read("src/nav/cpp/endpoint/CMakeLists.txt")

    assert '"${_NAV_PLAN_CPP_DIR}/nav_loop.cpp"' not in endpoint
    assert '"${_NAV_KERNEL_DIR}/src/path_follower_core.cpp"' not in endpoint
    assert "lingtu_nav_plan_loop" in endpoint
    assert "lingtu_nav_path_follower" in endpoint


def test_dds_types_are_compiled_once_per_endpoint_build_tree() -> None:
    endpoint = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    messages = _read("src/message/cpp/CMakeLists.txt")

    assert "_LINGTU_IDL_SOURCE" not in endpoint
    assert "lingtu_add_dds_c_messages(lingtu_dds_messages" in endpoint
    assert "function(lingtu_add_dds_c_messages" in messages
    assert "POSITION_INDEPENDENT_CODE ON" in messages
    assert "add_dependencies(test_transform_buffer lingtu_nav_client)" not in endpoint
    assert "add_dependencies(test_traversability_geometry lingtu_nav_client)" not in endpoint


def test_legacy_cpp_shim_directories_are_removed() -> None:
    assert not (ROOT / ("src/nav/services/plan/" + "cpp")).exists()
    assert not (ROOT / ("src/nav/services/" + "endpoint/cpp")).exists()
    assert not (ROOT / ("src/nav/commands/" + "cpp")).exists()

    # ``nav.kernel`` remains the Python extension-loader interface, but it is
    # no longer a second C++ source/build tree.
    assert (ROOT / "src/nav/kernel/loader.py").is_file()
    assert not (ROOT / ("src/nav/kernel/" + "CMakeLists.txt")).exists()
    assert not (ROOT / ("src/nav/kernel/" + "CMakeLists_nanobind_only.cmake")).exists()
    assert not (ROOT / ("src/nav/kernel/" + "package.xml")).exists()
    assert not (ROOT / ("src/nav/kernel/" + "include")).exists()
    assert not (ROOT / ("src/nav/kernel/" + "src")).exists()
    assert not (ROOT / ("src/nav/kernel/" + "tests")).exists()
    assert not (ROOT / ("src/nav/local/" + "cpp/CMakeLists.txt")).exists()


def test_global_planner_contract_does_not_expose_octomap_storage() -> None:
    contract = _read("src/nav/cpp/planning/global/global_planner_contract.hpp")
    adapter = _read("src/nav/cpp/endpoint/plan/active_octomap_gate.hpp")

    assert "map_path" not in contract
    assert "octomap" not in contract.lower()
    assert "pcd" not in contract.lower()
    assert "MapIdentity" in contract
    assert "configured_octomap_path" in adapter


def test_endpoint_rejects_stale_plans_and_reuses_validated_maps() -> None:
    endpoint = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    controller = _read("src/nav/cpp/endpoint/plan/goal_plan_controller.cpp")
    stale_guard = _read("src/nav/cpp/endpoint/plan/global_plan_task.cpp")
    gate = _read("src/nav/cpp/endpoint/plan/active_octomap_gate.cpp")

    assert "globalPlanStaleReason" in controller
    assert "frame_epoch" in stale_guard
    assert "goal_epoch" in stale_guard
    assert "PlannerSession" in endpoint
    assert "sameMapIdentity" in gate
    assert "cached_artifact_" in gate


def test_far_is_optional_native_backend_with_active_map_gate() -> None:
    root_cmake = _read("src/nav/cpp/CMakeLists.txt")
    endpoint = _read("src/nav/cpp/endpoint/nav_native_endpoint.cpp")
    gate = _read("src/nav/cpp/endpoint/plan/active_occupancy_gate.cpp")

    assert "lingtu_nav_far" in root_cmake
    assert "GlobalPlannerBackend::Far" in endpoint
    assert "runWithActiveOccupancy" in endpoint
    assert "ValidateArtifacts" in gate
    assert "cached_artifact_" in gate


def test_octoplanner_product_source_has_one_canonical_location() -> None:
    canonical = ROOT / "src/nav/cpp/planning/global/octoplanner"
    legacy = ROOT / (
        "src/nav/services/plan/global_planner/algorithm/" + "OctoPlanner3D"
    )

    assert (canonical / "CMakeLists.txt").is_file()
    assert (canonical / "vendor/planner/src/global_planner.cpp").is_file()
    assert not legacy.exists()


def test_active_sources_do_not_reference_retired_navigation_cpp_paths() -> None:
    forbidden = (
        "src/nav/services/" + "endpoint/cpp",
        "src/nav/services/plan/" + "cpp",
        "src/nav/commands/" + "cpp",
        "src/nav/services/plan/global_planner/algorithm/" + "OctoPlanner3D",
        "src/nav/kernel/" + "include",
        "src/nav/kernel/" + "src",
        "src/nav/kernel/" + "tests",
        "src/nav/kernel/" + "bindings",
        "src/nav/local/" + "cpp",
        "src/nav/services/plan/local_planner/" + "paths",
    )
    suffixes = {
        ".cmake",
        ".cpp",
        ".h",
        ".hpp",
        ".md",
        ".py",
        ".service",
        ".sh",
        ".toml",
        ".txt",
        ".yaml",
        ".yml",
    }
    scan_roots = (
        ROOT / "src",
        ROOT / "scripts",
        ROOT / "config",
        ROOT / "sim",
        ROOT / "tests",
        ROOT / "tools",
        ROOT / "docs" / "architecture",
    )
    skipped_directories = {
        ".git",
        ".pytest_cache",
        ".ruff_cache",
        ".tmp",
        ".venv",
        "__pycache__",
        "node_modules",
        "third_party",
        "vendor",
    }
    violations: list[str] = []
    this_file = Path(__file__).resolve()
    for scan_root in scan_roots:
        for directory, child_directories, filenames in os.walk(scan_root):
            child_directories[:] = [
                name
                for name in child_directories
                if name not in skipped_directories
                and not Path(directory, name, "CMakeCache.txt").is_file()
            ]
            for filename in filenames:
                path = Path(directory, filename)
                if path.resolve() == this_file:
                    continue
                if path.suffix.lower() not in suffixes:
                    continue
                relative = path.relative_to(ROOT).as_posix()
                text = path.read_text(encoding="utf-8", errors="ignore")
                for retired in forbidden:
                    if retired in text:
                        violations.append(f"{relative}: {retired}")
    assert not violations, "\n".join(violations)


def test_build_and_ci_use_the_canonical_navigation_cpp_root() -> None:
    endpoint_builder = _read("scripts/build/build_nav_endpoint.sh")
    kernel_builder = _read("scripts/build/build_nav_kernel.sh")
    native_ci = _read(".github/workflows/native-motion-build.yml")
    portable_ci = _read(".github/workflows/nav-core-tests.yml")

    assert 'NAV_CPP_DIR="$ROOT/src/nav/cpp"' in endpoint_builder
    assert 'cmake -S "$NAV_CPP_DIR"' in endpoint_builder
    assert "LEGACY_NAV_ENDPOINT_DIR" not in endpoint_builder
    assert "src/nav/services/" + "endpoint/cpp" not in endpoint_builder
    assert "rm -rf" not in endpoint_builder
    assert 'NAV_CPP_DIR="$REPO_ROOT/src/nav/cpp"' in kernel_builder
    assert 'validate_cleanup_target "$BUILD_DIR"' in kernel_builder
    assert '"$BUILD_ROOT"/*' in kernel_builder
    assert 'rm -rf -- "$BUILD_DIR"' in kernel_builder
    assert native_ci.count("src/nav/cpp/**") == 2
    assert "src/nav/services/" + "endpoint/cpp/**" not in native_ci
    assert "src/nav/kernel/**" not in native_ci
    assert "-S src/nav/cpp" in portable_ci
    assert "src/nav/kernel" not in portable_ci


def test_dds_services_delegate_to_checked_runners() -> None:
    nav_service = _read("scripts/deploy/thunder/lingtu-nav-dds.service")
    explore_service = _read("scripts/deploy/thunder/lingtu-explore-dds.service")
    nav_runner = _read("scripts/deploy/thunder/run_nav_dds.sh")
    explore_runner = _read("scripts/deploy/thunder/run_explore_dds.sh")

    assert "run_nav_dds.sh" in nav_service
    assert "run_explore_dds.sh" in explore_service
    assert "native navigation DDS endpoint is missing or not executable" in nav_runner
    assert "native exploration endpoint is missing or not executable" in explore_runner
    assert 'exec "${LINGTU_NAV_DDS_BIN}"' in nav_runner
    assert 'exec "${LINGTU_EXPLORE_DDS_BIN}"' in explore_runner

def test_release_activation_is_atomic_and_rollback_safe() -> None:
    release = _read("scripts/deploy/cut_release.sh")

    for required in (
        "thunder-runtime-env.sh",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu-explore-dds.service",
        "backup_activation_state",
        "install_activation_files",
        "verify_activation_files",
        "restore_activation_state",
        "release_rollback_trap",
        "sudo systemctl daemon-reload",
        "sudo systemd-analyze verify",
        "sudo cmp -s",
    ):
        assert required in release
    assert "sudo install -o root -g root -m 0644" in release
    assert "sudo mv -f" in release
    assert "rm -rf" not in release


def test_release_bundles_native_sensor_slam_and_driver_artifacts() -> None:
    release = _read("scripts/deploy/cut_release.sh")

    for required in (
        'LIVOX_BUILD_DIR="${LINGTU_LIVOX_SDK2_STREAM_BUILD_DIR:-$DEV_DIR/build/livox_sdk2_stream}"',
        'SLAM_BUILD_DIR="${LINGTU_SLAM_CORE_BUILD_DIR:-$DEV_DIR/build/slam_core}"',
        'DRIVER_BUILD_DIR="${LINGTU_DRIVER_BUILD_DIR:-$DEV_DIR/build/driver}"',
        'LINGTU_LIVOX_SDK2_STREAM_BUILD_DDS=ON',
        'bash "$DEV_DIR/scripts/build/build_livox_sdk2_stream.sh"',
        'LINGTU_SLAM_BUILD_DDS_RUNTIME=ON',
        'LINGTU_SLAM_BUILD_PYTHON_BINDINGS=OFF',
        'bash "$DEV_DIR/scripts/build/build_slam_core.sh"',
        'bash "$DEV_DIR/scripts/build/build_mapd.sh"',
        'bash "$DEV_DIR/scripts/build/build_dds_probe.sh"',
        'bash "$DEV_DIR/scripts/build/build_driver.sh"',
        'build/livox_sdk2_stream/livox_sdk2_stream',
        'build/slam_core/slamd',
        'build/maps/mapd',
        'build/dds_probe/lingtu_dds_probe',
        'build/driver/lingtu_driver',
        'write_release_native_sha256_manifest',
        'verify_release_native_sha256_manifest "$CURRENT_LINK"',
        'verify_driver_uses_current_release',
        'verify_mapd_uses_current_release',
    ):
        assert required in release

    activation = release.split("activation_destinations() {", 1)[1].split(
        "\n}\n\nrelease_source_for_destination()", 1
    )[0]
    source_mapping = release.split("release_source_for_destination() {", 1)[1].split(
        "\n}\n\nbackup_label_for_path()", 1
    )[0]
    verification = release.split("verify_activation_files() {", 1)[1].split(
        "\n}\n\ncleanup_activation_backup()", 1
    )[0]
    for section in (activation, source_mapping, verification):
        assert "lingtu-driver.service" in section
    assert "resolve_release_services" not in release
    assert 'driver = selected_process("driver")' in release
    assert 'CURRENT_DRIVER_UNIT="${fields[5]}"' in release
    assert 'systemctl is-enabled --quiet "$CURRENT_DRIVER_UNIT"' in release

    switch = release.index('sudo ln -sfn -- "$TARGET_DIR" "$CURRENT_LINK"')
    reapply = release.index('reapply_committed_run_plan "activation"', switch)
    assert switch < reapply
    assert 'sudo systemctl restart' not in release
    assert 'restart_service_list' not in release
    assert '/api/v1/health' not in release
    assert '/ready' not in release
    assert 'DEADLINE=$((SECONDS + 60))' not in release
    assert 'sleep 3' not in release
    assert 'while [ $SECONDS -lt $DEADLINE ]' not in release


def test_runtime_env_selects_the_map_for_the_configured_global_planner() -> None:
    runtime_env = _read("scripts/deploy/thunder/runtime-env.sh")
    runner = _read("scripts/deploy/thunder/run_nav_dds.sh")

    assert "LINGTU_ACTIVE_PLANNER_MAP" in runtime_env
    assert "octoplanner3d|octo)" in runtime_env
    assert "LINGTU_ACTIVE_OCTOMAP" in runtime_env
    assert "far)" in runtime_env
    assert "LINGTU_ACTIVE_OCCUPANCY" in runtime_env
    assert '--global-planner "${LINGTU_NAV_GLOBAL_PLANNER}"' in runner
    assert '--map "${LINGTU_ACTIVE_PLANNER_MAP}"' in runner

def test_explore_map_variant_declares_saved_map_localization_capability() -> None:
    from runtime.contracts.product_runtime import resolve_product_spec_contracts
    from runtime.graph.loader import load_runtime_graph, resolve_product_variant_spec

    graph = load_runtime_graph()
    product = resolve_product_variant_spec(
        "explore",
        graph.products["explore"],
        product_variant="map",
    )
    contract = resolve_product_spec_contracts(
        "explore",
        graph.products["explore"],
        product_variant="map",
    )

    assert product["slam_mode"] == "localization"
    assert product["requires_map"] is True
    assert "saved_map_relocalization" in contract.capabilities
    assert "native_slam_mapping" not in contract.capabilities

def test_mujoco_native_endpoint_closes_exploration_contract() -> None:
    sim_env = _read("config/runtime_graph/envs/sim.yaml")
    endpoint = sim_env.split("  mujoco_native:\n", 1)[1].split(
        "  mujoco_host:\n", 1
    )[0]

    for required in (
        "exploration_command_boundary:",
        "  request: /nav/exploration/command",
        "  ack: /nav/exploration/ack",
        "  - /nav/exploration_grid",
        "  - /nav/exploration_snapshot",
    ):
        assert required in endpoint
    if "native_services:" in endpoint:
        assert "  - lingtu_explore_dds" in endpoint
    else:
        assert "process_control: acceptance_runner" in endpoint
