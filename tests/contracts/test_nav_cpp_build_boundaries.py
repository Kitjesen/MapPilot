# ruff: noqa: D103, S101

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_navigation_cpp_has_one_portable_build_entrypoint() -> None:
    root_cmake = _read("src/nav/cpp/CMakeLists.txt")
    targets = _read("src/nav/cpp/cmake/NavCoreTargets.cmake")

    assert "lingtu_nav_local_planner" in targets
    assert "lingtu_nav_follower" in targets
    assert "lingtu_nav_navigation" in targets
    assert "LINGTU_NAV_CPP_BUILD_ENDPOINT" in root_cmake
    assert "add_subdirectory(endpoint)" in root_cmake
    assert "services/endpoint/cpp" not in root_cmake


def test_navigation_runtime_install_does_not_publish_internal_headers() -> None:
    root_cmake = _read("src/nav/cpp/CMakeLists.txt")
    targets = _read("src/nav/cpp/cmake/NavCoreTargets.cmake")
    endpoint = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    inspection = _read("src/nav/inspection/CMakeLists.txt")
    cmake_sources = (root_cmake, targets, endpoint, inspection)

    for source in cmake_sources:
        assert "INSTALL_INTERFACE:" not in source
        assert "DESTINATION include" not in source
        assert "ARCHIVE DESTINATION" not in source
        assert "install(TARGETS lingtu_nav_far_c_api" not in source

    assert "install(" not in root_cmake
    assert "install(" not in targets
    assert "install(TARGETS ${_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS}" in endpoint
    assert "install(TARGETS lingtu_inspection" in inspection
    assert "include(GNUInstallDirs)" in endpoint
    assert "RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}" in endpoint
    assert "LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}" in endpoint
    assert endpoint.count("COMPONENT lingtu_runtime") == 3
    assert 'INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"' in endpoint
    assert "DESTINATION ${CMAKE_INSTALL_DATADIR}/lingtu/cmu_paths" in endpoint
    assert "include(GNUInstallDirs)" in inspection
    assert "LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}" in inspection
    assert "RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}" in inspection
    assert inspection.count("COMPONENT lingtu_runtime") == 2

    maps = _read("src/maps/CMakeLists.txt")
    assert "include(GNUInstallDirs)" in maps
    assert "RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}" in maps
    assert maps.count("COMPONENT lingtu_runtime") == 1

    runtime_targets = endpoint.split("foreach(_target IN ITEMS", 1)[1].split(
        ")", 1
    )[0]
    for runtime_target in (
        "navd",
        "lingtu_traversability_dds",
        "lingtu_explore_dds",
        "lingtu_nav_control",
    ):
        assert runtime_target in runtime_targets
    assert "lingtu_motion_mock_dds" not in runtime_targets

    octoplanner_cmake = _read(
        "src/nav/cpp/planning/global/octoplanner/CMakeLists.txt"
    )
    assert "install(TARGETS octoplanner3d_pcd_to_octomap" in octoplanner_cmake
    assert "install(TARGETS octoplanner3d_route_viz" not in octoplanner_cmake

    package_release = _read("scripts/deploy/package_native_release.sh")
    assert "navigation runtime install must not contain C/C++ headers" in package_release
    assert "-name '*.h' -o -name '*.hpp'" in package_release
    assert "--exclude='/src/nav/cpp/***'" in package_release
    for pattern in (
        "*.c",
        "*.cc",
        "*.cpp",
        "*.cxx",
        "*.h",
        "*.hpp",
        "CMakeLists.txt",
        "README.md",
    ):
        assert f"--exclude='/src/nav/inspection/{pattern}'" in package_release

    assert "Native release must not contain internal source: src/nav/cpp" in package_release
    assert "Native release must not contain inspection C/C++ sources" in package_release


def test_endpoint_links_navigation_targets_instead_of_recompiling_sources() -> None:
    endpoint = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    targets = _read("src/nav/cpp/cmake/NavCoreTargets.cmake")

    assert '"${_NAV_PLAN_CPP_DIR}/nav_loop.cpp"' not in endpoint
    assert "lingtu_nav_navigation" in endpoint
    assert "lingtu_nav_follower" in targets


def test_navigation_algorithms_follow_domain_boundaries() -> None:
    cpp = ROOT / "src/nav/cpp"

    assert (cpp / "planning/global/contract.hpp").is_file()
    assert (cpp / "planning/local/planner.hpp").is_file()
    assert (cpp / "trajectory/spline.hpp").is_file()
    assert (cpp / "tracking/follower.hpp").is_file()
    assert (cpp / "navigation/executor.hpp").is_file()


def test_follower_uses_one_registered_tracking_interface() -> None:
    header = _read("src/nav/cpp/tracking/follower.hpp")
    implementation = _read("src/nav/cpp/tracking/follower.cpp")
    executor = _read("src/nav/cpp/navigation/executor.cpp")

    assert "FollowerOutput follow(const LocalPlan &plan, const FollowerState &state)" in header
    assert "registerAlgorithm(FollowerAlgorithm::Path" in implementation
    assert "FollowerAlgorithm::Spline" in implementation
    assert "follower_.follow(plan, follower_state)" in executor
    for retired in ("computeControl", "trackTrajectory", "compute_control"):
        assert retired not in header
        assert retired not in implementation
        assert retired not in executor


def test_endpoint_cmake_uses_portable_threads_and_compiler_options() -> None:
    root_cmake = _read("src/nav/cpp/CMakeLists.txt")
    endpoint = _read("src/nav/cpp/endpoint/CMakeLists.txt")

    assert "if(NOT UNIX)" not in root_cmake
    assert "find_package(Threads REQUIRED)" in endpoint
    assert "Threads::Threads" in endpoint
    assert "pthread" not in endpoint
    assert "function(lingtu_endpoint_enable_warnings target)" in root_cmake
    assert "if(MSVC)" in root_cmake
    assert "/W4" in root_cmake
    assert "-Wall -Wextra -Wpedantic" in root_cmake


def test_dds_types_are_compiled_once_per_endpoint_build_tree() -> None:
    endpoint = _read("src/nav/cpp/endpoint/CMakeLists.txt")
    messages = _read("src/message/cpp/CMakeLists.txt")

    assert "_LINGTU_IDL_SOURCE" not in endpoint
    assert "lingtu_add_dds_c_messages(lingtu_dds_messages" in endpoint
    assert "function(lingtu_add_dds_c_messages" in messages
    assert "POSITION_INDEPENDENT_CODE ON" in messages
    assert "add_dependencies(test_transform_buffer lingtu_nav_client)" not in endpoint
    assert "add_dependencies(test_traversability_geometry lingtu_nav_client)" not in endpoint


def test_dds_idlc_uses_only_the_runtime_installed_beside_the_tool() -> None:
    messages = _read("src/message/cpp/CMakeLists.txt")

    assert 'get_filename_component(_idlc_prefix "${_idlc_bin_dir}" DIRECTORY)' in messages
    assert '"${_idlc_prefix}/lib/${CMAKE_LIBRARY_ARCHITECTURE}"' in messages
    assert '"LD_LIBRARY_PATH=${_idlc_library_path}"' in messages
    assert "COMMAND ${_idlc_command}" in messages
    assert 'COMMAND "${_LINGTU_IDLC_EXECUTABLE}" -l c' not in messages


def test_native_dds_processes_share_the_message_generation_module() -> None:
    consumers = {
        "src/localization/slam/cpp/CMakeLists.txt": "messages_cyclone_idl",
        "src/drivers/real/lidar/sdk2_stream/CMakeLists.txt": "livox_sdk2_stream_messages",
        "src/drivers/real/motion/CMakeLists.txt": "lingtu_driver_messages",
        "src/maps/CMakeLists.txt": "lingtu_mapd_dds_messages",
    }

    for path, target in consumers.items():
        cmake = _read(path)
        assert "lingtu_add_dds_c_messages(" in cmake
        assert target in cmake
        assert "COMMAND \"${CYCLONEDDS_IDLC_EXECUTABLE}\"" not in cmake


def test_driver_build_can_omit_test_sources_from_a_runtime_staging_tree() -> None:
    build = _read("scripts/build/build_driver.sh")

    assert '-DBUILD_TESTING="${LINGTU_DRIVER_BUILD_TESTS:-ON}"' in build


def test_global_planner_contract_does_not_expose_octomap_storage() -> None:
    contract = _read("src/nav/cpp/planning/global/contract.hpp")
    adapter = _read("src/nav/cpp/endpoint/nav/input/active/octomap.hpp")

    assert "map_path" not in contract
    assert "octomap" not in contract.lower()
    assert "pcd" not in contract.lower()
    assert "MapIdentity" in contract
    assert "configured_octomap_path" in adapter


def test_endpoint_rejects_stale_plans_and_reuses_validated_maps() -> None:
    endpoint = _read("src/nav/cpp/endpoint/nav/main.cpp")
    controller = _read("src/nav/cpp/endpoint/nav/runtime/goal/plan.cpp")
    stale_guard = _read("src/nav/cpp/endpoint/nav/runtime/goal/task.cpp")
    gate = _read("src/nav/cpp/endpoint/nav/input/active/octomap.cpp")

    assert "globalPlanStaleReason" in controller
    assert "frame_epoch" in stale_guard
    assert "goal_epoch" in stale_guard
    assert "PlannerSession" in endpoint
    assert "sameMapIdentity" in controller
    assert "cached_artifact_" in gate


def test_navigation_map_identity_is_runtime_bound_not_store_discovered() -> None:
    identity_idl = _read("src/message/idl/messages.idl")
    declared = _read("src/nav/cpp/endpoint/nav/input/active/declared.cpp")
    octomap = _read("src/nav/cpp/endpoint/nav/input/active/octomap.cpp")
    occupancy = _read("src/nav/cpp/endpoint/nav/input/active/occupancy.cpp")

    map_identity = identity_idl.split("struct MapIdentity", 1)[1].split("};", 1)[0]
    assert "map_dir" not in map_identity
    combined = declared + octomap + occupancy
    assert "active_map.txt" not in combined
    assert "MapStore" not in combined
    assert "prepareActive" not in combined
    assert "currentDeclaredIdentity" not in combined


def test_far_is_optional_native_backend_with_active_map_gate() -> None:
    root_cmake = _read("src/nav/cpp/CMakeLists.txt")
    targets = _read("src/nav/cpp/cmake/NavCoreTargets.cmake")
    endpoint = _read("src/nav/cpp/endpoint/nav/main.cpp")
    gate = _read("src/nav/cpp/endpoint/nav/input/active/occupancy.cpp")

    assert "include(cmake/NavCoreTargets.cmake)" in root_cmake
    assert "lingtu_nav_far" in targets
    assert "GlobalPlannerBackend::Far" in endpoint
    assert "runWithActiveOccupancy" in endpoint
    assert "LoadOccupancyArtifact" in gate
    assert "cached_artifact_" in gate


def test_far_cpp_namespace_avoids_windows_far_macro() -> None:
    suffixes = {".cpp", ".h", ".hpp"}
    forbidden_substrings = (
        "lingtu::nav::plan::far::",
        "plan::far::",
        "#undef far",
        '#pragma push_macro("far")',
        '#pragma pop_macro("far")',
    )
    violations: list[str] = []
    for path in (ROOT / "src/nav/cpp").rglob("*"):
        if path.suffix.lower() not in suffixes:
            continue
        relative = path.relative_to(ROOT).as_posix()
        text = path.read_text(encoding="utf-8", errors="ignore")
        for needle in forbidden_substrings:
            if needle in text:
                violations.append(f"{relative}: {needle}")
        if re.search(r"namespace\s+lingtu::nav::plan::far\s*\{", text):
            violations.append(f"{relative}: namespace lingtu::nav::plan::far")
        if re.search(r"namespace\s+.*::far\s*\{", text):
            violations.append(f"{relative}: namespace ending in ::far")
    assert not violations, "\n".join(violations)


def test_octoplanner_product_source_has_one_canonical_location() -> None:
    canonical = ROOT / "src/nav/cpp/planning/global/octoplanner"

    assert (canonical / "CMakeLists.txt").is_file()
    assert (canonical / "vendor/planner/src/global_planner.cpp").is_file()


def test_build_and_ci_use_the_canonical_navigation_cpp_root() -> None:
    endpoint_builder = _read("scripts/build/build_nav_endpoint.sh")
    native_ci = _read(".github/workflows/native-motion-build.yml")
    portable_ci = _read(".github/workflows/nav-core-tests.yml")

    assert 'NAV_CPP_DIR="$ROOT/src/nav/cpp"' in endpoint_builder
    assert 'cmake -S "$NAV_CPP_DIR"' in endpoint_builder
    assert "LEGACY_NAV_ENDPOINT_DIR" not in endpoint_builder
    assert "src/nav/services/" + "endpoint/cpp" not in endpoint_builder
    assert "rm -rf" not in endpoint_builder
    assert "lingtu_cmu_paths" in _read("src/nav/cpp/endpoint/CMakeLists.txt")
    assert '"$BUILD_DIR/cmu_paths/$profile/$asset"' in endpoint_builder
    assert native_ci.count("src/nav/cpp/**") == 2
    assert "-S src/nav/cpp" in portable_ci


def test_dds_services_delegate_to_checked_runners() -> None:
    nav_service = _read("scripts/deploy/thunder/lt-nav.service")
    explore_service = _read("scripts/deploy/thunder/lt-explore.service")
    nav_runner = _read("scripts/deploy/thunder/run_nav_dds.sh")
    explore_runner = _read("scripts/deploy/thunder/run_explore_dds.sh")

    assert "run_nav_dds.sh" in nav_service
    assert "run_explore_dds.sh" in explore_service
    assert "native navigation DDS endpoint is missing or not executable" in nav_runner
    assert "native exploration endpoint is missing or not executable" in explore_runner
    assert 'exec "${LINGTU_NAV_DDS_BIN}"' in nav_runner
    assert 'exec "${LINGTU_EXPLORE_DDS_BIN}"' in explore_runner

def test_release_uses_the_deployer_and_native_packager_directly() -> None:
    assert not (ROOT / "scripts/deploy/cut_release.sh").exists()
    assert (ROOT / "scripts/deploy/deploy_robot.sh").is_file()
    assert (ROOT / "scripts/deploy/package_native_release.sh").is_file()


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
