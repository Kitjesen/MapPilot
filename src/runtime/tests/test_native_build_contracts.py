from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read_repo_file(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8", errors="ignore")


def test_nav_kernel_builder_does_not_force_user_site_install():
    script = _read_repo_file("scripts/build/build_nav_kernel.sh")

    assert "pip install --user nanobind" not in script
    assert "pip_install" not in script
    assert "python3-pip" not in script
    assert "FetchContent_Declare(nanobind" in script
    assert "GIT_TAG        v2.12.0" in script


def test_nav_kernel_loader_rejects_stale_or_wrong_platform_extensions():
    loader = _read_repo_file("src/nav/kernel/loader.py")

    assert '".pyd"' in loader
    assert "required_symbols" in loader
    assert "missing required symbols" in loader
    assert 'NAV_KERNEL_EXTENSION_MODULE = "lingtu_nav_kernel"' in loader


def test_disabled_bbs3d_stub_never_advertises_map_available():
    source = _read_repo_file(
        "src/localization/localizer/src/localizers/bbs3d_global_localizer.cpp"
    )
    disabled_branch = source.rsplit("#else", 1)[1].split("#endif", 1)[0]

    assert "has_map_ = false;" in disabled_branch
    assert "return false;" in disabled_branch
    assert "has_map_ = bool(map_cloud && !map_cloud->empty())" not in disabled_branch


def test_localizer_can_enable_bbs3d_from_local_prefix():
    cmake = _read_repo_file("src/localization/localizer/CMakeLists.txt")

    assert "CPU_BBS3D_ROOT" in cmake
    assert "$ENV{CPU_BBS3D_ROOT}" in cmake
    assert "PATH_SUFFIXES lib lib64" in cmake
    assert "PATH_SUFFIXES include" in cmake


def test_ros_workspace_builder_covers_native_server_dependencies():
    script = _read_repo_file("scripts/build/build_ros_workspace.sh")

    assert 'ros-${ROS_DISTRO}-pcl-ros' in script
    assert 'ros-${ROS_DISTRO}-sophus' in script
    assert "libgoogle-glog-dev" in script
    assert "colcon build --base-paths src" in script
    assert "ldd" in script


def test_ros_workspace_builder_skips_legacy_planning_packages_by_default():
    script = _read_repo_file("scripts/build/build_ros_workspace.sh")

    assert 'BUILD_LEGACY_PCT_ROS="${LINGTU_BUILD_LEGACY_PCT_ROS:-0}"' in script
    assert 'if [[ "${BUILD_LEGACY_PCT_ROS}" != "1" ]]; then' in script
    assert "skip+=(pct_planner pct_adapters)" in script
    assert "tare_planner" not in script


def test_ros_workspace_builder_skips_vendored_gtsam_by_default():
    script = _read_repo_file("scripts/build/build_ros_workspace.sh")

    assert 'BUILD_VENDORED_GTSAM="${LINGTU_BUILD_VENDORED_GTSAM:-0}"' in script
    assert 'if [[ "${BUILD_VENDORED_GTSAM}" != "1" ]]; then' in script
    assert "skip+=(gtsam)" in script
    assert "GTSAM_DIR" not in script
    assert "CMAKE_PREFIX_PATH" not in script


def test_ros_workspace_builder_is_not_hidden_by_build_gitignore_rule():
    ignore = _read_repo_file(".gitignore")

    assert "!scripts/build/" in ignore
    assert "!scripts/build/build_ros_workspace.sh" in ignore


def test_nav_kernel_exports_openmp_for_downstream_ament_consumers():
    cmake = _read_repo_file("src/nav/kernel/CMakeLists.txt")

    assert "target_link_libraries(${PROJECT_NAME} INTERFACE OpenMP::OpenMP_CXX)" in cmake
    assert "ament_export_dependencies(OpenMP)" in cmake


def test_nav_kernel_standalone_tests_do_not_require_python_headers():
    cmake = _read_repo_file("src/nav/kernel/CMakeLists.txt")

    assert 'option(NAV_KERNEL_BUILD_PYTHON_BINDINGS "Build nav_kernel Python bindings" OFF)' in cmake
    assert 'EXISTS "${Python3_INCLUDE_DIRS}/Python.h"' in cmake
    assert "Skipping nav_kernel Python bindings because Python development headers were not found" in cmake


def test_nav_kernel_build_script_is_ascii_for_server_terminals():
    script = _read_repo_file("scripts/build/build_nav_kernel.sh")

    assert script.isascii()


def test_nav_kernel_build_script_installs_real_release_artifact():
    script = _read_repo_file("scripts/build/build_nav_kernel.sh")

    assert 'cp -f "$SO_FILE" "$LINK_TARGET"' in script
    assert 'ln -s "$SO_FILE" "$LINK_TARGET"' not in script
    assert "LingTu auto-detects the installed runtime in src/" in script
    assert 'LOCAL_PLANNER_CPP_SRC "${NAV_KERNEL_SRC}/../services/plan/local_planner/cpp"' in script


def test_nav_kernel_loader_exposes_production_requirement_gate():
    loader = _read_repo_file("src/nav/kernel/loader.py")
    init = _read_repo_file("src/nav/kernel/__init__.py")

    assert "PRODUCTION_NAV_KERNEL_SYMBOLS" in loader
    assert "def require_nav_kernel(" in loader
    assert "fuse_traversability_cost" in loader
    assert "require_nav_kernel" in init


def test_nav_kernel_cmake_comments_are_ascii_to_avoid_mojibake():
    cmake = _read_repo_file("src/nav/kernel/CMakeLists.txt")

    for line in cmake.splitlines():
        if line.lstrip().startswith("#"):
            assert line.isascii(), line


def test_nav_kernel_does_not_enable_fast_math_globally():
    cmake = _read_repo_file("src/nav/kernel/CMakeLists.txt")

    assert "-ffast-math" not in cmake
    assert "validation must preserve NaN/Inf checks" in cmake
