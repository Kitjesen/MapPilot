from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read_repo_file(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8", errors="ignore")


def test_nav_core_builder_does_not_force_user_site_install():
    script = _read_repo_file("scripts/build/build_nav_core.sh")

    assert "pip install --user nanobind" not in script
    assert "CONDA_PREFIX" in script
    assert "base_prefix" in script
    assert "ENABLE_USER_SITE" in script
    assert '"$PYTHON" -m pip "${pip_install_args[@]}"' in script


def test_nav_core_loader_rejects_stale_or_wrong_platform_extensions():
    loader = _read_repo_file("src/base_autonomy/modules/_nav_core_loader.py")

    assert '".pyd"' in loader
    assert "required_symbols" in loader
    assert "missing required symbols" in loader


def test_disabled_bbs3d_stub_never_advertises_map_available():
    source = _read_repo_file(
        "src/slam/localizer/src/localizers/bbs3d_global_localizer.cpp"
    )
    disabled_branch = source.rsplit("#else", 1)[1].split("#endif", 1)[0]

    assert "has_map_ = false;" in disabled_branch
    assert "return false;" in disabled_branch
    assert "has_map_ = bool(map_cloud && !map_cloud->empty())" not in disabled_branch


def test_localizer_can_enable_bbs3d_from_local_prefix():
    cmake = _read_repo_file("src/slam/localizer/CMakeLists.txt")

    assert "CPU_BBS3D_ROOT" in cmake
    assert "$ENV{CPU_BBS3D_ROOT}" in cmake
    assert "PATH_SUFFIXES lib lib64" in cmake
    assert "PATH_SUFFIXES include" in cmake


def test_ros_workspace_builder_covers_native_server_dependencies():
    script = _read_repo_file("scripts/build/build_ros_workspace.sh")

    assert 'ros-${ROS_DISTRO}-pcl-ros' in script
    assert 'ros-${ROS_DISTRO}-sophus' in script
    assert "libgoogle-glog-dev" in script
    assert "fetch_ortools.sh" in script
    assert "colcon build --base-paths src" in script
    assert "ldd" in script


def test_ros_workspace_builder_is_not_hidden_by_build_gitignore_rule():
    ignore = _read_repo_file(".gitignore")

    assert "!scripts/build/" in ignore
    assert "!scripts/build/build_ros_workspace.sh" in ignore


def test_tare_builder_supports_non_passwordless_sudo_and_ros_pcl():
    script = _read_repo_file("scripts/build/build_tare.sh")

    assert "LINGTU_SUDO_PASSWORD" in script
    assert 'ros-${ROS_DISTRO}-pcl-ros' in script
    assert "libgoogle-glog-dev" in script
    assert '"/opt/ros/${ROS_DISTRO}/setup.bash"' in script


def test_nav_core_exports_openmp_for_downstream_ament_consumers():
    cmake = _read_repo_file("src/nav/core/CMakeLists.txt")

    assert "target_link_libraries(${PROJECT_NAME} INTERFACE OpenMP::OpenMP_CXX)" in cmake
    assert "ament_export_dependencies(OpenMP)" in cmake


def test_nav_core_standalone_tests_do_not_require_python_headers():
    cmake = _read_repo_file("src/nav/core/CMakeLists.txt")

    assert 'option(NAV_CORE_BUILD_PYTHON_BINDINGS "Build nav_core Python bindings" OFF)' in cmake
    assert 'EXISTS "${Python3_INCLUDE_DIRS}/Python.h"' in cmake
    assert "Skipping nav_core Python bindings because Python development headers were not found" in cmake


def test_nav_core_build_script_is_ascii_for_server_terminals():
    script = _read_repo_file("scripts/build/build_nav_core.sh")

    assert script.isascii()


def test_nav_core_cmake_comments_are_ascii_to_avoid_mojibake():
    cmake = _read_repo_file("src/nav/core/CMakeLists.txt")

    for line in cmake.splitlines():
        if line.lstrip().startswith("#"):
            assert line.isascii(), line


def test_nav_core_does_not_enable_fast_math_globally():
    cmake = _read_repo_file("src/nav/core/CMakeLists.txt")

    assert "-ffast-math" not in cmake
    assert "validation must preserve NaN/Inf checks" in cmake
