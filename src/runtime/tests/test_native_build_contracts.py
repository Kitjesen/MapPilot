from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]


def _read_repo_file(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8", errors="ignore")


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


def test_nav_cpp_propagates_openmp_to_local_planning():
    cmake = _read_repo_file("src/nav/cpp/cmake/NavCoreTargets.cmake")

    assert "find_package(OpenMP QUIET)" in cmake
    assert "target_link_libraries(lingtu_nav_local_planner PUBLIC" in cmake
    assert "OpenMP::OpenMP_CXX" in cmake
    assert "ament_" not in cmake


def test_nav_cpp_cmake_comments_are_ascii_to_avoid_mojibake():
    cmake = _read_repo_file("src/nav/cpp/CMakeLists.txt")

    for line in cmake.splitlines():
        if line.lstrip().startswith("#"):
            assert line.isascii(), line


def test_nav_cpp_does_not_enable_fast_math_globally():
    cmake = _read_repo_file("src/nav/cpp/CMakeLists.txt")
    targets = _read_repo_file("src/nav/cpp/cmake/NavCoreTargets.cmake")

    assert "-ffast-math" not in cmake
    assert "-ffast-math" not in targets
