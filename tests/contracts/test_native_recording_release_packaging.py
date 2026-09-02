"""Release packaging contracts for the ROS-free native recording tools."""

# ruff: noqa: D103, I001, S101 - pytest contracts use assertions by design.

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RECORDING_EXECUTABLES = (
    "lingtu_recorder",
    "lingtu_dds_recorder",
    "lingtu_dds_player",
    "lingtu_camera_recorder",
    "lingtu_camera_player",
)


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_native_recording_cmake_installs_the_five_tools_as_siblings() -> None:
    cmake = _read("src/native/recording/CMakeLists.txt")

    for executable in RECORDING_EXECUTABLES:
        assert executable in cmake
    assert "include(GNUInstallDirs)" in cmake
    assert "install(TARGETS ${LINGTU_RECORDING_EXECUTABLES}" in cmake
    assert "RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}" in cmake
    assert cmake.count("COMPONENT lingtu_runtime") == 1


def test_native_release_packager_installs_and_verifies_recording_tools() -> None:
    package = _read("scripts/deploy/package_native_release.sh")

    assert "LINGTU_NATIVE_RELEASE_INSTALL_SOURCE" in package
    assert "native-recording" in package
    assert 'install_cmake_tree "${ROOT}/build/${component}"' in package
    for executable in RECORDING_EXECUTABLES:
        assert executable in package
    assert 'require_install_executable "${relative}"' in package
    assert "Standard install prefix is missing executable" in package
    assert '"${INSTALL_PREFIX}/bin/${relative}"' in package
    assert "build/native-recording/${relative}" in package


def test_root_release_build_builds_native_recording() -> None:
    makefile = _read("Makefile")

    assert (
        'LINGTU_RECORDING_BUILD_DDS=ON '
        "bash scripts/build/build_native_recording.sh"
    ) in makefile
