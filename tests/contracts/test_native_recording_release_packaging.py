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
    assert "install(TARGETS ${LINGTU_RECORDING_EXECUTABLES}" in cmake
    assert "RUNTIME DESTINATION ." in cmake


def test_native_release_packager_installs_and_verifies_recording_tools() -> None:
    package = _read("scripts/deploy/package_native_release.sh")

    assert 'RECORDING_INSTALL_DIR="${PACKAGE_ROOT}/build/native-recording"' in package
    assert 'LINGTU_NATIVE_RELEASE_RECORDING_INSTALL_SOURCE' in package
    assert (
        'cmake --install "${ROOT}/build/native-recording" '
        '--prefix "${RECORDING_INSTALL_DIR}"'
    ) in package
    for executable in RECORDING_EXECUTABLES:
        assert executable in package
    assert 'if [[ ! -x "${RECORDING_INSTALL_DIR}/${relative}" ]]' in package
    assert "Native recording install is missing executable" in package
    assert "build/native-recording/${relative}" in package


def test_root_release_build_builds_native_recording() -> None:
    makefile = _read("Makefile")

    assert (
        'LINGTU_RECORDING_BUILD_DDS=ON '
        "bash scripts/build/build_native_recording.sh"
    ) in makefile


def test_release_cutter_delegates_recording_packaging_to_native_packager() -> None:
    release = _read("scripts/deploy/cut_release.sh")

    assert "package_native_release.sh" in release
    for executable in RECORDING_EXECUTABLES:
        assert executable not in release
