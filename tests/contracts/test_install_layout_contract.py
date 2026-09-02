# ruff: noqa: D103, S101

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_repository_axes_and_generated_roots_have_one_meaning() -> None:
    readme = _read("README.md")
    layout = _read("docs/REPO_LAYOUT.md")
    gitignore = _read(".gitignore")

    for path in ("src/", "sim/", "build/", "install/", "dist/"):
        assert path in readme
        assert path in layout
    for ignored in ("build/", "install/", "dist/"):
        assert ignored in gitignore

    assert "simulation workspace" in readme
    assert "env=sim" in readme
    for forbidden_root in ("real", "environments", "bin"):
        assert not (ROOT / forbidden_root).exists()


def test_native_release_uses_canonical_install_tree_with_phase_one_compat() -> None:
    package = _read("scripts/deploy/package_native_release.sh")
    installer = _read("scripts/deploy/install_native_release.sh")
    service_installer = _read("scripts/deploy/thunder/install_services.sh")

    assert "LINGTU_NATIVE_RELEASE_INSTALL_SOURCE" in package
    assert "install/${PLATFORM}-${ARCH}/${BUILD_TYPE}" in package
    assert "for directory in bin lib etc share" in package
    assert '"${PACKAGE_ROOT}/${directory}/"' in package
    for directory in ("etc/lingtu", "share/lingtu"):
        assert directory in package
    assert "stage_legacy_build_layout" in package

    assert 'NAV_INSTALL_DIR="${PACKAGE_ROOT}/build/nav_endpoint"' not in package
    assert 'RECORDING_INSTALL_DIR="${PACKAGE_ROOT}/build/native-recording"' not in package
    assert '"${PACKAGE_DIR}/bin/mapd"' in installer
    assert '"${PACKAGE_DIR}/bin/lingtu-mapctl"' in installer

    assert 'CURRENT_RELEASE="${LINGTU_CURRENT_LINK:-/opt/lingtu/current}"' in (
        service_installer
    )
    assert "Install a dual-layout release before switching systemd" in (
        service_installer
    )
    for sentinel in (
        "bin/navd",
        "lib/liblingtu_nav_client.so",
        "etc/lingtu",
        "share/lingtu",
    ):
        assert sentinel in service_installer


def test_native_release_owners_install_only_published_executables() -> None:
    owners = {
        "src/localization/slam/cpp/CMakeLists.txt": "install(TARGETS slamd slamctl",
        "src/drivers/real/lidar/sdk2_stream/CMakeLists.txt": (
            "install(TARGETS livox_sdk2_stream"
        ),
        "src/drivers/real/motion/CMakeLists.txt": "install(TARGETS lingtu_driver",
        "src/drivers/real/camera/native/CMakeLists.txt": (
            "install(TARGETS lingtu_camera_dds"
        ),
        "src/drivers/real/gnss/CMakeLists.txt": "install(TARGETS lingtu_gnss_dds",
        "src/maps/prune/cpp/CMakeLists.txt": "install(TARGETS prune",
        "src/nav/cpp/planning/global/octoplanner/CMakeLists.txt": (
            "install(TARGETS octoplanner3d_pcd_to_octomap"
        ),
        "src/localization/opt/CMakeLists.txt": "install(TARGETS lt_pgo",
    }

    for path, install_rule in owners.items():
        cmake = _read(path)
        assert "include(GNUInstallDirs)" in cmake
        assert install_rule in cmake
        assert "RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}" in cmake
        assert "COMPONENT lingtu_runtime" in cmake

    livox_vendor = _read(
        "src/drivers/real/lidar/deps/livox/Livox-SDK2/sdk_core/CMakeLists.txt"
    )
    assert "install(TARGETS" in livox_vendor
    assert "COMPONENT lingtu_runtime" not in livox_vendor


def test_direct_orbbec_build_is_relocatable_between_build_and_install() -> None:
    build = _read("scripts/build/build_orbbec_native.sh")
    camera = _read("src/drivers/real/camera/native/camera_dds.cpp")

    assert "$ORIGIN/lib:$ORIGIN/../lib" in build
    assert '-Wl,-rpath,"$PWD/$RUNTIME_LIB"' not in build
    assert '-Wl,-rpath,"$PWD/$SDK_LIB"' not in build
    assert 'std::string capture_bin{"orbbec_capture"};' in camera
    assert "build/orbbec_native/orbbec_capture" not in camera
