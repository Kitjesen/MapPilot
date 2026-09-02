from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NATIVE_SLAM = ROOT / "src" / "localization" / "slam" / "cpp"


def test_removed_backend_has_no_native_build_surface() -> None:
    """A removed backend name must not regain a native build surface."""

    assert not (NATIVE_SLAM / "pointlio.cpp").exists()
    assert "makePointLioBackend" not in (NATIVE_SLAM / "slam.hpp").read_text(
        encoding="utf-8"
    )
    assert "pointlio.cpp" not in (NATIVE_SLAM / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    native_sources = [
        *NATIVE_SLAM.glob("*.cpp"),
        *NATIVE_SLAM.glob("*.hpp"),
    ]
    assert [
        path.name
        for path in native_sources
        if "pointlio" in path.read_text(encoding="utf-8").lower()
    ] == []


def test_native_slam_has_no_python_binding_build_surface() -> None:
    """Native SLAM is a C++ DDS process, not an in-process Python extension."""

    cmake = (NATIVE_SLAM / "CMakeLists.txt").read_text(encoding="utf-8")

    assert not (NATIVE_SLAM / "bind.cpp").exists()
    for obsolete in (
        "LINGTU_SLAM_BUILD_PYTHON_BINDINGS",
        "nanobind",
        "FetchContent",
        "_native",
    ):
        assert obsolete not in cmake

    build_surfaces = (
        ROOT / "Makefile",
        ROOT / "docker" / "Dockerfile",
        ROOT / ".github" / "workflows" / "slam-aarch64-build.yml",
        ROOT / "scripts" / "build" / "build_slam_core.sh",
        ROOT / "scripts" / "build" / "build_slam_core_windows.ps1",
        ROOT / "scripts" / "deploy" / "thunder" / "run_slam_dds.sh",
    )
    assert [
        str(path.relative_to(ROOT))
        for path in build_surfaces
        if "LINGTU_SLAM_BUILD_PYTHON_BINDINGS"
        in path.read_text(encoding="utf-8")
    ] == []


def test_python_host_has_no_managed_slam_executor() -> None:
    assert not (ROOT / "src" / "localization" / "slam" / "module.py").exists()
