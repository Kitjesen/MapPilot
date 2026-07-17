from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_real_and_sim_camera_share_layout() -> None:
    real = ROOT / "src" / "drivers" / "real" / "camera"
    sim = ROOT / "src" / "drivers" / "sim" / "camera"

    for base in (real, sim):
        assert (base / "__init__.py").is_file()
        assert (base / "module.py").is_file()
        assert (base / "native").is_dir()
        assert (base / "impl").is_dir()
        assert (base / "deps").is_dir()

    assert (real / "impl" / "orbbec" / "camera.hpp").is_file()
    assert (real / "impl" / "orbbec" / "camera.cpp").is_file()
    assert (sim / "impl" / "mujoco" / "camera.py").is_file()


def test_native_camera_layer_has_no_vendor_dependency() -> None:
    native = ROOT / "src" / "drivers" / "real" / "camera" / "native"
    for path in native.glob("*"):
        if path.suffix not in {".hpp", ".cpp"}:
            continue
        text = path.read_text(encoding="utf-8")
        assert "libobsensor" not in text
        assert "OrbbecSDK" not in text


def test_orbbec_dependency_path_is_below_deps() -> None:
    clone_script = (ROOT / "scripts" / "build" / "clone_orbbec_ros2.sh").read_text(encoding="utf-8")
    fetch_script = (ROOT / "scripts" / "build" / "fetch_orbbec_sdk.sh").read_text(encoding="utf-8")
    build_script = (ROOT / "scripts" / "build" / "build_orbbec_native.sh").read_text(encoding="utf-8")

    assert "src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2" in clone_script
    assert "src/drivers/real/camera/deps/orbbec/OrbbecSDK_ROS2" in build_script
    assert "src/drivers/real/camera/deps/orbbec/OrbbecSDK" in fetch_script
    assert "orbbec/OrbbecSDK_v2" in fetch_script
    assert "dpkg-deb -x" in fetch_script
    assert ".lingtu-orbbec-sdk" in fetch_script
    assert "OrbbecSDK_ROS2" not in fetch_script
