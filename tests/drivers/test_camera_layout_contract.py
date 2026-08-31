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


def test_orbbec_dependency_uses_ignored_build_tree() -> None:
    fetch_script = (ROOT / "scripts" / "build" / "fetch_orbbec_sdk.sh").read_text(encoding="utf-8")
    build_script = (ROOT / "scripts" / "build" / "build_orbbec_native.sh").read_text(encoding="utf-8")

    assert "build/deps/orbbec-sdk" in fetch_script
    assert "build/deps/orbbec-sdk" in build_script
    assert 'REF="${LINGTU_ORBBEC_SDK_REF:-v2.8.7}"' in fetch_script
    assert "orbbec/OrbbecSDK_v2" in fetch_script
    assert "dpkg-deb -x" in fetch_script
    assert ".lingtu-orbbec-sdk" in fetch_script


def test_gemini335_udev_rule_is_installed_by_robot_setup() -> None:
    rule = ROOT / "scripts" / "deploy" / "99-lingtu-orbbec-gemini335.rules"
    setup = (ROOT / "tools" / "robot" / "setup_network.sh").read_text(encoding="utf-8")
    text = rule.read_text(encoding="utf-8")

    assert 'ATTR{idVendor}=="2bc5"' in text
    assert 'ATTR{idProduct}=="0800"' in text
    assert 'MODE:="0666"' in text
    assert "GROUP=" not in text
    assert rule.name in setup
