from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "sim" / "setup_linux_validation_host.sh"


def test_native_validation_host_setup_prepares_current_runtime() -> None:
    text = SCRIPT.read_text(encoding="utf-8")

    assert "install_system_deps" in text
    assert "install_python_deps" in text
    assert "verify_mid360_pattern_asset" in text
    assert "tests/sim/test_imu_dds_adapter.py" in text
    assert "tests/sim/test_truth_odom_dds_adapter.py" in text
    assert "np.load(path, mmap_mode=\"r\")" in text
