from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim" / "scripts" / "setup_linux_validation_host.sh"


def test_native_validation_host_setup_prepares_current_runtime() -> None:
    text = SCRIPT.read_text(encoding="utf-8")

    assert "install_system_deps" in text
    assert "install_python_deps" in text
    assert "verify_mid360_pattern_asset" in text
    assert "src/runtime/tests/test_sim_runtime_adapters.py" in text
    assert "np.load(path, mmap_mode=\"r\")" in text
