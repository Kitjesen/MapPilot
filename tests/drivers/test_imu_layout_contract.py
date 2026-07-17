from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_real_and_sim_imu_share_layout() -> None:
    real = ROOT / "src" / "drivers" / "real" / "imu"
    sim = ROOT / "src" / "drivers" / "sim" / "imu"

    for base in (real, sim):
        assert (base / "__init__.py").is_file()
        assert (base / "module.py").is_file()
        assert (base / "native").is_dir()
        assert (base / "impl").is_dir()
        assert (base / "deps").is_dir()

    assert (real / "native" / "sdk.py").is_file()
    assert (sim / "native" / "sdk.py").is_file()
    assert (real / "impl" / "livox").is_dir()
    assert (sim / "impl" / "mujoco").is_dir()
