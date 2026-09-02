from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_lingtu_shell_is_only_a_product_control_adapter() -> None:
    source = (ROOT / "scripts/lingtu").read_text(encoding="utf-8")

    assert 'exec "$LINGTU_PYTHON" -m lingtu.control "$@"' in source
    for forbidden in ("cmd_", "systemctl", "scripts/gates", "diagnostics.field"):
        assert forbidden not in source


def test_python_acceptance_implementations_live_with_diagnostics() -> None:
    field = ROOT / "src/diagnostics/field"
    for module in (
        "gate_support.py",
        "motion_smoke.py",
        "runtime_evidence.py",
        "map_artifacts.py",
        "map_acceptance.py",
        "system_acceptance.py",
        "service_readiness.py",
    ):
        assert (field / module).is_file()

    assert not list((ROOT / "scripts/gates").glob("*.py"))
