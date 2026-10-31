from __future__ import annotations

from pathlib import Path


def test_cli_map_activation_stays_with_product_control() -> None:
    root = Path(__file__).resolve().parents[2]
    sdk_cli = (root / "src/lingtu/sdk/cli.py").read_text(encoding="utf-8")
    sdk_client = (root / "src/lingtu/sdk/client.py").read_text(encoding="utf-8")
    adapter = (root / "scripts/lingtu").read_text(encoding="utf-8")
    entrypoints = (root / "pyproject.toml").read_text(encoding="utf-8")

    assert "set_active" not in sdk_cli + sdk_client
    assert "/pgo/save_maps" not in sdk_cli + sdk_client
    assert 'lingtu = "lingtu.control:main"' in entrypoints
    assert '-m lingtu.control "$@"' in adapter
