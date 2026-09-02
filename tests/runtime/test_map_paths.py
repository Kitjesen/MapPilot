from __future__ import annotations

from pathlib import Path

from runtime.endpoints.map_paths import DEFAULT_NAV_MAP_DIR, nav_map_root


def test_nav_map_root_uses_configured_root(tmp_path: Path, monkeypatch) -> None:
    configured = tmp_path / "maps"
    monkeypatch.setenv("NAV_MAP_DIR", str(configured))

    assert nav_map_root() == configured.resolve()


def test_nav_map_root_default_is_lingtu_local_data() -> None:
    assert DEFAULT_NAV_MAP_DIR == "~/data/lingtu/maps"
