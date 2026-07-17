from __future__ import annotations

from pathlib import Path

from maps.paths import active_map_dir, map_root_candidates, nav_map_root


def test_map_root_candidates_put_configured_root_first(
    tmp_path: Path,
    monkeypatch,
) -> None:
    configured = tmp_path / "maps"
    monkeypatch.setenv("NAV_MAP_DIR", str(configured))

    candidates = map_root_candidates()

    assert candidates[0] == configured.resolve()
    assert nav_map_root() == configured.resolve()
    assert len(candidates) == len(set(candidates))


def test_active_map_dir_resolves_native_state_file(tmp_path: Path) -> None:
    selected = tmp_path / "warehouse"
    selected.mkdir()
    (tmp_path / "active_map.txt").write_text("warehouse\n", encoding="utf-8")

    assert active_map_dir(tmp_path) == selected.resolve()


def test_active_map_dir_accepts_materialized_active_package(tmp_path: Path) -> None:
    active = tmp_path / "active"
    active.mkdir()

    assert active_map_dir(tmp_path) == active.resolve()


def test_active_map_dir_rejects_missing_native_selection(tmp_path: Path) -> None:
    (tmp_path / "active_map.txt").write_text("missing\n", encoding="utf-8")

    assert active_map_dir(tmp_path) is None
