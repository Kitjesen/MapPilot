"""The licensed source bay is inventory, not an invented collision world."""

import sys
from types import SimpleNamespace

import pytest
from sim.tools.worlds.factory_environment_collection.bay_intake import main, select_bay


def test_bay_inventory_excludes_distant_actors_and_flags_crossing_geometry():
    actors = [
        {"name": "cabinet", "bounds_min_cm": [10, 20, 0], "bounds_max_cm": [30, 40, 80]},
        {"name": "floor", "bounds_min_cm": [-1000, -1000, -10], "bounds_max_cm": [1000, 1000, 0]},
        {"name": "other_bay", "bounds_min_cm": [300, 300, 0], "bounds_max_cm": [400, 400, 80]},
    ]
    selected = select_bay(actors, minimum_cm=[0, 0, -20], maximum_cm=[100, 100, 100])
    assert [actor["name"] for actor in selected] == ["cabinet", "floor"]
    assert [actor["crosses_bay_boundary"] for actor in selected] == [False, True]
    assert "crosses_bay_boundary" not in actors[0]


def test_commandlet_uses_the_supported_paths_api_before_loading_map(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_FACTORY_BAY_INTAKE_DIR", str(tmp_path))
    monkeypatch.setitem(sys.modules, "unreal", SimpleNamespace(
        Paths=SimpleNamespace(get_project_file_path=lambda: "D:/FactoryPreview/FactoryProject.uproject"),
        EditorLoadingAndSavingUtils=SimpleNamespace(load_map=lambda _: None),
    ))
    with pytest.raises(RuntimeError, match="world did not load"):
        main()
    assert not (tmp_path / "bay-intake.json").exists()
