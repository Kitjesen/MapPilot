"""Map transport ownership must stay inside native mapd and runtime endpoints."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_retired_python_map_dds_adapter_is_absent():
    assert not (REPO_ROOT / "src/maps/adapters/dds/output.py").exists()
    assert not (REPO_ROOT / "src/maps/adapters/resolver.py").exists()
    assert not (REPO_ROOT / "src/runtime/adapters/dds/map_output.py").exists()
    assert not (REPO_ROOT / "src/nav/adapters/dds/nav/map_out.py").exists()
    assert not (REPO_ROOT / "src/runtime/adapters/mapping_slam.py").exists()
    assert not (REPO_ROOT / "src/runtime/tests/test_dds_map_adapter.py").exists()
    assert not (REPO_ROOT / "src/gateway/services/map_cache.py").exists()


def test_retired_map_implementations_are_not_reintroduced():
    retired = (
        "src/nav/services/map",
        "src/nav/services/maps.py",
        "src/runtime/map_save.py",
        "src/runtime/dynamic_filter.py",
        "src/runtime/same_source_map_artifacts.py",
        "src/gateway/services/map_paths.py",
        "src/gateway/services/pcd.py",
    )

    assert [path for path in retired if (REPO_ROOT / path).exists()] == []
