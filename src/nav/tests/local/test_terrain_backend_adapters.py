from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest

from nav.local.terrain_backend import (
    create_nanobind_terrain_backend,
)


class FakeTerrainParams:
    def __init__(self) -> None:
        self.scan_voxel_size = 0.05
        self.decay_time = 10.0
        self.no_decay_dis = 4.0
        self.obstacle_height_thre = 0.2
        self.vehicle_height = 0.6
        self.min_rel_z = -0.5
        self.max_rel_z = 0.25


class FakeTerrainAnalysisCore:
    def __init__(self, params: FakeTerrainParams) -> None:
        self.params = params


class FakeNavKernel:
    TerrainParams = FakeTerrainParams
    TerrainAnalysisCore = FakeTerrainAnalysisCore

def test_nanobind_terrain_adapter_assembles_fake_params() -> None:
    seen_symbols: list[tuple[str, ...]] = []
    cfg = SimpleNamespace(
        raw={
            "terrain_analysis": {
                "scan_voxel_size": 0.12,
                "decay_time": 7.5,
                "no_decay_dis": 3.25,
                "obstacle_height_thre": 0.31,
                "vehicle_height": 0.72,
                "min_rel_z": -0.33,
                "max_rel_z": 0.44,
            }
        }
    )

    def importer(symbols: tuple[str, ...]) -> Any:
        seen_symbols.append(symbols)
        return FakeNavKernel

    backend = create_nanobind_terrain_backend(
        nav_kernel_importer=importer,
        build_hint_provider=lambda: "build hint",
        config_getter=lambda: cfg,
    )

    assert seen_symbols == [("TerrainParams", "TerrainAnalysisCore")]
    assert isinstance(backend.core, FakeTerrainAnalysisCore)
    assert backend.runtime is FakeNavKernel
    assert backend.core.params is backend.params
    assert backend.params.scan_voxel_size == 0.12
    assert backend.params.decay_time == 7.5
    assert backend.params.no_decay_dis == 3.25
    assert backend.params.obstacle_height_thre == 0.31
    assert backend.params.vehicle_height == 0.72
    assert backend.params.min_rel_z == -0.33
    assert backend.params.max_rel_z == 0.44
    assert backend.effective_params == {
        "scan_voxel_size": 0.12,
        "decay_time": 7.5,
        "no_decay_dis": 3.25,
        "obstacle_height_thre": 0.31,
        "vehicle_height": 0.72,
        "min_rel_z": -0.33,
        "max_rel_z": 0.44,
    }


def test_nanobind_terrain_adapter_missing_runtime_raises_build_hint() -> None:
    with pytest.raises(RuntimeError) as excinfo:
        create_nanobind_terrain_backend(
            nav_kernel_importer=lambda _symbols: None,
            build_hint_provider=lambda: "run build_nav_kernel",
        )

    message = str(excinfo.value)
    assert "Terrain [nanobind]: compatible LingTu native navigation kernel not found" in message
    assert "backend='simple'" in message
    assert "run build_nav_kernel" in message
