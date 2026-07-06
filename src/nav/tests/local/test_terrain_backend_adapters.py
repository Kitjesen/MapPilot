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
        self.terrain_voxel_size = 1.0
        self.terrain_voxel_half_width = 10
        self.decay_time = 10.0
        self.no_decay_dis = 4.0
        self.clearing_dis = 8.0
        self.use_sorting = True
        self.quantile_z = 0.25
        self.consider_drop = False
        self.limit_ground_lift = False
        self.max_ground_lift = 0.15
        self.clear_dy_obs = False
        self.min_dy_obs_dis = 0.3
        self.min_dy_obs_angle = 0.0
        self.min_dy_obs_rel_z = -0.5
        self.abs_dy_obs_rel_z_thre = 0.2
        self.min_dy_obs_vfov = -16.0
        self.max_dy_obs_vfov = 16.0
        self.min_dy_obs_point_num = 1
        self.min_out_of_fov_point_num = 2
        self.obstacle_height_thre = 0.2
        self.no_data_obstacle = False
        self.no_data_block_skip_num = 0
        self.min_block_point_num = 10
        self.vehicle_height = 0.6
        self.voxel_point_update_thre = 100
        self.voxel_time_update_thre = 2.0
        self.min_rel_z = -0.5
        self.max_rel_z = 0.25
        self.dis_ratio_z = 0.2
        self.planar_voxel_size = 0.2
        self.planar_voxel_half_width = 25


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
            "terrain": {
                "clear_dy_obs": False,
                "no_data_obstacle": False,
            },
            "terrain_analysis": {
                "scan_voxel_size": 0.12,
                "terrain_voxel_size": 0.9,
                "terrain_voxel_half_width": 8,
                "decay_time": 7.5,
                "no_decay_dis": 3.25,
                "clearing_dis": 6.5,
                "use_sorting": False,
                "quantile_z": 0.4,
                "consider_drop": True,
                "limit_ground_lift": True,
                "max_ground_lift": 0.2,
                "clear_dy_obs": True,
                "min_dy_obs_dis": 0.45,
                "min_dy_obs_angle": 2.0,
                "min_dy_obs_rel_z": -0.4,
                "abs_dy_obs_rel_z_thre": 0.18,
                "min_dy_obs_vfov": -12.0,
                "max_dy_obs_vfov": 14.0,
                "min_dy_obs_point_num": 3,
                "min_out_of_fov_point_num": 4,
                "obstacle_height_thre": 0.31,
                "no_data_obstacle": True,
                "no_data_block_skip_num": 2,
                "min_block_point_num": 5,
                "vehicle_height": 0.72,
                "voxel_point_update_thre": 23,
                "voxel_time_update_thre": 1.4,
                "min_rel_z": -0.33,
                "max_rel_z": 0.44,
                "dis_ratio_z": 0.12,
                "planar_voxel_size": 0.15,
                "planar_voxel_half_width": 16,
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
    assert backend.params.terrain_voxel_size == 0.9
    assert backend.params.terrain_voxel_half_width == 8
    assert backend.params.decay_time == 7.5
    assert backend.params.no_decay_dis == 3.25
    assert backend.params.clearing_dis == 6.5
    assert backend.params.use_sorting is False
    assert backend.params.quantile_z == 0.4
    assert backend.params.consider_drop is True
    assert backend.params.limit_ground_lift is True
    assert backend.params.max_ground_lift == 0.2
    assert backend.params.clear_dy_obs is True
    assert backend.params.min_dy_obs_dis == 0.45
    assert backend.params.min_dy_obs_angle == 2.0
    assert backend.params.min_dy_obs_rel_z == -0.4
    assert backend.params.abs_dy_obs_rel_z_thre == 0.18
    assert backend.params.min_dy_obs_vfov == -12.0
    assert backend.params.max_dy_obs_vfov == 14.0
    assert backend.params.min_dy_obs_point_num == 3
    assert backend.params.min_out_of_fov_point_num == 4
    assert backend.params.obstacle_height_thre == 0.31
    assert backend.params.no_data_obstacle is True
    assert backend.params.no_data_block_skip_num == 2
    assert backend.params.min_block_point_num == 5
    assert backend.params.vehicle_height == 0.72
    assert backend.params.voxel_point_update_thre == 23
    assert backend.params.voxel_time_update_thre == 1.4
    assert backend.params.min_rel_z == -0.33
    assert backend.params.max_rel_z == 0.44
    assert backend.params.dis_ratio_z == 0.12
    assert backend.params.planar_voxel_size == 0.15
    assert backend.params.planar_voxel_half_width == 16
    assert backend.effective_params == {
        "scan_voxel_size": 0.12,
        "terrain_voxel_size": 0.9,
        "terrain_voxel_half_width": 8,
        "decay_time": 7.5,
        "no_decay_dis": 3.25,
        "clearing_dis": 6.5,
        "use_sorting": False,
        "quantile_z": 0.4,
        "consider_drop": True,
        "limit_ground_lift": True,
        "max_ground_lift": 0.2,
        "clear_dy_obs": True,
        "min_dy_obs_dis": 0.45,
        "min_dy_obs_angle": 2.0,
        "min_dy_obs_rel_z": -0.4,
        "abs_dy_obs_rel_z_thre": 0.18,
        "min_dy_obs_vfov": -12.0,
        "max_dy_obs_vfov": 14.0,
        "min_dy_obs_point_num": 3,
        "min_out_of_fov_point_num": 4,
        "obstacle_height_thre": 0.31,
        "no_data_obstacle": True,
        "no_data_block_skip_num": 2,
        "min_block_point_num": 5,
        "vehicle_height": 0.72,
        "voxel_point_update_thre": 23,
        "voxel_time_update_thre": 1.4,
        "min_rel_z": -0.33,
        "max_rel_z": 0.44,
        "dis_ratio_z": 0.12,
        "planar_voxel_size": 0.15,
        "planar_voxel_half_width": 16,
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
