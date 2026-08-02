"""Shared Blueprint and native-navigation defaults.

These values tune navigation implementations. They do not declare a Product
or select a Profile adapter.
"""

from __future__ import annotations

from runtime.profiles.catalog.runtime_paths import _resolve_octoplanner3d_map

ACTIVE_OCTOPLANNER3D_MAP = _resolve_octoplanner3d_map()

THUNDER_MAP_ARTIFACT_DEFAULTS = {
    "octomap_resolution": 0.1,
    "octomap_free_layers_above": 6,
    "octomap_free_dilation_cells": 1,
}

THUNDER_OCTOPLANNER_DEFAULTS = {
    **THUNDER_MAP_ARTIFACT_DEFAULTS,
    "preview_timeout": 30.0,
    "octoplanner3d_timeout_s": 30.0,
    "octoplanner3d_robot_radius": 0.25,
    "octoplanner3d_max_iterations": 500000,
    "octoplanner3d_snap_search_radius_cells": 24,
    "octoplanner3d_require_ground_support": True,
    "octoplanner3d_strict_direct_ground_support": False,
    "octoplanner3d_ground_support_xy_radius_cells": 2,
    "octoplanner3d_ground_support_depth_cells": 2,
    "octoplanner3d_enable_preblocked_costmap": True,
    "octoplanner3d_preblocked_costmap_radius_cells": 3,
    "octoplanner3d_preblocked_costmap_weight": 2.5,
    "octoplanner3d_lowest_traversable_only": False,
    "octoplanner3d_floor_change_penalty": 6.0,
    "octoplanner3d_max_step_height": 0.45,
    "octoplanner3d_max_slope": 0.0,
    "octoplanner3d_same_floor_preference": True,
    "octoplanner3d_same_floor_z_tolerance": 0.75,
    "octoplanner3d_max_same_floor_z_excursion": 2.0,
    "octoplanner3d_obstacle_clearance_radius_cells": 2,
    "octoplanner3d_obstacle_clearance_weight": 1.5,
}


__all__ = [
    "ACTIVE_OCTOPLANNER3D_MAP",
    "THUNDER_MAP_ARTIFACT_DEFAULTS",
    "THUNDER_OCTOPLANNER_DEFAULTS",
]
