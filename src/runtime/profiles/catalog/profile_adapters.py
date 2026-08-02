"""Local Host Profile adapter catalog.

Adapters connect local/development Host Profiles to a concrete driver, data
source, or external simulation launcher. They are not communication endpoints
and do not participate in Product or Env identity.
"""

from __future__ import annotations

import sys
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from runtime.profiles.catalog.profile_adapter_configs import (
    MUJOCO_LIVE_CONFIG,
)
from runtime.profiles.catalog.runtime_paths import (
    _resolve_octoplanner3d_map,
)


class ProfileAdapterError(ValueError):
    """Raised when a local Host Profile cannot use an adapter."""


@dataclass(frozen=True)
class ProfileAdapterSpec:
    """Adapter contract for one local Host Profile integration."""
    name: str
    description: str
    data_source: str
    driver_backend: str
    simulation_only: bool
    supported_profiles: tuple[str, ...] = ()
    module_transport: str = "local"
    endpoint_transport: str = "local"
    endpoint_contract: str | None = None
    external_launcher: str | None = None
    runtime_contract: str | None = None
    config_overrides: Mapping[str, Any] = field(default_factory=dict)
    profile_overrides: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    env_overrides: Mapping[str, str] = field(default_factory=dict)
    default_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    record_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    def require_profile(self, profile: str) -> None:
        """Raise if this adapter does not support the requested Profile."""
        if profile not in self.supported_profiles:
            supported = ", ".join(self.supported_profiles)
            raise ProfileAdapterError(
                f"profile adapter '{self.name}' does not support profile '{profile}' "
                f"(supported: {supported})"
            )

    def config_for_profile(self, profile: str) -> dict[str, Any]:
        """Return adapter overrides for the requested Profile."""
        from runtime.profiles.profile_adapter_config import profile_adapter_config_for_profile

        return profile_adapter_config_for_profile(self, profile)

@dataclass(frozen=True)
class RuntimeRunSpec:
    """Resolved launch/runtime contract for a local Host Profile."""
    profile: str
    adapter: str | None
    data_source: str
    runtime_contract: str | None
    module_transport: str
    endpoint_transport: str
    endpoint_contract: str | None
    route_contract: str | None
    simulation_only: bool
    command_sink: str
    slam_source: str
    localization_source: str
    mapping_source: str
    lidar_extrinsic_profile: str | None
    frames: Mapping[str, Any]
    frame_links: Mapping[str, Mapping[str, Any]]
    topic_allowed_frame_ids: Mapping[str, tuple[str, ...]]
    topic_default_frame_ids: Mapping[str, str]
    resolved_runtime_data_flow: tuple[Mapping[str, Any], ...]
    runtime_data_flow_stage_algorithm_interfaces: Mapping[str, tuple[str, ...]]
    launcher: str | None
    launcher_args: tuple[str, ...]
    env: Mapping[str, str]
    profile_semantic_overrides: tuple[Mapping[str, Any], ...] = ()
    localization_adapter: str | None = None
    global_planner: str | None = None
    fallback_global_planners: tuple[str, ...] = ()
    planner_latency_budget_ms: int | None = None
    plan_safety_policy: str | None = None
    autonomy_backends: Mapping[str, str] = field(default_factory=dict)

    def as_command(self) -> list[str]:
        """Return the launcher command for this runtime spec."""
        if not self.launcher:
            return []
        if Path(self.launcher).suffix == ".py":
            return [sys.executable, self.launcher, *self.launcher_args]
        return ["bash", self.launcher, *self.launcher_args]


PROFILE_ADAPTERS: dict[str, ProfileAdapterSpec] = {
    "thunder_lite": ProfileAdapterSpec(
        name="thunder_lite",
        description="Physical Thunder local lightweight Profile adapter without ROS, SLAM, maps, or gateway.",
        data_source="thunder_lite_local",
        driver_backend="thunder",
        supported_profiles=("lite",),
        simulation_only=False,
        runtime_contract="thunder_lite_local",
        config_overrides={
            "enable_hw": False,
        },
    ),
    "mujoco_live": ProfileAdapterSpec(
        name="mujoco_live",
        description="MuJoCo raw MID-360 + IMU Profile adapter feeding Fast-LIO.",
        data_source="mujoco_fastlio2_live",
        driver_backend="sim_endpoint",
        supported_profiles=(
            "sim_mujoco_live",
            "sim_mujoco_octo_live",
        ),
        simulation_only=True,
        external_launcher="sim/scripts/mujoco/launch_fastlio2_live.sh",
        runtime_contract="mujoco_fastlio2_live",
        config_overrides=MUJOCO_LIVE_CONFIG,
        profile_overrides={
            "sim_mujoco_live": {
                "enable_frontier": True,
                "exploration_backend": "none",
                "frontier_safe_distance": 0.80,
                "frontier_max_dist": 25.0,
                "frontier_rate": 2.0,
            },
            "sim_mujoco_octo_live": {
                "planner": "octoplanner3d",
                "planner_backend": "octoplanner3d",
                "map_path": _resolve_octoplanner3d_map(),
                "plan_safety_policy": "reject",
                "fallback_planner_name": "",
                "enable_frontier": False,
                "enable_traversable_frontier": False,
                "exploration_backend": "none",
                "local_planner_allow_direct_track_fallback": False,
                "local_planner_ignore_near_field_stop": False,
            },
        },
        default_actions={
            "sim_mujoco_live": ("gate",),
            "sim_mujoco_octo_live": ("octo-moving-obstacle-video",),
        },
        record_actions={
            "sim_mujoco_live": ("video",),
            "sim_mujoco_octo_live": ("octo-moving-obstacle-video",),
        },
    ),
}


def profile_adapter_names_for_profile(profile: str) -> tuple[str, ...]:
    """Return adapters that explicitly accept one Host Profile."""

    return tuple(
        adapter_name
        for adapter_name, adapter in PROFILE_ADAPTERS.items()
        if profile in adapter.supported_profiles
    )


__all__ = [
    "PROFILE_ADAPTERS",
    "ProfileAdapterError",
    "ProfileAdapterSpec",
    "RuntimeRunSpec",
    "profile_adapter_names_for_profile",
]
