from __future__ import annotations

import pytest

from runtime.runtime_policy import (
    backend_capability_defaults,
    default_slam_profile_for_mode,
    is_supported_slam_profile,
    normalize_slam_profile,
    session_transition_plan,
    slam_backend_contract,
    slam_switch_plan,
)


def test_slam_profile_aliases_are_canonical():
    assert normalize_slam_profile("super-lio") == "super_lio"
    assert normalize_slam_profile("superlio-reloc") == "super_lio_relocation"
    assert normalize_slam_profile("relocation") == "super_lio_relocation"
    assert is_supported_slam_profile("super-lio")
    assert is_supported_slam_profile("none")
    assert is_supported_slam_profile("stop", allow_stop=True)
    assert not is_supported_slam_profile("stop")


def test_backend_capabilities_preserve_super_lio_contracts():
    super_lio = backend_capability_defaults("super_lio")
    relocation = backend_capability_defaults("relocation")
    localizer = backend_capability_defaults("localizer")

    assert super_lio["map_save_supported"] is True
    assert super_lio["map_save_source"] == "live_map_cloud_snapshot"
    assert super_lio["recovery_method"] == "restart_super_lio"
    assert relocation["map_save_supported"] is False
    assert relocation["map_save_source"] == "active_map"
    assert relocation["saved_map_relocalization_supported"] is False
    assert relocation["recovery_method"] == "restart_super_lio_relocation"
    assert localizer["relocalization_supported"] is True
    assert localizer["recovery_method"] == "relocalize_service"


def test_slam_bridge_contract_adds_health_source_and_recovery_action():
    contract = slam_backend_contract("super_lio_relocation")

    assert contract["backend"] == "super_lio_relocation"
    assert contract["health_source"] == "odom_map_cloud"
    assert contract["map_save_source"] == "active_map"
    assert contract["recovery_method"] == "restart_super_lio_relocation"
    assert contract["recovery_action"] == "restart_super_lio_relocation"


def test_slam_switch_service_plans_preserve_order():
    assert slam_switch_plan("fastlio2").stop == (
        "localizer",
        "super_lio",
        "super_lio_relocation",
    )
    assert slam_switch_plan("fastlio2").ensure == ("slam", "slam_pgo")
    assert slam_switch_plan("fastlio2").wait_ready == ("slam", "slam_pgo")
    assert slam_switch_plan("localizer").stop == (
        "slam_pgo",
        "super_lio",
        "super_lio_relocation",
    )
    assert slam_switch_plan("super_lio").stop == (
        "slam",
        "slam_pgo",
        "localizer",
        "super_lio_relocation",
    )
    assert slam_switch_plan("super_lio_relocation").ensure == (
        "lidar",
        "super_lio_relocation",
    )
    assert slam_switch_plan("stop").stop == (
        "super_lio_relocation",
        "super_lio",
        "slam_pgo",
        "localizer",
        "slam",
    )


def test_session_transition_plans_preserve_existing_mode_behavior():
    assert default_slam_profile_for_mode("navigating") == "localizer"
    assert default_slam_profile_for_mode("mapping") == "fastlio2"

    mapping = session_transition_plan("mapping", "fastlio2")
    assert mapping.stop == ("localizer", "super_lio", "super_lio_relocation")
    assert mapping.ensure == ("slam", "slam_pgo")
    assert mapping.clear_live_map is True

    navigating_fastlio = session_transition_plan("navigating", "fastlio2")
    assert navigating_fastlio.stop == (
        "slam_pgo",
        "super_lio",
        "super_lio_relocation",
    )
    assert navigating_fastlio.ensure == ("slam", "localizer")
    assert navigating_fastlio.clear_live_map is False

    super_lio_mapping = session_transition_plan("mapping", "super_lio")
    assert super_lio_mapping.ensure == ("lidar", "super_lio")
    assert super_lio_mapping.clear_live_map is True


def test_external_sim_session_plan_none_does_not_start_robot_services():
    assert slam_switch_plan("none").stop == (
        "super_lio_relocation",
        "super_lio",
        "slam_pgo",
        "localizer",
        "slam",
    )

    exploring = session_transition_plan("exploring", "none")
    assert exploring.stop == ()
    assert exploring.ensure == ()
    assert exploring.wait_ready == ()
    assert exploring.clear_live_map is True

    navigating = session_transition_plan("navigating", "none")
    assert navigating.stop == ()
    assert navigating.ensure == ()
    assert navigating.wait_ready == ()
    assert navigating.clear_live_map is False


def test_unknown_slam_switch_plan_fails_closed():
    with pytest.raises(ValueError):
        slam_switch_plan("unknown")
