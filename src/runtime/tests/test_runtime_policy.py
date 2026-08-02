from __future__ import annotations

import pytest

from runtime.runtime_policy import (
    backend_capability_defaults,
    default_slam_profile_for_mode,
    is_supported_slam_profile,
    normalize_slam_profile,
    slam_backend_contract,
)


@pytest.mark.parametrize(
    "profile",
    ("none", "native_dds", "fastlio2", "genz", "localizer"),
)
def test_canonical_slam_profiles_are_supported(profile: str):
    assert normalize_slam_profile(f" {profile.upper()} ") == profile
    assert is_supported_slam_profile(profile)


@pytest.mark.parametrize(
    "alias",
    (
        "slam",
        "native",
        "native_slam",
        "cpp_dds_slam",
        "lingtu_slam_dds",
        "lingtu-slam-dds",
        "genz-icp",
        "genz_icp",
    ),
)
def test_legacy_slam_profile_aliases_are_not_supported(alias: str):
    assert normalize_slam_profile(alias) == alias
    assert not is_supported_slam_profile(alias)
    assert slam_backend_contract(alias)["backend"] == alias


def test_stop_is_only_supported_for_explicit_switch_validation():
    assert is_supported_slam_profile("stop", allow_stop=True)
    assert not is_supported_slam_profile("stop")


def test_field_mode_defaults_to_native_dds_slam():
    assert default_slam_profile_for_mode("navigating") == "native_dds"
    assert default_slam_profile_for_mode("mapping") == "native_dds"


def test_backend_capabilities_preserve_supported_contracts():
    genz = backend_capability_defaults("genz")
    localizer = backend_capability_defaults("localizer")
    native = backend_capability_defaults("native_dds")
    unknown = backend_capability_defaults("unknown_backend")

    assert genz["recovery_method"] == "restart_genz_icp"
    assert localizer["relocalization_supported"] is True
    assert native["map_save_source"] == "native_dds_slam_runtime"
    assert native["saved_map_relocalization_supported"] is True
    assert unknown["relocalization_supported"] is False


def test_retired_slam_alias_uses_unknown_backend_capabilities():
    assert backend_capability_defaults("slam") == backend_capability_defaults("unknown_backend")


def test_slam_bridge_contract_adds_health_source_and_recovery_action():
    contract = slam_backend_contract("genz")

    assert contract["backend"] == "genz"
    assert contract["health_source"] == "odom_map_cloud"
    assert contract["map_save_source"] is None
    assert contract["recovery_action"] == "restart_genz_icp"
