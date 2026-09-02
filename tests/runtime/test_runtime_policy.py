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
    ("none", "native_dds"),
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
        "messages_dds",
        "lingtu-slam-dds",
        "genz-icp",
        "genz_icp",
    ),
)
def test_legacy_slam_profile_aliases_are_not_supported(alias: str):
    assert normalize_slam_profile(alias) == alias
    assert not is_supported_slam_profile(alias)
    assert slam_backend_contract(alias)["backend"] == alias


def test_stop_is_not_a_runtime_profile():
    assert not is_supported_slam_profile("stop")


@pytest.mark.parametrize("profile", ("fastlio2", "localizer", "pointlio", "genz"))
def test_algorithm_and_placeholder_names_are_not_product_selectable(profile: str):
    assert not is_supported_slam_profile(profile)


def test_field_mode_defaults_to_native_dds_slam():
    assert default_slam_profile_for_mode("navigating") == "native_dds"
    assert default_slam_profile_for_mode("mapping") == "native_dds"
    assert default_slam_profile_for_mode("none") == "none"


def test_backend_capabilities_preserve_supported_contracts():
    localizer = backend_capability_defaults("localizer")
    native = backend_capability_defaults("native_dds")
    unknown = backend_capability_defaults("unknown_backend")

    assert localizer["relocalization_supported"] is False
    assert localizer["recovery_method"] == "unknown_backend"
    assert native["map_save_source"] == "native_slam_dds_control"
    assert native["saved_map_relocalization_supported"] is True
    assert unknown["relocalization_supported"] is False


def test_retired_slam_alias_uses_unknown_backend_capabilities():
    assert backend_capability_defaults("slam") == backend_capability_defaults("unknown_backend")


def test_slam_bridge_contract_preserves_native_and_unknown_defaults():
    native = slam_backend_contract("native_dds")
    unknown = slam_backend_contract("unknown_backend")

    assert native["backend"] == "native_dds"
    assert native["health_source"] == "slam_runtime"
    assert native["map_save_source"] == "native_slam_dds_control"
    assert native["recovery_action"] == "restart_native_dds_slam"
    assert unknown["backend"] == "unknown_backend"
    assert unknown["health_source"] == "unknown"
    assert unknown["map_save_source"] is None
    assert unknown["recovery_action"] == "none"
