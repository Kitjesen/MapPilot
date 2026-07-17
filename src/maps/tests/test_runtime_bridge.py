from __future__ import annotations

import pytest

from maps.services.runtime_bridge import MapRuntimeBridge


@pytest.mark.parametrize(
    ("profile", "expected"),
    [
        (
            "super_lio",
            {
                "map_save_supported": True,
                "map_save_source": "live_map_cloud_snapshot",
                "relocalization_supported": False,
                "saved_map_relocalization_supported": False,
                "restart_recovery_supported": True,
                "recovery_method": "restart_super_lio",
            },
        ),
        (
            "super_lio_relocation",
            {
                "map_save_supported": False,
                "map_save_source": "active_map",
                "relocalization_supported": False,
                "saved_map_relocalization_supported": False,
                "restart_recovery_supported": True,
                "recovery_method": "restart_super_lio_relocation",
            },
        ),
        (
            "localizer",
            {
                "map_save_supported": True,
                "map_save_source": "slam_service",
                "relocalization_supported": True,
                "saved_map_relocalization_supported": True,
                "restart_recovery_supported": True,
                "recovery_method": "relocalize_service",
            },
        ),
        (
            "native_dds",
            {
                "map_save_supported": True,
                "map_save_source": "native_slam_dds_control",
                "relocalization_supported": True,
                "saved_map_relocalization_supported": True,
                "restart_recovery_supported": False,
                "recovery_method": "restart_native_dds_slam",
            },
        ),
    ],
)
def test_map_save_capability_contract(profile: str, expected: dict) -> None:
    assert MapRuntimeBridge.map_save_capability_fields(profile) == expected


@pytest.mark.parametrize(
    ("alias", "canonical"),
    [
        ("super-lio", "super_lio"),
        ("super_lio_reloc", "super_lio_relocation"),
        ("relocation", "super_lio_relocation"),
    ],
)
def test_slam_profile_aliases_are_normalized(alias: str, canonical: str) -> None:
    assert MapRuntimeBridge.normalize_slam_profile(alias) == canonical
