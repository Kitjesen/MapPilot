from __future__ import annotations

import os


_CYCLONE_DEFAULT_PORT_BASE = 7400
_CYCLONE_DEFAULT_DOMAIN_GAIN = 250
_CYCLONE_MAXIMUM_FIXED_PORT_OFFSET = 11
_WINDOWS_DYNAMIC_PORT_START = 49152
_WINDOWS_PRODUCT_DDS_DOMAIN_ID = 17
_FIRST_NAVIGATION_TEST_DDS_DOMAIN_ID = 20


def domain_id_from_environment(name: str = "LINGTU_TEST_DDS_DOMAIN_ID") -> int:
    configured = os.environ.get(name)
    if configured is None:
        raise RuntimeError(f"{name} must be assigned by the test build")
    domain_id = int(configured)
    if not 0 <= domain_id < _FIRST_NAVIGATION_TEST_DDS_DOMAIN_ID:
        raise RuntimeError(f"{name} is outside the adapter low-domain test slots")
    if domain_id == _WINDOWS_PRODUCT_DDS_DOMAIN_ID:
        raise RuntimeError(f"{name} uses the Windows Product runtime DDS domain")
    highest_fixed_port = (
        _CYCLONE_DEFAULT_PORT_BASE
        + _CYCLONE_DEFAULT_DOMAIN_GAIN * domain_id
        + _CYCLONE_MAXIMUM_FIXED_PORT_OFFSET
    )
    if highest_fixed_port >= _WINDOWS_DYNAMIC_PORT_START:
        raise RuntimeError(f"{name} maps CycloneDDS fixed ports into Windows dynamic ports")
    return domain_id


def require_unique_domains(*domain_ids: int) -> None:
    if len(set(domain_ids)) != len(domain_ids):
        raise RuntimeError("DDS test domains must be unique")
