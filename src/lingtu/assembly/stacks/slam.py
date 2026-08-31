"""SLAM stack: an adapter for the external native SLAM runtime.

This stack only creates Module graph nodes. ProductControl owns field process
lifecycle through its internal SystemdRunner.
"""

from __future__ import annotations

from localization.adapters.resolver import localization_adapter_module
from runtime.blueprint import Blueprint
from runtime.runtime_policy import is_supported_slam_profile, normalize_slam_profile


def slam(
    profile: str = "native_dds",
    localization_adapter: str | None = None,
    endpoint_contract: str | None = None,
) -> Blueprint:
    """Build the SLAM/localization stack.

    C++ owns SLAM execution. The Python Host only instantiates an explicit
    transport/status adapter for the external native endpoint.
    """

    bp = Blueprint()
    profile = normalize_slam_profile(profile)
    adapter = str(localization_adapter or "").strip()

    if not profile or profile == "none":
        return bp
    if not adapter:
        raise ValueError(
            f"slam_profile={profile!r} requires an explicit localization_adapter; "
            "SLAM execution is owned by the external native runtime"
        )
    if not is_supported_slam_profile(profile):
        raise ValueError(f"unsupported native SLAM profile: {profile}")
    module_cls = localization_adapter_module(adapter)

    kwargs = {"backend_profile": profile}
    if endpoint_contract:
        kwargs["endpoint_contract"] = endpoint_contract
    bp.add(module_cls, alias="SlamAdapterModule", **kwargs)

    return bp
