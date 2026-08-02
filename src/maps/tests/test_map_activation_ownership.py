from __future__ import annotations

import json

from maps.services.facade import MapsFacadeMixin


def test_maps_domain_does_not_activate_product_runtime_directly() -> None:
    facade = object.__new__(MapsFacadeMixin)
    facade._map_set_active = lambda _name: (_ for _ in ()).throw(
        AssertionError("maps domain must not activate runtime state directly")
    )

    result = json.loads(facade.use_map("warehouse"))

    assert result == {
        "success": False,
        "reason_code": "product_map_transaction_required",
        "message": "Map activation is owned by the product runtime transaction.",
    }


def test_legacy_use_map_is_not_exported_as_mcp_skill() -> None:
    assert getattr(MapsFacadeMixin.use_map, "_skill_info", None) is None
