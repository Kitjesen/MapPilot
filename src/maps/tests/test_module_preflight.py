from __future__ import annotations

from types import SimpleNamespace

from maps.modules.service import MapsModule


def test_maps_module_preflight_accepts_native_map_types_contract() -> None:
    module = SimpleNamespace(
        api=SimpleNamespace(
            get_map_types=lambda: {
                "success": True,
                "schema_version": "map.types",
            }
        )
    )

    assert MapsModule.preflight(module) is None


def test_maps_module_preflight_rejects_unusable_native_service() -> None:
    class BrokenAPI:
        @staticmethod
        def get_map_types():
            raise RuntimeError("probe failed")

    reason = MapsModule.preflight(SimpleNamespace(api=BrokenAPI()))

    assert reason == "native_map_service_unavailable: probe failed"


def test_maps_module_preflight_rejects_wrong_contract() -> None:
    module = SimpleNamespace(
        api=SimpleNamespace(
            get_map_types=lambda: {
                "success": True,
                "schema_version": "unexpected",
            }
        )
    )

    assert MapsModule.preflight(module) == (
        "native_map_service_unavailable: expected map.types response"
    )
