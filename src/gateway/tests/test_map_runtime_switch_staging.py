# ruff: noqa: S101

from __future__ import annotations

import asyncio
import json

import pytest
from pydantic import ValidationError

pytest.importorskip("fastapi")


class FakeMaps:
    def __init__(self, *, active: str = "warehouse") -> None:
        self.active = active
        self.commands: list[dict[str, object]] = []
        self.versions = {"warehouse": 2, "office": 7}
        self.hashes = {"warehouse": "warehouse-sha", "office": "office-sha"}

    def _identity(self, name: str) -> dict[str, object]:
        version = self.versions[name]
        return {
            "active": name,
            "record": {
                "map_id": name,
                "version": version,
                "version_id": f"{name}-lineage:v{version}",
                "scope": {"frame_id": "map"},
                "artifacts": [
                    {
                        "type": "POINTCLOUD",
                        "uri": f"/maps/{name}/map.pcd",
                        "hash": self.hashes[name],
                    }
                ],
            },
        }

    def execute(self, request):
        command = request.to_mapping()
        self.commands.append(command)
        if command["action"] == "get_active":
            if not self.active:
                return {
                    "success": False,
                    "reason_code": "map_not_found",
                    "message": "no active map",
                }
            return {"success": True, **self._identity(self.active)}
        if command["action"] == "get_record":
            name = str(command["name"])
            if name not in self.versions:
                return {
                    "success": False,
                    "reason_code": "map_not_found",
                    "message": f"map not found: {name}",
                }
            return {"success": True, **self._identity(name)}
        if command["action"] == "set_active":
            previous = self.active
            self.active = str(command["name"])
            return {
                "success": True,
                "previous_active": previous,
                **self._identity(self.active),
            }
        if command["action"] == "clear_active":
            self.active = ""
            return {"success": True, "active": ""}
        raise AssertionError(command)


class MissingStagedIdentityMaps(FakeMaps):
    def __init__(self, *, active: str = "warehouse", fail_restore: bool = False) -> None:
        super().__init__(active=active)
        self.fail_restore = fail_restore

    def execute(self, request):
        command = request.to_mapping()
        if command["action"] == "set_active" and command["name"] == "office":
            response = super().execute(request)
            response.pop("record")
            return response
        if command["action"] == "set_active" and self.fail_restore:
            self.commands.append(command)
            return {
                "success": False,
                "reason_code": "rollback_injected",
                "message": "injected pointer restore failure",
            }
        return super().execute(request)


class MissingPreviousIdentityMaps(FakeMaps):
    def execute(self, request):
        command = request.to_mapping()
        if command["action"] == "get_active":
            self.commands.append(command)
            return {"success": True, "active": self.active}
        return super().execute(request)


def _endpoint(gateway, path: str = "/api/v1/map/stage-for-runtime-switch"):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response):
    if hasattr(response, "body"):
        return json.loads(response.body)
    return response.model_dump() if hasattr(response, "model_dump") else response


def _identity(name: str, version: int) -> dict[str, object]:
    return {
        "map_id": name,
        "version": version,
        "version_id": f"{name}-lineage:v{version}",
        "frame_id": "map",
        "artifacts": [
            {
                "type": "POINTCLOUD",
                "uri": f"/maps/{name}/map.pcd",
                "hash": f"{name}-sha",
            }
        ],
    }


def _restore_request(
    *,
    target: dict[str, object] | None = None,
    previous: dict[str, object] | None = None,
    changed: bool = True,
):
    from gateway.schemas import MapActivationRestoreRequest

    return MapActivationRestoreRequest(
        activation_token={
            "schema_version": "lingtu.map_activation.v1",
            "target": target or _identity("office", 7),
            "previous": previous,
            "changed": changed,
        }
    )


def test_runtime_switch_staging_requires_motion_to_be_stopped() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "navigating"

    response = asyncio.run(_endpoint(gateway)(MapNameRequest(name="office")))

    assert response.status_code == 409
    assert _payload(response)["reason_code"] == "motion_session_active"
    assert maps.commands == []


def test_runtime_switch_staging_is_explicitly_not_runtime_consistent() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(_endpoint(gateway)(MapNameRequest(name="office")))
    payload = _payload(response)

    assert payload["success"] is True
    assert payload["transaction"] == {
        "operation": "stage_map_for_runtime_switch",
        "state": "staged",
        "previous_active": "warehouse",
        "target_map": "office",
        "runtime_consistent": False,
        "restart_required": True,
    }
    assert payload["activation_token"] == {
        "schema_version": "lingtu.map_activation.v1",
        "changed": True,
        "target": {
            "map_id": "office",
            "version": 7,
            "version_id": "office-lineage:v7",
            "frame_id": "map",
            "artifacts": [
                {
                    "type": "POINTCLOUD",
                    "uri": "/maps/office/map.pcd",
                    "hash": "office-sha",
                }
            ],
        },
        "previous": {
            "map_id": "warehouse",
            "version": 2,
            "version_id": "warehouse-lineage:v2",
            "frame_id": "map",
            "artifacts": [
                {
                    "type": "POINTCLOUD",
                    "uri": "/maps/warehouse/map.pcd",
                    "hash": "warehouse-sha",
                }
            ],
        },
    }
    assert maps.active == "office"


def test_runtime_switch_staging_has_explicit_no_previous_semantics() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="")
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    payload = _payload(asyncio.run(_endpoint(gateway)(MapNameRequest(name="office"))))

    assert payload["success"] is True
    assert payload["activation_token"]["previous"] is None
    assert payload["activation_token"]["changed"] is True
    assert maps.active == "office"


def test_runtime_switch_staging_marks_an_already_active_target_unchanged() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="office")
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    payload = _payload(asyncio.run(_endpoint(gateway)(MapNameRequest(name="office"))))

    assert payload["success"] is True
    assert payload["activation_token"]["changed"] is False
    assert payload["activation_token"]["target"] == payload["activation_token"]["previous"]
    assert maps.commands == [{"action": "get_active"}]


def test_runtime_switch_staging_requires_previous_identity_before_mutation() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = MissingPreviousIdentityMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(_endpoint(gateway)(MapNameRequest(name="office")))
    payload = _payload(response)

    assert response.status_code == 503
    assert payload["reason_code"] == "map_identity_unavailable"
    assert payload["transaction"]["state"] == "rejected"
    assert maps.active == "warehouse"
    assert maps.commands == [{"action": "get_active"}]


def test_runtime_switch_staging_restores_previous_after_identity_projection_failure() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = MissingStagedIdentityMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(_endpoint(gateway)(MapNameRequest(name="office")))
    payload = _payload(response)

    assert response.status_code == 503
    assert payload["reason_code"] == "map_identity_unavailable"
    assert payload["transaction"]["state"] == "rolled_back"
    assert payload["transaction"]["rollback"]["verified"] is True
    assert maps.active == "warehouse"
    assert maps.commands == [
        {"action": "get_active"},
        {"action": "set_active", "name": "office"},
        {"action": "set_active", "name": "warehouse"},
        {"action": "get_active"},
    ]


def test_runtime_switch_staging_clears_target_after_no_previous_identity_failure() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = MissingStagedIdentityMaps(active="")
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(_endpoint(gateway)(MapNameRequest(name="office")))
    payload = _payload(response)

    assert response.status_code == 503
    assert payload["transaction"]["state"] == "rolled_back"
    assert payload["transaction"]["previous_active"] is None
    assert payload["transaction"]["rollback"]["verified"] is True
    assert maps.active == ""
    assert {"action": "clear_active"} in maps.commands


def test_runtime_switch_staging_reports_failed_internal_identity_rollback() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    maps = MissingStagedIdentityMaps(fail_restore=True)
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(_endpoint(gateway)(MapNameRequest(name="office")))
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["reason_code"] == "map_stage_rollback_failed"
    assert payload["transaction"]["state"] == "rollback_failed"
    assert payload["transaction"]["runtime_consistent"] is False
    assert payload["transaction"]["rollback"]["verified"] is False
    assert maps.active == "office"


def test_runtime_switch_restore_restores_previous_map_and_verifies_identity() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="office")
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/restore-staged-runtime-switch")(
            _restore_request(previous=_identity("warehouse", 2))
        )
    )
    payload = _payload(response)

    assert payload["success"] is True
    assert payload["active"] == "warehouse"
    assert payload["restored_identity"]["map_id"] == "warehouse"
    assert payload["transaction"]["operation"] == "restore_staged_map_for_runtime_switch"
    assert payload["transaction"]["state"] == "rolled_back"
    assert payload["transaction"]["target_identity"] == _identity("office", 7)
    assert payload["transaction"]["previous_identity"] == _identity("warehouse", 2)
    assert payload["transaction"]["verified"] is True
    assert maps.active == "warehouse"
    assert maps.commands == [
        {"action": "get_active"},
        {"action": "get_record", "name": "warehouse"},
        {"action": "set_active", "name": "warehouse"},
        {"action": "get_active"},
    ]


def test_runtime_switch_restore_clears_active_slot_when_no_previous_map() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="office")
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/restore-staged-runtime-switch")(
            _restore_request(previous=None)
        )
    )
    payload = _payload(response)

    assert payload["success"] is True
    assert payload["active"] == ""
    assert payload.get("restored_identity") is None
    assert payload["transaction"]["previous_active"] is None
    assert {"action": "clear_active"} in maps.commands
    assert maps.active == ""


def test_runtime_switch_restore_rejects_exact_target_drift_without_mutation() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="office")
    maps.versions["office"] = 8
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/restore-staged-runtime-switch")(
            _restore_request(previous=_identity("warehouse", 2))
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["reason_code"] == "staged_map_identity_changed"
    assert payload["transaction"]["state"] == "rejected"
    assert payload["pointer_mutated"] is False
    assert maps.active == "office"
    assert maps.commands == [{"action": "get_active"}]


@pytest.mark.parametrize("drift", ["version", "hash"])
def test_runtime_switch_restore_rejects_previous_identity_drift_without_mutation(drift: str) -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="office")
    if drift == "version":
        maps.versions["warehouse"] = 3
    else:
        maps.hashes["warehouse"] = "warehouse-new-sha"
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/restore-staged-runtime-switch")(
            _restore_request(previous=_identity("warehouse", 2))
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["reason_code"] == "previous_map_identity_changed"
    assert payload["transaction"]["state"] == "rejected"
    assert payload["exact_version_restore_available"] is False
    assert payload["pointer_mutated"] is False
    assert maps.active == "office"
    assert maps.commands == [
        {"action": "get_active"},
        {"action": "get_record", "name": "warehouse"},
    ]


def test_runtime_switch_restore_changed_false_is_verified_without_mutation() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    maps = FakeMaps(active="office")
    gateway._map_mgr = maps
    gateway._session_mode = "idle"
    identity = _identity("office", 7)

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/restore-staged-runtime-switch")(
            _restore_request(target=identity, previous=identity, changed=False)
        )
    )
    payload = _payload(response)

    assert payload["success"] is True
    assert payload["unchanged"] is True
    assert payload["transaction"]["state"] == "rolled_back"
    assert maps.active == "office"
    assert maps.commands == [
        {"action": "get_active"},
        {"action": "get_record", "name": "office"},
        {"action": "get_active"},
    ]


def test_runtime_switch_restore_rejects_legacy_string_only_token() -> None:
    from gateway.schemas import MapActivationRestoreRequest

    with pytest.raises(ValidationError):
        MapActivationRestoreRequest(target_map="office", previous_active="warehouse")


def test_runtime_switch_restore_rejects_unverified_active_identity() -> None:
    from gateway.gateway_module import GatewayModule

    class DriftingMaps(FakeMaps):
        def __init__(self) -> None:
            super().__init__(active="office")
            self._drift_on_query = False

        def execute(self, request):
            command = request.to_mapping()
            if command["action"] == "get_active" and self._drift_on_query:
                self.commands.append(command)
                return {"success": True, **self._identity("office")}
            response = super().execute(request)
            if command["action"] == "set_active":
                self._drift_on_query = True
            return response

    gateway = GatewayModule()
    gateway.setup()
    maps = DriftingMaps()
    gateway._map_mgr = maps
    gateway._session_mode = "idle"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/restore-staged-runtime-switch")(
            _restore_request(previous=_identity("warehouse", 2))
        )
    )
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["reason_code"] == "map_restore_verification_failed"
    assert payload["transaction"]["state"] == "rollback_failed"
