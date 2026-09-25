"""Read-only aggregate state, scene, and localization routes."""

from __future__ import annotations

import asyncio

from gateway.schemas import LocalizationStatusResponse, SceneGraphResponse, StateResponse
from gateway.services.runtime_status import build_localization_status
from gateway.services.state_snapshot import build_state_snapshot
from gateway.services.telemetry_normalizers import build_scene_graph_response


def register_status_routes(app, gw) -> None:
    @app.get(
        "/api/v1/state",
        summary="Full robot state snapshot",
        response_model=StateResponse,
    )
    async def get_state():
        # The session snapshot can wait for mapd; keep that wait off the WS loop.
        return await asyncio.to_thread(build_state_snapshot, gw)

    @app.get(
        "/api/v1/scene_graph",
        summary="Current scene graph",
        response_model=SceneGraphResponse,
    )
    async def get_scene_graph():
        with gw._state_lock:
            sg = gw._sg_json
        return build_scene_graph_response(sg)

    @app.get(
        "/api/v1/localization/status",
        summary="Localization status for app and web clients",
        response_model=LocalizationStatusResponse,
    )
    async def get_localization_status():
        return build_localization_status(gw)
