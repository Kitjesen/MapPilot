"""HTTP access to the same body-command owner used by public gRPC."""

import asyncio

from gateway.schemas import BodyCommandResponse, ControlCommandResponse, StopRequest, SwitchProfileRequest
from gateway.services.body import change
from gateway.services.stop import stop_motion


def register_body_routes(app, gw) -> None:
    @app.post("/api/v1/motion/stand", response_model=BodyCommandResponse)
    async def stand(body: StopRequest):
        return await asyncio.to_thread(change, gw, body, "stand")

    @app.post("/api/v1/motion/sit", response_model=BodyCommandResponse)
    async def sit(body: StopRequest):
        return await asyncio.to_thread(change, gw, body, "sit")

    @app.post("/api/v1/motion/motors/enable", response_model=BodyCommandResponse)
    async def enable_motors(body: StopRequest):
        return await asyncio.to_thread(change, gw, body, "enable_motors")

    @app.post("/api/v1/motion/motors/disable", response_model=BodyCommandResponse)
    async def disable_motors(body: StopRequest):
        return await asyncio.to_thread(change, gw, body, "disable_motors")

    @app.post("/api/v1/motion/profile", response_model=BodyCommandResponse)
    async def switch_profile(body: SwitchProfileRequest):
        return await asyncio.to_thread(change, gw, body, "switch_profile")

    @app.post("/api/v1/motion/stop", response_model=ControlCommandResponse)
    async def stop(body: StopRequest):
        return await asyncio.to_thread(stop_motion, gw, body)
