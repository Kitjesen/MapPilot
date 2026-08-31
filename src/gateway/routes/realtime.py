"""WebSocket route registration for GatewayModule."""

import asyncio
import json
import logging
import math
import time

from starlette.websockets import WebSocket as StarletteWebSocket
from starlette.websockets import WebSocketDisconnect as StarletteWebSocketDisconnect

from gateway.services.safety_status import safety_stop_active
from gateway.services.teleop import NativeTeleopSession, TeleopSessionResult

logger = logging.getLogger(__name__)

REALTIME_SEND_TIMEOUT_S = 2.0


def _public_control_rejection(result: TeleopSessionResult, *, request_id: str) -> dict[str, object]:
    if result.reason in {"authority_busy", "not_active_source", "connection_in_use"}:
        error = "control_in_use"
        message = "Another controller currently owns robot motion."
    else:
        error = "control_unavailable"
        message = "Robot motion control is temporarily unavailable."
    logger.warning("Web teleop rejected internally: %s", result.reason)
    return {
        "type": "control_rejected",
        "error": error,
        "message": message,
        "request_id": request_id,
    }

def register_realtime_routes(app, gw) -> None:
    async def send_bytes_or_disconnect(
        ws: StarletteWebSocket,
        payload: bytes,
        *,
        label: str,
    ) -> bool:
        try:
            await asyncio.wait_for(
                ws.send_bytes(payload),
                timeout=REALTIME_SEND_TIMEOUT_S,
            )
            return True
        except Exception as e:
            logger.debug("%s send failed: %s", label, e)
            return False

    async def send_camera_frames(ws: StarletteWebSocket, *, label: str) -> None:
        last_seq: int | None = None
        while True:
            with gw._jpeg_lock:
                frame = gw._latest_jpeg
                seq = getattr(gw, "_latest_jpeg_seq", 0)
            if frame and seq != last_seq:
                if not await send_bytes_or_disconnect(ws, frame, label=label):
                    break
                last_seq = seq
            await asyncio.sleep(0.1)

    async def ws_teleop_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        client_id = str(ws.query_params.get("client_id") or f"teleop-ws-{id(ws)}")
        conn_id = f"teleop-{id(ws)}"
        session = NativeTeleopSession(gw, f"web:{id(ws)}")
        client_ip = None
        try:
            client_ip = ws.client.host if ws.client else None
        except Exception:
            pass
        registry = getattr(gw, "_ws_registry", None)
        if registry is not None:
            registry.register(conn_id, "/ws/teleop", client_id=client_id, client_ip=client_ip)
        opened = session.open()
        if not opened.accepted:
            await ws.send_text(
                json.dumps(
                    {
                        "type": "control_rejected",
                        "error": "control_in_use",
                        "message": "Another operator is connected.",
                    }
                )
            )
            await ws.close(code=4409)
            if registry is not None:
                registry.unregister(conn_id)
            return
        client_count = gw._teleop_client_connected()
        media_lifecycle = getattr(gw, "_camera_module", None)
        if media_lifecycle is not None:
            media_lifecycle.on_client_connect()
        logger.info("Teleop WS connected (%d clients)", client_count)
        try:
            while True:
                msg = await ws.receive()
                if msg["type"] == "websocket.disconnect":
                    break
                raw = msg.get("text")
                if raw is None:
                    raw_bytes = msg.get("bytes")
                    if not raw_bytes:
                        continue
                    try:
                        raw = raw_bytes.decode()
                    except (AttributeError, UnicodeDecodeError):
                        continue
                if not raw:
                    continue
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if not isinstance(data, dict):
                    continue
                msg_type = data.get("type", "")
                if msg_type == "velocity":
                    request_id = str(data.get("request_id") or "") or (
                        f"web-velocity-{time.monotonic_ns()}"
                    )
                    if data.get("deadman") is not True:
                        held = await asyncio.to_thread(
                            session.hold,
                            request_id=request_id,
                        )
                        if held.accepted:
                            await ws.send_text(
                                json.dumps(
                                    {
                                        "type": "control_ack",
                                        "action": "hold",
                                        "accepted": True,
                                        "request_id": request_id,
                                        "stage": (
                                            "final_zero_published"
                                            if held.final_output_confirmed
                                            else "no_active_command"
                                        ),
                                        "final_cmd_vel_confirmed": held.final_output_confirmed,
                                        "motor_confirmed": False,
                                    }
                                )
                            )
                        else:
                            logger.error("Web teleop hold failed internally: %s", held.reason)
                            await ws.send_text(
                                json.dumps(
                                    {
                                        "type": "control_rejected",
                                        "error": "hold_unconfirmed",
                                        "message": "Robot did not confirm the hold command.",
                                        "request_id": request_id,
                                        "final_cmd_vel_confirmed": False,
                                        "motor_confirmed": False,
                                    }
                                )
                            )
                        continue
                    raw_manual_mode = data.get("manual_mode", False)
                    if not isinstance(raw_manual_mode, bool):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "invalid_manual_mode",
                                    "message": "manual_mode must be a boolean.",
                                    "request_id": request_id,
                                }
                            )
                        )
                        continue
                    try:
                        vx_mps = float(data.get("vx_mps", 0))
                        vy_mps = float(data.get("vy_mps", 0))
                        yaw_rps = float(data.get("yaw_rps", 0))
                    except (TypeError, ValueError):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "invalid_velocity",
                                    "message": "Velocity fields must be numeric m/s and rad/s values.",
                                    "request_id": request_id,
                                }
                            )
                        )
                        continue
                    if not all(math.isfinite(value) for value in (vx_mps, vy_mps, yaw_rps)):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "invalid_velocity",
                                    "message": "Velocity fields must be finite.",
                                    "request_id": request_id,
                                }
                            )
                        )
                        continue
                    with gw._state_lock:
                        safety = getattr(gw, "_navigation_state", None)
                    if safety_stop_active(safety):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "safety_stop",
                                    "message": "Safety STOP is active.",
                                    "request_id": request_id,
                                }
                            )
                        )
                        continue
                    submitted = await asyncio.to_thread(
                        session.move,
                        vx_mps,
                        vy_mps,
                        yaw_rps,
                        request_id=request_id,
                        manual_mode=raw_manual_mode,
                    )
                    if not submitted.accepted:
                        await ws.send_text(
                            json.dumps(_public_control_rejection(submitted, request_id=request_id))
                        )
                    else:
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "ingress_ack",
                                    "action": "queued",
                                    "ingress_accepted": True,
                                    "stage": "gateway_queue_accepted",
                                    "request_id": request_id,
                                    "replaceable": True,
                                    "final_cmd_vel_confirmed": False,
                                    "motor_confirmed": False,
                                }
                            )
                        )
                else:
                    await ws.send_text(
                        json.dumps(
                            {
                                "type": "control_rejected",
                                "error": "unsupported_message",
                                "message": "Web teleop accepts only velocity or hold input.",
                                **(
                                    {"request_id": str(data["request_id"])}
                                    if data.get("request_id")
                                    else {}
                                ),
                            }
                        )
                    )
        except StarletteWebSocketDisconnect:
            pass
        finally:
            if registry is not None:
                registry.unregister(conn_id)
            client_count = gw._teleop_client_disconnected()
            try:
                disconnected = session.disconnect(request_id=f"web-disconnect-{time.monotonic_ns()}")
            except Exception as exc:
                logger.error("Teleop WS disconnect zero release failed: %s", exc)
                disconnected = TeleopSessionResult(False, "disconnect_failed")
            if not disconnected.accepted and hasattr(gw, "push_event"):
                gw.push_event(
                    {
                        "type": "control_rejected",
                        "data": {
                            "error": "disconnect_unconfirmed",
                            "message": "Robot did not confirm the disconnect hold.",
                            "client_id": client_id,
                        },
                    }
                )
            if media_lifecycle is not None:
                media_lifecycle.on_client_disconnect()
            logger.info("Teleop WS disconnected (%d clients)", client_count)

    async def ws_camera_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        conn_id = f"camera-{id(ws)}"
        client_ip = None
        try:
            client_ip = ws.client.host if ws.client else None
        except Exception:
            pass
        registry = getattr(gw, "_ws_registry", None)
        if registry is not None:
            registry.register(conn_id, "/ws/camera", client_ip=client_ip)
        tm = getattr(gw, "_camera_module", None)
        if tm is not None and hasattr(tm, "on_camera_client_connect"):
            tm.on_camera_client_connect()
        try:
            await send_camera_frames(ws, label="camera ws")
        except StarletteWebSocketDisconnect:
            pass
        finally:
            if tm is not None and hasattr(tm, "on_camera_client_disconnect"):
                tm.on_camera_client_disconnect()
            if registry is not None:
                registry.unregister(conn_id)

    async def ws_cloud_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        conn_id = f"cloud-{id(ws)}"
        client_ip = None
        try:
            client_ip = ws.client.host if ws.client else None
        except Exception:
            pass
        registry = getattr(gw, "_ws_registry", None)
        if registry is not None:
            registry.register(conn_id, "/ws/cloud", client_ip=client_ip)
        q, latest = gw._cloud_viewer.cloud_subscribe()
        try:
            if latest is not None:
                if not await send_bytes_or_disconnect(ws, latest, label="cloud ws"):
                    return
            while True:
                buf = await q.get()
                if not await send_bytes_or_disconnect(ws, buf, label="cloud ws"):
                    break
        except StarletteWebSocketDisconnect:
            pass
        except Exception as e:
            logger.debug("cloud ws send failed: %s", e)
        finally:
            gw._cloud_viewer.cloud_unsubscribe(q)
            if registry is not None:
                registry.unregister(conn_id)

    async def ws_scan_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        conn_id = f"scan-{id(ws)}"
        client_ip = None
        try:
            client_ip = ws.client.host if ws.client else None
        except Exception:
            pass
        registry = getattr(gw, "_ws_registry", None)
        if registry is not None:
            registry.register(conn_id, "/ws/scan", client_ip=client_ip)
        q, latest = gw._cloud_viewer.scan_subscribe()
        try:
            if latest is not None:
                if not await send_bytes_or_disconnect(ws, latest, label="scan ws"):
                    return
            while True:
                buf = await q.get()
                if not await send_bytes_or_disconnect(ws, buf, label="scan ws"):
                    break
        except StarletteWebSocketDisconnect:
            pass
        except Exception as e:
            logger.debug("scan ws send failed: %s", e)
        finally:
            gw._cloud_viewer.scan_unsubscribe(q)
            if registry is not None:
                registry.unregister(conn_id)

    add_ws = getattr(app, "add_websocket_route", None) or app.add_api_websocket_route
    add_ws("/ws/teleop", ws_teleop_endpoint)
    add_ws("/ws/camera", ws_camera_endpoint)
    add_ws("/ws/cloud", ws_cloud_endpoint)
    add_ws("/ws/scan", ws_scan_endpoint)
