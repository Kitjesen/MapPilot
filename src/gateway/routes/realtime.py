"""WebSocket route registration for GatewayModule."""

from __future__ import annotations

import asyncio
import json
import logging

from runtime.msgs.geometry import Twist
from gateway.services.safety_status import safety_stop_active

logger = logging.getLogger(__name__)

REALTIME_SEND_TIMEOUT_S = 2.0


def _camera_stream_requested(query_params) -> bool:
    """Return True when a legacy teleop client explicitly asks for JPEG frames."""
    stream = str(query_params.get("stream", "")).lower()
    if stream in {"camera", "video", "jpeg"}:
        return True
    for key in ("video", "camera", "frames"):
        value = str(query_params.get(key, "")).lower()
        if value in {"1", "true", "yes", "on"}:
            return True
    return False


def register_realtime_routes(app, gw) -> None:
    from starlette.websockets import WebSocket as StarletteWebSocket
    from starlette.websockets import WebSocketDisconnect as StarletteWebSocketDisconnect

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
        client_count = gw._teleop_client_connected()
        tm = gw._teleop_module
        if tm is not None:
            tm.on_client_connect()
        stream_camera = _camera_stream_requested(ws.query_params)
        logger.info(
            "Teleop WS connected (%d clients, camera_stream=%s)",
            client_count,
            stream_camera,
        )

        frame_task = (
            asyncio.create_task(send_camera_frames(ws, label="teleop camera"))
            if stream_camera
            else None
        )
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
                if msg_type == "joy":
                    try:
                        lx = float(data.get("lx", 0))
                        ly = float(data.get("ly", 0))
                        az = float(data.get("az", 0))
                    except (TypeError, ValueError):
                        continue
                    with gw._state_lock:
                        safety = getattr(gw, "_safety", None)
                    if safety_stop_active(safety):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "safety_stop",
                                    "message": "Safety STOP is active.",
                                }
                            )
                        )
                        continue
                    gw._teleop_on_joy(lx, ly, az)
                elif msg_type == "stop":
                    gw.stop_cmd.publish(2)
                    if tm is not None:
                        tm.force_release()
                    else:
                        gw.cmd_vel.publish(Twist())
        except StarletteWebSocketDisconnect:
            pass
        finally:
            if frame_task is not None:
                frame_task.cancel()
                try:
                    await frame_task
                except asyncio.CancelledError:
                    pass
                except Exception as e:
                    logger.debug("teleop camera frame task ended with error: %s", e)
            client_count = gw._teleop_client_disconnected()
            if tm is not None:
                tm.on_client_disconnect()
            elif client_count == 0:
                gw._teleop_release()
            logger.info("Teleop WS disconnected (%d clients)", client_count)

    async def ws_camera_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        tm = gw._teleop_module
        if tm is not None and hasattr(tm, "on_camera_client_connect"):
            tm.on_camera_client_connect()
        try:
            await send_camera_frames(ws, label="camera ws")
        except StarletteWebSocketDisconnect:
            pass
        finally:
            if tm is not None and hasattr(tm, "on_camera_client_disconnect"):
                tm.on_camera_client_disconnect()

    async def ws_cloud_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        q, latest = gw._cloud_subscribe()
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
            gw._cloud_unsubscribe(q)

    add_ws = getattr(app, "add_websocket_route", None) or app.add_api_websocket_route
    add_ws("/ws/teleop", ws_teleop_endpoint)
    add_ws("/ws/camera", ws_camera_endpoint)
    add_ws("/ws/cloud", ws_cloud_endpoint)
