"""WebSocket route registration for GatewayModule."""

import asyncio
import json
import logging

from starlette.websockets import WebSocket as StarletteWebSocket
from starlette.websockets import WebSocketDisconnect as StarletteWebSocketDisconnect

from gateway.services.command_boundary import CommandBoundaryError
from gateway.services.native_control import (
    resume_autonomy as native_resume_autonomy,
)
from gateway.services.native_control import (
    stop as native_stop,
)
from gateway.services.safety_status import safety_stop_active
from gateway.services.teleop import (
    quiesce_native_teleop,
    resume_native_teleop,
    twist_from_joy,
)
from runtime.msgs.geometry import Twist

logger = logging.getLogger(__name__)

REALTIME_SEND_TIMEOUT_S = 2.0
TELEOP_LEASE_TTL_S = 1.0


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

    async def release_teleop(
        *,
        session_id: str | None = None,
        disposition: str = "hold",
        reason: str = "operator_release",
    ) -> bool:
        operator_motion = getattr(gw, "_operator_motion", None)
        if operator_motion is not None and session_id:
            receipt = await asyncio.to_thread(
                operator_motion.release,
                session_id,
                disposition=disposition,
                reason=reason,
            )
            return bool(isinstance(receipt, dict) and receipt.get("accepted"))
        if bool(getattr(gw, "_teleop_dds_enabled", False)):
            return bool(await asyncio.to_thread(gw._teleop_release))
        return bool(gw._teleop_release())

    def release_teleop_on_disconnect(
        *,
        session_id: str | None = None,
        disposition: str = "close",
        reason: str = "websocket_disconnect",
    ) -> bool:
        """Run the bounded zero barrier synchronously so teardown cannot cancel it."""

        operator_motion = getattr(gw, "_operator_motion", None)
        if operator_motion is not None and session_id:
            receipt = operator_motion.release(
                session_id,
                disposition=disposition,
                reason=reason,
            )
            return bool(isinstance(receipt, dict) and receipt.get("accepted"))
        return bool(gw._teleop_release())

    async def ws_teleop_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        client_id = str(ws.query_params.get("client_id") or f"teleop-ws-{id(ws)}")
        lease_token = f"ws:{id(ws)}:{client_id}"
        operator_motion = getattr(gw, "_operator_motion", None)
        operator_session_id: str | None = None
        if not gw._lease.acquire(lease_token, TELEOP_LEASE_TTL_S):
            await ws.send_text(
                json.dumps(
                    {
                        "type": "control_rejected",
                        "error": "lease_conflict",
                        "message": "Another operator currently owns teleoperation.",
                    }
                )
            )
            await ws.close(code=4409)
            return
        if operator_motion is not None:
            try:
                opened = await asyncio.to_thread(
                    operator_motion.open,
                    source_id=lease_token,
                    adapter="websocket",
                    lease_ttl_s=TELEOP_LEASE_TTL_S,
                )
            except Exception as exc:
                opened = {
                    "accepted": False,
                    "reason": str(exc) or "operator_motion_unavailable",
                }
            if not isinstance(opened, dict) or opened.get("accepted") is not True:
                gw._lease.release(lease_token)
                reason = str((opened or {}).get("reason") or "operator_motion_unavailable")
                await ws.send_text(
                    json.dumps(
                        {
                            "type": "control_rejected",
                            "error": reason,
                            "message": "Operator motion control is unavailable.",
                        }
                    )
                )
                await ws.close(code=4409 if reason == "lease_conflict" else 1011)
                return
            operator_session_id = str(opened.get("session_id") or "") or None
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

        frame_task = asyncio.create_task(send_camera_frames(ws, label="teleop camera")) if stream_camera else None
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
                    if not gw._lease.renew(lease_token, TELEOP_LEASE_TTL_S):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "lease_lost",
                                    "message": "Teleoperation lease is no longer owned by this client.",
                                }
                            )
                        )
                        continue
                    if operator_motion is not None and operator_session_id:
                        deadman = data.get("deadman") is True
                        if deadman:
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
                            request = twist_from_joy(gw, lx, ly, az)
                        else:
                            request = Twist()
                        try:
                            receipt = await asyncio.to_thread(
                                operator_motion.submit,
                                operator_session_id,
                                request.linear.x,
                                request.linear.y,
                                request.angular.z,
                                deadman=deadman,
                                sequence=data.get("sequence", data.get("seq")),
                                request_id=str(data.get("request_id") or "") or None,
                                ttl_ms=data.get("ttl_ms", 350),
                                source_stamp_ns=data.get("source_stamp_ns"),
                            )
                        except Exception as exc:
                            receipt = {
                                "accepted": False,
                                "reason": str(exc) or "operator_motion_unavailable",
                            }
                        if isinstance(receipt, dict) and receipt.get("accepted") is True:
                            stage = str(receipt.get("stage") or "queued")
                            action = "manual_hold" if stage == "held" else stage
                            await ws.send_text(
                                json.dumps(
                                    {
                                        "type": "control_ack",
                                        "action": action,
                                        "accepted": True,
                                        "stage": stage,
                                        "request_id": receipt.get("request_id"),
                                    }
                                )
                            )
                        else:
                            reason = str((receipt or {}).get("reason") or "native_command_unavailable")
                            error = "manual_hold_unconfirmed" if not deadman else reason
                            await ws.send_text(
                                json.dumps(
                                    {
                                        "type": "control_rejected",
                                        "error": error,
                                        "message": reason,
                                    }
                                )
                            )
                        continue
                    if data.get("deadman") is not True:
                        released = await release_teleop()
                        if released:
                            payload = {
                                "type": "control_ack",
                                "action": "manual_hold",
                                "accepted": True,
                            }
                        else:
                            payload = {
                                "type": "control_rejected",
                                "error": "manual_hold_unconfirmed",
                                "message": "Native endpoint did not acknowledge the zero command.",
                            }
                        await ws.send_text(json.dumps(payload))
                        continue
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
                    if not gw._teleop_on_joy(lx, ly, az):
                        publisher = getattr(gw, "_teleop_native_publisher", None)
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "native_command_unavailable",
                                    "message": getattr(publisher, "last_error", None)
                                    or "Native teleop command was not accepted.",
                                }
                            )
                        )
                elif msg_type == "stop":
                    quiesced = False
                    wrote_native = False
                    stop_request_id = str(data.get("request_id") or "") or None
                    try:

                        def _stop_with_barrier(request_id=stop_request_id):
                            if operator_motion is not None and operator_session_id:
                                receipt = operator_motion.release(
                                    operator_session_id,
                                    disposition="hold",
                                    reason="websocket_stop_barrier",
                                )
                                if not isinstance(receipt, dict) or receipt.get("accepted") is not True:
                                    raise CommandBoundaryError(
                                        str((receipt or {}).get("reason") or "operator motion hold failed")
                                    )
                                held = True
                            else:
                                held = quiesce_native_teleop(gw)
                            wrote = native_stop(
                                gw,
                                "websocket_stop",
                                request_id=request_id,
                            )
                            return held, wrote

                        quiesced, wrote_native = await asyncio.to_thread(_stop_with_barrier)
                    except CommandBoundaryError as exc:
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "native_command_rejected",
                                    "message": str(exc),
                                }
                            )
                        )
                        continue
                    finally:
                        if quiesced and wrote_native and operator_session_id is None:
                            resume_native_teleop(gw)
                    if not wrote_native:
                        gw.stop_cmd.publish(2)
                        if tm is not None:
                            tm.force_release()
                        else:
                            gw.cmd_vel.publish(Twist())
                elif msg_type == "resume_autonomy":
                    if not gw._lease.renew(lease_token, TELEOP_LEASE_TTL_S):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "lease_lost",
                                    "message": "Only the current teleoperation owner may resume autonomy.",
                                }
                            )
                        )
                        continue
                    quiesced = False
                    wrote_native = False
                    resume_request_id = str(data.get("request_id") or "") or None
                    try:

                        def _resume_with_barrier(request_id=resume_request_id):
                            if operator_motion is not None and operator_session_id:
                                receipt = operator_motion.release(
                                    operator_session_id,
                                    disposition="hold",
                                    reason="websocket_resume_barrier",
                                )
                                if not isinstance(receipt, dict) or receipt.get("accepted") is not True:
                                    raise CommandBoundaryError(
                                        str((receipt or {}).get("reason") or "operator motion hold failed")
                                    )
                                held = True
                            else:
                                held = quiesce_native_teleop(gw)
                            wrote = native_resume_autonomy(
                                gw,
                                "websocket_resume",
                                request_id=request_id,
                            )
                            return held, wrote

                        quiesced, wrote_native = await asyncio.to_thread(_resume_with_barrier)
                    except CommandBoundaryError as exc:
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "native_command_rejected",
                                    "message": str(exc),
                                }
                            )
                        )
                        continue
                    finally:
                        if quiesced and wrote_native and operator_session_id is None:
                            resume_native_teleop(gw)
                    await ws.send_text(
                        json.dumps(
                            {
                                "type": "control_ack",
                                "action": "resume_autonomy",
                                "accepted": bool(wrote_native),
                                "goal_reissue_required": True,
                            }
                        )
                    )
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
            released_owner = gw._lease.release(lease_token)
            if operator_session_id:
                try:
                    released = release_teleop_on_disconnect(
                        session_id=operator_session_id,
                        disposition="close",
                        reason="websocket_disconnect",
                    )
                except Exception as exc:
                    logger.error("Teleop WS disconnect zero release failed: %s", exc)
                    released = False
                if not released and hasattr(gw, "push_event"):
                    gw.push_event(
                        {
                            "type": "control_rejected",
                            "data": {
                                "error": "disconnect_zero_unconfirmed",
                                "message": "Native endpoint did not acknowledge disconnect zero.",
                                "client_id": client_id,
                            },
                        }
                    )
            elif bool(getattr(gw, "_teleop_dds_enabled", False)) and (released_owner or client_count == 0):
                try:
                    released = release_teleop_on_disconnect()
                except Exception as exc:
                    logger.error("Teleop WS disconnect zero release failed: %s", exc)
                    released = False
                if not released and hasattr(gw, "push_event"):
                    gw.push_event(
                        {
                            "type": "control_rejected",
                            "data": {
                                "error": "disconnect_zero_unconfirmed",
                                "message": "Native endpoint did not acknowledge disconnect zero.",
                                "client_id": client_id,
                            },
                        }
                    )
            elif tm is None and (released_owner or client_count == 0):
                release_teleop_on_disconnect()
            if tm is not None:
                tm.on_client_disconnect()
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

    async def ws_scan_endpoint(ws: StarletteWebSocket):
        await ws.accept()
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

    add_ws = getattr(app, "add_websocket_route", None) or app.add_api_websocket_route
    add_ws("/ws/teleop", ws_teleop_endpoint)
    add_ws("/ws/camera", ws_camera_endpoint)
    add_ws("/ws/cloud", ws_cloud_endpoint)
    add_ws("/ws/scan", ws_scan_endpoint)
