"""WebSocket route registration for GatewayModule."""

import asyncio
import json
import logging
import time

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
)
from runtime.msgs.geometry import Twist
from runtime.msgs.nav import OperatorMotionReceipt

logger = logging.getLogger(__name__)

REALTIME_SEND_TIMEOUT_S = 2.0
TELEOP_LEASE_TTL_S = 1.0

def _receipt_payload(receipt):
    if isinstance(receipt, OperatorMotionReceipt):
        return receipt.to_dict()
    return None


def _receipt_final_output_published(receipt) -> bool:
    return isinstance(receipt, OperatorMotionReceipt) and receipt.final_output_published


def _receipt_reason(receipt, fallback: str) -> str:
    if isinstance(receipt, OperatorMotionReceipt) and receipt.reason:
        return receipt.reason
    return fallback


def _teleop_sample_metadata(sequence: int) -> dict[str, int | str | bool]:
    return {
        "sample_accepted": True,
        "sample_ack_expected": False,
        "sample_ack_scope": "operator-motion-sample",
        "sample_sequence": int(sequence),
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

    async def release_teleop(
        *,
        source_id: str = "gateway:teleop",
        source_epoch: int = 1,
        sequence: int = 1,
        release_sequence: int | None = None,
        request_id: str | None = None,
        reason: str = "operator_hold",
    ):
        if bool(getattr(gw, "_teleop_dds_enabled", False)):
            return await asyncio.to_thread(
                gw._teleop_release,
                source_id=source_id,
                source_epoch=source_epoch,
                sequence=sequence,
                release_sequence=release_sequence,
                request_id=request_id,
                reason=reason,
            )
        return gw._teleop_release() is True

    def release_teleop_on_disconnect(
        *,
        source_id: str = "gateway:teleop",
        source_epoch: int = 1,
        sequence: int = 1,
        release_sequence: int | None = None,
        request_id: str | None = None,
    ):
        """Run the bounded zero barrier synchronously so teardown cannot cancel it."""

        return gw._teleop_release(
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
            release_sequence=release_sequence,
            request_id=request_id,
            reason="disconnect",
        )

    async def ws_teleop_endpoint(ws: StarletteWebSocket):
        await ws.accept()
        client_id = str(ws.query_params.get("client_id") or f"teleop-ws-{id(ws)}")
        conn_id = f"teleop-{id(ws)}"
        client_ip = None
        try:
            client_ip = ws.client.host if ws.client else None
        except Exception:
            pass
        registry = getattr(gw, "_ws_registry", None)
        if registry is not None:
            registry.register(conn_id, "/ws/teleop", client_id=client_id, client_ip=client_ip)
        lease_token = f"ws:{id(ws)}:{client_id}"
        adapter_source_id = lease_token
        adapter_source_epoch = time.monotonic_ns()
        operator_sequence = 0

        def next_operator_sequence() -> int:
            nonlocal operator_sequence
            operator_sequence += 1
            return operator_sequence

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
        if bool(getattr(gw, "_teleop_dds_enabled", False)):
            claimed = await asyncio.to_thread(
                gw._teleop_claim,
                source_id=adapter_source_id,
                source_epoch=adapter_source_epoch,
                sequence=next_operator_sequence(),
                lease_ttl_ms=int(TELEOP_LEASE_TTL_S * 1000),
                request_id=f"{adapter_source_id}:claim",
            )
            if not (isinstance(claimed, OperatorMotionReceipt) and claimed.source_accepted):
                gw._lease.release(lease_token)
                payload = {
                    "type": "control_rejected",
                    "error": "operator_motion_claim_failed",
                    "message": _receipt_reason(
                        claimed,
                        "Native endpoint did not accept teleoperation authority.",
                    ),
                }
                native_receipt = _receipt_payload(claimed)
                if native_receipt is not None:
                    payload["native_receipt"] = native_receipt
                await ws.send_text(json.dumps(payload))
                await ws.close(code=4510)
                return
        client_count = gw._teleop_client_connected()
        tm = gw._teleop_module
        media_lifecycle = tm or getattr(gw, "_camera_module", None)
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
                    if data.get("deadman") is not True:
                        hold_request_id = str(data.get("request_id") or "") or None
                        hold_sequence = next_operator_sequence()
                        if bool(getattr(gw, "_teleop_dds_enabled", False)) and hold_request_id is None:
                            hold_request_id = f"{adapter_source_id}:manual_hold:{hold_sequence}"
                        released = await release_teleop(
                            source_id=adapter_source_id,
                            source_epoch=adapter_source_epoch,
                            sequence=hold_sequence,
                            request_id=hold_request_id,
                            reason="manual_hold",
                        )
                        native_teleop_path = bool(getattr(gw, "_teleop_dds_enabled", False))
                        final_confirmed = (
                            _receipt_final_output_published(released)
                            if native_teleop_path
                            else False
                        )
                        hold_accepted = final_confirmed if native_teleop_path else released is True
                        native_receipt = _receipt_payload(released)
                        if hold_accepted:
                            payload = {
                                "type": "control_ack",
                                "action": "manual_hold",
                                "accepted": True,
                                "request_id": hold_request_id,
                                "source_sequence": hold_sequence,
                                "stage": (
                                    "final_zero_published"
                                    if native_teleop_path
                                    else "compatibility_source_zero"
                                ),
                                "final_cmd_vel_confirmed": final_confirmed,
                                "motor_confirmed": False,
                            }
                            if native_receipt is not None:
                                payload["native_receipt"] = native_receipt
                        else:
                            payload = {
                                "type": "control_rejected",
                                "error": "manual_hold_unconfirmed",
                                "message": _receipt_reason(
                                    released,
                                    "Native endpoint did not acknowledge the zero command.",
                                ),
                                "final_cmd_vel_confirmed": False,
                                "motor_confirmed": False,
                            }
                            if native_receipt is not None:
                                payload["native_receipt"] = native_receipt
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
                    sample_sequence = next_operator_sequence()
                    request_id = str(data.get("request_id") or "") or None
                    if request_id is None:
                        request_id = f"{adapter_source_id}:joy:{sample_sequence}"
                    sample_metadata = _teleop_sample_metadata(sample_sequence)
                    submission_accepted = bool(
                        gw._teleop_on_joy(
                            lx,
                            ly,
                            az,
                            request_id=request_id,
                            source_id=adapter_source_id,
                            source_epoch=adapter_source_epoch,
                            sequence=sample_sequence,
                        )
                    )
                    if not submission_accepted:
                        publisher = getattr(gw, "_teleop_native_publisher", None)
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "native_command_unavailable",
                                    "request_id": request_id,
                                    "source_sequence": sample_sequence,
                                    "sample_accepted": submission_accepted,
                                    "sample_ack_expected": sample_metadata["sample_ack_expected"],
                                    "sample_ack_scope": sample_metadata["sample_ack_scope"],
                                    "message": (
                                        getattr(publisher, "last_error", None)
                                        or "Native teleop command was not accepted."
                                    ),
                                }
                            )
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
                                    "source_id": adapter_source_id,
                                    "source_epoch": adapter_source_epoch,
                                    "source_sequence": sample_sequence,
                                    "replaceable": True,
                                    "sample_accepted": submission_accepted,
                                    "sample_ack_expected": sample_metadata["sample_ack_expected"],
                                    "sample_ack_scope": sample_metadata["sample_ack_scope"],
                                    "final_cmd_vel_confirmed": False,
                                    "motor_confirmed": False,
                                }
                            )
                        )
                elif msg_type == "stop":
                    quiesced = False
                    wrote_native = False
                    stop_request_id = str(data.get("request_id") or "") or None
                    stop_sequence = next_operator_sequence()
                    native_stop_path = bool(getattr(gw, "_teleop_dds_enabled", False))
                    if native_stop_path and stop_request_id is None:
                        stop_request_id = f"{adapter_source_id}:stop:{stop_sequence}"
                    final_cmd_vel_confirmed = False
                    native_receipt = None
                    try:

                        def _stop_with_barrier(request_id=stop_request_id):
                            held = quiesce_native_teleop(
                                gw,
                                source_id=adapter_source_id,
                                source_epoch=adapter_source_epoch,
                                sequence=stop_sequence,
                                request_id=request_id,
                                reason="websocket_stop",
                            )
                            wrote = native_stop(
                                gw,
                                "websocket_stop",
                                request_id=request_id,
                            )
                            return held, wrote

                        quiesced, wrote_native = await asyncio.to_thread(_stop_with_barrier)
                        final_cmd_vel_confirmed = (
                            _receipt_final_output_published(quiesced)
                            if native_stop_path
                            else quiesced is True
                        )
                        native_receipt = _receipt_payload(quiesced)
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
                        if final_cmd_vel_confirmed and wrote_native is True:
                            resume_native_teleop(gw)
                    if native_stop_path and (
                        wrote_native is not True or not final_cmd_vel_confirmed
                    ):
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "native_stop_unconfirmed",
                                    "request_id": stop_request_id,
                                    "source_sequence": stop_sequence,
                                    "final_cmd_vel_confirmed": final_cmd_vel_confirmed,
                                    "motor_confirmed": False,
                                    **({"native_receipt": native_receipt} if native_receipt is not None else {}),
                                }
                            )
                        )
                        continue
                    if not native_stop_path:
                        gw.stop_cmd.publish(2)
                        if tm is not None:
                            tm.force_release()
                        else:
                            gw.cmd_vel.publish(Twist())
                    await ws.send_text(
                        json.dumps(
                            {
                                "type": "control_ack",
                                "action": "stop",
                                "accepted": True,
                                "request_id": stop_request_id,
                                "source_sequence": stop_sequence,
                                "stage": (
                                    "native_stop_acknowledged"
                                    if native_stop_path
                                    else "local_zero_requested"
                                ),
                                "native_command_acknowledged": (
                                    native_stop_path and wrote_native is True
                                ),
                                "final_cmd_vel_confirmed": final_cmd_vel_confirmed if native_stop_path else False,
                                "motor_confirmed": False,
                                **({"native_receipt": native_receipt} if native_stop_path and native_receipt is not None else {}),
                            }
                        )
                    )
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
                    resume_sequence = next_operator_sequence()
                    if resume_request_id is None:
                        resume_request_id = f"{adapter_source_id}:resume:{resume_sequence}"
                    final_cmd_vel_confirmed = False
                    native_receipt = None
                    try:

                        def _resume_with_barrier(request_id=resume_request_id):
                            held = quiesce_native_teleop(
                                gw,
                                source_id=adapter_source_id,
                                source_epoch=adapter_source_epoch,
                                sequence=resume_sequence,
                                request_id=request_id,
                                reason="websocket_resume",
                            )
                            wrote = native_resume_autonomy(
                                gw,
                                "websocket_resume",
                                request_id=request_id,
                            )
                            return held, wrote

                        quiesced, wrote_native = await asyncio.to_thread(_resume_with_barrier)
                        final_cmd_vel_confirmed = _receipt_final_output_published(quiesced)
                        native_receipt = _receipt_payload(quiesced)
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
                        if final_cmd_vel_confirmed and wrote_native is True:
                            resume_native_teleop(gw)
                    if not final_cmd_vel_confirmed or wrote_native is not True:
                        await ws.send_text(
                            json.dumps(
                                {
                                    "type": "control_rejected",
                                    "error": "resume_autonomy_unconfirmed",
                                    "request_id": resume_request_id,
                                    "final_cmd_vel_confirmed": final_cmd_vel_confirmed,
                                    "motor_confirmed": False,
                                    **({"native_receipt": native_receipt} if native_receipt is not None else {}),
                                }
                            )
                        )
                        continue
                    await ws.send_text(
                        json.dumps(
                            {
                                "type": "control_ack",
                                "action": "resume_autonomy",
                                "accepted": True,
                                "goal_reissue_required": True,
                                "request_id": resume_request_id,
                                "final_cmd_vel_confirmed": final_cmd_vel_confirmed,
                                "motor_confirmed": False,
                                **({"native_receipt": native_receipt} if native_receipt is not None else {}),
                            }
                        )
                    )
        except StarletteWebSocketDisconnect:
            pass
        finally:
            if registry is not None:
                registry.unregister(conn_id)
            client_count = gw._teleop_client_disconnected()
            released_owner = gw._lease.release(lease_token)
            if bool(getattr(gw, "_teleop_dds_enabled", False)) and (released_owner or client_count == 0):
                try:
                    released = release_teleop_on_disconnect(
                        source_id=adapter_source_id,
                        source_epoch=adapter_source_epoch,
                        sequence=next_operator_sequence(),
                        release_sequence=next_operator_sequence(),
                        request_id=f"{adapter_source_id}:disconnect",
                    )
                except Exception as exc:
                    logger.error("Teleop WS disconnect zero release failed: %s", exc)
                    released = False
                if not _receipt_final_output_published(released) and hasattr(gw, "push_event"):
                    gw.push_event(
                        {
                            "type": "control_rejected",
                            "data": {
                                "error": "disconnect_zero_unconfirmed",
                                "message": "Native endpoint did not acknowledge disconnect zero.",
                                "client_id": client_id,
                                **({"native_receipt": _receipt_payload(released)} if _receipt_payload(released) is not None else {}),
                            },
                        }
                    )
            elif tm is None and (released_owner or client_count == 0):
                release_teleop_on_disconnect(
                    source_id=adapter_source_id,
                    source_epoch=adapter_source_epoch,
                    sequence=next_operator_sequence(),
                    release_sequence=next_operator_sequence(),
                    request_id=f"{adapter_source_id}:disconnect",
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
        tm = getattr(gw, "_camera_module", None) or gw._teleop_module
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
