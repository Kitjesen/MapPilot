"""JPEG access through the existing camera snapshot and relay cache."""

import os
import threading
import time

import grpc

from gateway.media_access import media_access_error
from gateway.routes.camera import snapshot_jpeg
from gateway.services.media_status import CAMERA_STREAM_STALE_MS, build_camera_status

from .codec import _abort, invoke
from .generated import observations_pb2 as pb


def authorize(context):
    token = os.environ.get("LINGTU_MEDIA_VIEWER_TOKEN")
    error = media_access_error(
        required=os.environ.get("LINGTU_MEDIA_REQUIRE_TOKEN") == "1",
        viewer_token=token.encode("utf-8") if token else None,
        authorization=[value.encode("utf-8") for key, value in context.invocation_metadata() if key == "authorization"],
    )
    if error is not None:
        status, reason = error
        code = {401: grpc.StatusCode.UNAUTHENTICATED, 403: grpc.StatusCode.PERMISSION_DENIED,
                503: grpc.StatusCode.UNAVAILABLE}[status]
        _abort(context, code, {"reason_code": reason}, reason)


def frame(gateway, context):
    authorize(context)
    response = snapshot_jpeg(gateway)
    if response.status_code != 200:
        invoke(context, lambda: response)
    return pb.CameraFrame(jpeg=response.body)


def watch(gateway, context, slots):
    authorize(context)
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
    stopped = threading.Event()
    context.add_callback(stopped.set)
    relay = getattr(gateway, "_camera_module", None)
    connect = getattr(relay, "on_camera_client_connect", None)
    registered = False
    try:
        if callable(connect):
            connect()
            registered = True
        last_sequence = None
        last_frame = time.monotonic()
        while context.is_active():
            with gateway._jpeg_lock:
                jpeg = gateway._latest_jpeg
                sequence = gateway._latest_jpeg_seq
            if jpeg and sequence != last_sequence:
                yield pb.CameraFrame(jpeg=jpeg)
                last_sequence = sequence
                last_frame = time.monotonic()
            elif time.monotonic() - last_frame >= CAMERA_STREAM_STALE_MS / 1000:
                context.abort(grpc.StatusCode.UNAVAILABLE, "Camera JPEG source has no new frame")
            elif not jpeg:
                status = build_camera_status(gateway)
                if status["status"] == "error" or (status["status"] == "not_loaded" and not registered):
                    context.abort(grpc.StatusCode.UNAVAILABLE, status["reason"])
            if stopped.wait(0.1):
                return
    finally:
        try:
            if registered:
                relay.on_camera_client_disconnect()
        finally:
            slots.release()
