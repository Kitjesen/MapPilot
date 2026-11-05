"""Forward the same bounded point-cloud preview frames used by the Web viewer."""

import grpc

from .generated import observations_pb2 as pb

MAX_FRAME_BYTES = 10 * 1024 * 1024


def watch(gateway, request, context, slots):
    methods = {pb.CLOUD_KIND_MAP: "cloud_subscribe_sync", pb.CLOUD_KIND_SCAN: "scan_subscribe_sync"}
    method = methods.get(request.kind)
    if method is None:
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Point-cloud kind must be MAP or SCAN")
    viewer = getattr(gateway, "_cloud_viewer", None)
    if viewer is None:
        context.abort(grpc.StatusCode.UNAVAILABLE, "Point-cloud viewer source is not configured")
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
    subscription = None
    try:
        subscription = getattr(viewer, method)()
        if not context.add_callback(subscription.close):
            return
        context.send_initial_metadata(())
        while context.is_active():
            frame = subscription.read(timeout=0.5)
            if frame is None:
                continue
            if len(frame) > MAX_FRAME_BYTES:
                context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "Point-cloud frame exceeds 10 MiB")
            yield pb.CloudFrame(pcld=frame)
    finally:
        if subscription is not None:
            subscription.close()
        slots.release()
