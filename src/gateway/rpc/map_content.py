"""Read saved point clouds from mapd's validated artifact descriptor."""

import grpc

from gateway.maps.routes import public_map_name_error
from gateway.maps.transport import map_client
from runtime.endpoints.mapd import MapClient, MapClientError

from .codec import _abort
from .generated import maps_pb2 as pb


def artifact_error(context, error):
    missing = {"artifact_not_found", "map_not_found", "missing_artifact", "missing_capability"}
    unavailable = {"connection_closed", "fd_passing_unsupported", "mapd_unavailable"}
    if error.reason_code in missing:
        code = grpc.StatusCode.NOT_FOUND
    elif error.reason_code == "map_write_in_progress":
        code = grpc.StatusCode.FAILED_PRECONDITION
    elif error.reason_code in unavailable:
        code = grpc.StatusCode.UNAVAILABLE
    else:
        code = grpc.StatusCode.INTERNAL
    _abort(context, code, {"reason_code": error.reason_code, "message": str(error)})


def point_cloud(gateway, request, context, slots):
    error = public_map_name_error(request.map_name)
    if error is not None:
        _abort(context, grpc.StatusCode.INVALID_ARGUMENT, {"reason_code": "invalid_map_id", "message": error})
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
    artifact = None
    chunks = None
    try:
        remaining = context.time_remaining()
        timeout = 10.0 if remaining is None else min(10.0, max(0.001, remaining))
        client = map_client(gateway, timeout_s=timeout)
        if isinstance(client, MapClient):
            # The shared query client may have a different request timeout.
            client = MapClient(client.socket_path, timeout_s=timeout)
        artifact = client.open_artifact(request.map_name, "source_pointcloud")
        if not context.is_active():
            return
        metadata = artifact.metadata
        if metadata.get("map_id") != request.map_name:
            _abort(context, grpc.StatusCode.DATA_LOSS, {
                "reason_code": "invalid_map_artifact", "message": "Map artifact identity does not match the request.",
            })
        info = pb.MapContentInfo(map_id=metadata["map_id"], size_bytes=artifact.size_bytes)
        if metadata.get("content_epoch") is not None:
            info.content_epoch = metadata["content_epoch"]
        if metadata.get("frame_id") is not None:
            info.frame_id = metadata["frame_id"]
        chunks = artifact.iter_bytes()
        yield pb.MapContent(info=info)
        size = 0
        for chunk in chunks:
            if not context.is_active():
                return
            size += len(chunk)
            if size > artifact.size_bytes:
                _abort(context, grpc.StatusCode.DATA_LOSS, {
                    "reason_code": "artifact_size_mismatch", "message": "Map artifact size changed while reading.",
                })
            yield pb.MapContent(chunk=chunk)
        if size != artifact.size_bytes:
            _abort(context, grpc.StatusCode.DATA_LOSS, {
                "reason_code": "artifact_size_mismatch", "message": "Map artifact ended before its declared size.",
            })
    except MapClientError as exc:
        artifact_error(context, exc)
    except OSError as exc:
        _abort(context, grpc.StatusCode.UNAVAILABLE, {"reason_code": "map_artifact_unavailable", "message": str(exc)})
    except (TypeError, ValueError, OverflowError) as exc:
        _abort(context, grpc.StatusCode.DATA_LOSS, {"reason_code": "invalid_map_artifact", "message": str(exc)})
    finally:
        if chunks is not None:
            chunks.close()
        if artifact is not None:
            artifact.close()
        slots.release()
