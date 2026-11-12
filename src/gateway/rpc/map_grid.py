"""Read the native stored-NPZ occupancy artifact within its 25M-cell contract."""

import io
import math
import zipfile

import grpc
import numpy as np

from gateway.maps.routes import public_map_name_error
from gateway.maps.transport import map_client
from runtime.endpoints.mapd import MapClient, MapClientError

from .codec import _abort
from .generated import common_pb2 as common_pb
from .generated import maps_pb2 as pb
from .map_content import artifact_error

MAX_CELLS = 25_000_000
MAX_ARTIFACT_BYTES = MAX_CELLS + 64 * 1024
MAX_HEADER_BYTES = 4096


def _array(archive, name, dtype, shape):
    entry = archive.getinfo(name)
    with archive.open(entry) as source:
        version = np.lib.format.read_magic(source)
        if version == (1, 0):
            actual_shape, fortran, actual_dtype = np.lib.format.read_array_header_1_0(source, max_header_size=MAX_HEADER_BYTES)
        elif version in ((2, 0), (3, 0)):
            actual_shape, fortran, actual_dtype = np.lib.format.read_array_header_2_0(source, max_header_size=MAX_HEADER_BYTES)
        else:
            raise ValueError("Unsupported NPY version")
        if fortran or actual_dtype.str != dtype:
            raise ValueError(f"Invalid occupancy array layout: {name}")
        if shape is None:
            if len(actual_shape) != 2 or any(type(value) is not int or value <= 0 for value in actual_shape):
                raise ValueError("Occupancy grid must have two positive dimensions")
            if math.prod(actual_shape) > MAX_CELLS:
                raise ValueError("Occupancy grid exceeds the native 25M-cell limit")
        elif actual_shape != shape:
            raise ValueError(f"Invalid occupancy array dimensions: {name}")
        size = math.prod(actual_shape) * actual_dtype.itemsize
        if entry.file_size - source.tell() != size:
            raise ValueError(f"Occupancy array payload size does not match its dimensions: {name}")
        payload = source.read(size + 1)
        if len(payload) != size:
            raise ValueError(f"Truncated occupancy array: {name}")
        return np.frombuffer(payload, dtype=actual_dtype).reshape(actual_shape)


def decode_grid(buffer, metadata):
    with zipfile.ZipFile(buffer) as archive:
        entries = archive.infolist()
        if len(entries) != 3 or {entry.filename for entry in entries} != {"grid.npy", "resolution.npy", "origin.npy"}:
            raise ValueError("Occupancy NPZ must contain grid, resolution and origin")
        for entry in entries:
            limit = MAX_CELLS + MAX_HEADER_BYTES if entry.filename == "grid.npy" else MAX_HEADER_BYTES + 16
            if entry.compress_type != zipfile.ZIP_STORED or entry.flag_bits & 9:
                raise ValueError("Occupancy NPZ requires native stored entries")
            if entry.file_size != entry.compress_size or entry.file_size > limit:
                raise ValueError("Occupancy NPZ entry exceeds its size limit")
        cells = _array(archive, "grid.npy", "|i1", None)
        resolution = float(_array(archive, "resolution.npy", "<f8", ()))
        origin = _array(archive, "origin.npy", "<f8", (2,))
        if not math.isfinite(resolution) or resolution <= 0 or not np.isfinite(origin).all():
            raise ValueError("Occupancy geometry is invalid")
        if cells.min() < -1 or cells.max() > 100:
            raise ValueError("Occupancy cells must be in [-1, 100]")
        result = pb.MapGrid(
            map_id=metadata["map_id"], frame_id=metadata["frame_id"],
            width=cells.shape[1], height=cells.shape[0], resolution_m=resolution,
            origin=common_pb.Pose(x=float(origin[0]), y=float(origin[1])),
            cells=cells.tobytes(order="C"),
        )
        if metadata.get("content_epoch") is not None:
            result.content_epoch = metadata["content_epoch"]
        return result


def grid(gateway, request, context, slots):
    error = public_map_name_error(request.map_name)
    if error is not None:
        _abort(context, grpc.StatusCode.INVALID_ARGUMENT, {"reason_code": "invalid_map_id", "message": error})
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
    artifact = chunks = None
    try:
        remaining = context.time_remaining()
        timeout = 10.0 if remaining is None else min(10.0, max(0.001, remaining))
        client = map_client(gateway, timeout_s=timeout)
        if isinstance(client, MapClient):
            client = MapClient(client.socket_path, timeout_s=timeout)
        artifact = client.open_artifact(request.map_name, "path_planning_2d")
        metadata = artifact.metadata
        epoch = metadata.get("content_epoch")
        if (
            metadata.get("map_id") != request.map_name
            or not isinstance(metadata.get("frame_id"), str)
            or not 0 < len(metadata["frame_id"]) <= 128
            or (epoch is not None and (type(epoch) is not int or not 0 <= epoch <= 2**64 - 1))
        ):
            raise ValueError("Occupancy artifact identity is invalid")
        if not 0 < artifact.size_bytes <= MAX_ARTIFACT_BYTES:
            _abort(context, grpc.StatusCode.RESOURCE_EXHAUSTED, {
                "reason_code": "map_grid_too_large", "message": "Occupancy artifact exceeds the native grid size limit.",
            })
        chunks = artifact.iter_bytes()
        with io.BytesIO() as buffer:
            size = 0
            for chunk in chunks:
                if not context.is_active():
                    context.abort(grpc.StatusCode.CANCELLED, "Occupancy read cancelled")
                size += len(chunk)
                if size > artifact.size_bytes:
                    raise ValueError("Occupancy artifact size changed while reading")
                buffer.write(chunk)
            if size != artifact.size_bytes:
                raise ValueError("Occupancy artifact ended before its declared size")
            if not context.is_active():
                context.abort(grpc.StatusCode.CANCELLED, "Occupancy read cancelled")
            buffer.seek(0)
            return decode_grid(buffer, metadata)
    except MapClientError as exc:
        artifact_error(context, exc)
    except OSError as exc:
        _abort(context, grpc.StatusCode.UNAVAILABLE, {"reason_code": "map_artifact_unavailable", "message": str(exc)})
    except (TypeError, ValueError, OverflowError, KeyError, zipfile.BadZipFile) as exc:
        _abort(context, grpc.StatusCode.DATA_LOSS, {"reason_code": "invalid_map_grid", "message": str(exc)})
    finally:
        if chunks is not None:
            chunks.close()
        if artifact is not None:
            artifact.close()
        slots.release()
