"""Map owner results to typed SDK replies at the transport boundary."""

from __future__ import annotations

import asyncio
import inspect
import json
import logging
from typing import Any

import grpc
from fastapi.encoders import jsonable_encoder
from google.protobuf.json_format import ParseDict
from google.protobuf.struct_pb2 import Struct
from pydantic import ValidationError
from starlette.exceptions import HTTPException
from starlette.responses import Response

from .generated import common_pb2 as common_pb
from .generated import error_pb2 as error_pb
from .generated import maps_pb2 as maps_pb
from .generated import navigation_pb2 as navigation_pb

logger = logging.getLogger(__name__)

_HTTP_STATUS = {
    400: grpc.StatusCode.INVALID_ARGUMENT,
    401: grpc.StatusCode.UNAUTHENTICATED,
    403: grpc.StatusCode.PERMISSION_DENIED,
    404: grpc.StatusCode.NOT_FOUND,
    409: grpc.StatusCode.FAILED_PRECONDITION,
    422: grpc.StatusCode.INVALID_ARGUMENT,
    429: grpc.StatusCode.RESOURCE_EXHAUSTED,
    503: grpc.StatusCode.UNAVAILABLE,
    504: grpc.StatusCode.DEADLINE_EXCEEDED,
}


def fields(value: dict[str, Any]) -> Struct:
    return ParseDict(jsonable_encoder(value), Struct())


def reason_code(*values) -> str:
    for value in values:
        if not isinstance(value, dict):
            continue
        for item in (value, value.get("detail")):
            if isinstance(item, dict) and isinstance(item.get("reason_code"), str):
                if item["reason_code"]:
                    return item["reason_code"]
    return ""


def _abort(context, code, value, message=None):
    payload = value if isinstance(value, dict) else {"detail": value}
    detail = payload.get("detail")
    if message is None:
        message = payload.get("message") or payload.get("error")
        if not message and isinstance(detail, dict):
            message = detail.get("message") or detail.get("reason") or detail.get("error")
        message = message or detail or "request failed"
    error = error_pb.ErrorDetail(reason_code=reason_code(payload), details=fields(payload))
    context.set_trailing_metadata((("doso-error-bin", error.SerializeToString()),))
    context.abort(code, str(message))


def invoke(context, function, *args, **kwargs):
    try:
        value = function(*args, **kwargs)
        if inspect.isawaitable(value):
            value = asyncio.run(value)
    except HTTPException as exc:
        _abort(context, _HTTP_STATUS.get(exc.status_code, grpc.StatusCode.INTERNAL), {"detail": exc.detail})
    except (ValidationError, ValueError) as exc:
        _abort(context, grpc.StatusCode.INVALID_ARGUMENT, {"message": str(exc)})
    except Exception:
        logger.exception("SDK operation failed")
        _abort(context, grpc.StatusCode.INTERNAL, {"reason_code": "internal_error"}, "Internal service error")
    if isinstance(value, Response):
        status = value.status_code
        value = json.loads(value.body)
        if status >= 400:
            _abort(context, _HTTP_STATUS.get(status, grpc.StatusCode.INTERNAL), value)
    return value


def model(context, cls, **values):
    try:
        return cls.model_validate(values)
    except ValidationError as exc:
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))


def command(context, value) -> dict[str, str]:
    if not value.request_id.strip() or not value.client_id.strip():
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "request_id and client_id are required")
    return {"request_id": value.request_id, "client_id": value.client_id}


def pose(value) -> dict[str, float]:
    return {name: getattr(value, name) for name in ("x", "y", "z", "yaw")}


def receipt(context, value, request_id):
    nested = value.get("command") or {}
    result = common_pb.Receipt(
        request_id=nested.get("request_id") or value.get("request_id") or request_id,
        reason=str(value.get("message") or value.get("reason") or value.get("error") or ""),
        stage=str(value.get("stage") or value.get("status") or ""),
        details=fields(value),
        reason_code=reason_code(nested, value),
    )
    admissions = [item["accepted"] for item in (value, nested) if isinstance(item.get("accepted"), bool)]
    if admissions:
        if any(accepted != admissions[0] for accepted in admissions):
            _abort(context, grpc.StatusCode.INTERNAL, value, "contradictory command admission")
        result.accepted = admissions[0]
        result.ok = admissions[0]
    elif isinstance(value.get("ok"), bool):
        result.ok = value["ok"]
    else:
        result.ok = value.get("success") is True
    for key in ("task_id", "operation_id", "run_id"):
        identity = nested.get(key) or value.get(key)
        if identity:
            setattr(result, key, str(identity))
    return result


def task(value):
    status = value.get("status") or {}
    state = status.get("lifecycle_state_name") or status.get("state_name") or "UNKNOWN"
    return navigation_pb.Task(
        task_id=str(value.get("task_id") or ""),
        found=value.get("found") is True,
        state=navigation_pb.TaskState.Value("TASK_" + state),
        reason=str(status.get("failure_reason") or status.get("reason") or value.get("reason") or ""),
        details=fields(value),
        reason_code=reason_code(status, value),
    )


def map_operation(value):
    operation = value.get("operation") or {}
    state = operation.get("state") or "UNKNOWN"
    if state == "CANCELED":
        state = "CANCELLED"
    return maps_pb.MapOperation(
        operation_id=str(operation.get("operation_id") or value.get("operation_id") or ""),
        state=maps_pb.MapState.Value("MAP_" + state),
        reason=str(operation.get("message") or operation.get("reason") or operation.get("reason_code") or ""),
        details=fields(value),
        reason_code=reason_code(operation, value),
    )


def product_operation(value):
    state = str(value.get("state") or "unknown").upper()
    result = value.get("result") or {}
    enum = maps_pb.PRODUCT_STATE_UNKNOWN if state == "UNKNOWN" else maps_pb.ProductState.Value("PRODUCT_" + state)
    return maps_pb.ProductOperation(
        request_id=str(value.get("request_id") or ""),
        state=enum,
        active=result.get("ok") is True and result.get("status") in {"active", "already_active"},
        reason=str(value.get("message") or ""),
        details=fields(value),
        reason_code=reason_code(result, value),
    )
