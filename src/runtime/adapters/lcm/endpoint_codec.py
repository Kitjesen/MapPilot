"""Schema-aware JSON payloads for LCM endpoint adapters."""

from __future__ import annotations

import base64
import json
from collections.abc import Mapping
from typing import Any

from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.transport.json_codec import FORMAT as LCM_PAYLOAD_FORMAT
from runtime.transport.json_codec import SCHEMA_VERSION
from runtime.transport.json_codec import loads_message
from runtime.transport.json_codec import message_frame_id, message_timestamp

from .contracts import LCMEndpointBinding

_BINARY_ENCODERS = {
    "lingtu.sensor.point_cloud2.v1": PointCloud2,
    "lingtu.sensor.imu.v1": Imu,
}

_DICT_ENCODERS = {
    "lingtu.nav.odometry.v1": Odometry,
    "lingtu.nav.path.v1": Path,
    "lingtu.geometry.pose_stamped.v1": PoseStamped,
    "lingtu.geometry.twist.v1": Twist,
}

_TEXT_SCHEMAS = {
    "lingtu.control.cancel.v1",
    "lingtu.control.instruction.v1",
}


def dumps_endpoint_message(binding: LCMEndpointBinding, msg: Any) -> bytes:
    """Encode one contract-bound message into a JSON endpoint envelope."""

    envelope = {
        "format": LCM_PAYLOAD_FORMAT,
        "schema_version": SCHEMA_VERSION,
        "topic": binding.topic,
        "schema": binding.schema,
        "frame_id": message_frame_id(msg, topic=binding.topic),
        "ts": message_timestamp(msg),
        "payload": _encode_payload(binding.schema, msg),
    }
    return json.dumps(
        envelope,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def loads_endpoint_message(binding: LCMEndpointBinding, data: Any) -> Any:
    """Decode one contract-bound endpoint message.

    The adapter also accepts the generic ``runtime.transport.json_codec`` envelope
    so older same-Python LCM peers can interoperate during migration.
    """

    if not isinstance(data, (bytes, bytearray, str)):
        return data
    raw = data.decode("utf-8") if isinstance(data, (bytes, bytearray)) else data
    envelope = json.loads(raw)
    if not isinstance(envelope, dict):
        raise ValueError("LCM endpoint payload must be a JSON object")
    if envelope.get("format") != LCM_PAYLOAD_FORMAT:
        raise ValueError("unsupported LingTu LCM endpoint payload")
    if "schema" not in envelope:
        return loads_message(data)

    _validate_endpoint_envelope(binding, envelope)
    return _decode_payload(binding.schema, envelope.get("payload"))


def _validate_endpoint_envelope(binding: LCMEndpointBinding, envelope: Mapping[str, Any]) -> None:
    try:
        version = int(envelope.get("schema_version"))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"LCM endpoint schema_version missing for {binding.topic}") from exc
    if version != SCHEMA_VERSION:
        raise ValueError(
            f"LCM endpoint schema_version mismatch for {binding.topic}: "
            f"expected {SCHEMA_VERSION}, got {version}"
        )

    schema = str(envelope.get("schema") or "")
    if schema != binding.schema:
        raise ValueError(
            f"LCM endpoint schema mismatch for {binding.topic}: "
            f"expected {binding.schema}, got {schema or '<missing>'}"
        )
    topic = str(envelope.get("topic") or "")
    if topic and topic != binding.topic:
        raise ValueError(
            f"LCM endpoint topic mismatch: expected {binding.topic}, got {topic}"
        )

    frame_id = str(envelope.get("frame_id") or "")
    if binding.frame_ids:
        if not frame_id:
            raise ValueError(f"LCM endpoint frame_id missing for {binding.topic}")
        if frame_id not in binding.frame_ids:
            allowed = ", ".join(binding.frame_ids)
            raise ValueError(
                f"LCM endpoint frame_id mismatch for {binding.topic}: "
                f"{frame_id} not in {allowed}"
            )

    try:
        float(envelope["ts"])
    except KeyError as exc:
        raise ValueError(f"LCM endpoint timestamp missing for {binding.topic}") from exc
    except (TypeError, ValueError) as exc:
        raise ValueError(f"LCM endpoint timestamp invalid for {binding.topic}") from exc


def _encode_payload(schema: str, msg: Any) -> Any:
    binary_type = _BINARY_ENCODERS.get(schema)
    if binary_type is not None:
        if not isinstance(msg, binary_type):
            raise TypeError(f"{schema} expects {binary_type.__name__}, got {type(msg).__name__}")
        return {
            "encoding": f"{binary_type.__module__}.{binary_type.__name__}.binary.v1",
            "data": base64.b64encode(msg.encode()).decode("ascii"),
        }

    dict_type = _DICT_ENCODERS.get(schema)
    if dict_type is not None:
        if not isinstance(msg, dict_type):
            raise TypeError(f"{schema} expects {dict_type.__name__}, got {type(msg).__name__}")
        to_dict = getattr(msg, "to_dict", None)
        if not callable(to_dict):
            raise TypeError(f"{type(msg).__name__} does not support to_dict()")
        return to_dict()

    if schema == "lingtu.status.localization_quality.v1":
        return float(msg)
    if schema == "lingtu.status.localization_health.v1":
        if not isinstance(msg, Mapping):
            raise TypeError("localization health payload must be a mapping")
        return dict(msg)
    if schema in _TEXT_SCHEMAS:
        return str(msg)
    return msg


def _decode_payload(schema: str, payload: Any) -> Any:
    binary_type = _BINARY_ENCODERS.get(schema)
    if binary_type is not None:
        if not isinstance(payload, Mapping):
            raise ValueError(f"{schema} payload must be a mapping")
        data = base64.b64decode(str(payload.get("data", "")))
        return binary_type.decode(data)

    dict_type = _DICT_ENCODERS.get(schema)
    if dict_type is not None:
        if not isinstance(payload, Mapping):
            raise ValueError(f"{schema} payload must be a mapping")
        from_dict = getattr(dict_type, "from_dict", None)
        if not callable(from_dict):
            raise TypeError(f"{dict_type.__name__} does not support from_dict()")
        return from_dict(dict(payload))

    if schema == "lingtu.status.localization_quality.v1":
        return float(payload)
    if schema == "lingtu.status.localization_health.v1":
        if not isinstance(payload, Mapping):
            raise ValueError("localization health payload must be a mapping")
        return dict(payload)
    if schema in _TEXT_SCHEMAS:
        return str(payload or "")
    return payload
