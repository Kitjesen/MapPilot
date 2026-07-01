"""JSON message codec for cross-process endpoint transports.

This codec is intentionally small and conservative. It uses message
``to_dict`` / ``from_dict`` helpers when available and wraps the payload in a
versioned envelope so LCM endpoints do not depend on Python pickle.
"""

from __future__ import annotations

import base64
import importlib
import json
from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Any

FORMAT = "lingtu.transport.json.v1"
_TYPE_KEY = "__type__"
_BYTES_TYPE = "bytes"


def dumps_message(msg: Any) -> bytes:
    """Encode a Python message into a UTF-8 JSON envelope."""

    envelope = {
        "format": FORMAT,
        "type": _type_name(msg),
        "payload": _to_jsonable(msg),
    }
    return json.dumps(
        envelope,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def loads_message(data: bytes | bytearray | str) -> Any:
    """Decode a UTF-8 JSON envelope produced by :func:`dumps_message`."""

    if isinstance(data, str):
        raw = data
    else:
        raw = bytes(data).decode("utf-8")
    envelope = json.loads(raw)
    if not isinstance(envelope, dict) or envelope.get("format") != FORMAT:
        raise ValueError("unsupported LingTu JSON transport payload")
    payload = _from_jsonable(envelope.get("payload"))
    msg_type = str(envelope.get("type") or "")
    cls = _resolve_message_type(msg_type)
    if cls is None:
        return payload
    from_dict = getattr(cls, "from_dict", None)
    if callable(from_dict) and isinstance(payload, dict):
        return from_dict(payload)
    return payload


def _type_name(msg: Any) -> str:
    cls = type(msg)
    return f"{cls.__module__}.{cls.__qualname__}"


def _to_jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (bytes, bytearray)):
        return {
            _TYPE_KEY: _BYTES_TYPE,
            "data": base64.b64encode(bytes(value)).decode("ascii"),
        }
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(k): _to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_to_jsonable(v) for v in value]
    to_dict = getattr(value, "to_dict", None)
    if callable(to_dict):
        return _to_jsonable(to_dict())
    if is_dataclass(value):
        return _to_jsonable(asdict(value))
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _to_jsonable(tolist())
    raise TypeError(f"Object of type {type(value).__name__} is not JSON transport serializable")


def _from_jsonable(value: Any) -> Any:
    if isinstance(value, list):
        return [_from_jsonable(v) for v in value]
    if isinstance(value, dict):
        if value.get(_TYPE_KEY) == _BYTES_TYPE:
            return base64.b64decode(str(value.get("data", "")))
        return {k: _from_jsonable(v) for k, v in value.items()}
    return value


def _resolve_message_type(type_name: str) -> type[Any] | None:
    if not type_name.startswith("core.msgs."):
        return None
    module_name, _, class_name = type_name.rpartition(".")
    if not module_name or not class_name:
        return None
    try:
        module = importlib.import_module(module_name)
    except ImportError:
        return None
    cls = getattr(module, class_name, None)
    if isinstance(cls, type):
        return cls
    return None
