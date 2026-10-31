"""Shared request identities and offline-command receipts."""

from __future__ import annotations

import secrets
import time
from typing import Any

_CONNECTION_ERROR = "robot not reachable"
_ULID_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"


def _connection_error_payload(path: str, exc: BaseException) -> dict[str, Any]:
    """Return a stable SDK error payload for offline or unreachable robots."""

    return {
        "ok": False,
        "success": False,
        "error": _CONNECTION_ERROR,
        "detail": str(exc),
        "path": path,
    }


def _new_request_id() -> str:
    """Return a canonical 26-character uppercase ULID."""
    timestamp_ms = (time.time_ns() // 1_000_000) & ((1 << 48) - 1)
    value = (timestamp_ms << 80) | secrets.randbits(80)
    encoded = ["0"] * 26
    for index in range(25, -1, -1):
        encoded[index] = _ULID_ALPHABET[value & 0x1F]
        value >>= 5
    return "".join(encoded)
