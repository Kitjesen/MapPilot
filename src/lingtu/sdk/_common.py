"""Shared request identities and offline-command receipts."""

from __future__ import annotations

import secrets
import tempfile
import time
import urllib.request
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .client import LingTuClient

_CONNECTION_ERROR = "robot not reachable"
_ULID_ALPHABET = "0123456789ABCDEFGHJKMNPQRSTVWXYZ"


def _download_file(client: LingTuClient, path: str, target: str | Path) -> Path:
    """Replace the destination only after the complete authenticated transfer."""
    destination = Path(target)
    request = urllib.request.Request(client.base_url + path)
    if client._api_key:
        request.add_header("X-API-Key", client._api_key)
    temporary_path: Path | None = None
    try:
        with urllib.request.urlopen(request, timeout=client._timeout) as response:
            content_length = response.headers.get("Content-Length")
            expected_bytes = int(content_length) if content_length is not None else None
            received_bytes = 0
            with tempfile.NamedTemporaryFile(
                mode="wb", dir=destination.parent, prefix=f".{destination.name}.",
                suffix=".part", delete=False,
            ) as output:
                temporary_path = Path(output.name)
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    output.write(chunk)
                    received_bytes += len(chunk)
                if expected_bytes is not None and received_bytes != expected_bytes:
                    raise OSError(f"Incomplete download: expected {expected_bytes} bytes, received {received_bytes}")
        temporary_path.replace(destination)
    except BaseException:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise
    return destination


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
