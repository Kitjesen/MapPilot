"""Session-bound confirmation that MuJoCo applied a terminal zero command."""

from __future__ import annotations

import json
import math
import os
import secrets
from collections.abc import Callable, Mapping
from pathlib import Path, PurePosixPath
from typing import Any

from lingtu.switch_contracts import is_product_session_id

MOTION_STOP_SCHEMA = "lingtu.sim.motion_stop.v2"
PROCESS_LAUNCH_ID_ENV = "LINGTU_PROCESS_LAUNCH_ID"
_FIELDS = frozenset(
    {
        "schema",
        "product_session_id",
        "product",
        "process",
        "outcome",
        "launch_id",
        "bridge_boot_id",
        "controller_boot_id",
        "bridge_command_seq",
        "applied_step_seq",
        "command_kind",
        "walk_x",
        "walk_y",
        "walk_z",
        "terminal_ack",
    }
)


class SimStopEvidenceError(RuntimeError):
    """The terminal zero confirmation is missing or invalid."""


def process_launch_id(environment: Mapping[str, str] | None = None) -> str:
    source = os.environ if environment is None else environment
    launch_id = source.get(PROCESS_LAUNCH_ID_ENV)
    if not isinstance(launch_id, str) or not launch_id:
        raise SimStopEvidenceError("process launch id is missing or invalid")
    return launch_id


def publish_motion_stop_evidence(
    *,
    session_root: Path,
    target: str,
    payload: Mapping[str, Any],
    environment: Mapping[str, str] | None = None,
    bound_publisher: Callable[[str, bytes], Path] | None = None,
) -> Mapping[str, Any]:
    if type(payload) is not dict or frozenset(payload) != _FIELDS - {"launch_id"}:
        raise SimStopEvidenceError("motion stop evidence fields are invalid")
    launch_id = process_launch_id(environment)
    complete = {**payload, "launch_id": launch_id}
    _validate_payload(
        complete,
        product_session_id=complete.get("product_session_id"),
        product=complete.get("product"),
        process=complete.get("process"),
        launch_id=launch_id,
    )
    raw = _json_bytes(complete)
    try:
        if bound_publisher is not None:
            bound_publisher(target, raw)
        else:
            _atomic_write(_session_file(session_root, target), raw)
    except OSError as exc:
        raise SimStopEvidenceError("motion stop evidence cannot be published") from exc
    return dict(complete)


def load_motion_stop_evidence(
    *,
    session_root: Path,
    target: str,
    product_session_id: str,
    product: str,
    process: str,
    launch_id: str,
) -> Mapping[str, Any]:
    path = _session_file(session_root, target)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise SimStopEvidenceError("motion stop evidence file is unavailable") from exc
    if len(raw) > 4096:
        raise SimStopEvidenceError("motion stop evidence exceeds 4096 bytes")
    try:
        payload = json.loads(raw)
    except (UnicodeError, ValueError, TypeError) as exc:
        raise SimStopEvidenceError("motion stop evidence is invalid JSON") from exc
    if type(payload) is not dict:
        raise SimStopEvidenceError("motion stop evidence must be a JSON object")
    _validate_payload(
        payload,
        product_session_id=product_session_id,
        product=product,
        process=process,
        launch_id=launch_id,
    )
    return payload


def _session_file(session_root: Path, target: str) -> Path:
    if not isinstance(target, str) or not target or "\\" in target:
        raise SimStopEvidenceError("motion stop evidence path is invalid")
    relative = PurePosixPath(target)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise SimStopEvidenceError("motion stop evidence path is invalid")
    return session_root / Path(*relative.parts)


def _json_bytes(payload: Mapping[str, Any]) -> bytes:
    try:
        return json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
    except (TypeError, ValueError, RecursionError) as exc:
        raise SimStopEvidenceError("motion stop evidence is not serializable") from exc


def _atomic_write(path: Path, raw: bytes) -> None:
    temporary = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")
    try:
        temporary.write_bytes(raw)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _validate_payload(
    payload: Mapping[str, Any],
    *,
    product_session_id: object,
    product: object,
    process: object,
    launch_id: object,
) -> None:
    if frozenset(payload) != _FIELDS:
        raise SimStopEvidenceError("motion stop evidence fields are invalid")
    if (
        payload["schema"] != MOTION_STOP_SCHEMA
        or not isinstance(payload["product_session_id"], str)
        or payload["product_session_id"] != product_session_id
        or not is_product_session_id(payload["product_session_id"])
        or payload["product"] != product
        or not _plain_text(payload["product"])
        or payload["process"] != process
        or not _plain_text(payload["process"])
        or payload["launch_id"] != launch_id
        or not _plain_text(payload["launch_id"])
        or payload["outcome"] != "zero_applied"
        or payload["command_kind"] != "deactivate_zero"
        or payload["terminal_ack"] is not True
    ):
        raise SimStopEvidenceError("motion stop evidence identity is invalid")
    if not _plain_text(payload["bridge_boot_id"]) or not _plain_text(
        payload["controller_boot_id"]
    ):
        raise SimStopEvidenceError("motion stop boot identity is invalid")
    for key in ("bridge_command_seq", "applied_step_seq"):
        value = payload[key]
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise SimStopEvidenceError("motion stop sequence is invalid")
    for key in ("walk_x", "walk_y", "walk_z"):
        value = payload[key]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value != 0.0
        ):
            raise SimStopEvidenceError("motion stop zero command is invalid")


def _plain_text(value: object) -> bool:
    return isinstance(value, str) and bool(value) and value == value.strip()


__all__ = [
    "MOTION_STOP_SCHEMA",
    "PROCESS_LAUNCH_ID_ENV",
    "SimStopEvidenceError",
    "load_motion_stop_evidence",
    "process_launch_id",
    "publish_motion_stop_evidence",
]
