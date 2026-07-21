"""Read-only telemetry for the native exploration endpoint.

The status file never carries command authority. Gateway commands still cross
``liblingtu_nav_client`` and wait for a typed DDS ExplorationCommandAck.
"""

from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Any

_SCHEMA = "lingtu.explore.status.v2"
_DEFAULT_PATH = "/dev/shm/lingtu/explore_status.json"


def status_path() -> Path:
    return Path(os.environ.get("LINGTU_EXPLORE_STATUS_FILE", "").strip() or _DEFAULT_PATH)


def read_status() -> dict[str, Any] | None:
    try:
        with status_path().open(encoding="utf-8") as handle:
            payload = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or payload.get("schema_version") != _SCHEMA:
        return None
    return payload


def status_is_fresh(
    payload: Mapping[str, Any] | None,
    *,
    now_s: float | None = None,
) -> bool:
    if not isinstance(payload, Mapping):
        return False
    try:
        stamp_s = float(payload.get("stamp_s"))
        max_age_s = float(os.environ.get("LINGTU_EXPLORE_STATUS_MAX_AGE_S", "6.0") or "6.0")
    except (TypeError, ValueError):
        return False
    if not math.isfinite(stamp_s) or not math.isfinite(max_age_s) or max_age_s <= 0.0:
        return False
    age_s = (time.time() if now_s is None else float(now_s)) - stamp_s
    return -0.25 <= age_s <= max_age_s


def read_fresh_status(*, now_s: float | None = None) -> dict[str, Any] | None:
    payload = read_status()
    return payload if status_is_fresh(payload, now_s=now_s) else None
