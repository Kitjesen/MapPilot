"""Read native snapshots without inferring freshness, readiness, or motion permission."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any


def read_json_snapshot(path: str) -> dict[str, Any] | None:
    try:
        with open(path, encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _navigation_status_path() -> str:
    session_root = os.environ.get("LINGTU_SESSION_ROOT", "").strip()
    default_path = (
        str(Path(session_root) / "nav.status.json")
        if os.environ.get("LINGTU_ENV") == "sim" and session_root
        else "/dev/shm/lingtu/nav_endpoint_status.json"
    )
    return os.environ.get("LINGTU_NAV_STATUS_FILE", "").strip() or default_path


def read_navigation_status() -> dict[str, Any] | None:
    return read_json_snapshot(_navigation_status_path())


def read_traversability_status() -> dict[str, Any] | None:
    session_root = os.environ.get("LINGTU_SESSION_ROOT", "").strip()
    default_path = (
        str(Path(session_root) / "traversability.status.json")
        if os.environ.get("LINGTU_ENV") == "sim" and session_root
        else "/dev/shm/lingtu/traversability_status.json"
    )
    path = os.environ.get("LINGTU_TRAVERSABILITY_STATUS_FILE", "").strip() or default_path
    return read_json_snapshot(path)
