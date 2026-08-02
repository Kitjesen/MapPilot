"""Structured audit logging for Gateway control commands.

Records who issued what command and when, without storing payloads that
may contain sensitive data.  Audit entries are written to a dedicated
logger (``lingtu.gateway.audit``) so they can be routed to a separate
file or syslog facility via standard Python logging configuration.

Environment variables:
  LINGTU_AUDIT_LOG_ENABLED  - "0"/"false" to disable (default: enabled)
  LINGTU_AUDIT_LOG_MAX_ENTRIES - in-memory ring buffer size (default: 256)
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque
from typing import Any

audit_logger = logging.getLogger("lingtu.gateway.audit")

_DEFAULT_MAX_ENTRIES = 256

# Commands that are audited (control-plane operations)
AUDITED_COMMANDS = frozenset(
    {
        "goal",
        "navigate_click",
        "navigation_cancel",
        "navigation_task_pause",
        "navigation_task_resume",
        "stop",
        "emergency_stop",
        "estop_reset",
        "cmd_vel",
        "mode",
        "instruction",
        "lease",
        "visual_servo",
        "driver_swap",
        "slam_switch",
        "slam_relocalize",
        "exploration_start",
        "exploration_stop",
        "map_save",
        "map_delete",
        "map_rename",
        "map_set_active",
        "bag_start",
        "bag_stop",
    }
)


def _env_enabled(name: str, default: bool = True) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off"}


class AuditJournal:
    """In-memory ring buffer of control command audit entries.

    Thread-safe.  Each entry records:
      - ts: Unix timestamp
      - command: command name
      - client_id: caller-provided client identifier
      - client_ip: source IP (when available)
      - ok: whether the command was accepted
      - error: error reason (if rejected)
    """

    def __init__(self, max_entries: int | None = None) -> None:
        resolved_max = max_entries or int(
            os.environ.get("LINGTU_AUDIT_LOG_MAX_ENTRIES", str(_DEFAULT_MAX_ENTRIES))
        )
        self._entries: deque[dict[str, Any]] = deque(maxlen=resolved_max)
        self._lock = threading.Lock()
        self._enabled = _env_enabled("LINGTU_AUDIT_LOG_ENABLED")
        self._total_commands = 0
        self._total_rejected = 0

    def record(
        self,
        command: str,
        *,
        client_id: str = "unknown",
        client_ip: str | None = None,
        ok: bool = True,
        error: str | None = None,
        detail: str | None = None,
    ) -> None:
        """Record an audited command."""
        if not self._enabled:
            return
        entry = {
            "ts": time.time(),
            "command": command,
            "client_id": client_id,
            "client_ip": client_ip,
            "ok": ok,
            "error": error,
        }
        if detail:
            entry["detail"] = detail[:200]

        with self._lock:
            self._entries.append(entry)
            self._total_commands += 1
            if not ok:
                self._total_rejected += 1

        # Structured log line for external log routing
        level = logging.INFO if ok else logging.WARNING
        audit_logger.log(
            level,
            "cmd=%s client=%s ip=%s ok=%s%s",
            command,
            client_id,
            client_ip or "-",
            ok,
            f" error={error}" if error else "",
        )

    def snapshot(self, *, limit: int = 50) -> dict[str, Any]:
        """Return recent audit entries for diagnostics endpoints."""
        with self._lock:
            entries = list(self._entries)[-limit:]
            return {
                "enabled": self._enabled,
                "total_commands": self._total_commands,
                "total_rejected": self._total_rejected,
                "buffer_size": len(self._entries),
                "recent": entries,
            }
