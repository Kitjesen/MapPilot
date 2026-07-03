"""Structured logging for CLI runs.

Two log sinks per run:
  1. stderr 閳?human-friendly, filtered (noisy loggers silenced)
  2. file   閳?JSON lines (machine-parseable) OR plain text

Set ``log_format="json"`` (or ``--log-format json`` on CLI) to get JSON file logs.
Default is plain text for backward compatibility.
"""

from __future__ import annotations

import json
import logging
import sys
import time
import traceback
from pathlib import Path

from .paths import logs_base_dir

# Loggers that produce high-volume INFO noise during startup.
# They are silenced on stderr but still written to the log file.
_QUIET_PREFIXES = (
    "root",                                     # open_clip / timm model loading
    "timm",
    "open_clip",
    "runtime.dds",                                 # DDS reader setup
    "runtime.service_manager",                     # systemd service start/stop
    "runtime.blueprint",                           # "System started: N modules"
    "localization.bridge",                              # DDS transport choice
    "perception.encoding.mobileclip_encoder",
    "perception.encoding.encoder_module",
    "perception.perception_module",
    "decision.vision.person_tracker",
    "drivers.sim.ros2_sim_driver",
    "gateway.mcp_server",
    "memory.modules.vector_memory_module",
    "memory.modules.semantic_mapper_module",
    "memory.knowledge.room_object_kg",
)

# Loggers whose WARNING/ERROR messages are also not useful on screen.
# These are "graceful degradation" notices 閳?the system handles them automatically.
_MUTE_PREFIXES = (
    "decision.llm.llm_client",  # "API key not found"
    "nav.local.terrain",           # native kernel unavailable, falling back
    "nav.local.local_planner",     # compatibility shim
)


class _StderrFilter(logging.Filter):
    """Allow only lingtu-important messages through to stderr."""

    def filter(self, record: logging.LogRecord) -> bool:
        name = record.name
        # Always show ERROR and above
        if record.levelno >= logging.ERROR:
            return True
        # Fully mute noisy loggers
        for p in _MUTE_PREFIXES:
            if name == p or name.startswith(p + "."):
                return False
        # Quiet loggers: only show WARNING+
        for p in _QUIET_PREFIXES:
            if name == p or name.startswith(p + "."):
                return record.levelno >= logging.WARNING
        return True


class _JsonFormatter(logging.Formatter):
    """Emit one JSON object per log record (JSON Lines / NDJSON)."""

    def format(self, record: logging.LogRecord) -> str:
        obj = {
            "ts": self.formatTime(record, datefmt="%Y-%m-%dT%H:%M:%S.") +
                  f"{int(record.msecs):03d}Z",
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        if record.exc_info and record.exc_info[1] is not None:
            obj["exception"] = traceback.format_exception(*record.exc_info)
        # Extra structured fields 閳?modules can pass extra={"module": ...}
        for key in ("module", "port", "latency_ms", "state", "event"):
            val = getattr(record, key, None)
            if val is not None:
                obj[key] = val
        return json.dumps(obj, ensure_ascii=False, default=str)


def setup_logging(
    level: str,
    profile_name: str,
    log_format: str = "text",
) -> str:
    """Stderr (filtered) + full per-run file. Returns log directory path.

    Args:
        level: Log level (DEBUG / INFO / WARNING / ERROR).
        profile_name: Profile name for log directory.
        log_format: ``"text"`` (default) or ``"json"`` for JSON Lines file.
    """
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_dir = logs_base_dir() / f"{ts}_{profile_name}"
    log_dir.mkdir(parents=True, exist_ok=True)

    # --- stderr: always human-friendly ---
    stderr_h = logging.StreamHandler(sys.stderr)
    stderr_h.setFormatter(logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    ))
    stderr_h.addFilter(_StderrFilter())

    # --- file: JSON or plain text ---
    if log_format == "json":
        log_file = log_dir / "lingtu.jsonl"
        file_h = logging.FileHandler(str(log_file), encoding="utf-8")
        file_h.setFormatter(_JsonFormatter())
    else:
        log_file = log_dir / "lingtu.log"
        file_h = logging.FileHandler(str(log_file), encoding="utf-8")
        file_h.setFormatter(logging.Formatter(
            "%(asctime)s\t%(levelname)s\t%(name)s\t%(message)s",
        ))

    # --- secret redaction (applied to all handlers) ---
    try:
        from runtime.utils.redact import RedactSecretsFilter
        _redact = RedactSecretsFilter()
        stderr_h.addFilter(_redact)
        file_h.addFilter(_redact)
    except ImportError:
        pass  # core not on sys.path (e.g. standalone script)

    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    root.addHandler(stderr_h)
    root.addHandler(file_h)

    return str(log_dir)
