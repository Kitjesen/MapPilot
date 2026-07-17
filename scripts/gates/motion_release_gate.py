#!/usr/bin/env python3
"""Run required motion-release checks and write a machine-readable report."""

from __future__ import annotations

import json
import os
import re
import subprocess
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

SCHEMA_VERSION = "lingtu.motion_release_gate.v1"
ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class GateStep:
    """One required release-gate command."""

    name: str
    command: Sequence[str]
    timeout_s: float


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _step_slug(name: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", name).strip("-")
    return slug or "step"


def _captured_text(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def run_gate(
    steps: Sequence[GateStep],
    *,
    report_path: Path,
    cwd: Path = ROOT,
) -> int:
    """Run all steps, write the report atomically, and return a process exit code."""

    started_at = _utc_now()
    gate_started = time.monotonic()
    step_reports: list[dict[str, object]] = []
    output_dir = report_path.parent / "steps"
    output_dir.mkdir(parents=True, exist_ok=True)

    for index, step in enumerate(steps, start=1):
        step_started_at = _utc_now()
        step_started = time.monotonic()
        slug = f"{index:02d}-{_step_slug(step.name)}"
        stdout_path = (output_dir / f"{slug}.stdout.txt").resolve()
        stderr_path = (output_dir / f"{slug}.stderr.txt").resolve()
        stdout = ""
        stderr = ""
        exit_code: int | None = None
        status = "missing"
        try:
            if not step.command:
                raise FileNotFoundError("command is empty")
            completed = subprocess.run(
                list(step.command),
                cwd=cwd,
                capture_output=True,
                text=True,
                timeout=step.timeout_s,
                check=False,
            )
            stdout = completed.stdout
            stderr = completed.stderr
            exit_code = completed.returncode
            status = "passed" if exit_code == 0 else "failed"
        except subprocess.TimeoutExpired as exc:
            stdout = _captured_text(exc.stdout)
            stderr = _captured_text(exc.stderr)
            stderr += f"command timed out after {step.timeout_s:g} seconds\n"
            status = "timeout"
        except FileNotFoundError:
            missing = step.command[0] if step.command else "<empty>"
            stderr = f"command not found: {missing}\n"
        stdout_path.write_text(stdout, encoding="utf-8")
        stderr_path.write_text(stderr, encoding="utf-8")
        step_reports.append(
            {
                "name": step.name,
                "command": list(step.command),
                "timeout_s": step.timeout_s,
                "started_at": step_started_at,
                "finished_at": _utc_now(),
                "duration_s": round(time.monotonic() - step_started, 6),
                "status": status,
                "exit_code": exit_code,
                "stdout_path": str(stdout_path),
                "stderr_path": str(stderr_path),
            }
        )

    ok = bool(step_reports) and all(step["status"] == "passed" for step in step_reports)
    report: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "ok": ok,
        "started_at": started_at,
        "finished_at": _utc_now(),
        "duration_s": round(time.monotonic() - gate_started, 6),
        "steps": step_reports,
    }
    _write_json_atomic(report_path, report)
    return 0 if ok else 2
