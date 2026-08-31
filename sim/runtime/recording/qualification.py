"""Atomic evidence verdicts built from completed simulation artifacts."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from sim.runtime.qualification.session_record import (
    QualificationRecordError,
    build_e2e_qualification_record,
    safe_qualification_directory_root,
)

QUALIFICATION_RESULT_SCHEMA = "lingtu.sim.qualification-result.v1"
QUALIFICATION_RESULT_FILENAME = "qualification_result.json"


def write_qualification_result(bundle_dir: Path, run_dir: Path) -> Path:
    """Build and atomically commit a verdict without changing episode outcome."""

    bundle_root = safe_qualification_directory_root(Path(bundle_dir), "bundle_dir")
    run_root = safe_qualification_directory_root(
        Path(run_dir), "run_dir", must_exist=False
    )
    run_root.mkdir(parents=True, exist_ok=True)
    run_root = safe_qualification_directory_root(run_root, "run_dir")
    try:
        # Deferred import keeps the recording package import graph acyclic.
        from sim.runtime.replay.timeline import SimulationReplay, SimulationReplayError

        SimulationReplay.open(run_root)
        record = build_e2e_qualification_record(
            bundle_root,
            run_root,
        )
    except (QualificationRecordError, SimulationReplayError) as exc:
        result = _rejected_result(str(exc))
    else:
        qualified = record.get("qualified") is True
        result = {
            "schema": QUALIFICATION_RESULT_SCHEMA,
            "qualified": qualified,
            "verdict": "EVIDENCE_QUALIFIED" if qualified else "EVIDENCE_REJECTED",
            "record_sha256": _document_digest(record),
            "record": record,
            "error": None,
        }
    destination = run_root / QUALIFICATION_RESULT_FILENAME
    _atomic_write_json(destination, result)
    return destination


def _rejected_result(message: str) -> dict[str, Any]:
    reason = message.strip() or "qualification evidence is invalid"
    return {
        "schema": QUALIFICATION_RESULT_SCHEMA,
        "qualified": False,
        "verdict": "EVIDENCE_REJECTED",
        "record_sha256": None,
        "record": None,
        "error": {
            "code": "QUALIFICATION_EVIDENCE_INVALID",
            "message": reason,
        },
    }


def _canonical_json(document: Mapping[str, Any]) -> bytes:
    return json.dumps(
        document,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def _document_digest(document: Mapping[str, Any]) -> str:
    return hashlib.sha256(_canonical_json(document)).hexdigest()


def _atomic_write_json(path: Path, document: Mapping[str, Any]) -> None:
    payload = (
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    descriptor, name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        from sim.runtime.coordinator.atomic_file import replace_file_with_retry

        replace_file_with_retry(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


__all__ = [
    "QUALIFICATION_RESULT_FILENAME",
    "QUALIFICATION_RESULT_SCHEMA",
    "write_qualification_result",
]
