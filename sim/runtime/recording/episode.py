"""Model and atomic writer for terminal simulation episode evidence."""

from __future__ import annotations

import json
import os
import re
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from types import MappingProxyType

EPISODE_RESULT_SCHEMA = "lingtu.sim.episode-result.v1"
EPISODE_RESULT_FILENAME = "episode_result.json"
_RUN_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")


def _validate_non_negative_integer(value: object, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")


def _validated_artifact_references(value: object) -> Mapping[str, str]:
    if not isinstance(value, Mapping):
        raise ValueError("artifact_references must be an object")
    references: dict[str, str] = {}
    for name, path in value.items():
        if not isinstance(name, str) or not name or name != name.strip():
            raise ValueError("artifact reference name must be non-empty text")
        if not isinstance(path, str) or not path or path != path.strip():
            raise ValueError("artifact reference path must be non-empty text")
        references[name] = path
    return MappingProxyType(dict(sorted(references.items())))


class EpisodeStatus(str, Enum):
    """Terminal outcome of one simulation episode."""

    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class EpisodeResult:
    """Terminal, generation-stamped evidence for one simulation run."""

    run_id: str
    session_id: str
    model_generation: int
    reset_generation: int
    start_sim_time_ns: int
    end_sim_time_ns: int
    status: EpisodeStatus
    failure_reason: str | None = None
    artifact_references: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.run_id, str) or _RUN_ID_RE.fullmatch(self.run_id) is None:
            raise ValueError("run_id has an invalid value")
        if not isinstance(self.status, EpisodeStatus):
            raise ValueError("status must be an EpisodeStatus")
        if not isinstance(self.session_id, str) or not self.session_id.strip():
            raise ValueError("session_id must be non-empty")
        for value, field_name in (
            (self.model_generation, "model_generation"),
            (self.reset_generation, "reset_generation"),
            (self.start_sim_time_ns, "start_sim_time_ns"),
            (self.end_sim_time_ns, "end_sim_time_ns"),
        ):
            _validate_non_negative_integer(value, field_name)
        if self.end_sim_time_ns < self.start_sim_time_ns:
            raise ValueError("end_sim_time_ns must not precede start_sim_time_ns")
        object.__setattr__(self, "artifact_references", _validated_artifact_references(self.artifact_references))
        if self.status is EpisodeStatus.FAILED:
            if not isinstance(self.failure_reason, str) or not self.failure_reason.strip():
                raise ValueError("FAILED episode requires a non-empty failure_reason")
        elif self.failure_reason is not None:
            raise ValueError("failure_reason is only valid for FAILED episodes")

    def to_dict(self) -> dict[str, object]:
        """Return the language-neutral v1 episode result document."""

        return {
            "schema": EPISODE_RESULT_SCHEMA,
            "run_id": self.run_id,
            "session_id": self.session_id,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "start_sim_time_ns": self.start_sim_time_ns,
            "end_sim_time_ns": self.end_sim_time_ns,
            "status": self.status.value,
            "failure_reason": self.failure_reason,
            "artifact_references": dict(self.artifact_references),
        }

    def to_json(self) -> str:
        """Serialize the result deterministically as strict UTF-8 JSON text."""

        return (
            json.dumps(
                self.to_dict(),
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n"
        )


class EpisodeRecorder:
    """Atomically materialize terminal episode evidence inside one run directory."""

    def __init__(self, run_dir: Path) -> None:
        self._run_dir = Path(run_dir)

    @property
    def path(self) -> Path:
        """Return the fixed episode result path for this run."""

        return self._run_dir / EPISODE_RESULT_FILENAME

    def write(self, result: EpisodeResult) -> Path:
        """Write one complete result and return its final path."""

        if not isinstance(result, EpisodeResult):
            raise TypeError("result must be an EpisodeResult")
        self._run_dir.mkdir(parents=True, exist_ok=True)
        _atomic_write(self.path, result.to_json().encode("utf-8"))
        return self.path


def _atomic_write(destination: Path, payload: bytes) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        dir=destination.parent,
        prefix=f".{destination.name}.",
        suffix=".tmp",
    )
    temporary = Path(temporary_name)
    try:
        stream = os.fdopen(descriptor, "wb")
        descriptor = -1
        with stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        # Deferred import avoids a recording/coordinator package import cycle.
        from sim.runtime.coordinator.atomic_file import replace_file_with_retry

        replace_file_with_retry(temporary, destination)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
