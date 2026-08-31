"""Validated recording read models for SimStudio-owned run artifacts."""

from __future__ import annotations

import hashlib
import os
from collections.abc import Mapping
from typing import Any

from sim.runtime.recording import RECORDING_FILENAME, TIMELINE_FILENAME
from sim.runtime.replay import SimulationReplay, SimulationReplayError

from .artifact_service import (
    ArtifactNotFound,
    ArtifactSecurityError,
    ArtifactService,
)
from .models import RecordNotFound, RunRecord
from .store import StudioStore

RECORDING_READ_MODEL_SCHEMA = "lingtu.sim.studio.recording.v1"
RECORDING_LIST_SCHEMA = "lingtu.sim.studio.recording-list.v1"
RECORDING_TIMELINE_SCHEMA = "lingtu.sim.studio.recording-timeline.v1"
RECORDING_FRAME_READ_MODEL_SCHEMA = "lingtu.sim.studio.recording-frame.v1"
_MAX_TIMELINE_PAGE_SIZE = 200


def _plain_json(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _plain_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_json(item) for item in value]
    return value


class RecordingBrowseError(ValueError):
    """Stable invalid-recording error for timeline and frame read models."""

    code = "SIMSTUDIO_RECORDING_INVALID"


class RecordingService:
    """Classify committed replay artifacts without accepting caller paths."""

    def __init__(self, store: StudioStore, artifacts: ArtifactService) -> None:
        self.store = store
        self.artifacts = artifacts

    def inspect(self, run_id: str) -> dict[str, Any]:
        """Return MISSING, INVALID, or fully validated recording metadata."""

        run = self._run(run_id)
        root = self.artifacts.trusted_run_root(run.id)
        manifest = root / RECORDING_FILENAME
        try:
            # These reads prove both fixed artifacts are run-owned regular
            # files with stable identities before the replay parser opens them.
            manifest_before = self.artifacts.get_artifact(
                run.id, RECORDING_FILENAME
            )
        except ArtifactNotFound:
            if os.path.lexists(manifest):
                return self._invalid(
                    run.id,
                    self._recording_id(run.id, "unsafe"),
                    "recording manifest is not a safe run-owned file",
                )
            return self._missing(run.id)
        except ArtifactSecurityError as exc:
            return self._invalid(
                run.id,
                self._recording_id(run.id, "unsafe"),
                str(exc) or type(exc).__name__,
            )

        manifest_digest = manifest_before.get("sha256")
        if not isinstance(manifest_digest, str):
            return self._invalid(
                run.id,
                self._recording_id(run.id, "invalid"),
                "recording manifest has no content digest",
            )
        recording_id = self._recording_id(run.id, manifest_digest)
        try:
            timeline_before = self.artifacts.get_artifact(
                run.id, TIMELINE_FILENAME
            )
            replay = SimulationReplay.open(root)
            manifest_after = self.artifacts.get_artifact(
                run.id, RECORDING_FILENAME
            )
            timeline_after = self.artifacts.get_artifact(
                run.id, TIMELINE_FILENAME
            )
            if manifest_before != manifest_after or timeline_before != timeline_after:
                raise ArtifactSecurityError(
                    "recording artifacts changed during validation"
                )
            self._validate_identity(run, replay)
        except (
            ArtifactNotFound,
            ArtifactSecurityError,
            SimulationReplayError,
            ValueError,
        ) as exc:
            return self._invalid(run.id, recording_id, str(exc) or type(exc).__name__)

        return {
            "schema": RECORDING_READ_MODEL_SCHEMA,
            "recording_id": recording_id,
            "run_id": run.id,
            "state": "VALID",
            "source_run_id": replay.run_id,
            "session_id": replay.session_id,
            "model_generation": replay.model_generation,
            "reset_generation": {
                "start": replay.start_reset_generation,
                "end": replay.end_reset_generation,
            },
            "frame_count": replay.frame_count,
            "duration_ns": replay.duration_ns,
            "terminal_state": replay.terminal_state,
            "event_order": list(replay.event_order),
            "replay": {
                "deterministic": True,
                "visual": True,
                "clock_authority": "recorded_mujoco",
            },
            "artifacts": {
                "manifest": RECORDING_FILENAME,
                "timeline": TIMELINE_FILENAME,
            },
            "diagnostics": [],
        }

    def list_recordings(self) -> dict[str, Any]:
        """List only runs that contain a recording commit marker."""

        recordings = []
        for run in self.store.list_runs():
            inspected = self.inspect(run.id)
            if inspected["state"] != "MISSING":
                recordings.append(inspected)
        return {
            "schema": RECORDING_LIST_SCHEMA,
            "recordings": recordings,
        }

    def timeline(
        self,
        run_id: str,
        *,
        offset: int = 0,
        limit: int = 100,
    ) -> dict[str, Any]:
        """Return one bounded page of validated replay-frame summaries."""

        if isinstance(offset, bool) or not isinstance(offset, int) or offset < 0:
            raise ValueError("recording timeline offset must be a non-negative integer")
        if (
            isinstance(limit, bool)
            or not isinstance(limit, int)
            or limit < 1
            or limit > _MAX_TIMELINE_PAGE_SIZE
        ):
            raise ValueError(
                f"recording timeline limit must be between 1 and {_MAX_TIMELINE_PAGE_SIZE}"
            )

        run, replay, recording_id = self._open_validated_replay(run_id)

        selected = replay.frames[offset : offset + limit]
        next_offset = offset + len(selected)
        if next_offset >= replay.frame_count:
            next_offset = None
        frames = []
        for frame in selected:
            snapshot = frame.snapshot
            frames.append(
                {
                    "frame_index": frame.frame_index,
                    "relative_time_ns": frame.relative_time_ns,
                    "sim_time_ns": snapshot.get("sim_time_ns"),
                    "sequence": snapshot.get("sequence"),
                    "physics_step": snapshot.get("physics_step"),
                    "model_generation": snapshot.get("model_generation"),
                    "reset_generation": snapshot.get("reset_generation"),
                    "body_count": len(snapshot.get("bodies", ())),
                    "joint_count": len(snapshot.get("joints", ())),
                    "actuator_count": len(snapshot.get("actuators", ())),
                    "sensor_count": len(snapshot.get("sensors", ())),
                    "has_command": frame.command is not None,
                    "scenario_event_count": len(frame.scenario_events),
                    "sensor_metadata_count": len(frame.sensor_metadata),
                    "sensor_payload_count": len(frame.sensor_payloads),
                    "sensor_payload_bytes": sum(
                        int(reference["bytes"])
                        for reference in frame.sensor_payloads
                    ),
                    "lifecycle_evidence_count": len(frame.lifecycle_evidence),
                }
            )
        return {
            "schema": RECORDING_TIMELINE_SCHEMA,
            "recording_id": recording_id,
            "run_id": run.id,
            "session_id": replay.session_id,
            "frame_count": replay.frame_count,
            "duration_ns": replay.duration_ns,
            "page": {
                "offset": offset,
                "limit": limit,
                "returned": len(frames),
                "next_offset": next_offset,
            },
            "frames": frames,
        }

    def frame(self, run_id: str, *, frame_index: int) -> dict[str, Any]:
        """Return one validated replay frame without accepting an artifact path."""

        if (
            isinstance(frame_index, bool)
            or not isinstance(frame_index, int)
            or frame_index < 0
        ):
            raise ValueError("recording frame index must be a non-negative integer")

        run, replay, recording_id = self._open_validated_replay(run_id)
        if frame_index >= replay.frame_count:
            raise ValueError(
                f"recording frame index {frame_index} is outside 0..{replay.frame_count - 1}"
            )

        frame = replay.frames[frame_index]
        return {
            "schema": RECORDING_FRAME_READ_MODEL_SCHEMA,
            "recording_id": recording_id,
            "run_id": run.id,
            "session_id": replay.session_id,
            "frame_index": frame.frame_index,
            "relative_time_ns": frame.relative_time_ns,
            "snapshot": _plain_json(frame.snapshot),
            "command": _plain_json(frame.command),
            "metadata": _plain_json(frame.metadata),
            "scenario_events": _plain_json(frame.scenario_events),
            "sensor_metadata": _plain_json(frame.sensor_metadata),
            "sensor_payloads": [
                self._public_payload_reference(reference, payload_index=index)
                for index, reference in enumerate(frame.sensor_payloads)
            ],
            "lifecycle_evidence": _plain_json(frame.lifecycle_evidence),
        }

    @staticmethod
    def _public_payload_reference(
        reference: Mapping[str, Any],
        *,
        payload_index: int,
    ) -> dict[str, Any]:
        public = _plain_json(reference)
        if not isinstance(public, dict):  # pragma: no cover - Mapping normalizes to dict
            raise RecordingBrowseError("sensor payload reference is invalid")
        public.pop("path", None)
        public["payload_index"] = payload_index
        return public

    def _open_validated_replay(
        self,
        run_id: str,
    ) -> tuple[RunRecord, SimulationReplay, str]:
        """Open one stable, run-owned replay behind the Studio identity boundary."""

        run = self._run(run_id)
        root = self.artifacts.trusted_run_root(run.id)
        manifest_before = self.artifacts.get_artifact(run.id, RECORDING_FILENAME)
        manifest_digest = manifest_before.get("sha256")
        if not isinstance(manifest_digest, str):
            raise ArtifactSecurityError("recording manifest has no content digest")
        timeline_before = self.artifacts.get_artifact(run.id, TIMELINE_FILENAME)
        try:
            replay = SimulationReplay.open(root)
            self._validate_identity(run, replay)
        except (SimulationReplayError, ValueError) as exc:
            raise RecordingBrowseError(str(exc) or type(exc).__name__) from exc
        manifest_after = self.artifacts.get_artifact(run.id, RECORDING_FILENAME)
        timeline_after = self.artifacts.get_artifact(run.id, TIMELINE_FILENAME)
        if manifest_before != manifest_after or timeline_before != timeline_after:
            raise ArtifactSecurityError("recording artifacts changed during validation")
        return run, replay, self._recording_id(run.id, manifest_digest)

    def _validate_identity(self, run: RunRecord, replay: SimulationReplay) -> None:
        if replay.run_id != run.id:
            raise ValueError("recording run_id does not match the SimStudio run")
        bundle_id = run.payload.get("bundle_id")
        if not isinstance(bundle_id, str):
            raise ValueError("SimStudio run has no bundle_id")
        try:
            bundle = self.store.get_bundle(bundle_id)
        except (KeyError, RecordNotFound) as exc:
            raise ValueError("recording references an unknown SimStudio bundle") from exc
        expected_digest = bundle.payload.get("session_id")
        if replay.session_id != expected_digest:
            raise ValueError(
                "recording session_id does not match the SimStudio bundle"
            )

    @staticmethod
    def _missing(run_id: str) -> dict[str, Any]:
        return {
            "schema": RECORDING_READ_MODEL_SCHEMA,
            "recording_id": None,
            "run_id": run_id,
            "state": "MISSING",
            "source_run_id": None,
            "session_id": None,
            "model_generation": None,
            "reset_generation": None,
            "frame_count": 0,
            "duration_ns": 0,
            "terminal_state": None,
            "event_order": [],
            "replay": {
                "deterministic": False,
                "visual": False,
                "clock_authority": None,
            },
            "artifacts": None,
            "diagnostics": [],
        }

    @staticmethod
    def _invalid(run_id: str, recording_id: str, message: str) -> dict[str, Any]:
        return {
            "schema": RECORDING_READ_MODEL_SCHEMA,
            "recording_id": recording_id,
            "run_id": run_id,
            "state": "INVALID",
            "source_run_id": None,
            "session_id": None,
            "model_generation": None,
            "reset_generation": None,
            "frame_count": 0,
            "duration_ns": 0,
            "terminal_state": None,
            "event_order": [],
            "replay": {
                "deterministic": False,
                "visual": False,
                "clock_authority": None,
            },
            "artifacts": {
                "manifest": RECORDING_FILENAME,
                "timeline": TIMELINE_FILENAME,
            },
            "diagnostics": [
                {
                    "code": "SIMSTUDIO_RECORDING_INVALID",
                    "message": message,
                }
            ],
        }

    @staticmethod
    def _recording_id(run_id: str, digest: str) -> str:
        return hashlib.sha256(f"{run_id}\0{digest}".encode()).hexdigest()[:32]

    def _run(self, run_id: str) -> RunRecord:
        try:
            return self.store.get_run(run_id)
        except (KeyError, RecordNotFound) as exc:
            raise ArtifactNotFound(f"run {run_id} was not found") from exc


__all__ = [
    "RECORDING_FRAME_READ_MODEL_SCHEMA",
    "RECORDING_LIST_SCHEMA",
    "RECORDING_READ_MODEL_SCHEMA",
    "RECORDING_TIMELINE_SCHEMA",
    "RecordingBrowseError",
    "RecordingService",
]
