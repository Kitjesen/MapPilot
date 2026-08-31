"""Pure read-side E2E qualification records for completed simulation runs."""

from __future__ import annotations

import hashlib
import json
import math
import os
from collections.abc import Mapping
from pathlib import Path
from typing import Any, TypeGuard

import yaml
from sim.runtime.qualification.scenario_observability import (
    has_strict_physics_observability,
)

_SCHEMA = "lingtu.sim.e2e-qualification-record.v1"
_RUNTIME_SCHEMA = "lingtu.sim.session-runtime.v1"
_EPISODE_SCHEMA = "lingtu.sim.episode-result.v1"
_RECORDING_SCHEMA = "lingtu.sim.recording.v1"
_MOTION_SCHEMAS = frozenset(
    {
        "lingtu.sim.motion-recording-evidence.v1",
        "lingtu.sim.motion-recording-evidence.v2",
    }
)
_TERMINAL_RUNTIME_STATES = frozenset({"STOPPED", "FAILED"})
_FACTORY_PARK_MOTION_PROFILE = "factory_park_motion"
_FACTORY_PARK_MOTION_PROFILE_VERSION = 1
_FACTORY_PARK_MINIMUM_ROTATION_RAD = 0.35
_FACTORY_PARK_MAXIMUM_TURN_DRIFT_M = 0.10
_SCENARIO_VISUAL_SCHEMA = "lingtu.sim.scenario-visual-evidence.v1"
_SCENARIO_VISUAL_SOURCE = "ue_registry_applied"
_SCENARIO_VISUAL_INPUT_SOURCE = "canonical_scenario_snapshot"
_SCENARIO_VISUAL_POSITION_TOLERANCE_M = 0.02
_SCENARIO_PHYSICS_SCHEMA = "lingtu.sim.scenario-physics-evidence.v1"
_SCENARIO_PHYSICS_SOURCE = "mujoco_applied_kinematic_proxy"
_SCENARIO_PHYSICS_INPUT_SOURCE = "canonical_scenario_snapshot"
_SCENARIO_PHYSICS_POSITION_TOLERANCE_M = 0.02
_FACTORY_PARK_MOTION_MANEUVERS = (
    "forward",
    "backward",
    "left",
    "right",
    "turn_left",
    "turn_right",
)


class QualificationRecordError(ValueError):
    """Raised when qualification evidence is malformed or identity-stale."""


def build_e2e_qualification_record(bundle_dir: Path, run_dir: Path) -> dict[str, Any]:
    """Build a strict JSON-ready qualification record without starting processes."""

    bundle_root = safe_qualification_directory_root(Path(bundle_dir), "bundle_dir")
    run_root = safe_qualification_directory_root(Path(run_dir), "run_dir")
    try:
        session_spec = yaml.safe_load((bundle_root / "session.yaml").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise QualificationRecordError(f"cannot read session.yaml: {exc}") from exc
    if not isinstance(session_spec, Mapping):
        raise QualificationRecordError("session.yaml must contain an object")
    transport_intent = _read_optional_object(bundle_root / "transport.intent.json")
    scenario_plan = _read_optional_object(bundle_root / "scenario.plan.json")
    physics_plan = _read_optional_object(bundle_root / "physics.plan.json")
    runtime = _read_object(run_root / "session.runtime.json")
    episode = _read_object(run_root / "episode_result.json")
    motion = _read_object(run_root / "motion-evidence.json")
    recording = _read_object(run_root / "simulation-recording.json")

    _require_schema(runtime, _RUNTIME_SCHEMA, "session.runtime.json")
    _require_schema(episode, _EPISODE_SCHEMA, "episode_result.json")
    _require_schema(recording, _RECORDING_SCHEMA, "simulation-recording.json")
    if motion.get("schema") not in _MOTION_SCHEMAS:
        raise QualificationRecordError("motion-evidence.json has invalid schema")

    run_id = _string(runtime, "run_id", "session.runtime.json")
    session_id = _string(runtime, "session_id", "session.runtime.json")
    model_generation = _int(runtime, "model_generation", "session.runtime.json")
    reset_generation = _int(runtime, "reset_generation", "session.runtime.json")
    _same_identity(
        "episode_result.json",
        episode,
        run_id=run_id,
        session_id=session_id,
        model_generation=model_generation,
        reset_generation=reset_generation,
    )
    _same_identity(
        "motion-evidence.json",
        motion,
        run_id=run_id,
        session_id=session_id,
    )
    _same_identity(
        "simulation-recording.json",
        recording,
        run_id=run_id,
        session_id=session_id,
        model_generation=model_generation,
    )
    if session_spec.get("session_id") != session_id:
        raise QualificationRecordError("session.yaml session_id mismatch")
    if transport_intent is not None:
        _same_optional_session(transport_intent, session_id, "transport.intent.json")

    required_facets = _required_facets(session_spec, runtime)
    sensor_observation = _sensor_observation(runtime)
    allocation = _allocation(runtime, run_root)
    artifact_refs = _episode_artifact_references(episode)
    artifacts = _artifact_records(run_root, artifact_refs)
    _validate_recording_timeline(run_root, recording)

    reasons: list[str] = []
    if runtime.get("state") not in _TERMINAL_RUNTIME_STATES:
        reasons.append(f"runtime state is {runtime.get('state')!r}")
    for facet, state in required_facets.items():
        if state != "ACTIVE":
            reasons.append(f"required facet {facet} is {state}")
    for stream_id, state in sensor_observation["required"].items():
        if state != "ACTIVE":
            reasons.append(f"required sensor {stream_id} is {state}")
    if "sensors" in required_facets and not sensor_observation["required"]:
        reasons.append("required sensors have no required_stream_ids")
    reasons.extend(sensor_observation["sample_failures"])
    episode_status = episode.get("status")
    if episode_status != "SUCCEEDED":
        reasons.append(f"episode status is {episode_status}")
    motion_qualified = _motion_qualified(motion)
    if not motion_qualified:
        reasons.append("motion evidence is not qualified")
    scenario_requirements = _scenario_requirements(scenario_plan, physics_plan)
    scenario_visual = _scenario_visual_observation(
        run_root,
        artifact_refs,
        runtime,
        expected_entity_ids=scenario_requirements["entity_ids"],
        required=scenario_requirements["required"],
    )
    reasons.extend(scenario_visual["failures"])
    scenario_physics = _scenario_physics_observation(
        run_root,
        artifact_refs,
        runtime,
        expected_proxies=scenario_requirements["proxies"],
        required=scenario_requirements["required"],
    )
    reasons.extend(scenario_physics["failures"])
    reasons.extend(_scenario_clock_failures(scenario_visual["record"], scenario_physics["record"]))

    return {
        "schema": _SCHEMA,
        "qualified": not reasons,
        "reasons": reasons,
        "identity": {
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": model_generation,
            "reset_generation": reset_generation,
        },
        "paths": {
            "bundle_dir": str(bundle_root),
            "run_dir": str(run_root),
        },
        "required_facets": required_facets,
        "sensor_streams": sensor_observation,
        "transport_allocation": allocation,
        "episode": {
            "status": episode_status,
            "failure_reason": episode.get("failure_reason"),
            "artifact_references": artifact_refs,
        },
        "recording": {
            "motion_evidence": {
                "qualified": motion_qualified,
                "schema": motion["schema"],
                "frames": motion.get("frames"),
                "maneuvers": motion.get("maneuvers"),
            },
            "simulation": {
                "schema": recording["schema"],
                "timeline": recording.get("timeline"),
            },
        },
        "scenario_visual": scenario_visual["record"],
        "scenario_physics": scenario_physics["record"],
        "artifacts": artifacts,
    }


def _read_optional_object(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return _read_object(path)


def _read_object(path: Path) -> dict[str, Any]:
    _reject_symlink_or_reparse_components(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise QualificationRecordError(f"cannot read {path.name}: {exc}") from exc
    if not text:
        raise QualificationRecordError(f"{path.name} is empty")

    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        obj: dict[str, Any] = {}
        for key, value in pairs:
            if key in obj:
                raise QualificationRecordError(f"{path.name} contains duplicate key {key!r}")
            obj[key] = value
        return obj

    def reject_constant(value: str) -> None:
        raise QualificationRecordError(f"{path.name} contains non-finite value {value}")

    try:
        value = json.loads(
            text,
            object_pairs_hook=object_from_pairs,
            parse_constant=reject_constant,
        )
    except QualificationRecordError:
        raise
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise QualificationRecordError(f"{path.name} is not strict JSON: {exc}") from exc
    if type(value) is not dict:
        raise QualificationRecordError(f"{path.name} must be a JSON object")
    return value


def _require_schema(document: Mapping[str, Any], schema: str, label: str) -> None:
    if document.get("schema") != schema:
        raise QualificationRecordError(f"{label} has invalid schema")


def _same_identity(
    label: str,
    document: Mapping[str, Any],
    *,
    run_id: str,
    session_id: str,
    model_generation: int | None = None,
    reset_generation: int | None = None,
) -> None:
    if document.get("run_id") != run_id:
        raise QualificationRecordError(f"{label} run_id mismatch")
    if document.get("session_id") != session_id:
        raise QualificationRecordError(f"{label} session_id mismatch")
    if (
        model_generation is not None
        and document.get("model_generation") != model_generation
    ):
        raise QualificationRecordError(f"{label} model_generation mismatch")
    if reset_generation is not None:
        reset_value = document.get("reset_generation")
        if type(reset_value) is dict:
            matches = (
                reset_value.get("start") == reset_generation
                and reset_value.get("end") == reset_generation
            )
        else:
            matches = reset_value == reset_generation
        if not matches:
            raise QualificationRecordError(f"{label} reset_generation mismatch")


def _same_optional_session(
    document: Mapping[str, Any], session_id: str, label: str
) -> None:
    value = document.get("session_id")
    if value is not None and value != session_id:
        raise QualificationRecordError(f"{label} session_id mismatch")


def _required_facets(
    session_spec: Mapping[str, Any], runtime: Mapping[str, Any]
) -> dict[str, str]:
    required = _required_binding_names(session_spec)
    bindings = _mapping(runtime, "bindings", "session.runtime.json")
    observations: dict[str, str] = {}
    for facet in sorted(required):
        binding = bindings.get(facet)
        if type(binding) is not dict:
            observations[facet] = "MISSING"
        else:
            observations[facet] = str(binding.get("state", "MISSING"))
    return observations


def _required_binding_names(session_spec: Mapping[str, Any]) -> tuple[str, ...]:
    runtime = _mapping(session_spec, "runtime", "session.yaml")
    raw = runtime.get("required_bindings")
    if type(raw) is not list or not raw:
        raise QualificationRecordError("session.yaml required_bindings is invalid")
    names: list[str] = []
    seen: set[str] = set()
    for item in raw:
        if not isinstance(item, str) or not item:
            raise QualificationRecordError(
                "session.yaml required_bindings contains invalid value"
            )
        if item in seen:
            raise QualificationRecordError(
                "session.yaml required_bindings contains duplicate value"
            )
        seen.add(item)
        names.append(item)
    return tuple(names)


def _sensor_observation(runtime: Mapping[str, Any]) -> dict[str, Any]:
    sensor_streams = _mapping(runtime, "sensor_streams", "session.runtime.json")
    stream_documents = _mapping(sensor_streams, "streams", "session.runtime.json.sensor_streams")
    raw_required = sensor_streams.get("required_stream_ids", [])
    if type(raw_required) is not list:
        raise QualificationRecordError("required_stream_ids must be an array")
    required: dict[str, str] = {}
    all_streams: dict[str, str] = {}
    sample_failures: list[str] = []
    seen: set[str] = set()
    for stream_id, stream in sorted(stream_documents.items()):
        if not isinstance(stream_id, str) or not stream_id:
            raise QualificationRecordError("sensor stream id is invalid")
        if type(stream) is not dict:
            raise QualificationRecordError(f"sensor stream {stream_id} is invalid")
        all_streams[stream_id] = str(stream.get("state", "MISSING"))
    for item in raw_required:
        if not isinstance(item, str) or not item:
            raise QualificationRecordError("required_stream_ids contains invalid value")
        if item in seen:
            raise QualificationRecordError("required_stream_ids contains duplicate value")
        seen.add(item)
        stream = stream_documents.get(item)
        if type(stream) is not dict:
            required[item] = "MISSING"
        else:
            required[item] = all_streams.get(item, "MISSING")
            if stream.get("required") is not True:
                sample_failures.append(f"required sensor {item} is not marked required")
    summary = _sensor_summary(sensor_streams, required.keys(), runtime)
    sample_failures.extend(_sensor_sample_failures(summary, required.keys()))
    return {
        "is_ready": sensor_streams.get("is_ready") is True,
        "required": required,
        "all": all_streams,
        "summary": summary,
        "sample_failures": sample_failures,
    }


def _sensor_summary(
    sensor_streams: Mapping[str, Any],
    required_stream_ids: Any,
    runtime: Mapping[str, Any],
) -> dict[str, Any] | None:
    summary = sensor_streams.get("summary")
    if summary is None:
        return None
    if type(summary) is not dict:
        raise QualificationRecordError("sensor stream summary must be an object")
    if summary.get("schema") != "lingtu.sim.sensor-stream-summary.v1":
        raise QualificationRecordError("sensor stream summary has invalid schema")
    if summary.get("run_id") is not None and summary.get("run_id") != runtime.get("run_id"):
        raise QualificationRecordError("sensor stream summary run_id mismatch")
    if summary.get("session_id") != runtime.get("session_id"):
        raise QualificationRecordError("sensor stream summary session_id mismatch")
    if summary.get("model_generation") != runtime.get("model_generation"):
        raise QualificationRecordError("sensor stream summary model_generation mismatch")
    if summary.get("reset_generation") != runtime.get("reset_generation"):
        raise QualificationRecordError("sensor stream summary reset_generation mismatch")
    streams = _mapping(summary, "streams", "sensor stream summary")
    return {
        stream_id: streams.get(stream_id)
        for stream_id in sorted(required_stream_ids)
        if stream_id in streams
    }


def _sensor_sample_failures(
    summary: Mapping[str, Any] | None,
    required_stream_ids: Any,
) -> list[str]:
    if summary is None:
        return (
            ["required sensor summary is missing"]
            if tuple(required_stream_ids)
            else []
        )
    failures: list[str] = []
    for stream_id in sorted(required_stream_ids):
        stream = summary.get(stream_id)
        if type(stream) is not dict:
            failures.append(f"required sensor {stream_id} is missing summary")
            continue
        if stream.get("required") is not True:
            failures.append(f"required sensor {stream_id} summary is not marked required")
        if stream.get("state") != "ACTIVE":
            failures.append(f"required sensor {stream_id} summary is {stream.get('state')}")
        sample_count = stream.get("sample_count")
        if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count <= 0:
            failures.append(f"required sensor {stream_id} has no samples")
    return failures


def _allocation(runtime: Mapping[str, Any], run_root: Path) -> dict[str, Any]:
    allocation = _mapping(runtime, "allocation", "session.runtime.json")
    run_dir = allocation.get("run_dir")
    if run_dir is not None and Path(str(run_dir)).resolve() != run_root:
        raise QualificationRecordError("runtime allocation run_dir mismatch")
    return {
        "dds_domain": allocation.get("dds_domain"),
        "ports": allocation.get("ports", {}),
        "shm": allocation.get("shm", allocation.get("shared_memory", {})),
    }


def _episode_artifact_references(episode: Mapping[str, Any]) -> dict[str, str]:
    references = _mapping(episode, "artifact_references", "episode_result.json")
    result: dict[str, str] = {}
    seen_paths: set[str] = set()
    for key, value in sorted(references.items()):
        if not isinstance(key, str) or not isinstance(value, str) or not value:
            raise QualificationRecordError("episode artifact reference is invalid")
        if value in seen_paths:
            raise QualificationRecordError("episode artifact reference path is duplicated")
        seen_paths.add(value)
        result[key] = value
    return result


def _artifact_records(
    run_root: Path, references: Mapping[str, str]
) -> list[dict[str, Any]]:
    required = {
        "runtime_manifest": "session.runtime.json",
        "run_allocation": "run-allocation.json",
        "simulation_recording": "simulation-recording.json",
        "simulation_timeline": "simulation-timeline.jsonl",
    }
    for key, path in required.items():
        if references.get(key) != path:
            raise QualificationRecordError(f"episode artifact {key} mismatch")
    paths = {
        "session.runtime.json",
        "episode_result.json",
        "motion-evidence.json",
        *references.values(),
    }
    return [
        {
            "path": relative,
            "sha256": _sha256_file(_resolve_run_artifact(run_root, relative)),
        }
        for relative in sorted(paths)
    ]


def _validate_recording_timeline(
    run_root: Path, recording: Mapping[str, Any]
) -> None:
    timeline = _mapping(recording, "timeline", "simulation-recording.json")
    path = _string(timeline, "path", "simulation-recording.json.timeline")
    timeline_path = _resolve_run_artifact(run_root, path)
    expected_bytes = _int(timeline, "bytes", "simulation-recording.json.timeline")
    if timeline_path.stat().st_size != expected_bytes:
        raise QualificationRecordError("timeline byte count mismatch")
    expected_sha = _sha_string(timeline, "sha256", "simulation-recording.json.timeline")
    if _sha256_file(timeline_path) != expected_sha:
        raise QualificationRecordError("timeline sha256 mismatch")


def _motion_qualified(motion: Mapping[str, Any]) -> bool:
    frames = motion.get("frames")
    if type(frames) is dict:
        captured = frames.get("captured_count")
        minimum = frames.get("minimum_frames")
        if (
            isinstance(captured, int)
            and isinstance(minimum, int)
            and captured < minimum
        ):
            return False
    maneuvers = motion.get("maneuvers")
    if type(maneuvers) is list:
        if not maneuvers:
            return False
        return _maneuver_sequence_qualified(motion, maneuvers)
    motion_record = motion.get("motion")
    if type(motion_record) is dict:
        return motion_record.get("expectation_met") is True
    return False


def _maneuver_sequence_qualified(
    motion: Mapping[str, Any], maneuvers: list[Any]
) -> bool:
    names: list[str] = []
    seen: set[str] = set()
    for item in maneuvers:
        if type(item) is not dict:
            return False
        name = item.get("name")
        if not isinstance(name, str) or not name or name in seen:
            return False
        seen.add(name)
        names.append(name)
    profile = motion.get("acceptance_profile")
    profile_name = profile.get("name") if type(profile) is dict else None
    factory_profile = profile_name == _FACTORY_PARK_MOTION_PROFILE
    if factory_profile:
        if type(profile) is not dict:
            return False
        if profile.get("version") != _FACTORY_PARK_MOTION_PROFILE_VERSION:
            return False
        if tuple(names) != _FACTORY_PARK_MOTION_MANEUVERS:
            return False
        if not _factory_profile_thresholds_not_wider(motion):
            return False
    return all(
        _maneuver_qualified(
            item,
            motion_schema=str(motion.get("schema")),
            factory_profile=factory_profile,
        )
        for item in maneuvers
    )


def _factory_profile_thresholds_not_wider(motion: Mapping[str, Any]) -> bool:
    profile = motion.get("acceptance_profile")
    profile_thresholds_ok = True
    if type(profile) is dict:
        profile_thresholds_ok = _optional_minimum_not_wider(
            profile.get("minimum_rotation_rad")
        ) and _optional_maximum_not_wider(profile.get("maximum_turn_drift_m"))
    return (
        _optional_minimum_not_wider(motion.get("minimum_rotation_rad"))
        and _optional_maximum_not_wider(motion.get("maximum_turn_drift_m"))
        and profile_thresholds_ok
    )


def _optional_minimum_not_wider(value: Any) -> bool:
    return value is None or (
        _is_finite_number(value) and float(value) >= _FACTORY_PARK_MINIMUM_ROTATION_RAD
    )


def _optional_maximum_not_wider(value: Any) -> bool:
    return value is None or (
        _is_finite_number(value) and float(value) <= _FACTORY_PARK_MAXIMUM_TURN_DRIFT_M
    )


def _maneuver_qualified(
    item: Any, *, motion_schema: str, factory_profile: bool
) -> bool:
    if type(item) is not dict or item.get("expectation_met") is not True:
        return False
    if motion_schema != "lingtu.sim.motion-recording-evidence.v2":
        return True
    command = item.get("command")
    if type(command) is not dict:
        return False
    linear_x = command.get("linear_x")
    linear_y = command.get("linear_y")
    angular_z = command.get("angular_z")
    if not (
        _is_finite_number(linear_x)
        and _is_finite_number(linear_y)
        and _is_finite_number(angular_z)
    ):
        return False
    rotation_commanded = item.get("rotation_commanded")
    translation_commanded = item.get("translation_commanded")
    is_rotation = abs(float(angular_z)) > 0.0
    is_translation = abs(float(linear_x)) > 0.0 or abs(float(linear_y)) > 0.0
    if rotation_commanded is not is_rotation or translation_commanded is not is_translation:
        return False
    signed_translation = item.get("signed_translation_m")
    minimum_displacement = item.get("minimum_displacement_m")
    signed_yaw = item.get("signed_yaw_rad")
    drift = item.get("horizontal_drift_m")
    minimum_rotation = item.get("minimum_rotation_rad")
    maximum_drift = item.get("maximum_turn_drift_m")
    if not (
        item.get("motion_verified") is True
        and _is_finite_number(signed_translation)
        and _is_finite_number(minimum_displacement)
        and _is_finite_number(signed_yaw)
        and _is_finite_number(drift)
        and _is_finite_number(minimum_rotation)
        and _is_finite_number(maximum_drift)
        and item.get("translation_met") is True
        and item.get("rotation_met") is True
        and item.get("turn_drift_met") is True
    ):
        return False
    if factory_profile and (
        float(minimum_rotation) < _FACTORY_PARK_MINIMUM_ROTATION_RAD
        or float(maximum_drift) > _FACTORY_PARK_MAXIMUM_TURN_DRIFT_M
    ):
        return False
    if is_translation and float(signed_translation) < float(minimum_displacement):
        return False
    required_rotation = (
        _FACTORY_PARK_MINIMUM_ROTATION_RAD
        if factory_profile
        else float(minimum_rotation)
    )
    allowed_drift = (
        _FACTORY_PARK_MAXIMUM_TURN_DRIFT_M
        if factory_profile
        else float(maximum_drift)
    )
    if is_rotation and (float(signed_yaw) < required_rotation or float(drift) > allowed_drift):
        return False
    return True


def _scenario_visual_observation(
    run_root: Path,
    references: Mapping[str, str],
    runtime: Mapping[str, Any],
    *,
    expected_entity_ids: frozenset[str],
    required: bool,
) -> dict[str, Any]:
    reference = references.get("scenario_visual_evidence")
    if reference is None:
        return {
            "record": None,
            "failures": (
                ["scenario visual evidence is missing"] if required else []
            ),
        }
    evidence = _read_object(_resolve_run_artifact(run_root, reference))
    failures: list[str] = []
    if evidence.get("schema") != _SCENARIO_VISUAL_SCHEMA:
        failures.append("scenario visual evidence schema is invalid")
    if evidence.get("source") != _SCENARIO_VISUAL_SOURCE:
        failures.append("scenario visual evidence source is not ue_registry_applied")
    if evidence.get("input_source") != _SCENARIO_VISUAL_INPUT_SOURCE:
        failures.append(
            "scenario visual evidence input_source is not canonical_scenario_snapshot"
        )
    if not _same_required_run_id(evidence, runtime):
        failures.append("scenario visual evidence run_id mismatch")
    if evidence.get("session_id") != runtime.get("session_id"):
        failures.append("scenario visual evidence session_id mismatch")
    if evidence.get("model_generation") != runtime.get("model_generation"):
        failures.append("scenario visual evidence model_generation mismatch")
    if evidence.get("reset_generation") != runtime.get("reset_generation"):
        failures.append("scenario visual evidence reset_generation mismatch")
    sequence = evidence.get("sequence")
    sim_time_ns = evidence.get("sim_time_ns")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        failures.append("scenario visual evidence sequence is invalid")
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int) or sim_time_ns < 0:
        failures.append("scenario visual evidence sim_time_ns is invalid")
    tolerance = evidence.get("position_tolerance_m")
    maximum_error = evidence.get("maximum_position_error_m", evidence.get("maximum_error_m"))
    if (
        not _is_finite_number(tolerance)
        or float(tolerance) > _SCENARIO_VISUAL_POSITION_TOLERANCE_M
    ):
        failures.append("scenario visual evidence tolerance is wider than 0.02 m")
    if not _is_finite_number(maximum_error):
        failures.append("scenario visual evidence maximum_error_m is invalid")
    elif float(maximum_error) > _SCENARIO_VISUAL_POSITION_TOLERANCE_M:
        failures.append("scenario visual evidence is outside tolerance")
    if evidence.get("within_tolerance") is not True:
        failures.append("scenario visual evidence is outside tolerance")
    if required and evidence.get("complete_actor_set") is not True:
        failures.append("scenario visual evidence actor set is incomplete")
    if required and evidence.get("all_actors_visible") is not True:
        failures.append("scenario visual evidence actors are not all visible")
    actors = evidence.get("actors")
    actor_count = len(actors) if type(actors) is list else 0
    if actor_count <= 0:
        failures.append("scenario visual evidence has no actors")
    observed_entity_ids: set[str] = set()
    if type(actors) is list:
        for actor in actors:
            if type(actor) is not dict:
                failures.append("scenario visual evidence actor is invalid")
                continue
            entity_id = actor.get("entity_id")
            if not isinstance(entity_id, str) or not entity_id:
                failures.append("scenario visual evidence actor entity_id is invalid")
                continue
            observed_entity_ids.add(entity_id)
            if required and actor.get("visible") is not True:
                failures.append(f"scenario visual actor {entity_id} is not visible")
            error = actor.get("position_error_m", actor.get("error_m"))
            if not _is_finite_number(error) or float(error) > _SCENARIO_VISUAL_POSITION_TOLERANCE_M:
                failures.append(f"scenario visual actor {entity_id} is outside tolerance")
    if required:
        if observed_entity_ids != set(expected_entity_ids):
            failures.append("scenario visual evidence actor set does not match scenario plan")
        expected_count = evidence.get("expected_actor_count")
        reported_count = evidence.get("actor_count")
        if expected_count != len(expected_entity_ids) or reported_count != len(expected_entity_ids):
            failures.append("scenario visual evidence actor count mismatch")
    return {
        "record": {
            "path": reference,
            "source": evidence.get("source"),
            "input_source": evidence.get("input_source"),
            "position_tolerance_m": evidence.get("position_tolerance_m"),
            "maximum_position_error_m": maximum_error,
            "within_tolerance": evidence.get("within_tolerance"),
            "sequence": sequence,
            "sim_time_ns": sim_time_ns,
            "actor_count": actor_count,
        },
        "failures": list(dict.fromkeys(failures)),
    }


def _scenario_physics_observation(
    run_root: Path,
    references: Mapping[str, str],
    runtime: Mapping[str, Any],
    *,
    expected_proxies: frozenset[tuple[str, str]],
    required: bool,
) -> dict[str, Any]:
    reference = references.get("scenario_physics_evidence")
    if reference is None:
        return {
            "record": None,
            "failures": (
                ["scenario physics evidence is missing"] if required else []
            ),
        }
    evidence = _read_object(_resolve_run_artifact(run_root, reference))
    failures: list[str] = []
    if evidence.get("schema") != _SCENARIO_PHYSICS_SCHEMA:
        failures.append("scenario physics evidence schema is invalid")
    if evidence.get("source") != _SCENARIO_PHYSICS_SOURCE:
        failures.append("scenario physics evidence source is not mujoco_applied_kinematic_proxy")
    if evidence.get("input_source") != _SCENARIO_PHYSICS_INPUT_SOURCE:
        failures.append(
            "scenario physics evidence input_source is not canonical_scenario_snapshot"
        )
    if not _same_required_run_id(evidence, runtime):
        failures.append("scenario physics evidence run_id mismatch")
    if evidence.get("session_id") != runtime.get("session_id"):
        failures.append("scenario physics evidence session_id mismatch")
    if evidence.get("model_generation") != runtime.get("model_generation"):
        failures.append("scenario physics evidence model_generation mismatch")
    if evidence.get("reset_generation") != runtime.get("reset_generation"):
        failures.append("scenario physics evidence reset_generation mismatch")
    sequence = evidence.get("sequence")
    sim_time_ns = evidence.get("sim_time_ns")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        failures.append("scenario physics evidence sequence is invalid")
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int) or sim_time_ns < 0:
        failures.append("scenario physics evidence sim_time_ns is invalid")
    tolerance = evidence.get("position_tolerance_m")
    maximum_error = evidence.get("maximum_position_error_m", evidence.get("maximum_error_m"))
    if (
        not _is_finite_number(tolerance)
        or float(tolerance) > _SCENARIO_PHYSICS_POSITION_TOLERANCE_M
    ):
        failures.append("scenario physics evidence tolerance is wider than 0.02 m")
    if not _is_finite_number(maximum_error):
        failures.append("scenario physics evidence maximum_position_error_m is invalid")
    elif float(maximum_error) > _SCENARIO_PHYSICS_POSITION_TOLERANCE_M:
        failures.append("scenario physics evidence is outside tolerance")
    if evidence.get("within_tolerance") is not True:
        failures.append("scenario physics evidence is outside tolerance")
    if required and evidence.get("complete_proxy_set") is not True:
        failures.append("scenario physics evidence proxy set is incomplete")
    if required and evidence.get("pose_applied") is not True:
        failures.append("scenario physics pose proof is missing")
    if required and not has_strict_physics_observability(
        evidence,
        runtime,
        expected_entities=dict(expected_proxies),
    ):
        failures.append("scenario physics readback proof is missing")
    proxies = evidence.get("proxies")
    proxy_count = len(proxies) if type(proxies) is list else 0
    observed_proxies: set[tuple[str, str]] = set()
    if type(proxies) is list:
        for proxy in proxies:
            if type(proxy) is not dict:
                failures.append("scenario physics proxy is invalid")
                continue
            entity_id = proxy.get("entity_id")
            body_stable_id = proxy.get("body_stable_id")
            if (
                not isinstance(entity_id, str)
                or not entity_id
                or not isinstance(body_stable_id, str)
                or not body_stable_id
            ):
                failures.append("scenario physics proxy identity is invalid")
                continue
            observed_proxies.add((entity_id, body_stable_id))
            error = proxy.get("position_error_m", proxy.get("error_m"))
            if not _is_finite_number(error) or float(error) > _SCENARIO_PHYSICS_POSITION_TOLERANCE_M:
                failures.append(f"scenario physics proxy {entity_id} is outside tolerance")
    if required:
        if observed_proxies != set(expected_proxies):
            failures.append("scenario physics proxy set does not match scenario plan")
        expected_count = evidence.get("expected_proxy_count")
        reported_count = evidence.get("proxy_count")
        if expected_count != len(expected_proxies) or reported_count != len(expected_proxies):
            failures.append("scenario physics proxy count mismatch")
    return {
        "record": {
            "path": reference,
            "source": evidence.get("source"),
            "input_source": evidence.get("input_source"),
            "position_tolerance_m": evidence.get("position_tolerance_m"),
            "maximum_position_error_m": maximum_error,
            "within_tolerance": evidence.get("within_tolerance"),
            "sequence": sequence,
            "sim_time_ns": sim_time_ns,
            "proxy_count": proxy_count,
        },
        "failures": list(dict.fromkeys(failures)),
    }


def _scenario_clock_failures(
    visual: Mapping[str, Any] | None, physics: Mapping[str, Any] | None
) -> list[str]:
    if visual is None or physics is None:
        return []
    if visual.get("sequence") != physics.get("sequence"):
        return ["scenario visual/physics sequence mismatch"]
    if visual.get("sim_time_ns") != physics.get("sim_time_ns"):
        return ["scenario visual/physics sim_time_ns mismatch"]
    return []


def _same_required_run_id(
    evidence: Mapping[str, Any],
    runtime: Mapping[str, Any],
) -> bool:
    run_id = evidence.get("run_id")
    return isinstance(run_id, str) and run_id == run_id.strip() and run_id == runtime.get("run_id")


def _scenario_requirements(
    scenario_plan: Mapping[str, Any] | None,
    physics_plan: Mapping[str, Any] | None,
) -> dict[str, Any]:
    if scenario_plan is None:
        return {"required": False, "entity_ids": frozenset(), "proxies": frozenset()}
    entities = scenario_plan.get("entities")
    if type(entities) is not list:
        raise QualificationRecordError("scenario.plan.json entities must be an array")
    entity_ids: set[str] = set()
    proxies: set[tuple[str, str]] = set()
    requires_dispatch = False
    for entity in entities:
        if type(entity) is not dict:
            raise QualificationRecordError("scenario.plan.json entity is invalid")
        entity_id = entity.get("entity_id")
        if not isinstance(entity_id, str) or not entity_id:
            raise QualificationRecordError("scenario.plan.json entity_id is invalid")
        entity_ids.add(entity_id)
        if entity.get("authority") == "scenario":
            requires_dispatch = True
        proxy = entity.get("physics_proxy")
        if type(proxy) is dict and proxy.get("mode") == "kinematic":
            body_stable_id = proxy.get("body_stable_id")
            if not isinstance(body_stable_id, str) or not body_stable_id:
                raise QualificationRecordError("scenario.plan.json kinematic body_stable_id is invalid")
            proxies.add((entity_id, body_stable_id))
            requires_dispatch = True
    physics_entity_ids = _physics_kinematic_entity_ids(physics_plan)
    if proxies and {entity_id for entity_id, _body in proxies} != physics_entity_ids:
        raise QualificationRecordError("scenario and physics plans disagree on kinematic proxies")
    return {
        "required": requires_dispatch,
        "entity_ids": frozenset(entity_ids),
        "proxies": frozenset(proxies),
    }


def _physics_kinematic_entity_ids(physics_plan: Mapping[str, Any] | None) -> set[str]:
    if physics_plan is None:
        return set()
    raw = physics_plan.get("kinematic_entities", [])
    if type(raw) is not list:
        raise QualificationRecordError("physics.plan.json kinematic_entities must be an array")
    result: set[str] = set()
    for item in raw:
        if type(item) is not dict:
            raise QualificationRecordError("physics.plan.json kinematic entity is invalid")
        entity_id = item.get("entity_id")
        if not isinstance(entity_id, str) or not entity_id:
            raise QualificationRecordError("physics.plan.json kinematic entity_id is invalid")
        result.add(entity_id)
    return result


def _is_finite_number(value: Any) -> TypeGuard[int | float]:
    return not isinstance(value, bool) and isinstance(value, int | float) and math.isfinite(value)


def _resolve_run_artifact(run_root: Path, relative: str) -> Path:
    relative_path = Path(relative)
    if (
        not relative
        or "\\" in relative
        or relative_path.is_absolute()
        or ":" in relative
        or any(part in {"", ".", ".."} for part in relative_path.parts)
    ):
        raise QualificationRecordError(f"artifact path is not run-relative: {relative!r}")
    candidate = run_root / relative_path
    _reject_symlink_or_reparse_components(candidate)
    resolved = candidate.resolve(strict=True)
    if run_root != resolved and run_root not in resolved.parents:
        raise QualificationRecordError(f"artifact path escapes run_dir: {relative!r}")
    if not resolved.is_file():
        raise QualificationRecordError(f"artifact is missing: {relative}")
    return resolved


def safe_qualification_directory_root(
    path: Path, label: str, *, must_exist: bool = True
) -> Path:
    """Return a trusted absolute directory after rejecting symlink/reparse parts."""

    absolute = _absolute_without_resolving(path)
    if must_exist:
        _reject_symlink_or_reparse_components(absolute)
    else:
        _reject_existing_symlink_or_reparse_components(absolute)
    if must_exist and not absolute.is_dir():
        raise QualificationRecordError(f"{label} must be an existing directory")
    return absolute.resolve(strict=must_exist) if must_exist else absolute


def _absolute_without_resolving(path: Path) -> Path:
    return path if path.is_absolute() else Path.cwd() / path


def _reject_symlink_or_reparse_components(path: Path) -> None:
    absolute = _absolute_without_resolving(path)
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        try:
            stat_result = os.lstat(current)
        except OSError as exc:
            raise QualificationRecordError(f"cannot lstat path component: {current}") from exc
        if current.is_symlink() or _is_windows_reparse_point(stat_result):
            raise QualificationRecordError(
                f"path component is symlink or reparse point: {current}"
            )


def _reject_existing_symlink_or_reparse_components(path: Path) -> None:
    absolute = _absolute_without_resolving(path)
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if not current.exists():
            return
        try:
            stat_result = os.lstat(current)
        except OSError as exc:
            raise QualificationRecordError(f"cannot lstat path component: {current}") from exc
        if current.is_symlink() or _is_windows_reparse_point(stat_result):
            raise QualificationRecordError(
                f"path component is symlink or reparse point: {current}"
            )


def _is_windows_reparse_point(stat_result: os.stat_result) -> bool:
    return bool(getattr(stat_result, "st_file_attributes", 0) & 0x400)


def _mapping(document: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    value = document.get(key)
    if type(value) is not dict:
        raise QualificationRecordError(f"{label}.{key} must be an object")
    return value


def _string(document: Mapping[str, Any], key: str, label: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value:
        raise QualificationRecordError(f"{label}.{key} must be a non-empty string")
    return value


def _sha_string(document: Mapping[str, Any], key: str, label: str) -> str:
    value = _string(document, key, label)
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise QualificationRecordError(f"{label}.{key} must be a sha256 digest")
    return value


def _int(document: Mapping[str, Any], key: str, label: str) -> int:
    value = document.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise QualificationRecordError(f"{label}.{key} must be an integer")
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


__all__ = [
    "QualificationRecordError",
    "build_e2e_qualification_record",
    "safe_qualification_directory_root",
]
