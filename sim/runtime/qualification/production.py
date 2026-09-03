"""Fail-closed G007 production qualification for completed sim runs."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, TypeGuard

from sim.runtime.qualification.scenario_observability import (
    has_strict_physics_observability,
)
from sim.runtime.qualification.session_record import (
    QualificationRecordError,
    build_e2e_qualification_record,
    safe_qualification_directory_root,
)
from sim.runtime.replay.timeline import (
    SimulationReplay,
    SimulationReplayError,
    compare_replays,
    replay_snapshots,
)
from sim.runtime.qualification.thunderv4 import THUNDERV4_NAVIGATION_STREAM_IDS

QUALIFICATION_RESULT_SCHEMA = "lingtu.sim.qualification-result.v1"
QUALIFICATION_RESULT_FILENAME = "qualification_result.json"
G007_PRODUCTION_RECORD_SCHEMA = "lingtu.sim.g007-production-qualification.v1"
SCENARIO_PHYSICS_EVIDENCE_SCHEMA = "lingtu.sim.scenario-physics-evidence.v1"
SCENARIO_PHYSICS_SOURCE = "mujoco_applied_kinematic_proxy"
SCENARIO_EVIDENCE_INPUT_SOURCE = "canonical_scenario_snapshot"
SCENARIO_VISUAL_EVIDENCE_SCHEMA = "lingtu.sim.scenario-visual-evidence.v1"
SCENARIO_VISUAL_SOURCE = "ue_registry_applied"
SCENARIO_POSITION_TOLERANCE_M = 0.02


class ProductionQualificationError(ValueError):
    """Raised when production qualification evidence is incomplete or stale."""


def write_g007_production_qualification_result(bundle_dir: Path, run_dir: Path) -> Path:
    """Atomically write the G007 qualification verdict for one completed run."""

    bundle_root = safe_qualification_directory_root(Path(bundle_dir), "bundle_dir")
    run_root = safe_qualification_directory_root(
        Path(run_dir), "run_dir", must_exist=False
    )
    run_root.mkdir(parents=True, exist_ok=True)
    run_root = safe_qualification_directory_root(run_root, "run_dir")
    try:
        record = build_g007_production_qualification_record(bundle_root, run_root)
    except (
        ProductionQualificationError,
        QualificationRecordError,
        SimulationReplayError,
    ) as exc:
        result: dict[str, Any] = {
            "schema": QUALIFICATION_RESULT_SCHEMA,
            "qualified": False,
            "verdict": "EVIDENCE_REJECTED",
            "record_sha256": None,
            "record": None,
            "error": {
                "code": "G007_PRODUCTION_EVIDENCE_INVALID",
                "message": str(exc).strip() or "G007 production evidence is invalid",
            },
        }
    else:
        qualified = record["qualified"] is True
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
    return Path(destination)


def build_g007_production_qualification_record(
    bundle_dir: Path,
    run_dir: Path,
) -> dict[str, Any]:
    """Build a read-only G007 production qualification record."""

    bundle_root = _safe_directory(Path(bundle_dir), "bundle_dir")
    run_root = _safe_directory(Path(run_dir), "run_dir")
    e2e_record = build_e2e_qualification_record(bundle_root, run_root)
    runtime = _read_object(run_root / "session.runtime.json")
    episode = _read_object(run_root / "episode_result.json")
    scenario_plan = _read_object(bundle_root / "scenario.plan.json")
    physics_plan = _read_object(bundle_root / "physics.plan.json")
    references = _artifact_references(episode)
    expected_visual_entities = _expected_visual_entities(scenario_plan)
    expected_entities = _expected_kinematic_entities(scenario_plan, physics_plan)
    replay = SimulationReplay.open(run_root)

    checks = {
        "session_contract": _session_contract(runtime),
        "sensor_streams": _exact_five_sensor_streams(e2e_record),
        "scenario_visual": _scenario_visual_check(
            run_root,
            references,
            runtime,
            expected_entity_ids=expected_visual_entities,
        ),
        "scenario_physics": _scenario_physics_check(
            run_root,
            references,
            runtime,
            expected_entities=expected_entities,
        ),
        "recording_replay": _recording_replay_check(replay),
    }
    reasons: list[str] = []
    if e2e_record.get("qualified") is not True:
        reasons.extend(str(reason) for reason in e2e_record.get("reasons", []))
    for check_name, check in checks.items():
        if check.get("qualified") is not True:
            failures = check.get("failures")
            if isinstance(failures, list) and failures:
                reasons.extend(str(item) for item in failures)
            else:
                reasons.append(f"{check_name} is not qualified")
    return {
        "schema": G007_PRODUCTION_RECORD_SCHEMA,
        "qualified": not reasons,
        "reasons": list(dict.fromkeys(reasons)),
        "identity": dict(e2e_record["identity"]),
        "e2e_record": e2e_record,
        "checks": checks,
    }


def _session_contract(runtime: Mapping[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    bindings = _mapping(runtime, "bindings", "session.runtime.json")
    required = {"physics", "visual", "sensors", "control"}
    for facet in sorted(required):
        binding = bindings.get(facet)
        if type(binding) is not dict or binding.get("state") != "ACTIVE":
            failures.append(f"required production facet {facet} is not ACTIVE")
    return {
        "qualified": not failures,
        "required_bindings": sorted(required),
        "failures": failures,
    }


def _exact_five_sensor_streams(e2e_record: Mapping[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    sensor_streams = _mapping(e2e_record, "sensor_streams", "e2e_record")
    required = _mapping(sensor_streams, "required", "e2e_record.sensor_streams")
    expected = set(THUNDERV4_NAVIGATION_STREAM_IDS)
    actual = set(required)
    if actual != expected:
        failures.append(
            "production sensors must be the exact ThunderV4 navigation five-stream set"
        )
    for stream_id in sorted(expected & actual):
        if required.get(stream_id) != "ACTIVE":
            failures.append(f"production sensor {stream_id} is not ACTIVE")
    return {
        "qualified": not failures,
        "required_stream_ids": sorted(actual),
        "expected_stream_ids": list(THUNDERV4_NAVIGATION_STREAM_IDS),
        "failures": failures,
    }


def _scenario_visual_check(
    run_root: Path,
    references: Mapping[str, str],
    runtime: Mapping[str, Any],
    *,
    expected_entity_ids: frozenset[str],
) -> dict[str, Any]:
    reference = references.get("scenario_visual_evidence")
    if reference is None:
        return {"qualified": False, "failures": ["scenario visual evidence is missing"]}
    evidence = _read_object(_resolve_run_artifact(run_root, reference))
    failures = _same_scenario_evidence_identity(
        evidence,
        runtime,
        schema=SCENARIO_VISUAL_EVIDENCE_SCHEMA,
        source=SCENARIO_VISUAL_SOURCE,
        label="scenario visual evidence",
    )
    maximum_error = _maximum_position_error(evidence)
    if maximum_error is None:
        failures.append("scenario visual evidence maximum position error is invalid")
    elif maximum_error > SCENARIO_POSITION_TOLERANCE_M:
        failures.append("scenario visual evidence is outside tolerance")
    if evidence.get("within_tolerance") is not True:
        failures.append("scenario visual evidence is outside tolerance")
    actors = _entity_records(evidence.get("actors"), label="scenario visual actors")
    if not actors:
        failures.append("scenario visual evidence has no actors")
    elif set(actors) != expected_entity_ids:
        failures.append("scenario visual actor set does not match the scenario plan")
    return {
        "qualified": not failures,
        "path": reference,
        "entity_ids": sorted(actors),
        "expected_entity_ids": sorted(expected_entity_ids),
        "maximum_position_error_m": maximum_error,
        "failures": list(dict.fromkeys(failures)),
    }


def _scenario_physics_check(
    run_root: Path,
    references: Mapping[str, str],
    runtime: Mapping[str, Any],
    *,
    expected_entities: Mapping[str, str],
) -> dict[str, Any]:
    reference = references.get("scenario_physics_evidence")
    if reference is None:
        return {"qualified": False, "failures": ["scenario physics evidence is missing"]}
    evidence = _read_object(_resolve_run_artifact(run_root, reference))
    failures = _same_scenario_evidence_identity(
        evidence,
        runtime,
        schema=SCENARIO_PHYSICS_EVIDENCE_SCHEMA,
        source=SCENARIO_PHYSICS_SOURCE,
        label="scenario physics evidence",
    )
    maximum_error = _maximum_position_error(evidence)
    if maximum_error is None:
        failures.append("scenario physics evidence maximum position error is invalid")
    elif maximum_error > SCENARIO_POSITION_TOLERANCE_M:
        failures.append("scenario physics evidence is outside tolerance")
    if evidence.get("within_tolerance") is not True:
        failures.append("scenario physics evidence is outside tolerance")
    proxies = _entity_records(evidence.get("proxies"), label="scenario physics proxies")
    if not proxies:
        failures.append("scenario physics evidence has no proxies")
    elif proxies != expected_entities:
        failures.append("scenario physics proxy set does not match compiled plans")
    if not has_strict_physics_observability(
        evidence,
        runtime,
        expected_entities=expected_entities,
    ):
        failures.append("scenario physics evidence has no MuJoCo readback/raycast/contact proof")
    return {
        "qualified": not failures,
        "path": reference,
        "entity_ids": sorted(proxies),
        "expected_entities": dict(sorted(expected_entities.items())),
        "maximum_position_error_m": maximum_error,
        "failures": list(dict.fromkeys(failures)),
    }


def _recording_replay_check(replay: SimulationReplay) -> dict[str, Any]:
    first_snapshots: list[dict[str, Any]] = []
    second_snapshots: list[dict[str, Any]] = []

    def capture_first(snapshot: dict[str, Any]) -> int:
        first_snapshots.append(snapshot)
        return 1

    def capture_second(snapshot: dict[str, Any]) -> int:
        second_snapshots.append(snapshot)
        return 1

    first = replay_snapshots(
        replay,
        capture_first,
        pace=False,
    )
    second = replay_snapshots(
        replay,
        capture_second,
        pace=False,
    )
    comparison = compare_replays(replay, replay)
    failures: list[str] = []
    if first.frames_dropped or second.frames_dropped:
        failures.append("deterministic replay dropped frames")
    if first.event_order != second.event_order:
        failures.append("deterministic replay event order changed")
    if first.generations != second.generations:
        failures.append("deterministic replay generations changed")
    if first.terminal_state != second.terminal_state:
        failures.append("deterministic replay terminal state changed")
    if first_snapshots != second_snapshots:
        failures.append("deterministic replay snapshots changed")
    if comparison.equivalent is not True:
        failures.append("recording replay comparison is not equivalent")
    return {
        "qualified": not failures,
        "first": _replay_report_dict(first),
        "second": _replay_report_dict(second),
        "comparison": {
            "equivalent": comparison.equivalent,
            "discrete_match": comparison.discrete_match,
            "continuous_match": comparison.continuous_match,
            "continuous_fields_compared": comparison.continuous_fields_compared,
            "mismatches": list(comparison.mismatches),
            "clock_authority": comparison.clock_authority,
            "tolerance_policy_version": comparison.tolerance_policy_version,
        },
        "failures": failures,
    }


def _same_scenario_evidence_identity(
    evidence: Mapping[str, Any],
    runtime: Mapping[str, Any],
    *,
    schema: str,
    source: str,
    label: str,
) -> list[str]:
    failures: list[str] = []
    if evidence.get("schema") != schema:
        failures.append(f"{label} schema is invalid")
    if evidence.get("source") != source:
        failures.append(f"{label} source is invalid")
    if evidence.get("input_source") != SCENARIO_EVIDENCE_INPUT_SOURCE:
        failures.append(f"{label} input_source is not canonical_scenario_snapshot")
    if not _same_required_run_id(evidence, runtime):
        failures.append(f"{label} run_id mismatch")
    for field in ("session_id", "model_generation", "reset_generation"):
        if evidence.get(field) != runtime.get(field):
            failures.append(f"{label} {field} mismatch")
    for field in ("sequence", "sim_time_ns"):
        if not _non_negative_int(evidence.get(field)):
            failures.append(f"{label} {field} is invalid")
    tolerance = evidence.get("position_tolerance_m")
    if not _finite_number(tolerance):
        failures.append(f"{label} tolerance is wider than 0.02 m")
    elif float(tolerance) > SCENARIO_POSITION_TOLERANCE_M:
        failures.append(f"{label} tolerance is wider than 0.02 m")
    return failures


def _same_required_run_id(
    evidence: Mapping[str, Any],
    runtime: Mapping[str, Any],
) -> bool:
    run_id = evidence.get("run_id")
    return isinstance(run_id, str) and run_id == run_id.strip() and run_id == runtime.get("run_id")


def _expected_kinematic_entities(
    scenario_plan: Mapping[str, Any],
    physics_plan: Mapping[str, Any],
) -> dict[str, str]:
    entities = _sequence(scenario_plan.get("entities"), "scenario.plan.json.entities")
    expected: dict[str, str] = {}
    for index, entity in enumerate(entities):
        if type(entity) is not dict:
            raise ProductionQualificationError(
                f"scenario.plan.json.entities[{index}] must be an object"
            )
        raw_proxy = entity.get("physics_proxy")
        if raw_proxy == "mujoco":
            continue
        if type(raw_proxy) is not dict:
            raise ProductionQualificationError(
                f"scenario.plan.json.entities[{index}].physics_proxy must be an object"
            )
        proxy = raw_proxy
        if proxy.get("mode") != "kinematic":
            continue
        entity_id = _text(entity.get("entity_id"), f"scenario entity {index}.entity_id")
        body_stable_id = _text(
            proxy.get("body_stable_id"),
            f"scenario entity {entity_id}.physics_proxy.body_stable_id",
        )
        expected[entity_id] = body_stable_id
    if not expected:
        raise ProductionQualificationError("scenario plan has no kinematic proxy entities")
    physics_entities = _sequence(
        physics_plan.get("kinematic_entities"),
        "physics.plan.json.kinematic_entities",
    )
    physics_ids = {
        _text(item.get("entity_id"), "physics kinematic entity_id")
        for item in physics_entities
        if type(item) is dict
    }
    if physics_ids != set(expected):
        raise ProductionQualificationError(
            "physics plan kinematic entity set does not match scenario plan"
        )
    return expected


def _expected_visual_entities(scenario_plan: Mapping[str, Any]) -> frozenset[str]:
    entities = _sequence(scenario_plan.get("entities"), "scenario.plan.json.entities")
    result: set[str] = set()
    for index, entity in enumerate(entities):
        if type(entity) is not dict:
            raise ProductionQualificationError(
                f"scenario.plan.json.entities[{index}] must be an object"
            )
        result.add(_text(entity.get("entity_id"), f"scenario entity {index}.entity_id"))
    if not result:
        raise ProductionQualificationError("scenario plan has no visual entities")
    return frozenset(result)


def _entity_records(value: object, *, label: str) -> dict[str, str]:
    if not isinstance(value, list):
        return {}
    result: dict[str, str] = {}
    for index, item in enumerate(value):
        if type(item) is not dict:
            raise ProductionQualificationError(f"{label}[{index}] must be an object")
        entity_id = _text(item.get("entity_id"), f"{label}[{index}].entity_id")
        body_stable_id = item.get("body_stable_id")
        if body_stable_id is None:
            body_stable_id = item.get("stable_id")
        if body_stable_id is None:
            body_stable_id = ""
        if not isinstance(body_stable_id, str):
            raise ProductionQualificationError(
                f"{label}[{index}].body_stable_id must be text"
            )
        if entity_id in result:
            raise ProductionQualificationError(f"{label} contains duplicate entity_id")
        result[entity_id] = body_stable_id
    return result


def _maximum_position_error(evidence: Mapping[str, Any]) -> float | None:
    value = evidence.get("maximum_position_error_m")
    if value is None:
        value = evidence.get("maximum_error_m")
    if not _finite_number(value):
        return None
    return float(value)


def _replay_report_dict(report: Any) -> dict[str, Any]:
    return {
        "session_id": report.session_id,
        "frames_presented": report.frames_presented,
        "frames_dropped": report.frames_dropped,
        "duration_ns": report.duration_ns,
        "event_order": list(report.event_order),
        "generations": [list(item) for item in report.generations],
        "terminal_state": report.terminal_state,
        "clock_authority": report.clock_authority,
        "tolerance_policy_version": report.tolerance_policy_version,
    }


def _artifact_references(episode: Mapping[str, Any]) -> dict[str, str]:
    references = _mapping(episode, "artifact_references", "episode_result.json")
    result: dict[str, str] = {}
    for key, value in references.items():
        result[_text(key, "artifact reference key")] = _text(
            value,
            f"artifact reference {key}",
        )
    return result


def _read_object(path: Path) -> dict[str, Any]:
    _reject_symlink_or_reparse_components(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ProductionQualificationError(f"cannot read {path.name}: {exc}") from exc
    if not text:
        raise ProductionQualificationError(f"{path.name} is empty")

    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        document: dict[str, Any] = {}
        for key, value in pairs:
            if key in document:
                raise ProductionQualificationError(
                    f"{path.name} contains duplicate key {key!r}"
                )
            document[key] = value
        return document

    def reject_constant(value: str) -> None:
        raise ProductionQualificationError(f"{path.name} contains non-finite value {value}")

    try:
        value = json.loads(
            text,
            object_pairs_hook=object_from_pairs,
            parse_constant=reject_constant,
        )
    except ProductionQualificationError:
        raise
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise ProductionQualificationError(f"{path.name} is not strict JSON") from exc
    if type(value) is not dict:
        raise ProductionQualificationError(f"{path.name} must be a JSON object")
    return value


def _resolve_run_artifact(run_root: Path, relative: str) -> Path:
    relative_path = Path(relative)
    if (
        not relative
        or "\\" in relative
        or relative_path.is_absolute()
        or ":" in relative
        or any(part in {"", ".", ".."} for part in relative_path.parts)
    ):
        raise ProductionQualificationError(
            f"artifact path is not run-relative: {relative!r}"
        )
    candidate = run_root / relative_path
    _reject_symlink_or_reparse_components(candidate)
    resolved = candidate.resolve(strict=True)
    if run_root != resolved and run_root not in resolved.parents:
        raise ProductionQualificationError(f"artifact path escapes run_dir: {relative!r}")
    if not resolved.is_file():
        raise ProductionQualificationError(f"artifact is missing: {relative}")
    return resolved


def _safe_directory(path: Path, label: str) -> Path:
    absolute = path if path.is_absolute() else Path.cwd() / path
    _reject_symlink_or_reparse_components(absolute)
    if not absolute.is_dir():
        raise ProductionQualificationError(f"{label} must be an existing directory")
    return absolute.resolve(strict=True)


def _reject_symlink_or_reparse_components(path: Path) -> None:
    absolute = path if path.is_absolute() else Path.cwd() / path
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        try:
            stat_result = os.lstat(current)
        except OSError as exc:
            raise ProductionQualificationError(
                f"cannot lstat path component: {current}"
            ) from exc
        if current.is_symlink() or bool(
            getattr(stat_result, "st_file_attributes", 0) & 0x400
        ):
            raise ProductionQualificationError(
                f"path component is symlink or reparse point: {current}"
            )


def _mapping(document: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    value = document.get(key)
    if type(value) is not dict:
        raise ProductionQualificationError(f"{label}.{key} must be an object")
    return value


def _sequence(value: object, label: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        raise ProductionQualificationError(f"{label} must be an array")
    return value


def _text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ProductionQualificationError(f"{label} must be non-empty trimmed text")
    return value


def _finite_number(value: object) -> TypeGuard[int | float]:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _non_negative_int(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, int) and value >= 0


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
    "G007_PRODUCTION_RECORD_SCHEMA",
    "ProductionQualificationError",
    "build_g007_production_qualification_record",
    "write_g007_production_qualification_result",
]
