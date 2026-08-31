"""Run one production SimStudio MuJoCo + RobotSimUE acceptance cycle."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.simstudio.service.application import SimulationStudioService

_FACETS = ("physics", "visual", "sensors", "control")
_SENSOR_CONTRACT: dict[str, dict[str, Any]] = {
    "rgb": {"width": 640, "height": 480, "rate_hz": 30},
    "depth": {"width": 640, "height": 480, "rate_hz": 30},
    "mid360": {"rate_hz": 10},
    "imu": {"rate_hz": 200},
    "truth_odom": {"rate_hz": 100, "estimator_input": False},
}


class AcceptanceError(RuntimeError):
    """Raised when runtime evidence does not meet the visual acceptance gate."""


def _object(value: Any, label: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise AcceptanceError(f"{label} must be an object")
    return value


def _json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise AcceptanceError(f"cannot read {path}: {exc}") from exc
    return _object(value, str(path))


def _text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise AcceptanceError(f"cannot read {path}: {exc}") from exc


def _canonical_directory(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value or not Path(value).is_absolute():
        raise AcceptanceError(f"{label} must be an absolute canonical directory")
    candidate = Path(value)
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise AcceptanceError(f"{label} must be an existing canonical directory: {exc}") from exc
    if not resolved.is_dir() or candidate != resolved:
        raise AcceptanceError(f"{label} must be an existing canonical directory")
    return resolved


def _run_directory(value: Path) -> Path:
    try:
        resolved = Path(value).resolve(strict=True)
    except OSError as exc:
        raise AcceptanceError(f"run_dir is not an existing directory: {exc}") from exc
    if not resolved.is_dir():
        raise AcceptanceError("run_dir is not an existing directory")
    return resolved


def _bound_file(root: Path, relative: Path, label: str) -> Path:
    candidate = root / relative
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise AcceptanceError(f"cannot read {candidate}: {exc}") from exc
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise AcceptanceError(f"{label} is not run-bound") from exc
    if candidate != resolved or not resolved.is_file():
        raise AcceptanceError(f"{label} is not run-bound")
    return resolved


def _require_bound_directory(value: Any, expected: Path, label: str) -> None:
    resolved = _canonical_directory(value, label)
    if resolved != expected:
        raise AcceptanceError(f"{label} is not bound to {expected}")


def _require_schema(document: Mapping[str, Any], expected: str, label: str) -> None:
    if document.get("schema") != expected:
        raise AcceptanceError(f"{label} schema is not {expected}")


def _runtime_identity(manifest: Mapping[str, Any]) -> dict[str, str | int]:
    run_id = manifest.get("run_id")
    if not isinstance(run_id, str) or not run_id or run_id != run_id.strip():
        raise AcceptanceError("runtime run_id identity is invalid")
    session_id = manifest.get("session_id")
    if not isinstance(session_id, str) or not session_id or session_id != session_id.strip():
        raise AcceptanceError("runtime session_id identity is invalid")
    result: dict[str, str | int] = {
        "run_id": run_id,
        "session_id": session_id,
    }
    for field in ("model_generation", "reset_generation"):
        value = manifest.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise AcceptanceError(f"runtime {field} identity is invalid")
        result[field] = value
    return result


def _plan_streams(sensor_plan: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    groups = _object(sensor_plan.get("streams"), "sensor.plan.streams")
    result: dict[str, dict[str, Any]] = {}
    for kind, entries in groups.items():
        if not isinstance(kind, str) or not isinstance(entries, list):
            raise AcceptanceError("sensor.plan streams must be arrays keyed by kind")
        for entry in entries:
            stream = _object(entry, f"sensor.plan.streams.{kind}[]")
            sensor_id = stream.get("sensor_id")
            if not isinstance(sensor_id, str) or not sensor_id:
                raise AcceptanceError(f"sensor.plan {kind} stream has no sensor_id")
            if sensor_id in result:
                raise AcceptanceError(f"duplicate SensorPlan sensor_id: {sensor_id}")
            result[sensor_id] = stream
    return result


def validate_sensor_contract(sensor_plan: Mapping[str, Any]) -> dict[str, str]:
    """Require the navigation sensor shape promised by the acceptance scenario."""

    groups = _object(sensor_plan.get("streams"), "sensor.plan.streams")
    for kind, expected in _SENSOR_CONTRACT.items():
        entries = groups.get(kind)
        if not isinstance(entries, list) or len(entries) != 1:
            raise AcceptanceError(f"sensor.plan must declare exactly one {kind} stream")
        stream = _object(entries[0], f"sensor.plan.streams.{kind}[0]")
        mismatches = [
            f"{field}={stream.get(field)!r} (expected {value!r})"
            for field, value in expected.items()
            if stream.get(field) != value
        ]
        if mismatches:
            raise AcceptanceError(f"{kind} contract mismatch: " + ", ".join(mismatches))

    return {
        "depth": "640x480@30Hz",
        "imu": "200Hz",
        "mid360": "10Hz",
        "rgb": "640x480@30Hz",
        "truth_odom": "100Hz (truth-only)",
    }


def validate_ready_manifest(
    manifest: Mapping[str, Any],
    sensor_plan: Mapping[str, Any],
) -> dict[str, Any]:
    """Require all four runtime facets and every compiled stream to be ACTIVE."""

    if manifest.get("schema") != "lingtu.sim.session-runtime.v1":
        raise AcceptanceError("runtime manifest schema is not lingtu.sim.session-runtime.v1")
    if manifest.get("state") not in {"READY", "RUNNING", "PAUSED"}:
        raise AcceptanceError(f"runtime state is not acceptance-ready: {manifest.get('state')!r}")
    model_generation = manifest.get("model_generation")
    reset_generation = manifest.get("reset_generation")
    bindings = _object(manifest.get("bindings"), "runtime.bindings")
    binding_summary: dict[str, str] = {}
    for facet in _FACETS:
        binding = _object(bindings.get(facet), f"runtime.bindings.{facet}")
        if binding.get("required") is not True or binding.get("state") != "ACTIVE":
            raise AcceptanceError(
                f"required {facet} binding is {binding.get('state')!r}: "
                f"{binding.get('failure_reason') or 'no failure reason'}"
            )
        for field, expected in (
            ("model_generation", model_generation),
            ("reset_generation", reset_generation),
        ):
            if field in binding and binding.get(field) != expected:
                raise AcceptanceError(f"{facet} {field} does not match runtime generation")
        binding_summary[facet] = "ACTIVE"

    sensor_runtime = _object(manifest.get("sensor_streams"), "runtime.sensor_streams")
    if sensor_runtime.get("is_ready") is not True:
        raise AcceptanceError("runtime sensor aggregate is not ready")
    runtime_streams = _object(sensor_runtime.get("streams"), "runtime.sensor_streams.streams")
    planned_streams = _plan_streams(sensor_plan)
    required_ids = sensor_runtime.get("required_stream_ids")
    if not isinstance(required_ids, list) or set(required_ids) != set(planned_streams):
        raise AcceptanceError("runtime required sensor IDs do not match SensorPlan")
    sensor_summary: dict[str, str] = {}
    for sensor_id in sorted(planned_streams):
        stream = _object(runtime_streams.get(sensor_id), f"runtime sensor {sensor_id}")
        if stream.get("required") is not True or stream.get("state") != "ACTIVE":
            raise AcceptanceError(
                f"required sensor {sensor_id} is {stream.get('state')!r}: "
                f"{stream.get('failure_reason') or 'no failure reason'}"
            )
        for field, expected in (
            ("model_generation", model_generation),
            ("reset_generation", reset_generation),
        ):
            if field in stream and stream.get(field) != expected:
                raise AcceptanceError(f"sensor {sensor_id} {field} does not match runtime generation")
        sensor_summary[sensor_id] = "ACTIVE"
    return {"bindings": binding_summary, "sensors": sensor_summary}


def diagnose_offline_startup(run_dir: Path) -> tuple[str, ...]:
    """Return startup blockers found in the artifacts of one existing run."""

    run_root = _run_directory(run_dir)
    manifest = _json_object(
        _bound_file(run_root, Path("session.runtime.json"), "session.runtime.json")
    )
    episode = _json_object(
        _bound_file(run_root, Path("episode_result.json"), "episode_result.json")
    )
    _require_schema(manifest, "lingtu.sim.session-runtime.v1", "runtime")
    _require_schema(episode, "lingtu.sim.episode-result.v1", "episode")
    identity = _runtime_identity(manifest)
    for field, expected in identity.items():
        if episode.get(field) != expected:
            raise AcceptanceError(f"episode {field} identity mismatch")
    references = episode.get("artifact_references")
    if (
        not isinstance(references, Mapping)
        or references.get("runtime_manifest") != "session.runtime.json"
    ):
        raise AcceptanceError("episode runtime_manifest reference is not session.runtime.json")
    allocation = _object(manifest.get("allocation"), "runtime.allocation")
    _require_bound_directory(allocation.get("run_dir"), run_root, "allocation run_dir")
    _require_bound_directory(
        allocation.get("log_dir"),
        run_root / "logs",
        "allocation log_dir",
    )
    bundle_value = manifest.get("bundle_dir")
    if not isinstance(bundle_value, str) or not bundle_value:
        raise AcceptanceError("runtime manifest has no bundle_dir")
    bundle_dir = _canonical_directory(bundle_value, "runtime bundle_dir")
    sensor_plan = _json_object(
        _bound_file(bundle_dir, Path("sensor.plan.json"), "sensor.plan.json")
    )
    _require_schema(sensor_plan, "lingtu.sim.sensor-plan.v1", "sensor plan")
    if sensor_plan.get("session_id") != identity["session_id"]:
        raise AcceptanceError("sensor session_id identity mismatch")
    unreal_log = _text(
        _bound_file(run_root, Path("logs") / "Unreal.log", "Unreal.log")
    )
    findings: list[str] = []
    unreal_lines = unreal_log.splitlines()
    validate_pending = False
    for line in unreal_lines:
        if "ValidatePlatforms started" in line or (
            "Launching UnrealBuildTool" in line and "-Mode=ValidatePlatforms" in line
        ):
            validate_pending = True
        if validate_pending and "UBT AutoSDK ReturnCode" in line:
            validate_pending = False
    if validate_pending:
        findings.append("ValidatePlatforms started but UBT AutoSDK ReturnCode is absent")
    turnkey_pending = False
    for line in unreal_lines:
        if "Turnkey VerifySdk started" in line or (
            "Running Turnkey SDK detection:" in line and "VerifySdk" in line
        ):
            turnkey_pending = True
        if turnkey_pending and "Completed SDK detection" in line:
            turnkey_pending = False
    if turnkey_pending:
        findings.append("Turnkey VerifySdk started but Completed SDK detection is absent")
    try:
        validate_ready_manifest(manifest, sensor_plan)
    except AcceptanceError as exc:
        findings.append(f"Runtime is not READY: {exc}")
    episode_status = episode.get("status")
    if episode_status == "FAILED":
        failure_reason = episode.get("failure_reason")
        if not isinstance(failure_reason, str) or not failure_reason.strip():
            raise AcceptanceError("FAILED episode has no failure_reason")
        findings.append(f"Episode FAILED: {failure_reason}")
    elif episode_status != "SUCCEEDED":
        raise AcceptanceError(f"episode status is unknown: {episode_status!r}")
    return tuple(findings)


def _operation(
    service: SimulationStudioService,
    operation: str,
    record: Mapping[str, Any],
) -> dict[str, Any]:
    revision = record.get("revision")
    run_id = record.get("id")
    if not isinstance(revision, int) or not isinstance(run_id, str):
        raise AcceptanceError("run record has no valid id/revision")
    updated: dict[str, Any] = service.run_operation(operation, run_id, revision=revision)
    print(
        json.dumps(
            {"operation": operation, "run": _run_summary(updated)},
            ensure_ascii=False,
        ),
        flush=True,
    )
    return updated


def _run_summary(record: Mapping[str, Any]) -> dict[str, Any]:
    """Return bounded console evidence without embedding full body snapshots."""

    summary = {
        field: record.get(field)
        for field in ("id", "revision", "status")
    }
    payload = record.get("payload")
    if not isinstance(payload, Mapping):
        return summary
    if isinstance(payload.get("artifact_path"), str):
        summary["artifact_path"] = payload["artifact_path"]
    event = payload.get("runtime_event")
    if isinstance(event, Mapping):
        for field in (
            "event",
            "model_generation",
            "reset_generation",
            "sequence",
        ):
            if field in event:
                summary[field] = event[field]
    for field in ("readiness", "sensor_summary"):
        value = payload.get(field)
        if isinstance(value, Mapping):
            summary[field] = dict(value)
    return summary


def _artifact_dir(studio_root: Path, record: Mapping[str, Any]) -> Path:
    payload = _object(record.get("payload"), "run.payload")
    relative = payload.get("artifact_path")
    if not isinstance(relative, str) or not relative:
        raise AcceptanceError("run payload has no artifact_path")
    candidate = (studio_root / Path(relative)).resolve()
    try:
        candidate.relative_to(studio_root)
    except ValueError as exc:
        raise AcceptanceError("run artifact path escapes the Studio root") from exc
    return candidate


def _write_result(path: Path, document: Mapping[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def run_acceptance(args: argparse.Namespace) -> dict[str, Any]:
    """Execute one fresh run and return its evidence summary."""

    repo_root = args.repo_root.resolve(strict=True)
    studio_root = args.studio_root.resolve(strict=True)
    service = SimulationStudioService.from_repository(repo_root, artifact_root=studio_root)
    bundle = service.get_bundle(args.bundle_id)
    payload = _object(bundle.get("payload"), "bundle.payload")
    bundle_relative = payload.get("bundle_path")
    if not isinstance(bundle_relative, str):
        raise AcceptanceError("bundle payload has no bundle_path")
    bundle_dir = (studio_root / bundle_relative).resolve(strict=True)
    try:
        bundle_dir.relative_to(studio_root)
    except ValueError as exc:
        raise AcceptanceError("bundle path escapes the Studio root") from exc
    sensor_plan = _json_object(bundle_dir / "sensor.plan.json")
    sensor_contract = validate_sensor_contract(sensor_plan)

    record: dict[str, Any] | None = None
    run_dir: Path | None = None
    try:
        record = service.create_run(bundle_id=args.bundle_id, launch_profile="visual")
        run_dir = _artifact_dir(studio_root, record)
        print(
            json.dumps(
                {"operation": "create", "run": _run_summary(record)},
                ensure_ascii=False,
            ),
            flush=True,
        )

        record = _operation(service, "prepare", record)
        ready_manifest = _json_object(run_dir / "session.runtime.json")
        ready_summary = validate_ready_manifest(ready_manifest, sensor_plan)
        initial_sequence = _object(ready_manifest.get("clock"), "runtime.clock").get("sequence")
        if not isinstance(initial_sequence, int):
            raise AcceptanceError("runtime clock has no integer sequence")

        record = _operation(service, "start", record)
        time.sleep(args.first_run_seconds)
        record = _operation(service, "pause", record)
        first_pause = _json_object(run_dir / "session.runtime.json")
        first_sequence = _object(first_pause.get("clock"), "runtime.clock").get("sequence")
        if not isinstance(first_sequence, int) or first_sequence <= initial_sequence:
            raise AcceptanceError("physics sequence did not advance during the first run interval")
        validate_ready_manifest(first_pause, sensor_plan)

        previous_reset = first_pause.get("reset_generation")
        if not isinstance(previous_reset, int):
            raise AcceptanceError("runtime has no reset_generation")
        record = _operation(service, "reset", record)
        after_reset = _json_object(run_dir / "session.runtime.json")
        if after_reset.get("reset_generation") != previous_reset + 1:
            raise AcceptanceError("reset_generation did not increment exactly once")
        reset_summary = validate_ready_manifest(after_reset, sensor_plan)

        record = _operation(service, "start", record)
        time.sleep(args.second_run_seconds)
        record = _operation(service, "pause", record)
        second_pause = _json_object(run_dir / "session.runtime.json")
        second_sequence = _object(second_pause.get("clock"), "runtime.clock").get("sequence")
        if not isinstance(second_sequence, int) or second_sequence <= 0:
            raise AcceptanceError("physics sequence did not advance after reset")
        validate_ready_manifest(second_pause, sensor_plan)

        record = _operation(service, "stop", record)
        episode = _json_object(run_dir / "episode_result.json")
        if episode.get("status") != "SUCCEEDED":
            raise AcceptanceError(f"episode did not succeed: {episode.get('failure_reason')!r}")
        required_evidence = (
            run_dir / "logs" / "visual-readiness.json",
            run_dir / "logs" / "sensor-readiness.json",
            run_dir / "logs" / "visual-first-frame.png",
        )
        missing = [str(path) for path in required_evidence if not path.is_file()]
        if missing:
            raise AcceptanceError("required Unreal evidence is missing: " + ", ".join(missing))

        result = {
            "schema": "lingtu.sim.studio.visual-acceptance.v1",
            "status": "PASSED",
            "run_id": record["id"],
            "bundle_id": args.bundle_id,
            "session_id": payload.get("session_id"),
            "sensor_contract": sensor_contract,
            "ready": ready_summary,
            "after_reset": reset_summary,
            "clock": {
                "initial_sequence": initial_sequence,
                "first_pause_sequence": first_sequence,
                "second_pause_sequence": second_sequence,
            },
            "evidence": [str(path) for path in required_evidence],
            "episode_result": str(run_dir / "episode_result.json"),
        }
        _write_result(run_dir / "simstudio-visual-acceptance.json", result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        return result
    finally:
        if record is not None and record.get("status") != "STOPPED":
            try:
                current = service.get_run(str(record["id"]))
                if current.get("status") != "STOPPED":
                    _operation(service, "stop", current)
            except Exception as cleanup_error:  # pragma: no cover - hardware cleanup path
                print(f"acceptance cleanup failed: {cleanup_error}", file=sys.stderr, flush=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a fresh server-trusted SimStudio MuJoCo + RobotSimUE acceptance",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument("--studio-root", type=Path, required=True)
    parser.add_argument("--bundle-id", required=True)
    parser.add_argument("--first-run-seconds", type=float, default=5.0)
    parser.add_argument("--second-run-seconds", type=float, default=3.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.first_run_seconds <= 0 or args.second_run_seconds <= 0:
        raise SystemExit("run intervals must be positive")
    try:
        run_acceptance(args)
    except Exception as exc:
        print(
            json.dumps(
                {"status": "FAILED", "error": f"{type(exc).__name__}: {exc}"},
                ensure_ascii=False,
            ),
            file=sys.stderr,
            flush=True,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "AcceptanceError",
    "diagnose_offline_startup",
    "main",
    "run_acceptance",
    "validate_ready_manifest",
    "validate_sensor_contract",
]
