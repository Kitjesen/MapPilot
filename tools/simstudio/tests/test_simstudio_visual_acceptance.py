"""Contract tests for the production SimStudio visual acceptance runner."""

# ruff: noqa: D103,S101

from __future__ import annotations

import json
from pathlib import Path

import pytest
from scripts.sim.run_simstudio_visual_acceptance import (
    AcceptanceError,
    _run_summary,
    diagnose_offline_startup,
    validate_ready_manifest,
    validate_sensor_contract,
)


def _sensor_plan() -> dict:
    return {
        "streams": {
            "rgb": [
                {
                    "sensor_id": "thunder_01.front_rgb",
                    "width": 640,
                    "height": 480,
                    "rate_hz": 30,
                }
            ],
            "depth": [
                {
                    "sensor_id": "thunder_01.front_depth",
                    "width": 640,
                    "height": 480,
                    "rate_hz": 30,
                }
            ],
            "mid360": [{"sensor_id": "thunder_01.mid360", "rate_hz": 10}],
            "imu": [{"sensor_id": "thunder_01.imu", "rate_hz": 200}],
            "truth_odom": [
                {
                    "sensor_id": "thunder_01.truth_odom",
                    "rate_hz": 100,
                    "estimator_input": False,
                }
            ],
        }
    }


def _ready_manifest() -> dict:
    stream_ids = {
        "thunder_01.front_rgb",
        "thunder_01.front_depth",
        "thunder_01.mid360",
        "thunder_01.imu",
        "thunder_01.truth_odom",
    }
    return {
        "schema": "lingtu.sim.session-runtime.v1",
        "state": "READY",
        "model_generation": 0,
        "reset_generation": 0,
        "bindings": {
            facet: {"required": True, "state": "ACTIVE"}
            for facet in ("physics", "visual", "sensors", "control")
        },
        "sensor_streams": {
            "is_ready": True,
            "required_stream_ids": sorted(stream_ids),
            "streams": {
                sensor_id: {"required": True, "state": "ACTIVE"}
                for sensor_id in stream_ids
            },
        },
    }


def _write_json(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")


def _offline_run(tmp_path: Path, unreal_log: str) -> Path:
    run_dir = tmp_path / "run01"
    bundle_dir = tmp_path / "bundle01"
    session_id = "session-alpha"
    sensor_plan = {
        **_sensor_plan(),
        "schema": "lingtu.sim.sensor-plan.v1",
        "session_id": session_id,
    }
    manifest = {
        **_ready_manifest(),
        "run_id": "run01",
        "session_id": session_id,
        "bundle_dir": str(bundle_dir.resolve()),
        "allocation": {
            "run_dir": str(run_dir.resolve()),
            "log_dir": str((run_dir / "logs").resolve()),
        },
    }
    episode = {
        "schema": "lingtu.sim.episode-result.v1",
        "run_id": "run01",
        "session_id": session_id,
        "model_generation": 0,
        "reset_generation": 0,
        "status": "SUCCEEDED",
        "failure_reason": None,
        "artifact_references": {"runtime_manifest": "session.runtime.json"},
    }
    _write_json(bundle_dir / "sensor.plan.json", sensor_plan)
    _write_json(run_dir / "session.runtime.json", manifest)
    _write_json(run_dir / "episode_result.json", episode)
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)
    (run_dir / "logs" / "Unreal.log").write_text(unreal_log, encoding="utf-8")
    return run_dir


def test_offline_startup_diagnoses_incomplete_validate_platforms(tmp_path: Path) -> None:
    run_dir = _offline_run(
        tmp_path,
        "LogTurnkeySupport: Display: ValidatePlatforms started\n",
    )

    assert diagnose_offline_startup(run_dir) == (
        "ValidatePlatforms started but UBT AutoSDK ReturnCode is absent",
    )


def test_offline_startup_ignores_a_validate_platforms_mention_without_a_start(
    tmp_path: Path,
) -> None:
    run_dir = _offline_run(
        tmp_path,
        "LogConfig: Runtime profile skips ValidatePlatforms checks\n",
    )

    assert diagnose_offline_startup(run_dir) == ()


def test_offline_startup_uses_the_latest_validate_platforms_attempt(
    tmp_path: Path,
) -> None:
    run_dir = _offline_run(
        tmp_path,
        "Launching UnrealBuildTool... [-Mode=ValidatePlatforms]\n"
        "LogTargetPlatformManager: UBT AutoSDK ReturnCode: 0\n"
        "Launching UnrealBuildTool... [-Mode=ValidatePlatforms]\n",
    )

    assert diagnose_offline_startup(run_dir) == (
        "ValidatePlatforms started but UBT AutoSDK ReturnCode is absent",
    )


def test_offline_startup_diagnoses_incomplete_turnkey_sdk_detection(tmp_path: Path) -> None:
    run_dir = _offline_run(
        tmp_path,
        "LogTurnkeySupport: Running Turnkey SDK detection: "
        "'Turnkey -command=VerifySdk -platform=all'\n",
    )

    assert diagnose_offline_startup(run_dir) == (
        "Turnkey VerifySdk started but Completed SDK detection is absent",
    )


def test_offline_startup_does_not_combine_unrelated_turnkey_and_verifysdk_lines(
    tmp_path: Path,
) -> None:
    run_dir = _offline_run(
        tmp_path,
        "LogInit: Turnkey support enabled\n"
        "LogConfig: VerifySdk policy loaded\n",
    )

    assert diagnose_offline_startup(run_dir) == ()


def test_offline_startup_uses_the_latest_turnkey_verifysdk_attempt(
    tmp_path: Path,
) -> None:
    turnkey_start = (
        "LogTurnkeySupport: Running Turnkey SDK detection: "
        "'Turnkey -command=VerifySdk -platform=all'\n"
    )
    run_dir = _offline_run(
        tmp_path,
        turnkey_start
        + "LogTurnkeySupport: Completed SDK detection: ExitCode=0\n"
        + turnkey_start,
    )

    assert diagnose_offline_startup(run_dir) == (
        "Turnkey VerifySdk started but Completed SDK detection is absent",
    )


def test_offline_startup_summarizes_the_first_not_ready_runtime_reason(tmp_path: Path) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    manifest_path = run_dir / "session.runtime.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["bindings"]["visual"].update(
        state="UNBOUND",
        failure_reason="RobotSimUE never reported ready",
    )
    _write_json(manifest_path, manifest)

    assert diagnose_offline_startup(run_dir) == (
        "Runtime is not READY: required visual binding is 'UNBOUND': "
        "RobotSimUE never reported ready",
    )


def test_offline_startup_includes_the_terminal_episode_failure(tmp_path: Path) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    episode_path = run_dir / "episode_result.json"
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    episode.update(status="FAILED", failure_reason="visual readiness timed out")
    _write_json(episode_path, episode)

    assert diagnose_offline_startup(run_dir) == (
        "Episode FAILED: visual readiness timed out",
    )


@pytest.mark.parametrize(
    "relative_path",
    (
        Path("logs/Unreal.log"),
        Path("session.runtime.json"),
        Path("episode_result.json"),
        Path("../bundle01/sensor.plan.json"),
    ),
)
def test_offline_startup_fails_closed_when_required_evidence_is_missing(
    tmp_path: Path,
    relative_path: Path,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    (run_dir / relative_path).unlink()

    with pytest.raises(AcceptanceError, match="cannot read"):
        diagnose_offline_startup(run_dir)


@pytest.mark.parametrize("field", ("run_dir", "log_dir"))
def test_offline_startup_rejects_foreign_runtime_allocation_paths(
    tmp_path: Path,
    field: str,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    manifest_path = run_dir / "session.runtime.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["allocation"][field] = str(foreign.resolve())
    _write_json(manifest_path, manifest)

    with pytest.raises(AcceptanceError, match=rf"allocation {field} is not bound"):
        diagnose_offline_startup(run_dir)


@pytest.mark.parametrize(
    "bundle_path_kind",
    ("relative", "missing", "noncanonical", "file"),
)
def test_offline_startup_rejects_invalid_bundle_paths(
    tmp_path: Path,
    bundle_path_kind: str,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    manifest_path = run_dir / "session.runtime.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if bundle_path_kind == "relative":
        bundle_dir = "bundle01"
    elif bundle_path_kind == "missing":
        bundle_dir = str((tmp_path / "missing").resolve())
    elif bundle_path_kind == "noncanonical":
        (tmp_path / "detour").mkdir()
        bundle_dir = str(tmp_path / "detour" / ".." / "bundle01")
    else:
        bundle_dir = str((run_dir / "episode_result.json").resolve())
    manifest["bundle_dir"] = bundle_dir
    _write_json(manifest_path, manifest)

    with pytest.raises(AcceptanceError, match="runtime bundle_dir"):
        diagnose_offline_startup(run_dir)


@pytest.mark.parametrize(
    ("artifact", "relative_path"),
    (
        ("runtime", Path("run01/session.runtime.json")),
        ("episode", Path("run01/episode_result.json")),
        ("sensor plan", Path("bundle01/sensor.plan.json")),
    ),
)
def test_offline_startup_rejects_unknown_artifact_schemas(
    tmp_path: Path,
    artifact: str,
    relative_path: Path,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    artifact_path = tmp_path / relative_path
    document = json.loads(artifact_path.read_text(encoding="utf-8"))
    document["schema"] = "unknown"
    _write_json(artifact_path, document)

    with pytest.raises(AcceptanceError, match=rf"{artifact} schema"):
        diagnose_offline_startup(run_dir)


@pytest.mark.parametrize(
    ("artifact", "field", "value"),
    (
        ("episode", "run_id", "other-run"),
        ("episode", "session_id", "session-beta"),
        ("episode", "model_generation", 1),
        ("episode", "reset_generation", 1),
        ("sensor", "session_id", "session-beta"),
    ),
)
def test_offline_startup_rejects_cross_artifact_identity_mismatches(
    tmp_path: Path,
    artifact: str,
    field: str,
    value: object,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    artifact_path = (
        run_dir / "episode_result.json"
        if artifact == "episode"
        else tmp_path / "bundle01" / "sensor.plan.json"
    )
    document = json.loads(artifact_path.read_text(encoding="utf-8"))
    document[field] = value
    _write_json(artifact_path, document)

    with pytest.raises(AcceptanceError, match=rf"{artifact} {field} identity mismatch"):
        diagnose_offline_startup(run_dir)


@pytest.mark.parametrize(
    "reference",
    (None, "runtime-copy.json", "../session.runtime.json"),
)
def test_offline_startup_rejects_a_foreign_episode_runtime_reference(
    tmp_path: Path,
    reference: str | None,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    episode_path = run_dir / "episode_result.json"
    episode = json.loads(episode_path.read_text(encoding="utf-8"))
    episode["artifact_references"]["runtime_manifest"] = reference
    _write_json(episode_path, episode)

    with pytest.raises(AcceptanceError, match="episode runtime_manifest reference"):
        diagnose_offline_startup(run_dir)


@pytest.mark.parametrize("path_kind", ("missing", "file"))
def test_offline_startup_rejects_an_invalid_run_directory(
    tmp_path: Path,
    path_kind: str,
) -> None:
    run_dir = tmp_path / path_kind
    if path_kind == "file":
        run_dir.write_text("not a run directory", encoding="utf-8")

    with pytest.raises(AcceptanceError, match="run_dir"):
        diagnose_offline_startup(run_dir)


def test_offline_startup_rejects_run_artifact_links_that_escape(
    tmp_path: Path,
) -> None:
    run_dir = _offline_run(tmp_path, "LogInit: Display: startup pending\n")
    foreign_log = tmp_path / "foreign.log"
    foreign_log.write_text("foreign evidence", encoding="utf-8")
    unreal_log = run_dir / "logs" / "Unreal.log"
    unreal_log.unlink()
    try:
        unreal_log.symlink_to(foreign_log)
    except OSError as exc:  # pragma: no cover - Windows without symlink capability
        pytest.skip(f"symlinks unavailable: {exc}")

    with pytest.raises(AcceptanceError, match=r"Unreal\.log is not run-bound"):
        diagnose_offline_startup(run_dir)


def test_visual_acceptance_requires_every_runtime_facet_and_sensor_stream() -> None:
    summary = validate_ready_manifest(_ready_manifest(), _sensor_plan())

    assert summary["bindings"] == {
        "control": "ACTIVE",
        "physics": "ACTIVE",
        "sensors": "ACTIVE",
        "visual": "ACTIVE",
    }
    assert len(summary["sensors"]) == 5

    failed = _ready_manifest()
    failed["bindings"]["visual"]["state"] = "UNBOUND"
    with pytest.raises(AcceptanceError, match="visual"):
        validate_ready_manifest(failed, _sensor_plan())

    failed = _ready_manifest()
    failed["sensor_streams"]["streams"]["thunder_01.mid360"]["state"] = "FAILED"
    with pytest.raises(AcceptanceError, match=r"thunder_01\.mid360"):
        validate_ready_manifest(failed, _sensor_plan())


def test_visual_acceptance_locks_the_navigation_sensor_contract() -> None:
    summary = validate_sensor_contract(_sensor_plan())

    assert summary == {
        "depth": "640x480@30Hz",
        "imu": "200Hz",
        "mid360": "10Hz",
        "rgb": "640x480@30Hz",
        "truth_odom": "100Hz (truth-only)",
    }

    invalid = _sensor_plan()
    invalid["streams"]["depth"][0]["rate_hz"] = 10
    with pytest.raises(AcceptanceError, match="depth"):
        validate_sensor_contract(invalid)

    invalid = _sensor_plan()
    invalid["streams"]["truth_odom"][0]["estimator_input"] = True
    with pytest.raises(AcceptanceError, match="truth_odom"):
        validate_sensor_contract(invalid)


def test_visual_acceptance_console_summary_omits_large_snapshot_arrays() -> None:
    record = {
        "id": "run01",
        "revision": 7,
        "status": "READY",
        "payload": {
            "artifact_path": "artifacts/runs/run01",
            "readiness": {"physics": "ACTIVE"},
            "sensor_summary": {"robot.imu": "ACTIVE"},
            "runtime_event": {
                "event": "snapshot",
                "model_generation": 0,
                "reset_generation": 1,
                "sequence": 0,
                "bodies": [{"stable_id": "robot/base"}],
                "joints": [{"stable_id": "robot/joint"}],
            },
        },
    }

    summary = _run_summary(record)

    assert summary == {
        "id": "run01",
        "revision": 7,
        "status": "READY",
        "artifact_path": "artifacts/runs/run01",
        "event": "snapshot",
        "model_generation": 0,
        "reset_generation": 1,
        "sequence": 0,
        "readiness": {"physics": "ACTIVE"},
        "sensor_summary": {"robot.imu": "ACTIVE"},
    }
    assert "bodies" not in summary
