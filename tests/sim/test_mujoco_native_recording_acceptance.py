from __future__ import annotations

import json
import shutil
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

import pytest

from sim.scripts.mujoco import native_recording_acceptance as acceptance


def _recording_paths(request) -> tuple[Path, Path, Path]:
    work_dir = Path.cwd() / f".native-recording-test-{uuid4().hex}"
    work_dir.mkdir()
    request.addfinalizer(lambda: shutil.rmtree(work_dir, ignore_errors=True))
    recorder = work_dir / "lingtu_recorder"
    verifier = work_dir / "lingtu_dds_player"
    recorder.write_text("fake recorder", encoding="utf-8")
    verifier.write_text("fake verifier", encoding="utf-8")
    recorder.chmod(0o755)
    verifier.chmod(0o755)
    return work_dir / "native-recording", recorder, verifier


def _write_session(
    session_dir: Path,
    *,
    state: str = "completed",
    mcap_bytes: bytes = b"valid-mcap",
) -> None:
    mcap_path = session_dir / "dds" / "sensors.mcap"
    mcap_path.parent.mkdir(parents=True)
    mcap_path.write_bytes(mcap_bytes)
    manifest = {
        "version": 1,
        "session_id": session_dir.name,
        "state": state,
        "session_directory": str(session_dir),
        "context": {
            "product": "nav",
            "product_session_id": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "robot_id": "sim",
            "task_id": "",
        },
        "ended_at_unix_ns": 123 if state == "completed" else None,
        "children": [
            {
                "name": "dds",
                "required": True,
                "state": "exited",
                "exit_code": 0,
                "artifacts": ["dds/sensors.mcap"],
                "selected_topics": [
                    "/imu/raw",
                    "/lidar/raw_frame",
                    "/slam/odometry",
                    "/slam/registered_cloud",
                ],
                "required_topics": ["/imu/raw", "/lidar/raw_frame"],
            }
        ],
    }
    (session_dir / "session.json").write_text(json.dumps(manifest), encoding="utf-8")


def _run_args(session_dir: Path, recorder: Path, verifier: Path) -> list[str]:
    return [
        "--output-dir",
        str(session_dir),
        "--dds-domain",
        "84",
        "--product",
        "nav",
        "--product-session-id",
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        "--recorder",
        str(recorder),
        "--verifier",
        str(verifier),
        "--seconds",
        "5",
    ]


def _allow_storage(monkeypatch) -> None:
    monkeypatch.setattr(acceptance.sys, "platform", "linux")
    available = acceptance.MINIMUM_FREE_BYTES + 1
    monkeypatch.setattr(
        acceptance.shutil,
        "disk_usage",
        lambda _path: SimpleNamespace(
            free=available,
            total=available * 2,
            used=available,
        ),
    )


def test_dry_run_reports_attach_only_plan_without_native_binaries(
    capsys,
) -> None:
    session_dir = Path("missing-native-recording-session")
    recorder = Path("missing-lingtu-recorder")
    verifier = Path("missing-lingtu-dds-player")

    result = acceptance.main(
        [
            "--output-dir",
            str(session_dir),
            "--dds-domain",
            "84",
            "--product",
            "nav",
            "--product-session-id",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "--recorder",
            str(recorder),
            "--verifier",
            str(verifier),
            "--seconds",
            "2.5",
            "--dry-run",
        ]
    )

    report = json.loads(capsys.readouterr().out)
    assert result == 0
    assert report["ok"] is True
    assert report["mode"] == "dry_run"
    assert report["attach_only"] is True
    assert report["capture"]["required_topics"] == ["/imu/raw", "/lidar/raw_frame"]
    assert report["capture"]["declared_mcap"] == str(session_dir.resolve() / "dds" / "sensors.mcap")
    assert report["commands"]["record"] == [
        str(recorder),
        "record",
        "--output-dir",
        str(session_dir.resolve()),
        "--seconds",
        "2.5",
        "--stop-grace-ms",
        "5000",
        "--product",
        "nav",
        "--product-session-id",
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        "--dds",
        "on",
        "--camera",
        "off",
        "--dds-domain",
        "84",
        "--dds-preset",
        "generic-sensors-v1",
    ]
    assert report["commands"]["catalog"] == [
        str(recorder),
        "list",
        "--root",
        str(session_dir.resolve().parent),
    ]
    assert report["commands"]["verify"] == [
        str(verifier),
        str(session_dir.resolve() / "dds" / "sensors.mcap"),
        "--dry-run",
    ]
    assert report["commands"]["inspect_topics"][-2:] == [
        "--info",
        str(session_dir.resolve() / "dds" / "sensors.mcap"),
    ]
    assert report["preconditions"]["product_runtime"] == "already_running"
    assert report["preconditions"]["native_binaries_required"] is False
    assert report["blockers"] == []


def test_preflight_reports_missing_native_binaries_without_invoking_them(
    monkeypatch,
    capsys,
) -> None:
    session_dir = Path("missing-native-preflight-session")
    recorder = Path("missing-preflight-recorder")
    verifier = Path("missing-preflight-verifier")

    _allow_storage(monkeypatch)
    result = acceptance.main(
        [
            "--output-dir",
            str(session_dir),
            "--dds-domain",
            "84",
            "--product",
            "nav",
            "--product-session-id",
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "--recorder",
            str(recorder),
            "--verifier",
            str(verifier),
            "--preflight-only",
        ]
    )

    report = json.loads(capsys.readouterr().out)
    assert result == 1
    assert report["ok"] is False
    assert report["mode"] == "preflight"
    assert report["checks"]["recorder"] == {
        "ok": False,
        "path": str(recorder),
        "reason": "not_found",
    }
    assert report["checks"]["verifier"] == {
        "ok": False,
        "path": str(verifier),
        "reason": "not_found",
    }
    assert report["checks"]["output_dir_absent"]["ok"] is True
    assert report["blockers"] == ["recorder_not_found", "verifier_not_found"]


def test_run_accepts_completed_nonempty_mcap_with_required_topics(
    request,
    monkeypatch,
    capsys,
) -> None:
    session_dir, recorder, verifier = _recording_paths(request)
    _allow_storage(monkeypatch)
    observed: list[list[str]] = []

    def fake_run(command, **kwargs):
        command = [str(value) for value in command]
        observed.append(command)
        assert kwargs["capture_output"] is True
        assert kwargs["text"] is True
        assert kwargs["check"] is False
        assert kwargs["timeout"] > 5.0
        if command[0] == str(recorder) and command[1] == "record":
            _write_session(session_dir)
            return SimpleNamespace(returncode=0, stdout="state=completed\n", stderr="")
        if command[-1] == "--dry-run":
            return SimpleNamespace(
                returncode=0,
                stdout="validated=9 domain=84 rate=1 skipped_record_only=0\n",
                stderr="",
            )
        if command[1] == "list":
            catalog = {
                "control_version": 1,
                "ok": True,
                "root": str(session_dir.parent.resolve()),
                "sessions": [
                    {
                        "session_id": session_dir.name,
                        "state": "completed",
                        "manager_pid": 123,
                        "session_directory": str(session_dir.resolve()),
                    }
                ],
                "truncated": False,
                "disk_free": acceptance.MINIMUM_FREE_BYTES + 1,
                "disk_total": acceptance.MINIMUM_FREE_BYTES * 2,
            }
            return SimpleNamespace(
                returncode=0,
                stdout=json.dumps(catalog) + "\n",
                stderr="",
            )
        assert command[1] == "--info"
        return SimpleNamespace(
            returncode=0,
            stdout=(
                "profile=lingtu.dds.v1 messages=9 duration_ns=100\n"
                "topic=/imu/raw dds_topic=rt/imu/raw count=6 "
                "idl_type=lingtu.dds.Imu policy=replayable\n"
                "topic=/lidar/raw_frame dds_topic=rt/lidar/raw_frame count=3 "
                "idl_type=lingtu.dds.LivoxFrame policy=replayable\n"
            ),
            stderr="",
        )

    monkeypatch.setattr(acceptance.subprocess, "run", fake_run)

    result = acceptance.main(_run_args(session_dir, recorder, verifier))

    report = json.loads(capsys.readouterr().out)
    assert result == 0
    assert report["ok"] is True
    assert report["blockers"] == []
    assert report["checks"]["terminal_session"]["state"] == "completed"
    assert report["checks"]["declared_mcap"]["bytes"] == len(b"valid-mcap")
    assert report["checks"]["verifier"]["validated_messages"] == 9
    assert report["checks"]["required_topics"]["counts"] == {
        "/imu/raw": 6,
        "/lidar/raw_frame": 3,
    }
    assert report["checks"]["catalog_session"]["session_id"] == session_dir.name
    assert report["checks"]["catalog_session"]["state"] == "completed"
    assert report["checks"]["catalog_session"]["directory_matches"] is True
    assert observed == [
        report["commands"]["record"],
        report["commands"]["catalog"],
        report["commands"]["verify"],
        report["commands"]["inspect_topics"],
    ]


def test_preflight_enforces_native_default_free_space(
    request,
    monkeypatch,
    capsys,
) -> None:
    session_dir, recorder, verifier = _recording_paths(request)
    available = acceptance.MINIMUM_FREE_BYTES - 1
    observed: list[Path] = []

    def fake_disk_usage(path):
        observed.append(Path(path))
        return SimpleNamespace(
            free=available,
            total=acceptance.MINIMUM_FREE_BYTES * 2,
            used=acceptance.MINIMUM_FREE_BYTES + 1,
        )

    monkeypatch.setattr(acceptance.sys, "platform", "linux")
    monkeypatch.setattr(acceptance.shutil, "disk_usage", fake_disk_usage)

    result = acceptance.main([*_run_args(session_dir, recorder, verifier), "--preflight-only"])

    report = json.loads(capsys.readouterr().out)
    assert result == 1
    assert observed == [session_dir.parent.resolve()]
    assert report["checks"]["storage"] == {
        "ok": False,
        "path": str(session_dir.parent.resolve()),
        "minimum_free_bytes": acceptance.MINIMUM_FREE_BYTES,
        "available_bytes": available,
    }
    assert report["blockers"] == ["insufficient_free_space"]


def test_preflight_rejects_non_linux_host(
    request,
    monkeypatch,
    capsys,
) -> None:
    session_dir, recorder, verifier = _recording_paths(request)
    _allow_storage(monkeypatch)
    monkeypatch.setattr(acceptance.sys, "platform", "win32")

    result = acceptance.main(
        [*_run_args(session_dir, recorder, verifier), "--preflight-only"]
    )

    report = json.loads(capsys.readouterr().out)
    assert result == 1
    assert report["checks"]["host_platform"] == {
        "ok": False,
        "actual": "win32",
        "required": "linux",
    }
    assert report["blockers"] == ["unsupported_platform"]


def test_help_states_attach_only_runtime_preconditions(capsys) -> None:
    with pytest.raises(SystemExit) as exit_info:
        acceptance.main(["--help"])

    help_text = capsys.readouterr().out
    assert exit_info.value.code == 0
    assert "already running" in help_text
    assert "--dds-domain" in help_text
    assert "/imu/raw and /lidar/raw_frame" in help_text
    assert "does not exist" in help_text
    assert "at least 5 GiB free" in help_text
    assert "needs no native binaries or DDS" in help_text


@pytest.mark.parametrize(
    ("case", "expected_blocker"),
    [
        ("nonterminal", "session_not_completed"),
        ("empty_mcap", "declared_mcap_empty"),
        ("missing_topic", "required_topics_missing"),
        ("catalog_wrong_directory", "catalog_session_not_completed"),
    ],
)
def test_run_fails_closed_on_incomplete_recording_evidence(
    case: str,
    expected_blocker: str,
    request,
    monkeypatch,
    capsys,
) -> None:
    session_dir, recorder, verifier = _recording_paths(request)
    _allow_storage(monkeypatch)

    def fake_run(command, **_kwargs):
        command = [str(value) for value in command]
        if command[1] == "record":
            _write_session(
                session_dir,
                state="recording" if case == "nonterminal" else "completed",
                mcap_bytes=b"" if case == "empty_mcap" else b"valid-mcap",
            )
            return SimpleNamespace(returncode=0, stdout="recorded\n", stderr="")
        if command[1] == "list":
            catalog_state = "recording" if case == "nonterminal" else "completed"
            payload = {
                "control_version": 1,
                "ok": True,
                "sessions": [
                    {
                        "session_id": session_dir.name,
                        "state": catalog_state,
                        "session_directory": str(
                            (session_dir.parent / "different-session").resolve()
                            if case == "catalog_wrong_directory"
                            else session_dir.resolve()
                        ),
                    }
                ],
            }
            return SimpleNamespace(
                returncode=0,
                stdout=json.dumps(payload),
                stderr="",
            )
        if command[-1] == "--dry-run":
            return SimpleNamespace(
                returncode=0,
                stdout="validated=3 domain=84 rate=1 skipped_record_only=0\n",
                stderr="",
            )
        info = "topic=/imu/raw dds_topic=rt/imu/raw count=3 idl_type=Imu policy=replayable\n"
        if case != "missing_topic":
            info += (
                "topic=/lidar/raw_frame dds_topic=rt/lidar/raw_frame count=1 idl_type=LivoxFrame policy=replayable\n"
            )
        return SimpleNamespace(returncode=0, stdout=info, stderr="")

    monkeypatch.setattr(acceptance.subprocess, "run", fake_run)

    result = acceptance.main(_run_args(session_dir, recorder, verifier))

    report = json.loads(capsys.readouterr().out)
    assert result == 1
    assert report["ok"] is False
    assert expected_blocker in report["blockers"]
