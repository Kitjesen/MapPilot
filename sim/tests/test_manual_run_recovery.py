"""Contracts for explicit abandoned manual-run recovery."""

# ruff: noqa: S101

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

import sim.runtime.coordinator.manual_run_recovery as recovery_module
from sim.runtime.coordinator.manual_run_recovery import (
    ManualRunRecoveryError,
    ManualRunRecoveryProbes,
    recover_manual_abandoned_run,
    recover_manual_abandoned_startup,
)
from sim.runtime.coordinator.run_allocation import create_run_allocation


def _write_bundle(root: Path) -> tuple[Path, str]:
    bundle = root / "bundle"
    bundle.mkdir(parents=True)
    session_id = "manual-recovery"
    session = {
        "schema": "lingtu.sim.session.v1",
        "session_id": session_id,
        "seed": 11,
        "world": {"package": "test.world@1.0.0"},
        "robots": [{"instance_id": "thunder_01"}],
    }
    (bundle / "session.yaml").write_text(
        json.dumps(session),
        encoding="utf-8",
    )
    for filename, schema in {
        "physics.plan.json": "lingtu.sim.physics-plan.v1",
        "visual.plan.json": "lingtu.sim.visual-plan.v1",
        "sensor.plan.json": "lingtu.sim.sensor-plan.v1",
        "control.plan.json": "lingtu.sim.control-plan.v1",
        "transport.intent.json": "lingtu.sim.transport-intent.v1",
    }.items():
        document: dict[str, object] = {
            "schema": schema,
            "session_id": session_id,
        }
        if filename == "sensor.plan.json":
            document["streams"] = {}
        if filename == "transport.intent.json":
            document["allocation_boundary"] = {
                "owner": "RunAllocation",
                "runtime_values_external": True,
            }
        (bundle / filename).write_text(json.dumps(document), encoding="utf-8")
    return bundle, session_id


def _write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )


def _abandoned_manual_run(tmp_path: Path, *, state: str = "RUNNING") -> tuple[Path, str]:
    bundle, session_id = _write_bundle(tmp_path)
    run_root = tmp_path / "runs"
    allocation = create_run_allocation(
        bundle,
        run_root,
        run_id="manual-20260811-175346-dfb260b8",
        boot_id="manual-boot",
        repo_root=tmp_path,
        dds_domain=42,
        ports={"visual_snapshot_udp": 25001, "control_intent_udp": 25002},
        shm={"thunder_01.front_rgb": f"lingtu.sim.camera_shm.manual.{os.getpid()}.rgb"},
    )
    runtime = {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": allocation.run_id,
        "session_id": session_id,
        "model_generation": 0,
        "reset_generation": 0,
        "state": state,
        "bindings": {},
        "bundle_dir": str(bundle.resolve()),
        "allocation": {
            "run_dir": str(allocation.run_dir),
            "log_dir": str(allocation.log_dir),
            "boot_id": allocation.boot_id,
            "physics_pid": 987654,
            "dds_domain": allocation.dds_domain,
            "ports": dict(allocation.ports),
            "shm": dict(allocation.shm),
            "shared_memory": dict(allocation.shm),
        },
        "clock": {"sequence": 17, "physics_step": 100, "sim_time_ns": 20_000_000},
    }
    _write_json(allocation.run_dir / "session.runtime.json", runtime)
    _write_json(
        allocation.run_dir / "manual-runtime-health.json",
        {
            "schema": "lingtu.sim.manual-runtime-health.v1",
            "qualification": False,
            "evidence_class": "manual_diagnostic_only",
            "run_id": allocation.run_id,
            "session_id": session_id,
            "session_state": state,
            "owner_thread_alive": False,
            "monitor_sequence": 4,
            "event_sequence": 3,
            "non_running_transition_count": 0,
            "owner_thread_stop_count": 1,
            "model_generation": 0,
            "reset_generation": 0,
            "last_truth_sequence": 17,
            "updated_unix_ns": 1_000_000_000_000,
        },
    )
    (allocation.run_dir / "truth-snapshots.jsonl").write_text(
        json.dumps({"event": "snapshot", "sequence": 17}) + "\n",
        encoding="utf-8",
    )
    return allocation.run_dir, session_id


def _clean_probes() -> ManualRunRecoveryProbes:
    return ManualRunRecoveryProbes(
        process_exists=lambda pid: False,
        known_processes_clean=lambda: {"clean": True, "processes": []},
        udp_ports_available=lambda ports: {"available": True, "ports": list(ports)},
        camera_shm_available=lambda names: {"available": True, "names": list(names)},
        sleep=lambda _seconds: None,
        now_ns=lambda: 2_000_000_000_000,
    )


def test_recovery_terminalizes_one_explicit_abandoned_manual_run_and_releases_resources(
    tmp_path: Path,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path)
    original_runtime = (run_dir / "session.runtime.json").read_bytes()

    result = recover_manual_abandoned_run(
        run_dir=run_dir,
        expected_run_id=run_dir.name,
        expected_session_id=session_id,
        expected_dds_domain=42,
        stale_after_s=1.0,
        probes=_clean_probes(),
    )

    assert result.recovered is True
    recovery = json.loads((run_dir / "stale-allocation-recovery.json").read_text(encoding="utf-8"))
    assert recovery["schema"] == "lingtu.sim.stale-allocation-recovery.v1"
    assert recovery["manual_diagnostic_only"] is True
    assert recovery["qualification"] is False
    assert recovery["run_id"] == run_dir.name
    assert recovery["session_id"] == session_id
    assert recovery["dds_domain"] == 42
    assert recovery["recovered_from_state"] == "RUNNING"
    assert recovery["original_runtime_manifest"]["bytes"] == len(original_runtime)
    assert recovery["observed_dead_pids"] == {"physics_pid": 987654}

    episode = json.loads((run_dir / "episode_result.json").read_text(encoding="utf-8"))
    assert episode["schema"] == "lingtu.sim.episode-result.v1"
    assert episode["status"] == "FAILED"
    assert "manual abandoned run recovery" in episode["failure_reason"]
    assert episode["artifact_references"]["stale_allocation_recovery"] == "stale-allocation-recovery.json"

    runtime = json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))
    assert runtime["state"] == "FAILED"
    assert runtime["allocation"]["dds_domain"] == 42
    assert runtime["clock"]["sequence"] == 17
    assert not (run_dir / "shutdown-evidence.json").exists()

    bundle = tmp_path / "bundle"
    replacement = create_run_allocation(
        bundle,
        tmp_path / "runs",
        run_id="manual-20260811-183431-112638ea",
        boot_id="manual-boot",
        repo_root=tmp_path,
        dds_domain=42,
        ports={"visual_snapshot_udp": 25001},
    )
    assert replacement.dds_domain == 42


def test_startup_recovery_terminalizes_preparing_run_before_journals_exist(
    tmp_path: Path,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path, state="PREPARING")
    (run_dir / "manual-runtime-health.json").unlink()
    (run_dir / "truth-snapshots.jsonl").unlink()
    runtime_path = run_dir / "session.runtime.json"
    os.utime(runtime_path, ns=(1_000_000_000_000, 1_000_000_000_000))

    result = recover_manual_abandoned_startup(
        run_dir=run_dir,
        expected_run_id=run_dir.name,
        expected_session_id=session_id,
        expected_dds_domain=42,
        stale_after_s=1.0,
        probes=_clean_probes(),
    )

    assert result.recovered is True
    recovery = json.loads(
        (run_dir / "stale-allocation-recovery.json").read_text(encoding="utf-8")
    )
    assert recovery["recovered_from_state"] == "PREPARING"
    assert recovery["startup_journals_missing"] == [
        "manual-runtime-health.json",
        "truth-snapshots.jsonl",
    ]
    assert set(recovery["stable_files"]) == {
        "run-allocation.json",
        "session.runtime.json",
    }
    runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
    assert runtime["state"] == "FAILED"
    assert not (run_dir / "shutdown-evidence.json").exists()


@pytest.mark.parametrize("state", ["READY", "RUNNING"])
def test_startup_recovery_rejects_missing_journals_after_preparing(
    tmp_path: Path,
    state: str,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path, state=state)
    (run_dir / "manual-runtime-health.json").unlink()
    (run_dir / "truth-snapshots.jsonl").unlink()

    with pytest.raises(ManualRunRecoveryError, match="PREPARING"):
        recover_manual_abandoned_startup(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=_clean_probes(),
        )


def test_startup_recovery_rejects_partially_missing_journals(tmp_path: Path) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path, state="PREPARING")
    (run_dir / "manual-runtime-health.json").unlink()

    with pytest.raises(ManualRunRecoveryError, match="both be absent"):
        recover_manual_abandoned_startup(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=_clean_probes(),
        )


def test_recovery_is_idempotent_after_partial_crash_before_manifest_write(
    tmp_path: Path,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path)
    first = recover_manual_abandoned_run(
        run_dir=run_dir,
        expected_run_id=run_dir.name,
        expected_session_id=session_id,
        expected_dds_domain=42,
        stale_after_s=1.0,
        probes=_clean_probes(),
        stop_before_manifest_replace_for_test=True,
    )
    assert first.recovered is False
    assert json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))["state"] == "RUNNING"
    recovery_before = (run_dir / "stale-allocation-recovery.json").read_bytes()
    episode_before = (run_dir / "episode_result.json").read_bytes()

    second = recover_manual_abandoned_run(
        run_dir=run_dir,
        expected_run_id=run_dir.name,
        expected_session_id=session_id,
        expected_dds_domain=42,
        stale_after_s=1.0,
        probes=_clean_probes(),
    )

    assert second.recovered is True
    assert (run_dir / "stale-allocation-recovery.json").read_bytes() == recovery_before
    assert (run_dir / "episode_result.json").read_bytes() == episode_before
    assert json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))["state"] == "FAILED"


def test_recovery_rejects_live_pid_busy_ports_busy_shm_and_identity_mismatch(
    tmp_path: Path,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path)
    with pytest.raises(ManualRunRecoveryError, match="physics_pid is still alive"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=ManualRunRecoveryProbes(
                process_exists=lambda pid: pid == 987654,
                known_processes_clean=lambda: {"clean": True, "processes": []},
                udp_ports_available=lambda ports: {"available": True, "ports": list(ports)},
                camera_shm_available=lambda names: {"available": True, "names": list(names)},
                sleep=lambda _seconds: None,
                now_ns=lambda: 2_000_000_000_000,
            ),
        )

    with pytest.raises(ManualRunRecoveryError, match="UDP ports are still busy"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=ManualRunRecoveryProbes(
                process_exists=lambda pid: False,
                known_processes_clean=lambda: {"clean": True, "processes": []},
                udp_ports_available=lambda ports: {"available": False, "ports": list(ports), "busy": [25001]},
                camera_shm_available=lambda names: {"available": True, "names": list(names)},
                sleep=lambda _seconds: None,
                now_ns=lambda: 2_000_000_000_000,
            ),
        )

    with pytest.raises(ManualRunRecoveryError, match="camera SHM names are still busy"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=ManualRunRecoveryProbes(
                process_exists=lambda pid: False,
                known_processes_clean=lambda: {"clean": True, "processes": []},
                udp_ports_available=lambda ports: {"available": True, "ports": list(ports)},
                camera_shm_available=lambda names: {"available": False, "names": list(names), "busy": list(names)},
                sleep=lambda _seconds: None,
                now_ns=lambda: 2_000_000_000_000,
            ),
        )

    with pytest.raises(ManualRunRecoveryError, match="expected session_id mismatch"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id="0" * 64,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=_clean_probes(),
        )


def test_recovery_rejects_non_manual_terminal_and_existing_artifact_conflicts(
    tmp_path: Path,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path)
    with pytest.raises(ManualRunRecoveryError, match="manual-"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id="run-not-manual",
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=_clean_probes(),
        )

    terminal_dir, terminal_digest = _abandoned_manual_run(tmp_path / "terminal", state="FAILED")
    terminal_health_path = terminal_dir / "manual-runtime-health.json"
    terminal_health = json.loads(terminal_health_path.read_text(encoding="utf-8"))
    terminal_health["session_state"] = "RUNNING"
    _write_json(terminal_health_path, terminal_health)
    before = (terminal_dir / "session.runtime.json").read_bytes()
    result = recover_manual_abandoned_run(
        run_dir=terminal_dir,
        expected_run_id=terminal_dir.name,
        expected_session_id=terminal_digest,
        expected_dds_domain=42,
        stale_after_s=1.0,
        probes=_clean_probes(),
    )
    assert result.recovered is False
    assert result.already_terminal is True
    assert (terminal_dir / "session.runtime.json").read_bytes() == before

    (run_dir / "stale-allocation-recovery.json").write_text('{"schema":"foreign"}\n', encoding="utf-8")
    with pytest.raises(ManualRunRecoveryError, match="existing stale-allocation-recovery"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=_clean_probes(),
        )


def test_recovery_rejects_unstable_or_fresh_health_files(
    tmp_path: Path,
) -> None:
    run_dir, session_id = _abandoned_manual_run(tmp_path)
    with pytest.raises(ManualRunRecoveryError, match="manual runtime health is not stale"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1200.0,
            probes=_clean_probes(),
        )

    probes = _clean_probes()

    def mutate(_seconds: float) -> None:
        (run_dir / "truth-snapshots.jsonl").write_text("changed\n", encoding="utf-8")

    unstable = ManualRunRecoveryProbes(
        process_exists=probes.process_exists,
        known_processes_clean=probes.known_processes_clean,
        udp_ports_available=probes.udp_ports_available,
        camera_shm_available=probes.camera_shm_available,
        sleep=mutate,
        now_ns=probes.now_ns,
    )
    with pytest.raises(ManualRunRecoveryError, match="changed during recovery inspection"):
        recover_manual_abandoned_run(
            run_dir=run_dir,
            expected_run_id=run_dir.name,
            expected_session_id=session_id,
            expected_dds_domain=42,
            stale_after_s=1.0,
            probes=unstable,
        )


def test_windows_default_process_probe_is_case_insensitive_and_ignores_python(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(recovery_module.os, "name", "nt")
    monkeypatch.setattr(
        recovery_module,
        "_windows_toolhelp_processes",
        lambda: [
            {"name": "python.exe", "pid": 10},
            {"name": "robotsimue-win64-shipping.exe", "pid": 20},
            {"name": "LINGTU_MUJOCO_DRIVER_BRIDGE.EXE", "pid": 21},
        ],
    )
    monkeypatch.setattr(
        recovery_module,
        "_windows_process_image_path",
        lambda pid: f"C:/runtime/{pid}.exe",
    )

    probe = recovery_module._known_processes_clean()

    assert probe["clean"] is False
    assert [process["pid"] for process in probe["processes"]] == [20, 21]


def test_windows_probe_helpers_fail_closed_on_access_denied_or_unknown_errors() -> None:
    assert recovery_module._windows_open_process_exists_result(0, 87, pid=123) is False
    with pytest.raises(ManualRunRecoveryError, match="access denied"):
        recovery_module._windows_open_process_exists_result(0, 5, pid=123)
    with pytest.raises(ManualRunRecoveryError, match="Windows error 999"):
        recovery_module._windows_open_process_exists_result(0, 999, pid=123)

    assert recovery_module._windows_open_file_mapping_absent_result(2) is True
    with pytest.raises(ManualRunRecoveryError, match="access denied"):
        recovery_module._windows_open_file_mapping_absent_result(5)
    assert recovery_module._windows_open_file_mapping_absent_result(999) is False
    recovery_module._windows_process_next_finished_or_raise(18)
    with pytest.raises(ManualRunRecoveryError, match="process table"):
        recovery_module._windows_process_next_finished_or_raise(999)


def test_new_artifact_publish_uses_complete_temp_before_no_overwrite_link(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destination = tmp_path / "stale-allocation-recovery.json"
    payload = {"schema": "test.schema", "value": 7}
    expected = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
        allow_nan=False,
    ).encode("utf-8") + b"\n"
    observed: list[bytes] = []

    def publish(source: Path, target: Path) -> None:
        observed.append(source.read_bytes())
        target.write_bytes(source.read_bytes())

    monkeypatch.setattr(recovery_module.os, "link", publish)

    recovery_module._write_new_or_matching_json(destination, payload, label="recovery")

    assert observed == [expected]
    assert destination.read_bytes() == expected


def test_matching_existing_artifact_rejects_symlink_or_hardlink(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = {"schema": "test.schema", "value": 7}
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
        allow_nan=False,
    ).encode("utf-8") + b"\n"

    target = tmp_path / "target.json"
    target.write_bytes(encoded)
    symlink = tmp_path / "stale-allocation-recovery.json"
    symlink_created = True
    try:
        symlink.symlink_to(target)
    except (NotImplementedError, OSError):
        symlink_created = False
    if symlink_created:
        with pytest.raises(ManualRunRecoveryError, match="plain file"):
            recovery_module._write_new_or_matching_json(
                symlink,
                payload,
                label="stale-allocation-recovery",
            )

    hardlink = tmp_path / "episode_result.json"
    try:
        os.link(target, hardlink)
    except (NotImplementedError, OSError):
        return

    def fail_exists(_source: Path, _target: Path) -> None:
        raise FileExistsError

    monkeypatch.setattr(recovery_module.os, "link", fail_exists)
    with pytest.raises(ManualRunRecoveryError, match="multiple links"):
        recovery_module._write_new_or_matching_json(
            hardlink,
            payload,
            label="episode_result",
        )


def test_runtime_manifest_replace_uses_unique_temp_and_rejects_target_swap(
    tmp_path: Path,
) -> None:
    runtime_path = tmp_path / "session.runtime.json"
    runtime_path.write_text('{"state":"RUNNING"}\n', encoding="utf-8")
    fixed_tmp = tmp_path / "session.runtime.json.tmp"
    trap = tmp_path / "trap.json"
    trap.write_text("do not touch\n", encoding="utf-8")
    try:
        fixed_tmp.symlink_to(trap)
    except (NotImplementedError, OSError):
        fixed_tmp.write_text("do not touch\n", encoding="utf-8")
    sample = recovery_module._sample_required_files([runtime_path])[runtime_path]

    recovery_module._atomic_replace_json(
        runtime_path,
        {"state": "FAILED"},
        expected_sample=sample,
    )

    assert json.loads(runtime_path.read_text(encoding="utf-8")) == {"state": "FAILED"}
    if fixed_tmp.is_symlink():
        assert trap.read_text(encoding="utf-8") == "do not touch\n"
    else:
        assert fixed_tmp.read_text(encoding="utf-8") == "do not touch\n"

    swapped = tmp_path / "swapped-runtime.json"
    swapped.write_text('{"state":"RUNNING"}\n', encoding="utf-8")
    stale_sample = recovery_module._sample_required_files([swapped])[swapped]
    swapped.write_text('{"state":"OTHER"}\n', encoding="utf-8")
    with pytest.raises(ManualRunRecoveryError, match="changed before terminal write"):
        recovery_module._atomic_replace_json(
            swapped,
            {"state": "FAILED"},
            expected_sample=stale_sample,
        )


def test_cli_sanitizes_unexpected_exceptions_without_absolute_path_leak(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    leaked = tmp_path / "secret" / "session.runtime.json"

    def fail_with_os_error(**_kwargs: object) -> object:
        raise OSError(f"cannot open {leaked}")

    monkeypatch.setattr(recovery_module, "recover_manual_abandoned_run", fail_with_os_error)

    code = recovery_module.main(
        [
            "--run-dir",
            str(tmp_path.resolve()),
            "--expected-run-id",
            "manual-20260811-deadbeef",
            "--expected-session-id",
            "manual-recovery",
            "--expected-dds-domain",
            "42",
        ]
    )

    captured = capsys.readouterr()
    assert code == 1
    assert captured.err.strip() == "manual run recovery failed"
    assert str(leaked) not in captured.err
