"""Offline contracts for the manual playable diagnostic launcher and short gate."""

# ruff: noqa: S101

from __future__ import annotations

import importlib.util
import json
import sys
import threading
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest

from sim.runtime.coordinator.manual_locomotion_gate import ManualLocomotionGateResult
from sim.runtime.coordinator.manual_short_gate import ManualShortGateResult
from sim.runtime.coordinator.unreal_process import UnrealLaunchProfile
from sim.runtime.windows_cpu_isolation import WindowsCpuIsolationConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCHER_PATH = REPO_ROOT / "build/manual-playable/launch_manual.py"
PROBE_PATH = REPO_ROOT / "build/manual-playable/probe_runtime_performance.ps1"
_SESSION_ID = "manual-playable-session"
_OTHER_SESSION_ID = "other-manual-session"


def _load_launcher() -> ModuleType:
    name = "_lingtu_manual_playable_launcher_contract"
    spec = importlib.util.spec_from_file_location(name, LAUNCHER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_launcher_requires_explicit_bundle_and_product_session_id() -> None:
    launcher = _load_launcher()
    with pytest.raises(SystemExit):
        launcher.parse_arguments([])

    bundle = REPO_ROOT / "build/session-bundles/future-final-v101"
    arguments = launcher.parse_arguments(
        [
            "--bundle",
            str(bundle),
            "--product-session-id",
            _SESSION_ID,
        ]
    )

    assert arguments.bundle == bundle
    assert arguments.product_session_id == _SESSION_ID
    assert not hasattr(launcher, "BUNDLE")


def test_bundle_loader_uses_injected_absolute_path_and_checks_session_id(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    bundle_path = (tmp_path / "bundle").resolve()
    bundle_path.mkdir()
    calls: list[tuple[Path, Path]] = []

    def fake_loader(path: Path, *, repo_root: Path) -> SimpleNamespace:
        calls.append((path, repo_root))
        return SimpleNamespace(session_id=_SESSION_ID)

    monkeypatch.setattr(launcher, "_load_stable_playable_bundle", fake_loader)

    bundle = launcher.load_manual_bundle(
        bundle_path,
        product_session_id=_SESSION_ID,
    )

    assert bundle.session_id == _SESSION_ID
    assert calls == [(bundle_path, launcher.REPO)]
    with pytest.raises(RuntimeError, match="session ID"):
        launcher.load_manual_bundle(
            bundle_path,
            product_session_id=_OTHER_SESSION_ID,
        )


def test_manual_runtime_config_explicitly_enables_windows_cpu_isolation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    launcher = _load_launcher()
    captured: dict[str, Any] = {}

    def fake_runtime_config(**kwargs: Any) -> dict[str, Any]:
        captured.update(kwargs)
        return kwargs

    monkeypatch.setattr(launcher, "PlayableRuntimeConfig", fake_runtime_config)
    ports = {
        "visual_snapshot_udp": 25001,
        "control_intent_udp": 25002,
        "control_status_udp": 25003,
    }

    result = launcher.build_manual_runtime_config(
        "manual-test-run",
        ports=ports,
        media_toolchain=object(),
    )

    assert result == captured
    assert captured["ports"] is ports
    assert captured["windows_cpu_isolation"] == WindowsCpuIsolationConfig()
    assert captured["frame_capture_max"] == 600
    assert captured["depth_capture_in_main_renderer"] is False
    assert captured["shared_color_depth_capture"] is False
    assert captured["main_view_screen_percentage"] == 100


def test_manual_cli_injects_bounded_main_view_screen_percentage() -> None:
    launcher = _load_launcher()

    arguments = launcher.parse_arguments(
        [
            "--bundle",
            "D:/bundle",
            "--product-session-id",
            _SESSION_ID,
            "--main-view-screen-percentage",
            "80",
        ]
    )

    assert arguments.main_view_screen_percentage == 80
    with pytest.raises(SystemExit):
        launcher.parse_arguments(
            [
                "--bundle",
                "D:/bundle",
                "--product-session-id",
                _SESSION_ID,
                "--main-view-screen-percentage",
                "49",
            ]
        )


def test_manual_cli_can_enable_shared_color_depth_capture() -> None:
    launcher = _load_launcher()

    default_arguments = launcher.parse_arguments(
        [
            "--bundle",
            "D:/bundle",
            "--product-session-id",
            _SESSION_ID,
        ]
    )
    enabled_arguments = launcher.parse_arguments(
        [
            "--bundle",
            "D:/bundle",
            "--product-session-id",
            _SESSION_ID,
            "--shared-color-depth-capture",
        ]
    )

    assert default_arguments.shared_color_depth_capture is False
    assert enabled_arguments.shared_color_depth_capture is True


def test_manual_cli_selects_one_explicit_diagnostic_mode() -> None:
    launcher = _load_launcher()
    common = [
        "--bundle",
        "D:/bundle",
        "--product-session-id",
        _SESSION_ID,
    ]

    default_arguments = launcher.parse_arguments(common)
    locomotion_arguments = launcher.parse_arguments(
        [*common, "--diagnostic-mode", "locomotion"]
    )

    assert default_arguments.diagnostic_mode == "performance"
    assert default_arguments.locomotion_input_method == "foreground"
    assert locomotion_arguments.diagnostic_mode == "locomotion"
    targeted_arguments = launcher.parse_arguments(
        [
            *common,
            "--diagnostic-mode",
            "locomotion",
            "--locomotion-input-method",
            "targeted",
        ]
    )
    assert targeted_arguments.locomotion_input_method == "targeted"
    with pytest.raises(SystemExit):
        launcher.parse_arguments([*common, "--diagnostic-mode", "combined"])


def test_manual_unreal_factory_preserves_sdk_quiet_playable_profile_and_exact_argv(
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    process = launcher.manual_unreal_factory(
        tmp_path / "UnrealEditor.exe",
        tmp_path / "RobotSimUE.uproject",
        "/Game/RobotSim/Maps/FactoryPark_HF",
        frame_capture_dir=tmp_path / "ignored-capture",
        frame_capture_every=1,
        frame_capture_max=600,
        session_camera_tag="LingTuSessionCamera",
        motion_camera_stable_id="thunder_01/base_link",
        launch_profile=UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
        depth_capture_in_main_renderer=False,
        shared_color_depth_capture=True,
        main_view_screen_percentage=80,
        affinity_mask=0b1100,
    )
    allocation = SimpleNamespace(
        run_id="manual-argv-test",
        path=tmp_path / "run-allocation.json",
        artifact_root=tmp_path,
        log_dir=tmp_path / "logs",
        ports={
            "visual_snapshot_udp": 25101,
            "control_intent_udp": 25102,
            "control_status_udp": 25103,
        },
    )

    command = process.command(
        bundle_dir=tmp_path / "bundle",
        allocation=allocation,
        snapshot_port=25101,
        model_generation=0,
        reset_generation=0,
    )

    assert command.count("-unattended") == 1
    assert command.count("-UnattendedInput") == 1
    assert command.count("-NoSound") == 1
    assert command.count(
        "-ExecCmds=t.MaxFPS 30,r.AntiAliasingMethod 4,"
        "r.DynamicRes.OperationMode 0,r.ScreenPercentage 80"
    ) == 1
    assert command.count("-ResX=1920") == 1
    assert command.count("-ResY=1080") == 1
    assert "-LingTuDepthCaptureInMainRenderer" not in command
    assert command.count("-LingTuSharedColorDepthCapture") == 1
    assert command.count("-LingTuRuntimeUI") == 1
    assert (
        "-DisablePlugins=ModelContextProtocol,AllToolsets,Terminal,Tripo3DUEBridge"
        in command
    )
    assert process._launch_profile is UnrealLaunchProfile.PLAYABLE_SDK_QUIET
    assert process._affinity_mask == 0b1100
    assert process._frame_capture_dir is None


def test_locomotion_diagnostic_selects_a_real_interactive_unreal_window(
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()

    assert (
        launcher.select_manual_unreal_factory("performance")
        is launcher.manual_unreal_factory
    )
    assert (
        launcher.select_manual_unreal_factory("locomotion", "foreground")
        is launcher.manual_locomotion_unreal_factory
    )
    assert (
        launcher.select_manual_unreal_factory("locomotion", "targeted")
        is launcher.manual_unreal_factory
    )
    process = launcher.manual_locomotion_unreal_factory(
        tmp_path / "UnrealEditor.exe",
        tmp_path / "RobotSimUE.uproject",
        "/Game/RobotSim/Maps/FactoryPark_HF",
        frame_capture_dir=None,
        session_camera_tag="LingTuSessionCamera",
        motion_camera_stable_id="thunder_01/base_link",
        launch_profile=UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
        depth_capture_in_main_renderer=False,
        shared_color_depth_capture=False,
        main_view_screen_percentage=100,
        affinity_mask=0b1100,
    )

    assert process._launch_profile is UnrealLaunchProfile.FOREGROUND_SDK_QUIET
    allocation = SimpleNamespace(
        run_id="manual-locomotion-window",
        path=tmp_path / "run-allocation.json",
        artifact_root=tmp_path,
        log_dir=tmp_path / "logs",
        ports={
            "visual_snapshot_udp": 25201,
            "control_intent_udp": 25202,
            "control_status_udp": 25203,
        },
    )
    command = process.command(
        bundle_dir=tmp_path / "bundle",
        allocation=allocation,
        snapshot_port=25201,
        model_generation=0,
        reset_generation=0,
    )
    assert command.count("-unattended") == 1
    assert "-UnattendedInput" not in command
    assert command.count("-NoCompile") == 1
    assert command.count("-LingTuRuntimeUI") == 1


def test_manual_health_journal_pins_identity_and_remembers_transient_failures(
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()

    class FakeSession:
        def __init__(self) -> None:
            self.state = SimpleNamespace(value="RUNNING")
            self.background_thread_alive = True
            self.observer = None
            self.detached: list[int] = []

        def attach_event_observer(self, observer: Any, *, replay_latest_snapshot: bool) -> int:
            assert replay_latest_snapshot is True
            self.observer = observer
            observer(
                {
                    "model_generation": 3,
                    "reset_generation": 4,
                    "sequence": 10,
                }
            )
            return 17

        def detach_event_observer(self, token: int) -> bool:
            self.detached.append(token)
            return True

    session = FakeSession()
    path = tmp_path / "manual-runtime-health.json"
    journal = launcher.ManualRuntimeHealthJournal(
        session,
        path,
        run_id="manual-test-run",
        session_id=_SESSION_ID,
        period_s=60.0,
    )
    journal.start()
    try:
        initial = json.loads(path.read_text(encoding="utf-8"))
        assert initial["run_id"] == "manual-test-run"
        assert initial["session_id"] == _SESSION_ID
        assert initial["session_state"] == "RUNNING"
        assert initial["owner_thread_alive"] is True
        assert initial["non_running_transition_count"] == 0
        assert initial["owner_thread_stop_count"] == 0

        session.state.value = "PAUSED"
        assert session.observer is not None
        session.observer(
            {
                "model_generation": 3,
                "reset_generation": 4,
                "sequence": 11,
            }
        )
        session.state.value = "RUNNING"
        session.observer(
            {
                "model_generation": 3,
                "reset_generation": 4,
                "sequence": 12,
            }
        )
        session.background_thread_alive = False
        journal.write_snapshot()

        final = json.loads(path.read_text(encoding="utf-8"))
        assert final["session_state"] == "RUNNING"
        assert final["owner_thread_alive"] is False
        assert final["non_running_transition_count"] == 1
        assert final["owner_thread_stop_count"] == 1
        assert final["event_sequence"] == 3
        assert final["last_truth_sequence"] == 12
        assert final["monitor_sequence"] > initial["monitor_sequence"]
    finally:
        journal.close()

    assert session.detached == [17]


def test_manual_health_journal_retries_transient_windows_replace_conflict(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()

    class FakeSession:
        def __init__(self) -> None:
            self.state = SimpleNamespace(value="RUNNING")
            self.background_thread_alive = True
            self.observer = None

        def attach_event_observer(self, observer: Any, *, replay_latest_snapshot: bool) -> int:
            assert replay_latest_snapshot is True
            self.observer = observer
            observer(
                {
                    "model_generation": 0,
                    "reset_generation": 0,
                    "sequence": 1,
                }
            )
            return 19

        def detach_event_observer(self, _token: int) -> bool:
            return True

    replace_calls = 0

    def flaky_replace(source: Path, destination: Path) -> None:
        nonlocal replace_calls
        replace_calls += 1
        if replace_calls < 3:
            error = PermissionError("health reader owns the destination")
            error.winerror = 5  # type: ignore[attr-defined]
            raise error
        source.replace(destination)

    real_retry = launcher.replace_file_with_retry

    def retry_without_delay(temporary: Path, destination: Path) -> None:
        real_retry(
            temporary,
            destination,
            attempts=10,
            delay_s=0.0,
            replace=flaky_replace,
            sleep=lambda _seconds: None,
        )

    monkeypatch.setattr(launcher, "replace_file_with_retry", retry_without_delay)
    path = tmp_path / "manual-runtime-health.json"
    journal = launcher.ManualRuntimeHealthJournal(
        FakeSession(),
        path,
        run_id="manual-retry-run",
        session_id=_SESSION_ID,
        period_s=60.0,
    )

    journal.start()
    try:
        journal.raise_if_failed()
        document = json.loads(path.read_text(encoding="utf-8"))
        assert document["model_generation"] == 0
        assert document["reset_generation"] == 0
        assert replace_calls == 3
    finally:
        journal.close()


def test_manual_health_journal_background_update_retries_and_advances(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()

    class FakeSession:
        def __init__(self) -> None:
            self.state = SimpleNamespace(value="RUNNING")
            self.background_thread_alive = True
            self.observer = None

        def attach_event_observer(self, observer: Any, *, replay_latest_snapshot: bool) -> int:
            assert replay_latest_snapshot is True
            self.observer = observer
            observer(
                {
                    "model_generation": 0,
                    "reset_generation": 0,
                    "sequence": 1,
                }
            )
            return 23

        def detach_event_observer(self, _token: int) -> bool:
            return True

    real_retry = launcher.replace_file_with_retry
    publication_calls = 0
    background_attempts = 0
    background_published = threading.Event()

    def controlled_retry(temporary: Path, destination: Path) -> None:
        nonlocal publication_calls, background_attempts
        publication_calls += 1
        if publication_calls == 1:
            temporary.replace(destination)
            return

        def flaky_replace(source: Path, target: Path) -> None:
            nonlocal background_attempts
            background_attempts += 1
            if background_attempts < 3:
                error = PermissionError("health reader owns the destination")
                error.winerror = 5  # type: ignore[attr-defined]
                raise error
            source.replace(target)

        real_retry(
            temporary,
            destination,
            attempts=10,
            delay_s=0.0,
            replace=flaky_replace,
            sleep=lambda _seconds: None,
        )
        background_published.set()

    monkeypatch.setattr(launcher, "replace_file_with_retry", controlled_retry)
    session = FakeSession()
    path = tmp_path / "manual-runtime-health.json"
    journal = launcher.ManualRuntimeHealthJournal(
        session,
        path,
        run_id="manual-background-retry",
        session_id=_SESSION_ID,
        period_s=1.0,
    )

    journal.start()
    try:
        initial = json.loads(path.read_text(encoding="utf-8"))
        assert session.observer is not None
        session.observer(
            {
                "model_generation": 0,
                "reset_generation": 0,
                "sequence": 2,
            }
        )
        if not background_published.wait(timeout=3.0):
            journal.raise_if_failed()
        assert background_published.is_set()
        journal.raise_if_failed()
        updated = json.loads(path.read_text(encoding="utf-8"))
        assert updated["model_generation"] == 0
        assert updated["reset_generation"] == 0
        assert updated["last_truth_sequence"] == 2
        assert updated["monitor_sequence"] > initial["monitor_sequence"]
        assert background_attempts >= 3
    finally:
        journal.close()


def test_manual_health_journal_background_retry_exhaustion_preserves_cause(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()

    class FakeSession:
        def __init__(self) -> None:
            self.state = SimpleNamespace(value="RUNNING")
            self.background_thread_alive = True

        def attach_event_observer(self, observer: Any, *, replay_latest_snapshot: bool) -> int:
            assert replay_latest_snapshot is True
            observer(
                {
                    "model_generation": 0,
                    "reset_generation": 0,
                    "sequence": 1,
                }
            )
            return 29

        def detach_event_observer(self, _token: int) -> bool:
            return True

    real_retry = launcher.replace_file_with_retry
    publication_calls = 0
    exhausted = threading.Event()
    failure = PermissionError("health destination remains locked")
    failure.winerror = 5  # type: ignore[attr-defined]

    def controlled_retry(temporary: Path, destination: Path) -> None:
        nonlocal publication_calls
        publication_calls += 1
        if publication_calls == 1:
            temporary.replace(destination)
            return

        def always_fail(_source: Path, _target: Path) -> None:
            raise failure

        try:
            real_retry(
                temporary,
                destination,
                attempts=2,
                delay_s=0.0,
                replace=always_fail,
                sleep=lambda _seconds: None,
            )
        finally:
            exhausted.set()

    monkeypatch.setattr(launcher, "replace_file_with_retry", controlled_retry)
    journal = launcher.ManualRuntimeHealthJournal(
        FakeSession(),
        tmp_path / "manual-runtime-health.json",
        run_id="manual-background-failure",
        session_id=_SESSION_ID,
        period_s=0.001,
    )

    journal.start()
    assert exhausted.wait(timeout=2.0)
    with pytest.raises(
        RuntimeError,
        match="manual runtime health journal failed",
    ) as captured:
        journal.close()
    assert captured.value.__cause__ is failure


def test_manual_health_journal_close_persists_last_observed_truth(
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()

    class FakeSession:
        def __init__(self) -> None:
            self.state = SimpleNamespace(value="RUNNING")
            self.background_thread_alive = True
            self.observer = None

        def attach_event_observer(self, observer: Any, *, replay_latest_snapshot: bool) -> int:
            assert replay_latest_snapshot is True
            self.observer = observer
            observer(
                {
                    "model_generation": 0,
                    "reset_generation": 0,
                    "sequence": 1,
                }
            )
            return 31

        def detach_event_observer(self, _token: int) -> bool:
            return True

    session = FakeSession()
    path = tmp_path / "manual-runtime-health.json"
    journal = launcher.ManualRuntimeHealthJournal(
        session,
        path,
        run_id="manual-close-persist",
        session_id=_SESSION_ID,
        period_s=60.0,
    )
    journal.start()
    initial = json.loads(path.read_text(encoding="utf-8"))
    assert session.observer is not None
    session.observer(
        {
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": 2,
        }
    )

    journal.close()

    final = json.loads(path.read_text(encoding="utf-8"))
    assert final["last_truth_sequence"] == 2
    assert final["event_sequence"] == 2
    assert final["monitor_sequence"] > initial["monitor_sequence"]
    assert not journal._thread.is_alive()


def test_manual_health_journal_detach_failure_still_stops_writer(
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    detach_failure = RuntimeError("detach failed")

    class FakeSession:
        def __init__(self) -> None:
            self.state = SimpleNamespace(value="RUNNING")
            self.background_thread_alive = True

        def attach_event_observer(self, observer: Any, *, replay_latest_snapshot: bool) -> int:
            assert replay_latest_snapshot is True
            observer(
                {
                    "model_generation": 0,
                    "reset_generation": 0,
                    "sequence": 1,
                }
            )
            return 37

        def detach_event_observer(self, _token: int) -> bool:
            raise detach_failure

    journal = launcher.ManualRuntimeHealthJournal(
        FakeSession(),
        tmp_path / "manual-runtime-health.json",
        run_id="manual-detach-failure",
        session_id=_SESSION_ID,
        period_s=60.0,
    )
    journal.start()

    with pytest.raises(RuntimeError, match="detach failed") as captured:
        journal.close()

    assert captured.value is detach_failure
    assert not journal._thread.is_alive()


def test_latest_status_retries_transient_windows_replace_conflict(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    status_path = tmp_path / "latest.json"
    monkeypatch.setattr(launcher, "STATUS", status_path)
    real_retry = launcher.replace_file_with_retry
    attempts = 0

    def retry_without_delay(temporary: Path, destination: Path) -> None:
        def flaky_replace(source: Path, target: Path) -> None:
            nonlocal attempts
            attempts += 1
            if attempts < 3:
                error = PermissionError("status reader owns the destination")
                error.winerror = 5  # type: ignore[attr-defined]
                raise error
            source.replace(target)

        real_retry(
            temporary,
            destination,
            attempts=3,
            delay_s=0.0,
            replace=flaky_replace,
            sleep=lambda _seconds: None,
        )

    monkeypatch.setattr(launcher, "replace_file_with_retry", retry_without_delay)
    launcher.write_status("RUNNING", run_id="manual-status-retry")

    document = json.loads(status_path.read_text(encoding="utf-8"))
    assert document["state"] == "RUNNING"
    assert document["run_id"] == "manual-status-retry"
    assert attempts == 3


def test_short_probe_is_run_bound_strict_and_explicitly_non_qualification() -> None:
    script = PROBE_PATH.read_text(encoding="utf-8")

    assert "[Parameter(Mandatory = $true)][string]$RunDir" in script
    assert "[Parameter(Mandatory = $true)][string]$ExpectedRunId" in script
    assert "[Parameter(Mandatory = $true)][string]$ProductSessionId" in script
    assert "[ValidateRange(10, 15)]" in script
    assert "[int]$DurationSeconds = 12" in script
    assert "$MinimumMedianRealtimeFactor = 0.80" in script
    assert "$MinimumCameraRateHz = 29.0" in script
    assert '"thunder_01.front_rgb"' in script
    assert '"thunder_01.front_depth"' in script
    assert "ExpectedWidth = 640" in script
    assert "ExpectedHeight = 480" in script
    assert "ExpectedRateHz = 30.0" in script
    assert 'state -cne "ACTIVE"' in script
    assert 'Join-Path $RunDir "manual-runtime-health.json"' in script
    assert "non_running_transition_count" in script
    assert "owner_thread_stop_count" in script
    assert "monitor_sequence" in script
    assert "LatestStatusPath" not in script
    assert "qualification = $false" in script
    assert "manual_diagnostic_only" in script


def test_launcher_short_gate_adapter_binds_live_launch_and_public_window_proof(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    run_dir = (tmp_path / "manual-adapter").resolve()
    run_dir.mkdir()
    captured: dict[str, Any] = {}
    proof_calls: list[int] = []
    expected_proof = object()
    expected = ManualShortGateResult(
        passed=True,
        failure_reason=None,
        summary_path=run_dir / "manual-short-gate.json",
        window_proof_path=run_dir / "manual-window-presence-proof.json",
        performance_probe_path=run_dir / "runtime-performance-probe.json",
        probe_invocation_count=1,
    )

    def fake_supervisor(config: Any, **kwargs: Any) -> ManualShortGateResult:
        captured["config"] = config
        captured.update(kwargs)
        captured["proof"] = kwargs["prove_presence"](config.unreal_pid)
        return expected

    def fake_public_proof(pid: int) -> object:
        proof_calls.append(pid)
        return expected_proof

    monkeypatch.setattr(launcher, "supervise_manual_short_gate", fake_supervisor)
    monkeypatch.setattr(
        launcher,
        "prove_owned_robotsimue_window_presence",
        fake_public_proof,
    )
    launch = SimpleNamespace(
        run_dir=run_dir,
        run_id="manual-adapter",
        session_id=_SESSION_ID,
        unreal_process=SimpleNamespace(pid=4567),
    )

    result = launcher.run_manual_short_gate(
        launch,
        profile_deadline_monotonic=130.0,
    )

    assert result is expected
    config = captured["config"]
    assert config.run_dir == run_dir
    assert config.run_id == "manual-adapter"
    assert config.session_id == _SESSION_ID
    assert config.unreal_pid == 4567
    assert config.profile_deadline_monotonic == 130.0
    assert config.probe_duration_s == 12
    assert config.minimum_trigger_budget_s == 20.0
    assert (
        captured["prove_presence"]
        is launcher.prove_owned_robotsimue_window_presence
    )
    assert captured["proof"] is expected_proof
    assert proof_calls == [4567]


def test_launcher_locomotion_gate_adapter_binds_current_run_and_drive_probe(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    run_dir = (tmp_path / "manual-locomotion-adapter").resolve()
    run_dir.mkdir()
    probe = (tmp_path / "probe_drive_input.ps1").resolve()
    probe.write_text("# fixture", encoding="utf-8")
    expected = ManualLocomotionGateResult(
        passed=True,
        summary_path=run_dir / "manual-locomotion-gate.json",
        probe_path=run_dir / "runtime-locomotion-probe.json",
        probe_invocation_count=1,
    )
    captured: dict[str, Any] = {}

    def fake_supervisor(config: object, *, prove_presence: object) -> object:
        captured["config"] = config
        captured["proof"] = prove_presence
        return expected

    monkeypatch.setattr(launcher, "DRIVE_PROBE", probe)
    monkeypatch.setattr(launcher, "supervise_manual_locomotion_gate", fake_supervisor)
    launch = SimpleNamespace(
        run_dir=run_dir,
        run_id="manual-locomotion-adapter",
        session_id=_SESSION_ID,
        unreal_process=SimpleNamespace(pid=4567),
    )

    result = launcher.run_manual_locomotion_gate(
        launch,
        profile_deadline_monotonic=130.0,
    )

    assert result is expected
    config = captured["config"]
    assert config.run_dir == run_dir
    assert config.run_id == launch.run_id
    assert config.session_id == launch.session_id
    assert config.unreal_pid == 4567
    assert config.drive_probe_script == probe
    assert captured["proof"] is launcher.prove_owned_robotsimue_foreground


def _run_launcher_without_short_gate_evidence(
    launcher: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> tuple[int, list[tuple[str, dict[str, Any]]], int, list[str]]:
    bundle = (tmp_path / "bundle").resolve()
    bundle.mkdir()
    run_dir = (tmp_path / "manual-run").resolve()
    run_dir.mkdir()
    statuses: list[tuple[str, dict[str, Any]]] = []
    gate_calls = 0
    cleanup_events: list[str] = []

    class FakeJournal:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def start(self) -> None:
            pass

        def close(self) -> None:
            pass

        def raise_if_failed(self) -> None:
            pass

    def fake_runtime(run_id: str, **_kwargs: Any) -> SimpleNamespace:
        return SimpleNamespace(run_id=run_id)

    launch = SimpleNamespace(
        run_dir=run_dir,
        run_id="pending",
        session_id=_SESSION_ID,
        session=SimpleNamespace(
            state=SimpleNamespace(value="RUNNING"),
            background_thread_alive=True,
            last_error=None,
        ),
        unreal_process=SimpleNamespace(pid=101, poll=lambda: 0),
        physics_host=SimpleNamespace(pid=202),
        ports={"visual_snapshot_udp": 1},
    )

    def fake_create(_bundle: object, **kwargs: Any) -> SimpleNamespace:
        launch.run_id = kwargs["runtime"].run_id
        return launch

    def fake_short_gate(*_args: Any, **_kwargs: Any) -> None:
        nonlocal gate_calls
        gate_calls += 1

    def fake_run_playable(_launch: object, *, run_body: Any) -> None:
        try:
            run_body()
        finally:
            cleanup_events.append("owned-launch-cleanup")

    monkeypatch.setattr(launcher, "load_manual_bundle", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(
        launcher,
        "snapshot_playable_media_toolchain",
        lambda **_kwargs: object(),
    )
    monkeypatch.setattr(launcher, "allocate_udp_ports", lambda: {})
    monkeypatch.setattr(launcher, "build_manual_runtime_config", fake_runtime)
    monkeypatch.setattr(launcher, "create_playable_launch", fake_create)
    monkeypatch.setattr(launcher, "ManualTruthSnapshotJournal", FakeJournal)
    monkeypatch.setattr(launcher, "ManualRuntimeHealthJournal", FakeJournal)
    monkeypatch.setattr(launcher, "attach_manual_runtime_timing", lambda _launch: None)
    monkeypatch.setattr(launcher, "run_manual_short_gate", fake_short_gate)
    monkeypatch.setattr(
        launcher,
        "write_status",
        lambda state, **fields: statuses.append((state, fields)),
    )
    monkeypatch.setattr(
        launcher,
        "run_playable_launch",
        fake_run_playable,
    )

    exit_code = launcher.main(
        [
            "--bundle",
            str(bundle),
            "--product-session-id",
            _SESSION_ID,
        ]
    )
    return exit_code, statuses, gate_calls, cleanup_events


def test_unset_profile_duration_cannot_pass_without_short_gate_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    monkeypatch.delenv("LINGTU_MANUAL_PROFILE_SECONDS", raising=False)

    exit_code, statuses, gate_calls, _cleanup = _run_launcher_without_short_gate_evidence(
        launcher,
        monkeypatch,
        tmp_path,
    )

    assert gate_calls == 0
    assert exit_code == 1
    assert statuses[-1][0] == "CLOSED"
    assert statuses[-1][1]["short_gate_passed"] is False
    assert statuses[-1][1]["cli_exit_code"] == 1


def test_zero_profile_duration_cannot_pass_without_short_gate_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", "0")

    exit_code, statuses, gate_calls, _cleanup = _run_launcher_without_short_gate_evidence(
        launcher,
        monkeypatch,
        tmp_path,
    )

    assert gate_calls == 0
    assert exit_code == 1
    assert statuses[-1][0] == "CLOSED"
    assert statuses[-1][1]["short_gate_passed"] is False
    assert statuses[-1][1]["cli_exit_code"] == 1


def test_negative_profile_duration_cannot_pass_without_short_gate_evidence(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", "-5")

    exit_code, statuses, gate_calls, _cleanup = _run_launcher_without_short_gate_evidence(
        launcher,
        monkeypatch,
        tmp_path,
    )

    assert gate_calls == 0
    assert exit_code == 1
    assert statuses[-1][0] == "CLOSED"
    assert statuses[-1][1]["short_gate_passed"] is False
    assert statuses[-1][1]["cli_exit_code"] == 1


def test_short_gate_returning_no_result_cannot_pass(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", "30")

    exit_code, statuses, gate_calls, _cleanup = _run_launcher_without_short_gate_evidence(
        launcher,
        monkeypatch,
        tmp_path,
    )

    assert gate_calls == 1
    assert exit_code == 1
    assert statuses[-1][0] == "CLOSED"
    assert statuses[-1][1]["short_gate_passed"] is False
    assert statuses[-1][1]["cli_exit_code"] == 1


def test_nan_profile_duration_fails_terminally_after_owned_cleanup(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", "nan")

    exit_code, statuses, gate_calls, cleanup = (
        _run_launcher_without_short_gate_evidence(
            launcher,
            monkeypatch,
            tmp_path,
        )
    )

    assert gate_calls == 0
    assert cleanup == ["owned-launch-cleanup"]
    assert exit_code == 1
    assert statuses[-1][0] == "FAILED"
    assert "must be finite" in statuses[-1][1]["error"]


@pytest.mark.parametrize("profile_duration", ["inf", "-inf"])
def test_infinite_profile_duration_fails_terminally_after_owned_cleanup(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    profile_duration: str,
) -> None:
    launcher = _load_launcher()
    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", profile_duration)

    exit_code, statuses, gate_calls, cleanup = (
        _run_launcher_without_short_gate_evidence(
            launcher,
            monkeypatch,
            tmp_path,
        )
    )

    assert gate_calls == 0
    assert cleanup == ["owned-launch-cleanup"]
    assert exit_code == 1
    assert statuses[-1][0] == "FAILED"
    assert "must be finite" in statuses[-1][1]["error"]


def test_nonnumeric_profile_duration_fails_terminally_after_owned_cleanup(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", "not-a-number")

    exit_code, statuses, gate_calls, cleanup = (
        _run_launcher_without_short_gate_evidence(
            launcher,
            monkeypatch,
            tmp_path,
        )
    )

    assert gate_calls == 0
    assert cleanup == ["owned-launch-cleanup"]
    assert exit_code == 1
    assert statuses[-1][0] == "FAILED"
    assert statuses[-1][1]["error"].startswith("ValueError:")


def test_failed_short_gate_returns_nonzero_only_after_normal_run_body_close(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    launcher = _load_launcher()
    bundle = (tmp_path / "bundle").resolve()
    bundle.mkdir()
    run_dir = (tmp_path / "manual-run").resolve()
    run_dir.mkdir()
    events: list[str] = []
    statuses: list[tuple[str, dict[str, Any]]] = []

    class FakeJournal:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def start(self) -> None:
            events.append("journal.start")

        def close(self) -> None:
            events.append("journal.close")

        def raise_if_failed(self) -> None:
            pass

    run_id_box: dict[str, str] = {}

    def fake_runtime(run_id: str, **_kwargs: Any) -> SimpleNamespace:
        run_id_box["value"] = run_id
        return SimpleNamespace(run_id=run_id)

    launch = SimpleNamespace(
        run_dir=run_dir,
        run_id="pending",
        session_id=_SESSION_ID,
        session=SimpleNamespace(
            state=SimpleNamespace(value="RUNNING"),
            background_thread_alive=True,
            last_error=None,
        ),
        unreal_process=SimpleNamespace(pid=101, poll=lambda: 0),
        physics_host=SimpleNamespace(pid=202),
        ports={"visual_snapshot_udp": 1},
    )

    def fake_create(_bundle: object, **kwargs: Any) -> SimpleNamespace:
        launch.run_id = kwargs["runtime"].run_id
        return launch

    failed = ManualShortGateResult(
        passed=False,
        failure_reason="performance gate failed",
        summary_path=run_dir / "manual-short-gate.json",
        window_proof_path=run_dir / "manual-window-presence-proof.json",
        performance_probe_path=run_dir / "runtime-performance-probe.json",
        probe_invocation_count=1,
    )

    monkeypatch.setenv("LINGTU_MANUAL_PROFILE_SECONDS", "30")
    monkeypatch.setattr(launcher, "load_manual_bundle", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(
        launcher,
        "snapshot_playable_media_toolchain",
        lambda **_kwargs: object(),
    )
    monkeypatch.setattr(launcher, "allocate_udp_ports", lambda: {})
    monkeypatch.setattr(launcher, "build_manual_runtime_config", fake_runtime)
    monkeypatch.setattr(launcher, "create_playable_launch", fake_create)
    monkeypatch.setattr(launcher, "ManualTruthSnapshotJournal", FakeJournal)
    monkeypatch.setattr(launcher, "ManualRuntimeHealthJournal", FakeJournal)
    monkeypatch.setattr(launcher, "attach_manual_runtime_timing", lambda _launch: None)
    monkeypatch.setattr(launcher, "run_manual_short_gate", lambda *_args, **_kwargs: failed)
    monkeypatch.setattr(
        launcher,
        "write_status",
        lambda state, **fields: statuses.append((state, fields)),
    )

    def fake_run_playable(_launch: object, *, run_body: Any) -> None:
        events.append("run_body.begin")
        run_body()
        events.append("run_body.normal_return")

    monkeypatch.setattr(launcher, "run_playable_launch", fake_run_playable)

    exit_code = launcher.main(
        [
            "--bundle",
            str(bundle),
            "--product-session-id",
            _SESSION_ID,
        ]
    )

    assert exit_code == 1
    assert "run_body.normal_return" in events
    assert statuses[-1][0] == "CLOSED"
    assert statuses[-1][1]["short_gate_passed"] is False
    assert statuses[-1][1]["cli_exit_code"] == 1
    assert not any(state == "FAILED" for state, _fields in statuses)

    events.clear()
    statuses.clear()

    def fail_gate_internally(*_args: Any, **_kwargs: Any) -> None:
        raise OSError(r"private path C:\secret\must-not-leak")

    monkeypatch.setattr(launcher, "run_manual_short_gate", fail_gate_internally)
    internal_exit_code = launcher.main(
        [
            "--bundle",
            str(bundle),
            "--product-session-id",
            _SESSION_ID,
        ]
    )

    assert internal_exit_code == 1
    assert "run_body.normal_return" in events
    assert statuses[-1][0] == "CLOSED"
    assert statuses[-1][1]["cli_exit_code"] == 1
    internal_error = statuses[-1][1]["short_gate_internal_error"]
    assert internal_error == "OSError: short-gate artifact publication failed closed"
    assert "secret" not in internal_error
    assert not any(state == "FAILED" for state, _fields in statuses)
