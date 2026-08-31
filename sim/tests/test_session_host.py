# ruff: noqa: S101

from __future__ import annotations

import json
import shutil
import subprocess
import uuid
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

import sim.runtime.coordinator.unreal_process as unreal_process_module
from sim.catalog import CatalogResolver
from sim.runtime.control.fake import zero_output_components
from sim.runtime.coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState
from sim.runtime.coordinator.external_evidence import ExternalEvidenceWatcher
from sim.runtime.coordinator.readiness import BindingFacet, BindingReadiness, BindingState
from sim.runtime.coordinator.session_host import SessionHost
from sim.runtime.coordinator.unreal_process import (
    PackagedUnrealProcess,
    UnrealLaunchProfile,
    UnrealProcess,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "scenarios" / "catalog" / "thunderv4_unreal" / "session.yaml"
CONTRACT_SESSION = (
    REPO_ROOT
    / "sim"
    / "scenarios"
    / "catalog"
    / "thunder_omni_contract"
    / "session.yaml"
)
ACTUATORS = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "FR_foot_joint",
    "FL_foot_joint",
    "RR_foot_joint",
    "RL_foot_joint",
)


class _Readiness:
    blocking_reasons = {"visual": "binding is UNBOUND"}


class _SensorReadiness:
    blocking_reasons = {"thunder_01.front_rgb": "stream is UNBOUND"}


class _Coordinator:
    def __init__(self, order: list[str], *, ready_on_prepare: bool = False) -> None:
        self.order = order
        self.state = RuntimeState.NEW
        self.bundle_dir = Path("C:/bundle with spaces")
        self.allocation = SimpleNamespace(
            run_id="session-host-test",
            ports={"visual_snapshot_udp": 25001},
            path=Path("C:/run with spaces/run-allocation.json"),
            artifact_root=Path("C:/artifact root with spaces"),
            log_dir=Path("C:/run with spaces/logs"),
        )
        self.plan = SimpleNamespace(repo_root=Path("C:/repo"), model_generation=0)
        self.readiness = _Readiness()
        self._sensor_readiness = _SensorReadiness()
        self.ready_on_prepare = ready_on_prepare
        self.sequence = 0

    @property
    def sensor_readiness(self) -> _SensorReadiness:
        return self._sensor_readiness

    def prepare(self) -> dict[str, Any]:
        self.order.append("coordinator.prepare")
        self.state = RuntimeState.READY if self.ready_on_prepare else RuntimeState.PREPARING
        return self._event("ready")

    def snapshot(self) -> dict[str, Any]:
        self.order.append("coordinator.snapshot")
        return self._event("snapshot")

    def warmup(self, steps: int = 1) -> dict[str, Any]:
        self.order.append(f"coordinator.warmup:{steps}")
        assert self.state is RuntimeState.PREPARING
        self.sequence += steps
        return self._event("snapshot")

    def start(self) -> dict[str, Any]:
        self.order.append("coordinator.start")
        if self.state is not RuntimeState.READY:
            raise CoordinatorError("start requires READY")
        self.state = RuntimeState.RUNNING
        return self._event("running")

    def advance(self, steps: int = 1) -> dict[str, Any]:
        self.order.append(f"coordinator.advance:{steps}")
        self.sequence += steps
        return self._event("snapshot")

    def advance_realtime(self, steps: int = 1) -> dict[str, Any]:
        self.order.append(f"coordinator.advance_realtime:{steps}")
        self.sequence += steps
        return self._event("snapshot")

    def pause(self) -> dict[str, Any]:
        self.order.append("coordinator.pause")
        if self.state is not RuntimeState.RUNNING:
            raise CoordinatorError("pause requires RUNNING")
        self.state = RuntimeState.PAUSED
        return self._event("paused")

    def reset(self) -> dict[str, Any]:
        self.order.append("coordinator.reset")
        self.sequence = 0
        return {
            **self._event("snapshot"),
            "reset_generation": 1,
        }

    def stop(
        self,
        *,
        failure_reason: str | None = None,
    ) -> dict[str, Any]:
        self.order.append("coordinator.stop")
        self.state = RuntimeState.FAILED if failure_reason is not None else RuntimeState.STOPPED
        return self._event("stopped")

    def finalize_terminal_failure(self, failure_reason: str) -> dict[str, Any]:
        self.order.append(f"coordinator.finalize_terminal_failure:{failure_reason}")
        self.state = RuntimeState.FAILED
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": "a" * 64,
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": self.sequence,
            "physics_step": self.sequence,
            "sim_time_ns": self.sequence,
            "bodies": [],
        }


class _Unreal:
    def __init__(self, order: list[str], *, exit_code: int | None = None) -> None:
        self.order = order
        self.exit_code = exit_code

    def start(self, **_: Any) -> None:
        self.order.append("unreal.start")

    def poll(self) -> int | None:
        self.order.append("unreal.poll")
        return self.exit_code

    def terminate(self) -> None:
        self.order.append("unreal.terminate")


class _Publisher:
    def __init__(
        self,
        order: list[str],
        *,
        fail_close: bool = False,
        fail_publish: BaseException | None = None,
    ) -> None:
        self.order = order
        self.fail_close = fail_close
        self.fail_publish = fail_publish
        self.snapshots: list[Any] = []

    def publish(self, event: Any) -> int:
        self.order.append("publisher.publish")
        if self.fail_publish is not None:
            raise self.fail_publish
        self.snapshots.append(event)
        return 1

    def close(self) -> None:
        self.order.append("publisher.close")
        if self.fail_close:
            raise RuntimeError("publisher close failed")


class _Evidence:
    def __init__(self, order: list[str], name: str, *, ready_after: int) -> None:
        self.order = order
        self.name = name
        self.ready_after = ready_after
        self.calls = 0

    def apply(self, target: _Coordinator) -> bool:
        self.order.append(f"evidence.apply:{self.name}")
        self.calls += 1
        if self.calls >= self.ready_after:
            target.state = RuntimeState.READY
        return True

    def advance_generation(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> None:
        self.order.append(
            f"evidence.advance:{self.name}:{model_generation}:{reset_generation}"
        )


class _VisualReadinessEvidence:
    def __init__(self, order: list[str], *, active_after: int) -> None:
        self.order = order
        self.active_after = active_after
        self.calls = 0

    def apply(self, target: _Coordinator) -> bool:
        self.order.append("evidence.apply:visual-readiness")
        self.calls += 1
        if (
            self.calls >= self.active_after
            and target.readiness.state(BindingFacet.VISUAL) is not BindingState.ACTIVE
        ):
            target.readiness = target.readiness.mark_prepared(
                BindingFacet.VISUAL,
                model_generation=0,
                reset_generation=0,
            ).mark_active(
                BindingFacet.VISUAL,
                model_generation=0,
                reset_generation=0,
            )
        return True

    def advance_generation(
        self,
        *,
        model_generation: int,
        reset_generation: int,
    ) -> None:
        self.order.append(
            f"evidence.advance:visual-readiness:{model_generation}:{reset_generation}"
        )


def test_prepare_publishes_and_applies_first_evidence_before_warmup() -> None:
    order: list[str] = []
    coordinator = _Coordinator(order)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=_Unreal(order),
        publisher=_Publisher(order),
        evidence_watchers=(
            _Evidence(order, "camera", ready_after=99),
            _Evidence(order, "visual", ready_after=2),
        ),
        warmup_steps=3,
        sleep_s=0,
    )

    host.prepare()

    assert coordinator.state is RuntimeState.READY
    assert order.index("publisher.publish") < order.index("coordinator.warmup:3")
    assert order.index("evidence.apply:camera") < order.index("coordinator.warmup:3")
    assert order.index("evidence.apply:visual") < order.index("coordinator.warmup:3")
    assert "coordinator.warmup:3" in order


def test_prepare_until_visual_applied_freezes_physics_while_waiting_for_active() -> None:
    order: list[str] = []
    coordinator = _Coordinator(order)
    coordinator.readiness = BindingReadiness.for_required(
        (BindingFacet.PHYSICS, BindingFacet.VISUAL)
    )
    publisher = _Publisher(order)
    evidence = _VisualReadinessEvidence(order, active_after=3)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=_Unreal(order),
        publisher=publisher,
        evidence_watchers=(evidence,),
        warmup_steps=7,
        sleep_s=0,
    )

    event = host.prepare_until_visual_applied()

    assert event["model_generation"] == 0
    assert event["reset_generation"] == 0
    assert event["sequence"] == 0
    assert coordinator.state is RuntimeState.PREPARING
    assert evidence.calls == 3
    assert len(publisher.snapshots) == 3
    assert [snapshot["sequence"] for snapshot in publisher.snapshots] == [0, 0, 0]
    assert [snapshot["physics_step"] for snapshot in publisher.snapshots] == [0, 0, 0]
    assert [snapshot["model_generation"] for snapshot in publisher.snapshots] == [0, 0, 0]
    assert [snapshot["reset_generation"] for snapshot in publisher.snapshots] == [0, 0, 0]
    assert not any(entry.startswith("coordinator.warmup") for entry in order)
    assert not any(entry.startswith("coordinator.advance") for entry in order)


def test_unreal_exits_during_prepare_fail_closed() -> None:
    order: list[str] = []
    host = SessionHost(
        coordinator=_Coordinator(order),
        unreal_process=_Unreal(order, exit_code=7),
        publisher=_Publisher(order),
        evidence_watchers=(_Evidence(order, "visual", ready_after=99),),
        sleep_s=0,
    )

    with pytest.raises(CoordinatorError, match=r"Unreal process exited before session READY"):
        host.prepare()

    assert order[-3:] == ["unreal.terminate", "publisher.close", "coordinator.stop"]


def test_prepare_timeout_reports_binding_and_sensor_blocking_reasons() -> None:
    order: list[str] = []
    times = iter([0.0, 2.0])
    host = SessionHost(
        coordinator=_Coordinator(order),
        unreal_process=_Unreal(order),
        publisher=_Publisher(order),
        evidence_watchers=(_Evidence(order, "visual", ready_after=99),),
        ready_timeout_s=1.0,
        sleep_s=0,
        monotonic=lambda: next(times),
    )

    with pytest.raises(CoordinatorError) as exc_info:
        host.prepare()

    message = str(exc_info.value)
    assert "binding visual: binding is UNBOUND" in message
    assert "sensor thunder_01.front_rgb: stream is UNBOUND" in message


def test_run_cleanup_pauses_running_coordinator_before_external_shutdown() -> None:
    order: list[str] = []
    publisher = _Publisher(order)
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order),
        publisher=publisher,
        sleep_s=0,
    )

    assert host.run(frame_limit=1, steps_per_frame=4) == 1
    assert publisher.snapshots[-1]["physics_step"] == 4
    assert order[-4:] == [
        "coordinator.pause",
        "unreal.terminate",
        "publisher.close",
        "coordinator.stop",
    ]


def test_realtime_advance_publishes_one_chunk_snapshot() -> None:
    order: list[str] = []
    publisher = _Publisher(order)
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order),
        publisher=publisher,
        sleep_s=0,
    )
    host.prepare()
    host.start()

    snapshot = host.advance_realtime(10)

    assert snapshot["physics_step"] == 10
    assert publisher.snapshots[-1]["physics_step"] == 10
    assert "coordinator.advance_realtime:10" in order


def test_realtime_visual_snapshots_are_capped_without_throttling_physics() -> None:
    order: list[str] = []
    publisher = _Publisher(order)
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order),
        publisher=publisher,
        realtime_snapshot_period_ns=20,
        sleep_s=0,
    )
    host.prepare()
    host.start()

    first = host.advance_realtime(10)
    second = host.advance_realtime(10)
    third = host.advance_realtime(10)

    assert [first["physics_step"], second["physics_step"], third["physics_step"]] == [
        10,
        20,
        30,
    ]
    assert [snapshot["physics_step"] for snapshot in publisher.snapshots] == [10, 30]


def test_realtime_external_evidence_is_polled_at_wall_clock_rate() -> None:
    order: list[str] = []
    now = [0.0]
    evidence = _Evidence(order, "camera", ready_after=99)
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order),
        publisher=_Publisher(order),
        evidence_watchers=(evidence,),
        realtime_evidence_period_s=0.02,
        monotonic=lambda: now[0],
        sleep_s=0,
    )
    host.prepare()
    host.start()

    host.advance_realtime(1)
    now[0] = 0.019
    host.advance_realtime(1)
    now[0] = 0.020
    host.advance_realtime(1)

    assert evidence.calls == 2


def test_reset_applies_external_evidence_immediately_during_realtime_throttle() -> None:
    order: list[str] = []
    now = [0.0]
    evidence = _Evidence(order, "camera", ready_after=99)
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order),
        publisher=_Publisher(order),
        evidence_watchers=(evidence,),
        realtime_evidence_period_s=1.0,
        monotonic=lambda: now[0],
        sleep_s=0,
    )
    host.prepare()
    host.start()
    host.advance_realtime(1)
    now[0] = 0.001

    host.reset()

    assert evidence.calls == 2


def test_interactive_exit_stops_core_then_keeps_unreal_until_natural_readback() -> None:
    order: list[str] = []
    coordinator = _Coordinator(order, ready_on_prepare=True)
    unreal = _Unreal(order)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=unreal,
        publisher=_Publisher(order),
        sleep_s=0,
    )
    host.prepare()
    host.start()

    terminal_event = host.stop_runtime_before_visual()

    assert terminal_event["event"] == "stopped"
    assert coordinator.state is RuntimeState.STOPPED
    assert order[-2:] == ["coordinator.stop", "unreal.poll"]
    assert "unreal.terminate" not in order
    assert "publisher.close" not in order

    order.append("terminal.status.send")
    unreal.exit_code = 0
    assert host.finalize_visual_after_terminal() == 0

    core_stop = order.index("coordinator.stop")
    terminal_send = order.index("terminal.status.send", core_stop)
    natural_readback = order.index("unreal.poll", terminal_send)
    final_terminate = order.index("unreal.terminate", natural_readback)
    publisher_close = order.index("publisher.close", final_terminate)
    assert core_stop < terminal_send < natural_readback < final_terminate < publisher_close

    before_idempotent_close = list(order)
    host.close()
    assert order == before_idempotent_close


def test_interactive_exit_rejects_unreal_that_exited_before_terminal_status() -> None:
    order: list[str] = []
    coordinator = _Coordinator(order, ready_on_prepare=True)
    unreal = _Unreal(order)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=unreal,
        publisher=_Publisher(order),
        sleep_s=0,
    )
    host.prepare()
    host.start()
    unreal.exit_code = 0

    with pytest.raises(CoordinatorError, match="before the simulation core terminal"):
        host.stop_runtime_before_visual()

    assert "coordinator.stop" not in order
    host.close()


def test_post_stop_terminal_status_failure_is_persisted_as_failed_with_real_coordinator(
    tmp_path: Path,
) -> None:
    test_root = REPO_ROOT / "tmp" / f"session-host-terminal-send-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        order: list[str] = []
        physics = _PhysicsHost()
        coordinator = RuntimeCoordinator(
            bundle_dir=_contract_bundle(test_root),
            repo_root=REPO_ROOT,
            run_root=test_root / "runs",
            physics_host=physics,
            controller_factory=zero_output_components,
            run_id="terminal-send-failure",
            dds_domain=126,
            ports={"visual_snapshot_udp": 25126},
        )
        host = SessionHost(
            coordinator=coordinator,
            unreal_process=_Unreal(order),
            publisher=_Publisher(order),
            sleep_s=0,
        )
        host.prepare()
        host.start()

        terminal_event = host.stop_runtime_before_visual()
        assert terminal_event["event"] == "stopped"

        host.close(failure_reason="terminal status send failed")

        manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
        episode = json.loads(
            (coordinator.allocation.run_dir / "episode_result.json").read_text(
                encoding="utf-8"
            )
        )
        assert coordinator.state is RuntimeState.FAILED
        assert manifest["state"] == "FAILED"
        assert episode["status"] == "FAILED"
        assert episode["failure_reason"] == "terminal status send failed"
        assert "unreal.terminate" in order
        assert "publisher.close" in order
        assert physics.calls.count("stop") == 1
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_terminal_unreal_readback_failure_promotes_stopped_runtime_to_failed() -> None:
    order: list[Any] = []
    coordinator = _Coordinator(order, ready_on_prepare=True)
    unreal = _Unreal(order, exit_code=4)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=unreal,
        publisher=_Publisher(order),
        sleep_s=0,
    )
    host.prepare()
    host.start()
    unreal.exit_code = None
    host.stop_runtime_before_visual()
    unreal.exit_code = 4

    with pytest.raises(CoordinatorError, match="non-zero code 4"):
        host.finalize_visual_after_terminal()

    assert coordinator.state is RuntimeState.FAILED
    assert any(
        call.startswith("coordinator.finalize_terminal_failure:")
        and "non-zero code 4" in call
        for call in order
    )


def test_visual_run_can_remain_alive_until_external_artifacts_are_collected() -> None:
    order: list[str] = []
    coordinator = _Coordinator(order, ready_on_prepare=True)
    coordinator.readiness = BindingReadiness.for_required(
        (BindingFacet.PHYSICS, BindingFacet.VISUAL)
    )
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=_Unreal(order),
        publisher=_Publisher(order),
        evidence_watchers=(_VisualReadinessEvidence(order, active_after=1),),
        sleep_s=0,
    )

    assert host.run_visual_only(
        frame_limit=1,
        steps_per_frame=2,
        close_on_finish=False,
    ) == 1
    assert "unreal.terminate" not in order
    assert "coordinator.stop" not in order

    host.close()
    assert order[-4:] == [
        "coordinator.pause",
        "unreal.terminate",
        "publisher.close",
        "coordinator.stop",
    ]


def test_reset_advances_watchers_before_publishing_and_applying_evidence() -> None:
    order: list[str] = []
    coordinator = _Coordinator(order, ready_on_prepare=True)
    publisher = _Publisher(order)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=_Unreal(order),
        publisher=publisher,
        evidence_watchers=(_Evidence(order, "camera", ready_after=99),),
        sleep_s=0,
    )
    host.prepare()
    coordinator.state = RuntimeState.READY
    order.clear()

    event = host.reset()

    assert event["reset_generation"] == 1
    assert publisher.snapshots[-1]["reset_generation"] == 1
    assert order == [
        "coordinator.reset",
        "evidence.advance:camera:0:1",
        "publisher.publish",
        "evidence.apply:camera",
    ]


def test_reset_ignores_real_stale_evidence_file_written_before_reset(
    tmp_path: Path,
) -> None:
    order: list[str] = []
    evidence_path = tmp_path / "sensor-readiness.json"
    coordinator = _Coordinator(order, ready_on_prepare=True)
    publisher = _Publisher(order)
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=_Unreal(order),
        publisher=publisher,
        evidence_watchers=(
            ExternalEvidenceWatcher(
                evidence_path,
                session_id="a" * 64,
                model_generation=0,
                reset_generation=0,
                expected_source_id="robotsimue-camera",
            ),
        ),
        sleep_s=0,
    )
    host.prepare()
    evidence_path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.sensor-readiness-evidence.v1",
                "session_id": "a" * 64,
                "model_generation": 0,
                "reset_generation": 0,
                "source_id": "robotsimue-camera",
                "basis": "real_rendered_frame_to_camera_shm",
                "visual": {"state": "PREPARING"},
                "sensors": {"camera_streams": "ACTIVE", "overall": "PREPARING"},
                "streams": [
                    {
                        "sensor_id": "thunder_01.front_rgb",
                        "state": "ACTIVE",
                        "published_frames": 1,
                        "last_sample_truth_sequence": 29,
                        "last_sample_sim_time_ns": 3_000_000_000,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    coordinator.state = RuntimeState.READY
    order.clear()

    event = host.reset()

    assert event["reset_generation"] == 1
    assert publisher.snapshots[-1]["reset_generation"] == 1
    assert order == ["coordinator.reset", "publisher.publish"]


def test_cleanup_error_does_not_cover_original_exception() -> None:
    order: list[str] = []
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order, exit_code=5),
        publisher=_Publisher(order, fail_close=True),
        sleep_s=0,
    )

    with pytest.raises(CoordinatorError, match="Unreal process exited during session run") as exc_info:
        host.run(frame_limit=1)

    assert "publisher close failed" in "\n".join(getattr(exc_info.value, "__notes__", ()))


@pytest.mark.parametrize(
    ("entrypoint", "ready_on_prepare", "expected_cleanup"),
    [
        (
            "prepare",
            False,
            ["unreal.terminate", "publisher.close", "coordinator.stop"],
        ),
        (
            "prepare_until_visual_applied",
            False,
            ["unreal.terminate", "publisher.close", "coordinator.stop"],
        ),
        (
            "run",
            True,
            [
                "coordinator.pause",
                "unreal.terminate",
                "publisher.close",
                "coordinator.stop",
            ],
        ),
        (
            "run_visual_only",
            False,
            ["unreal.terminate", "publisher.close", "coordinator.stop"],
        ),
    ],
)
def test_managed_entry_preserves_keyboard_interrupt_after_cleanup(
    entrypoint: str,
    ready_on_prepare: bool,
    expected_cleanup: list[str],
) -> None:
    order: list[str] = []
    interruption = KeyboardInterrupt()
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=ready_on_prepare),
        unreal_process=_Unreal(order),
        publisher=_Publisher(order, fail_publish=interruption),
        sleep_s=0,
    )

    with pytest.raises(KeyboardInterrupt) as exc_info:
        if entrypoint == "prepare":
            host.prepare()
        elif entrypoint == "prepare_until_visual_applied":
            host.prepare_until_visual_applied()
        elif entrypoint == "run":
            host.run(frame_limit=1)
        else:
            host.run_visual_only(frame_limit=1)

    assert exc_info.value is interruption
    assert order[-len(expected_cleanup) :] == expected_cleanup


def test_run_preserves_system_exit_after_cleanup() -> None:
    order: list[str] = []
    requested_exit = SystemExit(37)
    host = SessionHost(
        coordinator=_Coordinator(order, ready_on_prepare=True),
        unreal_process=_Unreal(order),
        publisher=_Publisher(order, fail_publish=requested_exit),
        sleep_s=0,
    )

    with pytest.raises(SystemExit) as exc_info:
        host.run(frame_limit=1)

    assert exc_info.value is requested_exit
    assert exc_info.value.code == 37
    assert order[-4:] == [
        "coordinator.pause",
        "unreal.terminate",
        "publisher.close",
        "coordinator.stop",
    ]


def test_unreal_command_uses_argv_and_preserves_paths_with_spaces() -> None:
    process = UnrealProcess(
        Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
        Path("D:/inovxio/brain/lingtu/sim/runtime/visual/RobotSimUE/RobotSimUE.uproject"),
        "/Game/Maps/Open Field",
    )
    allocation = SimpleNamespace(
        run_id="live-run-with-spaces",
        path=Path("C:/runs/live run/run-allocation.json"),
        artifact_root=Path("C:/artifact roots/live artifact root"),
        log_dir=Path("C:/runs/live run/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundles/session bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=5,
        reset_generation=2,
    )

    assert command[:4] == [
        "D:\\Program Files\\Epic Games\\UE_5.8\\Engine\\Binaries\\Win64\\UnrealEditor.exe",
        "D:\\inovxio\\brain\\lingtu\\sim\\runtime\\visual\\RobotSimUE\\RobotSimUE.uproject",
        "/Game/Maps/Open Field",
        "-game",
    ]
    assert "-log" in command
    assert "-abslog=C:\\runs\\live run\\logs\\Unreal.log" in command
    assert command.count("-unattended") == 1
    assert "-UnattendedInput" not in command
    assert "-LingTuRuntimeUI" not in command
    assert "-NoSound" not in command
    assert not any(argument.startswith("-ExecCmds=") for argument in command)
    assert "-nop4" in command
    assert "-NoSplash" in command
    assert command.count("-NoCompile") == 1
    assert command.count("-DDC=InstalledNoZenLocalFallback") == 1
    assert (
        "-LocalDataCachePath=C:\\artifact roots\\live artifact root\\build\\unreal-ddc"
        in command
    )
    assert "-windowed" in command
    assert "-ResX=1920" in command
    assert "-ResY=1080" in command
    assert "-ForceRes" in command
    assert "-LingTuBundle=C:\\bundles\\session bundle" in command
    assert "-LingTuRunAllocation=C:\\runs\\live run\\run-allocation.json" in command
    assert "-LingTuRunId=live-run-with-spaces" in command
    assert "-LingTuArtifactRoot=C:\\artifact roots\\live artifact root" in command
    assert "-LingTuSnapshotPort=25123" in command
    assert "-LingTuModelGeneration=5" in command
    assert "-LingTuResetGeneration=2" in command
    assert "-LingTuScreenshot=C:\\runs\\live run\\logs\\visual-first-frame.png" in command


def test_interactive_unreal_command_is_windowed_capped_and_not_unattended() -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        launch_profile=UnrealLaunchProfile.INTERACTIVE,
    )
    allocation = SimpleNamespace(
        run_id="interactive-playable",
        path=Path("C:/runs/interactive-playable/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/interactive-playable/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert command.count("-game") == 1
    assert command.count("-windowed") == 1
    assert command.count("-ResX=1920") == 1
    assert command.count("-ResY=1080") == 1
    assert "-unattended" not in command
    assert "-UnattendedInput" not in command
    assert "-LingTuRuntimeUI" not in command
    assert command.count("-NoSound") == 1
    assert [arg for arg in command if arg.startswith("-ExecCmds=")] == [
        "-ExecCmds=t.MaxFPS 30"
    ]


def test_main_view_screen_percentage_is_explicit_tsr_without_changing_output_size() -> None:
    allocation = SimpleNamespace(
        run_id="tsr-eighty-percent",
        path=Path("C:/runs/tsr-eighty-percent/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/tsr-eighty-percent/logs"),
    )
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        launch_profile=UnrealLaunchProfile.PLAYABLE_SDK_QUIET,
        main_view_screen_percentage=80,
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert [arg for arg in command if arg.startswith("-ExecCmds=")] == [
        "-ExecCmds=t.MaxFPS 30,r.AntiAliasingMethod 4,"
        "r.DynamicRes.OperationMode 0,r.ScreenPercentage 80"
    ]
    assert command.count("-ResX=1920") == 1
    assert command.count("-ResY=1080") == 1


@pytest.mark.parametrize("value", [True, 49, 101, 80.0])
def test_main_view_screen_percentage_rejects_invalid_values(value: object) -> None:
    with pytest.raises((TypeError, ValueError), match="main_view_screen_percentage"):
        UnrealProcess(
            Path("D:/UnrealEditor.exe"),
            Path("D:/RobotSimUE.uproject"),
            "/Game/Maps/FactoryPark_HF",
            main_view_screen_percentage=value,  # type: ignore[arg-type]
        )


def test_depth_main_renderer_is_explicit_and_emitted_once() -> None:
    allocation = SimpleNamespace(
        run_id="depth-main-renderer",
        path=Path("C:/runs/depth-main-renderer/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/depth-main-renderer/logs"),
    )
    default_process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
    )
    enabled_process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        depth_capture_in_main_renderer=True,
    )

    common = {
        "bundle_dir": Path("C:/bundle"),
        "allocation": allocation,
        "snapshot_port": 25123,
        "model_generation": 0,
        "reset_generation": 0,
    }
    assert "-LingTuDepthCaptureInMainRenderer" not in default_process.command(**common)
    assert enabled_process.command(**common).count(
        "-LingTuDepthCaptureInMainRenderer"
    ) == 1


def test_shared_color_depth_capture_is_explicit_and_emitted_once() -> None:
    allocation = SimpleNamespace(
        run_id="shared-color-depth",
        path=Path("C:/runs/shared-color-depth/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/shared-color-depth/logs"),
    )
    default_process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
    )
    enabled_process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        shared_color_depth_capture=True,
    )

    common = {
        "bundle_dir": Path("C:/bundle"),
        "allocation": allocation,
        "snapshot_port": 25123,
        "model_generation": 0,
        "reset_generation": 0,
    }
    assert "-LingTuSharedColorDepthCapture" not in default_process.command(**common)
    assert enabled_process.command(**common).count(
        "-LingTuSharedColorDepthCapture"
    ) == 1


@pytest.mark.parametrize("value", [None, 1, "true"])
def test_shared_color_depth_capture_rejects_non_boolean_values(value: object) -> None:
    with pytest.raises(TypeError, match="shared_color_depth_capture"):
        UnrealProcess(
            Path("D:/UnrealEditor.exe"),
            Path("D:/RobotSimUE.uproject"),
            "/Game/Maps/FactoryPark_HF",
            shared_color_depth_capture=value,  # type: ignore[arg-type]
        )


@pytest.mark.parametrize(
    ("launch_profile", "unattended", "unattended_input", "runtime_ui"),
    [
        (UnrealLaunchProfile.PLAYABLE_SDK_QUIET, True, True, True),
        (UnrealLaunchProfile.FOREGROUND_SDK_QUIET, True, False, True),
        (UnrealLaunchProfile.INTERACTIVE_SDK_QUIET, False, False, False),
    ],
)
def test_sdk_quiet_editor_start_skips_probes_with_exact_interaction_mode(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    launch_profile: UnrealLaunchProfile,
    unattended: bool,
    unattended_input: bool,
    runtime_ui: bool,
) -> None:
    popen_calls: list[dict[str, Any]] = []

    class FakeProcess:
        pid = 4106

        @staticmethod
        def poll() -> None:
            return None

    class FakeProcessOwner:
        def popen_options(self, *, creationflags: int = 0) -> dict[str, int]:
            return {"creationflags": creationflags}

        def attach(self, _process: FakeProcess) -> None:
            return None

        def terminate(self, _process: FakeProcess, *, timeout_s: float) -> None:
            del timeout_s

        def close_after_exit(self) -> None:
            return None

        def close(self) -> None:
            return None

    def fake_popen(command: list[str], **kwargs: Any) -> FakeProcess:
        popen_calls.append({"command": command, **kwargs})
        return FakeProcess()

    monkeypatch.setattr(unreal_process_module, "ProcessTreeOwner", FakeProcessOwner)
    log_dir = tmp_path / "run" / "logs"
    log_dir.mkdir(parents=True)
    allocation = SimpleNamespace(
        run_id="sdk-quiet-playable",
        path=tmp_path / "run" / "run-allocation.json",
        artifact_root=tmp_path / "artifacts",
        log_dir=log_dir,
        child_environment=lambda: {
            "LINGTU_HOST_BOOT_ID": "boot-sdk-quiet",
            "UE_SKIP_UBT_SDK_SETUP": "0",
            "ue_skip_ubt_sdk_setup": "inherited-case-variant",
        },
        ports={},
    )
    editor = tmp_path / "UnrealEditor.exe"
    project = tmp_path / "RobotSimUE.uproject"
    editor.write_bytes(b"")
    project.write_text("{}\n", encoding="utf-8")
    process = UnrealProcess(
        editor,
        project,
        "/Game/Maps/FactoryPark_HF",
        launch_profile=launch_profile,
        popen_factory=fake_popen,  # type: ignore[arg-type]
    )

    process.start(
        bundle_dir=tmp_path / "bundle",
        allocation=allocation,  # type: ignore[arg-type]
        plan=SimpleNamespace(model_generation=0, reset_generation=0, repo_root=tmp_path),
        snapshot_port=25123,
    )
    process.terminate()

    assert len(popen_calls) == 1
    popen_call = popen_calls[0]
    command = popen_call["command"]
    assert command.count("-unattended") == (1 if unattended else 0)
    assert command.count("-UnattendedInput") == (1 if unattended_input else 0)
    assert command.count("-LingTuRuntimeUI") == (1 if runtime_ui else 0)
    assert command.count("-NoSound") == 1
    assert [arg for arg in command if arg.startswith("-ExecCmds=")] == [
        "-ExecCmds=t.MaxFPS 30"
    ]
    assert popen_call["env"] == {
        "LINGTU_HOST_BOOT_ID": "boot-sdk-quiet",
        "UE_SKIP_UBT_SDK_SETUP": "1",
    }


def test_unreal_launch_profile_is_type_strict_and_constructor_is_inert(
    tmp_path: Path,
) -> None:
    absent_root = tmp_path / "constructor-must-not-create"

    with pytest.raises(TypeError, match="launch_profile must be UnrealLaunchProfile"):
        UnrealProcess(
            absent_root / "UnrealEditor.exe",
            absent_root / "RobotSimUE.uproject",
            "/Game/Maps/FactoryPark_HF",
            launch_profile="interactive",  # type: ignore[arg-type]
        )

    assert absent_root.exists() is False


@pytest.mark.parametrize("process_kind", ["editor", "packaged"])
def test_unreal_command_binds_playable_control_to_run_allocation_ports(
    process_kind: str,
) -> None:
    process: UnrealProcess | PackagedUnrealProcess
    if process_kind == "editor":
        process = UnrealProcess(
            Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
            Path("D:/inovxio/brain/lingtu/sim/runtime/visual/RobotSimUE/RobotSimUE.uproject"),
            "/Game/Maps/FactoryPark_HF",
        )
    else:
        process = PackagedUnrealProcess(
            Path("C:/trusted builds/RobotSimUE/RobotSimUE-Win64-Release.exe"),
            "/Game/Maps/FactoryPark_HF",
        )
    allocation = SimpleNamespace(
        run_id="playable-run-001",
        path=Path("C:/runs/playable-run-001/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/playable-run-001/logs"),
        ports={
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25124,
            "control_status_udp": 25125,
        },
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=7,
        reset_generation=3,
    )

    assert command.count("-LingTuControlIntentPort=25124") == 1
    assert command.count("-LingTuControlStatusPort=25125") == 1
    assert command.count("-LingTuControlSourceId=robotsimue.local_player.0") == 1
    expected_hud_arguments = {
        "-LingTuHudDriveScreenshot="
        "C:\\runs\\playable-run-001\\screenshots\\hud-drive.png",
        "-LingTuHudTacticalScreenshot="
        "C:\\runs\\playable-run-001\\screenshots\\hud-tactical.png",
        "-LingTuHudMenuRecordingScreenshot="
        "C:\\runs\\playable-run-001\\screenshots\\hud-menu-recording.png",
    }
    for argument in expected_hud_arguments:
        assert command.count(argument) == 1
    assert not any(argument.startswith("-LingTuHudScreenshot=") for argument in command)


@pytest.mark.parametrize("process_kind", ["editor", "packaged"])
def test_unreal_command_has_no_implicit_playable_control_ports(
    process_kind: str,
) -> None:
    process: UnrealProcess | PackagedUnrealProcess
    if process_kind == "editor":
        process = UnrealProcess(
            Path("D:/UnrealEditor.exe"),
            Path("D:/RobotSimUE.uproject"),
            "/Game/Maps/FactoryPark_HF",
        )
    else:
        process = PackagedUnrealProcess(
            Path("C:/RobotSimUE/RobotSimUE-Win64-Release.exe"),
            "/Game/Maps/FactoryPark_HF",
        )
    allocation = SimpleNamespace(
        run_id="viewer-run-001",
        path=Path("C:/runs/viewer-run-001/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/viewer-run-001/logs"),
        ports={"visual_snapshot_udp": 25123},
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert not any(argument.startswith("-LingTuControl") for argument in command)
    assert not any(argument.startswith("-LingTuHud") for argument in command)


@pytest.mark.parametrize(
    "ports",
    [
        {"visual_snapshot_udp": 25123, "control_intent_udp": 25124},
        {"visual_snapshot_udp": 25123, "control_status_udp": 25125},
        {
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 0,
            "control_status_udp": 25125,
        },
        {
            "visual_snapshot_udp": 25123,
            "control_intent_udp": True,
            "control_status_udp": 25125,
        },
        {
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25124,
            "control_status_udp": 25124,
        },
        {
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25123,
            "control_status_udp": 25125,
        },
    ],
)
def test_unreal_command_rejects_partial_invalid_or_colliding_control_ports(
    ports: dict[str, int],
) -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
    )
    allocation = SimpleNamespace(
        run_id="playable-run-invalid",
        path=Path("C:/runs/playable-run-invalid/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/playable-run-invalid/logs"),
        ports=ports,
    )

    with pytest.raises(ValueError, match=r"control.*port|distinct"):
        process.command(
            bundle_dir=Path("C:/bundle"),
            allocation=allocation,  # type: ignore[arg-type]
            snapshot_port=25123,
            model_generation=0,
            reset_generation=0,
        )


@pytest.mark.parametrize("snapshot_port", [0, 65536])
def test_unreal_command_rejects_out_of_range_snapshot_port_with_playable_control(
    snapshot_port: int,
) -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
    )
    allocation = SimpleNamespace(
        run_id="playable-run-invalid-snapshot",
        path=Path("C:/runs/playable-run-invalid-snapshot/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/playable-run-invalid-snapshot/logs"),
        ports={
            "visual_snapshot_udp": snapshot_port,
            "control_intent_udp": 25124,
            "control_status_udp": 25125,
        },
    )

    with pytest.raises(ValueError, match="snapshot_port"):
        process.command(
            bundle_dir=Path("C:/bundle"),
            allocation=allocation,  # type: ignore[arg-type]
            snapshot_port=snapshot_port,
            model_generation=0,
            reset_generation=0,
        )


def test_unreal_command_rejects_snapshot_port_mismatching_run_allocation() -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
    )
    allocation = SimpleNamespace(
        run_id="playable-run-mismatched-snapshot",
        path=Path("C:/runs/playable-run-mismatched-snapshot/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/playable-run-mismatched-snapshot/logs"),
        ports={
            "visual_snapshot_udp": 25123,
            "control_intent_udp": 25124,
            "control_status_udp": 25125,
        },
    )

    with pytest.raises(ValueError, match="snapshot_port does not match"):
        process.command(
            bundle_dir=Path("C:/bundle"),
            allocation=allocation,  # type: ignore[arg-type]
            snapshot_port=25126,
            model_generation=0,
            reset_generation=0,
        )


@pytest.mark.parametrize("run_id", ["", "   ", "../other-run", "run/id"])
def test_unreal_command_rejects_noncanonical_allocation_run_id(run_id: str) -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/OpenField_HF",
    )
    allocation = SimpleNamespace(
        run_id=run_id,
        path=Path("C:/runs/live/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/live/logs"),
    )

    with pytest.raises(ValueError, match="allocation run_id"):
        process.command(
            bundle_dir=Path("C:/bundle"),
            allocation=allocation,  # type: ignore[arg-type]
            snapshot_port=25123,
            model_generation=0,
            reset_generation=0,
        )


def test_unreal_command_can_enable_bounded_frame_capture() -> None:
    process = UnrealProcess(
        Path("D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe"),
        Path("D:/inovxio/brain/lingtu/sim/runtime/visual/RobotSimUE/RobotSimUE.uproject"),
        "/Game/Maps/OpenField_HF",
        frame_capture_dir=Path("C:/runs/live run/frame capture"),
        frame_capture_every=4,
        frame_capture_max=90,
    )
    allocation = SimpleNamespace(
        run_id="frame-capture-run",
        path=Path("C:/runs/live run/run-allocation.json"),
        artifact_root=Path("C:/artifact roots/live artifact root"),
        log_dir=Path("C:/runs/live run/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundles/session bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=5,
        reset_generation=2,
    )

    assert "-LingTuFrameCaptureDir=C:\\runs\\live run\\frame capture" in command
    assert "-LingTuFrameCaptureEvery=4" in command
    assert "-LingTuFrameCaptureMax=90" in command


@pytest.mark.parametrize(
    ("frame_capture_dir", "frame_capture_every", "frame_capture_max"),
    [
        (Path("capture"), 0, 90),
        (Path("capture"), True, 90),
        (Path("capture"), 4, 0),
        (Path("capture"), 4, True),
        (Path("   "), 4, 90),
    ],
)
def test_unreal_frame_capture_requires_a_directory_and_positive_integer_bounds(
    frame_capture_dir: Path,
    frame_capture_every: int,
    frame_capture_max: int,
) -> None:
    with pytest.raises(ValueError, match="frame capture"):
        UnrealProcess(
            Path("D:/UnrealEditor.exe"),
            Path("D:/RobotSimUE.uproject"),
            "/Game/Maps/OpenField_HF",
            frame_capture_dir=frame_capture_dir,
            frame_capture_every=frame_capture_every,
            frame_capture_max=frame_capture_max,
        )


def test_unreal_command_can_select_an_exact_session_camera_tag() -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        session_camera_tag="PreviewTarget:south_gate_robot_eye",
    )
    allocation = SimpleNamespace(
        run_id="session-camera-run",
        path=Path("C:/runs/live/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/live/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert (
        "-LingTuSessionCameraTag=PreviewTarget:south_gate_robot_eye" in command
    )


def test_unreal_session_camera_tag_rejects_whitespace() -> None:
    with pytest.raises(ValueError, match="session camera tag"):
        UnrealProcess(
            Path("D:/UnrealEditor.exe"),
            Path("D:/RobotSimUE.uproject"),
            "/Game/Maps/FactoryPark_HF",
            session_camera_tag="   ",
        )


def test_unreal_command_can_request_a_fixed_motion_camera() -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
        motion_camera_stable_id="thunder_01/base_link",
    )
    allocation = SimpleNamespace(
        run_id="motion-camera-run",
        path=Path("C:/runs/live/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/live/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert "-LingTuMotionCameraStableId=thunder_01/base_link" in command


def test_unreal_motion_camera_stable_id_rejects_whitespace() -> None:
    with pytest.raises(ValueError, match="motion camera stable ID"):
        UnrealProcess(
            Path("D:/UnrealEditor.exe"),
            Path("D:/RobotSimUE.uproject"),
            "/Game/Maps/FactoryPark_HF",
            motion_camera_stable_id="   ",
        )


def test_unreal_recording_options_are_absent_by_default() -> None:
    process = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/Maps/FactoryPark_HF",
    )
    allocation = SimpleNamespace(
        run_id="recording-options-run",
        path=Path("C:/runs/live/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/live/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert not any(argument.startswith("-LingTuFrameCapture") for argument in command)
    assert not any(argument.startswith("-LingTuSessionCameraTag") for argument in command)
    assert not any(argument.startswith("-LingTuMotionCameraStableId") for argument in command)


def test_unreal_command_resolves_relative_process_paths(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.chdir(tmp_path)
    process = UnrealProcess(
        Path("tools/UnrealEditor.exe"),
        Path("projects/RobotSimUE.uproject"),
        "/Game/Maps/OpenField_HF",
    )
    allocation = SimpleNamespace(
        run_id="relative-path-run",
        path=tmp_path / "runs/run-allocation.json",
        artifact_root=tmp_path / "artifact-root",
        log_dir=tmp_path / "runs/logs",
    )

    command = process.command(
        bundle_dir=tmp_path / "bundle",
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=0,
        reset_generation=0,
    )

    assert command[0] == str((tmp_path / "tools/UnrealEditor.exe").resolve())
    assert command[1] == str((tmp_path / "projects/RobotSimUE.uproject").resolve())


def test_packaged_unreal_command_uses_exe_map_and_runtime_identity_only() -> None:
    process = PackagedUnrealProcess(
        Path("C:/trusted builds/RobotSimUE/RobotSimUE-Win64-Release.exe"),
        "/Game/Maps/FactoryPark_HF",
        frame_capture_dir=Path("C:/runs/live run/frame capture"),
        frame_capture_every=4,
        frame_capture_max=90,
        session_camera_tag="PreviewTarget:south_gate_robot_eye",
        motion_camera_stable_id="thunder_01/base_link",
    )
    allocation = SimpleNamespace(
        run_id="packaged-run-001",
        path=Path("C:/runs/live run/run-allocation.json"),
        artifact_root=Path("C:/artifact roots/live artifact root"),
        log_dir=Path("C:/runs/live run/logs"),
    )

    command = process.command(
        bundle_dir=Path("C:/bundles/session bundle"),
        allocation=allocation,  # type: ignore[arg-type]
        snapshot_port=25123,
        model_generation=5,
        reset_generation=2,
    )

    assert command[:2] == [
        "C:\\trusted builds\\RobotSimUE\\RobotSimUE-Win64-Release.exe",
        "/Game/Maps/FactoryPark_HF",
    ]
    assert not any(argument.endswith(".uproject") for argument in command)
    assert "-game" not in command
    assert "-nop4" not in command
    assert "-NoCompile" not in command
    assert not any(argument.startswith("-DDC=") for argument in command)
    assert not any(argument.startswith("-LocalDataCachePath=") for argument in command)
    assert "-abslog=C:\\runs\\live run\\logs\\RobotSimUE.log" in command
    assert command.count("-unattended") == 1
    assert "-NoSound" not in command
    assert not any(argument.startswith("-ExecCmds=") for argument in command)
    assert "-LingTuBundle=C:\\bundles\\session bundle" in command
    assert "-LingTuRunAllocation=C:\\runs\\live run\\run-allocation.json" in command
    assert "-LingTuRunId=packaged-run-001" in command
    assert "-LingTuArtifactRoot=C:\\artifact roots\\live artifact root" in command
    assert "-LingTuSnapshotPort=25123" in command
    assert "-LingTuModelGeneration=5" in command
    assert "-LingTuResetGeneration=2" in command
    assert "-LingTuScreenshot=C:\\runs\\live run\\logs\\visual-first-frame.png" in command
    assert "-LingTuFrameCaptureDir=C:\\runs\\live run\\frame capture" in command
    assert "-LingTuFrameCaptureEvery=4" in command
    assert "-LingTuFrameCaptureMax=90" in command
    assert "-LingTuSessionCameraTag=PreviewTarget:south_gate_robot_eye" in command
    assert "-LingTuMotionCameraStableId=thunder_01/base_link" in command


@pytest.mark.parametrize("run_id", ["", "   ", "../other-run", "run/id"])
def test_packaged_unreal_command_rejects_noncanonical_allocation_run_id(
    run_id: str,
) -> None:
    process = PackagedUnrealProcess(
        Path("C:/trusted builds/RobotSimUE/RobotSimUE-Win64-Release.exe"),
        "/Game/Maps/OpenField_HF",
    )
    allocation = SimpleNamespace(
        run_id=run_id,
        path=Path("C:/runs/live/run-allocation.json"),
        artifact_root=Path("C:/artifact-root"),
        log_dir=Path("C:/runs/live/logs"),
    )

    with pytest.raises(ValueError, match="allocation run_id"):
        process.command(
            bundle_dir=Path("C:/bundle"),
            allocation=allocation,  # type: ignore[arg-type]
            snapshot_port=25123,
            model_generation=0,
            reset_generation=0,
        )


@pytest.mark.parametrize(
    ("exe", "map_name", "message"),
    [
        (
            Path("C:/trusted builds/RobotSimUE/Other.exe"),
            "/Game/Maps/OpenField_HF",
            "RobotSimUE-Win64-Release.exe",
        ),
        (
            Path("C:/trusted builds/RobotSimUE/RobotSimUE.exe"),
            "/Game/Maps/OpenField_HF",
            "RobotSimUE-Win64-Release.exe",
        ),
        (
            Path(
                "C:/trusted builds/RobotSimUE/"
                "RobotSimUE-Win64-Shipping.exe"
            ),
            "/Game/Maps/OpenField_HF",
            "RobotSimUE-Win64-Release.exe",
        ),
        (
            Path("C:/trusted builds/RobotSimUE/RobotSimUE-Win64-Release.exe"),
            "",
            "map_name",
        ),
        (
            Path("C:/trusted builds/RobotSimUE/RobotSimUE-Win64-Release.exe"),
            "   ",
            "map_name",
        ),
    ],
)
def test_packaged_unreal_validates_exe_and_map_identity(
    exe: Path,
    map_name: str,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        PackagedUnrealProcess(exe, map_name)


def test_packaged_unreal_start_owns_process_logs_env_and_no_shell(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    calls: list[Any] = []

    class FakeProcess:
        pid = 4104

        def poll(self) -> int | None:
            return None

    class FakeProcessOwner:
        def popen_options(self, *, creationflags: int = 0) -> dict[str, int]:
            calls.append(("popen_options", creationflags))
            return {"creationflags": creationflags}

        def attach(self, process: FakeProcess) -> None:
            calls.append(("attach", process.pid))

        def terminate(self, process: FakeProcess, *, timeout_s: float) -> None:
            calls.append(("terminate", process.pid, timeout_s))

        def close_after_exit(self) -> None:
            calls.append("close_after_exit")

        def close(self) -> None:
            calls.append("close")

    popen_calls: list[dict[str, Any]] = []

    def fake_popen(command: list[str], **kwargs: Any) -> FakeProcess:
        popen_calls.append({"command": command, **kwargs})
        return FakeProcess()

    monkeypatch.setattr(unreal_process_module, "ProcessTreeOwner", FakeProcessOwner)
    exe = tmp_path / "trusted package" / "RobotSimUE-Win64-Release.exe"
    exe.parent.mkdir(parents=True)
    exe.write_text("", encoding="utf-8")
    log_dir = tmp_path / "run" / "logs"
    log_dir.mkdir(parents=True)
    allocation = SimpleNamespace(
        run_id="packaged-start-run",
        path=tmp_path / "run" / "run-allocation.json",
        artifact_root=tmp_path / "artifacts",
        log_dir=log_dir,
        child_environment=lambda: {"LINGTU_HOST_BOOT_ID": "boot-packaged"},
    )
    process = PackagedUnrealProcess(
        exe,
        "/Game/Maps/OpenField_HF",
        popen_factory=fake_popen,  # type: ignore[arg-type]
        timeout_s=3.5,
    )

    process.start(
        bundle_dir=tmp_path / "bundle",
        allocation=allocation,  # type: ignore[arg-type]
        plan=SimpleNamespace(model_generation=9, reset_generation=4),
        snapshot_port=25123,
    )
    process.terminate()

    assert popen_calls
    popen_call = popen_calls[0]
    assert popen_call["command"][:2] == [str(exe.resolve()), "/Game/Maps/OpenField_HF"]
    assert popen_call["cwd"] == exe.parent.resolve()
    assert popen_call["stdin"] is subprocess.DEVNULL
    assert popen_call["stdout"] is subprocess.DEVNULL
    assert popen_call["shell"] is False
    assert popen_call["env"] == {"LINGTU_HOST_BOOT_ID": "boot-packaged"}
    assert popen_call["stderr"].name == str(log_dir / "robotsimue.stderr.log")
    assert calls[-3:] == [("attach", 4104), ("terminate", 4104, 3.5), "close_after_exit"]


def test_packaged_unreal_configures_job_affinity_before_popen_and_attach(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    calls: list[Any] = []

    class FakeProcess:
        pid = 4105

        @staticmethod
        def poll() -> None:
            return None

    class FakeProcessOwner:
        def __init__(self, *, affinity_mask: int) -> None:
            calls.append(("owner", affinity_mask))

        def popen_options(self, *, creationflags: int = 0) -> dict[str, int]:
            calls.append(("popen_options", creationflags))
            return {"creationflags": creationflags}

        def attach(self, process: FakeProcess) -> None:
            calls.append(("attach", process.pid))

        def terminate(self, process: FakeProcess, *, timeout_s: float) -> None:
            calls.append(("terminate", process.pid, timeout_s))

        def close_after_exit(self) -> None:
            calls.append("close_after_exit")

        def close(self) -> None:
            calls.append("close")

    def fake_popen(_command: list[str], **_kwargs: Any) -> FakeProcess:
        calls.append("popen")
        return FakeProcess()

    monkeypatch.setattr(unreal_process_module, "ProcessTreeOwner", FakeProcessOwner)
    exe = tmp_path / "trusted" / "RobotSimUE-Win64-Release.exe"
    exe.parent.mkdir(parents=True)
    exe.write_text("", encoding="utf-8")
    log_dir = tmp_path / "run" / "logs"
    log_dir.mkdir(parents=True)
    allocation = SimpleNamespace(
        run_id="affinity-order",
        path=tmp_path / "run" / "run-allocation.json",
        artifact_root=tmp_path / "artifacts",
        log_dir=log_dir,
        child_environment=lambda: {},
    )
    process = PackagedUnrealProcess(
        exe,
        "/Game/Maps/OpenField_HF",
        popen_factory=fake_popen,  # type: ignore[arg-type]
        affinity_mask=0b0011,
    )

    process.start(
        bundle_dir=tmp_path / "bundle",
        allocation=allocation,  # type: ignore[arg-type]
        plan=SimpleNamespace(model_generation=0, reset_generation=0),
        snapshot_port=25123,
    )
    process.terminate()

    assert calls[:4] == [
        ("owner", 0b0011),
        ("popen_options", getattr(subprocess, "CREATE_NO_WINDOW", 0)),
        "popen",
        ("attach", 4105),
    ]


class _PhysicsHost:
    def __init__(self) -> None:
        self.pid: int | None = 8102
        self.calls: list[Any] = []
        self.session_id = ""
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0
        self.coordinator: RuntimeCoordinator | None = None

    def prepare(self, plan: Any, allocation: Any) -> dict[str, Any]:
        del allocation
        self.calls.append("prepare")
        self.session_id = plan.session_id
        return self._event("ready")

    def bind_actuators(self, **binding: Any) -> dict[str, Any]:
        self.calls.append(("bind_actuators", binding["source_id"]))
        return {
            "event": "actuator_bound",
            "source_id": binding["source_id"],
            "instance_id": binding["instance_id"],
            "command_type": binding["command_type"],
            "channel_count": len(binding["channels"]),
        }

    def apply_actuator_command(self, command: Any) -> dict[str, Any]:
        self.calls.append(("actuate", command.sequence))
        return {
            "event": "actuator_command",
            "source_id": command.controller_id,
            "sequence": command.sequence,
            "result": "applied",
        }

    def start(self) -> dict[str, Any]:
        if self.coordinator is not None:
            assert self.coordinator.state is RuntimeState.PREPARING
        self.calls.append("start")
        return self._event("running")

    def advance(self, steps: int) -> dict[str, Any]:
        if self.coordinator is not None:
            assert self.coordinator.state is RuntimeState.PREPARING
        self.calls.append(("advance", steps))
        self.sequence += 1
        self.physics_step += steps
        self.sim_time_ns += 2_000_000
        return self._snapshot()

    def snapshot(self) -> dict[str, Any]:
        self.calls.append("snapshot")
        return self._snapshot()

    def pause(self) -> dict[str, Any]:
        if self.coordinator is not None:
            assert self.coordinator.state is RuntimeState.PREPARING
        self.calls.append("pause")
        return self._event("paused")

    def reset(self) -> dict[str, Any]:
        raise NotImplementedError

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        self.pid = None
        return self._event("stopped")

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": self.session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": self.sequence,
            "physics_step": self.physics_step,
            "sim_time_ns": self.sim_time_ns,
        }

    def _snapshot(self) -> dict[str, Any]:
        return {
            **self._event("snapshot"),
            "bodies": [
                {
                    "stable_id": "thunder_01/base_link",
                    "instance_id": "thunder_01",
                    "frame_id": "base_link",
                    "position_m": [0.0, 0.0, 0.45],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "linear_velocity_mps": [0.0, 0.0, 0.0],
                    "angular_velocity_rps": [0.0, 0.0, 0.0],
                }
            ],
            "joints": [
                {
                    "stable_id": f"thunder_01/{channel}",
                    "instance_id": "thunder_01",
                    "position_rad": [0.0],
                    "velocity_rps": [0.0],
                }
                for channel in ACTUATORS
            ],
            "actuators": [],
        }


def _bundle(tmp_path: Path) -> Path:
    return CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(tmp_path / "bundle")


def _contract_bundle(tmp_path: Path) -> Path:
    return CatalogResolver.from_repository(REPO_ROOT).resolve(CONTRACT_SESSION).write_bundle(tmp_path / "bundle")


def test_coordinator_warmup_advances_controls_without_exposing_running() -> None:
    test_root = REPO_ROOT / "tmp" / f"session-host-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        physics = _PhysicsHost()
        coordinator = RuntimeCoordinator(
            bundle_dir=_bundle(test_root),
            repo_root=REPO_ROOT,
            run_root=test_root / "runs",
            physics_host=physics,
            controller_factory=zero_output_components,
            run_id="warmup-state",
        )
        physics.coordinator = coordinator
        coordinator.prepare()

        assert coordinator.state is RuntimeState.PREPARING
        snapshot = coordinator.warmup(2)

        assert snapshot["event"] == "snapshot"
        assert coordinator.state is RuntimeState.PREPARING
        assert "start" in physics.calls
        assert "pause" in physics.calls
        assert ("advance", 1) in physics.calls
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_keyboard_interrupt_commits_failed_episode_without_hiding_interrupt() -> None:
    test_root = REPO_ROOT / "tmp" / f"session-host-interrupt-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        order: list[str] = []
        physics = _PhysicsHost()
        coordinator = RuntimeCoordinator(
            bundle_dir=_bundle(test_root),
            repo_root=REPO_ROOT,
            run_root=test_root / "runs",
            physics_host=physics,
            controller_factory=zero_output_components,
            run_id="interrupt-manifest",
            dds_domain=127,
            ports={"visual_snapshot_udp": 25127},
        )
        interruption = KeyboardInterrupt()
        publisher = _Publisher(order, fail_publish=interruption)
        host = SessionHost(
            coordinator=coordinator,
            unreal_process=_Unreal(order),
            publisher=publisher,
            sleep_s=0,
        )

        with pytest.raises(KeyboardInterrupt) as error:
            host.prepare()

        assert error.value is interruption
        manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
        assert coordinator.state is RuntimeState.FAILED
        assert manifest["state"] == "FAILED"
        assert manifest["allocation"]["dds_domain"] == 127
        episode = json.loads((coordinator.allocation.run_dir / "episode_result.json").read_text(encoding="utf-8"))
        assert episode["status"] == "FAILED"
        assert "KeyboardInterrupt" in episode["failure_reason"]
        assert order[-2:] == ["unreal.terminate", "publisher.close"]
        assert physics.calls.count("stop") == 1
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_external_camera_batch_failure_preserves_first_terminal_reason() -> None:
    test_root = REPO_ROOT / "tmp" / f"session-host-camera-failure-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        physics = _PhysicsHost()
        coordinator = RuntimeCoordinator(
            bundle_dir=_bundle(test_root),
            repo_root=REPO_ROOT,
            run_root=test_root / "runs",
            physics_host=physics,
            controller_factory=zero_output_components,
            run_id="camera-batch-failure",
            dds_domain=125,
            ports={"visual_snapshot_udp": 25125},
        )
        coordinator.prepare()
        evidence_path = coordinator.allocation.log_dir / "sensor-readiness.json"
        evidence_path.write_text(
            json.dumps(
                {
                    "schema": "lingtu.sim.sensor-readiness-evidence.v1",
                    "session_id": coordinator.plan.session_id,
                    "model_generation": 0,
                    "reset_generation": 0,
                    "source_id": "robotsimue-camera",
                    "basis": "real_rendered_frame_to_camera_shm",
                    "visual": {"state": "PREPARED"},
                    "sensors": {
                        "camera_streams": "FAILED",
                        "overall": "FAILED",
                    },
                    "streams": [
                        {
                            "sensor_id": "thunder_01.front_rgb",
                            "state": "FAILED",
                            "published_frames": 0,
                            "last_sample_truth_sequence": 0,
                            "last_sample_sim_time_ns": 0,
                            "reason": "rgb GPU readback timed out",
                        },
                        {
                            "sensor_id": "thunder_01.front_depth",
                            "state": "FAILED",
                            "published_frames": 0,
                            "last_sample_truth_sequence": 0,
                            "last_sample_sim_time_ns": 0,
                            "reason": "rgb GPU readback timed out",
                        },
                    ],
                }
            ),
            encoding="utf-8",
        )
        watcher = ExternalEvidenceWatcher(
            evidence_path,
            session_id=coordinator.plan.session_id,
            model_generation=0,
            reset_generation=0,
            expected_source_id="robotsimue-camera",
        )

        assert watcher.apply(coordinator) is True

        assert coordinator.state is RuntimeState.FAILED
        episode = json.loads(
            (coordinator.allocation.run_dir / "episode_result.json").read_text(
                encoding="utf-8"
            )
        )
        assert episode["status"] == "FAILED"
        assert episode["failure_reason"] == (
            "thunder_01.front_rgb: rgb GPU readback timed out"
        )
        manifest = coordinator.runtime_manifest_snapshot()
        assert manifest["sensor_streams"]["failures"] == {
            "thunder_01.front_depth": "rgb GPU readback timed out",
            "thunder_01.front_rgb": "rgb GPU readback timed out",
        }
        assert physics.calls.count("stop") == 1

        with pytest.raises(CoordinatorError, match="session_id"):
            coordinator.report_sensor_stream_failed(
                "thunder_01.front_depth",
                source_id="robotsimue-camera",
                reason="forged post-terminal failure",
                session_id="b" * 64,
                model_generation=0,
                reset_generation=0,
            )
        with pytest.raises(CoordinatorError, match="invalid in state FAILED"):
            coordinator.report_sensor_stream_active(
                "thunder_01.front_depth",
                source_id="robotsimue-camera",
                session_id=coordinator.plan.session_id,
                model_generation=0,
                reset_generation=0,
            )
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def _prepared_camera_failure_coordinator(
    test_root: Path,
    *,
    run_id: str,
    dds_domain: int,
    snapshot_port: int,
) -> tuple[RuntimeCoordinator, _PhysicsHost]:
    physics = _PhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(test_root),
        repo_root=REPO_ROOT,
        run_root=test_root / "runs",
        physics_host=physics,
        controller_factory=zero_output_components,
        run_id=run_id,
        dds_domain=dds_domain,
        ports={"visual_snapshot_udp": snapshot_port},
    )
    coordinator.prepare()
    return coordinator, physics


def _report_rgb_terminal_failure(coordinator: RuntimeCoordinator) -> None:
    coordinator.report_sensor_stream_failed(
        "thunder_01.front_rgb",
        source_id="robotsimue-camera",
        reason="rgb GPU readback timed out",
        session_id=coordinator.plan.session_id,
        model_generation=0,
        reset_generation=0,
    )
    assert coordinator.state is RuntimeState.FAILED


def test_post_terminal_sensor_failure_append_requires_the_first_failure_source() -> None:
    test_root = REPO_ROOT / "tmp" / f"sensor-failure-source-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        coordinator, physics = _prepared_camera_failure_coordinator(
            test_root,
            run_id="sensor-failure-source",
            dds_domain=124,
            snapshot_port=25124,
        )
        _report_rgb_terminal_failure(coordinator)

        with pytest.raises(CoordinatorError, match="first failed stream source"):
            coordinator.report_sensor_stream_failed(
                "thunder_01.front_depth",
                source_id="foreign-camera",
                reason="foreign failure",
                session_id=coordinator.plan.session_id,
                model_generation=0,
                reset_generation=0,
            )

        assert coordinator.sensor_readiness.state("thunder_01.front_depth").value == "UNBOUND"
        assert physics.calls.count("stop") == 1
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_post_terminal_sensor_failure_append_requires_a_prior_failed_stream() -> None:
    test_root = REPO_ROOT / "tmp" / f"sensor-failure-prior-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        coordinator, _physics = _prepared_camera_failure_coordinator(
            test_root,
            run_id="sensor-failure-prior",
            dds_domain=123,
            snapshot_port=25123,
        )
        coordinator.stop(failure_reason="unrelated terminal failure")

        with pytest.raises(CoordinatorError, match="prior failed sensor stream"):
            coordinator.report_sensor_stream_failed(
                "thunder_01.front_depth",
                source_id="robotsimue-camera",
                reason="late camera failure",
                session_id=coordinator.plan.session_id,
                model_generation=0,
                reset_generation=0,
            )
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("session_id", "b" * 64, "session_id"),
        ("model_generation", 1, "model_generation"),
        ("reset_generation", 1, "reset_generation"),
    ),
)
def test_post_terminal_sensor_failure_append_preserves_generation_identity(
    field: str,
    value: str | int,
    message: str,
) -> None:
    test_root = REPO_ROOT / "tmp" / f"sensor-failure-identity-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        coordinator, _physics = _prepared_camera_failure_coordinator(
            test_root,
            run_id="sensor-failure-identity",
            dds_domain=122,
            snapshot_port=25122,
        )
        _report_rgb_terminal_failure(coordinator)
        evidence: dict[str, object] = {
            "source_id": "robotsimue-camera",
            "reason": "depth failure",
            "session_id": coordinator.plan.session_id,
            "model_generation": 0,
            "reset_generation": 0,
        }
        evidence[field] = value

        with pytest.raises(CoordinatorError, match=message):
            coordinator.report_sensor_stream_failed(
                "thunder_01.front_depth",
                **evidence,  # type: ignore[arg-type]
            )
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


@pytest.mark.parametrize(
    "report_name",
    (
        "report_sensor_stream_prepared",
        "report_sensor_stream_active",
        "report_sensor_stream_retracted",
    ),
)
def test_post_terminal_nonfailure_sensor_evidence_remains_rejected(
    report_name: str,
) -> None:
    test_root = REPO_ROOT / "tmp" / f"sensor-nonfailure-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        coordinator, _physics = _prepared_camera_failure_coordinator(
            test_root,
            run_id="sensor-nonfailure",
            dds_domain=121,
            snapshot_port=25121,
        )
        _report_rgb_terminal_failure(coordinator)
        report = getattr(coordinator, report_name)

        with pytest.raises(CoordinatorError, match="invalid in state FAILED"):
            report(
                "thunder_01.front_depth",
                source_id="robotsimue-camera",
                session_id=coordinator.plan.session_id,
                model_generation=0,
                reset_generation=0,
            )
    finally:
        shutil.rmtree(test_root, ignore_errors=True)


def test_post_terminal_failed_stream_reason_is_immutable() -> None:
    test_root = REPO_ROOT / "tmp" / f"sensor-failure-reason-{uuid.uuid4().hex}"
    test_root.mkdir(parents=True, exist_ok=False)
    try:
        coordinator, physics = _prepared_camera_failure_coordinator(
            test_root,
            run_id="sensor-failure-reason",
            dds_domain=120,
            snapshot_port=25120,
        )
        _report_rgb_terminal_failure(coordinator)

        with pytest.raises(CoordinatorError, match="already FAILED"):
            coordinator.report_sensor_stream_failed(
                "thunder_01.front_rgb",
                source_id="robotsimue-camera",
                reason="replacement reason",
                session_id=coordinator.plan.session_id,
                model_generation=0,
                reset_generation=0,
            )

        assert coordinator.sensor_readiness.failure_reason("thunder_01.front_rgb") == (
            "rgb GPU readback timed out"
        )
        episode = json.loads(
            (coordinator.allocation.run_dir / "episode_result.json").read_text(
                encoding="utf-8"
            )
        )
        assert episode["failure_reason"] == (
            "thunder_01.front_rgb: rgb GPU readback timed out"
        )
        assert physics.calls.count("stop") == 1
    finally:
        shutil.rmtree(test_root, ignore_errors=True)
