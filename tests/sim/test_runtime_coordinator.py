# ruff: noqa: S101

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import pytest

import sim.runtime.coordinator.coordinator as coordinator_module
from sim.catalog import CatalogResolver
from sim.runtime.coordinator import CoordinatorError, RuntimeCoordinator, RuntimeState

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunder_omni_contract" / "session.yaml"
PEDESTRIAN_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "open_field_pedestrian_crossing"
    / "session.yaml"
)
PAYLOAD_SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_rws01_preview"
    / "session.yaml"
)


class FakePhysicsHost:
    def __init__(self) -> None:
        self.pid = 4242
        self.calls: list[object] = []
        self.session_id = ""
        self.model_generation = 0
        self.reset_generation = 0
        self.physics_step = 0

    def prepare(self, plan: object, allocation: object) -> dict[str, object]:
        self.calls.append(("prepare", plan, allocation))
        self.session_id = plan.session_id  # type: ignore[attr-defined]
        self.model_generation = plan.model_generation  # type: ignore[attr-defined]
        assert [robot.instance_id for robot in plan.robots] == [  # type: ignore[attr-defined]
            "thunder_01",
            "cart_01",
        ]
        assert plan.world_model_path.is_absolute()  # type: ignore[attr-defined]
        assert all(robot.model_path.is_absolute() for robot in plan.robots)  # type: ignore[attr-defined]
        assert allocation.log_dir.is_dir()  # type: ignore[attr-defined]
        return self._event("ready")

    def start(self) -> dict[str, object]:
        self.calls.append("start")
        return self._event("running")

    def advance(self, steps: int) -> dict[str, object]:
        self.calls.append(("advance", steps))
        self.physics_step += steps
        event = self._event("snapshot")
        event["bodies"] = []
        return event

    def pause(self) -> dict[str, object]:
        self.calls.append("pause")
        return self._event("paused")

    def reset(self) -> dict[str, object]:
        self.calls.append("reset")
        self.reset_generation += 1
        self.physics_step = 0
        event = self._event("snapshot")
        event["bodies"] = []
        return event

    def stop(self) -> dict[str, object]:
        self.calls.append("stop")
        event = self._event("stopped")
        self.pid = None
        return event

    def _event(self, event: str) -> dict[str, object]:
        return {
            "event": event,
            "session_id": self.session_id,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "physics_step": self.physics_step,
            "sequence": 0,
            "sim_time_ns": 0,
        }


class PrepareFailingPhysicsHost(FakePhysicsHost):
    def prepare(self, plan: object, allocation: object) -> dict[str, object]:
        self.calls.append(("prepare", plan, allocation))
        self.session_id = plan.session_id  # type: ignore[attr-defined]
        self.model_generation = plan.model_generation  # type: ignore[attr-defined]
        raise CoordinatorError("physics prepare failed")


def _bundle(tmp_path: Path) -> Path:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    bundle = resolved.write_bundle(tmp_path / "bundle")
    (bundle / "session.yaml").write_text(SESSION.read_text(encoding="utf-8"), encoding="utf-8")
    return bundle


def test_physics_plan_loads_package_declared_kinematic_entities(tmp_path: Path) -> None:
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(
        PEDESTRIAN_SESSION
    ).write_bundle(tmp_path / "pedestrian-bundle")

    plan = coordinator_module.load_physics_plan(bundle, REPO_ROOT)

    assert len(plan.kinematic_entities) == 1
    pedestrian = plan.kinematic_entities[0]
    assert pedestrian.entity_id == "pedestrian_01"
    assert pedestrian.attach_root == "proxy_root"
    assert pedestrian.model_path == (
        REPO_ROOT
        / "sim"
        / "packages"
        / "scenarios"
        / "open_field_pedestrian_crossing"
        / "physics"
        / "pedestrian_capsule.xml"
    )
    assert pedestrian.position_m == (4.0, -6.0, 0.0)


def test_physics_plan_v2_flattens_robot_payloads_into_typed_runtime_inputs(
    tmp_path: Path,
) -> None:
    bundle = CatalogResolver.from_repository(REPO_ROOT).resolve(
        PAYLOAD_SESSION
    ).write_bundle(tmp_path / "payload-bundle")

    plan = coordinator_module.load_physics_plan(bundle, REPO_ROOT)

    assert len(plan.payloads) == 1
    payload = plan.payloads[0]
    assert isinstance(payload, coordinator_module.PhysicsPayloadPlan)
    assert payload.instance_id == "rws_01"
    assert payload.namespace == "rws_01"
    assert payload.robot_instance_id == "thunder_01"
    assert payload.package == {
        "id": "fictional_rws_01",
        "version": "1.0.0",
        "kind": "payload",
        "manifest": (
            "sim/packages/payloads/fictional_rws_01/1.0.0/"
            "payload.package.yaml"
        ),
    }
    assert payload.parent_frame == "payload_top"
    assert payload.parent_body == "base_link"
    assert payload.model_path == (
        REPO_ROOT
        / "sim"
        / "packages"
        / "payloads"
        / "fictional_rws_01"
        / "1.0.0"
        / "mjcf"
        / "fictional_rws_01.xml"
    )
    assert payload.attach_root == "payload_base"
    assert payload.position_m == (0.0, 0.0, 0.14)
    assert payload.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert payload.authority == "mujoco"
    assert payload.collision_representation == "primitive_proxy"
    assert payload.frames[0]["name"] == "payload_base"


def test_manifest_writer_uses_retrying_atomic_replace(
    tmp_path: Path,
    monkeypatch,
) -> None:
    replacements: list[tuple[Path, Path]] = []

    def replace(source: Path, destination: Path) -> None:
        replacements.append((source, destination))
        source.replace(destination)

    monkeypatch.setattr(coordinator_module, "replace_file_with_retry", replace)
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=FakePhysicsHost(),
        run_id="manifest-replace",
    )

    coordinator.prepare()
    coordinator.stop()

    expected_temporary = coordinator.manifest_path.with_suffix(".json.tmp")
    assert replacements
    assert all(
        source == expected_temporary and destination == coordinator.manifest_path
        for source, destination in replacements
    )


def test_manifest_writer_serializes_concurrent_runtime_evidence_updates(
    tmp_path: Path,
    monkeypatch,
) -> None:
    active_replacements = 0
    maximum_active_replacements = 0
    observation_lock = threading.Lock()
    start = threading.Barrier(3)

    def replace(source: Path, destination: Path) -> None:
        nonlocal active_replacements, maximum_active_replacements
        with observation_lock:
            active_replacements += 1
            maximum_active_replacements = max(maximum_active_replacements, active_replacements)
        try:
            time.sleep(0.05)
            source.replace(destination)
        finally:
            with observation_lock:
                active_replacements -= 1

    monkeypatch.setattr(coordinator_module, "replace_file_with_retry", replace)
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=FakePhysicsHost(),
        run_id="concurrent-manifest-replace",
    )
    coordinator.prepare()

    errors: list[BaseException] = []

    def write_manifest() -> None:
        start.wait()
        try:
            coordinator._write_manifest()
        except BaseException as exc:
            errors.append(exc)

    threads = [threading.Thread(target=write_manifest) for _ in range(2)]
    for thread in threads:
        thread.start()
    start.wait()
    for thread in threads:
        thread.join()

    assert errors == []
    assert maximum_active_replacements == 1


def test_realtime_advance_persists_manifest_at_bounded_wall_clock_rate(
    tmp_path: Path,
    monkeypatch,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=FakePhysicsHost(),
        run_id="realtime-manifest-cadence",
    )
    coordinator.prepare()
    coordinator.start()
    writes: list[int] = []
    clock_values = iter((0, 50_000_000, 100_000_000))
    monkeypatch.setattr(
        coordinator,
        "_write_manifest_unlocked",
        lambda: writes.append(len(writes)),
    )
    monkeypatch.setattr(
        coordinator_module.time,
        "monotonic_ns",
        lambda: next(clock_values, 200_000_000),
    )
    coordinator._next_realtime_manifest_write_ns = 0

    coordinator.advance_realtime(1)
    coordinator.advance_realtime(1)
    coordinator.advance_realtime(1)

    assert len(writes) == 2
    coordinator.stop()


def test_coordinator_drives_runtime_lifecycle_without_mutating_bundle(
    tmp_path: Path,
) -> None:
    bundle = _bundle(tmp_path)
    bundle_before = {path.name: path.read_bytes() for path in bundle.iterdir() if path.is_file()}
    host = FakePhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="run-test",
    )

    ready = coordinator.prepare()
    assert ready["event"] == "ready"
    assert coordinator.state is RuntimeState.READY
    assert coordinator.manifest_path == (tmp_path / "runs" / "run-test" / "session.runtime.json")

    coordinator.start()
    assert coordinator.state is RuntimeState.RUNNING
    snapshot = coordinator.advance(5)
    assert snapshot["physics_step"] == 5

    coordinator.pause()
    assert coordinator.state is RuntimeState.PAUSED
    reset = coordinator.reset()
    assert reset["reset_generation"] == 1

    coordinator.stop()
    assert coordinator.state is RuntimeState.STOPPED
    assert host.calls[1:] == ["start", ("advance", 5), "pause", "reset", "stop"]
    assert bundle_before == {path.name: path.read_bytes() for path in bundle.iterdir() if path.is_file()}

    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "lingtu.sim.session-runtime.v1"
    assert manifest["state"] == "STOPPED"
    assert manifest["session_id"] == coordinator.plan.session_id
    assert manifest["allocation"]["physics_pid"] == 4242
    assert manifest["allocation"]["ports"] == {}
    assert manifest["allocation"]["shared_memory"] == {}
    in_memory_manifest = coordinator.runtime_manifest_snapshot()
    assert in_memory_manifest == manifest
    in_memory_manifest["state"] = "FAILED"
    assert coordinator.runtime_manifest_snapshot()["state"] == "STOPPED"

    episode_path = coordinator.allocation.run_dir / "episode_result.json"
    episode_bytes = episode_path.read_bytes()
    assert json.loads(episode_bytes) == {
        "schema": "lingtu.sim.episode-result.v1",
        "run_id": "run-test",
        "session_id": coordinator.plan.session_id,
        "model_generation": 0,
        "reset_generation": 1,
        "start_sim_time_ns": 0,
        "end_sim_time_ns": 0,
        "status": "SUCCEEDED",
        "failure_reason": None,
        "artifact_references": {
            "run_allocation": "run-allocation.json",
            "runtime_manifest": "session.runtime.json",
        },
    }

    stopped_event = coordinator.stop()

    assert stopped_event["event"] == "stopped"
    assert episode_path.read_bytes() == episode_bytes
    assert host.calls.count("stop") == 1


def test_terminal_failure_finalizer_rewrites_stopped_manifest_and_episode(
    tmp_path: Path,
) -> None:
    host = FakePhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="terminal-finalizer",
    )
    coordinator.prepare()
    stopped = coordinator.stop()
    assert stopped["event"] == "stopped"

    finalized = coordinator.finalize_terminal_failure(
        "terminal status evidence failed"
    )

    assert finalized["event"] == "stopped"
    assert coordinator.state is RuntimeState.FAILED
    assert host.calls.count("stop") == 1
    manifest = json.loads(coordinator.manifest_path.read_text(encoding="utf-8"))
    episode = json.loads(
        (coordinator.allocation.run_dir / "episode_result.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["state"] == "FAILED"
    assert coordinator.runtime_manifest_snapshot()["state"] == "FAILED"
    assert episode["status"] == "FAILED"
    assert episode["failure_reason"] == "terminal status evidence failed"


def test_coordinator_adopts_trusted_existing_empty_run_directory(
    tmp_path: Path,
) -> None:
    run_root = tmp_path / "runs"
    run_dir = run_root / "run-studio-precreated"
    run_dir.mkdir(parents=True)
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=run_root,
        physics_host=FakePhysicsHost(),
        run_id=run_dir.name,
        adopt_existing_empty_run_dir=True,
    )

    ready = coordinator.prepare()

    assert ready["event"] == "ready"
    assert coordinator.allocation.run_dir == run_dir.resolve()
    assert coordinator.allocation.path.is_file()
    coordinator.stop()


def test_episode_closure_includes_only_registered_run_local_artifacts(
    tmp_path: Path,
) -> None:
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=FakePhysicsHost(),
        run_id="run-recorded",
    )
    coordinator.prepare()
    recording = coordinator.allocation.run_dir / "simulation-recording.json"
    recording.write_text("{}\n", encoding="utf-8")

    for reserved in ("run_allocation", "runtime_manifest"):
        with pytest.raises(CoordinatorError, match="reserved by the runtime"):
            coordinator.register_episode_artifact(reserved, recording.name)
    coordinator.register_episode_artifact("simulation_recording", recording.name)
    coordinator.declare_episode_artifact(
        "recording_manifest",
        "recording/recording.manifest.json",
    )
    with pytest.raises(CoordinatorError, match="run-local"):
        coordinator.register_episode_artifact("escape", "../outside.json")
    coordinator.stop()

    episode = json.loads(
        (coordinator.allocation.run_dir / "episode_result.json").read_text(encoding="utf-8")
    )
    assert episode["artifact_references"]["simulation_recording"] == recording.name
    assert episode["artifact_references"]["recording_manifest"] == (
        "recording/recording.manifest.json"
    )


def test_stopped_run_releases_same_boot_resources_for_a_new_run(
    tmp_path: Path,
) -> None:
    bundle = _bundle(tmp_path)
    run_root = tmp_path / "runs"
    first_host = FakePhysicsHost()
    first = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=run_root,
        physics_host=first_host,
        run_id="run-first",
        boot_id="boot-resource-reuse",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )
    first.prepare()
    stopped = first.stop()
    assert first.stop() == stopped
    assert first_host.calls.count("stop") == 1

    second = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=run_root,
        physics_host=FakePhysicsHost(),
        run_id="run-second",
        boot_id="boot-resource-reuse",
        dds_domain=83,
        ports={"gateway": 15050},
        shm={"front_camera": "camera-shm"},
    )

    assert second.prepare()["event"] == "ready"
    assert second.state is RuntimeState.READY


def test_prepare_failure_rolls_back_and_releases_same_boot_resources(
    tmp_path: Path,
) -> None:
    bundle = _bundle(tmp_path)
    run_root = tmp_path / "runs"
    failing_host = PrepareFailingPhysicsHost()
    failed = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=run_root,
        physics_host=failing_host,
        run_id="run-failed",
        boot_id="boot-prepare-failure",
        dds_domain=84,
        ports={"gateway": 25050},
        shm={"front_camera": "failed-camera-shm"},
    )

    with pytest.raises(CoordinatorError, match="physics prepare failed"):
        failed.prepare()

    assert failed.state is RuntimeState.FAILED
    assert failing_host.calls[-1] == "stop"
    manifest = json.loads(failed.manifest_path.read_text(encoding="utf-8"))
    assert manifest["state"] == "FAILED"
    assert failed.stop() == {}
    assert failed.stop() == {}
    assert failing_host.calls.count("stop") == 1
    assert failed.state is RuntimeState.FAILED

    replacement = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=run_root,
        physics_host=FakePhysicsHost(),
        run_id="run-replacement",
        boot_id="boot-prepare-failure",
        dds_domain=84,
        ports={"gateway": 25050},
        shm={"front_camera": "failed-camera-shm"},
    )
    assert replacement.prepare()["event"] == "ready"


def test_coordinator_rejects_plan_session_mismatch_before_starting_host(
    tmp_path: Path,
) -> None:
    bundle = _bundle(tmp_path)
    plan_path = bundle / "physics.plan.json"
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan["session_id"] = "0" * 64
    plan_path.write_text(json.dumps(plan), encoding="utf-8")
    host = FakePhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="run-test",
    )

    with pytest.raises(CoordinatorError, match="session_id"):
        coordinator.prepare()

    assert host.calls == []
