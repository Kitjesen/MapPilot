from __future__ import annotations

# ruff: noqa: S101
import importlib
import json
from pathlib import Path
from typing import Any

import pytest

from sim.catalog import CatalogResolver
from sim.runtime.coordinator import (
    BindingFacet,
    BindingState,
    CoordinatorError,
    MujocoProcess,
    RuntimeCoordinator,
    RuntimeState,
)
from sim.runtime.scenario import ScenarioSnapshot

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = REPO_ROOT / "sim" / "sessions" / "examples" / "thunder_omni_contract" / "session.yaml"
MUJOCO_HEADLESS = (
    REPO_ROOT
    / "build"
    / "mujoco-runtime-physics-win"
    / "Release"
    / "lingtu_mujoco_headless.exe"
)


def _transform(x: float, y: float = 0.0, z: float = 0.0) -> dict[str, object]:
    return {
        "position_m": [x, y, z],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }


def _scenario_plan(session_id: str) -> dict[str, object]:
    return {
        "schema": "lingtu.sim.scenario-plan.v1",
        "session_id": session_id,
        "env": "sim",
        "backend": "mujoco",
        "package": {"id": "coordinator_test", "version": "1.0.0"},
        "model_generation": 0,
        "reset_generation": 0,
        "seed": 7,
        "clock": {
            "unit": "ns",
            "source": "mujoco_sim_time",
            "sim_time_ns": 0,
        },
        "authority_policy": {
            "robot_physics_owner": "mujoco",
            "dynamic_behavior_owner": "scenario",
            "visual_animation_owner": "ue_animation",
        },
        "entities": [
            {
                "entity_id": "thunder_01",
                "entity_type": "robot",
                "authority": "mujoco",
                "source_epoch": 0,
                "initial_transform": _transform(0.0),
                "physics_proxy": "mujoco",
                "semantic_class": "quadruped",
            },
            {
                "entity_id": "pedestrian_01",
                "entity_type": "pedestrian",
                "authority": "scenario",
                "source_epoch": 0,
                "initial_transform": _transform(0.0, -2.0),
                "physics_proxy": "kinematic",
                "semantic_class": "person",
                "behavior": {
                    "profile": "linear_crossing",
                    "seed": 7,
                    "parameters": {
                        "start_time_s": 0.0,
                        "duration_s": 4.0,
                        "speed_mps": 1.0,
                        "end_position_m": [0.0, 2.0, 0.0],
                    },
                },
            },
            {
                "entity_id": "flag_01",
                "entity_type": "prop",
                "authority": "ue_animation",
                "source_epoch": 0,
                "initial_transform": _transform(3.0, 1.0),
                "physics_proxy": "none",
                "semantic_class": "flag",
            },
        ],
    }


def _bundle(tmp_path: Path) -> Path:
    return CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION).write_bundle(tmp_path / "bundle")


class ScenarioPhysicsHost:
    def __init__(self) -> None:
        self.pid: int | None = 8101
        self.calls: list[object] = []
        self.session_id = ""
        self.model_generation = 0
        self.reset_generation = 0
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0

    def prepare(self, plan: Any, allocation: Any) -> dict[str, Any]:
        del allocation
        self.calls.append("prepare")
        self.session_id = plan.session_id
        self.model_generation = plan.model_generation
        return self._event("ready")

    def start(self) -> dict[str, Any]:
        self.calls.append("start")
        return self._event("running")

    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.sequence += steps
        self.physics_step += steps
        self.sim_time_ns += steps * 1_000_000_000
        return self._snapshot()

    def snapshot(self) -> dict[str, Any]:
        self.calls.append("snapshot")
        return self._snapshot()

    def pause(self) -> dict[str, Any]:
        self.calls.append("pause")
        return self._event("paused")

    def reset(self) -> dict[str, Any]:
        self.calls.append("reset")
        self.reset_generation += 1
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0
        return self._snapshot()

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        event = self._event("stopped")
        self.pid = None
        return event

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "session_id": self.session_id,
            "model_generation": self.model_generation,
            "reset_generation": self.reset_generation,
            "sequence": self.sequence,
            "physics_step": self.physics_step,
            "sim_time_ns": self.sim_time_ns,
        }

    def _snapshot(self) -> dict[str, Any]:
        return {**self._event("snapshot"), "bodies": [], "joints": [], "actuators": []}


class SequenceSkippingPhysicsHost(ScenarioPhysicsHost):
    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.sequence += steps + 1
        self.physics_step += steps
        self.sim_time_ns += steps * 1_000_000_000
        return self._snapshot()


class StaleClockPhysicsHost(ScenarioPhysicsHost):
    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.sequence += steps
        self.physics_step += steps
        return self._snapshot()


class UnexpectedGenerationPhysicsHost(ScenarioPhysicsHost):
    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.reset_generation += 1
        self.sequence = 0
        self.physics_step += steps
        self.sim_time_ns += steps * 1_000_000_000
        return self._snapshot()


class WrongModelGenerationOnResetHost(ScenarioPhysicsHost):
    def reset(self) -> dict[str, Any]:
        self.calls.append("reset")
        self.model_generation += 1
        self.reset_generation += 1
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 0
        return self._snapshot()


class RecordingScenarioDispatcher:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def dispatch(self, snapshot: ScenarioSnapshot) -> None:
        self.snapshots.append(snapshot)


class FailingScenarioDispatcher(RecordingScenarioDispatcher):
    def __init__(self, fail_on_sequence: int) -> None:
        super().__init__()
        self._fail_on_sequence = fail_on_sequence

    def dispatch(self, snapshot: ScenarioSnapshot) -> None:
        if snapshot.sequence == self._fail_on_sequence:
            raise RuntimeError("scenario apply failed")
        super().dispatch(snapshot)


class RecordingScenarioVisualSink:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def apply_visual_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {"result": "applied"}


class RecordingScenarioSensorSink:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def apply_sensor_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {"result": "applied"}


class FailingScenarioVisualSink(RecordingScenarioVisualSink):
    def apply_visual_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        if snapshot.sequence == 1:
            raise RuntimeError("UE scenario registry rejected pedestrian_01")
        return super().apply_visual_entities(snapshot)


class ClosableScenarioVisualSink(RecordingScenarioVisualSink):
    def __init__(self) -> None:
        super().__init__()
        self.closed = False

    def apply_visual_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {
            "result": "queued",
            "delivery_stage": "udp_sender",
            "ue_application_verified": False,
        }

    def close(self) -> None:
        self.closed = True


class KinematicScenarioPhysicsHost(ScenarioPhysicsHost):
    def __init__(self) -> None:
        super().__init__()
        self.applied_snapshots: list[ScenarioSnapshot] = []
        self.body_positions: dict[str, list[float]] = {}

    def prepare(self, plan: Any, allocation: Any) -> dict[str, Any]:
        event = super().prepare(plan, allocation)
        self.body_positions = {
            entity.entity_id: list(entity.position_m) for entity in plan.kinematic_entities
        }
        return event

    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.sequence += steps
        self.physics_step += steps
        self.sim_time_ns += steps * 1_000_000_000
        return self._snapshot()

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> dict[str, Any]:
        self.calls.append(("kinematic", snapshot.sequence))
        self.applied_snapshots.append(snapshot)
        for entity in snapshot.entities:
            self.body_positions[entity.entity_id] = list(entity.transform.position_m)
        return {
            "event": "kinematic_poses",
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "entity_count": len(snapshot.entities),
            "result": "applied",
        }

    def raycast(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(("raycast", kwargs.get("sequence")))
        sequence = int(kwargs["sequence"])
        sim_time_ns = int(kwargs["sim_time_ns"])
        directions = kwargs["directions_sensor"]
        direction = list(directions[0])
        position = self.body_positions["pedestrian_01"]
        return {
            "event": "raycast",
            "sensor_frame_id": kwargs["sensor_frame_id"],
            "session_id": kwargs["session_id"],
            "model_generation": kwargs["model_generation"],
            "reset_generation": kwargs["reset_generation"],
            "sequence": sequence,
            "sim_time_ns": sim_time_ns,
            "hit_count": 1,
            "hits": [
                {
                    "entity_id": "pedestrian_01",
                    "body_stable_id": "pedestrian_01/proxy_root",
                    "distance_m": max(0.01, float(abs(position[1]))),
                    "origin_world_m": [0.0, 0.0, 0.0],
                    "direction_world": direction,
                    "position_world_m": position,
                    "xyz_sensor": position,
                    "offset_time_ns": 0,
                    "reflectivity": 15,
                    "tag": 0,
                    "line": 0,
                }
            ],
        }

    def _snapshot(self) -> dict[str, Any]:
        return {
            **self._event("snapshot"),
            "bodies": [
                {
                    "stable_id": "thunder_01/base_link",
                    "instance_id": "thunder_01",
                    "frame_id": "base_link",
                    "position_m": [0.0, 0.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "linear_velocity_mps": [0.0, 0.0, 0.0],
                    "angular_velocity_rps": [0.0, 0.0, 0.0],
                },
                *[
                    {
                        "stable_id": f"{entity_id}/proxy_root",
                        "instance_id": entity_id,
                        "frame_id": "proxy_root",
                        "position_m": position,
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                        "linear_velocity_mps": [0.0, 0.0, 0.0],
                        "angular_velocity_rps": [0.0, 0.0, 0.0],
                    }
                    for entity_id, position in self.body_positions.items()
                ],
            ],
            "joints": [],
            "actuators": [],
        }


class NonZeroClockOnResetHost(ScenarioPhysicsHost):
    def reset(self) -> dict[str, Any]:
        self.calls.append("reset")
        self.reset_generation += 1
        self.sequence = 0
        self.physics_step = 0
        self.sim_time_ns = 1_000_000_000
        return self._snapshot()


def _physics_only_scenario_bundle(
    tmp_path: Path,
    *,
    visual_required: bool = False,
) -> Path:
    session = tmp_path / "scenario-session.yaml"
    mode = "unreal" if visual_required else "headless"
    required_bindings = "[physics, visual]" if visual_required else "[physics]"
    session.write_text(
        f"""schema: lingtu.sim.session.v1
session_id: scenario_builtin_dispatch
mujoco_version: 3.10.0
seed: 20260805
world: open_field@1.0.0
scenario: open_field_pedestrian_crossing@1.0.0
robots:
  - instance_id: thunder_01
    package: thunderv4@1.0.3
    controller: null
    sensor_rig: null
    spawn:
      position_m: [0.0, 0.0, 0.0]
      quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
runtime:
  backend: mujoco
  mode: {mode}
  required_bindings: {required_bindings}
""",
        encoding="utf-8",
    )
    return CatalogResolver.from_repository(REPO_ROOT).resolve(session).write_bundle(
        tmp_path / "scenario-bundle"
    )


def test_coordinator_automatically_routes_package_kinematics_through_mujoco(
    tmp_path: Path,
) -> None:
    host = KinematicScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_physics_only_scenario_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="scenario-built-in-mujoco-dispatch",
    )

    coordinator.prepare()
    coordinator.start()
    event = coordinator.advance(4)

    pedestrian = next(
        body for body in event["bodies"] if body["stable_id"] == "pedestrian_01/proxy_root"
    )
    assert pedestrian["position_m"] == pytest.approx([4.0, -3.0, 0.0])
    assert [(snapshot.sequence, snapshot.sim_time_ns) for snapshot in host.applied_snapshots] == [
        (0, 0),
        (1, 1_000_000_000),
        (2, 2_000_000_000),
        (3, 3_000_000_000),
        (4, 4_000_000_000),
    ]


def test_coordinator_composes_physics_visual_and_sensor_scenario_sinks(
    tmp_path: Path,
) -> None:
    host = KinematicScenarioPhysicsHost()
    visual = RecordingScenarioVisualSink()
    sensors = RecordingScenarioSensorSink()
    coordinator = RuntimeCoordinator(
        bundle_dir=_physics_only_scenario_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_visual_sink=visual,
        scenario_sensor_sink=sensors,
        run_id="scenario-composite-sinks",
    )

    coordinator.prepare()
    coordinator.start()
    coordinator.advance()

    assert [snapshot.sequence for snapshot in host.applied_snapshots] == [0, 1]
    assert [snapshot.sequence for snapshot in visual.snapshots] == [0, 1]
    assert [snapshot.sequence for snapshot in sensors.snapshots] == [0, 1]
    assert [entity.entity_id for entity in visual.snapshots[-1].entities] == ["pedestrian_01"]
    assert [entity.entity_id for entity in sensors.snapshots[-1].entities] == ["pedestrian_01"]


def test_composite_sink_failure_aborts_with_route_generation_and_sequence(
    tmp_path: Path,
) -> None:
    host = KinematicScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_physics_only_scenario_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_visual_sink=FailingScenarioVisualSink(),
        run_id="scenario-composite-failure",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(
        CoordinatorError,
        match=(
            r"scenario dispatcher failed:.*visual sink failed.*"
            r"reset_generation=0 sequence=1.*UE scenario registry rejected pedestrian_01"
        ),
    ):
        coordinator.advance()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]
    episode = json.loads(
        (coordinator.allocation.run_dir / "episode_result.json").read_text(encoding="utf-8")
    )
    assert episode["status"] == "FAILED"
    assert "visual sink failed" in episode["failure_reason"]
    assert "reset_generation=0 sequence=1" in episode["failure_reason"]
    assert "UE scenario registry rejected pedestrian_01" in episode["failure_reason"]


def test_visual_snapshot_port_auto_wires_and_closes_scenario_udp_sink(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    coordinator_module = importlib.import_module("sim.runtime.coordinator.coordinator")
    visual = ClosableScenarioVisualSink()
    created_ports: list[int] = []

    def create_visual_sink(port: int) -> ClosableScenarioVisualSink:
        created_ports.append(port)
        return visual

    monkeypatch.setattr(coordinator_module, "UdpScenarioVisualSink", create_visual_sink)
    host = KinematicScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_physics_only_scenario_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        ports={"visual_snapshot_udp": 25123},
        run_id="scenario-auto-udp-sink",
    )

    coordinator.prepare()
    coordinator.stop()

    assert created_ports == [25123]
    assert [snapshot.sequence for snapshot in visual.snapshots] == [0]
    assert visual.closed is True


def test_queued_udp_snapshot_does_not_qualify_visual_binding(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    coordinator_module = importlib.import_module("sim.runtime.coordinator.coordinator")
    visual = ClosableScenarioVisualSink()

    monkeypatch.setattr(
        coordinator_module,
        "UdpScenarioVisualSink",
        lambda _port: visual,
    )
    coordinator = RuntimeCoordinator(
        bundle_dir=_physics_only_scenario_bundle(tmp_path, visual_required=True),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=KinematicScenarioPhysicsHost(),
        ports={"visual_snapshot_udp": 25123},
        run_id="scenario-udp-is-not-ue-evidence",
    )

    coordinator.prepare()

    assert [snapshot.sequence for snapshot in visual.snapshots] == [0]
    assert coordinator.state is RuntimeState.PREPARING
    assert coordinator.readiness.state(BindingFacet.VISUAL) is BindingState.UNBOUND
    coordinator.stop()


def test_real_coordinator_moves_the_pedestrian_inside_the_shared_mujoco_model(
    tmp_path: Path,
) -> None:
    if not MUJOCO_HEADLESS.is_file():
        pytest.skip(f"MuJoCo headless executable is missing: {MUJOCO_HEADLESS}")
    coordinator = RuntimeCoordinator(
        bundle_dir=_physics_only_scenario_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "real-runs",
        physics_host=MujocoProcess(MUJOCO_HEADLESS),
        run_id="scenario-real-mujoco-dispatch",
    )

    coordinator.prepare()
    coordinator.start()
    event = coordinator.advance(1_250)
    coordinator.stop()

    pedestrian = next(
        body for body in event["bodies"] if body["stable_id"] == "pedestrian_01/proxy_root"
    )
    assert event["sim_time_ns"] == 2_500_000_000
    assert pedestrian["position_m"] == pytest.approx([4.0, -5.25, 0.0])


def test_scenario_plan_with_applicable_entities_requires_dispatcher(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = ScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="scenario-missing-dispatcher",
    )

    with pytest.raises(CoordinatorError, match=r"scenario.*dispatcher"):
        coordinator.prepare()

    assert host.calls == []


def test_dispatches_authoritative_prepare_warmup_advance_and_reset_snapshots(
    tmp_path: Path,
) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = ScenarioPhysicsHost()
    dispatcher = RecordingScenarioDispatcher()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=dispatcher,
        run_id="scenario-lifecycle",
    )

    coordinator.prepare()
    coordinator.warmup(2)
    coordinator.start()
    coordinator.advance(2)
    coordinator.reset()

    assert [
        (
            snapshot.sequence,
            snapshot.sim_time_ns,
            snapshot.model_generation,
            snapshot.reset_generation,
        )
        for snapshot in dispatcher.snapshots
    ] == [
        (0, 0, 0, 0),
        (1, 1_000_000_000, 0, 0),
        (2, 2_000_000_000, 0, 0),
        (3, 3_000_000_000, 0, 0),
        (4, 4_000_000_000, 0, 0),
        (0, 0, 0, 1),
    ]
    assert [[entity.entity_id for entity in snapshot.entities] for snapshot in dispatcher.snapshots] == [
        ["pedestrian_01", "flag_01"]
    ] * 6


def test_missing_scenario_plan_preserves_existing_physics_call_pattern(tmp_path: Path) -> None:
    host = ScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=_bundle(tmp_path),
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="scenario-optional",
    )

    coordinator.prepare()
    coordinator.start()
    coordinator.advance(3)
    coordinator.pause()
    coordinator.reset()
    coordinator.stop()

    assert host.calls == ["prepare", "start", ("advance", 3), "pause", "reset", "stop"]


def test_dispatcher_failure_aborts_and_stops_active_session(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = ScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=FailingScenarioDispatcher(fail_on_sequence=1),
        run_id="scenario-dispatch-failure",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(CoordinatorError, match=r"scenario dispatcher failed.*scenario apply failed"):
        coordinator.advance()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]


def test_mujoco_only_scenario_plan_does_not_require_or_invoke_dispatcher(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    plan = _scenario_plan(session_id)
    plan["entities"] = [plan["entities"][0]]  # type: ignore[index]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(plan, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = ScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        run_id="scenario-mujoco-authority-only",
    )

    coordinator.prepare()
    coordinator.start()
    coordinator.advance(2)
    coordinator.stop()

    assert host.calls == ["prepare", "start", ("advance", 2), "stop"]


def test_physics_sequence_gap_aborts_and_stops_session(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = SequenceSkippingPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=RecordingScenarioDispatcher(),
        run_id="scenario-sequence-gap",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(CoordinatorError, match="scenario sequence"):
        coordinator.advance()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]


def test_stale_authoritative_clock_aborts_and_stops_session(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = StaleClockPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=RecordingScenarioDispatcher(),
        run_id="scenario-stale-clock",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(CoordinatorError, match=r"scenario evaluation failed:.*stale"):
        coordinator.advance()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]


def test_generation_change_outside_reset_aborts_and_stops_session(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = UnexpectedGenerationPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=RecordingScenarioDispatcher(),
        run_id="scenario-generation-fault",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(CoordinatorError, match=r"generation changed outside reset"):
        coordinator.advance()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]


def test_scenario_session_mismatch_is_rejected_before_physics_start(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan("other-session"), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = ScenarioPhysicsHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=RecordingScenarioDispatcher(),
        run_id="scenario-session-mismatch",
    )

    with pytest.raises(CoordinatorError, match=r"scenario plan session_id"):
        coordinator.prepare()

    assert host.calls == []


def test_bad_model_generation_on_reset_aborts_and_stops_session(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = WrongModelGenerationOnResetHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=RecordingScenarioDispatcher(),
        run_id="scenario-reset-generation-fault",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(CoordinatorError, match=r"model_generation"):
        coordinator.reset()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]


def test_nonzero_reset_clock_aborts_and_stops_session(tmp_path: Path) -> None:
    bundle = _bundle(tmp_path)
    session_id = json.loads((bundle / "physics.plan.json").read_text(encoding="utf-8"))["session_id"]
    (bundle / "scenario.plan.json").write_text(
        json.dumps(_scenario_plan(session_id), sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    host = NonZeroClockOnResetHost()
    coordinator = RuntimeCoordinator(
        bundle_dir=bundle,
        repo_root=REPO_ROOT,
        run_root=tmp_path / "runs",
        physics_host=host,
        scenario_dispatcher=RecordingScenarioDispatcher(),
        run_id="scenario-reset-clock-fault",
    )

    coordinator.prepare()
    coordinator.start()
    with pytest.raises(CoordinatorError, match=r"reset clock.*initial sim_time"):
        coordinator.reset()

    assert coordinator.state.value == "FAILED"
    assert host.calls[-2:] == ["pause", "stop"]
