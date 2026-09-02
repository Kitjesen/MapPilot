from __future__ import annotations

# ruff: noqa: S101
import json
from dataclasses import FrozenInstanceError, replace
from pathlib import Path

import pytest

from sim.runtime.scenario import (
    MAX_SCENARIO_DATAGRAM_BYTES,
    CompositeScenarioDispatcher,
    GenerationStamp,
    ScenarioClock,
    ScenarioDispatchError,
    ScenarioPlanError,
    ScenarioRuntime,
    ScenarioSnapshot,
    UdpScenarioVisualSink,
    encode_scenario_snapshot,
    load_scenario_plan,
)

SESSION_ID = "scenario-test"


def _transform(x: float, y: float = 0.0, z: float = 0.0) -> dict[str, object]:
    return {
        "position_m": [x, y, z],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }


def _plan() -> dict[str, object]:
    return {
        "schema": "lingtu.sim.scenario-plan.v1",
        "session_id": SESSION_ID,
        "env": "sim",
        "backend": "mujoco",
        "package": {"id": "crossing", "version": "1.0.0"},
        "model_generation": 0,
        "reset_generation": 0,
        "seed": 12,
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
                "entity_id": "robot-1",
                "entity_type": "robot",
                "authority": "mujoco",
                "source_epoch": 0,
                "initial_transform": _transform(9.0, 1.0, 0.0),
                "physics_proxy": "mujoco",
                "semantic_class": "quadruped",
            },
            {
                "entity_id": "pedestrian-1",
                "entity_type": "pedestrian",
                "authority": "scenario",
                "source_epoch": 0,
                "initial_transform": _transform(0.0),
                "physics_proxy": {
                    "mode": "kinematic",
                    "body_stable_id": "pedestrian-1/proxy_root",
                },
                "semantic_class": "person",
                "behavior": {
                    "profile": "linear_crossing",
                    "seed": 0,
                    "parameters": {
                        "start_time_s": 2.0,
                        "duration_s": 4.0,
                        "speed_mps": 1.0,
                        "end_position_m": [4.0, 0.0, 0.0],
                    },
                },
            },
            {
                "entity_id": "flag-anim",
                "entity_type": "prop",
                "authority": "ue_animation",
                "source_epoch": 0,
                "initial_transform": _transform(3.0, 2.0, 0.0),
                "physics_proxy": "none",
                "semantic_class": "flag",
            },
        ],
    }


def _runtime(plan: dict[str, object] | None = None) -> ScenarioRuntime:
    return ScenarioRuntime.from_plan(plan or _plan())


def _clock(
    sim_time_ns: int,
    *,
    model_generation: int = 0,
    reset_generation: int = 0,
    session_id: str = SESSION_ID,
) -> ScenarioClock:
    return ScenarioClock(
        session_id=session_id,
        model_generation=model_generation,
        reset_generation=reset_generation,
        sim_time_ns=sim_time_ns,
        source="mujoco_sim_time",
    )


def _entity(snapshot, entity_id: str):
    return {entity.entity_id: entity for entity in snapshot.entities}[entity_id]


@pytest.mark.parametrize(
    ("sim_time_ns", "expected_x", "motion_state"),
    [
        (1_000_000_000, 0.0, "pending"),
        (2_000_000_000, 0.0, "active"),
        (4_000_000_000, 2.0, "active"),
        (6_000_000_000, 4.0, "complete"),
        (7_000_000_000, 4.0, "complete"),
    ],
)
def test_linear_crossing_clamps_before_midpoint_end_and_after(
    sim_time_ns: int,
    expected_x: float,
    motion_state: str,
) -> None:
    snapshot = _runtime().snapshot(_clock(sim_time_ns))

    pedestrian = _entity(snapshot, "pedestrian-1")
    assert pedestrian.transform.position_m == pytest.approx((expected_x, 0.0, 0.0))
    assert pedestrian.motion_state == motion_state
    assert pedestrian.authority == "scenario"
    assert pedestrian.source_epoch == 0
    assert pedestrian.semantic_class == "person"
    assert pedestrian.physics_proxy_mode == "kinematic"
    assert pedestrian.body_stable_id == "pedestrian-1/proxy_root"


def test_scenario_entity_without_behavior_stays_at_its_initial_transform() -> None:
    plan = _plan()
    entities = [dict(entity) for entity in plan["entities"]]  # type: ignore[index]
    entities[1].pop("behavior")
    plan["entities"] = entities

    snapshot = _runtime(plan).snapshot(_clock(4_000_000_000))

    pedestrian = _entity(snapshot, "pedestrian-1")
    assert pedestrian.transform.position_m == pytest.approx((0.0, 0.0, 0.0))
    assert pedestrian.motion_state == "stationary"


def test_mujoco_authority_robot_passes_through_initial_transform() -> None:
    snapshot = _runtime().snapshot(_clock(4_000_000_000))

    robot = _entity(snapshot, "robot-1")
    assert robot.transform.position_m == pytest.approx((9.0, 1.0, 0.0))
    assert robot.motion_state == "mujoco_authority"


def test_ue_animation_entities_remain_intent_only_not_physics_truth() -> None:
    snapshot = _runtime().snapshot(_clock(4_000_000_000))

    flag = _entity(snapshot, "flag-anim")
    assert flag.transform.position_m == pytest.approx((3.0, 2.0, 0.0))
    assert flag.motion_state == "intent_only"
    assert flag.authority == "ue_animation"


def test_reset_generation_restores_initial_state_and_sequence() -> None:
    runtime = _runtime()
    first = runtime.snapshot(_clock(4_000_000_000))
    reset = runtime.snapshot(_clock(0, reset_generation=1))

    assert first.sequence == 0
    assert reset.sequence == 0
    assert reset.reset_generation == 1
    assert _entity(reset, "pedestrian-1").transform.position_m == pytest.approx((0.0, 0.0, 0.0))


def test_reset_generation_rejects_a_clock_that_did_not_return_to_plan_origin() -> None:
    runtime = _runtime()
    runtime.snapshot(_clock(4_000_000_000))

    with pytest.raises(ScenarioPlanError, match=r"reset clock.*initial sim_time"):
        runtime.snapshot(_clock(1_000_000_000, reset_generation=1))


def test_duplicate_entity_ids_are_rejected() -> None:
    plan = _plan()
    entities = list(plan["entities"])  # type: ignore[index]
    duplicate = dict(entities[1])
    duplicate["entity_id"] = "robot-1"
    entities.append(duplicate)
    plan["entities"] = entities

    with pytest.raises(ScenarioPlanError, match="duplicate entity_id"):
        _runtime(plan)


@pytest.mark.parametrize("field", ["package_digest", "model_sha256"])
def test_package_hash_fields_are_rejected(field: str) -> None:
    plan = _plan()
    package = dict(plan["package"])  # type: ignore[arg-type]
    package[field] = "0" * 64
    plan["package"] = package

    with pytest.raises(ScenarioPlanError, match="unknown field"):
        _runtime(plan)


@pytest.mark.parametrize(
    ("entity_patch", "message"),
    [
        ({"authority": "scenario", "entity_type": "robot"}, "robot.*mujoco"),
        ({"authority": "mujoco", "entity_type": "pedestrian"}, "mujoco.*robot"),
        ({"authority": "scenario", "behavior": {"profile": "orbit", "seed": 0, "parameters": {}}}, "profile"),
    ],
)
def test_bad_authority_or_behavior_profile_is_rejected(
    entity_patch: dict[str, object],
    message: str,
) -> None:
    plan = _plan()
    entities = [dict(entity) for entity in plan["entities"]]  # type: ignore[index]
    entities[1].update(entity_patch)
    plan["entities"] = entities

    with pytest.raises(ScenarioPlanError, match=message):
        _runtime(plan)


def test_same_plan_and_time_are_deterministic() -> None:
    left = _runtime().snapshot(_clock(4_000_000_000)).to_dict()
    right = _runtime().snapshot(_clock(4_000_000_000)).to_dict()

    assert left == right


def test_optional_missing_plan_produces_empty_generation_stamped_snapshot() -> None:
    snapshot = ScenarioRuntime.from_plan(None).snapshot(_clock(0))

    assert snapshot.session_id == SESSION_ID
    assert snapshot.sequence == 0
    assert snapshot.entities == ()


def test_snapshot_and_nested_values_are_immutable() -> None:
    snapshot = _runtime().snapshot(_clock(0))

    with pytest.raises(FrozenInstanceError):
        snapshot.sequence = 12  # type: ignore[misc]
    with pytest.raises(TypeError):
        snapshot.entities[0] = snapshot.entities[0]  # type: ignore[index]


def test_strict_json_loader_rejects_duplicates_non_finite_and_unknown_fields(tmp_path: Path) -> None:
    duplicate = tmp_path / "duplicate.plan.json"
    duplicate.write_text(
        '{"schema":"lingtu.sim.scenario-plan.v1","schema":"lingtu.sim.scenario-plan.v1"}',
        encoding="utf-8",
    )
    with pytest.raises(ScenarioPlanError, match="duplicate"):
        load_scenario_plan(duplicate)

    non_finite = tmp_path / "nan.plan.json"
    non_finite.write_text(json.dumps(_plan()).replace("12", "NaN", 1), encoding="utf-8")
    with pytest.raises(ScenarioPlanError, match="non-finite"):
        load_scenario_plan(non_finite)

    unknown = _plan()
    unknown["surprise"] = True
    unknown_path = tmp_path / "unknown.plan.json"
    unknown_path.write_text(json.dumps(unknown), encoding="utf-8")
    with pytest.raises(ScenarioPlanError, match="unknown field"):
        load_scenario_plan(unknown_path)


def test_clock_validation_rejects_wrong_session_source_backward_and_stale_time() -> None:
    runtime = _runtime()
    with pytest.raises(ScenarioPlanError, match="session_id"):
        runtime.snapshot(_clock(0, session_id="other-session"))
    with pytest.raises(ScenarioPlanError, match="clock source"):
        runtime.snapshot(
            ScenarioClock(
                session_id=SESSION_ID,
                model_generation=0,
                reset_generation=0,
                sim_time_ns=0,
                source="wall_clock",
            )
        )

    runtime.snapshot(_clock(2_000_000_000))
    with pytest.raises(ScenarioPlanError, match="stale"):
        runtime.snapshot(_clock(2_000_000_000))
    with pytest.raises(ScenarioPlanError, match="backward"):
        runtime.snapshot(_clock(1_000_000_000))

    generation_runtime = _runtime()
    generation_runtime.snapshot(_clock(0, model_generation=1))
    with pytest.raises(ScenarioPlanError, match="generation"):
        generation_runtime.snapshot(_clock(0, model_generation=0))


class _RecordingPhysicsSink:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {"result": "applied"}


class _RecordingVisualSink:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def apply_visual_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {"result": "applied"}


class _RecordingSensorSink:
    def __init__(self) -> None:
        self.snapshots: list[ScenarioSnapshot] = []

    def apply_sensor_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        self.snapshots.append(snapshot)
        return {"result": "applied"}


class _FailingVisualSink(_RecordingVisualSink):
    def apply_visual_entities(self, snapshot: ScenarioSnapshot) -> dict[str, object]:
        if snapshot.reset_generation == 1:
            raise RuntimeError("visual mailbox refused reset")
        return super().apply_visual_entities(snapshot)


def test_composite_dispatcher_routes_one_stamped_snapshot_by_authority() -> None:
    runtime = _runtime()
    snapshot = runtime.snapshot(_clock(0)).for_dispatch()
    physics = _RecordingPhysicsSink()
    visual = _RecordingVisualSink()
    sensors = _RecordingSensorSink()
    dispatcher = CompositeScenarioDispatcher(
        session_id=SESSION_ID,
        initial_generation=GenerationStamp(0, 0),
        physics_sink=physics,
        visual_sink=visual,
        sensor_sink=sensors,
    )

    dispatcher.dispatch(snapshot)

    assert [entity.entity_id for entity in physics.snapshots[0].entities] == ["pedestrian-1"]
    assert [entity.entity_id for entity in visual.snapshots[0].entities] == [
        "pedestrian-1",
        "flag-anim",
    ]
    assert [entity.entity_id for entity in sensors.snapshots[0].entities] == ["pedestrian-1"]
    assert {
        (
            routed.session_id,
            routed.model_generation,
            routed.reset_generation,
            routed.sequence,
            routed.sim_time_ns,
        )
        for routed in (physics.snapshots[0], visual.snapshots[0], sensors.snapshots[0])
    } == {(SESSION_ID, 0, 0, 0, 0)}


def test_composite_dispatcher_rejects_session_sequence_and_generation_replay() -> None:
    runtime = _runtime()
    first = runtime.snapshot(_clock(0)).for_dispatch()
    second = runtime.snapshot(_clock(1_000_000_000)).for_dispatch()
    dispatcher = CompositeScenarioDispatcher(
        session_id=SESSION_ID,
        initial_generation=GenerationStamp(0, 0),
        physics_sink=_RecordingPhysicsSink(),
        visual_sink=_RecordingVisualSink(),
    )

    with pytest.raises(ScenarioDispatchError, match="session_id"):
        dispatcher.dispatch(replace(first, session_id="other-session"))

    dispatcher.dispatch(first)
    with pytest.raises(ScenarioDispatchError, match=r"sequence.*expected 1"):
        dispatcher.dispatch(first)
    with pytest.raises(ScenarioDispatchError, match=r"sequence.*expected 1"):
        dispatcher.dispatch(replace(second, sequence=2))

    dispatcher.dispatch(second)
    reset = runtime.snapshot(_clock(0, reset_generation=1)).for_dispatch()
    dispatcher.dispatch(reset)
    with pytest.raises(ScenarioDispatchError, match=r"generation.*backward|stale generation"):
        dispatcher.dispatch(replace(second, sequence=2))


def test_composite_dispatcher_switches_reset_generation_only_after_all_sinks_accept() -> None:
    runtime = _runtime()
    first = runtime.snapshot(_clock(0)).for_dispatch()
    reset = runtime.snapshot(_clock(0, reset_generation=1)).for_dispatch()
    physics = _RecordingPhysicsSink()
    visual = _FailingVisualSink()
    dispatcher = CompositeScenarioDispatcher(
        session_id=SESSION_ID,
        initial_generation=GenerationStamp(0, 0),
        physics_sink=physics,
        visual_sink=visual,
    )
    dispatcher.dispatch(first)

    with pytest.raises(
        ScenarioDispatchError,
        match=r"visual sink failed.*reset_generation=1.*sequence=0.*visual mailbox refused reset",
    ):
        dispatcher.dispatch(reset)

    assert dispatcher.active_generation == GenerationStamp(0, 0)
    assert dispatcher.last_sequence == 0


def test_composite_dispatcher_requires_a_target_for_each_effect() -> None:
    snapshot = _runtime().snapshot(_clock(0)).for_dispatch()
    dispatcher = CompositeScenarioDispatcher(
        session_id=SESSION_ID,
        initial_generation=GenerationStamp(0, 0),
        visual_sink=_RecordingVisualSink(),
    )

    with pytest.raises(ScenarioDispatchError, match=r"physics sink.*pedestrian-1"):
        dispatcher.dispatch(snapshot)


def test_composite_dispatcher_rejects_mismatched_physics_stable_id_before_sinks() -> None:
    snapshot = _runtime().snapshot(_clock(0)).for_dispatch()
    invalid = replace(snapshot.entities[0], body_stable_id="other/proxy_root")
    physics = _RecordingPhysicsSink()
    visual = _RecordingVisualSink()
    dispatcher = CompositeScenarioDispatcher(
        session_id=SESSION_ID,
        initial_generation=GenerationStamp(0, 0),
        physics_sink=physics,
        visual_sink=visual,
    )

    with pytest.raises(ScenarioDispatchError, match=r"pedestrian-1.*body_stable_id"):
        dispatcher.dispatch(replace(snapshot, entities=(invalid, snapshot.entities[1])))

    assert physics.snapshots == []
    assert visual.snapshots == []


class _DatagramSocket:
    def __init__(self, *, error: OSError | None = None, truncate: bool = False) -> None:
        self.error = error
        self.truncate = truncate
        self.blocking: bool | None = None
        self.sent: list[tuple[bytes, tuple[str, int]]] = []
        self.closed = False

    def setblocking(self, blocking: bool) -> None:
        self.blocking = blocking

    def sendto(self, payload: bytes, destination: tuple[str, int]) -> int:
        if self.error is not None:
            raise self.error
        self.sent.append((payload, destination))
        return len(payload) - 1 if self.truncate else len(payload)

    def close(self) -> None:
        self.closed = True


def test_udp_visual_sink_sends_strict_original_scenario_snapshot_to_existing_port() -> None:
    snapshot = _runtime().snapshot(_clock(0)).for_dispatch()
    sock = _DatagramSocket()
    sink = UdpScenarioVisualSink(25123, socket_factory=lambda *_: sock)  # type: ignore[arg-type]

    result = sink.apply_visual_entities(snapshot)

    payload, destination = sock.sent[0]
    assert destination == ("127.0.0.1", 25123)
    assert json.loads(payload) == snapshot.to_dict()
    assert payload == encode_scenario_snapshot(snapshot)
    assert result["result"] == "queued"
    assert result["delivery_stage"] == "udp_sender"
    assert result["ue_application_verified"] is False
    assert result["required_application_evidence"] == {
        "artifact": "scenario-visual-evidence.json",
        "source": "ue_registry_applied",
        "input_source": "canonical_scenario_snapshot",
    }
    assert result["session_id"] == snapshot.session_id
    assert result["reset_generation"] == snapshot.reset_generation
    assert result["sequence"] == snapshot.sequence
    assert sock.blocking is False
    sink.close()
    assert sock.closed is True


@pytest.mark.parametrize(
    ("socket", "message"),
    [
        (_DatagramSocket(error=BlockingIOError("busy")), "send failed.*busy"),
        (_DatagramSocket(error=OSError("offline")), "send failed.*offline"),
        (_DatagramSocket(truncate=True), "truncated"),
    ],
)
def test_udp_visual_sink_fails_closed_on_send_error_or_truncation(
    socket: _DatagramSocket,
    message: str,
) -> None:
    sink = UdpScenarioVisualSink(25123, socket_factory=lambda *_: socket)  # type: ignore[arg-type]

    with pytest.raises(ScenarioDispatchError, match=message):
        sink.apply_visual_entities(_runtime().snapshot(_clock(0)).for_dispatch())


def test_udp_visual_sink_rejects_oversized_snapshot_before_send() -> None:
    snapshot = _runtime().snapshot(_clock(0)).for_dispatch()
    oversized_entity = replace(
        snapshot.entities[0],
        semantic_class="x" * MAX_SCENARIO_DATAGRAM_BYTES,
    )
    oversized = replace(snapshot, entities=(oversized_entity,))
    sock = _DatagramSocket()
    sink = UdpScenarioVisualSink(25123, socket_factory=lambda *_: sock)  # type: ignore[arg-type]

    with pytest.raises(ScenarioDispatchError, match=r"exceeds.*datagram limit"):
        sink.apply_visual_entities(oversized)

    assert sock.sent == []
