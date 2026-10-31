import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from sim.catalog import CatalogResolver
from sim.runtime.control.contracts import (
    CommandSubmitResult,
    ControllerCommand,
    ControllerRuntimeError,
    ControllerState,
    GenerationStamp,
    SafeStopReason,
)
from sim.runtime.control.factory import create_production_components
from sim.runtime.control.plan import (
    ActuatorLayout,
    AdapterSpec,
    CommandChannelSpec,
    ControllerSpec,
    ControlPlan,
    PolicySpec,
    load_control_plan,
)
from sim.runtime.control.runtime import ControllerRuntime
from sim.runtime.control.thunder.cts_controller import ThunderCtsController
from sim.runtime.control.thunder.cts_policy import TERM_DIMS
from sim.runtime.control.thunder.legacy_controller import DART_ACTUATOR_ORDER, MUJOCO_ACTUATOR_ORDER

ROOT = Path(__file__).resolve().parents[2]
POLICY = "sim/packages/controllers/doso/thunder_v4/cts_moe/policy/policy_cts_moe_v39_61128.onnx"
GENERATION = GenerationStamp(0, 0)


class _Session:
    def __init__(self):
        self.contract = json.loads((ROOT / POLICY).with_name("runtime_contract.json").read_text())
        self.action = np.ones(16, dtype=np.float32)
        self.feeds = []

    def get_modelmeta(self):
        return SimpleNamespace(custom_metadata_map={
            "policy_contract": "thunder_cts_moe_v1", "runtime_contract": json.dumps(self.contract),
        })

    def get_inputs(self):
        return [SimpleNamespace(name="history", shape=["batch", 530]),
                SimpleNamespace(name="single_obs", shape=["batch", 53])]

    def get_outputs(self):
        return [SimpleNamespace(name="actions", shape=["batch", 16])]

    def run(self, names, feeds):
        assert names == ["actions"]
        self.feeds.append({name: value.copy() for name, value in feeds.items()})
        return [self.action[None].copy()]


def _spec():
    return ControllerSpec(
        controller_id="thunder.controller", instance_id="thunder",
        adapter=AdapterSpec(plugin="thunderv4_cts_moe", abi="lingtu.sim.controller-adapter.v1"),
        policy=PolicySpec(runtime="onnxruntime", artifact=POLICY, manifest="unused.json"),
        inference_hz=50.0, low_level_hz=200.0,
        state_channels=("joint_position", "joint_velocity", "base_angular_velocity", "projected_gravity"),
        command_channels=("base_twist",), actuators=ActuatorLayout(tuple(reversed(MUJOCO_ACTUATOR_ORDER))),
    )


def _state(controller, position=None, velocity=None):
    by_name = {name: index for index, name in enumerate(DART_ACTUATOR_ORDER)}
    order = [by_name[name] for name in controller._actuators.channels]
    position = controller.runner.default if position is None else position
    velocity = np.zeros(16) if velocity is None else velocity
    return ControllerState(
        session_id="cts", instance_id="thunder", generation=GENERATION, sequence=0, sim_time_ns=0,
        channels={"joint_position": position[order], "joint_velocity": velocity[order],
                  "base_angular_velocity": [4., -8., 12.], "projected_gravity": [0., 0., -1.]},
    )


def _observe(controller, *, direction=(.2, -.1, .3), **state_kwargs):
    command = ControllerCommand(
        channel_id="base_twist", instance_id="thunder", generation=GENERATION, sequence=0, apply_time_ns=0,
        payload=dict(zip(("linear_x", "linear_y", "angular_z"), direction)),
    )
    return controller.observe(_state(controller, **state_kwargs), command, controller._actuators)


def _runtime(controller):
    spec = replace(_spec(), command_channels=("base_twist", "actuators"))
    plan = ControlPlan(
        "cts", (spec,),
        (
            CommandChannelSpec("base_twist", "subscribe", "simulation", "dds_base_twist/thunder",
                               "typed_dds", "lingtu.dds.FinalVelocityCommand", "base_twist", "base"),
            CommandChannelSpec("actuators", "publish", "physics", spec.controller_id,
                               "in_process", "lingtu.sim.joint-torque.v1", "joint_torque", "actuators"),
        ),
        100_000_000,
    )
    return ControllerRuntime(plan=plan, controller_id=spec.controller_id, generation=GENERATION,
                             adapter=controller, policy=controller)


def test_cts_adapter_loads_through_the_shared_policy_entry(monkeypatch):
    from sim.runtime.control import policy_loader

    original = policy_loader.load_policy
    calls = []

    def record(path, **kwargs):
        calls.append((path, kwargs))
        return original(path, **kwargs)

    session = _Session()
    monkeypatch.setattr(policy_loader, "load_policy", record)
    controller = ThunderCtsController(_spec(), ROOT, session=session)
    assert calls == [(str((ROOT / POLICY).resolve()), {"cpu_threads": 1, "session": session})]
    assert controller.runner.session is session


def test_cts_adapter_preserves_controller_errors_for_invalid_metadata_and_missing_runtime(monkeypatch):
    from sim.runtime.control import policy_loader

    class MissingContractSession(_Session):
        def get_modelmeta(self):
            return SimpleNamespace(custom_metadata_map={})

    with pytest.raises(ControllerRuntimeError, match="policy load failed") as error:
        ThunderCtsController(_spec(), ROOT, session=MissingContractSession())
    assert isinstance(error.value.__cause__, ValueError)

    def unavailable(*_args, **_kwargs):
        raise ImportError("onnxruntime is unavailable")

    monkeypatch.setattr(policy_loader, "load_policy", unavailable)
    with pytest.raises(ControllerRuntimeError, match="requires onnxruntime") as error:
        ThunderCtsController(_spec(), ROOT)
    assert isinstance(error.value.__cause__, ImportError)


def test_cts_adapter_rejects_a_different_policy_implementation(monkeypatch):
    from sim.runtime.control import policy_loader

    monkeypatch.setattr(policy_loader, "load_policy", lambda *_args, **_kwargs: object())
    with pytest.raises(ControllerRuntimeError, match="metadata must declare"):
        ThunderCtsController(_spec(), ROOT)


def test_cts_adapter_routes_plan_joint_order_command_and_term_major_history():
    session = _Session()
    controller = ThunderCtsController(_spec(), ROOT, session=session)
    position = controller.runner.default + np.arange(16) * .01
    velocity = np.arange(16, dtype=np.float64)
    first = _observe(controller, position=position, velocity=velocity)
    np.testing.assert_allclose(first[:9], [1., -2., 3., 0., 0., -1., .2, -.1, .3])
    np.testing.assert_allclose(first[9:21], np.arange(12) * .01, atol=1e-7)
    np.testing.assert_allclose(first[21:37], velocity * .05)
    controller.infer(first)
    second = _observe(controller, direction=(.4, .2, -.1), position=position)
    np.testing.assert_array_equal(second[-16:], session.action)
    controller.infer(second)
    frames = np.array([first] * 9 + [second])
    expected = np.concatenate([
        block.reshape(-1) for block in np.split(frames, np.cumsum(TERM_DIMS)[:-1], axis=1)
    ])[None]
    np.testing.assert_array_equal(session.feeds[-1]["history"], expected)
    np.testing.assert_array_equal(session.feeds[-1]["single_obs"], second[None])


def test_cts_idle_brakes_wheels_without_discarding_policy_history_and_reset_reanchors_targets():
    controller = ThunderCtsController(_spec(), ROOT, session=_Session())
    moving = controller.infer(_observe(controller))
    assert np.all(moving[12:] != 0.)
    idle = controller.infer(_observe(controller, direction=(0., 0., 0.)))
    np.testing.assert_array_equal(idle[12:], np.zeros(4))
    assert len(controller.runner.session.feeds) == 2
    np.testing.assert_array_equal(controller.runner.last_action, np.ones(16))
    controller.reset(GenerationStamp(0, 1))
    assert controller.runner._frames is None and controller.runner.previous_target is None
    measured = controller.runner.default + .05
    first = _observe(controller, position=measured)
    np.testing.assert_array_equal(first[-16:], np.zeros(16))
    np.testing.assert_allclose(controller.runner.previous_target, measured[:12])
    targets = controller.infer(first)
    assert np.all(np.abs(targets[:12] - measured[:12]) <= controller.runner.delta + 1e-7)


@pytest.mark.parametrize("stop_reason", [SafeStopReason.STALE_COMMAND, SafeStopReason.NO_COMMAND],
                         ids=["stale", "hold"])
def test_cts_runtime_stop_resume_reanchors_targets_and_policy_history(stop_reason):
    session = _Session()
    session.action[:] = 4.
    controller = ThunderCtsController(_spec(), ROOT, session=session)
    runtime = _runtime(controller)
    command = ControllerCommand("base_twist", "thunder", GENERATION, 0, 0,
                                {"linear_x": .2, "linear_y": 0., "angular_z": 0.})
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED
    for index in range(5):
        assert runtime.step(replace(_state(controller), sequence=index,
                                    sim_time_ns=index * 20_000_000)).inference_ran
    old_target = controller.runner.previous_target.copy()
    if stop_reason is SafeStopReason.NO_COMMAND:
        runtime.hold()
    stopped = runtime.step(replace(_state(controller), sequence=5, sim_time_ns=100_000_000))
    assert stopped.actuator_command.safe_stop_reason is stop_reason
    assert not stopped.inference_ran
    assert controller.runner.previous_target is None
    assert controller.runner._frames is None
    np.testing.assert_array_equal(controller.runner.last_action, np.zeros(16))

    runtime.step(replace(_state(controller), sequence=6, sim_time_ns=1_000_000_000))
    measured = controller.runner.default + np.linspace(.05, .16, 16)
    command = replace(command, sequence=1, apply_time_ns=1_020_000_000,
                      payload={"linear_x": -.1, "linear_y": .1, "angular_z": -.2})
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED
    resumed_state = replace(_state(controller, position=measured), sequence=7, sim_time_ns=1_020_000_000)
    resumed = runtime.step(resumed_state)
    target = controller.runner.previous_target
    assert resumed.inference_ran and not resumed.actuator_command.safe_stop
    assert np.max(np.abs(old_target - measured[:12])) > controller.runner.delta
    assert np.all(np.abs(target - measured[:12]) <= controller.runner.delta + 1e-7)
    observation = session.feeds[-1]["single_obs"][0]
    np.testing.assert_array_equal(observation[-16:], np.zeros(16))
    np.testing.assert_allclose(observation[9:21], measured[:12] - controller.runner.default[:12], atol=1e-7)
    np.testing.assert_array_equal(controller.runner._frames, np.repeat(observation[None], 10, axis=0))
    np.testing.assert_array_equal(controller.runner.last_action, session.action)
    assert resumed.actuator_command.channels == controller._actuators.channels
    assert np.isfinite(resumed.actuator_command.values).all()
    expected = controller.actuate(resumed_state, runtime._last_action, controller._actuators)
    for name, value in expected.items():
        assert resumed.actuator_command.value_for(name) == value


@pytest.mark.parametrize("direction", [(0., 0., 0.), (-.1, .1, -.2)], ids=["zero", "moving"])
def test_cts_hold_resets_before_any_stop_step(direction):
    session = _Session()
    session.action[:] = 4.
    controller = ThunderCtsController(_spec(), ROOT, session=session)
    runtime = _runtime(controller)
    command = ControllerCommand("base_twist", "thunder", GENERATION, 0, 0,
                                {"linear_x": .2, "linear_y": 0., "angular_z": 0.})
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED
    for index in range(5):
        runtime.step(replace(_state(controller), sequence=index, sim_time_ns=index * 20_000_000))

    runtime.hold()
    assert controller.runner.previous_target is None
    assert controller.runner._frames is None
    np.testing.assert_array_equal(controller.runner.last_action, np.zeros(16))
    assert runtime.submit_command(command) is CommandSubmitResult.REJECTED_OUT_OF_ORDER

    measured = controller.runner.default + np.linspace(.05, .16, 16)
    command = replace(command, sequence=1, apply_time_ns=100_000_000,
                      payload=dict(zip(("linear_x", "linear_y", "angular_z"), direction)))
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED
    resumed = runtime.step(replace(_state(controller, position=measured), sequence=5, sim_time_ns=100_000_000))
    assert resumed.inference_ran and not resumed.actuator_command.safe_stop
    assert resumed.actuator_command.sequence == 6
    observation = session.feeds[-1]["single_obs"][0]
    np.testing.assert_array_equal(observation[-16:], np.zeros(16))
    np.testing.assert_allclose(observation[9:21], measured[:12] - controller.runner.default[:12], atol=1e-7)
    np.testing.assert_array_equal(controller.runner._frames, np.repeat(observation[None], 10, axis=0))
    assert np.all(np.abs(controller.runner.previous_target - measured[:12]) <= controller.runner.delta + 1e-7)
    if not any(direction):
        np.testing.assert_array_equal(runtime._last_action[12:], np.zeros(4))


def test_cts_runtime_fresh_zero_commands_preserve_policy_history():
    session = _Session()
    controller = ThunderCtsController(_spec(), ROOT, session=session)
    runtime = _runtime(controller)
    command = ControllerCommand("base_twist", "thunder", GENERATION, 0, 0,
                                {"linear_x": .2, "linear_y": 0., "angular_z": 0.})
    assert runtime.submit_command(command) is CommandSubmitResult.ACCEPTED
    runtime.step(_state(controller))
    first = session.feeds[-1]["single_obs"][0].copy()
    for index in range(1, 4):
        time_ns = index * 20_000_000
        idle = replace(command, sequence=index, apply_time_ns=time_ns,
                       payload={"linear_x": 0., "linear_y": 0., "angular_z": 0.})
        assert runtime.submit_command(idle) is CommandSubmitResult.ACCEPTED
        step = runtime.step(replace(_state(controller), sequence=index, sim_time_ns=time_ns))
        assert step.inference_ran and not step.actuator_command.safe_stop
        np.testing.assert_array_equal(runtime._last_action[12:], np.zeros(4))
        np.testing.assert_array_equal(session.feeds[-1]["single_obs"][0, -16:], session.action)
        np.testing.assert_array_equal(controller.runner._frames[0], first)
    assert len(session.feeds) == 4


def test_cts_actuation_and_safe_stop_use_metadata_gains_and_motor_envelope():
    session = _Session()
    session.contract["actuator_gains"]["hip"]["stiffness"] = 1.
    session.contract["actuator_gains"]["hip"]["damping"] = 0.
    controller = ThunderCtsController(_spec(), ROOT, session=session)
    velocity = np.array([7.5] * 12 + [20.] * 4)
    state = _state(controller, velocity=velocity)
    targets = controller.runner.default + 20.
    targets[12:] = 100.
    result = controller.actuate(state, targets, controller._actuators)
    expected = np.array([20., 60., 60.] * 4 + [17.] * 4)
    np.testing.assert_allclose([result[name] for name in DART_ACTUATOR_ORDER], expected)
    stopped = controller.safe_stop(state, controller._actuators)
    np.testing.assert_array_equal([stopped[name] for name in DART_ACTUATOR_ORDER[12:]], [-17.] * 4)


@pytest.mark.parametrize("changes", [{"inference_hz": 100.}, {"low_level_hz": 500.}])
def test_cts_rejects_plan_timing_that_changes_exported_control_contract(changes):
    with pytest.raises(ControllerRuntimeError, match="must match ONNX metadata"):
        ThunderCtsController(replace(_spec(), **changes), ROOT, session=_Session())


def test_factory_loads_actual_cts_export_and_produces_finite_named_torques():
    pytest.importorskip("onnxruntime")
    adapter, policy = create_production_components(_spec(), ROOT)
    assert adapter is policy and isinstance(adapter, ThunderCtsController)
    assert adapter.artifact_path == (ROOT / POLICY).resolve()
    state = _state(adapter)
    targets = policy.infer(_observe(adapter))
    torque = adapter.actuate(state, targets, adapter._actuators)
    assert tuple(torque) == adapter._actuators.channels
    assert np.isfinite(list(torque.values())).all()


@pytest.mark.parametrize("session_path", [
    "sim/sessions/products/doso/thunder_v4/default.yaml",
    "sim/sessions/examples/thunderv4_stairs/session.yaml",
])
def test_resolved_sessions_execute_cts_through_compiled_control_plan(session_path, tmp_path):
    pytest.importorskip("onnxruntime")
    resolved = CatalogResolver.from_repository(ROOT).resolve(ROOT / session_path)
    plan_path = tmp_path / "control_plan.json"
    plan_path.write_text(json.dumps(resolved.control_plan), encoding="utf-8")
    plan = load_control_plan(plan_path)
    controller, = plan.controllers
    assert controller.adapter.plugin == "thunderv4_cts_moe"
    assert controller.policy.artifact == POLICY
    assert (controller.inference_hz, controller.low_level_hz) == (50., 200.)
    adapter, policy = create_production_components(controller, ROOT)
    state = replace(_state(adapter), session_id=plan.session_id, instance_id=controller.instance_id)
    channel, = [item for item in plan.command_channels if item.direction == "subscribe"]
    command = ControllerCommand(
        channel_id=channel.channel_id, instance_id=controller.instance_id, generation=state.generation,
        sequence=0, apply_time_ns=0, payload={"linear_x": .2, "linear_y": 0., "angular_z": 0.},
    )
    targets = policy.infer(adapter.observe(state, command, controller.actuators))
    torque = adapter.actuate(state, targets, controller.actuators)
    assert tuple(torque) == controller.actuators.channels
    assert np.isfinite(list(torque.values())).all()
