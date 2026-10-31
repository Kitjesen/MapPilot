"""Loading must retain each historical model's input and action semantics."""

from types import SimpleNamespace

import numpy as np
import pytest
from sim.runtime.control import policy_loader
from sim.runtime.control.thunder import flat as thunderv4_flat
from sim.runtime.control.thunder import legacy_policy as thunder_legacy


@pytest.mark.parametrize(
    "path,input_dim,hold_steps,standing,scale",
    [
        ("model.onnx", 57, 0, thunder_legacy.STANDING_POSE, thunder_legacy.ACTION_SCALE),
        (
            "thunderv4/policy/frame.onnx",
            57,
            10,
            thunder_legacy.STANDING_POSE,
            thunder_legacy.DEMO_SCALE,
        ),
        (
            "thunder_v4/locomotion/policy/policy_1119.onnx",
            285,
            25,
            thunder_legacy.STANDING_POSE,
            thunder_legacy.ACTION_SCALE,
        ),
        (
            "thunder_v4/locomotion/policy/policy_4998.onnx",
            53,
            25,
            thunderv4_flat.STANDING_POSE,
            thunderv4_flat.ACTION_SCALE,
        ),
    ],
)
def test_legacy_actions(
    monkeypatch,
    path,
    input_dim,
    hold_steps,
    standing,
    scale,
):
    raw_action = np.linspace(-1.0, 1.0, 16, dtype=np.float32)
    feeds = []

    def run(names, inputs):
        assert names == ["actions"]
        feeds.append(inputs["obs"].copy())
        return [raw_action[None].copy()]

    session = SimpleNamespace(
        get_modelmeta=lambda: SimpleNamespace(custom_metadata_map={}),
        get_inputs=lambda: [SimpleNamespace(name="obs", shape=[1, input_dim])],
        get_outputs=lambda: [SimpleNamespace(name="actions", shape=[1, 16])],
        run=run,
    )

    def unexpected_session(*_args):
        raise AssertionError("A supplied session must not be reopened")

    monkeypatch.setattr(policy_loader, "open_session", unexpected_session)
    runner = policy_loader.load_policy(path, session=session)
    assert runner.session is session
    observation = np.arange(53 if input_dim == 53 else 57, dtype=np.float32)

    for _ in range(hold_steps):
        np.testing.assert_array_equal(runner.infer(observation), standing)
    np.testing.assert_allclose(runner.infer(observation), standing + raw_action * scale)
    assert feeds[-1].shape == (1, input_dim)
    np.testing.assert_array_equal(feeds[-1][0], np.tile(observation, input_dim // observation.size))

    runner.reset()
    expected = standing if hold_steps else standing + raw_action * scale
    np.testing.assert_allclose(runner.infer(observation), expected)


def test_torch_load(tmp_path, monkeypatch):
    torch = pytest.importorskip("torch")
    model = torch.jit.trace(torch.nn.Linear(57, 16), torch.zeros(1, 57))
    path = tmp_path / "policy.jit"
    model.save(str(path))

    def unexpected_session(*_args):
        raise AssertionError("TorchScript must not open an ONNX session")

    monkeypatch.setattr(policy_loader, "open_session", unexpected_session)
    runner = policy_loader.load_policy(str(path))
    observation = np.arange(57, dtype=np.float32)
    for _ in range(10):
        np.testing.assert_array_equal(runner.infer(observation), thunder_legacy.STANDING_POSE)
    with torch.inference_mode():
        raw_action = model(torch.from_numpy(observation[None])).numpy()[0]
    expected = thunder_legacy.STANDING_POSE + raw_action * thunder_legacy.DEMO_SCALE
    np.testing.assert_allclose(runner.infer(observation), expected)


def test_history_observation():
    session = SimpleNamespace(
        get_inputs=lambda: [SimpleNamespace(name="obs", shape=[1, 285])],
        get_outputs=lambda: [SimpleNamespace(name="actions", shape=[1, 16])],
    )
    policy = thunder_legacy.HistoryPolicy("model.onnx", session=session)
    position = np.arange(16, dtype=np.float64) * 0.1
    velocity = np.arange(16, dtype=np.float64) * 2
    gyro = np.array([1.0, -2.0, 3.0])
    gravity = np.array([0.0, 0.0, -1.0])
    command = np.array([0.2, -0.1, 0.3])
    obs = policy.build_obs(gyro, gravity, command, position, velocity)
    # Literal MuJoCo indices make joint-order regressions visible.
    order = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15]
    np.testing.assert_allclose(obs[:3], gyro * 0.25)
    np.testing.assert_allclose(obs[3:6], gravity)
    np.testing.assert_allclose(obs[6:9], command)
    np.testing.assert_allclose(obs[9:21], (position[order] - thunder_legacy.STANDING_POSE)[:12])
    np.testing.assert_array_equal(obs[21:25], np.zeros(4))
    np.testing.assert_allclose(obs[25:41], velocity[order] * 0.05)
    np.testing.assert_array_equal(obs[41:], np.zeros(16))
