from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from drivers.sim.mujoco.runtime import build_engine
from sim.engine.core.engine import VelocityCommand
from sim.runtime.control.thunderv4 import DART_ACTUATOR_ORDER


@pytest.mark.parametrize(
    ("initial_keyframe", "expected_mj"),
    [
        (
            "v4_nominal_stand",
            np.asarray(
                [
                    -0.1,
                    -0.8,
                    1.8,
                    0.0,
                    0.1,
                    0.8,
                    -1.8,
                    0.0,
                    0.1,
                    0.8,
                    -1.8,
                    0.0,
                    -0.1,
                    -0.8,
                    1.8,
                    0.0,
                ]
            ),
        ),
        (None, np.zeros(16)),
    ],
)
def test_reset_uses_keyframe_joints_or_standing_pose(
    monkeypatch: pytest.MonkeyPatch,
    initial_keyframe: str | None,
    expected_mj: np.ndarray,
) -> None:
    mujoco = pytest.importorskip("mujoco")
    from sim.engine.core.robot import RobotConfig
    from sim.engine.mujoco.engine import MuJoCoEngine

    root = Path(__file__).resolve().parents[2]
    config = RobotConfig.default_thunder_v4()
    config.robot_xml = str(root / "sim/robots/doso/thunder_v4/mjcf/thunderv4.xml")
    config.standing_pose = [0.0] * 16
    engine = MuJoCoEngine(
        robot_config=config,
        drive_mode="kinematic",
        initial_keyframe=initial_keyframe,
    )
    try:
        engine.load(config.robot_xml)
        monkeypatch.setattr(mujoco, "mj_step", lambda _model, _data: None)

        state = engine.reset()

        np.testing.assert_allclose(state.joint_positions, expected_mj)
        np.testing.assert_allclose(engine._last_leg_targets_mj, expected_mj)
    finally:
        engine.close()


def test_product_keyframe_and_spawn_offset_form_a_stable_base_pose() -> None:
    pytest.importorskip("mujoco")
    root = Path(__file__).resolve().parents[2]
    engine = build_engine(
        world=root / "sim/worlds/mujoco/open_field.xml",
        robot_xml=root / "sim/robots/doso/thunder_v4/mjcf/thunderv4.xml",
        drive_mode="kinematic",
        start=[1.0, 2.0, 0.0],
        start_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        initial_keyframe="v4_nominal_stand",
        mujoco_memory="64M",
        mid360_pattern=None,
        mid360_samples_per_frame=32,
        require_product_lidar_backend=False,
    )
    try:
        start = engine.get_robot_state().position.copy()
        for _ in range(100):
            state = engine.step(VelocityCommand(linear_x=0.2))
    finally:
        engine.close()

    assert start[0] == pytest.approx(1.0, abs=0.05)
    assert start[1] == pytest.approx(2.0, abs=0.05)
    assert 0.3 <= start[2] <= 0.8
    assert state.position[2] == pytest.approx(start[2], abs=0.02)
    assert math.hypot(
        float(state.position[0] - start[0]),
        float(state.position[1] - start[1]),
    ) == pytest.approx(0.2, abs=0.01)


def test_policy_mode_requires_the_declared_thunderv4_baseline(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from drivers.sim.mujoco import driver as driver_module

    missing = tmp_path / "policy_1119.onnx"
    monkeypatch.setattr(driver_module, "_THUNDERV4_POLICY", missing)

    with pytest.raises(FileNotFoundError, match="ThunderV4 policy is missing"):
        driver_module.MujocoDriverModule(drive_mode="policy")

    driver = driver_module.MujocoDriverModule(drive_mode="kinematic")
    assert driver._policy_path == ""


def test_keyboard_entrypoint_loads_policy_1119_as_onnx() -> None:
    torch = pytest.importorskip("torch")
    pytest.importorskip("onnxruntime")
    from sim.controllers.doso.thunder_v4.locomotion import keyboard

    policy = keyboard._load_policy(str(keyboard.DEFAULT_POLICY_PATH))
    action = policy(torch.zeros((1, 57), dtype=torch.float32))

    assert tuple(action.shape) == (1, 16)
    assert keyboard.DEFAULT_PHYSICS_TIMESTEP_S == pytest.approx(0.005)
    assert keyboard.DEFAULT_POLICY_DECIMATION == 4


def test_root_joint_resolution_uses_the_configured_base_body() -> None:
    mujoco = pytest.importorskip("mujoco")
    from sim.engine.core.robot import RobotConfig
    from sim.engine.mujoco.engine import MuJoCoEngine

    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="foreign">
              <freejoint name="foreign_root"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
            <body name="base_link">
              <freejoint name="robot_root"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    config = RobotConfig.default_thunder_v4()
    config.base_body_name = "base_link"
    engine = MuJoCoEngine(robot_config=config, drive_mode="kinematic")
    engine._model = model
    engine._base_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "base_link",
    )

    engine._resolve_root_joint_adrs()

    robot_joint = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "robot_root",
    )
    assert engine._root_qposadr == int(model.jnt_qposadr[robot_joint])
    assert engine._root_dofadr == int(model.jnt_dofadr[robot_joint])


def test_controller_channel_order_does_not_replace_physical_joint_order() -> None:
    pytest.importorskip("mujoco")
    pytest.importorskip("torch")
    root = Path(__file__).resolve().parents[2]
    engine = build_engine(
        world=root / "sim/worlds/mujoco/open_field.xml",
        robot_xml=root / "sim/robots/doso/thunder_v4/mjcf/thunderv4.xml",
        drive_mode="policy",
        start=[0.0, 0.0, 0.0],
        start_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        initial_keyframe="v4_nominal_stand",
        controller_actuator_names=list(DART_ACTUATOR_ORDER),
        policy_path=(root / "sim/controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx"),
        policy_freq_hz=50.0,
        mujoco_memory="64M",
        mid360_pattern=None,
        mid360_samples_per_frame=32,
        require_product_lidar_backend=False,
    )
    heights: list[float] = []
    try:
        zero = VelocityCommand(linear_x=0.0, linear_y=0.0, angular_z=0.0)
        for _ in range(500):
            state = engine.step_sensor_tick(zero, 0.002)
            heights.append(float(state.position[2]))
    finally:
        engine.close()

    assert min(heights) >= 0.35
    assert max(heights) - min(heights) <= 0.20


def test_exact_package_policy_remains_upright_at_the_formal_sensor_period() -> None:
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    root = Path(__file__).resolve().parents[2]
    random_state = np.random.get_state()
    engine = None
    zero = VelocityCommand(linear_x=0.0, linear_y=0.0, angular_z=0.0)
    try:
        np.random.seed(1)
        engine = build_engine(
            world=(root / "sim/packages/worlds/open_field/1.1.0/physics/open_field.xml"),
            robot_xml=(root / "sim/robots/doso/thunder_v4/mjcf/thunderv4.xml"),
            drive_mode="policy",
            start=[0.0, 0.0, 0.0],
            start_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
            initial_keyframe="v4_nominal_stand",
            controller_actuator_names=list(DART_ACTUATOR_ORDER),
            policy_path=(root / "sim/controllers/doso/thunder_v4/locomotion/policy/policy_1119.onnx"),
            policy_freq_hz=50.0,
            physics_timestep_s=0.001,
            mujoco_memory="64M",
            mid360_pattern=None,
            mid360_samples_per_frame=32,
            require_product_lidar_backend=False,
        )
        for step in range(1, 501):
            state = engine.step_sensor_tick(zero, 1.0 / 200.0)
            x, y, z, w = (float(value) for value in state.orientation)
            roll_deg = math.degrees(
                math.atan2(
                    2.0 * (w * x + y * z),
                    1.0 - 2.0 * (x * x + y * y),
                )
            )
            pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))
            height = float(state.position[2])
            assert height >= 0.2, f"step={step} height={height:.6f} roll={roll_deg:.6f} pitch={pitch_deg:.6f}"
            assert abs(roll_deg) <= 30.0, f"step={step} height={height:.6f} roll={roll_deg:.6f} pitch={pitch_deg:.6f}"
            assert abs(pitch_deg) <= 30.0, f"step={step} height={height:.6f} roll={roll_deg:.6f} pitch={pitch_deg:.6f}"
    finally:
        if engine is not None:
            engine.close()
        np.random.set_state(random_state)
