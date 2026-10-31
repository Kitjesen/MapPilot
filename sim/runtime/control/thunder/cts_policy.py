"""Blind Thunder CTS MoE student with its exported training contract."""

import json

import numpy as np

from .joints import MJ_TO_DART

POLICY_CONTRACT = "thunder_cts_moe_v1"
TERM_DIMS = (3, 3, 3, 12, 16, 16)
TERM_SCALES = (0.25, 1.0, 1.0, 1.0, 0.05, 1.0)
LEG_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("fr", "fl", "rr", "rl")
    for joint in ("hip", "thigh", "calf")
)
WHEEL_NAMES = tuple(f"{side}_foot_joint" for side in ("fr", "fl", "rr", "rl"))


def _vector(values, count):
    value = np.asarray(values, dtype=np.float32)
    if value.shape != (count,) or not np.isfinite(value).all():
        raise ValueError(f"CTS policy needs {count} finite values")
    return value


class CtsPolicy:
    """Keep original term-major history and policy-rate target slew semantics."""

    def __init__(self, policy_path: str, *, session):
        self.session = session
        metadata = session.get_modelmeta().custom_metadata_map
        self.contract = json.loads(metadata["runtime_contract"])
        inputs = {node.name: tuple(node.shape) for node in session.get_inputs()}
        outputs = session.get_outputs()
        if (set(inputs) != {"history", "single_obs"}
                or inputs["history"][1:] != (530,)
                or inputs["single_obs"][1:] != (53,)
                or len(outputs) != 1 or tuple(outputs[0].shape)[1:] != (16,)):
            raise ValueError("CTS ONNX must accept history[N,530] and single_obs[N,53], returning actions[N,16]")
        self.output_name = outputs[0].name
        if self.contract["policy_leg_order"] != list(LEG_NAMES):
            raise ValueError("CTS policy leg order differs from the Thunder training contract")
        if self.contract["policy_wheel_order"] != list(WHEEL_NAMES):
            raise ValueError("CTS policy wheel order differs from the Thunder training contract")
        physics_order = self.contract["physics_order"]
        self._physics_to_policy = np.array([physics_order.index(name) for name in LEG_NAMES + WHEEL_NAMES])
        self.default = _vector(self.contract["default_q"], 16)[self._physics_to_policy]
        limits = np.asarray(self.contract["soft_joint_pos_limits"], dtype=np.float32)[self._physics_to_policy[:12]]
        if limits.shape != (12, 2) or not np.isfinite(limits).all():
            raise ValueError("CTS soft joint limits must contain twelve finite lower/upper pairs")
        self.lower, self.upper = limits[:, 0], limits[:, 1]
        self.position_scale = _vector(self.contract["position_scale"], 12)
        self.clip_actions = self.contract["clip_actions"]
        self.delta = float(self.contract["actuator_limits"]["leg_target_rate_rad_s"] * self.contract["dt"])
        self.wheel_limit = float(self.contract["actuator_limits"]["wheel_speed_rad_s"])
        self.velocity_scale = float(self.contract["velocity_scale"])
        self.run_at_idle = True
        self.zero_wheels_at_idle = True
        self.wheel_torque_limit = float(self.contract["actuator_limits"]["wheel_torque_nm"])
        self._frames = None
        self.previous_target = None
        self.last_action = np.zeros(16, dtype=np.float32)
        self.torque_limit = np.array(
            [self.contract["actuator_limits"]["leg_torque_nm"]] * 12 + [self.wheel_torque_limit] * 4,
            dtype=np.float64,
        )
        gains = self.contract["actuator_gains"]
        groups = ("hip", "thigh", "calf") * 4 + ("wheel",) * 4
        self.kp = np.array([gains[group]["stiffness"] for group in groups], dtype=np.float64)
        self.kd = np.array([gains[group]["damping"] for group in groups], dtype=np.float64)
        self.motor_speed = np.array(
            [self.contract["actuator_velocity_limit"][group] for group in groups[:12]], dtype=np.float64,
        )
        self.motor_saturation = np.array(
            [self.contract["actuator_saturation_effort"][group] for group in groups[:12]], dtype=np.float64,
        )
        self.motor_corner_speed = self.motor_speed * (1.0 + self.torque_limit[:12] / self.motor_saturation)
        print(f"[Policy] Loaded Thunder CTS MoE {policy_path}")

    @property
    def control_settings(self):
        return dict(
            standing_pose=self.default.tolist(), torque_kp=self.kp.tolist(), torque_kd=self.kd.tolist(),
            torque_limit=self.torque_limit.tolist(), obs_dim=53, history_len=10,
            policy_freq_hz=1.0 / float(self.contract["dt"]),
        )

    def reset(self):
        self._frames = None
        self.previous_target = None
        self.last_action.fill(0.0)

    def warm_up(self, gyroscope, projected_gravity, joint_position, joint_velocity):
        single = self.build_obs(gyroscope, projected_gravity, np.zeros(3), joint_position, joint_velocity)
        self._frames = np.repeat(single[None], 10, axis=0)

    def build_obs(self, gyroscope, projected_gravity, direction, joint_position, joint_velocity):
        position = _vector(joint_position, 16)[MJ_TO_DART]
        velocity = _vector(joint_velocity, 16)[MJ_TO_DART]
        if self.previous_target is None:
            self.previous_target = np.clip(position[:12], self.lower, self.upper)
        terms = (
            _vector(gyroscope, 3), _vector(projected_gravity, 3), _vector(direction, 3),
            position[:12] - self.default[:12], velocity, self.last_action,
        )
        return np.concatenate([
            np.clip(term, -100.0, 100.0) * scale for term, scale in zip(terms, TERM_SCALES)
        ]).astype(np.float32)

    def infer(self, observation):
        single = _vector(observation, 53)
        if self._frames is None:
            self._frames = np.repeat(single[None], 10, axis=0)
        else:
            self._frames[:-1] = self._frames[1:].copy()
            self._frames[-1] = single
        start, blocks = 0, []
        for size in TERM_DIMS:
            blocks.append(self._frames[:, start:start + size].reshape(-1))
            start += size
        history = np.concatenate(blocks)[None].astype(np.float32)
        raw = self.session.run([self.output_name], {"history": history, "single_obs": single[None]})[0]
        action = _vector(np.asarray(raw).reshape(-1), 16)
        if self.clip_actions is not None:
            action = np.clip(action, -self.clip_actions, self.clip_actions)
        if self.previous_target is None:
            raise RuntimeError("CTS targets must be initialized from measured joints before inference")
        requested = self.default[:12] + action[:12] * self.position_scale
        bounded = np.clip(requested, self.lower, self.upper)
        self.previous_target = np.clip(
            self.previous_target + np.clip(bounded - self.previous_target, -self.delta, self.delta),
            self.lower, self.upper,
        )
        wheels = np.clip(action[12:] * self.velocity_scale, -self.wheel_limit, self.wheel_limit)
        self.last_action = action.copy()
        return np.concatenate((self.previous_target, wheels)).astype(np.float64)

    def clip_torques(self, torques_dart, velocity_dart):
        """Original DCMotor leg torque-speed envelope; wheel PD has constant caps."""
        velocity = np.clip(
            np.asarray(velocity_dart, dtype=np.float64)[:12], -self.motor_corner_speed, self.motor_corner_speed,
        )
        upper = np.minimum(self.motor_saturation * (1.0 - velocity / self.motor_speed), self.torque_limit[:12])
        lower = np.maximum(self.motor_saturation * (-1.0 - velocity / self.motor_speed), -self.torque_limit[:12])
        limited = np.clip(torques_dart, -self.torque_limit, self.torque_limit)
        limited[:12] = np.clip(limited[:12], lower, upper)
        return limited
