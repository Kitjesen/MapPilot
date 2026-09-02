"""ONNX policy controller — PolicyRunner
# Extracted from src/drivers/sim/nova_nav_bridge.py

Original source: src/drivers/sim/nova_nav_bridge.py
Extracted content: PolicyRunner class + all related constants
Logic preserved exactly as-is.
"""

from collections import deque
from pathlib import Path

import numpy as np

# ── Original constants (ported directly from nova_nav_bridge.py) ──────────────
# Extracted from src/drivers/sim/nova_nav_bridge.py

# Dart standingPose (use as-is, no sign flip)
STANDING_POSE = np.array(
    [
        -0.1,
        -0.8,
        1.8,  # FR: hip, thigh, calf
        0.1,
        0.8,
        -1.8,  # FL
        0.1,
        0.8,
        -1.8,  # RR
        -0.1,
        -0.8,
        1.8,  # RL
        0.0,
        0.0,
        0.0,
        0.0,  # foot
    ],
    dtype=np.float64,
)

ACTION_SCALE = np.array(
    [
        0.125,
        0.25,
        0.25,  # FR
        0.125,
        0.25,
        0.25,  # FL
        0.125,
        0.25,
        0.25,  # RR
        0.125,
        0.25,
        0.25,  # RL
        5.0,
        5.0,
        5.0,
        5.0,  # foot
    ],
    dtype=np.float64,
)

THUNDERV4_STANDING_POSE = np.array(
    [
        -0.1,
        -0.8,
        1.8,
        0.1,
        0.8,
        -1.8,
        0.1,
        0.8,
        -1.8,
        -0.1,
        -0.8,
        1.8,
        0.0,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float64,
)

THUNDERV4_ACTION_SCALE = np.array(
    [
        0.15,
        0.25,
        0.25,
        0.15,
        0.25,
        0.25,
        0.15,
        0.25,
        0.25,
        0.15,
        0.25,
        0.25,
        5.0,
        5.0,
        5.0,
        5.0,
    ],
    dtype=np.float64,
)

JOINT_VEL_SCALE = np.array([0.05, 0.05, 0.05, 0.05], dtype=np.float64)
IMU_GYRO_SCALE = 0.25
OBS_DIM = 57
HISTORY_LEN = 5
POLICY_1119_STARTUP_HOLD_INFERENCE_STEPS = 25

# MuJoCo <-> Dart/ONNX joint order mapping
# MuJoCo (4+4+4+4): FR(hip,thigh,calf,foot), FL(...), RR(...), RL(...)
# Dart   (3+3+3+3+4): FR(hip,thigh,calf), FL(h,t,c), RR, RL, FR_f, FL_f, RR_f, RL_f
MJ_TO_DART = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15])
DART_TO_MJ = np.array([0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15])


class UnsupportedPolicyInputError(ValueError):
    """Raised when an ONNX policy uses an unknown observation contract."""


def _create_onnx_session(onnx_path: str, cpu_threads: int):
    import onnxruntime as ort

    threads = int(cpu_threads)
    if not 1 <= threads <= 8:
        raise ValueError("ONNX policy cpu_threads must be in [1, 8]")
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    return ort.InferenceSession(
        onnx_path,
        sess_options=options,
        providers=["CPUExecutionProvider"],
    )


class PolicyRunner:
    """ONNX gait policy inference, matching brainstem StandardObservationBuilder.

    # Extracted from src/drivers/sim/nova_nav_bridge.py — logic preserved exactly.
    """

    def __init__(
        self,
        onnx_path: str,
        *,
        session=None,
        cpu_threads: int = 1,
        startup_hold_inference_steps: int = 0,
    ):
        self.session = session or _create_onnx_session(onnx_path, cpu_threads)
        inp = self.session.get_inputs()[0]
        out = self.session.get_outputs()[0]
        print(f"[Policy] Loaded {onnx_path}")
        print(f"  input:  {inp.name} {inp.shape}")
        print(f"  output: {out.name} {out.shape}")
        self.input_name = inp.name
        self.output_name = out.name

        policy_input_dim = self._extract_input_dim(inp.shape)
        self._policy_input_dim = policy_input_dim or (OBS_DIM * HISTORY_LEN)
        self._history_len = self._resolve_history_len(self._policy_input_dim)
        self.history: deque = deque(maxlen=self._history_len)
        self.last_action = STANDING_POSE.copy()
        self.run_at_idle = False
        self.zero_wheels_at_idle = True
        self.wheel_torque_limit = 60.0
        self._startup_hold_steps = max(0, int(startup_hold_inference_steps))
        self._startup_hold_remaining = self._startup_hold_steps
        # Will be filled with real sensor data in warm_up()

    @staticmethod
    def _extract_input_dim(shape: list | tuple) -> int | None:
        if not shape:
            return None
        dim = shape[1] if len(shape) > 1 else shape[0]
        if isinstance(dim, int) and dim > 0:
            return dim
        return None

    @staticmethod
    def _resolve_history_len(policy_input_dim: int) -> int:
        if policy_input_dim % OBS_DIM == 0:
            return max(1, policy_input_dim // OBS_DIM)
        raise UnsupportedPolicyInputError(
            "Unsupported ONNX policy observation input dimension "
            f"{policy_input_dim}. This runner can only build the legacy "
            f"{OBS_DIM}-D Thunder/Dart observation, optionally stacked as "
            f"{OBS_DIM} * N history frames. A checkpoint with a "
            f"{policy_input_dim}-D input needs its original training "
            "observation field order, normalization, and action semantics "
            "before it can be run safely."
        )

    def warm_up(
        self, gyroscope: np.ndarray, projected_gravity: np.ndarray, joint_pos_16: np.ndarray, joint_vel_16: np.ndarray
    ) -> None:
        """Fill history buffer with real sensor data (matches brainstem Memory init)."""
        init_obs = self.build_obs(
            gyroscope,
            projected_gravity,
            np.zeros(3),  # direction = 0 (idle)
            joint_pos_16,
            joint_vel_16,
        )
        init_obs = self._adapt_obs_to_policy_input(init_obs)
        self.history.clear()
        for _ in range(self._history_len):
            self.history.append(init_obs.copy())
        print(f"[Policy] History warmed up: pg={projected_gravity[:3]}")

    @staticmethod
    def clamp_action(action: np.ndarray) -> np.ndarray:
        """Return real action unchanged, matching brainstem Walk.clampAction()."""
        return np.asarray(action, dtype=np.float64).copy()

    def build_obs(
        self,
        gyroscope: np.ndarray,
        projected_gravity: np.ndarray,
        direction: np.ndarray,
        joint_pos_16: np.ndarray,
        joint_vel_16: np.ndarray,
    ) -> np.ndarray:
        """Build 57-dim observation, fully matching brainstem StandardObservationBuilder.

        Note: joint_pos_16/joint_vel_16 are in MuJoCo order (4+4+4+4).
        They must be converted to Dart order (3+3+3+3+4) before computation.
        """
        gyro = gyroscope * IMU_GYRO_SCALE
        pg = projected_gravity

        # MuJoCo -> Dart joint order conversion
        jp_dart = joint_pos_16[MJ_TO_DART]
        jv_dart = joint_vel_16[MJ_TO_DART]

        # joint position: (current - standingPose), foot zeroed out
        jp = jp_dart - STANDING_POSE
        jp[12:] = 0.0  # discardFoot()

        # joint velocity: scale(hip, thigh, calf, foot)
        # brainstem: jointVelocity * (0.05, 0.05, 0.05, 0.05)
        jv = jv_dart.copy()
        for leg in range(4):
            base = leg * 3
            jv[base + 0] *= JOINT_VEL_SCALE[0]  # hip
            jv[base + 1] *= JOINT_VEL_SCALE[1]  # thigh
            jv[base + 2] *= JOINT_VEL_SCALE[2]  # calf
        for i in range(12, 16):
            jv[i] *= JOINT_VEL_SCALE[3]  # foot

        # last action: (action - standingPose) / actionScale
        # last_action is already in Dart order
        act = (self.last_action - STANDING_POSE) / ACTION_SCALE

        obs = np.concatenate([gyro, pg, direction, jp, jv, act]).astype(np.float32)
        return obs

    def _adapt_obs_to_policy_input(self, obs: np.ndarray) -> np.ndarray:
        """Validate the legacy 57-D observation before history stacking."""
        obs = np.asarray(obs, dtype=np.float32).reshape(-1)
        if obs.size == OBS_DIM:
            return obs
        raise ValueError(
            f"PolicyRunner expected one {OBS_DIM}-D observation frame before history stacking, got {obs.size} values."
        )

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Inference: obs -> append to history -> ONNX -> action(16)."""
        obs = self._adapt_obs_to_policy_input(obs)
        self.history.append(obs)
        while len(self.history) < self._history_len:
            self.history.appendleft(obs.copy())
        obs_history = np.concatenate(list(self.history)).reshape(1, -1).astype(np.float32)
        result = self.session.run([self.output_name], {self.input_name: obs_history})[0]
        raw_action = result[0]  # (16,)

        # realAction = clamp(onnxOutput * actionScale + standingPose)
        real_action = raw_action * ACTION_SCALE + STANDING_POSE
        real_action = self.clamp_action(real_action)
        self.last_action = real_action.copy()
        if self._startup_hold_remaining > 0:
            self._startup_hold_remaining -= 1
            return STANDING_POSE.copy()
        return real_action

    def reset(self) -> None:
        """Reset history buffer (call on simulation reset)."""
        self.history.clear()
        self.last_action = STANDING_POSE.copy()
        self._startup_hold_remaining = self._startup_hold_steps


class TorchScriptPolicyRunner:
    """TorchScript gait policy matching the imported Thunder v4 demo script."""

    def __init__(self, policy_path: str):
        import torch

        self._torch = torch
        self.model = torch.jit.load(policy_path, map_location="cpu")
        self.model.eval()
        self.last_action = np.zeros(16, dtype=np.float64)
        self.run_at_idle = True
        self.zero_wheels_at_idle = True
        self.wheel_torque_limit = 17.0
        self._startup_hold_steps = 10
        self._startup_hold_remaining = self._startup_hold_steps
        print(f"[Policy] Loaded TorchScript {policy_path}")

    def warm_up(
        self, gyroscope: np.ndarray, projected_gravity: np.ndarray, joint_pos_16: np.ndarray, joint_vel_16: np.ndarray
    ) -> None:
        del gyroscope, projected_gravity, joint_pos_16, joint_vel_16
        self.last_action = np.zeros(16, dtype=np.float64)
        self._startup_hold_remaining = self._startup_hold_steps

    @staticmethod
    def clamp_action(action: np.ndarray) -> np.ndarray:
        return np.asarray(action, dtype=np.float64).copy()

    def build_obs(
        self,
        gyroscope: np.ndarray,
        projected_gravity: np.ndarray,
        direction: np.ndarray,
        joint_pos_16: np.ndarray,
        joint_vel_16: np.ndarray,
    ) -> np.ndarray:
        jp_dart = joint_pos_16[MJ_TO_DART]
        jv_dart = joint_vel_16[MJ_TO_DART]
        q = jp_dart - THUNDERV4_STANDING_POSE
        q[12:] = 0.0
        dq = jv_dart * 0.05
        return np.concatenate(
            [
                gyroscope * IMU_GYRO_SCALE,
                projected_gravity,
                direction,
                q,
                dq,
                self.last_action,
            ]
        ).astype(np.float32)

    def infer(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if obs.shape[1] != OBS_DIM:
            raise ValueError(f"TorchScriptPolicyRunner expected {OBS_DIM}-D obs, got {obs.shape[1]}")
        with self._torch.inference_mode():
            tensor = self._torch.from_numpy(obs).to(dtype=self._torch.float32)
            raw_action = self.model(tensor).detach().cpu().numpy().reshape(-1)
        if raw_action.size != 16:
            raise ValueError(f"TorchScriptPolicyRunner expected 16-D action, got {raw_action.size}")
        self.last_action = raw_action.astype(np.float64, copy=True)
        if self._startup_hold_remaining > 0:
            self._startup_hold_remaining -= 1
            return THUNDERV4_STANDING_POSE.copy()
        real_action = raw_action * THUNDERV4_ACTION_SCALE + THUNDERV4_STANDING_POSE
        return self.clamp_action(real_action)

    def reset(self) -> None:
        self.last_action = np.zeros(16, dtype=np.float64)
        self._startup_hold_remaining = self._startup_hold_steps


class ThunderV4OnnxPolicyRunner(TorchScriptPolicyRunner):
    """ONNX runtime for the converted ThunderV4 TorchScript policy."""

    def __init__(self, policy_path: str, *, session=None, cpu_threads: int = 1):
        self.session = session or _create_onnx_session(policy_path, cpu_threads)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.last_action = np.zeros(16, dtype=np.float64)
        self.run_at_idle = True
        self.zero_wheels_at_idle = True
        self.wheel_torque_limit = 17.0
        self._startup_hold_steps = 10
        self._startup_hold_remaining = self._startup_hold_steps
        print(f"[Policy] Loaded ThunderV4 ONNX {policy_path}")

    def infer(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if obs.shape[1] != OBS_DIM:
            raise ValueError(f"ThunderV4OnnxPolicyRunner expected {OBS_DIM}-D obs, got {obs.shape[1]}")
        raw_action = self.session.run([self.output_name], {self.input_name: obs})[0].reshape(-1)
        if raw_action.size != 16:
            raise ValueError(f"ThunderV4OnnxPolicyRunner expected 16-D action, got {raw_action.size}")
        self.last_action = raw_action.astype(np.float64, copy=True)
        if self._startup_hold_remaining > 0:
            self._startup_hold_remaining -= 1
            return THUNDERV4_STANDING_POSE.copy()
        real_action = raw_action * THUNDERV4_ACTION_SCALE + THUNDERV4_STANDING_POSE
        return self.clamp_action(real_action)


def _is_thunderv4_policy(path: Path) -> bool:
    normalized = path.as_posix().lower()
    return (
        "thunder_v4/locomotion/policy/" in normalized
        or "thunderv4/policy" in normalized
        or path.name.startswith("pose_flat_low_kpkd")
    )


def load_policy_runner(policy_path: str, *, cpu_threads: int = 1):
    path = Path(policy_path)
    if path.suffix.lower() in {".pt", ".pth", ".jit"}:
        return TorchScriptPolicyRunner(policy_path)
    if path.suffix.lower() == ".onnx":
        session = _create_onnx_session(policy_path, cpu_threads)
        input_dim = PolicyRunner._extract_input_dim(session.get_inputs()[0].shape)
        if input_dim == OBS_DIM and _is_thunderv4_policy(path):
            return ThunderV4OnnxPolicyRunner(
                policy_path, session=session, cpu_threads=cpu_threads
            )
        startup_hold_steps = (
            POLICY_1119_STARTUP_HOLD_INFERENCE_STEPS
            if path.name.lower() == "policy_1119.onnx"
            else 0
        )
        return PolicyRunner(
            policy_path,
            session=session,
            cpu_threads=cpu_threads,
            startup_hold_inference_steps=startup_hold_steps,
        )
    startup_hold_steps = (
        POLICY_1119_STARTUP_HOLD_INFERENCE_STEPS
        if path.name.lower() == "policy_1119.onnx"
        else 0
    )
    return PolicyRunner(
        policy_path,
        cpu_threads=cpu_threads,
        startup_hold_inference_steps=startup_hold_steps,
    )
