"""Historical Thunder policy runners for explicit simulation comparisons.

Sessions are opened by policy_loader; these runners retain their training-time
observation, history, action scaling, and startup behavior.
"""

from collections import deque
from pathlib import Path

import numpy as np

from . import flat
from .joints import MJ_TO_DART

STANDING_POSE = np.array(
    (-0.1, -0.8, 1.8, 0.1, 0.8, -1.8, 0.1, 0.8, -1.8, -0.1, -0.8, 1.8) + (0.0,) * 4,
    dtype=np.float64,
)
ACTION_SCALE = np.array((0.125, 0.25, 0.25) * 4 + (5.0,) * 4, dtype=np.float64)
DEMO_SCALE = np.array((0.15, 0.25, 0.25) * 4 + (5.0,) * 4, dtype=np.float64)
IMU_GYRO_SCALE = 0.25
OBS_DIM = 57
HISTORY_LEN = 5


class HistoryPolicy:
    """Legacy 57-D Thunder/Dart observations with optional frame history."""

    def __init__(
        self,
        onnx_path: str,
        *,
        session,
        startup_steps: int = 0,
    ):
        self.session = session
        inp = self.session.get_inputs()[0]
        out = self.session.get_outputs()[0]
        print(f"[Policy] Loaded {onnx_path}")
        print(f"  input:  {inp.name} {inp.shape}")
        print(f"  output: {out.name} {out.shape}")
        self.input_name = inp.name
        self.output_name = out.name

        policy_input_dim = self.input_size(inp.shape)
        frames = self.history_frames(policy_input_dim or (OBS_DIM * HISTORY_LEN))
        self.history = deque(maxlen=frames)
        self.last_action = STANDING_POSE.copy()
        self.run_at_idle = False
        self.zero_wheels_at_idle = True
        self.wheel_torque_limit = 60.0
        self.startup_steps = max(0, int(startup_steps))
        self._hold = self.startup_steps

    @staticmethod
    def input_size(shape: list | tuple) -> int | None:
        if not shape:
            return None
        dim = shape[1] if len(shape) > 1 else shape[0]
        if isinstance(dim, int) and dim > 0:
            return dim
        return None

    @staticmethod
    def history_frames(policy_input_dim: int) -> int:
        if policy_input_dim % OBS_DIM == 0:
            return max(1, policy_input_dim // OBS_DIM)
        raise ValueError(f"Unsupported {policy_input_dim}-D input; expected {OBS_DIM} * N values.")

    def warm_up(
        self, gyroscope: np.ndarray, projected_gravity: np.ndarray, joint_pos_16: np.ndarray, joint_vel_16: np.ndarray
    ) -> None:
        """Fill history with measured sensor data (matches brainstem Memory init)."""
        init_obs = self.build_obs(
            gyroscope,
            projected_gravity,
            np.zeros(3),  # direction = 0 (idle)
            joint_pos_16,
            joint_vel_16,
        )
        init_obs = self._frame(init_obs)
        self.history.clear()
        for _ in range(self.history.maxlen):
            self.history.append(init_obs.copy())
        print(f"[Policy] History warmed up: pg={projected_gravity[:3]}")

    def build_obs(
        self,
        gyroscope: np.ndarray,
        projected_gravity: np.ndarray,
        direction: np.ndarray,
        joint_pos_16: np.ndarray,
        joint_vel_16: np.ndarray,
    ) -> np.ndarray:
        """Build a 57-D training observation from MuJoCo joint order."""
        position = joint_pos_16[MJ_TO_DART] - STANDING_POSE
        # Wheel angles are unbounded; training discards them.
        position[12:] = 0.0
        velocity = joint_vel_16[MJ_TO_DART] * 0.05
        previous_action = (self.last_action - STANDING_POSE) / ACTION_SCALE
        return np.concatenate(
            [gyroscope * IMU_GYRO_SCALE, projected_gravity, direction, position, velocity, previous_action]
        ).astype(np.float32)

    def _frame(self, obs: np.ndarray) -> np.ndarray:
        """Validate the legacy 57-D observation before history stacking."""
        obs = np.asarray(obs, dtype=np.float32).reshape(-1)
        if obs.size == OBS_DIM:
            return obs
        raise ValueError(
            f"HistoryPolicy expected one {OBS_DIM}-D observation frame before history stacking, got {obs.size} values."
        )

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Inference: obs -> append to history -> ONNX -> action(16)."""
        obs = self._frame(obs)
        self.history.append(obs)
        while len(self.history) < self.history.maxlen:
            self.history.appendleft(obs.copy())
        obs_history = np.concatenate(list(self.history)).reshape(1, -1).astype(np.float32)
        result = self.session.run([self.output_name], {self.input_name: obs_history})[0]
        raw_action = result[0]  # (16,)

        real_action = raw_action * ACTION_SCALE + STANDING_POSE
        self.last_action = real_action.copy()
        if self._hold > 0:
            self._hold -= 1
            return STANDING_POSE.copy()
        return real_action

    def reset(self) -> None:
        """Reset history buffer (call on simulation reset)."""
        self.history.clear()
        self.last_action = STANDING_POSE.copy()
        self._hold = self.startup_steps


class TorchPolicy:
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
        self.startup_steps = 10
        self._hold = self.startup_steps
        print(f"[Policy] Loaded TorchScript {policy_path}")

    def warm_up(
        self, gyroscope: np.ndarray, projected_gravity: np.ndarray, joint_pos_16: np.ndarray, joint_vel_16: np.ndarray
    ) -> None:
        del gyroscope, projected_gravity, joint_pos_16, joint_vel_16
        self.last_action = np.zeros(16, dtype=np.float64)
        self._hold = self.startup_steps

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
        q = jp_dart - STANDING_POSE
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
            raise ValueError(f"TorchPolicy expected {OBS_DIM}-D obs, got {obs.shape[1]}")
        with self._torch.inference_mode():
            tensor = self._torch.from_numpy(obs).to(dtype=self._torch.float32)
            raw_action = self.model(tensor).detach().cpu().numpy().reshape(-1)
        if raw_action.size != 16:
            raise ValueError(f"TorchPolicy expected 16-D action, got {raw_action.size}")
        self.last_action = raw_action.astype(np.float64, copy=True)
        if self._hold > 0:
            self._hold -= 1
            return STANDING_POSE.copy()
        real_action = raw_action * DEMO_SCALE + STANDING_POSE
        return real_action

    def reset(self) -> None:
        self.last_action = np.zeros(16, dtype=np.float64)
        self._hold = self.startup_steps


class OnnxPolicy(TorchPolicy):
    """ONNX runtime for the converted ThunderV4 TorchScript policy."""

    def __init__(self, policy_path: str, *, session):
        self.session = session
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.last_action = np.zeros(16, dtype=np.float64)
        self.run_at_idle = True
        self.zero_wheels_at_idle = True
        self.wheel_torque_limit = 17.0
        self.startup_steps = 10
        self._hold = self.startup_steps
        print(f"[Policy] Loaded ThunderV4 ONNX {policy_path}")

    def infer(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if obs.shape[1] != OBS_DIM:
            raise ValueError(f"OnnxPolicy expected {OBS_DIM}-D obs, got {obs.shape[1]}")
        raw_action = self.session.run([self.output_name], {self.input_name: obs})[0].reshape(-1)
        if raw_action.size != 16:
            raise ValueError(f"OnnxPolicy expected 16-D action, got {raw_action.size}")
        self.last_action = raw_action.astype(np.float64, copy=True)
        if self._hold > 0:
            self._hold -= 1
            return STANDING_POSE.copy()
        real_action = raw_action * DEMO_SCALE + STANDING_POSE
        return real_action


class FlatPolicy(OnnxPolicy):
    """Single-frame 53-D G2 policy with its original observation and action layout."""

    def __init__(self, policy_path: str, *, session):
        super().__init__(policy_path, session=session)
        self.startup_steps = 25
        self._hold = self.startup_steps
        # Keep leg balance active while the existing idle controller parks the wheels.

    def build_obs(self, gyroscope, projected_gravity, direction, joint_pos_16, joint_vel_16):
        return flat.observation(
            gyroscope,
            projected_gravity,
            direction,
            joint_pos_16[MJ_TO_DART],
            joint_vel_16[MJ_TO_DART],
            self.last_action,
        )

    def infer(self, obs: np.ndarray) -> np.ndarray:
        frame = np.asarray(obs, dtype=np.float32).reshape(1, 53)
        raw_action = self.session.run([self.output_name], {self.input_name: frame})[0].reshape(-1)
        if raw_action.size != 16 or not np.isfinite(raw_action).all():
            raise ValueError("Thunder flat policy must produce 16 finite actions")
        self.last_action = raw_action.astype(np.float64, copy=True)
        if self._hold > 0:
            self._hold -= 1
            return flat.STANDING_POSE.copy()
        return flat.action_targets(raw_action)


def _is_thunder(path: Path) -> bool:
    name = path.as_posix().lower()
    return (
        "thunder_v4/locomotion/policy/" in name
        or "thunderv4/policy" in name
        or path.name.startswith("pose_flat_low_kpkd")
    )


def load(path: str, *, session):
    """Select an explicitly requested historical Thunder model."""
    file = Path(path)
    size = HistoryPolicy.input_size(session.get_inputs()[0].shape)
    if _is_thunder(file):
        if size == 53:
            return FlatPolicy(path, session=session)
        if size == OBS_DIM:
            return OnnxPolicy(path, session=session)
    startup_steps = 25 if file.name.lower() == "policy_1119.onnx" else 0
    return HistoryPolicy(path, session=session, startup_steps=startup_steps)
