import threading
import time
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import torch
from pynput import keyboard as pynput_keyboard
from scipy.spatial.transform import Rotation as R

ASSET_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = ASSET_DIR / "mjcf" / "thunderv4.xml"
DEFAULT_POLICY_PATH = ASSET_DIR / "policy" / "pose_flat_low_kpkd_microterrain_model29600_policy.pt"

ACTUATOR_JOINTS = (
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
HARDWARE_VELOCITY_NUMERIC = "hardware_joint_velocity_limit_rad_s"


def resolve_joint_addresses(model):
    """Resolve state and control addresses from names, not XML tree order."""
    joint_ids = []
    actuator_ids = []
    for name in ACTUATOR_JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if joint_id < 0 or actuator_id < 0:
            raise RuntimeError(f"Thunder V4 MJCF is missing required joint or actuator: {name}")
        joint_ids.append(joint_id)
        actuator_ids.append(actuator_id)

    actuator_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) for actuator_id in range(model.nu)
    )
    if actuator_names != ACTUATOR_JOINTS:
        raise RuntimeError(
            "Thunder V4 policy action order does not match the loaded MJCF. "
            f"Expected {ACTUATOR_JOINTS}, got {actuator_names}."
        )

    joint_ids = np.asarray(joint_ids, dtype=np.int32)
    return (
        model.jnt_qposadr[joint_ids].astype(np.int32),
        model.jnt_dofadr[joint_ids].astype(np.int32),
        np.asarray(actuator_ids, dtype=np.int32),
    )


def resolve_hardware_velocity_limits(model):
    """Load per-joint speed limits generated from the V4 source URDF."""
    numeric_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_NUMERIC, HARDWARE_VELOCITY_NUMERIC)
    if numeric_id < 0:
        raise RuntimeError(
            "Thunder V4 MJCF is missing hardware_joint_velocity_limit_rad_s. "
            "Regenerate it with generate_thunderv4_mjcf.py."
        )
    start = model.numeric_adr[numeric_id]
    size = model.numeric_size[numeric_id]
    limits = np.asarray(model.numeric_data[start : start + size], dtype=np.float64)
    if limits.shape != (len(ACTUATOR_JOINTS),) or np.any(limits <= 0.0):
        raise RuntimeError(f"Invalid Thunder V4 joint velocity limits: {limits}")
    return limits


def attenuate_overspeed_torque(tau, qvel, velocity_limits):
    """Remove same-direction drive torque from 90% of the hardware speed limit."""
    speed_ratio = np.abs(qvel) / velocity_limits
    rolloff = np.clip((1.0 - speed_ratio) / 0.10, 0.0, 1.0)
    accelerated = tau * qvel > 0.0
    guarded_tau = tau.copy()
    guarded_tau[accelerated] *= rolloff[accelerated]
    return guarded_tau


class SimConfig:
    def __init__(self, dt=0.001, decimation=20, sim_duration=60.0):
        self.sim_duration = sim_duration
        self.dt = dt
        self.decimation = decimation


class RobotConfig:
    def __init__(self):
        # Joint names (order matches action order)
        self.joint_names = [
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
        ]

        # Individual joint stiffness (kp) [Nm/rad]
        # NOTE: Hip values increased for better tracking (differs from training)
        self.kp = {
            "FR_hip_joint": 50.0,
            "FL_hip_joint": 50.0,
            "RR_hip_joint": 50.0,
            "RL_hip_joint": 50.0,
            "FR_thigh_joint": 50.0,
            "FL_thigh_joint": 50.0,
            "RR_thigh_joint": 50.0,
            "RL_thigh_joint": 50.0,
            "FR_calf_joint": 50.0,
            "FL_calf_joint": 50.0,
            "RR_calf_joint": 50.0,
            "RL_calf_joint": 50.0,
            "FR_foot_joint": 0.0,
            "FL_foot_joint": 0.0,
            "RR_foot_joint": 0.0,
            "RL_foot_joint": 0.0,
        }

        # Individual joint damping (kd) [Nm·s/rad]
        # NOTE: Hip values increased for better tracking (differs from training)
        self.kd = {
            "FR_hip_joint": 7.5,
            "FL_hip_joint": 7.5,
            "RR_hip_joint": 7.5,
            "RL_hip_joint": 7.5,
            "FR_thigh_joint": 7.5,
            "FL_thigh_joint": 7.5,
            "RR_thigh_joint": 7.5,
            "RL_thigh_joint": 7.5,
            "FR_calf_joint": 7.5,
            "FL_calf_joint": 7.5,
            "RR_calf_joint": 7.5,
            "RL_calf_joint": 7.5,
            "FR_foot_joint": 1.0,
            "FL_foot_joint": 1.0,
            "RR_foot_joint": 1.0,
            "RL_foot_joint": 1.0,
        }

        # Individual joint effort limits [Nm]
        self.tau_limit = {
            "FR_hip_joint": 120.0,
            "FL_hip_joint": 120.0,
            "RR_hip_joint": 120.0,
            "RL_hip_joint": 120.0,
            "FR_thigh_joint": 120.0,
            "FL_thigh_joint": 120.0,
            "RR_thigh_joint": 120.0,
            "RL_thigh_joint": 120.0,
            "FR_calf_joint": 120.0,
            "FL_calf_joint": 120.0,
            "RR_calf_joint": 120.0,
            "RL_calf_joint": 120.0,
            "FR_foot_joint": 17.0,
            "FL_foot_joint": 17.0,
            "RR_foot_joint": 17.0,
            "RL_foot_joint": 17.0,
        }

        # Action scaling
        # Action scales (must match training config!)
        self.hip_scale = 0.15  # hip joints
        self.thigh_calf_scale = 0.25  # thigh/calf joints
        self.wheel_scale = 5.0  # wheel joints

        # Initial height
        self.init_height = 0.55  # [m], matches the Isaac training default base height

        # Convert dicts to arrays (ordered by joint_names)
        self.kp_array = np.array([self.kp[name] for name in self.joint_names])
        self.kd_array = np.array([self.kd[name] for name in self.joint_names])
        self.tau_limit_array = np.array([self.tau_limit[name] for name in self.joint_names])

    def print_parameters(self):
        """Print all PD control parameters for verification"""
        print("\n" + "=" * 70)
        print("PD Control Parameters Configuration")
        print("=" * 70)
        print(f"{'Joint Name':<20} {'Kp [Nm/rad]':>12} {'Kd [Nm·s/rad]':>15} {'τ_limit [Nm]':>15}")
        print("-" * 70)
        for name in self.joint_names:
            print(f"{name:<20} {self.kp[name]:>12.1f} {self.kd[name]:>15.1f} {self.tau_limit[name]:>15.1f}")
        print("=" * 70 + "\n")


class Config:
    def __init__(self):
        self.sim_config = SimConfig()
        self.robot_config = RobotConfig()


cfg = Config()


default_joint_angles = {
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": -0.8,
    "FR_calf_joint": 1.8,
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 0.8,
    "FL_calf_joint": -1.8,
    "RR_hip_joint": 0.1,
    "RR_thigh_joint": 0.8,
    "RR_calf_joint": -1.8,
    "RL_hip_joint": -0.1,
    "RL_thigh_joint": -0.8,
    "RL_calf_joint": 1.8,
    "FR_foot_joint": 0.0,
    "FL_foot_joint": 0.0,
    "RR_foot_joint": 0.0,
    "RL_foot_joint": 0.0,
}

# Define the two poses
POSE_A_ANGLES = default_joint_angles.copy()  # Legacy policy observation reference.

# 定义中间过渡姿态 (Mid Pose)
# 取 Pose A 和 Pose B 的中间值，用于缓冲
POSE_MID_ANGLES = {
    # 中间姿态 (Mid Pose)
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": -1.0,
    "FR_calf_joint": 2.35,
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 1.0,
    "FL_calf_joint": -2.35,
    "RR_hip_joint": 0.1,
    "RR_thigh_joint": 1.0,
    "RR_calf_joint": -2.35,
    "RL_hip_joint": -0.1,
    "RL_thigh_joint": -1.0,
    "RL_calf_joint": 2.35,
    "FR_foot_joint": 0.0,
    "FL_foot_joint": 0.0,
    "RR_foot_joint": 0.0,
    "RL_foot_joint": 0.0,
}

POSE_B_ANGLES = {
    # 站高姿态 (High Stand)：用户实测参数
    # 增大 Thigh 和 Calf 的绝对值以实现站高
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": -1.2,
    "FR_calf_joint": 2.7,
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 1.2,
    "FL_calf_joint": -2.7,
    "RR_hip_joint": 0.1,
    "RR_thigh_joint": 1.2,
    "RR_calf_joint": -2.7,
    "RL_hip_joint": -0.1,
    "RL_thigh_joint": -1.2,
    "RL_calf_joint": 2.7,
    "FR_foot_joint": 0.0,
    "FL_foot_joint": 0.0,
    "RR_foot_joint": 0.0,
    "RL_foot_joint": 0.0,
}

current_pose_name = "A"


def transition_pose(target_pose_list, duration=5.0, pause=5.0):
    """
    执行序列化过渡：当前 -> Target 1 -> 停顿 -> Target 2 -> ...
    target_pose_list: 可以是单个字典，也可以是字典的列表 [Pose_Mid, Pose_B]
    duration: 每个阶段的过渡时间
    pause: 每个阶段结束后的停顿稳定时间
    """
    global default_angle

    # 如果传入的是单个字典，转为列表统一处理
    if isinstance(target_pose_list, dict):
        target_pose_list = [target_pose_list]

    def worker():
        print(f" Starting sequence transition ({len(target_pose_list)} stages)...")

        # 遍历每一个目标姿态
        for idx, target_pose in enumerate(target_pose_list):
            # 1. 准备当前阶段的目标数组
            target_arr = np.array(
                [
                    target_pose["FR_hip_joint"],
                    target_pose["FR_thigh_joint"],
                    target_pose["FR_calf_joint"],
                    target_pose["FL_hip_joint"],
                    target_pose["FL_thigh_joint"],
                    target_pose["FL_calf_joint"],
                    target_pose["RR_hip_joint"],
                    target_pose["RR_thigh_joint"],
                    target_pose["RR_calf_joint"],
                    target_pose["RL_hip_joint"],
                    target_pose["RL_thigh_joint"],
                    target_pose["RL_calf_joint"],
                    target_pose["FR_foot_joint"],
                    target_pose["FL_foot_joint"],
                    target_pose["RR_foot_joint"],
                    target_pose["RL_foot_joint"],
                ],
                dtype=np.double,
            )

            start_arr = default_angle.copy()
            steps = 100  # 增加步数使插值更细腻
            interval = duration / steps

            # 执行插值
            for i in range(steps + 1):
                alpha = i / steps
                current_vals = (1 - alpha) * start_arr + alpha * target_arr
                default_angle[:] = current_vals[:]
                time.sleep(interval)

            # 阶段完成，打印日志并停顿
            print(f"  -> Stage {idx + 1}/{len(target_pose_list)} reached.")
            if pause > 0:
                time.sleep(pause)

        print("✅ Transition sequence complete.")

    t = threading.Thread(target=worker, daemon=True)
    t.start()


default_angle = np.array(
    [
        default_joint_angles["FR_hip_joint"],
        default_joint_angles["FR_thigh_joint"],
        default_joint_angles["FR_calf_joint"],
        default_joint_angles["FL_hip_joint"],
        default_joint_angles["FL_thigh_joint"],
        default_joint_angles["FL_calf_joint"],
        default_joint_angles["RR_hip_joint"],
        default_joint_angles["RR_thigh_joint"],
        default_joint_angles["RR_calf_joint"],
        default_joint_angles["RL_hip_joint"],
        default_joint_angles["RL_thigh_joint"],
        default_joint_angles["RL_calf_joint"],
        default_joint_angles["FR_foot_joint"],
        default_joint_angles["FL_foot_joint"],
        default_joint_angles["RR_foot_joint"],
        default_joint_angles["RL_foot_joint"],
    ],
    dtype=np.double,
)

HISTORY_LEN = 10  # Must match training configuration
GAIT_CYCLE_TIME = 0.5
GAIT_COMMAND_THRESHOLD = 0.10


class ObservationHistory:
    """Maintain a FIFO buffer of observations for HIM-style policies."""

    def __init__(self, history_len: int, frame_dim: int):
        self.history_len = history_len
        self.frame_dim = frame_dim
        self.buffer = np.zeros((history_len, frame_dim), dtype=np.float32)
        self.count = 0

    def push(self, obs: np.ndarray):
        if self.count < self.history_len:
            self.buffer[self.count] = obs
            self.count += 1
        else:
            self.buffer[:-1] = self.buffer[1:]
            self.buffer[-1] = obs

    def get_tensor(self) -> torch.Tensor:
        if self.count == 0:
            padded = self.buffer.copy()
        elif self.count < self.history_len:
            padded = self.buffer.copy()
            last = padded[self.count - 1]
            for idx in range(self.count, self.history_len):
                padded[idx] = last
        else:
            padded = self.buffer
        flat = padded.reshape(1, -1)
        return torch.from_numpy(flat).to(dtype=torch.float32)


class Cmd:
    def __init__(self):
        # Velocity command range
        self.vx = 2.0  # [-4.5, 4.5] m/s
        self.vy = 1.0  # [-1.5, 1.5] m/s
        self.dyaw = 0.0  # [-1.0, 1.0] rad/s

        self.vx_step = 0.1
        self.vy_step = 0.1
        self.vyaw_step = 0.1
        self.vx_max = 2.0
        self.vy_max = 1.0
        self.vyaw_max = 1.0

        self.target_vx = 1.0  # Base speed for W/S
        self.target_vy = 0.5  # Base speed for A/D
        self.target_dyaw = 0.5  # Base speed for Q/E

    def set_vx(self, val):
        self.vx = val

    def set_vy(self, val):
        self.vy = val

    def set_dyaw(self, val):
        self.dyaw = val

    def speed_up(self):
        self.target_vx = min(self.target_vx + 0.1, 3.0)
        self.target_vy = min(self.target_vy + 0.1, 1.5)
        self.target_dyaw = min(self.target_dyaw + 0.1, 2.0)

    def speed_down(self):
        self.target_vx = max(self.target_vx - 0.1, 0.1)
        self.target_vy = max(self.target_vy - 0.1, 0.1)
        self.target_dyaw = max(self.target_dyaw - 0.1, 0.1)

    def stop_all(self):
        self.vx = 0.0
        self.vy = 0.0
        self.dyaw = 0.0

    def stop_vx(self):
        self.vx = 0.0

    def stop_vy(self):
        self.vy = 0.0

    def stop_vyaw(self):
        self.dyaw = 0.0


vel_cmd = Cmd()


def start_keyboard_listener():
    def on_press(key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = None

        if k == "w":
            vel_cmd.set_vx(vel_cmd.target_vx)
            print(f"⬆ Forward: vx={vel_cmd.vx:.2f}")
        elif k == "s":
            vel_cmd.set_vx(-vel_cmd.target_vx)
            print(f"⬇ Backward: vx={vel_cmd.vx:.2f}")
        elif k == "a":
            vel_cmd.set_vy(vel_cmd.target_vy)
            print(f"⬅ Left: vy={vel_cmd.vy:.2f}")
        elif k == "d":
            vel_cmd.set_vy(-vel_cmd.target_vy)
            print(f"➡ Right: vy={vel_cmd.vy:.2f}")
        elif k == "q":
            vel_cmd.set_dyaw(vel_cmd.target_dyaw)
            print(f"↺ CCW: vyaw={vel_cmd.dyaw:.2f}")
        elif k == "e":
            vel_cmd.set_dyaw(-vel_cmd.target_dyaw)
            print(f"↻ CW: vyaw={vel_cmd.dyaw:.2f}")

        # Speed adjustment (Gear Shift)
        elif k == "u":
            vel_cmd.speed_up()
            print(f" Speed UP: Vx={vel_cmd.target_vx:.1f}, Vy={vel_cmd.target_vy:.1f}, Yaw={vel_cmd.target_dyaw:.1f}")
        elif k == "j":
            vel_cmd.speed_down()
            print(f" Speed DOWN: Vx={vel_cmd.target_vx:.1f}, Vy={vel_cmd.target_vy:.1f}, Yaw={vel_cmd.target_dyaw:.1f}")

        # Pose Toggle
        elif k == "p":
            global current_pose_name
            if current_pose_name == "A":
                print("Switching to Pose B (via Mid)...")
                # 路径: A -> Mid -> B
                transition_pose([POSE_MID_ANGLES, POSE_B_ANGLES], duration=0.5, pause=0.1)
                current_pose_name = "B"
            else:
                print("Switching to Pose A (via Mid)...")
                # 路径: B -> Mid -> A
                # 关键点：pause=1.0 给足时间稳定，防止下蹲过快摔倒
                transition_pose([POSE_MID_ANGLES, POSE_A_ANGLES], duration=0.5, pause=0.1)
                current_pose_name = "A"

        if key == pynput_keyboard.Key.space:
            vel_cmd.stop_all()
            print("⏹ Emergency stop")
        elif k == "x":
            vel_cmd.stop_vx()
        elif k == "c":
            vel_cmd.stop_vy()
        elif k == "v":
            vel_cmd.stop_vyaw()

    def on_release(key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = None

        # Stop specific axis when key is released
        if k in ["w", "s"]:
            vel_cmd.stop_vx()
        elif k in ["a", "d"]:
            vel_cmd.stop_vy()
        elif k in ["q", "e"]:
            vel_cmd.stop_vyaw()

    listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.daemon = True
    listener.start()


def get_obs(data, vel_cmd, last_action, qpos_ids, qvel_ids, debug=False):
    q = data.qpos[qpos_ids].astype(np.double) - default_angle
    q += np.random.uniform(-0.01, 0.01, q.shape)

    dq = data.qvel[qvel_ids].astype(np.double) * 0.05
    q[-4:] = 0.0  # zero wheel joints

    imu_quat = data.sensor("orientation").data[[1, 2, 3, 0]].astype(np.double)
    r_imu = R.from_quat(imu_quat)
    proj = r_imu.apply(np.array([0.0, 0.0, -1.0]), inverse=True).astype(np.double)

    gyro_local = data.sensor("angular-velocity").data.astype(np.double)
    base_quat = data.qpos[3:7][[1, 2, 3, 0]].astype(np.double)
    r_base = R.from_quat(base_quat)

    gyro_world = r_imu.apply(gyro_local)
    gyro = r_base.apply(gyro_world, inverse=True) * 0.25

    obs = np.concatenate(
        [
            gyro,
            proj,
            np.array([vel_cmd.vx, vel_cmd.vy, vel_cmd.dyaw]),
            q,
            dq,
            last_action,
        ]
    ).astype(np.float32)

    if debug:
        print(f"gyro: {gyro}, proj: {proj}, q: {q}, dq: {dq}")

    return obs


class PDController:
    def __init__(self, kp, kd, tau_limit):
        self.kp = kp
        self.kd = kd
        self.tau_limit = tau_limit

    def compute(self, target_q, q, target_dq, dq):
        tau = self.kp * (target_q - q) + self.kd * (target_dq - dq)
        return np.clip(tau, -self.tau_limit, self.tau_limit)


def scale_action(raw_action, cfg):
    scaled = np.zeros_like(raw_action)
    for i in range(4):
        base = i * 3
        scaled[base + 0] = raw_action[base + 0] * cfg.robot_config.hip_scale
        scaled[base + 1] = raw_action[base + 1] * cfg.robot_config.thigh_calf_scale
        scaled[base + 2] = raw_action[base + 2] * cfg.robot_config.thigh_calf_scale
    # 轮子速度（XML文件已修复，不需要反转）
    scaled[12:] = raw_action[12:] * cfg.robot_config.wheel_scale
    return scaled


def plot_joint_data(plot_data):
    """Plot two figures: 1) Target vs Actual, 2) Error vs Torque"""
    joint_names = [
        "FR_hip",
        "FR_thigh",
        "FR_calf",
        "FL_hip",
        "FL_thigh",
        "FL_calf",
        "RR_hip",
        "RR_thigh",
        "RR_calf",
        "RL_hip",
        "RL_thigh",
        "RL_calf",
        "FR_wheel",
        "FL_wheel",
        "RR_wheel",
        "RL_wheel",
    ]

    time = np.array(plot_data["time"])

    # Figure 1: Target vs Actual
    fig1, axes1 = plt.subplots(4, 4, figsize=(16, 12))
    fig1.suptitle("Target vs Actual (Position/Velocity)", fontsize=16, fontweight="bold")

    for i in range(16):
        row = i // 4
        col = i % 4
        ax = axes1[row, col]

        targets = np.array(plot_data["targets"][i])
        actuals = np.array(plot_data["actuals"][i])

        # Plot target (blue dashed) and actual (green solid)
        ax.plot(time, targets, "b--", linewidth=1.5, label="Target", alpha=0.8)
        ax.plot(time, actuals, "g-", linewidth=1.5, label="Actual", alpha=0.8)

        if i < 12:
            ax.set_ylabel("Position [rad]", fontsize=8)
        else:
            ax.set_ylabel("Velocity [rad/s]", fontsize=8)

        ax.tick_params(axis="y", labelsize=7)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)
        ax.set_title(joint_names[i], fontsize=10, fontweight="bold")
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.tick_params(axis="x", labelsize=7)

    plt.tight_layout()
    fig1.savefig("joint_tracking.png", dpi=150, bbox_inches="tight")
    print("✓ Figure 1 saved: joint_tracking.png")

    # Figure 2: Error vs Torque
    fig2, axes2 = plt.subplots(4, 4, figsize=(16, 12))
    fig2.suptitle("Error and Torque Output", fontsize=16, fontweight="bold")

    for i in range(16):
        row = i // 4
        col = i % 4
        ax = axes2[row, col]

        errors = np.array(plot_data["errors"][i])
        torques = np.array(plot_data["torques"][i])

        # Plot error (blue)
        ax.plot(time, errors, "b-", linewidth=1.5, label="Error", alpha=0.8)
        if i < 12:
            ax.set_ylabel("Error [rad]", color="b", fontsize=8)
        else:
            ax.set_ylabel("Error [rad/s]", color="b", fontsize=8)
        ax.tick_params(axis="y", labelcolor="b", labelsize=7)
        ax.grid(True, alpha=0.3)

        # Plot torque on secondary axis (red)
        ax2 = ax.twinx()
        ax2.plot(time, torques, "r-", linewidth=1.5, label="Torque", alpha=0.8)
        ax2.set_ylabel("Torque [Nm]", color="r", fontsize=8)
        ax2.tick_params(axis="y", labelcolor="r", labelsize=7)

        ax.set_title(joint_names[i], fontsize=10, fontweight="bold")
        ax.set_xlabel("Time [s]", fontsize=8)
        ax.tick_params(axis="x", labelsize=7)

    plt.tight_layout()
    fig2.savefig("joint_error_torque.png", dpi=150, bbox_inches="tight")
    print("✓ Figure 2 saved: joint_error_torque.png")

    plt.show()


def run_mujoco(
    policies,
    mujoco_model_path,
    sim_duration,
    dt,
    decimation,
    debug=False,
    plot=False,
    keyboard_control=False,
):
    model = mujoco.MjModel.from_xml_path(mujoco_model_path)
    model.opt.timestep = dt
    qpos_ids, qvel_ids, actuator_ids = resolve_joint_addresses(model)
    velocity_limits = resolve_hardware_velocity_limits(model)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)
    viewer = mujoco.viewer.launch_passive(model, data)

    if keyboard_control:
        print("\n⌨️ Keyboard control active (W/S/A/D, Q/E, space/X/C/V)\n")

    # Set initial state
    data.qpos[:3] = [0, 0, cfg.robot_config.init_height]
    data.qpos[3:7] = [1, 0, 0, 0]  # quaternion [w, x, y, z]
    data.qpos[qpos_ids] = default_angle.copy()
    data.qvel[:] = 0.0

    target_q = default_angle.copy()
    action = np.zeros(16, dtype=np.float32)
    last_action = np.zeros(16, dtype=np.float32)

    # Vectorized PD control parameters
    kp_array = cfg.robot_config.kp_array
    kd_array = cfg.robot_config.kd_array
    tau_limit_array = cfg.robot_config.tau_limit_array

    # Data recording for plotting
    if plot:
        plot_data = {
            "time": [],
            "targets": [[] for _ in range(16)],
            "actuals": [[] for _ in range(16)],
            "errors": [[] for _ in range(16)],
            "torques": [[] for _ in range(16)],
        }

    steps = int(sim_duration / dt)
    render_every = max(decimation, int(0.033 / dt))  # ~30 fps
    try:
        for step in range(steps):
            if step % decimation == 0:
                obs = get_obs(data, vel_cmd, last_action, qpos_ids, qvel_ids, debug=debug)

                # Dynamic Policy Switching
                # Select policy based on current pose state
                current_policy = policies.get(current_pose_name, policies["A"])  # Default to A if not found

                with torch.inference_mode():
                    obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(dtype=torch.float32)
                    # print("obs_tensor: ", obs_tensor)
                    raw_action = current_policy(obs_tensor).cpu().numpy().squeeze()
                action[:] = raw_action
                # print("action: ", action)
                # print("================================================")
                if step > 100:
                    scaled_action = scale_action(action, cfg)
                    target_q = scaled_action + default_angle
                    target_q[12:] = np.clip(target_q[12:], -velocity_limits[12:], velocity_limits[12:])
                else:
                    target_q = default_angle
                last_action = action.copy()

            q = data.qpos[qpos_ids]
            dq = data.qvel[qvel_ids]

            tau = np.zeros(16)

            # Leg joints (0-11): position control
            tau[:12] = kp_array[:12] * (target_q[:12] - q[:12]) + kd_array[:12] * (0 - dq[:12])

            # Wheel joints (12-15): velocity control (target_q stores target velocity for wheels)
            tau[12:] = kd_array[12:] * (target_q[12:] - dq[12:])

            tau = np.clip(tau, -tau_limit_array, tau_limit_array)
            tau = attenuate_overspeed_torque(tau, dq, velocity_limits)

            # Record data for plotting (every decimation steps)
            if plot and step % decimation == 0:
                plot_data["time"].append(step * dt)
                # For leg joints (0-11): position control
                for i in range(12):
                    plot_data["targets"][i].append(target_q[i])
                    plot_data["actuals"][i].append(q[i])
                    plot_data["errors"][i].append(target_q[i] - q[i])
                    plot_data["torques"][i].append(tau[i])
                # For wheel joints (12-15): velocity control
                for i in range(12, 16):
                    plot_data["targets"][i].append(target_q[i])
                    plot_data["actuals"][i].append(dq[i])
                    plot_data["errors"][i].append(target_q[i] - dq[i])
                    plot_data["torques"][i].append(tau[i])

            # Debug mode disabled when plotting (too much output)
            # if debug and step % (decimation * 10) == 0:
            #     pass

            # Apply computed torques to actuators
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids] = tau
            mujoco.mj_step(model, data)

            if step % render_every == 0:
                viewer.sync()
            if not viewer.is_running():
                break

    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    finally:
        pass  # native viewer closes via context manager

        # Plot results
        if plot and len(plot_data["time"]) > 0:
            plot_joint_data(plot_data)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Mujoco deployment")
    parser.add_argument(
        "--model-path",
        type=str,
        default=str(DEFAULT_MODEL_PATH),
    )
    parser.add_argument(
        "--policy-path",
        type=str,
        default=str(DEFAULT_POLICY_PATH),
        help="Path to Policy A (Default)",
    )
    parser.add_argument(
        "--policy-path-b",
        type=str,
        default=str(DEFAULT_POLICY_PATH),
        help="Path to Policy B (High Stand)",
    )  # 默认暂时用A，请修改为实际B路径
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--decimation", type=int, default=10)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--vx", type=float, default=0.0, help="Forward velocity command [-4.5, 4.5] m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral velocity command [-1.5, 1.5] m/s")
    parser.add_argument("--vyaw", type=float, default=0.0, help="Yaw velocity command [-1.0, 1.0] rad/s")
    parser.add_argument("--plot", action="store_true", help="Generate plots of joint errors and torques")
    parser.add_argument("--keyboard", action="store_true", help="Enable keyboard control (WASD/QE)")

    args = parser.parse_args()

    cfg.sim_config.dt = args.dt
    cfg.sim_config.decimation = args.decimation

    vel_cmd.vx = args.vx
    vel_cmd.vy = args.vy
    vel_cmd.dyaw = args.vyaw

    # Print PD control parameters
    cfg.robot_config.print_parameters()

    print(f"Loading Policy A from: {args.policy_path}")
    policy_a = torch.jit.load(args.policy_path)

    print(f"Loading Policy B from: {args.policy_path_b}")
    try:
        policy_b = torch.jit.load(args.policy_path_b)
    except Exception as e:
        print(f"⚠️ Warning: Failed to load Policy B ({e}). Using Policy A as fallback.")
        policy_b = policy_a

    policies = {"A": policy_a, "B": policy_b}

    print(f"Velocity command: vx={vel_cmd.vx:.2f}, vy={vel_cmd.vy:.2f}, vyaw={vel_cmd.dyaw:.2f}")
    if args.keyboard:
        start_keyboard_listener()
        print("Keyboard control enabled: W/S (vx), A/D (vy), Q/E (yaw), space to stop, P to toggle Pose/Policy")
    if args.plot:
        print("Plot mode enabled: will generate joint_analysis.png after simulation\n")

    run_mujoco(
        policies,
        args.model_path,
        args.duration,
        args.dt,
        args.decimation,
        debug=args.debug,
        plot=args.plot,
        keyboard_control=args.keyboard,
    )
