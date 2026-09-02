"""Verify that a MuJoCo scene uses the real Thunder V4 small-wheel asset.

This is intentionally independent of policy playback.  It validates the V4
asset contract and settles a known-valid standing pose under the hardware torque
limits, so a bad legacy checkpoint cannot be mistaken for a bad robot model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ASSET_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ASSET_DIR / "mjcf" / "thunderv4.xml"

ACTUATOR_ORDER = (
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

EXPECTED_TOTAL_MASS = 45.8086
LEG_TORQUE_LIMIT = 120.0
WHEEL_TORQUE_LIMIT = 17.0
WHEEL_RADIUS = 0.093
LEG_VELOCITY_LIMIT = 17.48
WHEEL_VELOCITY_LIMIT = 44.0
HARDWARE_VELOCITY_NUMERIC = "hardware_joint_velocity_limit_rad_s"
REQUIRED_LIDAR_SENSORS = {
    "lidar-orientation",
    "lidar-position",
    "lidar-angular-velocity",
    "lidar-linear-velocity",
    "lidar-linear-acceleration",
}
EXPECTED_VELOCITY_LIMITS = np.array(
    [LEG_VELOCITY_LIMIT] * 12 + [WHEEL_VELOCITY_LIMIT] * 4,
    dtype=np.float64,
)


def _name_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, obj_type, name)
    if object_id < 0:
        raise RuntimeError(f"Missing {obj_type.name} named {name}")
    return int(object_id)


def control_layout(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return qpos, qvel, and actuator addresses in policy action order."""
    actuator_names = tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) for index in range(model.nu))
    if actuator_names != ACTUATOR_ORDER:
        raise RuntimeError(f"Action order mismatch. Expected {ACTUATOR_ORDER}, received {actuator_names}.")

    joint_ids = np.asarray(
        [_name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ACTUATOR_ORDER],
        dtype=np.int32,
    )
    actuator_ids = np.asarray(
        [_name_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ACTUATOR_ORDER],
        dtype=np.int32,
    )
    return (
        model.jnt_qposadr[joint_ids].astype(np.int32),
        model.jnt_dofadr[joint_ids].astype(np.int32),
        actuator_ids,
    )


def validate_asset_contract(model: mujoco.MjModel) -> dict[str, float | int]:
    """Raise when the compiled model differs from the real V4 contract."""
    qpos_ids, _, actuator_ids = control_layout(model)
    total_mass = float(model.body_mass.sum())
    if not np.isclose(total_mass, EXPECTED_TOTAL_MASS, atol=1e-3):
        raise RuntimeError(f"Unexpected total mass {total_mass:.6f} kg; expected V4 {EXPECTED_TOTAL_MASS:.4f} kg")
    if model.nq != 23 or model.nv != 22:
        raise RuntimeError(f"Unexpected floating-base state shape nq={model.nq}, nv={model.nv}")

    expected_limits = np.array(
        [[-LEG_TORQUE_LIMIT, LEG_TORQUE_LIMIT]] * 12 + [[-WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT]] * 4,
        dtype=np.float64,
    )
    if not np.allclose(model.actuator_ctrlrange[actuator_ids], expected_limits):
        raise RuntimeError("Actuator effort limits do not match the V4 hardware contract")

    numeric_id = _name_id(model, mujoco.mjtObj.mjOBJ_NUMERIC, HARDWARE_VELOCITY_NUMERIC)
    numeric_start = model.numeric_adr[numeric_id]
    numeric_size = model.numeric_size[numeric_id]
    velocity_limits = model.numeric_data[numeric_start : numeric_start + numeric_size]
    if not np.allclose(velocity_limits, EXPECTED_VELOCITY_LIMITS):
        raise RuntimeError(f"Joint velocity limits do not match the V4 hardware contract: {velocity_limits.tolist()}")

    for wheel_name in ("FR_wheel", "FL_wheel", "RR_wheel", "RL_wheel"):
        wheel_id = _name_id(model, mujoco.mjtObj.mjOBJ_GEOM, wheel_name)
        if not np.isclose(model.geom_size[wheel_id, 0], WHEEL_RADIUS, atol=1e-6):
            raise RuntimeError(f"{wheel_name} has non-V4 radius {model.geom_size[wheel_id, 0]}")

    sensor_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id) for sensor_id in range(model.nsensor)
    }
    missing_sensors = sorted(REQUIRED_LIDAR_SENSORS - sensor_names)
    if missing_sensors:
        raise RuntimeError(f"Missing LiDAR IMU sensors: {missing_sensors}")

    key_id = _name_id(model, mujoco.mjtObj.mjOBJ_KEY, "v4_nominal_stand")
    return {
        "total_mass_kg": total_mass,
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "stand_keyframe": key_id,
        "first_joint_qpos_address": int(qpos_ids[0]),
        "leg_velocity_limit_rad_s": LEG_VELOCITY_LIMIT,
        "wheel_velocity_limit_rad_s": WHEEL_VELOCITY_LIMIT,
    }


def run_stand_settle(model: mujoco.MjModel, duration_s: float) -> dict[str, float | int]:
    """Settle the V4 nominal stand under hardware torque limits."""
    qpos_ids, qvel_ids, actuator_ids = control_layout(model)
    data = mujoco.MjData(model)
    key_id = _name_id(model, mujoco.mjtObj.mjOBJ_KEY, "v4_nominal_stand")
    data.qpos[:] = model.key_qpos[key_id]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    kp = np.concatenate((np.full(12, 80.0), np.zeros(4)))
    kd = np.concatenate((np.full(12, 10.0), np.full(4, 1.0)))
    torque_limit = np.concatenate((np.full(12, LEG_TORQUE_LIMIT), np.full(4, WHEEL_TORQUE_LIMIT)))
    target_q = data.qpos[qpos_ids].copy()
    base_id = _name_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    steps = max(1, round(duration_s / model.opt.timestep))
    settle_start = int(steps * 0.8)
    leg_velocities: list[float] = []
    wheel_velocities: list[float] = []

    for step in range(steps):
        q = data.qpos[qpos_ids]
        dq = data.qvel[qvel_ids]
        torque = np.clip(kp * (target_q - q) - kd * dq, -torque_limit, torque_limit)
        data.ctrl[:] = 0.0
        data.ctrl[actuator_ids] = torque
        mujoco.mj_step(model, data)
        if step >= settle_start:
            leg_velocities.append(float(np.max(np.abs(dq[:12]))))
            wheel_velocities.append(float(np.max(np.abs(dq[12:]))))

    q = data.qpos[qpos_ids]
    joint_ids = np.asarray(
        [_name_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in ACTUATOR_ORDER[:12]],
        dtype=np.int32,
    )
    lower = model.jnt_range[joint_ids, 0]
    upper = model.jnt_range[joint_ids, 1]
    range_margin = np.minimum(q[:12] - lower, upper - q[:12])
    base_rotation = data.xmat[base_id].reshape(3, 3)
    return {
        "base_height_m": float(data.xpos[base_id, 2]),
        "uprightness": float(base_rotation[2, 2]),
        "last_20pct_leg_velocity_max_rad_s": max(leg_velocities),
        "last_20pct_wheel_velocity_max_rad_s": max(wheel_velocities),
        "minimum_leg_limit_margin_rad": float(np.min(range_margin)),
        "contacts": int(data.ncon),
    }


def main() -> None:
    """Run the asset contract and nominal-stand checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model))
    report = {
        "model": str(args.model),
        "contract": validate_asset_contract(model),
        "stand_settle": run_stand_settle(model, args.duration),
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
