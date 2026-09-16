"""Simulation mounts and RobotConfig-backed physical LiDAR extrinsics."""

from pathlib import Path

from runtime.config import load_config
from runtime.tf.frames import FRAMES, Transform3D
from runtime.yaml_helpers import load_yaml

_SIM_LIDAR_MOUNTS = {
    "gazebo_proxy": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        x=0.28,
        y=0.0,
        z=0.20,
    ),
    "mujoco_thunder_v3": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        # Compiled thunderv4.xml lidar_site pose in the base_link frame.
        # Keep this synchronized with the compiled-site acceptance test rather
        # than copying a nested MJCF local position by inspection.
        x=-0.30638,
        y=0.0,
        z=0.19417,
    ),
    "portable_mid360_like": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        x=0.0,
        y=0.0,
        z=0.20,
    ),
}


def lidar_extrinsics() -> dict[str, Transform3D]:
    """Project simulation mounts and physical RobotConfig calibrations."""
    mounts = dict(_SIM_LIDAR_MOUNTS)
    robot_root = Path(__file__).resolve().parents[3] / "config" / "robots"
    for model_path in sorted(robot_root.glob("*/*/model.yaml")):
        model = load_yaml(model_path)
        profile = model.get("sensors", {}).get("mid360", {}).get("extrinsic_profile")
        if not profile:
            continue
        config_path = model_path.with_name("robot.yaml")
        if not config_path.is_file():
            raise FileNotFoundError(f"Robot calibration not found: {config_path}")
        lidar = load_config(str(config_path)).lidar
        mounts[profile] = Transform3D(
            parent=FRAMES.body,
            child=FRAMES.real_lidar,
            x=lidar.offset_x,
            y=lidar.offset_y,
            z=lidar.offset_z,
            roll=lidar.roll,
            pitch=lidar.pitch,
            yaw=lidar.yaw,
        )
    return mounts


def lidar_extrinsic(profile: str) -> Transform3D:
    """Resolve a simulation mount or a physical robot's declared calibration."""
    if profile in _SIM_LIDAR_MOUNTS:
        return _SIM_LIDAR_MOUNTS[profile]
    try:
        return lidar_extrinsics()[profile]
    except KeyError as exc:
        raise ValueError(f"unknown LiDAR extrinsic profile {profile!r}") from exc
