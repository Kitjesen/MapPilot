# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

MODEL = (
    Path(__file__).resolve().parents[1]
    / "packages"
    / "robots"
    / "thunderv4"
    / "mjcf"
    / "thunderv4.xml"
)


def test_front_camera_uses_the_real_mount_with_a_clear_forward_view() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    camera = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
    mount = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera1_link")
    assert np.allclose(model.cam_pos[camera], model.body_pos[mount])
    assert model.cam_fovy[camera] == 60.0

    forward = -data.cam_xmat[camera].reshape(3, 3)[:, 2]
    assert np.allclose(forward, (1.0, 0.0, 0.0))

    hit = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(model, data, data.cam_xpos[camera], forward, None, True, -1, hit)
    assert distance < 0, f"front camera center ray hits robot geom {hit[0]} at {distance:.3f} m"

    axes = data.cam_xmat[camera].reshape(3, 3)
    half_y = np.tan(np.deg2rad(model.cam_fovy[camera] / 2))
    half_x = half_y * 640 / 480
    for y in np.linspace(-half_y, half_y, 7):
        for x in np.linspace(-half_x, half_x, 9):
            direction = forward + x * axes[:, 0] + y * axes[:, 1]
            direction /= np.linalg.norm(direction)
            mujoco.mj_ray(model, data, data.cam_xpos[camera], direction, None, True, -1, hit)
            assert hit[0] < 0 or model.geom_bodyid[hit[0]] == 0, f"front camera FOV ray hits robot geom {hit[0]}"
