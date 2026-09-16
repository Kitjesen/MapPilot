"""Mouse gestures and picking against a displayed MuJoCo scene."""

from __future__ import annotations

from typing import Any

import numpy as np


class ClickGesture:
    """Emit a release only after a focused, stationary left-button press."""

    def __init__(self) -> None:
        self._down: tuple[float, float] | None = None
        self._dragged = False
        self._armed = False

    def update(self, focused: bool, pressed: bool, x: float, y: float) -> bool:
        if not focused:
            self._down = None
            self._armed = False
            return False
        if not self._armed:
            self._armed = not pressed
            return False
        if pressed and self._down is None:
            self._down = (x, y)
            self._dragged = False
        if self._down is None:
            return False
        if np.hypot(x - self._down[0], y - self._down[1]) >= 5:
            self._dragged = True
        if pressed:
            return False
        self._down = None
        return not self._dragged


class GoalPicker:
    """Raycast displayed geometry and add a selected-point marker."""

    def __init__(self, model: Any, body_name: str) -> None:
        import mujoco

        self._scene = mujoco.MjvScene(model, maxgeom=max(1000, model.ngeom * 2))
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self.body_id <= 0:
            raise ValueError(f"MuJoCo goal picking requires robot body {body_name!r}")
        self.marker: np.ndarray | None = None

    def pick(self, model: Any, data: Any, viewer: Any, x: float, y: float) -> np.ndarray:
        import mujoco

        mujoco.mjv_updateScene(
            model, data, viewer.opt, None, viewer.cam,
            mujoco.mjtCatBit.mjCAT_ALL, self._scene,
        )
        return self.pick_displayed(model, data, viewer.opt, self._scene,
                                   viewer.viewport.width / viewer.viewport.height, x, y)

    def pick_displayed(self, model: Any, data: Any, opt: Any, scene: Any,
                       aspect: float, x: float, y: float) -> np.ndarray:
        """Raycast the immutable state and camera used for the displayed frame."""
        import mujoco

        point = np.zeros(3)
        geom, flex, skin = (np.array([-1], dtype=np.int32) for _ in range(3))
        body = mujoco.mjv_select(
            model, data, opt, aspect, x, y, scene, point, geom, flex, skin,
        )
        if body < 0:
            raise ValueError("No surface selected; click the ground")
        cursor = body
        while cursor > 0:
            if cursor == self.body_id:
                raise ValueError("Robot selected; click the ground")
            cursor = int(model.body_parentid[cursor])
        return point

    def draw(self, viewer: Any) -> None:
        import mujoco

        scene = viewer.user_scn
        if self.marker is None or scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        mujoco.mjv_initGeom(
            geom, mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.10, 0.10, 0.10]), self.marker + np.array([0, 0, 0.08]),
            np.eye(3).reshape(-1), np.array([1.0, 0.65, 0.0, 0.9], dtype=np.float32),
        )
