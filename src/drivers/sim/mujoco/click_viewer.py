"""A MuJoCo GLFW window whose input and presentation share one owner thread."""

from __future__ import annotations

import copy
import threading
from collections import deque
from contextlib import nullcontext
from typing import Any

import glfw
import mujoco

from drivers.sim.mujoco.goal_picker import ClickGesture


class ClickViewer:
    """Resolve clicks before replacing the scene that was actually drawn."""

    def __init__(self, model: Any, data: Any, picker: Any, *, visible: bool = True) -> None:
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.user_scn = mujoco.MjvScene(model, maxgeom=1200)
        self.viewport = mujoco.MjrRect(0, 0, 1, 1)
        self._model, self._data, self._picker = model, data, picker
        self._scene = mujoco.MjvScene(model, maxgeom=model.ngeom + 1200)
        self._displayed_data = mujoco.MjData(model)
        self._displayed_opt = copy.copy(self.opt)
        self._displayed_size = (0, 0)
        self._gesture = ClickGesture()
        self._down = False
        self._cursor = (0.0, 0.0)
        self._clicks: deque[Any] = deque(maxlen=1)
        self._texts = None
        self._closed = threading.Event()
        self._window = None
        self._context = None
        if not glfw.init():
            raise RuntimeError("MuJoCo window initialization failed")
        try:
            glfw.window_hint(glfw.VISIBLE, glfw.TRUE if visible else glfw.FALSE)
            self._window = glfw.create_window(1200, 800, "MuJoCo: click navigation", None, None)
            if not self._window:
                raise RuntimeError("MuJoCo window creation failed")
            glfw.make_context_current(self._window)
            glfw.swap_interval(1)
            self._context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
            glfw.set_mouse_button_callback(self._window, self._button)
            glfw.set_cursor_pos_callback(self._window, self._motion)
            glfw.set_scroll_callback(self._window, self._scroll)
            glfw.set_window_focus_callback(self._window, self._focus)
        except Exception:
            self.dispose()
            raise

    def lock(self):
        return nullcontext()  # All scene, camera and input work runs on this thread.

    def is_running(self) -> bool:
        return not self._closed.is_set()

    def close(self) -> None:
        self._closed.set()

    def dispose(self) -> None:
        self.close()
        if self._context is not None:
            self._context.free()
        if self._window is not None:
            glfw.destroy_window(self._window)

    def set_texts(self, texts: Any) -> None:
        self._texts = texts

    def poll_click(self) -> Any:
        try:
            result = self._clicks.popleft()
        except IndexError:
            return None
        if isinstance(result, ValueError):
            raise result
        return result

    def _focus(self, window: Any, focused: bool) -> None:
        if not focused:
            self._down = False
            self._gesture.update(False, False, *self._cursor)

    def _button(self, window: Any, button: int, action: int, mods: int) -> None:
        if button != glfw.MOUSE_BUTTON_LEFT:
            return
        x, y = glfw.get_cursor_pos(window)
        self._cursor = (x, y)
        focused = bool(glfw.get_window_attrib(window, glfw.FOCUSED)) and not mods
        if action == glfw.PRESS:
            self._gesture.update(focused, False, x, y)
        self._down = action == glfw.PRESS
        if not self._gesture.update(focused, self._down, x, y):
            return
        width, height = self._displayed_size
        # A resize invalidates pixel coordinates until the resized frame is drawn.
        if (width, height) != glfw.get_window_size(window) or width <= 0 or height <= 0:
            self._clicks.append(ValueError("Waiting for the resized view; click again"))
            return
        if not (0 <= x < width and 0 <= y < height):
            return
        try:
            self._clicks.append(self._picker.pick_displayed(
                self._model, self._displayed_data, self._displayed_opt, self._scene,
                self.viewport.width / self.viewport.height, x / width, 1 - y / height,
            ))
        except ValueError as exc:
            self._clicks.append(exc)

    def _motion(self, window: Any, x: float, y: float) -> None:
        previous = self._cursor
        self._cursor = (x, y)
        if not self._down:
            return
        focused = bool(glfw.get_window_attrib(window, glfw.FOCUSED))
        self._gesture.update(focused, True, x, y)
        height = max(1, glfw.get_window_size(window)[1])
        mujoco.mjv_moveCamera(self._model, mujoco.mjtMouse.mjMOUSE_ROTATE_V,
                             (x - previous[0]) / height, (y - previous[1]) / height,
                             self._scene, self.cam)

    def _scroll(self, window: Any, x: float, y: float) -> None:
        mujoco.mjv_moveCamera(self._model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                             0, -0.05 * y, self._scene, self.cam)

    def sync(self) -> None:
        # Callbacks consume the previous displayed scene before any replacement.
        glfw.poll_events()
        if glfw.window_should_close(self._window):
            self.close()
            return
        width, height = glfw.get_framebuffer_size(self._window)
        if width <= 0 or height <= 0:
            return
        self.viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mj_copyData(self._displayed_data, self._model, self._data)
        self._displayed_opt = copy.copy(self.opt)
        mujoco.mjv_updateScene(self._model, self._displayed_data, self._displayed_opt,
                             None, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self._scene)
        for index in range(self.user_scn.ngeom):
            source = self.user_scn.geoms[index]
            target = self._scene.geoms[self._scene.ngeom]
            mujoco.mjv_initGeom(target, source.type, source.size, source.pos,
                               source.mat.reshape(-1), source.rgba)
            self._scene.ngeom += 1
        mujoco.mjr_render(self.viewport, self._scene, self._context)
        if self._texts is not None:
            font, grid, left, right = self._texts
            mujoco.mjr_overlay(font, grid, self.viewport, left, right, self._context)
        glfw.swap_buffers(self._window)
        self._displayed_size = glfw.get_window_size(self._window)
