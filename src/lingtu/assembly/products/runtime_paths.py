"""Runtime path defaults shared by Product assembly and local tools."""

from __future__ import annotations

from runtime.runtime_interface import map_frame_id

DEFAULT_GATEWAY_PORT = 5050


DEFAULT_PLANNING_FRAME_ID = map_frame_id()
