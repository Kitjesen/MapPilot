"""Legacy stub driver module kept for backward compatibility.

DEPRECATED: Use StubDogModule from runtime/blueprints/stub.py instead.
StubDogModule is the canonical stub driver registered as (``driver``, ``stub``)
in the registry, with periodic odom publishing and health reporting.

This is a stub connection -- no hardware, dead-reckoning odometry for CI/testing.
Same port interface as ThunderDriver so it drops into any blueprint.

TODO: Remove once all consumers are migrated.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

from runtime.module import Module

if TYPE_CHECKING:
    from runtime.blueprint import Blueprint
from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out

STUB_LEGACY_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.map_cloud)
STUB_BODY_FRAME_ID = topic_default_frame_id(TOPICS.cmd_vel)


class StubConnection(Module, layer=1):
    """Fake robot driver for testing. Integrates cmd_vel into odometry."""

    cmd_vel: In[Twist]
    stop_signal: In[int]
    slam_odom: In[Odometry]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(
        self, dt: float = 0.02, initial_x: float = 0.0, initial_y: float = 0.0, initial_yaw: float = 0.0, **kw
    ):
        super().__init__(**kw)
        self._dt = dt
        self._x = initial_x
        self._y = initial_y
        self._yaw = initial_yaw
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self):
        super().start()
        self.alive.publish(True)
        self._publish_odom()  # bootstrap: let consumers know initial position

    def stop(self):
        self.alive.publish(False)
        super().stop()

    def _on_cmd(self, twist: Twist):
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._integrate()
        self._publish_odom()

    def _on_stop(self, level: int):
        if level >= 1:
            self._vx = self._vy = self._wz = 0.0

    def _integrate(self):
        self._yaw += self._wz * self._dt
        self._x += (self._vx * math.cos(self._yaw) - self._vy * math.sin(self._yaw)) * self._dt
        self._y += (self._vx * math.sin(self._yaw) + self._vy * math.cos(self._yaw)) * self._dt

    def _publish_odom(self):
        q = Quaternion.from_yaw(self._yaw)
        self.odometry.publish(
            Odometry(
                pose=Pose(Vector3(self._x, self._y, 0.0), q),
                twist=Twist(Vector3(self._vx, self._vy, 0.0), Vector3(0.0, 0.0, self._wz)),
                ts=time.time(),
                frame_id=STUB_LEGACY_ODOM_FRAME_ID,
                child_frame_id=STUB_BODY_FRAME_ID,
            )
        )


def stub_blueprint(**config) -> Blueprint:
    """Test blueprint -- StubConnection + new module architecture.

    NOTE: Blueprint factory -- cross-layer module resolution uses stack_module()
    to avoid eager cross-package imports at module load time. Navigation
    and SafetyRing (both from nav/) are resolved lazily via the registry
    helper when this factory function is called.
    """
    from runtime.blueprint import Blueprint
    from runtime.blueprints.stacks._registry import stack_module

    Navigation = stack_module(
        "navigation",
        "default",
        seed_group="navigation",
        fallback="nav.navigation.Navigation",
    )
    SafetyRing = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.services.safety.safety_ring.SafetyRing",
    )

    bp = Blueprint()
    bp.add(StubConnection)
    bp.add(Navigation, planner=config.get("planner_backend", "octoplanner3d"))
    bp.add(SafetyRing)
    bp.wire("nav.safety", "stop_cmd", "StubConnection", "stop_signal")
    bp.auto_wire()
    return bp
