"""Stub blueprint -- no hardware, fake odometry for CI / unit testing.

StubDogModule is a drop-in replacement for HanDogModule that publishes
synthetic odometry by integrating cmd_vel (simple dead reckoning).  No
gRPC, no hardware, no threads -- completely self-contained.

The blueprint factory stub_blueprint() composes StubDogModule with
Navigation and SafetyRing from nav/. Cross-package module
resolution uses stack_module() to avoid eager cross-package imports at
module load time.

Usage::

    from runtime.blueprints.stub import stub_blueprint

    system = stub_blueprint().build()
    system.start()
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from runtime.blueprint import Blueprint
from runtime.module import Module
from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.runtime_interface import (
    body_frame_id as runtime_body_frame_id,
)
from runtime.runtime_interface import (
    odom_frame_id as runtime_odom_frame_id,
)
from runtime.stream import In, Out

STUB_ODOM_FRAME_ID = runtime_odom_frame_id()
STUB_BODY_FRAME_ID = runtime_body_frame_id()


@register("driver", "stub", priority=0, description="Fake driver for CI/testing, no hardware")
@register("driver_protocol", "stub", priority=0, description="Stub driver for CI/testing, no hardware")
class StubDogModule(Module, layer=1):
    """Stub driver for CI/testing.  No hardware.  Publishes fake odometry.

    Also declares (but does not publish) the camera/depth/scene_graph Out
    ports that real drivers expose. Lets `full_stack_blueprint()` autowire
    downstream consumers (Perception, Reconstruction, Teleop, VisualServo,
    DepthVisualOdom) without errors. Consumers simply never see any frame
    when running against the stub 鈥?correct behaviour for CI.
    """

    # -- Port declarations (same interface as HanDogModule) --
    cmd_vel: In[Twist]
    stop_signal: In[int]
    odometry: Out[Odometry]
    alive: Out[bool]
    robot_state: Out[dict]
    # Camera / perception placeholders 鈥?declared for wire compatibility,
    # never published by the stub driver.
    camera_image: Out[Image]
    depth_image: Out[Image]
    scene_graph: Out[SceneGraph]

    def __init__(
        self,
        initial_x: float = 0.0,
        initial_y: float = 0.0,
        initial_yaw: float = 0.0,
        odom_frame_id: str = STUB_ODOM_FRAME_ID,
        child_frame_id: str = STUB_BODY_FRAME_ID,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._pos_x = initial_x
        self._pos_y = initial_y
        self._yaw = initial_yaw
        self._last_ts: float = 0.0
        self._stopped = False
        self._odom_hz = 50.0
        self._odom_frame_id = odom_frame_id
        self._child_frame_id = child_frame_id
        self._odom_thread: threading.Thread | None = None
        self._running = False

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self) -> None:
        super().start()
        self._last_ts = time.time()
        self._running = True
        self.alive.publish(True)
        self._publish_initial_odom()
        self._publish_robot_state()
        # Periodic odom loop (like a real robot)
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._odom_thread:
            self._odom_thread.join(timeout=1.0)
        self.alive.publish(False)
        self._publish_robot_state()
        super().stop()

    def _publish_initial_odom(self) -> None:
        now = time.time()
        self.odometry.publish(Odometry(
            pose=Pose(
                position=Vector3(self._pos_x, self._pos_y, 0.0),
                orientation=Quaternion(
                    0.0, 0.0,
                    math.sin(self._yaw / 2),
                    math.cos(self._yaw / 2),
                ),
            ),
            twist=Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
            ts=now, frame_id=self._odom_frame_id, child_frame_id=self._child_frame_id,
        ))

    # -- Port callbacks ------------------------------------------------

    def _on_cmd(self, twist: Twist) -> None:
        """Accept velocity command 鈥?integration happens in _odom_loop."""
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._stopped = False

    def _odom_loop(self) -> None:
        """Periodic dead-reckoning integration + odom publish (like a real robot)."""
        dt = 1.0 / self._odom_hz
        while self._running:
            now = time.time()
            cos_y = math.cos(self._yaw)
            sin_y = math.sin(self._yaw)
            self._pos_x += (self._vx * cos_y - self._vy * sin_y) * dt
            self._pos_y += (self._vx * sin_y + self._vy * cos_y) * dt
            self._yaw += self._wz * dt

            self.odometry.publish(Odometry(
                pose=Pose(
                    position=Vector3(self._pos_x, self._pos_y, 0.0),
                    orientation=Quaternion(
                        0.0, 0.0,
                        math.sin(self._yaw / 2),
                        math.cos(self._yaw / 2),
                    ),
                ),
                twist=Twist(
                    linear=Vector3(self._vx, self._vy, 0.0),
                    angular=Vector3(0.0, 0.0, self._wz),
                ),
                ts=now,
                frame_id=self._odom_frame_id,
                child_frame_id=self._child_frame_id,
            ))
            time.sleep(dt)

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
            self._stopped = True

    # -- Robot state --------------------------------------------------

    def _publish_robot_state(self) -> None:
        """Publish stub operational state."""
        self.robot_state.publish({
            "standing": True,
            "enabled": True,
            "emergency": False,
            "connected": True,
            "battery_voltage": 0.0,
            "battery_soc": 0.0,
            "current_gait": "none",
            "timestamp": time.time(),
        })

    # -- Health --------------------------------------------------------

    def health(self) -> dict[str, Any]:
        stats = super().port_summary()
        stats["stub"] = {"stopped": self._stopped}
        return stats


# -- Blueprint factory -------------------------------------------------

def stub_blueprint(**config: Any) -> Blueprint:
    """Test blueprint -- StubDogModule + new module architecture.

    NOTE: Blueprint factory -- cross-layer module resolution uses stack_module()
    to avoid eager cross-package imports at module load time. Navigation
    and SafetyRing (both from nav/) are resolved lazily via the registry
    helper when this factory function is called.
    """
    from runtime.blueprints.stacks._registry import stack_module

    Navigation = stack_module(
        "navigation",
        "default",
        seed_group="navigation",
        fallback="nav.mission.navigation.Navigation",
    )
    SafetyRing = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.services.safety.safety_ring.SafetyRing",
    )

    bp = Blueprint()
    bp.add(StubDogModule)
    bp.add(Navigation, planner=config.get("planner_backend", "direct"))
    bp.add(SafetyRing)

    bp.wire("nav.safety", "stop_cmd", "StubDogModule", "stop_signal")
    bp.auto_wire()
    return bp
