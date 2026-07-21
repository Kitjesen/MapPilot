"""No-hardware robot driver for tests and local simulation."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.runtime_interface import body_frame_id, odom_frame_id
from runtime.stream import In, Out

STUB_ODOM_FRAME_ID = odom_frame_id()
STUB_BODY_FRAME_ID = body_frame_id()


@register("driver", "stub", priority=0, description="Fake driver for CI/testing, no hardware")
@register("driver_protocol", "stub", priority=0, description="Stub driver for CI/testing, no hardware")
class StubDogModule(Module, layer=1):
    """Integrate velocity commands and publish deterministic fake odometry."""

    cmd_vel: In[Twist]
    stop_signal: In[int]
    odometry: Out[Odometry]
    alive: Out[bool]
    robot_state: Out[dict]
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
        self._last_ts = 0.0
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
        self._publish_odom(time.time())
        self._publish_robot_state()
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._odom_thread:
            self._odom_thread.join(timeout=1.0)
        self.alive.publish(False)
        self._publish_robot_state()
        super().stop()

    def _on_cmd(self, twist: Twist) -> None:
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._stopped = False

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
            self._stopped = True

    def _odom_loop(self) -> None:
        period_s = 1.0 / self._odom_hz
        while self._running:
            cos_yaw = math.cos(self._yaw)
            sin_yaw = math.sin(self._yaw)
            self._pos_x += (self._vx * cos_yaw - self._vy * sin_yaw) * period_s
            self._pos_y += (self._vx * sin_yaw + self._vy * cos_yaw) * period_s
            self._yaw += self._wz * period_s
            self._publish_odom(time.time())
            time.sleep(period_s)

    def _publish_odom(self, timestamp: float) -> None:
        self.odometry.publish(
            Odometry(
                pose=Pose(
                    position=Vector3(self._pos_x, self._pos_y, 0.0),
                    orientation=Quaternion(
                        0.0,
                        0.0,
                        math.sin(self._yaw / 2.0),
                        math.cos(self._yaw / 2.0),
                    ),
                ),
                twist=Twist(
                    linear=Vector3(self._vx, self._vy, 0.0),
                    angular=Vector3(0.0, 0.0, self._wz),
                ),
                ts=timestamp,
                frame_id=self._odom_frame_id,
                child_frame_id=self._child_frame_id,
            )
        )

    def _publish_robot_state(self) -> None:
        self.robot_state.publish(
            {
                "standing": True,
                "enabled": True,
                "emergency": False,
                "connected": True,
                "battery_voltage": 0.0,
                "battery_soc": 0.0,
                "current_gait": "none",
                "timestamp": time.time(),
            }
        )

    def health(self) -> dict[str, Any]:
        stats = super().port_summary()
        stats["stub"] = {"stopped": self._stopped}
        return stats
