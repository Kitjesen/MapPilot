"""TAREROS2BridgeModule -- native CycloneDDS subscriber for TARE exploration topics.

Extracted from ``module.py`` where ROS fallbacks polluted the main module.
This bridge uses native CycloneDDS (no rclpy/ROS2 dependency) to subscribe to
TARE's DDS topics and publish framework port messages. The main
``TAREExplorerModule`` uses the in-process policy by default and never imports
rclpy.

Usage
-----
Add this module to the blueprint when the TARE C++ node publishes over DDS
topics from the TARE exploration topic contract::

    bp.add(
        TAREROS2BridgeModule,
        alias="TAREROS2BridgeModule",
        way_point_topic=TOPICS.exploration_way_point,
        ...
    )
    bp.wire("TAREROS2BridgeModule", "exploration_goal", "nav.mission", "goal_pose")
    ...

Output port contract matches ``TAREExplorerModule.exploration_goal``
so ``autoconnect`` wiring is identical.
"""

from __future__ import annotations

import logging
import os
from typing import Any

from explore.tare.dds_types import (
    HAS_CYCLONEDDS,
    DDS_Bool,
    DDS_Float32,
    DDS_Path,
    DDS_PointStamped,
)
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

_DEFAULT_GOAL_FRAME_ID = topic_default_frame_id(TOPICS.goal_pose)


@register(
    "exploration",
    "tare_ros2_bridge",
    description="Native CycloneDDS bridge for TARE exploration topics",
)
class TAREROS2BridgeModule(Module, layer=5):
    """Subscribe to TARE DDS topics and publish via framework ports.

    Uses native CycloneDDS (no rclpy/ROS2 dependency). Publishes the same
    output port contract so the rest of the system is unchanged.
    """

    exploration_goal: Out[PoseStamped]  # -> Navigation.goal_pose
    exploration_path: Out[list]  # optional executable strategy path
    exploring: Out[bool]  # activity indicator
    runtime: Out[float]  # per-cycle runtime ms
    finish: Out[bool]  # exploration done
    alive: Out[bool]  # bridge health

    # Receive start/stop signals from blueprint (wired from TAREExplorerModule
    # or directly from user control).
    start_signal: In[bool]

    def __init__(
        self,
        way_point_topic: str = TOPICS.exploration_way_point,
        path_topic: str = TOPICS.exploration_local_path,
        runtime_topic: str = TOPICS.exploration_runtime,
        finish_topic: str = TOPICS.exploration_finish,
        start_topic: str = TOPICS.exploration_start,
        goal_frame_id: str = "",
        qos_depth: int = 10,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._way_point_topic = way_point_topic
        self._path_topic = path_topic
        self._runtime_topic = runtime_topic
        self._finish_topic = finish_topic
        self._start_topic = start_topic
        self._goal_frame_id = str(goal_frame_id or "")
        self._qos_depth = int(qos_depth)

        self._reader = None
        self._participant = None
        self._publisher = None
        self._dds_bool_type = None

    # -- lifecycle -------------------------------------------------------

    def setup(self) -> None:
        self.start_signal.subscribe(self._on_start_signal)

        if not HAS_CYCLONEDDS:
            logger.warning("TAREROS2BridgeModule: cyclonedds not available, bridge disabled")
            return

        try:
            from runtime.adapters.dds.reader import DDSReader

            domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
            reader = DDSReader(domain_id=domain_id)
            reader.subscribe(self._way_point_topic, DDS_PointStamped, self._on_waypoint)
            reader.subscribe(self._path_topic, DDS_Path, self._on_path)
            reader.subscribe(self._runtime_topic, DDS_Float32, self._on_runtime)
            reader.subscribe(self._finish_topic, DDS_Bool, self._on_finish)
            if not reader.start():
                logger.warning("TAREROS2BridgeModule: DDSReader failed to start")
                return
            reader.spin_background()
            self._reader = reader

            # Publisher for the start/stop signal (native cyclonedds).
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.pub import DataWriter, Publisher
            from cyclonedds.topic import Topic

            dp = DomainParticipant(domain_id)
            self._participant = dp
            self._publisher = DataWriter(
                Publisher(dp),
                Topic(dp, self._start_topic, DDS_Bool),
            )
            self._dds_bool_type = DDS_Bool
            logger.info(
                "TAREROS2BridgeModule: subscribed to TARE DDS topics (domain_id=%s)",
                domain_id,
            )
        except Exception as exc:
            self._cleanup_dds()
            logger.warning("TAREROS2BridgeModule disabled: %s", exc)

    def start(self) -> None:
        super().start()
        self.alive.publish(self._reader is not None)

    def stop(self) -> None:
        self._cleanup_dds()
        super().stop()

    def _cleanup_dds(self) -> None:
        if self._reader is not None:
            try:
                self._reader.stop()
            except Exception:
                pass
            self._reader = None
        # DataWriter and Publisher are owned by DomainParticipant;
        # clearing references lets GC reclaim them.
        self._publisher = None
        self._participant = None
        self._dds_bool_type = None

    # -- DDS callbacks -> framework ports --------------------------------

    def _on_waypoint(self, msg) -> None:
        try:
            frame = msg.header.frame_id or _DEFAULT_GOAL_FRAME_ID
            output_frame = self._goal_frame_id or frame
            pose = PoseStamped(
                pose=Pose(
                    position=Vector3(
                        x=float(msg.point.x),
                        y=float(msg.point.y),
                        z=float(msg.point.z),
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                frame_id=output_frame,
            )
            self.exploration_goal.publish(pose)
        except Exception as e:
            logger.debug("TARE bridge waypoint error: %s", e)

    def _on_path(self, msg) -> None:
        try:
            frame = str(getattr(getattr(msg, "header", None), "frame_id", "") or _DEFAULT_GOAL_FRAME_ID)
            pts = [
                {
                    "x": float(ps.pose.position.x),
                    "y": float(ps.pose.position.y),
                    "z": float(ps.pose.position.z),
                    "frame_id": frame,
                }
                for ps in msg.poses
            ]
            self.exploration_path.publish(pts)
        except Exception as e:
            logger.debug("TARE bridge path error: %s", e)

    def _on_runtime(self, msg) -> None:
        try:
            self.runtime.publish(float(msg.data))
        except Exception:
            pass

    def _on_finish(self, msg) -> None:
        try:
            self.finish.publish(bool(msg.data))
        except Exception:
            pass

    def _on_start_signal(self, enable: bool) -> None:
        """Forward start/stop signal to the TARE node via DDS."""
        if self._publisher is None or self._dds_bool_type is None:
            return
        try:
            msg = self._dds_bool_type(data=bool(enable))
            self._publisher.write(msg)
        except Exception as e:
            logger.debug("TARE bridge start signal error: %s", e)


# Alias reflecting the native DDS transport.
TAREDDSBridgeModule = TAREROS2BridgeModule
