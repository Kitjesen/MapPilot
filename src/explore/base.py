"""Standardized exploration Module contract for LingTu.

``ExploreModule`` formalizes the Python-side exploration port contract so that
every exploration backend (TARE, wavefront, future planners) presents the same
interface to ``autoconnect`` and the mission stack. The heavy algorithm lives in
C++ (``src/explore/cpp`` exposed through ``lingtu_explore_kernel``); Python
subclasses only convert data and orchestrate ports.

Port contract
-------------
Output ``exploration_goal: Out[PoseStamped]`` mirrors
``WavefrontFrontierExplorer`` exactly so autoconnect can wire either backend to
``Navigation.goal_pose`` and the backend can be hot-swapped without other
modules noticing.

Subclasses implement :meth:`_plan_once` to produce goals and use
:meth:`_build_pose_stamped` for the shared PoseStamped conversion.
"""

from __future__ import annotations

import logging

from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

DEFAULT_GOAL_FRAME_ID = topic_default_frame_id(TOPICS.goal_pose)


class ExploreModule(Module, layer=5):
    """Base class for LingTu exploration modules.

    Declares the shared exploration port contract. ``Module.__init__`` scans
    type hints across the full MRO, so subclasses inherit these ports without
    redeclaring them. Subclasses add backend-specific ports (e.g. TARE adds
    ``tare_stats``) and implement :meth:`_plan_once`.
    """

    exploration_goal: Out[PoseStamped]  # -> Navigation.goal_pose
    exploration_path: Out[list]  # optional executable strategy path
    exploring: Out[bool]  # activity indicator
    alive: Out[bool]
    odometry: In[Odometry]
    exploration_grid: In[dict]
    navigation_status: In[dict]

    # -- shared helpers ------------------------------------------------------

    @staticmethod
    def _build_pose_stamped(
        x: float,
        y: float,
        z: float,
        frame_id: str = DEFAULT_GOAL_FRAME_ID,
    ) -> PoseStamped:
        """Convert an exploration point to a LingTu PoseStamped.

        Exploration backends only provide a point; downstream Navigation derives
        heading from the path tangent, so identity orientation is intentional.
        """
        return PoseStamped(
            pose=Pose(
                position=Vector3(x=x, y=y, z=z),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            frame_id=frame_id,
        )

    # -- planning contract ---------------------------------------------------

    def _plan_once(self) -> bool:
        """Run one planning cycle. Return True when a goal was emitted.

        Subclasses must implement this. The base raises to make the contract
        explicit for new exploration backends.
        """
        raise NotImplementedError(f"{type(self).__name__} must implement _plan_once()")
