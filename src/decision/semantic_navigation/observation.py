"""Low-rate observation-goal proposals; native planning decides feasibility."""

import math
from dataclasses import dataclass

from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3


@dataclass(frozen=True)
class ObjectTarget:
    object_id: str
    label: str
    position: tuple[float, float, float]


@dataclass(frozen=True)
class ObservationRequest:
    instruction_epoch: int
    instruction: str
    target: ObjectTarget
    robot_pose: PoseStamped
    verification_task_id: str = ""


def observation_candidates(target: ObjectTarget, robot: PoseStamped, standoff: float) -> list[PoseStamped]:
    """Propose poses facing the target at the current robot navigation height.

    These are hypotheses, not traversability or visibility decisions. Prefer
    the observed side of the object; native preview must admit a proposal.
    """
    x, y, z = target.position
    if not all(math.isfinite(v) for v in (x, y, z, robot.x, robot.y, robot.z, standoff)) or standoff <= 0:
        raise ValueError("observation coordinates and positive standoff must be finite")
    bearing = math.atan2(robot.y - y, robot.x - x)
    poses = []
    for offset in (0, -math.pi / 4, math.pi / 4, -math.pi / 2, math.pi / 2, -3 * math.pi / 4, 3 * math.pi / 4, math.pi):
        angle = bearing + offset
        px, py = x + standoff * math.cos(angle), y + standoff * math.sin(angle)
        poses.append(PoseStamped(
            Pose(Vector3(px, py, robot.z), Quaternion.from_yaw(math.atan2(y - py, x - px))),
            frame_id=robot.frame_id,
        ))
    return poses
