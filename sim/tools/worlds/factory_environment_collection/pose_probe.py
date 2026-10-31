"""Offline robot placement checks on unchanged physical Factory geometry.

This is not a planner, controller, spawn qualification, or a motion authority.
It performs no physics steps and never inserts a ground plane.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np


class FactoryPoseProbe:
    """Compose the packaged robot once; query actual support and world contacts."""

    _PREFIX = "placement_world__"

    def __init__(
        self,
        world_xml: Path,
        robot_xml: Path,
        *,
        root_joint: str,
        initial_keyframe: str,
        support_geoms: list[str],
        foot_geoms: list[str],
    ) -> None:
        world = mujoco.MjSpec.from_file(str(world_xml))
        self.world = world.compile()
        if self.world.nq or self.world.nv:
            raise ValueError("placement probe requires a static world")
        self.world_data = mujoco.MjData(self.world)
        mujoco.mj_forward(self.world, self.world_data)
        robot = mujoco.MjSpec.from_file(str(robot_xml))
        # Match the runtime merge: packaged development ground is not this world.
        for geom in list(robot.worldbody.geoms):
            if geom.name in {"ground", "floor"} or geom.type == mujoco.mjtGeom.mjGEOM_PLANE:
                robot.delete(geom)
        robot.attach(world, prefix=self._PREFIX, frame=robot.worldbody.add_frame())
        self.model = robot.compile()
        self.data = mujoco.MjData(self.model)
        joint = self.model.joint(root_joint)
        if int(self.model.jnt_type[joint.id]) != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("placement root must be the declared free joint")
        self._qadr = int(joint.qposadr[0])
        self._key = self.model.key(initial_keyframe).id
        self._support = set(support_geoms)
        for name in support_geoms:
            self.world.geom(name)
        self._feet = [self.model.geom(name).id for name in foot_geoms]
        if not self._feet:
            raise ValueError("placement requires explicit physical support geoms")
        self._world_geoms = {
            self.model.geom(self._PREFIX + self.world.geom(i).name).id: self.world.geom(i).name
            for i in range(self.world.ngeom)
        }

    def _bottom_extent(self, geom_id: int) -> float:
        kind = int(self.model.geom_type[geom_id])
        size = self.model.geom_size[geom_id]
        rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
        if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            return float(size[0])
        if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            axis_z = float(rotation[2, 2])
            return float(size[0] * np.sqrt(max(0, 1 - axis_z * axis_z)) + size[1] * abs(axis_z))
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            return float(np.abs(rotation[2]) @ size)
        raise ValueError("placement foot geometry needs an explicit analytic support extent")

    def probe(
        self, *, position_m, quaternion_wxyz, max_support_gap_m: float = 0.05, max_support_penetration_m: float = 0.003
    ) -> dict:
        position, quaternion = np.asarray(position_m, dtype=float), np.asarray(quaternion_wxyz, dtype=float)
        if (
            position.shape != (3,)
            or quaternion.shape != (4,)
            or not np.isfinite(position).all()
            or not np.isfinite(quaternion).all()
            or not np.isclose(quaternion @ quaternion, 1, atol=1e-6)
        ):
            raise ValueError("placement pose requires finite XYZ and unit WXYZ")
        mujoco.mj_resetDataKeyframe(self.model, self.data, self._key)
        self.data.qpos[self._qadr : self._qadr + 3] = position
        self.data.qpos[self._qadr + 3 : self._qadr + 7] = quaternion
        mujoco.mj_forward(self.model, self.data)
        support = []
        for geom_id in self._feet:
            center = self.data.geom_xpos[geom_id].copy()
            hit = np.array([-1], dtype=np.int32)
            distance = float(
                mujoco.mj_ray(
                    self.world,
                    self.world_data,
                    center,
                    np.array([0.0, 0.0, -1.0]),
                    np.ones(6, dtype=np.uint8),
                    1,
                    -1,
                    hit,
                )
            )
            name = self.world.geom(int(hit[0])).name if distance >= 0 else None
            gap = distance - self._bottom_extent(geom_id) if distance >= 0 else None
            valid = name in self._support and gap is not None and -max_support_penetration_m <= gap <= max_support_gap_m
            support.append(
                dict(
                    robot_geom=self.model.geom(geom_id).name,
                    world_geom=name,
                    height_m=float(center[2] - distance) if distance >= 0 else None,
                    gap_m=gap,
                    valid=bool(valid),
                )
            )
        blocking = []
        for contact in self.data.contact[: self.data.ncon]:
            first, second = map(int, contact.geom)
            if (first in self._world_geoms) == (second in self._world_geoms):
                continue
            world_id, robot_id = (first, second) if first in self._world_geoms else (second, first)
            name = self._world_geoms[world_id]
            if robot_id in self._feet and name in self._support and contact.dist >= -max_support_penetration_m:
                continue
            blocking.append(
                dict(
                    robot_geom_id=robot_id,
                    robot_body=self.model.body(int(self.model.geom_bodyid[robot_id])).name,
                    world_geom=name,
                    distance_m=float(contact.dist),
                )
            )
        return dict(
            position_m=position.tolist(),
            quaternion_wxyz=quaternion.tolist(),
            placement_clear=not blocking and all(row["valid"] for row in support),
            support=support,
            blocking_contacts=blocking,
            motion_admitted=False,
            physics_steps=0,
            source_world_geom_count=self.world.ngeom,
            composed_world_geom_count=len(self._world_geoms),
            composed_total_geom_count=self.model.ngeom,
        )
