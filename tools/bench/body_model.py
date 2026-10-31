"""Query actual URDF collision primitives against offline obstacle boxes.

This is a static pose oracle for experiments. It neither supplies ground
support nor models gait, sensor uncertainty, or the field collision bitmap.
Visual meshes are deliberately excluded; collision geometry is never dropped.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


class BodyModel:
    """Preserve URDF link transforms and query robot/environment distances.

    ``geometry`` exposes MuJoCo's primitive sizes: box half extents, cylinder
    radius/half height, and sphere radius. Positive clearance is a conservative
    AABB separation bound when primitive bounds do not overlap. Overlapping
    bounds use MuJoCo's signed narrow-phase distance. ``cutoff`` caps the result.
    The nominal Go2 joint angles are a display/test stance, not measured gait.
    """

    def __init__(
        self,
        urdf: Path,
        obstacles: list[dict],
        joint_angles: dict[str, float] | None = None,
    ):
        self.urdf = Path(urdf)
        robot = ET.parse(self.urdf).getroot()
        links = robot.findall("link")
        children = {joint.find("child").get("link") for joint in robot.findall("joint")}
        roots = [link.get("name") for link in links if link.get("name") not in children]
        if len(roots) != 1:
            raise ValueError("URDF must have one root link")
        root_link = roots[0]

        collision_links = []
        visual_count = 0
        for link in links:
            for visual in link.findall("visual"):
                link.remove(visual)
                visual_count += 1
            for collision in link.findall("collision"):
                shape = collision.find("geometry")
                if shape is None or len(shape) != 1:
                    raise ValueError("Each URDF collision must declare one geometry")
                kind = shape[0].tag
                if kind not in {"box", "cylinder", "sphere"}:
                    raise ValueError(f"Unsupported collision geometry {kind!r}; it was not discarded")
                collision.set("name", f"body_oracle_robot_{len(collision_links)}")
                collision_links.append(link.get("name"))

        fixture = ET.SubElement(robot, "link", name="body_oracle_fixture")
        joint = ET.SubElement(robot, "joint", name="body_oracle_mount", type="fixed")
        ET.SubElement(joint, "parent", link="body_oracle_fixture")
        ET.SubElement(joint, "child", link=root_link)
        obstacle_names = []
        for index, obstacle in enumerate(obstacles):
            name = str(obstacle["name"])
            if not name or name in obstacle_names:
                raise ValueError("Obstacle names must be nonempty and unique")
            center = self._vector(obstacle["center"], 3, "obstacle center")
            half_size = self._vector(obstacle["half_size"], 3, "obstacle half size")
            if np.any(half_size <= 0):
                raise ValueError("Obstacle half sizes must be positive")
            collision = ET.SubElement(fixture, "collision", name=f"body_oracle_obstacle_{index}")
            ET.SubElement(collision, "origin", xyz=" ".join(map(str, center)))
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(geometry, "box", size=" ".join(map(str, 2 * half_size)))
            obstacle_names.append(name)

        extension = robot.find("mujoco")
        if extension is None:
            extension = ET.SubElement(robot, "mujoco")
        compiler = extension.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(extension, "compiler")
        compiler.set("fusestatic", "false")
        compiler.set("discardvisual", "true")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self._fromto = np.zeros(6)
        self._body_id = self.model.body(root_link).id
        self._robot_geoms = [self.model.geom(f"body_oracle_robot_{i}").id
                             for i in range(len(collision_links))]
        self._obstacle_geoms = [self.model.geom(f"body_oracle_obstacle_{i}").id
                                for i in range(len(obstacle_names))]
        if self.model.ngeom != len(self._robot_geoms) + len(self._obstacle_geoms):
            raise ValueError("Imported geometry count differs from the declared collision count")
        self._links = dict(zip(self._robot_geoms, collision_links))
        self._obstacles = dict(zip(self._obstacle_geoms, obstacle_names))
        self.data.qpos[:] = self.model.qpos0
        assignments = {}
        go2_stance = robot.get("name", "").startswith("go2")
        if go2_stance:
            for leg in ("FR", "FL", "RR", "RL"):
                for part, angle in (("hip", 0.0), ("thigh", 0.8), ("calf", -1.6)):
                    name = f"{leg}_{part}_joint"
                    if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0:
                        assignments[name] = angle
        assignments.update(joint_angles or {})
        for name, angle in assignments.items():
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"Unknown URDF joint {name!r}")
            if self.model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"Joint {name!r} is not a revolute joint")
            if not math.isfinite(angle):
                raise ValueError(f"Nonfinite angle for joint {name!r}")
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = angle
        source_path = self.urdf.resolve()
        try:
            source_path = source_path.relative_to(Path(__file__).resolve().parents[2])
        except ValueError:
            pass
        pose_source = "nominal Go2 stance" if go2_stance else "URDF default joint positions"
        self.report = {
            "urdf": source_path.as_posix(),
            "source_collision_count": len(collision_links),
            "imported_collision_count": len(self._robot_geoms),
            "collision_links": collision_links,
            "visual_elements_removed": visual_count,
            "joint_angles": assignments,
            "joint_state_source": pose_source + (" with explicit overrides" if joint_angles else ""),
            "distance_solver": "exact primitive AABB separation bound; MuJoCo native CCD for overlapping bounds",
            "positive_clearance_semantics": "conservative lower bound, not universally exact surface distance",
            "obstacles": obstacle_names,
            "gait_or_ground_support_validated": False,
        }

    @staticmethod
    def _vector(value, length: int, name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (length,) or not np.isfinite(vector).all():
            raise ValueError(f"{name} must contain {length} finite values")
        return vector

    def _pose(self, pose) -> None:
        x, y, z, yaw = self._vector(pose, 4, "pose")
        self.model.body_pos[self._body_id] = (x, y, z)
        self.model.body_quat[self._body_id] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        mujoco.mj_forward(self.model, self.data)

    def clearance(self, pose, cutoff: float = 1.0) -> tuple[float, str, str]:
        """Return conservative clearance and its limiting robot/obstacle pair."""
        if not math.isfinite(cutoff) or cutoff <= 0:
            raise ValueError("cutoff must be finite and positive")
        self._pose(pose)
        nearest = float(cutoff)
        nearest_link = nearest_obstacle = ""
        obstacle_bounds = [(geom_id, self.data.geom_xpos[geom_id], self._extent(geom_id))
                           for geom_id in self._obstacle_geoms]
        for robot_id in self._robot_geoms:
            robot_extent = self._extent(robot_id)
            robot_center = self.data.geom_xpos[robot_id]
            for obstacle_id, obstacle_center, obstacle_extent in obstacle_bounds:
                separation = np.abs(robot_center - obstacle_center)
                gap = np.maximum(0, separation - robot_extent - obstacle_extent)
                distance = float(np.linalg.norm(gap))
                if distance == 0:
                    self._fromto[:] = 0
                    distance = mujoco.mj_geomDistance(
                        self.model, self.data, robot_id, obstacle_id, cutoff, self._fromto)
                if distance < nearest:
                    nearest = float(distance)
                    nearest_link = self._links[robot_id]
                    nearest_obstacle = self._obstacles[obstacle_id]
        return nearest, nearest_link, nearest_obstacle

    def _extent(self, geom_id: int) -> np.ndarray:
        """Exact world-axis half extents for each supported collision primitive."""
        kind = self.model.geom_type[geom_id]
        size = self.model.geom_size[geom_id]
        rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            return np.abs(rotation) @ size
        if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            axis = rotation[:, 2]
            return size[0] * np.sqrt(np.maximum(0, 1 - axis**2)) + size[1] * np.abs(axis)
        return np.full(3, size[0])

    def geometry(self, pose) -> list[dict]:
        """Return all imported robot collision primitives in world coordinates."""
        self._pose(pose)
        kinds = {
            mujoco.mjtGeom.mjGEOM_BOX: "box",
            mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
            mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
        }
        return [{
            "link": self._links[geom_id],
            "type": kinds[self.model.geom_type[geom_id]],
            "center": self.data.geom_xpos[geom_id].tolist(),
            "rotation": self.data.geom_xmat[geom_id].reshape(3, 3).tolist(),
            "size": self.model.geom_size[geom_id].tolist(),
        } for geom_id in self._robot_geoms]
