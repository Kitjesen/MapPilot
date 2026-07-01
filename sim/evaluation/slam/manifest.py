"""Declarative SLAM simulation evaluation case manifests."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SlamEvalRobot:
    name: str
    model_path: str
    frame_id: str = "body"
    config_path: str = "config/robot_config.yaml"


@dataclass(frozen=True)
class SlamEvalSensorSuite:
    lidar: str = "livox_mid360"
    camera: str = "front_rgbd"
    imu: str = "sim_imu"
    gnss: str | None = None
    lidar_topic: str = "/livox/lidar"
    imu_topic: str = "/livox/imu"
    ground_truth_topic: str = "/sim/ground_truth"


@dataclass(frozen=True)
class SlamEvalBackend:
    name: str
    profile: str
    odometry_topic: str = "/slam/odometry"
    map_topic: str = "/slam/map_cloud"


@dataclass(frozen=True)
class SlamEvalCase:
    name: str
    world: str
    robot: SlamEvalRobot
    sensors: SlamEvalSensorSuite
    backend: SlamEvalBackend
    duration_s: float = 120.0
    reference_source: str = "sim_ground_truth"
    output_dir: str = "sim/output/slam_eval"
    max_association_dt_s: float = 0.02
    tags: tuple[str, ...] = field(default_factory=tuple)

    def validate(self, *, repo_root: str | Path | None = None) -> list[str]:
        errors: list[str] = []
        if not self.name:
            errors.append("name is required")
        if self.duration_s <= 0.0:
            errors.append("duration_s must be positive")
        if self.max_association_dt_s < 0.0:
            errors.append("max_association_dt_s must be non-negative")
        if not self.backend.name:
            errors.append("backend.name is required")
        if not self.backend.profile:
            errors.append("backend.profile is required")
        if repo_root is not None:
            root = Path(repo_root)
            for label, raw_path in (
                ("world", self.world),
                ("robot.model_path", self.robot.model_path),
            ):
                if raw_path and not (root / raw_path).exists():
                    errors.append(f"{label} does not exist: {raw_path}")
        return errors


def case_from_dict(data: dict[str, Any]) -> SlamEvalCase:
    robot = SlamEvalRobot(**data["robot"])
    sensors = SlamEvalSensorSuite(**data.get("sensors", {}))
    backend = SlamEvalBackend(**data["backend"])
    tags = tuple(str(tag) for tag in data.get("tags", ()))
    return SlamEvalCase(
        name=str(data["name"]),
        world=str(data["world"]),
        robot=robot,
        sensors=sensors,
        backend=backend,
        duration_s=float(data.get("duration_s", 120.0)),
        reference_source=str(data.get("reference_source", "sim_ground_truth")),
        output_dir=str(data.get("output_dir", "sim/output/slam_eval")),
        max_association_dt_s=float(data.get("max_association_dt_s", 0.02)),
        tags=tags,
    )


def load_case(path: str | Path) -> SlamEvalCase:
    manifest_path = Path(path)
    with manifest_path.open("r", encoding="utf-8") as handle:
        return case_from_dict(json.load(handle))
