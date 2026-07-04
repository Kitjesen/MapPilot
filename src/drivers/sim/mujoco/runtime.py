"""MuJoCo live-runtime helpers used by LingTu simulation bridges.

This module owns the product-side MuJoCo scene resolution and engine
construction. Validation scripts may call it, but they should not duplicate
runtime construction logic in ``sim/scripts``.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[4]
SIM_ROOT = ROOT / "sim"
DEFAULT_MID360_PATTERN = SIM_ROOT / "assets" / "livox" / "mid360.npy"
DEFAULT_MID360_SAMPLES_PER_FRAME = 15000


def resolve_mid360_pattern(path: Path | str | None) -> Path | None:
    """Resolve the official MID-360 pattern asset, or return ``None`` explicitly."""

    if path is None or str(path).strip() == "":
        return None
    candidate = Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = (ROOT / candidate).resolve()
    if not candidate.is_file():
        raise FileNotFoundError(
            f"MID-360 scan pattern not found: {candidate}. "
            "Pass --mid360-pattern or --allow-golden-spiral-lidar explicitly."
        )
    return candidate


def resolve_world(world: str) -> Path:
    """Resolve a registered MuJoCo world name or filesystem path."""

    from drivers.sim.mujoco.driver import WORLDS, _WORLDS_DIR

    candidate = Path(world)
    if candidate.exists():
        return candidate.resolve()
    mapped = WORLDS.get(world, world)
    path = (_WORLDS_DIR / mapped).resolve()
    if not path.exists():
        raise FileNotFoundError(f"MuJoCo world not found: {world} -> {path}")
    return path


def scene_start(scene_xml: Path) -> list[float] | None:
    """Return the robot start pose declared by a LingTu MuJoCo scene."""

    from drivers.sim.mujoco.driver import _scene_placeholder_start

    start = _scene_placeholder_start(scene_xml)
    if start is not None:
        return start
    try:
        import xml.etree.ElementTree as ET

        root = ET.fromstring(scene_xml.read_text(encoding="utf-8", errors="ignore"))
        worldbody = root.find("worldbody")
        if worldbody is None:
            return None
        for body in worldbody.findall("body"):
            if body.attrib.get("name") != "base_link":
                continue
            parts = [float(v) for v in body.attrib.get("pos", "").split()]
            if len(parts) >= 3:
                return parts[:3]
    except (ET.ParseError, AttributeError, KeyError, ValueError, TypeError):
        return None
    return None


def parse_start(value: str) -> list[float] | None:
    """Parse CLI start pose formatted as ``x,y,z``."""

    if not value:
        return None
    parts = [float(item.strip()) for item in value.replace(";", ",").split(",") if item.strip()]
    if len(parts) != 3:
        raise ValueError("--start must be formatted as x,y,z")
    return parts


def scene_with_memory(scene_xml: Path, memory: str) -> Path:
    """Return a temporary scene with a MuJoCo ``<size memory=...>`` element."""

    if not memory:
        return scene_xml
    text = scene_xml.read_text(encoding="utf-8", errors="ignore")
    if "<size " in text:
        return scene_xml
    marker_start = text.find("<mujoco")
    if marker_start < 0:
        return scene_xml
    marker_end = text.find(">", marker_start)
    if marker_end < 0:
        return scene_xml
    patched = text[: marker_end + 1] + f'\n  <size memory="{memory}"/>' + text[marker_end + 1 :]
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml",
        prefix=f"{scene_xml.stem}_memory_",
        dir=str(scene_xml.parent),
        mode="w",
        encoding="utf-8",
        delete=False,
    )
    tmp.write(patched)
    tmp.close()
    return Path(tmp.name)


def build_engine(
    *,
    world: Path,
    drive_mode: str,
    n_rays: int,
    start: list[float] | None,
    mujoco_memory: str,
    camera_configs: list[Any] | None = None,
    robot_xml: Path | None = None,
    base_body_name: str = "base_link",
    lidar_body_name: str = "lidar_link",
    leg_joint_names: list[str] | None = None,
    mid360_pattern: Path | str | None = DEFAULT_MID360_PATTERN,
    mid360_samples_per_frame: int = DEFAULT_MID360_SAMPLES_PER_FRAME,
    lidar_backend: str = "mujoco_lidar",
    mujoco_lidar_backend: str = "cpu",
    require_mature_lidar_backend: bool = True,
    allow_legacy_lidar_fallback: bool = False,
    policy_path: Path | str | None = None,
):
    """Build the canonical in-process MuJoCo engine for live LingTu gates."""

    from sim.engine.core.robot import RobotConfig
    from sim.engine.core.sensor import LidarConfig
    from sim.engine.core.world import WorldConfig
    from sim.engine.mujoco.engine import MuJoCoEngine

    robot_cfg = RobotConfig.default_thunder_v3()
    robot_cfg.resolve_paths(base_dir=str(SIM_ROOT))
    if robot_xml is not None:
        robot_cfg.robot_xml = str(robot_xml)
    robot_cfg.base_body_name = str(base_body_name)
    robot_cfg.lidar_body_name = str(lidar_body_name)
    if leg_joint_names is not None:
        robot_cfg.leg_joint_names = list(leg_joint_names)
    if policy_path is not None and str(policy_path).strip():
        candidate = Path(policy_path).expanduser()
        if not candidate.is_absolute():
            candidate = (ROOT / candidate).resolve()
        robot_cfg.policy_onnx = str(candidate)
    start = start or scene_start(world)
    if start is not None:
        robot_cfg.init_position = [float(v) for v in start[:3]]
    pattern_path = resolve_mid360_pattern(mid360_pattern)

    load_world = scene_with_memory(world, mujoco_memory)
    engine = MuJoCoEngine(
        robot_config=robot_cfg,
        world_config=WorldConfig(scene_xml=str(load_world)),
        lidar_config=LidarConfig(
            body_name=robot_cfg.lidar_body_name,
            n_rays=int(n_rays),
            geom_group=0,
            add_noise=True,
            backend=str(lidar_backend),
            mujoco_lidar_backend=str(mujoco_lidar_backend),
            require_mature_backend=bool(require_mature_lidar_backend),
            allow_legacy_fallback=bool(allow_legacy_lidar_fallback),
            site_name="lidar_site",
            mid360_npy_path=str(pattern_path) if pattern_path is not None else None,
            samples_per_frame=int(mid360_samples_per_frame),
        ),
        camera_configs=camera_configs or [],
        headless=True,
        drive_mode=drive_mode,
    )
    try:
        engine.load(str(load_world))
        engine.reset()
    finally:
        if load_world != world:
            load_world.unlink(missing_ok=True)
    return engine
