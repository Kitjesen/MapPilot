"""Scene configuration — WorldConfig and SimWorld."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class ObstacleConfig:
    """Static obstacle configuration."""

    name: str
    shape: str                    # "box" | "sphere" | "cylinder"
    size: List[float]             # box:[lx,ly,lz], sphere:[r], cylinder:[r,h]
    position: List[float]         # [x, y, z]
    rgba: List[float] = field(default_factory=lambda: [0.7, 0.7, 0.7, 1.0])
    is_static: bool = True        # True=fixed obstacle, False=dynamic rigid body


@dataclass
class WorldConfig:
    """Scene configuration, engine-agnostic.

    Contains terrain, obstacles, lighting, and physical parameters.
    Can be loaded from a YAML file or constructed directly in code.
    """

    # Scene XML / terrain file
    scene_xml: Optional[str] = None         # full scene XML path (highest priority)
    terrain_type: str = "flat"              # "flat" | "rough" | "stairs" | "slope"
    terrain_size: List[float] = field(default_factory=lambda: [30.0, 30.0])

    # Physical parameters
    gravity: List[float] = field(default_factory=lambda: [0.0, 0.0, -9.81])
    timestep: float = 0.002                 # MuJoCo physics timestep (s)
    floor_friction: List[float] = field(default_factory=lambda: [1.0, 0.5, 0.5])

    # Obstacle list
    obstacles: List[ObstacleConfig] = field(default_factory=list)

    # Lighting (MuJoCo visual params)
    shadow_quality: int = 2048
    ambient: List[float] = field(default_factory=lambda: [0.4, 0.4, 0.4])

    # Render resolution (offscreen)
    offscreen_width: int = 1280
    offscreen_height: int = 960

    # Navigation goal marker (for visualization)
    goal_position: Optional[List[float]] = None   # [x, y, z]

    # Additional MuJoCo XML fragments to insert inside worldbody
    extra_xml_bodies: str = ""

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "WorldConfig":
        """Construct from dictionary (supports YAML loading)."""
        obstacles = [ObstacleConfig(**o) for o in d.pop("obstacles", [])]
        cfg = cls(**{k: v for k, v in d.items() if k != "obstacles"})
        cfg.obstacles = obstacles
        return cfg

    def to_scene_xml(self, robot_xml_path: str) -> str:
        """Generate MuJoCo scene XML string.

        If scene_xml is set, returns its content directly;
        otherwise generates dynamically from WorldConfig.

        Args:
            robot_xml_path: relative or absolute path to robot.xml (used in <include>)
        """
        if self.scene_xml is not None:
            p = Path(self.scene_xml)
            if p.exists():
                return p.read_text()

        gravity_str = " ".join(str(g) for g in self.gravity)
        friction_str = " ".join(str(f) for f in self.floor_friction)
        ambient_str = " ".join(str(a) for a in self.ambient)

        # Generate obstacle XML
        obstacle_xml = ""
        for obs in self.obstacles:
            rgba_str = " ".join(str(c) for c in obs.rgba)
            pos_str = " ".join(str(p) for p in obs.position)
            if obs.shape == "box":
                size_str = " ".join(str(s) for s in obs.size)
            elif obs.shape == "sphere":
                size_str = str(obs.size[0])
            elif obs.shape == "cylinder":
                size_str = f"{obs.size[0]} {obs.size[1]}"
            else:
                size_str = " ".join(str(s) for s in obs.size)
            contype = "0" if not obs.is_static else "1"
            obstacle_xml += (
                f'    <geom name="{obs.name}" type="{obs.shape}" size="{size_str}" '
                f'pos="{pos_str}" rgba="{rgba_str}" '
                f'contype="{contype}" conaffinity="1" group="1"/>\n'
            )

        # Goal marker
        goal_xml = ""
        if self.goal_position is not None:
            gp = " ".join(str(c) for c in self.goal_position)
            goal_xml = (
                f'    <geom name="goal_marker" type="sphere" size="0.3" pos="{gp}" '
                f'contype="0" conaffinity="0" rgba="1 0.2 0.1 0.5" group="1"/>\n'
            )

        terrain_size_str = f"{self.terrain_size[0]} {self.terrain_size[1]} 0.1"

        return f"""\
<mujoco model="lingtu_sim">
  <compiler angle="radian"/>
  <option gravity="{gravity_str}" timestep="{self.timestep}"/>
  <visual>
    <global offwidth="{self.offscreen_width}" offheight="{self.offscreen_height}"/>
    <headlight ambient="{ambient_str}"/>
    <quality shadowsize="{self.shadow_quality}"/>
    <map znear="0.01" zfar="200"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".85 .85 .82" rgb2=".7 .7 .68"/>
    <material name="grid" texture="grid" texrepeat="20 20" texuniform="true"/>
    <texture name="wall_tex" type="2d" builtin="flat" width="1" height="1"
             rgb1=".88 .85 .80"/>
    <material name="wall_mat" texture="wall_tex"/>
  </asset>

  <include file="{robot_xml_path}"/>

  <worldbody>
    <light pos="5 -5 8" dir="-0.3 0.3 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>
    <light pos="-5 5 8" dir="0.3 -0.3 -1" diffuse="0.4 0.42 0.5" castshadow="false"/>
    <geom name="floor" type="plane" size="{terrain_size_str}" material="grid"
          conaffinity="1" condim="3" friction="{friction_str}" group="1"/>
{obstacle_xml}{goal_xml}{self.extra_xml_bodies}  </worldbody>
</mujoco>
"""


class SimWorld:
    """Runtime scene manager.

    Holds a WorldConfig and provides scene XML generation and obstacle management
    to the engine. Engines obtain initialization parameters via SimWorld.engine_config().
    """

    def __init__(self, config: WorldConfig) -> None:
        self.config = config
        self._dynamic_obstacles: List[ObstacleConfig] = []

    def get_scene_xml(self, robot_xml_path: str) -> str:
        """Get scene XML (including dynamic obstacles)."""
        merged = WorldConfig(
            scene_xml=self.config.scene_xml,
            terrain_type=self.config.terrain_type,
            terrain_size=self.config.terrain_size,
            gravity=self.config.gravity,
            timestep=self.config.timestep,
            floor_friction=self.config.floor_friction,
            obstacles=self.config.obstacles + self._dynamic_obstacles,
            shadow_quality=self.config.shadow_quality,
            ambient=self.config.ambient,
            offscreen_width=self.config.offscreen_width,
            offscreen_height=self.config.offscreen_height,
            goal_position=self.config.goal_position,
            extra_xml_bodies=self.config.extra_xml_bodies,
        )
        return merged.to_scene_xml(robot_xml_path)

    def add_obstacle(self, obs: ObstacleConfig) -> None:
        """Add a dynamic obstacle (takes effect on next reset)."""
        self._dynamic_obstacles.append(obs)

    def remove_obstacle(self, name: str) -> bool:
        """Remove a dynamic obstacle; returns True if found and removed."""
        before = len(self._dynamic_obstacles)
        self._dynamic_obstacles = [o for o in self._dynamic_obstacles if o.name != name]
        return len(self._dynamic_obstacles) < before

    def set_goal(self, position: List[float]) -> None:
        """Set navigation goal position (visualization marker)."""
        self.config.goal_position = position

    @property
    def all_obstacles(self) -> List[ObstacleConfig]:
        """All obstacles (static + dynamic)."""
        return self.config.obstacles + self._dynamic_obstacles
