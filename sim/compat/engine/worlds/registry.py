"""
Simulation world registry

Manages available simulation scene XMLs, supports path lookup and alias registration.
"""
from pathlib import Path
from typing import Dict, Optional

_LINGTU_ROOT = Path(__file__).resolve().parents[4]
_SIM_WORLDS = Path(__file__).resolve().parent
_PACKAGE_WORLDS = _LINGTU_ROOT / "sim" / "packages" / "worlds"

# Inline minimal scene (empty world)
_EMPTY_WORLD_XML = """\
<mujoco model="empty">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.5 0.5 0.5"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".85 .85 .82" rgb2=".7 .7 .68"/>
    <material name="grid" texture="grid" texrepeat="20 20" texuniform="true"/>
  </asset>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>
    <geom name="floor" type="plane" size="50 50 0.1" material="grid"
          conaffinity="1" condim="3" friction="1 0.5 0.5"/>
  </worldbody>
</mujoco>
"""


class WorldRegistry:
    """World scene registry.

    Entry format: name -> {"path": Path | None, "xml": str | None, "description": str}
    Provide either path or xml: path takes priority.
    """

    def __init__(self):
        self._worlds: Dict[str, dict] = {}
        self._register_defaults()

    def _register_defaults(self):
        """Register built-in scenes."""
        # factory — industrial facility scene
        self.register(
            name="factory",
            path=_SIM_WORLDS / "factory_scene.xml",
            description="Factory building scene (machinery, gates, corridors)",
        )
        # open_field — open flat ground
        self.register(
            name="open_field",
            path=_PACKAGE_WORLDS / "open_field" / "physics" / "open_field.xml",
            description="Open flat scene (no obstacles, for basic navigation tests)",
        )
        # building — corridor scene
        self.register(
            name="building",
            path=_PACKAGE_WORLDS / "building" / "physics" / "building_scene.xml",
            description="Corridor scene (narrow passage test)",
        )
        # empty — flat ground only, inline XML
        self.register(
            name="empty",
            xml=_EMPTY_WORLD_XML,
            description="Empty flat scene (minimal test environment)",
        )

    def register(
        self,
        name: str,
        path: Optional[Path] = None,
        xml: Optional[str] = None,
        description: str = "",
    ):
        """Register a scene.

        Args:
            name:        unique scene identifier
            path:        MJCF XML file path (optional)
            xml:         inline XML string (optional, used if path is unavailable)
            description: scene description
        """
        if path is None and xml is None:
            raise ValueError(f"world '{name}': must provide path or xml")
        self._worlds[name] = {"path": path, "xml": xml, "description": description}

    def get(self, name: str) -> dict:
        """Get scene info.

        Returns:
            {"path": Path|None, "xml": str|None, "description": str}

        Raises:
            KeyError: scene not found
        """
        if name not in self._worlds:
            available = ", ".join(sorted(self._worlds.keys()))
            raise KeyError(
                f"World '{name}' not found. Available: {available}"
            )
        return self._worlds[name]

    def get_xml_or_path(self, name: str) -> tuple:
        """Return (xml_string_or_None, path_or_None); caller uses whichever is set.

        Returns file path (if file exists), otherwise returns inline XML.

        Returns:
            (xml: str | None, path: Path | None)
        """
        info = self.get(name)
        path = info.get("path")
        xml = info.get("xml")

        if path is not None and Path(path).exists():
            return None, Path(path)
        if xml is not None:
            return xml, None
        # path does not exist -> return None, None for caller to handle
        return None, path

    def list(self) -> Dict[str, str]:
        """Return {name: description} dict."""
        return {k: v["description"] for k, v in self._worlds.items()}


# Module-level singleton
_registry = WorldRegistry()


def get_world(name: str) -> dict:
    """Get scene info (module-level function)."""
    return _registry.get(name)


def list_worlds() -> Dict[str, str]:
    """List all registered scenes (module-level function)."""
    return _registry.list()
