import pytest

pytestmark = [pytest.mark.sim]

import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_NAV_LAUNCH = REPO_ROOT / "sim" / "planning" / "sim_navigation.launch.py"
PRODUCT_MUJOCO_WORLD = REPO_ROOT / "sim" / "worlds" / "mujoco" / "industrial_park_scene.xml"


def _read_strip_bom(path: Path) -> str:
    """Read file, stripping UTF-8 BOM if present."""
    raw = path.read_bytes()
    if raw[:3] == b"\xef\xbb\xbf":
        return raw[3:].decode("utf-8")
    return raw.decode("utf-8")


def test_sim_navigation_launch_text_is_ascii_to_avoid_mojibake() -> None:
    text = _read_strip_bom(SIM_NAV_LAUNCH)

    assert text.isascii()


def test_product_mujoco_world_text_is_ascii_to_avoid_mojibake() -> None:
    text = _read_strip_bom(PRODUCT_MUJOCO_WORLD)

    assert text.isascii()


def test_sim_navigation_launch_keeps_path_follower_enabled() -> None:
    text = SIM_NAV_LAUNCH.read_text(encoding="utf-8")

    assert re.search(r"^\s*local_planner_node,", text, re.MULTILINE)
    assert re.search(r"^\s*path_follower_node,", text, re.MULTILINE)
