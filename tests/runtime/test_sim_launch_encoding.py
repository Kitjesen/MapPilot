import pytest

pytestmark = [pytest.mark.sim]

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PRODUCT_MUJOCO_WORLD = (
    REPO_ROOT / "sim" / "packages" / "worlds" / "industrial_park" / "physics" / "industrial_park_scene.xml"
)


def _read_strip_bom(path: Path) -> str:
    """Read file, stripping UTF-8 BOM if present."""
    raw = path.read_bytes()
    if raw[:3] == b"\xef\xbb\xbf":
        return raw[3:].decode("utf-8")
    return raw.decode("utf-8")


def test_product_mujoco_world_text_is_ascii_to_avoid_mojibake() -> None:
    text = _read_strip_bom(PRODUCT_MUJOCO_WORLD)

    assert text.isascii()
