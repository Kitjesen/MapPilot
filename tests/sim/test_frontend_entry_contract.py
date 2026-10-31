"""Normal game startup must select the product front end, not a model exhibit."""

import configparser
import json
from pathlib import Path


def test_default_game_map_is_the_dedicated_front_end():
    project = Path(__file__).resolve().parents[2] / "sim/runtime/visual/RobotSimUE"
    config = configparser.RawConfigParser(strict=False)
    config.read(project / "Config/DefaultEngine.ini", encoding="utf-8")
    settings = config["/Script/EngineSettings.GameMapsSettings"]
    assert settings["GameDefaultMap"] == "/Game/RobotSim/Maps/LingTuFrontEnd"
    assert "RuntimePreview" not in settings["GameDefaultMap"]


def test_shipping_map_selection_includes_the_new_entry_not_the_retired_exhibit():
    root = Path(__file__).resolve().parents[2]
    config = configparser.RawConfigParser(strict=False)
    config.read(root / "sim/runtime/visual/RobotSimUE/Config/DefaultEngine.ini", encoding="utf-8")
    default_map = config["/Script/EngineSettings.GameMapsSettings"]["GameDefaultMap"]
    policy = json.loads((root / "sim/distribution/windows/policy.v1.json").read_text(encoding="utf-8"))
    assert default_map in policy["product"]["maps"]
    assert not any("RuntimePreview" in path for path in policy["product"]["maps"])
