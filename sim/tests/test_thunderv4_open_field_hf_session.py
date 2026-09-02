from __future__ import annotations

from pathlib import Path

import yaml

from sim.catalog import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_open_field_hf"
    / "session.yaml"
)


def test_thunderv4_open_field_hf_compiles_one_consistent_world() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)

    assert resolved.session == yaml.safe_load(SESSION.read_text(encoding="utf-8"))
    assert resolved.visual_plan["world"] == {
        "binding": "WorldVisual:OpenFieldHF",
        "level": "/Game/RobotSim/Maps/OpenFieldRuntime",
        "package": {
            "id": "open_field_hf",
            "kind": "world",
            "manifest": "sim/packages/worlds/open_field_hf/1.1.0/world.package.yaml",
            "version": "1.1.0",
        },
    }
    assert (
        resolved.physics_plan["world"]["mjcf"]
        == "sim/packages/worlds/open_field_hf/1.1.0/physics/open_field_hf.xml"
    )
    assert resolved.physics_plan["robots"][0]["spawn"] == {
        "position_m": [0.0, 0.0, 0.0],
        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    assert resolved.session["runtime"]["required_bindings"] == [
        "physics",
        "visual",
        "sensors",
        "control",
    ]
