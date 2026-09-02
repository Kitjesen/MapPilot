# ruff: noqa: S101

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "sim"
PLUGIN = ROOT / "runtime" / "visual" / "RobotSimUE" / "Plugins" / "LingTuSim"
FRONT_END = PLUGIN / "Resources" / "FrontEnd"
BUILD_RULES = PLUGIN / "Source" / "LingTuSimUI" / "LingTuSimUI.Build.cs"

EXPECTED_FRONT_END_PNGS = {
    "lingtu-field-ops-hero-v1.png",
    "preview-forest-birch-v1.png",
    "preview-forest-boulder-v1.png",
    "preview-forest-pine-v1.png",
    "preview-rws-01-v1.png",
    "preview-thunder-v4-v1.png",
}


def test_versioned_front_end_pngs_exist() -> None:
    assert {path.name for path in FRONT_END.glob("*.png")} == EXPECTED_FRONT_END_PNGS
    for filename in EXPECTED_FRONT_END_PNGS:
        resource = FRONT_END / filename
        assert resource.is_file()
        assert resource.stat().st_size > 0


def test_build_rules_stage_each_front_end_png_explicitly_as_non_ufs() -> None:
    build_rules = BUILD_RULES.read_text(encoding="utf-8")

    private_dependencies = build_rules.split(
        "PrivateDependencyModuleNames.AddRange", 1
    )[1].split(");", 1)[0]
    assert '"Projects"' in private_dependencies
    assert 'Path.Combine(PluginDirectory, "Resources", "FrontEnd", "*.png")' not in build_rules

    staged_pngs = set(
        re.findall(
            r'RuntimeDependencies\.Add\(\s*'
            r'Path\.Combine\(PluginDirectory, "Resources", "FrontEnd", "([^"]+\.png)"\),\s*'
            r'StagedFileType\.NonUFS\s*\);',
            build_rules,
        )
    )
    assert staged_pngs == EXPECTED_FRONT_END_PNGS
    assert build_rules.count("StagedFileType.NonUFS") == len(EXPECTED_FRONT_END_PNGS)
