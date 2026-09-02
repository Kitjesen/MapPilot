# ruff: noqa: S101
"""End-to-end world import and Catalog promotion contracts."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import cast

import pytest
import yaml

from sim.catalog.resolver import CatalogResolver
from sim.catalog.importers import CatalogPromoter, ImportFailure, WorldImporter
from sim.tests.test_world_importer import _request, _source_tree


def _copy_resolvable_robot_fixture(repo_root: Path) -> None:
    source_root = Path(__file__).resolve().parents[2]
    relative = "sim/packages/robots/omni_cart"
    shutil.copytree(source_root / relative, repo_root / relative)


def test_world_import_promotes_with_catalog_identity(tmp_path: Path) -> None:
    draft = WorldImporter(tmp_path).import_world(_request(_source_tree(tmp_path)), draft_root=tmp_path / "draft")

    result = CatalogPromoter(tmp_path).promote(draft)

    assert result.package_root.is_dir()
    assert result.qualification_path.is_file()
    qualification = json.loads(result.qualification_path.read_text(encoding="utf-8"))
    assert qualification["package"] == {"kind": "world", "id": "field", "version": "1.0.0"}
    assert qualification["provenance"]["path"] == "provenance/world.provenance.json"


def test_promoted_world_projection_is_present_in_the_resolved_visual_plan(tmp_path: Path) -> None:
    draft = WorldImporter(tmp_path).import_world(_request(_source_tree(tmp_path)), draft_root=tmp_path / "draft")
    result = CatalogPromoter(tmp_path).promote(draft)
    _copy_resolvable_robot_fixture(tmp_path)
    session_path = tmp_path / "session.yaml"
    session_path.write_text(
        yaml.safe_dump(
            {
                "schema": "lingtu.sim.session.v1",
                "session_id": "imported_world_projection",
                "mujoco_version": "3.10.0",
                "seed": 20260808,
                "world": "field@1.0.0",
                "robots": [
                    {
                        "instance_id": "cart_01",
                        "package": "omni_cart@1.0.0",
                        "controller": None,
                        "sensor_rig": None,
                        "spawn": {
                            "position_m": [0.0, 0.0, 0.0],
                            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                        },
                    }
                ],
                "runtime": {
                    "backend": "mujoco",
                    "mode": "headless",
                    "required_bindings": ["physics", "visual"],
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    resolved = CatalogResolver.from_repository(tmp_path).resolve(session_path)

    projection = resolved.visual_plan["world"]["projection"]
    assert (
        projection["path"]
        == result.package_root.relative_to(tmp_path).joinpath("visual/world.visual-projection.json").as_posix()
    )


def test_world_import_promotion_rejects_an_unknown_qualification_field(tmp_path: Path) -> None:
    draft = WorldImporter(tmp_path).import_world(_request(_source_tree(tmp_path)), draft_root=tmp_path / "draft")
    qualification_path = cast(Path, draft.qualification_path)
    qualification = json.loads(qualification_path.read_text(encoding="utf-8"))
    qualification["package"]["unused"] = True
    qualification_path.write_text(json.dumps(qualification, sort_keys=True), encoding="utf-8")

    with pytest.raises(ImportFailure, match=r"invalid fields"):
        CatalogPromoter(tmp_path).promote(draft)

    assert not (tmp_path / "sim" / "packages" / "worlds" / "field").exists()
    assert not (tmp_path / "sim" / "evaluation" / "package_qualifications" / "world" / "field").exists()
