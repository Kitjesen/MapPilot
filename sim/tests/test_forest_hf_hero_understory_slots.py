# ruff: noqa: S101

"""TDD contracts for conditioned understory assets in the review-only hero scene."""

from __future__ import annotations

import hashlib
import importlib
import os
from pathlib import Path
from types import ModuleType
from typing import Any, cast

import pytest


def _subject() -> ModuleType:
    return importlib.import_module(
        "sim.tools.worlds.forest_hf.blender_hero_diorama"
    )


def _tree_slots() -> dict[str, dict[str, object]]:
    return {
        semantic_class: {
            "path": f"conditioned/{semantic_class}.glb",
            "bytes": 100 + index,
            "sha256": str(index + 1) * 64,
            "embed_depth_m": 0.1,
        }
        for index, semantic_class in enumerate(("pine", "birch", "boulder"))
    }


def _write_understory_assets(root: Path) -> dict[str, dict[str, object]]:
    slots: dict[str, dict[str, object]] = {}
    for semantic_class in (
        "grass_clump",
        "fern_clump",
        "forest_floor_debris",
    ):
        payload = f"conditioned-understory:{semantic_class}".encode()
        relative_path = Path("conditioned") / f"{semantic_class}.glb"
        path = root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        slots[semantic_class] = {
            "path": relative_path.as_posix(),
            "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    return slots


def _plan(
    understory_asset_slots: dict[str, dict[str, object]] | None = None,
) -> dict[str, Any]:
    return cast(
        dict[str, Any],
        _subject().build_hero_diorama_plan(
            _tree_slots(),
            seed=5808,
            understory_asset_slots=understory_asset_slots,
        ),
    )


def test_review_plan_without_understory_slots_keeps_procedural_fallback() -> None:
    plan = _plan()

    assert plan["understory_asset_slots"] == {}


def test_review_plan_preserves_exact_conditioned_understory_identities(
    tmp_path: Path,
) -> None:
    slots = _write_understory_assets(tmp_path)

    plan = _plan(slots)

    assert plan["understory_asset_slots"] == slots


@pytest.mark.parametrize(
    "bad_path",
    ["C:/foreign/grass.glb", "/foreign/grass.glb", "../escaped.glb"],
)
def test_review_plan_rejects_nonportable_understory_paths(bad_path: str) -> None:
    slots = {
        semantic_class: {
            "path": f"conditioned/{semantic_class}.glb",
            "bytes": 12,
            "sha256": "a" * 64,
        }
        for semantic_class in (
            "grass_clump",
            "fern_clump",
            "forest_floor_debris",
        )
    }
    slots["grass_clump"]["path"] = bad_path

    with pytest.raises(ValueError, match=r"relative|portable|contain|path"):
        _plan(slots)


def test_review_asset_loader_rejects_understory_byte_mismatch(tmp_path: Path) -> None:
    slots = _write_understory_assets(tmp_path)
    slots["fern_clump"]["bytes"] = int(slots["fern_clump"]["bytes"]) + 1

    with pytest.raises(ValueError, match=r"bytes|identity"):
        _subject()._load_understory_assets(tmp_path, slots)


def test_review_asset_loader_rejects_understory_sha256_mismatch(
    tmp_path: Path,
) -> None:
    slots = _write_understory_assets(tmp_path)
    slots["forest_floor_debris"]["sha256"] = "0" * 64

    with pytest.raises(ValueError, match=r"sha256|identity"):
        _subject()._load_understory_assets(tmp_path, slots)


def test_review_asset_loader_rejects_understory_symlink(tmp_path: Path) -> None:
    slots = _write_understory_assets(tmp_path)
    target = tmp_path / "conditioned" / "grass_clump.glb"
    link = tmp_path / "conditioned" / "grass-link.glb"
    try:
        os.symlink(target, link)
    except OSError as exc:
        pytest.skip(f"symlinks unavailable on this host: {exc}")
    slots["grass_clump"] = {
        "path": "conditioned/grass-link.glb",
        "bytes": target.stat().st_size,
        "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
    }

    with pytest.raises(ValueError, match=r"link|regular|reparse"):
        _subject()._load_understory_assets(tmp_path, slots)


def test_conditioned_understory_templates_replace_grass_and_fern() -> None:
    procedural = {
        "pine": object(),
        "birch": object(),
        "boulder": object(),
        "grass": object(),
        "fern": object(),
    }
    conditioned = {
        "grass_clump": object(),
        "fern_clump": object(),
        "forest_floor_debris": object(),
    }

    selected = _subject().select_hero_understory_templates(
        procedural,
        conditioned,
    )

    assert selected["grass"] is conditioned["grass_clump"]
    assert selected["fern"] is conditioned["fern_clump"]
    assert selected["forest_floor_debris"] is conditioned["forest_floor_debris"]


def test_missing_understory_slots_leave_procedural_grass_and_fern_unchanged() -> None:
    procedural = {
        "pine": object(),
        "birch": object(),
        "boulder": object(),
        "grass": object(),
        "fern": object(),
    }

    selected = _subject().select_hero_understory_templates(procedural, {})

    assert selected["grass"] is procedural["grass"]
    assert selected["fern"] is procedural["fern"]
    assert "forest_floor_debris" not in selected


def test_conditioned_debris_is_sparse_and_does_not_replace_ground_detail() -> None:
    plan = _plan(
        {
            semantic_class: {
                "path": f"conditioned/{semantic_class}.glb",
                "bytes": 12,
                "sha256": "a" * 64,
            }
            for semantic_class in (
                "grass_clump",
                "fern_clump",
                "forest_floor_debris",
            )
        }
    )
    debris = [
        item
        for item in plan["dressing"]
        if item["kind"] == "forest_floor_debris"
    ]

    assert 12 <= len(debris) <= 96
    assert _subject().hero_ground_detail_counts()["wet_leaves"] > 0


def test_conditioned_understory_remains_visual_only_and_mujoco_authoritative() -> None:
    plan = _plan(
        {
            semantic_class: {
                "path": f"conditioned/{semantic_class}.glb",
                "bytes": 12,
                "sha256": "a" * 64,
            }
            for semantic_class in (
                "grass_clump",
                "fern_clump",
                "forest_floor_debris",
            )
        }
    )
    understory = [
        item
        for item in plan["dressing"]
        if item["kind"] in {"grass", "fern", "forest_floor_debris"}
    ]

    assert plan["runtime_contract"] == {
        "classification": "VisualOnly",
        "collision_profile": "NoCollision",
        "physics_authority": "MuJoCo",
    }
    assert all(item["classification"] == "VisualOnly" for item in understory)
    assert all(item["collision_profile"] == "NoCollision" for item in understory)
    assert all(item["collision"] is False for item in understory)
    assert all(item["simulate_physics"] is False for item in understory)


def test_understory_review_outputs_exclude_production_assets() -> None:
    outputs = _plan()["outputs"]

    assert [output["filename"] for output in outputs] == [
        "forest_hf.hero-diorama.png",
        "forest_hf.hero-diorama.review.json",
    ]
    assert not any(
        output["filename"].lower().endswith((".blend", ".fbx", ".glb", ".gltf"))
        for output in outputs
    )
