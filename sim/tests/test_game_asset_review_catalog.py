"""Contract tests for the deterministic game asset review catalog."""

from __future__ import annotations

import concurrent.futures
import copy
import hashlib
import importlib
import json
import os
from pathlib import Path
from typing import Any

import pytest


def _module() -> Any:
    return importlib.import_module("sim.tools.game_asset_review_catalog")


def _pin(root: Path, relative: str, payload: bytes) -> dict[str, Any]:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    return {
        "role": "source_model",
        "path": relative,
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def _card(
    root: Path,
    card_id: str,
    *,
    order: int,
    stage: str = "source_review",
    disposition: str = "quarantined",
) -> dict[str, Any]:
    if stage == "unavailable":
        evidence: list[dict[str, Any]] = []
    else:
        first = _pin(root, f"evidence/{card_id}.glb", card_id.encode())
        if stage == "catalog_review":
            first["role"] = "package_manifest"
            second = _pin(root, f"evidence/{card_id}.json", b"{}")
            second["role"] = "visual_projection"
            evidence = [first, second]
        elif stage == "conditioned_review":
            first["role"] = "conditioned_mesh"
            second = _pin(root, f"evidence/{card_id}.json", b"{}")
            second["role"] = "conditioning_report"
            evidence = [first, second]
        elif stage == "proxy_only":
            first["role"] = "physics_proxy"
            evidence = [first]
        else:
            second = _pin(root, f"evidence/{card_id}.json", b"{}")
            second["role"] = "source_task"
            evidence = [first, second]
    return {
        "id": card_id,
        "title": card_id.replace("_", " ").title(),
        "description": f"Review {card_id}",
        "order": order,
        "asset_class": "movable_object" if stage == "unavailable" else "vegetation",
        "review": {
            "stage": stage,
            "disposition": disposition,
            "reason": "Not qualified for runtime use",
        },
        "evidence": evidence,
        "tags": ["review", card_id],
    }


def _spec(*cards: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.game-asset-review-spec.v1",
        "title": "Asset Review",
        "cards": list(cards),
    }


def _digest(document: dict[str, Any]) -> str:
    body = {key: value for key, value in document.items() if key != "digest"}
    payload = json.dumps(
        body,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def test_catalog_is_canonical_and_pins_exact_evidence(tmp_path: Path) -> None:
    module = _module()
    late = _card(tmp_path, "forest_grass", order=8)
    early = _card(
        tmp_path,
        "forest_birch",
        order=2,
        stage="conditioned_review",
    )

    catalog = module.build_game_asset_review_catalog(
        tmp_path,
        spec=_spec(late, early),
    )

    assert catalog["schema"] == "lingtu.sim.game-asset-review-catalog.v1"
    assert [card["id"] for card in catalog["cards"]] == [
        "forest_birch",
        "forest_grass",
    ]
    assert catalog["digest"] == _digest(catalog)
    evidence = catalog["cards"][0]["evidence"]
    assert [item["role"] for item in evidence] == [
        "conditioned_mesh",
        "conditioning_report",
    ]
    for item in evidence:
        payload = (tmp_path / item["path"]).read_bytes()
        assert item["bytes"] == len(payload)
        assert item["sha256"] == hashlib.sha256(payload).hexdigest()


def test_catalog_hard_codes_presentation_only_authority(tmp_path: Path) -> None:
    module = _module()
    catalog = module.build_game_asset_review_catalog(
        tmp_path,
        spec=_spec(_card(tmp_path, "forest_grass", order=0)),
    )

    assert catalog["policy"] == {
        "purpose": "presentation_review_only",
        "runnable": False,
        "qualified_visual": False,
        "unreal_collision_profile": "NoCollision",
        "unreal_simulate_physics": False,
        "physics_authority": "mujoco",
    }
    assert catalog["cards"][0]["capabilities"] == {
        "selectable_for_review": True,
        "runnable": False,
        "qualified_visual": False,
    }


def test_unavailable_card_records_no_qualified_movable_visual(tmp_path: Path) -> None:
    module = _module()
    unavailable = _card(
        tmp_path,
        "movable_object_visual_unavailable",
        order=9,
        stage="unavailable",
        disposition="unavailable",
    )

    catalog = module.build_game_asset_review_catalog(
        tmp_path,
        spec=_spec(unavailable),
    )

    assert catalog["coverage"]["qualified_movable_object_visual"] == {
        "available": False,
        "reason": "no qualified movable-object visual is registered",
    }
    assert catalog["cards"][0]["evidence"] == []
    assert catalog["cards"][0]["capabilities"]["selectable_for_review"] is False


def test_default_catalog_contains_the_required_review_cards() -> None:
    module = _module()
    repo_root = Path(__file__).resolve().parents[2]

    catalog = module.build_game_asset_review_catalog(repo_root)
    by_id = {card["id"]: card for card in catalog["cards"]}

    assert set(by_id) == {
        "forest_birch",
        "forest_boulder",
        "forest_fern",
        "forest_grass",
        "forest_litter",
        "forest_pine",
        "movable_object_visual_unavailable",
        "pedestrian_capsule",
        "rws_01",
        "thunder_v4",
    }
    forest_ids = {
        "forest_birch",
        "forest_pine",
        "forest_boulder",
        "forest_grass",
        "forest_fern",
        "forest_litter",
    }
    assert {
        (by_id[card_id]["review"]["stage"], by_id[card_id]["review"]["disposition"])
        for card_id in forest_ids
    } == {("unavailable", "unavailable")}
    assert all(by_id[card_id]["evidence"] == [] for card_id in forest_ids)
    assert by_id["pedestrian_capsule"]["review"]["stage"] == "proxy_only"
    assert all(not card["capabilities"]["runnable"] for card in by_id.values())


@pytest.mark.parametrize(
    ("mutator", "message"),
    [
        (
            lambda spec: spec["cards"][0]["evidence"][0].__setitem__("bytes", 999),
            "byte length",
        ),
        (
            lambda spec: spec["cards"][0]["evidence"][0].__setitem__(
                "sha256", "0" * 64
            ),
            "sha256",
        ),
        (
            lambda spec: spec["cards"][0]["evidence"][0].__setitem__(
                "path", "../escape.glb"
            ),
            "repository-relative",
        ),
    ],
)
def test_catalog_rejects_untrusted_evidence(
    tmp_path: Path,
    mutator: Any,
    message: str,
) -> None:
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    mutator(spec)

    with pytest.raises(module.GameAssetReviewCatalogError, match=message):
        module.build_game_asset_review_catalog(tmp_path, spec=spec)


def test_catalog_rejects_source_drift_after_pin(tmp_path: Path) -> None:
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    source = tmp_path / spec["cards"][0]["evidence"][0]["path"]
    source.write_bytes(b"changed after review")

    with pytest.raises(module.GameAssetReviewCatalogError, match=r"byte length|sha256"):
        module.build_game_asset_review_catalog(tmp_path, spec=spec)


def test_catalog_rejects_invalid_stage_contracts(tmp_path: Path) -> None:
    module = _module()
    invalid = _card(
        tmp_path,
        "forest_birch",
        order=0,
        stage="conditioned_review",
    )
    invalid["evidence"] = invalid["evidence"][:1]

    with pytest.raises(module.GameAssetReviewCatalogError, match="evidence roles"):
        module.build_game_asset_review_catalog(tmp_path, spec=_spec(invalid))


def test_write_is_idempotent_but_refuses_conflicting_output(tmp_path: Path) -> None:
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    output = Path("out/catalog.json")

    first = module.write_game_asset_review_catalog(tmp_path, output, spec=spec)
    second = module.write_game_asset_review_catalog(tmp_path, output, spec=spec)
    assert first == second

    changed = copy.deepcopy(spec)
    changed["title"] = "Changed"
    with pytest.raises(module.GameAssetReviewCatalogError, match="conflicting"):
        module.write_game_asset_review_catalog(tmp_path, output, spec=changed)


def test_concurrent_identical_publish_is_idempotent(tmp_path: Path) -> None:
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    output = Path("out/catalog.json")

    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
        results = list(
            executor.map(
                lambda _: module.write_game_asset_review_catalog(
                    tmp_path, output, spec=spec
                ),
                range(24),
            )
        )

    assert results == [tmp_path / output] * 24
    assert json.loads((tmp_path / output).read_text(encoding="utf-8"))["digest"]


def test_concurrent_conflicting_publish_never_replaces_winner(tmp_path: Path) -> None:
    module = _module()
    first = _spec(_card(tmp_path, "forest_grass", order=0))
    second = copy.deepcopy(first)
    second["title"] = "Different catalog"
    output = Path("out/catalog.json")

    def publish(spec: dict[str, Any]) -> tuple[str, str]:
        try:
            module.write_game_asset_review_catalog(tmp_path, output, spec=spec)
            return ("published", spec["title"])
        except module.GameAssetReviewCatalogError as exc:
            return ("conflict", str(exc))

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(publish, (first, second)))

    assert sorted(result[0] for result in results) == ["conflict", "published"]
    observed = json.loads((tmp_path / output).read_text(encoding="utf-8"))
    assert observed["title"] in {first["title"], second["title"]}


def test_catalog_rejects_oversized_spec_before_yaml_parse(tmp_path: Path) -> None:
    module = _module()
    spec_path = tmp_path / "oversized.yaml"
    spec_path.write_bytes(b"x" * (module.MAX_SPEC_BYTES + 1))

    with pytest.raises(module.GameAssetReviewCatalogError, match="size limit"):
        module.build_game_asset_review_catalog(
            tmp_path, spec=Path("oversized.yaml")
        )


def test_catalog_rejects_too_many_cards(tmp_path: Path) -> None:
    module = _module()
    card = _card(
        tmp_path,
        "movable_object_visual_unavailable",
        order=0,
        stage="unavailable",
        disposition="unavailable",
    )
    cards = []
    for index in range(module.MAX_CARDS + 1):
        item = copy.deepcopy(card)
        item["id"] = f"unavailable_{index}"
        item["order"] = index
        cards.append(item)

    with pytest.raises(module.GameAssetReviewCatalogError, match="too many cards"):
        module.build_game_asset_review_catalog(tmp_path, spec=_spec(*cards))


def test_catalog_rejects_evidence_larger_than_limit_without_reading_it(
    tmp_path: Path,
) -> None:
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    evidence = spec["cards"][0]["evidence"][0]
    source = tmp_path / evidence["path"]
    with source.open("wb") as stream:
        stream.truncate(module.MAX_EVIDENCE_BYTES + 1)
    evidence["bytes"] = module.MAX_EVIDENCE_BYTES + 1

    with pytest.raises(module.GameAssetReviewCatalogError, match="size limit"):
        module.build_game_asset_review_catalog(tmp_path, spec=spec)


def test_catalog_binds_hash_and_size_to_one_open_evidence_handle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    if os.name == "nt":
        pytest.skip("Windows no-delete sharing intentionally blocks path replacement")
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    source = tmp_path / spec["cards"][0]["evidence"][0]["path"]
    original_open = module._open_regular_nofollow

    def open_then_replace(
        path: Path,
        *,
        context: str,
        parent_descriptor: int | None = None,
    ) -> int:
        descriptor = original_open(
            path,
            context=context,
            parent_descriptor=parent_descriptor,
        )
        replacement = source.with_suffix(".replacement")
        replacement.write_bytes(b"different")
        os.replace(replacement, source)
        return int(descriptor)

    monkeypatch.setattr(module, "_open_regular_nofollow", open_then_replace)

    catalog = module.build_game_asset_review_catalog(tmp_path, spec=spec)
    evidence = catalog["cards"][0]["evidence"][0]
    assert evidence["bytes"] == len(b"forest_grass")
    assert evidence["sha256"] == hashlib.sha256(b"forest_grass").hexdigest()


def test_catalog_opens_evidence_from_stable_parent_after_path_swap(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _module()
    spec = _spec(_card(tmp_path, "forest_grass", order=0))
    model = spec["cards"][0]["evidence"][0]
    task = spec["cards"][0]["evidence"][1]
    original_model = (tmp_path / model["path"]).read_bytes()
    original_task = (tmp_path / task["path"]).read_bytes()
    model["path"] = "stable/model/model.glb"
    task["path"] = "stable/task/task.json"
    model_path = tmp_path / model["path"]
    task_path = tmp_path / task["path"]
    model_path.parent.mkdir(parents=True)
    task_path.parent.mkdir(parents=True)
    model_path.write_bytes(original_model)
    task_path.write_bytes(original_task)
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / model_path.name).write_bytes(b"attacker-controlled")
    original_open = module._open_regular_nofollow
    swapped = False
    replacement_denied = False

    def swap_parent_then_open(
        path: Path,
        *,
        context: str,
        parent_descriptor: int | None = None,
    ) -> int:
        nonlocal replacement_denied, swapped
        if path == model_path and not swapped:
            held = model_path.parent.with_name("model-held")
            try:
                model_path.parent.rename(held)
            except OSError:
                replacement_denied = True
            else:
                swapped = True
                model_path.parent.symlink_to(outside, target_is_directory=True)
        return int(
            original_open(
                path,
                context=context,
                parent_descriptor=parent_descriptor,
            )
        )

    monkeypatch.setattr(module, "_open_regular_nofollow", swap_parent_then_open)

    catalog = module.build_game_asset_review_catalog(tmp_path, spec=spec)

    observed = catalog["cards"][0]["evidence"][0]
    assert observed["sha256"] == hashlib.sha256(original_model).hexdigest()
    assert swapped or replacement_denied


def test_global_evidence_budget_rejects_before_any_hash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _module()
    cards = [_card(tmp_path, f"forest_{index}", order=index) for index in range(5)]
    for card in cards:
        for evidence in card["evidence"]:
            evidence["bytes"] = module.MAX_EVIDENCE_BYTES
    hash_calls = 0

    def count_hash(*args: Any, **kwargs: Any) -> str:
        nonlocal hash_calls
        hash_calls += 1
        return "0" * 64

    monkeypatch.setattr(module, "_hash_regular_bounded", count_hash)

    with pytest.raises(module.GameAssetReviewCatalogError, match="size limit"):
        module.build_game_asset_review_catalog(tmp_path, spec=_spec(*cards))
    assert hash_calls == 0


def test_write_rejects_symlinked_output_parent_when_supported(tmp_path: Path) -> None:
    module = _module()
    outside = tmp_path.parent / f"{tmp_path.name}-outside"
    outside.mkdir()
    linked = tmp_path / "out"
    try:
        linked.symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("directory symlink creation is unavailable")
    spec = _spec(_card(tmp_path, "forest_grass", order=0))

    with pytest.raises(module.GameAssetReviewCatalogError, match=r"link|reparse"):
        module.write_game_asset_review_catalog(
            tmp_path, Path("out/catalog.json"), spec=spec
        )
    assert not (outside / "catalog.json").exists()


def test_catalog_rejects_symlinked_evidence_when_supported(tmp_path: Path) -> None:
    module = _module()
    target = tmp_path / "target.glb"
    target.write_bytes(b"target")
    link = tmp_path / "evidence" / "link.glb"
    link.parent.mkdir(parents=True)
    try:
        link.symlink_to(target)
    except OSError:
        pytest.skip("symlink creation is unavailable")
    evidence = {
        "role": "source_model",
        "path": "evidence/link.glb",
        "bytes": len(target.read_bytes()),
        "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
    }
    task = _pin(tmp_path, "evidence/task.json", b"{}")
    task["role"] = "source_task"
    card = _card(tmp_path, "unused", order=0)
    card["id"] = "linked"
    card["evidence"] = [evidence, task]

    with pytest.raises(module.GameAssetReviewCatalogError, match=r"link|reparse"):
        module.build_game_asset_review_catalog(tmp_path, spec=_spec(card))
