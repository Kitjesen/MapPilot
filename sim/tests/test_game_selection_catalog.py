"""Contract tests for the deterministic game selection catalog compiler."""

from __future__ import annotations

import copy
import hashlib
import importlib
import json
from pathlib import Path
from typing import Any

import pytest


def _module() -> Any:
    return importlib.import_module("sim.tools.game_selection_catalog")


def _canonical_digest(document: dict[str, Any]) -> str:
    body = {key: value for key, value in document.items() if key != "digest"}
    payload = json.dumps(
        body,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _selection_entry(
    entry_id: str,
    *,
    order: int,
    state: str = "runnable",
    session_spec: str | None = None,
) -> dict[str, Any]:
    return {
        "id": entry_id,
        "title": f"Choice {entry_id}",
        "description": f"Run {entry_id}",
        "order": order,
        "session_spec": session_spec or f"sim/sessions/examples/{entry_id}/session.yaml",
        "availability": {
            "state": state,
            "reason": "Ready" if state == "runnable" else f"{state} asset review pending",
        },
        "presentation": {
            "robot": {"id": "thunderv4", "version": "1.0.3", "label": "Thunder V4"},
            "world": {
                "id": "forest_hf",
                "version": "2.0.0",
                "label": "High-fidelity Forest",
            },
            "scenario": {
                "id": "forest_patrol",
                "version": "1.1.0",
                "label": "Forest Patrol",
            },
            "mode": "unreal",
        },
        "tags": ["forest", "robot"],
    }


def _selection_spec(*entries: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.game-selection-spec.v1",
        "title": "LingTu Simulation Choices",
        "entries": list(entries),
    }


class _Resolved:
    def __init__(
        self,
        entry_id: str,
        *,
        write_files: bool = True,
    ) -> None:
        self.session_id = entry_id
        self._write_files = write_files
        self.write_calls: list[Path] = []

    def write_bundle(self, bundle_dir: Path) -> Path:
        bundle_dir = Path(bundle_dir)
        self.write_calls.append(bundle_dir)
        if self._write_files:
            bundle_dir.mkdir(parents=True, exist_ok=True)
            for filename in (
                "physics.plan.json",
                "visual.plan.json",
                "sensor.plan.json",
                "control.plan.json",
                "transport.intent.json",
            ):
                (bundle_dir / filename).write_text("{}\n", encoding="utf-8")
        return bundle_dir


class _RecordingResolver:
    repository_calls: list[Path] = []
    resolve_calls: list[Path] = []
    resolved: dict[str, _Resolved] = {}

    @classmethod
    def reset(cls) -> None:
        cls.repository_calls = []
        cls.resolve_calls = []
        cls.resolved = {}

    @classmethod
    def from_repository(cls, repo_root: Path) -> _RecordingResolver:
        cls.repository_calls.append(Path(repo_root))
        return cls()

    def resolve(self, session_spec: Path) -> _Resolved:
        path = Path(session_spec)
        self.resolve_calls.append(path)
        entry_id = path.parent.name
        resolved = self.resolved.setdefault(entry_id, _Resolved(entry_id))
        return resolved


@pytest.fixture
def catalog_module(monkeypatch: pytest.MonkeyPatch) -> Any:
    module = _module()
    _RecordingResolver.reset()
    monkeypatch.setattr(module, "CatalogResolver", _RecordingResolver)
    return module


def _write_session_specs(repo_root: Path, *entry_ids: str) -> None:
    for entry_id in entry_ids:
        path = repo_root / "sim" / "sessions" / "examples" / entry_id / "session.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            f"schema: lingtu.sim.session.v1\nsession_id: {entry_id}\n",
            encoding="utf-8",
        )


def _asset_library(path: Path) -> Path:
    body: dict[str, Any] = {
        "schema": "lingtu.sim.ue-asset-library.v1",
        "entries": [
            {"entry_id": "robot:thunderv4@1.0.3", "availability": "CATALOG_PACKAGE"},
            {"entry_id": "world:forest_hf@2.0.0", "availability": "CATALOG_PACKAGE"},
            {"entry_id": "candidate:grass", "availability": "SOURCE_CANDIDATE"},
            {"entry_id": "candidate:fern", "availability": "QUARANTINED"},
            {"entry_id": "candidate:debris", "availability": "UNVERIFIED"},
        ],
    }
    document = {**body, "digest": hashlib.sha256(
        json.dumps(body, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def _asset_review_catalog(
    path: Path, *, reason: str = "Awaiting UE visual review"
) -> Path:
    body: dict[str, Any] = {
        "schema": "lingtu.sim.game-asset-review-catalog.v1",
        "title": "LingTu Asset Review Library",
        "policy": {
            "purpose": "presentation_review_only",
            "runnable": False,
            "qualified_visual": False,
            "unreal_collision_profile": "NoCollision",
            "unreal_simulate_physics": False,
            "physics_authority": "mujoco",
        },
        "coverage": {
            "qualified_movable_object_visual": {
                "available": False,
                "reason": "no qualified movable-object visual is registered",
            }
        },
        "cards": [
            {
                "id": "forest_grass",
                "title": "Forest Grass",
                "description": "Conditioned grass candidate for review.",
                "order": 3,
                "asset_class": "vegetation",
                "review": {
                    "stage": "conditioned_review",
                    "disposition": "quarantined",
                    "reason": reason,
                },
                "evidence": [],
                "render_policy": {"physics_authority": "mujoco"},
                "capabilities": {
                    "selectable_for_review": True,
                    "runnable": False,
                    "qualified_visual": False,
                },
                "tags": ["forest", "grass"],
            }
        ],
    }
    document = {**body, "digest": _canonical_digest(body)}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def test_build_compiles_runnable_choice_into_selectable_catalog(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    output_root = repo_root / "build" / "game-selection"
    _write_session_specs(repo_root, "forest_patrol")

    asset_library_path = _asset_library(repo_root / "build" / "ue-assets.json")
    asset_library_digest = json.loads(
        asset_library_path.read_text(encoding="utf-8")
    )["digest"]
    asset_review_path = _asset_review_catalog(
        repo_root / "build" / "game-asset-review.catalog.json"
    )
    asset_review = json.loads(asset_review_path.read_text(encoding="utf-8"))
    catalog = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(_selection_entry("forest_patrol", order=10)),
        output_root=output_root,
        asset_library_path=asset_library_path,
        asset_review_catalog_path=asset_review_path,
    )

    bundle_dir = output_root / "bundles" / "forest_patrol"
    bundle_artifacts = [
        {"path": path.name}
        for path in sorted(bundle_dir.iterdir(), key=lambda item: item.name)
        if path.is_file()
    ]

    assert catalog == {
        "schema": "lingtu.sim.game-selection-catalog.v1",
        "title": "LingTu Simulation Choices",
        "asset_summary": {
            "availability": {
                "state": "available",
                "reason": "validated asset library",
            },
            "library_digest": asset_library_digest,
            "catalog_package_count": 2,
            "source_candidate_count": 1,
            "quarantined_count": 1,
            "unverified_count": 1,
        },
        "asset_review": {
            "availability": {
                "state": "available",
                "reason": "validated asset review catalog",
            },
            "catalog": asset_review,
        },
        "entries": [
            {
                "id": "forest_patrol",
                "title": "Choice forest_patrol",
                "description": "Run forest_patrol",
                "order": 10,
                "availability": {"state": "runnable", "reason": "Ready"},
                "bundle": {
                    "directory": "bundles/forest_patrol",
                    "session_id": "forest_patrol",
                    "artifacts": bundle_artifacts,
                },
                "robot": {"id": "thunderv4", "version": "1.0.3", "label": "Thunder V4"},
                "world": {"id": "forest_hf", "version": "2.0.0", "label": "High-fidelity Forest"},
                "scenario": {"id": "forest_patrol", "version": "1.1.0", "label": "Forest Patrol"},
                "mode": "unreal",
                "tags": ["forest", "robot"],
            }
        ],
        "digest": catalog["digest"],
    }
    assert catalog["digest"] == _canonical_digest(catalog)


def test_selection_spec_requires_at_least_one_entry(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match=r"entries.*non-empty"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(),
            output_root=repo_root / "build" / "game-selection",
        )


def test_selection_order_must_be_non_negative(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match=r"order.*non-negative"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("negative", order=-1, state="unavailable")
            ),
            output_root=repo_root / "build" / "game-selection",
        )


def test_missing_asset_library_is_explicitly_unavailable(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    catalog = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(
            _selection_entry("preview", order=1, state="preview_only")
        ),
        output_root=repo_root / "build" / "game-selection",
    )

    assert catalog["asset_summary"] == {
        "availability": {
            "state": "unavailable",
            "reason": "asset library not provided",
        },
        "library_digest": None,
        "catalog_package_count": 0,
        "source_candidate_count": 0,
        "quarantined_count": 0,
        "unverified_count": 0,
    }
    assert catalog["asset_review"] == {
        "availability": {
            "state": "unavailable",
            "reason": "asset review catalog not provided",
        },
        "catalog": None,
    }


def test_build_uses_one_canonical_resolver_for_all_runnable_choices(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "first", "second")

    catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(
            _selection_entry("first", order=1),
            _selection_entry("second", order=2),
        ),
        output_root=repo_root / "build" / "game-selection",
    )

    assert _RecordingResolver.repository_calls == [repo_root]
    assert _RecordingResolver.resolve_calls == [
        repo_root / "sim/sessions/examples/first/session.yaml",
        repo_root / "sim/sessions/examples/second/session.yaml",
    ]


@pytest.mark.parametrize("state", ["preview_only", "quarantined", "unavailable"])
def test_non_runnable_choice_is_confirmable_without_compiling_bundle(
    state: str,
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, state)

    catalog = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(_selection_entry(state, order=1, state=state)),
        output_root=repo_root / "build" / "game-selection",
    )

    assert _RecordingResolver.resolve_calls == []
    assert catalog["entries"][0]["availability"] == {
        "state": state,
        "reason": f"{state} asset review pending",
    }
    assert catalog["entries"][0]["bundle"] is None
    assert catalog["entries"][0]["robot"] == {
        "id": "thunderv4",
        "version": "1.0.3",
        "label": "Thunder V4",
    }
    assert catalog["entries"][0]["world"] == {
        "id": "forest_hf",
        "version": "2.0.0",
        "label": "High-fidelity Forest",
    }
    assert catalog["entries"][0]["scenario"] == {
        "id": "forest_patrol",
        "version": "1.1.0",
        "label": "Forest Patrol",
    }
    assert catalog["entries"][0]["mode"] == "unreal"


def test_runnable_choice_rejects_presentation_that_differs_from_compiled_bundle(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "mismatch")
    entry = _selection_entry("mismatch", order=1)
    entry["presentation"]["world"] = {
        "id": "other_world",
        "version": "9.9.9",
        "label": "Wrong World",
    }

    with pytest.raises(ValueError, match="presentation"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(entry),
            output_root=repo_root / "build" / "game-selection",
        )


def test_choice_requires_complete_presentation_metadata(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    entry = _selection_entry("missing_presentation", order=1, state="unavailable")
    del entry["presentation"]["robot"]

    with pytest.raises(ValueError, match="presentation"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(entry),
            output_root=repo_root / "build" / "game-selection",
        )


def test_entries_are_sorted_by_order_then_id(tmp_path: Path, catalog_module: Any) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "zeta", "alpha", "middle")

    catalog = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(
            _selection_entry("zeta", order=20),
            _selection_entry("middle", order=10),
            _selection_entry("alpha", order=20),
        ),
        output_root=repo_root / "build" / "game-selection",
    )

    assert [entry["id"] for entry in catalog["entries"]] == ["middle", "alpha", "zeta"]


def test_write_is_byte_stable_when_input_entry_order_changes(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "alpha", "beta")
    entries = [_selection_entry("alpha", order=2), _selection_entry("beta", order=1)]

    first = catalog_module.write_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(*entries),
        output_root=repo_root / "build" / "first",
    )
    second = catalog_module.write_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(*reversed(entries)),
        output_root=repo_root / "build" / "second",
    )

    assert Path(first).read_bytes() == Path(second).read_bytes()


def test_asset_review_embedding_is_byte_stable_across_json_formatting(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    entry = _selection_entry("preview", order=1, state="preview_only")
    first_review = _asset_review_catalog(repo_root / "build" / "review-first.json")
    review_document = json.loads(first_review.read_text(encoding="utf-8"))
    second_review = repo_root / "build" / "review-second.json"
    second_review.write_text(
        json.dumps(review_document, ensure_ascii=False, indent=4, sort_keys=True),
        encoding="utf-8",
    )

    first = catalog_module.write_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(entry),
        output_root=repo_root / "build" / "first",
        asset_review_catalog_path=first_review,
    )
    second = catalog_module.write_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(entry),
        output_root=repo_root / "build" / "second",
        asset_review_catalog_path=second_review,
    )

    assert first.read_bytes() == second.read_bytes()


def test_asset_review_metadata_binds_catalog_digest_without_changing_bundle(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "forest")
    first_review = _asset_review_catalog(
        repo_root / "build" / "review-first.json", reason="First review"
    )
    second_review = _asset_review_catalog(
        repo_root / "build" / "review-second.json", reason="Second review"
    )

    first = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(_selection_entry("forest", order=1)),
        output_root=repo_root / "build" / "first",
        asset_review_catalog_path=first_review,
    )
    first_bundle = {
        path.name: path.read_bytes()
        for path in (repo_root / "build" / "first" / "bundles" / "forest").iterdir()
    }
    second = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(_selection_entry("forest", order=1)),
        output_root=repo_root / "build" / "second",
        asset_review_catalog_path=second_review,
    )
    second_bundle = {
        path.name: path.read_bytes()
        for path in (repo_root / "build" / "second" / "bundles" / "forest").iterdir()
    }

    assert first["digest"] != second["digest"]
    assert first["entries"][0]["bundle"] == second["entries"][0]["bundle"]
    assert first_bundle == second_bundle


@pytest.mark.parametrize(
    "session_spec",
    ["../outside/session.yaml", "sim/sessions/examples/../../outside.yaml", "C:/outside/session.yaml"],
)
def test_session_spec_must_be_repo_relative_without_traversal(
    session_spec: str,
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match="session_spec"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("unsafe", order=1, session_spec=session_spec)
            ),
            output_root=repo_root / "build" / "game-selection",
        )


def test_duplicate_choice_ids_are_rejected(tmp_path: Path, catalog_module: Any) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    duplicate = _selection_entry("same", order=1)

    with pytest.raises(ValueError, match=r"duplicate.*same"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(duplicate, copy.deepcopy(duplicate)),
            output_root=repo_root / "build" / "game-selection",
        )


def test_unknown_availability_state_is_rejected(tmp_path: Path, catalog_module: Any) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()

    with pytest.raises(ValueError, match="availability"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("invalid", order=1, state="maybe")
            ),
            output_root=repo_root / "build" / "game-selection",
        )


def test_runnable_choice_requires_materialized_bundle(tmp_path: Path, catalog_module: Any) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "missing")
    _RecordingResolver.resolved["missing"] = _Resolved("missing", write_files=False)

    with pytest.raises(ValueError, match="bundle"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(_selection_entry("missing", order=1)),
            output_root=repo_root / "build" / "game-selection",
        )


def test_asset_review_catalog_rejects_noncanonical_card_order(
    tmp_path: Path, catalog_module: Any
) -> None:
    repo_root = tmp_path / "repo"
    review_path = _asset_review_catalog(repo_root / "build" / "review.json")
    review = json.loads(review_path.read_text(encoding="utf-8"))
    later = copy.deepcopy(review["cards"][0])
    later.update({"id": "forest_fern", "order": 8})
    review["cards"] = [later, review["cards"][0]]
    review["digest"] = _canonical_digest(review)
    review_path.write_text(json.dumps(review), encoding="utf-8")

    with pytest.raises(ValueError, match=r"sorted by order then id"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("preview", order=1, state="preview_only")
            ),
            output_root=repo_root / "build" / "game-selection",
            asset_review_catalog_path=review_path,
        )


def test_asset_review_catalog_rejects_duplicate_json_fields(
    tmp_path: Path, catalog_module: Any
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    review_path = repo_root / "review.json"
    review_path.write_text(
        '{"schema":"lingtu.sim.game-asset-review-catalog.v1",'
        '"schema":"lingtu.sim.game-asset-review-catalog.v1"}',
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match=r"duplicate JSON field: schema"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("preview", order=1, state="preview_only")
            ),
            output_root=repo_root / "build" / "game-selection",
            asset_review_catalog_path=review_path,
        )


def test_asset_review_catalog_path_must_remain_inside_repository(
    tmp_path: Path, catalog_module: Any
) -> None:
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    outside_review = _asset_review_catalog(tmp_path / "outside-review.json")

    with pytest.raises(ValueError, match=r"asset review catalog.*escapes"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("preview", order=1, state="preview_only")
            ),
            output_root=repo_root / "build" / "game-selection",
            asset_review_catalog_path=outside_review,
        )


def test_asset_review_catalog_path_rejects_reparse_components(
    tmp_path: Path, catalog_module: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo_root = tmp_path / "repo"
    review_path = _asset_review_catalog(repo_root / "build" / "review.json")
    original_is_reparse = catalog_module._is_reparse
    monkeypatch.setattr(
        catalog_module,
        "_is_reparse",
        lambda path: Path(path) == review_path or original_is_reparse(path),
    )

    with pytest.raises(ValueError, match=r"link or reparse point"):
        catalog_module.build_game_selection_catalog(
            repo_root,
            selection_spec=_selection_spec(
                _selection_entry("preview", order=1, state="preview_only")
            ),
            output_root=repo_root / "build" / "game-selection",
            asset_review_catalog_path=review_path,
        )


def test_output_catalog_contains_no_absolute_or_parent_paths(
    tmp_path: Path,
    catalog_module: Any,
) -> None:
    repo_root = tmp_path / "repo"
    _write_session_specs(repo_root, "forest")

    catalog = catalog_module.build_game_selection_catalog(
        repo_root,
        selection_spec=_selection_spec(_selection_entry("forest", order=1)),
        output_root=repo_root / "build" / "game-selection",
    )

    serialized = json.dumps(catalog, ensure_ascii=False)
    assert str(repo_root) not in serialized
    assert str(tmp_path) not in serialized
    assert ".." not in catalog["entries"][0]["bundle"]["directory"].split("/")
