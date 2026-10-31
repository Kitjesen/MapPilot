from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "tools/bench/paper_bundle.py"
SPEC = importlib.util.spec_from_file_location("paper_bundle", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture(autouse=True)
def disk_budget_for_tiny_fixtures(monkeypatch, tmp_path):
    usage = MODULE.shutil.disk_usage(tmp_path)
    monkeypatch.setattr(MODULE.shutil, "disk_usage", lambda path: usage._replace(free=200 * 1024**3))


def put(root, relative, content="fixture"):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path


def test_bundle_preserves_working_tree_maps_failures_and_external_inventory(tmp_path):
    repo = tmp_path / "repo"
    put(repo, "src/example.py", "uncommitted source")
    put(repo, "sim/project/Source/LingTu.Build.cs", "Unreal build source")
    put(repo, "scripts/deploy/lingtu.service", "deployment unit")
    put(repo, "src/nav/cpp/vendor/module.cpp", "owned vendor modifications")
    put(repo, "build/map-run/map.ot", "output map")
    put(repo, "build/map-run/patch_bundle.manifest", "map builder evidence")
    put(repo, "build/map-run/pass.json", '{"ok": false}')
    put(repo, "datasets/raw/input.bag", "raw")
    put(repo, "sim/evaluation/data/scene/rgb.png", "input image")
    put(repo, "web/node_modules/pkg/index.js", "dependency")
    put(repo, "build/map-run/.builds/copy/map.ot", "staging")
    put(repo, "build/map-run/.feeder/data.json", "feeder")
    put(repo, "build/map-run/.feeder-run/data.json", "feeder")
    put(repo, "build/map-run/.formal-feeder-artifacts-run/data.json", "feeder")
    put(repo, "build/map-run/CMakeScratch/check.json", "generated")
    put(repo, "build/ui-review/profile-cdp-edge/Default/private.json", "fixture profile")
    put(repo, "build/deps/runtime/include/header.h", "dependency")
    put(repo, "build/unreal-sandbox-root-v2/Engine/Source/module.cpp", "engine dependency")
    put(repo, "build/dataset-mapping-20261021/bin/ltlio_replay.exe", "frozen program")
    put(repo, "build/map-repair-20261021/loop-replay-build/libltlio.a", "frozen library")
    put(repo, "build/map-repair-20261021/save-memory/after/ltlio_replay.exe", "new frozen program")
    put(repo, "build/map-repair-20261021/save-memory/before/ltlio_replay.exe", "before program")
    put(repo, "build/map-repair-20261021/save-memory/default-restored/ltlio_replay.exe", "restored default program")
    put(repo, ".env", "fixture credential")
    put(repo, ".env.example", "example config")
    put(repo, "build/map-run/unknown.custom", "unknown")
    output = tmp_path / "bundle"
    summary = MODULE.collect(repo, output, None, run_id="one")
    run = output / "runs/one"
    assert (run / "workspace/src/example.py").read_text() == "uncommitted source"
    assert (run / "workspace/sim/project/Source/LingTu.Build.cs").read_text() == "Unreal build source"
    assert (run / "workspace/scripts/deploy/lingtu.service").read_text() == "deployment unit"
    assert (run / "workspace/src/nav/cpp/vendor/module.cpp").read_text() == "owned vendor modifications"
    assert (run / "workspace/build/map-run/map.ot").read_text() == "output map"
    assert (run / "workspace/build/map-run/patch_bundle.manifest").read_text() == "map builder evidence"
    assert json.loads((run / "workspace/build/map-run/pass.json").read_text()) == {"ok": False}
    with (run / "inventory.csv").open(encoding="utf-8-sig", newline="") as handle:
        rows = {r["repository_path"]: r for r in csv.DictReader(handle)}
    assert rows["datasets/raw/input.bag"]["disposition"] == "external"
    assert rows["sim/evaluation/data/scene/rgb.png"]["disposition"] == "external"
    pruned = {r["repository_path"]: r for r in summary["not_traversed"] if "repository_path" in r}
    assert pruned["web/node_modules"]["disposition"] == "external"
    assert pruned["build/map-run/.builds"]["disposition"] == "excluded"
    assert pruned["build/map-run/.feeder"]["disposition"] == "excluded"
    assert pruned["build/map-run/.feeder-run"]["disposition"] == "excluded"
    assert pruned["build/map-run/.formal-feeder-artifacts-run"]["disposition"] == "excluded"
    assert pruned["build/map-run/CMakeScratch"]["disposition"] == "excluded"
    assert pruned["build/ui-review/profile-cdp-edge"]["bytes"] is None
    assert pruned["build/deps"]["disposition"] == "external"
    assert pruned["build/unreal-sandbox-root-v2/Engine"]["disposition"] == "external"
    assert not (run / "workspace/build/ui-review/profile-cdp-edge").exists()
    assert "web/node_modules/pkg/index.js" not in rows
    assert rows["build/dataset-mapping-20261021/bin/ltlio_replay.exe"]["disposition"] == "copied"
    assert rows["build/map-repair-20261021/loop-replay-build/libltlio.a"]["disposition"] == "copied"
    assert rows["build/map-repair-20261021/save-memory/after/ltlio_replay.exe"]["disposition"] == "copied"
    assert rows["build/map-repair-20261021/save-memory/before/ltlio_replay.exe"]["disposition"] == "copied"
    assert rows["build/map-repair-20261021/save-memory/default-restored/ltlio_replay.exe"]["disposition"] == "copied"
    assert rows[".env"]["disposition"] == "external"
    assert rows[".env.example"]["disposition"] == "copied"
    assert not (run / "workspace/.env").exists()
    assert rows["build/map-run/unknown.custom"]["disposition"] == "unreviewed"
    assert rows["src/example.py"]["source_path"] == str(repo / "src/example.py")
    assert int(rows["src/example.py"]["bytes"]) == len("uncommitted source")
    assert not summary["errors"]
    put(repo, "src/example.py", "second version")
    MODULE.collect(repo, output, None, run_id="two")
    assert (run / "workspace/src/example.py").read_text() == "uncommitted source"
    assert (output / "runs/two/workspace/src/example.py").read_text() == "second version"
    assert json.loads((output / "latest.json").read_text())["run_id"] == "two"


def test_unrelated_output_is_preserved_and_repo_output_is_rejected(tmp_path):
    repo = tmp_path / "repo"
    put(repo, "src/test.py")
    output = tmp_path / "existing"
    put(output, "user.txt", "keep")
    with pytest.raises(ValueError, match="unrelated"):
        MODULE.collect(repo, output, None)
    assert (output / "user.txt").read_text() == "keep"
    with pytest.raises(ValueError, match="outside"):
        MODULE.collect(repo, repo / "bundle", None)


def test_copy_budget_records_exact_external_source_and_historical_context(tmp_path):
    repo, archive = tmp_path / "repo", tmp_path / "archive"
    put(repo, "src/program.py", "source")
    put(archive, "ARTIFACTS.md", "historical record")
    put(archive, "artifacts/run/result.csv", "a,b")
    summary = MODULE.collect(repo, tmp_path / "bundle", archive, budget_bytes=0, run_id="limited")
    assert summary["counts"] == {"external": 3}
    assert summary["bytes"]["external"] == len("sourcehistorical recorda,b")


def test_deep_source_paths_survive_the_added_snapshot_prefix(tmp_path):
    repo = tmp_path / "repo"
    relative = Path("src", *(["segment123"] * 16), "program.cpp")
    source = MODULE.filesystem_path(repo / relative)
    source.parent.mkdir(parents=True)
    source.write_text("long path source", encoding="utf-8")
    output = tmp_path / "bundle"
    summary = MODULE.collect(repo, output, None, run_id="deep")
    target = MODULE.filesystem_path(output / "runs/deep/workspace" / relative)
    assert target.read_text(encoding="utf-8") == "long path source"
    assert not summary["errors"]
