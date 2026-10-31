"""Offline dependency-download contracts for our selected asset intake tool."""

import json

import pytest
from sim.tools.assets import polyhaven_intake as intake


def fixture_payload():
    return {
        "gltf": {
            "2k": {
                "gltf": {
                    "url": "https://dl.polyhaven.org/asset.gltf",
                    "size": 4,
                    "include": {"textures/color.jpg": {"url": "https://dl.polyhaven.org/color.jpg", "size": 3}},
                }
            }
        }
    }


def test_downloads_declared_dependencies_and_reuses_complete_files(tmp_path, monkeypatch):
    calls = []

    def fetch(url):
        calls.append(url)
        if "/info/" in url:
            return json.dumps({"name": "Asset", "authors": {"Artist": "All"}}).encode()
        if "/files/" in url:
            return json.dumps(fixture_payload()).encode()
        return b"glTF" if url.endswith(".gltf") else b"jpg"

    monkeypatch.setattr(intake, "fetch", fetch)
    result = intake.intake("asset", "2k", tmp_path)
    assert result["files"] == 2 and result["bytes"] == 7
    folder = tmp_path / "asset/2k"
    assert (folder / "textures/color.jpg").read_bytes() == b"jpg"
    record = json.loads((folder / "source.json").read_text())
    assert record["license"] == "CC0-1.0" and not record["ue_imported"]
    calls.clear()
    intake.intake("asset", "2k", tmp_path)
    assert len(calls) == 2


def test_incomplete_model_is_not_saved_as_complete(tmp_path, monkeypatch):
    def fetch(url):
        if "/info/" in url:
            return b'{"name":"Asset"}'
        if "/files/" in url:
            return json.dumps(fixture_payload()).encode()
        return b"x"

    monkeypatch.setattr(intake, "fetch", fetch)
    with pytest.raises(ValueError, match="Incomplete download"):
        intake.intake("asset", "2k", tmp_path)
    assert not (tmp_path / "asset/2k/asset.gltf").exists()
