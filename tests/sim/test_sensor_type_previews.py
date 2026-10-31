"""Sensor selector art is complete, reproducible and explicitly non-measurement."""

from __future__ import annotations

import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from sim.tools.assets import render_sensor_type_previews as previews

MODALITIES = {"rgb", "depth", "lidar", "imu", "thermal", "acoustic", "semantic", "gnss", "truth_odom"}
ASSETS = Path(__file__).resolve().parents[2] / ("sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Resources/Sensors")
NS = {"svg": "http://www.w3.org/2000/svg"}
EXPECTED_FILES = {f"{key}.{ext}" for key in MODALITIES for ext in ("png", "svg")} | {
    "manifest.json",
    "SOURCES.md",
}


def check_pack(directory):
    assert {p.name for p in directory.iterdir()} == EXPECTED_FILES
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["meta"]["version"] == 1
    assert manifest["meta"]["root"] == "."
    assert manifest["meta"]["classification"] == "schematic"
    assert manifest["meta"]["measurement"] is False
    assert manifest["meta"]["generator"] == "sim/tools/assets/render_sensor_type_previews.py"
    assert set(manifest["images"]) == MODALITIES
    for key, entry in manifest["images"].items():
        assert entry["path"] == f"{key}.png"
        assert entry["source"] == f"{key}.svg"
        assert (entry["width"], entry["height"]) == (480, 240)
        assert entry["classification"] == "schematic"
        assert entry["measurement"] is False
        assert entry["depicts"]
        png = (directory / entry["path"]).read_bytes()
        assert png[:8] == b"\x89PNG\r\n\x1a\n"
        assert png[12:16] == b"IHDR"
        assert struct.unpack(">II", png[16:24]) == (480, 240)
        svg = ET.parse(directory / entry["source"]).getroot()
        assert svg.attrib["width"] == "480"
        assert svg.attrib["height"] == "240"
        assert svg.attrib["viewBox"] == "0 0 480 240"
        assert svg.find("svg:title", NS).text
        assert "not measurement data" in svg.find("svg:desc", NS).text
        assert "SCHEMATIC" in [t.text for t in svg.findall("svg:text", NS)]
        # Sources are self-contained vector commands, not embedded/linked images.
        assert svg.find(".//svg:image", NS) is None
        assert all("href" not in attr for node in svg.iter() for attr in node.attrib)
    sources = (directory / "SOURCES.md").read_text(encoding="utf-8")
    assert "not measurement data" in sources
    assert "No third-party image assets" in sources
    for key in MODALITIES:
        assert f"`{key}.png`" in sources
        assert f"`{key}.svg`" in sources


def test_bundled_sensor_preview_pack():
    """Shipped resources are testable without installing a rendering dependency."""
    check_pack(ASSETS)


@pytest.mark.parametrize(
    ("modality", "labels"),
    [
        ("rgb", ("R", "G", "B")),
        ("depth", ("NEAR", "FAR", "Range / m")),
        ("lidar", ("EMIT / RETURN", "Point cloud / m")),
        ("imu", ("X", "Y", "Z", "a / m/s^2", "Angular rate / rad/s")),
        ("thermal", ("330 K", "280 K")),
        ("acoustic", ("Pressure / Pa", "Time / s")),
        ("semantic", ("C2 / I7", "C5 / I3", "Class + instance IDs")),
        ("gnss", ("Satellite positioning", "POSITION FIX")),
        ("truth_odom", ("Simulator reference pose", "REFERENCE / NOT ESTIMATE")),
    ],
)
def test_modality_meaning_and_units(modality, labels):
    root = ET.parse(ASSETS / f"{modality}.svg").getroot()
    text = {node.text for node in root.findall("svg:text", NS)}
    assert set(labels) <= text


def test_regeneration_matches_bundled_sources_and_manifest(tmp_path):
    image = pytest.importorskip("PIL.Image")
    generated = previews.generate(tmp_path / "fresh")
    assert {p.name for p in generated} == EXPECTED_FILES
    check_pack(tmp_path / "fresh")
    for path in generated:
        if path.suffix != ".png":
            assert path.read_bytes() == (ASSETS / path.name).read_bytes()
    rasters = []
    for key in MODALITIES:
        with image.open(tmp_path / "fresh" / f"{key}.png") as png:
            png.load()
            assert png.format == "PNG"
            assert png.size == (480, 240)
            assert png.mode == "RGB"
            assert png.getpixel((0, 0)) == (237, 240, 243)
            assert len(png.getcolors(maxcolors=480 * 240)) > 100
            rasters.append(png.tobytes())
        with image.open(ASSETS / f"{key}.png") as bundled:
            bundled.load()
            assert bundled.size == (480, 240)
            assert bundled.tobytes() == rasters[-1]
    assert len(set(rasters)) == len(MODALITIES)


def test_render_failure_does_not_publish_partial_pack(tmp_path, monkeypatch):
    pytest.importorskip("PIL")
    original = previews.draw_modality

    def fail_after_first(modality):
        if modality == "depth":
            raise RuntimeError("Rendering failed")
        return original(modality)

    monkeypatch.setattr(previews, "draw_modality", fail_after_first)
    destination = tmp_path / "not-created"
    with pytest.raises(RuntimeError, match="Rendering failed"):
        previews.generate(destination)
    assert not destination.exists()
