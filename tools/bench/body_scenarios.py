"""Shared geometry for native-map and actual MuJoCo component fixtures.

Obstacle poses sent to navigation are deterministic fixture ground truth. They
are not LiDAR detections and do not validate perception or field behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def read_scene_boxes(path: Path) -> list[tuple[list[float], list[float]]]:
    values = path.read_text(encoding="utf-8-sig").split()
    if len(values) < 8:
        raise ValueError("Scene requires resolution, XYZ bounds and box count")
    resolution = float(values[0])
    bounds = [float(value) for value in values[1:7]]
    count = int(values[7])
    if (not math.isfinite(resolution) or resolution <= 0 or count < 0
            or len(values) != 8 + count * 6 or not all(math.isfinite(v) for v in bounds)
            or any(bounds[i] >= bounds[i + 3] for i in range(3))):
        raise ValueError("Invalid native scene geometry")
    boxes = []
    for index in range(count):
        row = [float(value) for value in values[8 + 6 * index:14 + 6 * index]]
        if not all(math.isfinite(v) for v in row) or any(row[i] >= row[i + 3] for i in range(3)):
            raise ValueError("Scene boxes require finite increasing bounds")
        boxes.append((row[:3], row[3:]))
    return boxes


@dataclass(frozen=True)
class DynamicBox:
    name: str
    half_size: tuple[float, float, float]
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    start_s: float
    duration_s: float

    def at(self, elapsed_s: float) -> tuple[list[float], list[float]]:
        alpha = min(1.0, max(0.0, (elapsed_s - self.start_s) / self.duration_s))
        position = [a + (b - a) * alpha for a, b in zip(self.start, self.end)]
        moving = self.start_s <= elapsed_s < self.start_s + self.duration_s
        velocity = [(b - a) / self.duration_s if moving else 0.0 for a, b in zip(self.start, self.end)]
        return position, velocity


def read_dynamic_boxes(path: Path | None) -> list[DynamicBox]:
    if path is None:
        return []
    result = []
    for row in json.loads(path.read_text(encoding="utf-8-sig")):
        box = DynamicBox(name=str(row["name"]), half_size=tuple(row["half_size"]),
                         start=tuple(row["start"]), end=tuple(row["end"]),
                         start_s=float(row["start_s"]), duration_s=float(row["duration_s"]))
        if (not box.name or box.name in {item.name for item in result}
                or any(len(v) != 3 for v in (box.half_size, box.start, box.end))
                or not all(math.isfinite(v) for v in (*box.half_size, *box.start, *box.end,
                                                       box.start_s, box.duration_s))
                or min(box.half_size) <= 0 or box.start_s < 0 or box.duration_s <= 0):
            raise ValueError("Dynamic boxes require unique names, positive size/duration and finite XYZ poses")
        result.append(box)
    return result


def prepare_world(base: Path, scene: Path | None, dynamic: list[DynamicBox], output: Path) -> Path:
    """Replace static world collision geoms with the exact native scene boxes."""
    if scene is None and not dynamic:
        return base
    from sim.compat.engine.mujoco.engine import resolve_scene_asset_paths

    tree = ET.parse(base)
    root = tree.getroot()
    resolve_scene_asset_paths(root, base.resolve())
    world = root.find("worldbody")
    if world is None:
        raise ValueError("Physics world requires worldbody")
    if scene is not None:
        for geom in list(world.findall("geom")):
            if geom.get("contype", "1") != "0" or geom.get("conaffinity", "1") != "0":
                world.remove(geom)
        for index, (low, high) in enumerate(read_scene_boxes(scene)):
            center = [(a + b) / 2 for a, b in zip(low, high)]
            half = [(b - a) / 2 for a, b in zip(low, high)]
            attrs = {"name": f"fixture_static_{index}", "contype": "1", "conaffinity": "1",
                     "condim": "3", "rgba": "0.40 0.46 0.49 1"}
            if high[2] == 0 and low[2] < 0:
                # A bounded flat heightfield has the exact same solid volume.
                # Bound cell size: huge thin heightfield triangles can also
                # invert near-tangent contact normals. The volume is unchanged.
                assets = root.find("asset")
                if assets is None:
                    assets = ET.SubElement(root, "asset")
                name = f"fixture_floor_{index}"
                nrow = math.ceil((high[1] - low[1]) / .5) + 1
                ncol = math.ceil((high[0] - low[0]) / .5) + 1
                ET.SubElement(assets, "hfield", name=name, nrow=str(nrow), ncol=str(ncol),
                              elevation=" ".join("0" for _ in range(nrow * ncol)),
                              size=f"{half[0]} {half[1]} 1 {-low[2]}")
                ET.SubElement(world, "geom", **attrs, type="hfield", hfield=name,
                              pos=f"{center[0]} {center[1]} 0")
            else:
                ET.SubElement(world, "geom", **attrs, type="box",
                              pos=" ".join(map(str, center)), size=" ".join(map(str, half)))
    for box in dynamic:
        body = ET.SubElement(world, "body", name=box.name, mocap="true", pos=" ".join(map(str, box.start)))
        ET.SubElement(body, "geom", name=box.name + "_geom", type="box",
                      size=" ".join(map(str, box.half_size)), contype="1", conaffinity="1",
                      condim="3", rgba="0.72 0.22 0.16 1")
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def write_fixtures(output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    scenarios = {
        "multi-turn-22m": {"bounds": [-2, -3, -.1, 19, 9, 2], "walls": [],
                            "waypoints": [[0, 0, 0], [8, 0, 0], [8, 0, 90], [8, 6, 90],
                                          [8, 6, 0], [16, 6, 0]], "duration_s": 100,
                            "expectation": "arrive; 22m requested segmented route, not automatic global turn generation"},
        "corridor-positive": {"bounds": [-2, -2, -.1, 10, 2, 2], "width": 1.10,
                               "duration_s": 60, "expectation": "arrive without physical contact"},
        "corridor-negative": {"bounds": [-2, -2, -.1, 10, 2, 2], "width": .60,
                               "duration_s": 30, "expectation": "reject or stop safely before impossible opening"},
        "crossing": {"bounds": [-2, -3, -.1, 10, 3, 2], "walls": [], "duration_s": 60,
                     "dynamic": [{"name": "fixture_actor", "half_size": [.25, .25, .75],
                                  "start": [3, -2, .75], "end": [3, 2, .75], "start_s": 3, "duration_s": 8}],
                     "expectation": "avoid contact, retain goal and resume after crossing"},
        "persistent": {"bounds": [-2, -3, -.1, 10, 3, 2], "walls": [], "duration_s": 75,
                       "dynamic": [{"name": "fixture_actor", "half_size": [.35, .35, .75],
                                    "start": [3, -2, .75], "end": [3, 0, .75], "start_s": 2, "duration_s": 4}],
                       "expectation": "replan around persistent blocker and retain original goal"},
    }
    manifest: dict[str, Any] = {"scope": "native component + actual policy/MuJoCo; synthetic fully observed map; deterministic ground-truth obstacle injection; not LiDAR or Product evidence", "scenarios": {}}
    for name, config in scenarios.items():
        folder = output / name
        folder.mkdir(exist_ok=True)
        low, high = config["bounds"][:3], config["bounds"][3:]
        boxes = [[*low, high[0], high[1], 0.0]]
        if "width" in config:
            width = config["width"]
            boxes += [[2, low[1], 0, 6, -width / 2, 1.5], [2, width / 2, 0, 6, high[1], 1.5]]
        scene = folder / "scene.txt"
        values = [.1, *low, *high, len(boxes), *(v for box in boxes for v in box)]
        scene.write_text(" ".join(map(str, values)) + "\n", encoding="utf-8")
        points = config.get("waypoints", [[0, 0, 0], [8, 0, 0]])
        (folder / "waypoints.json").write_text(json.dumps(points, indent=2) + "\n", encoding="utf-8")
        if "dynamic" in config:
            (folder / "dynamic.json").write_text(json.dumps(config["dynamic"], indent=2) + "\n", encoding="utf-8")
        manifest["scenarios"][name] = {**config, "scene": str(scene.resolve()),
                                            "waypoints": str((folder / "waypoints.json").resolve())}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    write_fixtures(parser.parse_args().out)
