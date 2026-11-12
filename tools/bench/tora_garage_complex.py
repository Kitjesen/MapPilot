"""Build the larger staggered-parking garage using the existing Tora scene assets."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from tools.bench.tora_garage_scene import BODY_HEIGHT, add_box, add_robot, build_scene, overview_camera, vector

DEFAULT_OUTPUT = ROOT / "artifacts/nav/tora-garage-complex-20261106"


def build_complex_scene(output: Path = DEFAULT_OUTPUT) -> dict:
    output = Path(output)
    description = build_scene(output)
    model = ET.parse(output / "world.xml").getroot()
    model.set("model", "tora_staggered_parking_garage")
    model.find("statistic").set("extent", "16")
    world = model.find("worldbody")
    templates = {element.get("name"): deepcopy(element) for element in world}
    world.clear()
    floor = templates["floor"]
    floor.set("size", "12 9 .1")
    world.append(floor)
    for index, (x, y) in enumerate((x, y) for y in (-5, 5) for x in (-8, 0, 8)):
        ET.SubElement(world, "light", name=f"ceiling_light_{index}", pos=f"{x} {y} 4.5", dir="0 0 -1",
                      diffuse=".58 .58 .56", specular=".1 .1 .1", castshadow="true")
        add_box(world, f"lamp_{index}", [x, y, 3.5], [1.1, .07, .025], [.95, .95, .90, 1], physical=False)
    for index, y in enumerate((-8, 8)):
        add_box(world, f"ceiling_beam_{index}", [0, y, 3.65], [12, .15, .17],
                [.65, .68, .70, 1], physical=False)
    obstacles = []

    def obstacle(name, center, half_size, rgba):
        add_box(world, name, center, half_size, rgba)
        obstacles.append({"name": name, "center": center, "half_size": half_size})

    obstacle("north_wall", [0, 9, 1.6], [12, .12, 1.6], [.74, .77, .79, 1])
    obstacle("south_cutaway_wall", [0, -9, .35], [12, .12, .35], [.74, .77, .79, 1])
    obstacle("west_cutaway_wall", [-12, 0, .35], [.12, 9, .35], [.74, .77, .79, 1])
    obstacle("east_cutaway_wall", [12, 0, .35], [.12, 9, .35], [.74, .77, .79, 1])
    # Parking-row dividers leave alternating end aisles and a 2.7 m centre aisle.
    obstacle("lower_parking_divider", [-3.3, -1.5, .325], [8.6, .15, .325], [.62, .65, .66, 1])
    obstacle("upper_parking_divider", [3.3, 1.5, .325], [8.6, .15, .325], [.62, .65, .66, 1])
    positions = [(x, -3.4) for x in (-10, -7.2, -4.4, -1.6, 1.2, 4)]
    positions += [(x, 3.4) for x in (-4, -1.2, 1.6, 4.4, 7.2, 10)]
    for index, (x, y) in enumerate(positions):
        source = index % 6
        old_center = np.fromstring(templates[f"car_{source}_body"].get("pos"), sep=" ")
        delta = np.asarray([x, y, .48]) - old_center
        for name, template in templates.items():
            if not name.startswith((f"car_{source}_", f"parking_line_{source}_")):
                continue
            element = deepcopy(template)
            element.set("name", name.replace(f"_{source}_", f"_{index}_", 1))
            center = np.fromstring(element.get("pos"), sep=" ") + delta
            element.set("pos", vector(center))
            world.append(element)
            if element.get("contype") == "1":
                obstacles.append({"name": element.get("name"), "center": center.tolist(),
                                  "half_size": np.fromstring(element.get("size"), sep=" ").tolist()})
    columns = [(x, -1.9) for x in (-8.6, -5.8, -3, -.2)]
    columns += [(x, 1.9) for x in (.2, 3, 5.8, 8.6)]
    for index, (x, y) in enumerate(columns):
        obstacle(f"column_{index}", [x, y, 1.7], [.25, .25, 1.7], [.76, .78, .79, 1])
        for band, z in enumerate((.4, .7, 1.0)):
            add_box(world, f"column_band_{index}_{band}", [x, y, z], [.254, .254, .08],
                    [.91, .66, .13, 1], physical=False)
    for index, (x, y) in enumerate([(-10.5, -1.9), (10.5, 1.9)]):
        obstacle(f"electrical_cabinet_{index}", [x, y, .7], [.45, .28, .7], [.34, .42, .45, 1])
    for index, x in enumerate(range(-9, 10, 3)):
        for lane, y in enumerate((-6.7, 0, 6.7)):
            add_box(world, f"lane_dash_{lane}_{index}", [x, y, .008], [.65, .025, .004],
                    [.85, .86, .82, 1], physical=False)
    for name, template in templates.items():
        if name.startswith("parking_sign"):
            element = deepcopy(template)
            center = np.add(np.fromstring(element.get("pos"), sep=" "), [0, 3, 0])
            element.set("pos", vector(center))
            world.append(element)
    start = [-10, -6.7, BODY_HEIGHT, 0.0]
    goal = [10, 6.7, BODY_HEIGHT, 0.0]
    description.update(name="tora_staggered_parking_garage", bounds=[-12, -9, 12, 9],
                       start=start, goal=goal, obstacles=obstacles, parked_cars=12, columns=8,
                       centre_aisle_width_m=2.7,
                       layout="two staggered parking rows with alternating end circulation aisles")
    ET.indent(model)
    ET.ElementTree(model).write(output / "world.xml", encoding="utf-8", xml_declaration=True)
    combined = deepcopy(model)
    add_robot(combined, start)
    ET.indent(combined)
    ET.ElementTree(combined).write(output / "render_scene.xml", encoding="utf-8", xml_declaration=True)
    (output / "scene.json").write_text(json.dumps(description, indent=2) + "\n", encoding="utf-8")
    return description


def render_snapshots(output: Path) -> None:
    scene = json.loads((output / "scene.json").read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(output / "render_scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = overview_camera()
    camera.distance, camera.elevation = 30, -62
    with mujoco.Renderer(model, height=1080, width=1920) as renderer:
        renderer.update_scene(data, camera=camera)
        cv2.imwrite(str(output / "overview.png"), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
        camera.lookat[:] = np.add(scene["start"][:3], [0, 0, .65])
        camera.distance, camera.azimuth, camera.elevation = 5.5, 45, -32
        renderer.update_scene(data, camera=camera)
        cv2.imwrite(str(output / "pose_snapshot.png"), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    scene = build_complex_scene(args.output)
    render_snapshots(args.output)
    print(json.dumps({"output": str(args.output), "obstacles": len(scene["obstacles"]),
                      "start": scene["start"], "goal": scene["goal"]}))


if __name__ == "__main__":
    main()
