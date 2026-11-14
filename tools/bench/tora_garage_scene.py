"""Build and render a clean garage with the locked-pose Tora entity."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from copy import deepcopy
from itertools import pairwise
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from tools.robot.tora_geometry import DEFAULT_URDF, REFERENCE_LINK, load_collision_meshes, origin

DEFAULT_OUTPUT = ROOT / "artifacts/nav/tora-garage-20261106"
BODY_HEIGHT = 0.178000853265161


def vector(values) -> str:
    return " ".join(format(float(value), ".12g") for value in values)


def add_box(parent, name, center, half_size, rgba, *, physical=True):
    return ET.SubElement(parent, "geom", name=name, type="box", pos=vector(center), size=vector(half_size),
                         rgba=vector(rgba), contype=str(int(physical)), conaffinity=str(int(physical)))


def add_robot(model: ET.Element, start: list) -> None:
    """Keep all original collision triangles as shaded solid visual meshes."""
    _, metadata = load_collision_meshes()
    robot = ET.parse(DEFAULT_URDF).getroot()
    transforms = {"base_link": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        remaining = []
        for joint in pending:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                remaining.append(joint)
                continue
            transforms[joint.find("child").get("link")] = transforms[parent] @ origin(joint.find("origin"))
        pending = remaining
    reference_from_root = np.linalg.inv(np.asarray(metadata["root_from_reference"]))
    assets = model.find("asset")
    entity = ET.SubElement(model.find("worldbody"), "body", name="tora", mocap="true", pos=vector(start[:3]))
    for link in robot.findall("link"):
        name = link.get("name")
        color = link.find("visual/material/color")
        rgba = color.get("rgba") if color is not None else ".55 .58 .62 1"
        for index, collision in enumerate(link.findall("collision")):
            mesh = collision.find("geometry/mesh")
            filename = mesh.get("filename").split("/", 3)[-1]
            path = DEFAULT_URDF.parent.parent / filename
            mesh_name = f"tora_{name}_{index}"
            ET.SubElement(assets, "mesh", name=mesh_name, file=path.as_posix(), scale=mesh.get("scale", "1 1 1"))
            transform = reference_from_root @ transforms[name] @ origin(collision.find("origin"))
            quaternion = np.zeros(4)
            mujoco.mju_mat2Quat(quaternion, transform[:3, :3].ravel())
            ET.SubElement(entity, "geom", name=mesh_name, type="mesh", mesh=mesh_name,
                          pos=vector(transform[:3, 3]), quat=vector(quaternion), rgba=rgba,
                          contype="0", conaffinity="0", group="1")


def build_scene(output: Path = DEFAULT_OUTPUT, *, include_robot: bool = True) -> dict:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    for filename in ("world.xml", "render_scene.xml", "scene.json"):
        if (output / filename).exists():
            raise FileExistsError(f"Scene output already exists: {output / filename}")
    model = ET.Element("mujoco", model="tora_clean_underground_garage")
    ET.SubElement(model, "compiler", angle="radian")
    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", offwidth="1920", offheight="1080", fovy="45")
    ET.SubElement(visual, "quality", shadowsize="4096", offsamples="4")
    ET.SubElement(visual, "headlight", ambient=".35 .35 .35", diffuse=".45 .45 .45", specular=".1 .1 .1")
    ET.SubElement(visual, "rgba", haze=".75 .79 .82 1")
    ET.SubElement(model, "statistic", center="0 0 1", extent="12")
    assets = ET.SubElement(model, "asset")
    ET.SubElement(assets, "texture", type="skybox", builtin="gradient", width="512", height="512",
                  rgb1=".82 .86 .88", rgb2=".68 .73 .77")
    ET.SubElement(assets, "texture", name="floor_tex", type="2d", builtin="checker", width="512", height="512",
                  rgb1=".50 .53 .55", rgb2=".505 .535 .555")
    ET.SubElement(assets, "material", name="floor_material", texture="floor_tex", texrepeat="18 12",
                  reflectance="0", shininess=".08")
    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="9 6 .1", material="floor_material")
    for index, (x, y) in enumerate([(-5, -2), (0, 0), (5, 2)]):
        ET.SubElement(world, "light", name=f"ceiling_light_{index}", pos=f"{x} {y} 4.5", dir="0 0 -1",
                      diffuse=".75 .75 .72", specular=".2 .2 .2", castshadow="true")
        add_box(world, f"lamp_{index}", [x, y, 3.5], [1.0, .07, .025], [.95, .95, .90, 1], physical=False)
    for index, y in enumerate((-4.9, 4.9)):
        add_box(world, f"ceiling_beam_{index}", [0, y, 3.65], [9, .15, .17], [.65, .68, .70, 1], physical=False)
    obstacles = []

    def obstacle(name, center, half_size, rgba):
        add_box(world, name, center, half_size, rgba)
        obstacles.append({"name": name, "center": center, "half_size": half_size})

    obstacle("north_wall", [0, 6, 1.6], [9, .12, 1.6], [.74, .77, .79, 1])
    obstacle("south_cutaway_wall", [0, -6, .35], [9, .12, .35], [.74, .77, .79, 1])
    obstacle("west_cutaway_wall", [-9, 0, .35], [.12, 6, .35], [.74, .77, .79, 1])
    obstacle("east_cutaway_wall", [9, 0, .35], [.12, 6, .35], [.74, .77, .79, 1])
    for index, (x, y) in enumerate([(-2, -.7), (2, .7)]):
        obstacle(f"column_{index}", [x, y, 1.7], [.35, .35, 1.7], [.76, .78, .79, 1])
        add_box(world, f"column_band_{index}", [x, y, .7], [.354, .354, .16], [.91, .66, .13, 1], physical=False)
    colors = [[.22, .39, .52, 1], [.77, .78, .76, 1], [.27, .31, .35, 1],
              [.64, .24, .20, 1], [.46, .51, .54, 1], [.16, .35, .34, 1]]
    for index, (x, y) in enumerate((x, y) for y in (-4.1, 4.1) for x in (-6, 0, 6)):
        obstacle(f"car_{index}_body", [x, y, .48], [.86, 1.75, .38], colors[index])
        obstacle(f"car_{index}_cabin", [x, y + .10, 1.10], [.69, .95, .25], colors[index])
        add_box(world, f"car_{index}_windshield", [x, y - .858, 1.10], [.61, .006, .19], [.14, .22, .27, 1], physical=False)
        for side in (-1, 1):
            add_box(world, f"car_{index}_side_window_{side}", [x + side * .696, y + .1, 1.1],
                    [.005, .80, .18], [.14, .22, .27, 1], physical=False)
            for axle in (-1, 1):
                ET.SubElement(world, "geom", name=f"car_{index}_wheel_{side}_{axle}", type="cylinder",
                                      pos=vector([x + side * .81, y + axle * 1.15, .28]), size=".27 .06",
                                      quat=".707106781187 0 .707106781187 0", rgba=".09 .10 .11 1", contype="0", conaffinity="0")
        for edge in (-1, 1):
            add_box(world, f"parking_line_{index}_{edge}", [x + edge * 1.15, y, .006],
                    [.025, 1.85, .004], [.93, .94, .92, 1], physical=False)
    for x in (-5, 0, 5):
        add_box(world, f"lane_dash_{x}", [x, 0, .008], [.65, .025, .004], [.85, .86, .82, 1], physical=False)
    add_box(world, "parking_sign", [0, 5.85, 2.2], [1.35, .025, .35], [.14, .32, .40, 1], physical=False)
    for index, (x, z, hx, hz) in enumerate([(-.24, 2.2, .04, .23), (-.04, 2.39, .20, .04),
                                          (.12, 2.29, .04, .10), (-.04, 2.19, .20, .04)]):
        add_box(world, f"parking_sign_letter_{index}", [x, 5.817, z], [hx, .008, hz], [.95, .96, .96, 1], physical=False)
    start = [-7, -1.7, BODY_HEIGHT, 0.0]
    goal = [7, 1.7, BODY_HEIGHT, math.pi / 2]
    description = {
        "name": "tora_clean_underground_garage", "bounds": [-9, -6, 9, 6],
        "ground_z": 0.0, "start": start, "goal": goal, "obstacles": obstacles,
        "robot": "paxini/tora_double_one_x", "body_reference": REFERENCE_LINK,
        "evidence": "native planner with ideal base integration; locked URDF pose rendered as solid meshes",
        "world_xml": "world.xml", "render_scene_xml": "render_scene.xml",
    }
    ET.indent(model)
    ET.ElementTree(model).write(output / "world.xml", encoding="utf-8", xml_declaration=True)
    combined = deepcopy(model)
    if include_robot:
        add_robot(combined, start)
    ET.indent(combined)
    ET.ElementTree(combined).write(output / "render_scene.xml", encoding="utf-8", xml_declaration=True)
    (output / "scene.json").write_text(json.dumps(description, indent=2) + "\n", encoding="utf-8")
    return description


def overview_camera():
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0, 0, .5]
    camera.distance, camera.azimuth, camera.elevation = 20, 90, -47
    return camera


def render_snapshot(output: Path = DEFAULT_OUTPUT) -> Path:
    scene = json.loads((output / "scene.json").read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(output / "render_scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=1080, width=1920) as renderer:
        camera = overview_camera()
        xmin, ymin, xmax, ymax = scene["bounds"]
        camera.lookat[:] = [(xmin+xmax)/2, (ymin+ymax)/2, .5]
        camera.distance = max(xmax-xmin, ymax-ymin) * 1.15
        renderer.update_scene(data, camera=camera)
        frame = renderer.render()
        cv2.imwrite(str(output / "overview.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        camera = overview_camera()
        camera.lookat[:] = [*scene["start"][:2], .8]
        camera.distance, camera.azimuth, camera.elevation = 4.8, 45, -24
        renderer.update_scene(data, camera=camera)
        frame = renderer.render()
    destination = output / "pose_snapshot.png"
    cv2.imwrite(str(destination), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    return destination


def replay_timeline(rows: list[dict], fps: int) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray([float(row["time_s"]) for row in rows])
    if fps <= 0 or not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Replay needs increasing finite feedback times and positive fps")
    targets = times[0] + np.arange(math.ceil((times[-1] - times[0]) * fps) + 1) / fps
    indices = np.searchsorted(times, targets, side="right") - 1
    indices = np.minimum(indices, len(rows) - 1)
    return targets, indices


def floor_path(renderer, points, rgba, width):
    points = np.array(points, dtype=float, copy=True).reshape(-1, 3)
    points[:, 2] = .025
    for first, second in pairwise(points):
        if np.linalg.norm(second - first) < 1e-9:
            continue
        if renderer.scene.ngeom >= renderer.scene.maxgeom:
            raise RuntimeError("Replay path exhausted MuJoCo scene capacity")
        geom = renderer.scene.geoms[renderer.scene.ngeom]
        # Path lines stay outside the shadow-casting pass used by solid meshes.
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE,
                           np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba, dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, first, second)
        renderer.scene.ngeom += 1


def replay_minimap(frame, scene, row, global_path, local_path, trail):
    left, top, width, height = 1470, 28, 420, 290
    panel = np.full((height, width, 3), (44, 52, 58), dtype=np.uint8)
    x0, y0, x1, y1 = scene["bounds"]

    def pixel(point):
        return (int(16 + (point[0] - x0) / (x1 - x0) * (width - 32)),
                int(height - 16 - (point[1] - y0) / (y1 - y0) * (height - 32)))

    for box in scene["obstacles"]:
        center, half = np.asarray(box["center"]), np.asarray(box["half_size"])
        cv2.rectangle(panel, pixel(center - half), pixel(center + half), (105, 116, 123), -1)
    for path, color, thickness in [(global_path, (235, 144, 60), 2), (local_path, (104, 231, 99), 3),
                                   (trail, (230, 234, 235), 1)]:
        if len(path) > 1:
            cv2.polylines(panel, [np.asarray([pixel(point) for point in path])], False, color, thickness, cv2.LINE_AA)
    position = [float(row["x_m"]), float(row["y_m"])]
    cv2.circle(panel, pixel(position), 7, (245, 245, 245), -1, cv2.LINE_AA)
    heading = float(row["yaw_rad"])
    cv2.line(panel, pixel(position), pixel([position[0] + .65 * math.cos(heading), position[1] + .65 * math.sin(heading)]),
             (60, 191, 245), 2, cv2.LINE_AA)
    frame[top:top + height, left:left + width] = panel


def render_replay(run: Path, scene_dir: Path = DEFAULT_OUTPUT, *, fps: int = 25) -> dict:
    run, scene_dir = Path(run), Path(scene_dir)
    feedback = run / "feedback.csv"
    rows = []
    if feedback.exists():
        with feedback.open(newline="", encoding="utf-8") as source:
            rows = list(csv.DictReader(source))
    report = {"run": str(run), "fps": fps, "physical_simulation": False, "pose_interpolation": False}
    if not rows:
        render_snapshot(scene_dir)
        report.update(status="no_feedback", frames=0, snapshot=str(scene_dir / "pose_snapshot.png"))
        (run / "replay-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return report
    _, indices = replay_timeline(rows, fps)
    selected_ticks = {int(rows[index]["tick"]) for index in indices}
    with (run / "global-route.csv").open(newline="", encoding="utf-8") as source:
        global_path = [[float(row[key]) for key in ("x_m", "y_m", "z_m")] for row in csv.DictReader(source)]
    local_paths = {}
    with (run / "feedback-previews.csv").open(newline="", encoding="utf-8") as source:
        for point in csv.DictReader(source):
            tick = int(point["tick"])
            if tick in selected_ticks:
                local_paths.setdefault(tick, []).append([float(point[key]) for key in ("x_m", "y_m", "z_m")])
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("Existing ffmpeg executable is required")
    destination = run / "navigation.mp4"
    if destination.exists():
        raise FileExistsError(destination)
    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "bgr24",
               "-video_size", "1920x1080", "-framerate", str(fps), "-i", "pipe:0", "-an", "-c:v", "libx264",
               "-preset", "fast", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(destination)]
    scene = json.loads((scene_dir / "scene.json").read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(scene_dir / "render_scene.xml"))
    # Hide ceiling decorations in the observer cutaway; keep lights and all obstacles.
    for index in range(model.ngeom):
        if model.geom(index).name.startswith(("ceiling_beam_", "lamp_")):
            model.geom_rgba[index, 3] = 0
    data = mujoco.MjData(model)
    mocap = int(model.body_mocapid[model.body("tora").id])
    camera = overview_camera()
    camera.distance, camera.azimuth, camera.elevation = 6.0, 45, -45
    position_keys = ("x_m", "y_m", "z_m")
    trajectory = np.asarray([[float(row[key]) for key in position_keys] for row in rows])
    with (run / "ffmpeg.log").open("w", encoding="utf-8") as error_log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=error_log)
        try:
            with mujoco.Renderer(model, height=1080, width=1920, max_geom=12000) as renderer:
                for frame_index, source_index in enumerate(indices):
                    row = rows[source_index]
                    position = trajectory[source_index]
                    yaw = float(row["yaw_rad"])
                    data.mocap_pos[mocap] = position
                    data.mocap_quat[mocap] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
                    mujoco.mj_forward(model, data)
                    camera.lookat[:] = np.add(position, [0, 0, .65])
                    renderer.update_scene(data, camera=camera)
                    local = local_paths.get(int(row["tick"]), [])
                    trail = trajectory[:source_index + 1:5]
                    floor_path(renderer, global_path, [.18, .48, .94, 1], 3)
                    floor_path(renderer, local, [.22, .90, .24, 1], 4)
                    frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                    replay_minimap(frame, scene, row, global_path, local, trail)
                    process.stdin.write(frame.tobytes())
                    if frame_index in {0, len(indices) // 2, len(indices) - 1}:
                        cv2.imwrite(str(run / f"frame_{frame_index:05d}.png"), frame)
        finally:
            process.stdin.close()
            returncode = process.wait()
        if returncode:
            raise RuntimeError(f"ffmpeg failed with exit code {returncode}; see {run / 'ffmpeg.log'}")
    report.update(status="rendered", frames=len(indices), source_rows=len(rows),
                  source_start_s=float(rows[0]["time_s"]), source_end_s=float(rows[-1]["time_s"]),
                  duration_s=len(indices) / fps, goal_reached=rows[-1].get("goal_reached") == "1",
                  global_route_points=len(global_path), video=str(destination),
                  final_reason=rows[-1].get("reason"), final_geometry_reason=rows[-1].get("final_geometry_reason"))
    (run / "replay-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--fps", type=int, default=25)
    args = parser.parse_args()
    if args.run:
        print(json.dumps(render_replay(args.run, args.output, fps=args.fps)))
        return
    scene = build_scene(args.output)
    snapshot = render_snapshot(args.output)
    print(json.dumps({"output": str(args.output), "obstacles": len(scene["obstacles"]), "snapshot": str(snapshot)}))


if __name__ == "__main__":
    main()
