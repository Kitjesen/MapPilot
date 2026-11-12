"""Run native global/SCAN planning on the garage's exact 3D geometry.

This is an ideal planar feedback experiment, without DDS, a sensor pipeline,
wheel physics, or a hardware driver. The renderer only replays its output.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from lingtu.assembly.config.native_navigation import compile_native_nav_config

SCENE_DIR = ROOT / "artifacts/nav/tora-garage-20261106"
PROFILE = ROOT / "config/robots/paxini/tora_double_one_x/navigation.yaml"
BINARY = ROOT / "build/body-execution-20261022/native/tests/nav/Release/benchmark_saved_route.exe"


def native_environment(profile: Path) -> dict[str, str]:
    geometry = yaml.safe_load(profile.read_text(encoding="utf-8"))["geometry"]
    config = {f"{key}_m": geometry[key] for key in (
        "vehicle_length", "vehicle_width", "vehicle_height", "collision_cylinder_radius",
        "collision_cylinder_offset", "collision_hard_margin", "collision_clearance_below",
        "collision_clearance_above",
    )}
    config.update({f"octoplanner3d_{key}_m": geometry[key]
                   for key in ("support_height", "support_height_tolerance")})
    config.update(native_control_mode="autonomy", native_nav={"local_planner": "scan", "tick_hz": 100.0})
    return dict(compile_native_nav_config("nav", config).environment)


def write_scene_map_input(scene: dict, path: Path) -> None:
    xmin, ymin, xmax, ymax = scene["bounds"]
    floor = {"center": [(xmin+xmax)/2, (ymin+ymax)/2, -.05],
             "half_size": [(xmax-xmin)/2, (ymax-ymin)/2, .05]}
    boxes = [floor, *scene["obstacles"]]
    lines = [f"0.05 {xmin-.2} {ymin-.2} -0.1 {xmax+.2} {ymax+.2} 3.6 {len(boxes)}"]
    for box in boxes:
        center, half = box["center"], box["half_size"]
        lines.append(" ".join(str(value) for value in (
            *(c-h for c, h in zip(center, half, strict=True)),
            *(c+h for c, h in zip(center, half, strict=True)),
        )))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, default=SCENE_DIR)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--binary", type=Path, default=BINARY)
    parser.add_argument("--run", required=True, help="New evidence directory name inside scene/runs")
    parser.add_argument("--seconds", type=float, default=90.0)
    args = parser.parse_args()
    scene = json.loads((args.scene_dir / "scene.json").read_text(encoding="utf-8"))
    run = args.scene_dir / "runs" / args.run
    run.mkdir(parents=True, exist_ok=False)
    environment = native_environment(args.profile)
    (run / "environment.json").write_text(json.dumps(environment, indent=2) + "\n", encoding="utf-8")
    (run / "navigation.yaml").write_bytes(args.profile.read_bytes())
    map_file = args.scene_dir / "map.ot"
    if not map_file.exists():
        scene_input = args.scene_dir / "scene-map.txt"
        write_scene_map_input(scene, scene_input)
        with (args.scene_dir / "map-build.log").open("w", encoding="utf-8") as log:
            subprocess.run([str(args.binary), "--build-scene", str(scene_input), str(map_file)],
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=240)
    command = [str(args.binary), str(map_file), *(str(v) for v in scene["start"][:3]),
               *(str(v) for v in scene["goal"][:3]), str(run), str(args.seconds), str(scene["goal"][3])]
    (run / "invocation.json").write_text(json.dumps(command, indent=2) + "\n", encoding="utf-8")
    with (run / "native.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, env={**os.environ, **environment}, stdout=log,
                                stderr=subprocess.STDOUT, check=False, timeout=360)
    if not (run / "summary.json").is_file():
        raise RuntimeError(f"Native probe exited {result.returncode}; see {run / 'native.log'}")
    print((run / "summary.json").read_text(encoding="utf-8"))
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
