#!/usr/bin/env python3
"""Render screenshots for MuJoCo/Fast-LIO2 SLAM validation artifacts.

This script is offline and simulation-only. It renders a MuJoCo scene snapshot,
a LiDAR point-cloud top-down image, and a compact report panel from a
`mujoco/live_gate.py` JSON report.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco.live_gate import _build_engine, _parse_start, _resolve_world, _scene_start  # noqa: E402


def _save_rgb(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v2 as imageio  # type: ignore

        imageio.imwrite(path, image)
        return
    except Exception:
        pass
    try:
        from PIL import Image  # type: ignore

        Image.fromarray(image).save(path)
        return
    except Exception:
        pass
    # Last-resort binary PPM writer. Keep the requested suffix but write a valid
    # portable pixmap so there is still a visual artifact.
    rgb = np.asarray(image, dtype=np.uint8)
    header = f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode("ascii")
    path.write_bytes(header + rgb.tobytes())


def _render_scene(engine: Any, out: Path, *, width: int, height: int) -> None:
    import mujoco

    renderer = mujoco.Renderer(engine.model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    state = engine.get_robot_state()
    camera.lookat[:] = [float(state.position[0] + 1.5), float(state.position[1] + 1.0), float(state.position[2] + 0.5)]
    camera.distance = 10.0
    camera.azimuth = -135.0
    camera.elevation = -35.0
    renderer.update_scene(engine.data, camera)
    _save_rgb(out, renderer.render().copy())
    renderer.close()


def _plot_lidar(engine: Any, out: Path) -> dict[str, Any]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cloud = engine.get_lidar_points()
    pts = np.asarray(cloud, dtype=np.float32)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 7), dpi=140)
    if pts.size:
        ax.scatter(pts[:, 0], pts[:, 1], s=0.8, c=pts[:, 2], cmap="viridis", alpha=0.65, linewidths=0)
    state = engine.get_robot_state()
    ax.scatter([state.position[0]], [state.position[1]], s=80, marker="x", c="red", label="robot")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("MuJoCo MID-360 LiDAR Snapshot (world XY)")
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return {
        "point_count": int(len(pts)) if pts.ndim == 2 else 0,
        "path": str(out),
    }


def _plot_report(report: dict[str, Any], out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    final = report.get("final_bridge_status") or {}
    outputs = report.get("outputs") or {}
    counts = report.get("counts") or {}
    rows = [
        ("Gate OK", report.get("ok")),
        ("Final state", final.get("state")),
        ("Health source", final.get("health_source")),
        ("Map state", final.get("map_state")),
        ("Live LiDAR", report.get("live_mujoco_lidar_verified")),
        ("Live IMU", report.get("live_mujoco_imu_verified")),
        ("Fast-LIO2 odom", outputs.get("fastlio2_odometry")),
        ("Fast-LIO2 cloud_map", outputs.get("fastlio2_cloud_map")),
        ("Published clouds", counts.get("cloud_published")),
        ("Published IMU", counts.get("imu_published")),
        ("Sim moved / m", report.get("sim_moved_m")),
        ("Fast-LIO2 moved / m", report.get("fastlio2_moved_m")),
        ("Real robot motion", report.get("real_robot_motion")),
    ]
    fig, ax = plt.subplots(figsize=(9, 6), dpi=150)
    ax.axis("off")
    ax.set_title("MuJoCo Live Fast-LIO2 SLAM Gate", loc="left", fontsize=16, weight="bold")
    y = 0.9
    for key, value in rows:
        color = "#0f7b32" if value is True or str(value) == "TRACKING" else "#222222"
        if value is False or str(value) in {"LOST", "UNINIT"}:
            color = "#a83b2d"
        ax.text(0.02, y, key, fontsize=11, color="#444444", transform=ax.transAxes)
        ax.text(0.45, y, str(value), fontsize=11, color=color, transform=ax.transAxes, weight="bold")
        y -= 0.065
    ax.text(0.02, 0.02, str(report.get("world", "")), fontsize=8, color="#666666", transform=ax.transAxes)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)


def render_artifacts(
    *,
    world: Path,
    start: list[float] | None,
    report_path: Path,
    output_dir: Path,
    mujoco_memory: str,
    width: int,
    height: int,
) -> dict[str, Any]:
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else {}
    start = start or report.get("start_position") or _scene_start(world)
    engine = _build_engine(
        world=world,
        drive_mode="kinematic",
        n_rays=6400,
        start=start,
        mujoco_memory=mujoco_memory,
    )
    try:
        scene_png = output_dir / "scene_screenshot.png"
        lidar_png = output_dir / "lidar_snapshot.png"
        report_png = output_dir / "slam_gate_summary.png"
        _render_scene(engine, scene_png, width=width, height=height)
        lidar_meta = _plot_lidar(engine, lidar_png)
        _plot_report(report, report_png)
    finally:
        engine.close()

    manifest = {
        "world": str(world),
        "start_position": start,
        "report": str(report_path),
        "screenshots": {
            "scene": str(scene_png),
            "lidar_snapshot": str(lidar_png),
            "slam_gate_summary": str(report_png),
        },
        "lidar": lidar_meta,
    }
    manifest_path = output_dir / "screenshots_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="factory_scene")
    parser.add_argument("--start", default="")
    parser.add_argument("--report", required=True)
    parser.add_argument("--output-dir", default="artifacts/mujoco_fastlio2_live/screenshots")
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1000)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    manifest = render_artifacts(
        world=_resolve_world(args.world),
        start=_parse_start(args.start),
        report_path=Path(args.report),
        output_dir=Path(args.output_dir),
        mujoco_memory=args.mujoco_memory,
        width=args.width,
        height=args.height,
    )
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
