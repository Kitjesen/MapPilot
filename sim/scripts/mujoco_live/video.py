"""Video rendering helpers for the MuJoCo live gate."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from runtime.msgs.numpy_compat import is_numpy_array, np
from runtime.runtime_interface import TOPICS, topic_default_frame_id

SIM_FASTLIO_LIVE_MAP_FRAME_ID = topic_default_frame_id(TOPICS.odometry)

def _sample_points(points: np.ndarray, max_points: int) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0:
        return np.empty((0, 3), dtype=np.float32)
    if pts.shape[1] < 3:
        return np.empty((0, 3), dtype=np.float32)
    pts = pts[:, :3]
    if pts.shape[0] <= max_points:
        return pts.copy()
    step = max(1, int(np.ceil(pts.shape[0] / max_points)))
    return pts[::step][:max_points].copy()

def _render_mujoco_overview(engine: Any, state: dict[str, Any]) -> np.ndarray | None:
    """Render a third-person MuJoCo frame for validation videos.

    This is visual evidence only; sensor and SLAM validation still comes from
    the raw LiDAR/IMU and Fast-LIO topics.
    """
    if state.get("disabled"):
        return None
    try:
        import mujoco  # type: ignore

        model = getattr(engine, "_model", None)
        data = getattr(engine, "_data", None)
        if model is None or data is None:
            return None
        renderer = state.get("renderer")
        camera = state.get("camera")
        if renderer is None or camera is None:
            renderer = mujoco.Renderer(model, 360, 640)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.distance = 16.0
            camera.azimuth = -55.0
            camera.elevation = -58.0
            state["renderer"] = renderer
            state["camera"] = camera
        robot_state = engine.get_robot_state()
        robot_pos = np.asarray(robot_state.position[:3], dtype=np.float64)
        target = np.array([robot_pos[0], robot_pos[1], max(robot_pos[2], 0.5) + 0.8], dtype=np.float64)
        prev = state.get("lookat")
        if prev is None:
            lookat = target
        else:
            lookat = np.asarray(prev, dtype=np.float64) * 0.8 + target * 0.2
        state["lookat"] = lookat
        camera.lookat[:] = lookat
        renderer.update_scene(data, camera=camera)
        return renderer.render().copy()
    except Exception as exc:
        state["disabled"] = True
        state["error"] = f"{type(exc).__name__}: {exc}"
        return None

def _write_stage_video(
    samples: list[dict[str, Any]],
    report: dict[str, Any],
    out_path: Path,
    *,
    fps: float = 8.0,
) -> None:
    """Render an offline acceptance video from the same live gate samples."""
    if not samples:
        return
    try:
        import cv2  # type: ignore
    except Exception as exc:
        report["video_error"] = f"opencv unavailable: {exc}"
        return

    out_path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1280, 720
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, max(float(fps), 1.0), (width, height))
    if not writer.isOpened():
        report["video_error"] = f"failed to open video writer: {out_path}"
        return

    def finite_series(key: str) -> np.ndarray:
        values = np.asarray(
            [sample.get(key, [np.nan, np.nan]) for sample in samples],
            dtype=np.float32,
        )
        if values.ndim != 2 or values.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float32)
        values = values[:, :2]
        return values[np.isfinite(values).all(axis=1)]

    sim_series = finite_series("sim_xy")
    odom_series = finite_series("odom_xy")

    def finite_series_xyz(key: str) -> np.ndarray:
        values = np.asarray(
            [sample.get(key, [np.nan, np.nan, np.nan]) for sample in samples],
            dtype=np.float32,
        )
        if values.ndim != 2 or values.shape[1] < 3:
            return np.empty((0, 3), dtype=np.float32)
        values = values[:, :3]
        return values[np.isfinite(values).all(axis=1)]

    sim_xyz_series = finite_series_xyz("sim_xyz")
    odom_xyz_series = finite_series_xyz("odom_xyz")
    z_offset = 0.0
    if len(sim_xyz_series) > 0 and len(odom_xyz_series) > 0:
        z_offset = float(odom_xyz_series[0, 2] - sim_xyz_series[0, 2])

    def fit_sim_to_odom() -> dict[str, Any]:
        count = min(len(sim_series), len(odom_series))
        if count <= 0:
            return {
                "method": "identity",
                "rotation_rad": 0.0,
                "translation_xy": [0.0, 0.0],
                "rmse_m": None,
                "max_error_m": None,
                "sample_count": 0,
            }
        src = np.asarray(sim_series[:count], dtype=np.float64)
        dst = np.asarray(odom_series[:count], dtype=np.float64)
        finite = np.isfinite(src).all(axis=1) & np.isfinite(dst).all(axis=1)
        src = src[finite]
        dst = dst[finite]
        if len(src) < 3 or float(np.linalg.norm(src[-1] - src[0])) < 0.2:
            translation = dst[0] - src[0]
            transformed = src + translation
            errors = np.linalg.norm(transformed - dst, axis=1)
            return {
                "method": "translation_only",
                "rotation_rad": 0.0,
                "translation_xy": [float(translation[0]), float(translation[1])],
                "rmse_m": float(np.sqrt(np.mean(errors * errors))) if len(errors) else None,
                "max_error_m": float(np.max(errors)) if len(errors) else None,
                "sample_count": int(len(src)),
            }
        src_center = np.mean(src, axis=0)
        dst_center = np.mean(dst, axis=0)
        src0 = src - src_center
        dst0 = dst - dst_center
        h = src0.T @ dst0
        u, _s, vt = np.linalg.svd(h)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T
        translation = dst_center - rotation @ src_center
        transformed = (rotation @ src.T).T + translation
        errors = np.linalg.norm(transformed - dst, axis=1)
        return {
            "method": "se2_best_fit",
            "rotation_rad": float(math.atan2(rotation[1, 0], rotation[0, 0])),
            "translation_xy": [float(translation[0]), float(translation[1])],
            "rmse_m": float(np.sqrt(np.mean(errors * errors))),
            "max_error_m": float(np.max(errors)),
            "sample_count": int(len(src)),
        }

    alignment = fit_sim_to_odom()
    theta = float(alignment.get("rotation_rad") or 0.0)
    c, s = math.cos(theta), math.sin(theta)
    alignment_rotation = np.asarray([[c, -s], [s, c]], dtype=np.float32)
    alignment_translation = np.asarray(
        alignment.get("translation_xy") or [0.0, 0.0],
        dtype=np.float32,
    )

    def transform_sim_xy(xy: Any) -> np.ndarray:
        point = np.asarray(xy, dtype=np.float32)
        if point.shape != (2,) or not np.isfinite(point).all():
            return np.array([np.nan, np.nan], dtype=np.float32)
        return alignment_rotation @ point + alignment_translation

    def transform_sim_xyz(xyz: Any) -> np.ndarray:
        point = np.asarray(xyz, dtype=np.float32)
        if point.shape != (3,) or not np.isfinite(point).all():
            return np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        xy = alignment_rotation @ point[:2] + alignment_translation
        return np.array([xy[0], xy[1], point[2] + z_offset], dtype=np.float32)

    def transform_sim_points(points: Any) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 2:
            return np.empty((0, 3), dtype=np.float32)
        out = pts[:, :3].copy()
        finite = np.isfinite(out[:, :2]).all(axis=1)
        out = out[finite]
        if out.shape[0] <= 0:
            return np.empty((0, 3), dtype=np.float32)
        out[:, :2] = out[:, :2] @ alignment_rotation.T + alignment_translation
        if out.shape[1] >= 3:
            out[:, 2] += float(z_offset)
        return out

    report["video_alignment"] = alignment
    report["video_alignment"]["z_offset_m"] = float(z_offset)

    def bounds_for(key: str) -> tuple[float, float, float, float]:
        chunks: list[np.ndarray] = []
        for sample in samples:
            pts = sample.get(key)
            if is_numpy_array(pts) and pts.size:
                if key == "raw_points":
                    chunks.append(transform_sim_points(pts)[:, :2])
                else:
                    chunks.append(pts[:, :2])
        if key in {"raw_points", "map_points"}:
            aligned_sim = np.asarray(
                [transform_sim_xy(sample.get("sim_xy")) for sample in samples],
                dtype=np.float32,
            )
            aligned_sim = aligned_sim[np.isfinite(aligned_sim).all(axis=1)]
            if aligned_sim.size:
                chunks.append(aligned_sim)
            if odom_series.size:
                chunks.append(odom_series)
        if not chunks:
            return (-5.0, 5.0, -5.0, 5.0)
        xy = np.vstack(chunks)
        finite = np.isfinite(xy).all(axis=1)
        xy = xy[finite]
        if xy.size == 0:
            return (-5.0, 5.0, -5.0, 5.0)
        min_x, min_y = np.min(xy, axis=0)
        max_x, max_y = np.max(xy, axis=0)
        pad = max(1.0, float(max(max_x - min_x, max_y - min_y)) * 0.1)
        return (float(min_x - pad), float(max_x + pad), float(min_y - pad), float(max_y + pad))

    map_bounds = bounds_for("map_points")

    def sim_world_bounds2d() -> tuple[float, float, float, float]:
        chunks: list[np.ndarray] = []
        for sample in samples:
            raw = sample.get("raw_points")
            if is_numpy_array(raw) and raw.size and raw.shape[1] >= 2:
                chunks.append(raw[:, :2])
        if sim_series.size:
            chunks.append(sim_series)
        if not chunks:
            return (-5.0, 5.0, -5.0, 5.0)
        xy = np.vstack(chunks)
        xy = xy[np.isfinite(xy).all(axis=1)]
        if xy.size == 0:
            return (-5.0, 5.0, -5.0, 5.0)
        min_x, min_y = np.min(xy, axis=0)
        max_x, max_y = np.max(xy, axis=0)
        pad = max(1.0, float(max(max_x - min_x, max_y - min_y)) * 0.15)
        return (float(min_x - pad), float(max_x + pad), float(min_y - pad), float(max_y + pad))

    sim_bounds = sim_world_bounds2d()

    traj_chunks: list[np.ndarray] = []
    aligned_sim_all = np.asarray(
        [transform_sim_xy(sample.get("sim_xy")) for sample in samples],
        dtype=np.float32,
    )
    if aligned_sim_all.ndim == 2:
        aligned_sim_all = aligned_sim_all[np.isfinite(aligned_sim_all).all(axis=1)]
        if aligned_sim_all.size:
            traj_chunks.append(aligned_sim_all)
    if odom_series.size:
        traj_chunks.append(odom_series)
    if traj_chunks:
        traj_xy = np.vstack(traj_chunks)
        min_x, min_y = np.min(traj_xy, axis=0)
        max_x, max_y = np.max(traj_xy, axis=0)
        pad = max(0.5, float(max(max_x - min_x, max_y - min_y)) * 0.25)
        traj_bounds = (float(min_x - pad), float(max_x + pad), float(min_y - pad), float(max_y + pad))
    else:
        traj_bounds = (-2.0, 2.0, -2.0, 2.0)
    panel_bg = (28, 31, 36)
    grid_color = (58, 63, 72)
    text = (235, 238, 245)
    yellow = (40, 210, 245)
    green = (110, 220, 130)
    blue = (230, 160, 80)
    red = (80, 90, 245)

    def world_to_px(
        x: float,
        y: float,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
    ) -> tuple[int, int]:
        x0, y0, x1, y1 = rect
        min_x, max_x, min_y, max_y = bounds
        sx = (x - min_x) / max(max_x - min_x, 1e-6)
        sy = (y - min_y) / max(max_y - min_y, 1e-6)
        px = int(x0 + sx * (x1 - x0))
        py = int(y1 - sy * (y1 - y0))
        return px, py

    def draw_panel(frame: np.ndarray, rect: tuple[int, int, int, int], title: str) -> None:
        x0, y0, x1, y1 = rect
        cv2.rectangle(frame, (x0, y0), (x1, y1), panel_bg, -1)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (85, 92, 105), 1)
        for i in range(1, 5):
            x = x0 + int((x1 - x0) * i / 5)
            y = y0 + int((y1 - y0) * i / 5)
            cv2.line(frame, (x, y0), (x, y1), grid_color, 1)
            cv2.line(frame, (x0, y), (x1, y), grid_color, 1)
        cv2.putText(frame, title, (x0 + 14, y0 + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, text, 2)

    def draw_points(
        frame: np.ndarray,
        pts: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
        color: tuple[int, int, int],
        *,
        max_draw: int = 2500,
        radius: int = 0,
    ) -> None:
        if pts.size == 0:
            return
        pts = _sample_points(pts, max_draw)
        for x, y in pts[:, :2]:
            px, py = world_to_px(float(x), float(y), rect, bounds)
            if rect[0] <= px <= rect[2] and rect[1] <= py <= rect[3]:
                if radius > 0:
                    cv2.circle(frame, (px, py), radius, color, -1)
                else:
                    frame[py, px] = color

    def draw_polyline(
        frame: np.ndarray,
        xy: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float],
        color: tuple[int, int, int],
    ) -> None:
        xy = np.asarray(xy, dtype=np.float32)
        if xy.ndim != 2 or xy.shape[0] < 2 or xy.shape[1] < 2:
            return
        xy = xy[:, :2]
        xy = xy[np.isfinite(xy).all(axis=1)]
        if xy.shape[0] < 2:
            return
        pts = [world_to_px(float(x), float(y), rect, bounds) for x, y in xy]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(frame, a, b, color, 2)

    def bounds3d_for(kind: str) -> tuple[float, float, float, float, float, float]:
        chunks: list[np.ndarray] = []
        if kind == "map":
            for sample in samples:
                map_pts = sample.get("map_points")
                if is_numpy_array(map_pts) and map_pts.size:
                    chunks.append(np.asarray(map_pts[:, :3], dtype=np.float32))
                raw_pts = sample.get("raw_points")
                raw_aligned = transform_sim_points(raw_pts)
                if raw_aligned.size:
                    chunks.append(raw_aligned[:, :3])
        if kind == "trajectory":
            aligned_sim = np.asarray(
                [transform_sim_xyz(sample.get("sim_xyz")) for sample in samples],
                dtype=np.float32,
            )
            aligned_sim = aligned_sim[np.isfinite(aligned_sim).all(axis=1)]
            if aligned_sim.size:
                chunks.append(aligned_sim)
            if odom_xyz_series.size:
                chunks.append(odom_xyz_series)
        if not chunks:
            return (-5.0, 5.0, -5.0, 5.0, -1.0, 3.0)
        xyz = np.vstack(chunks)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.size == 0:
            return (-5.0, 5.0, -5.0, 5.0, -1.0, 3.0)
        mins = np.min(xyz, axis=0)
        maxs = np.max(xyz, axis=0)
        spans = np.maximum(maxs - mins, 1e-3)
        pad_xy = max(1.0, float(max(spans[0], spans[1])) * 0.12)
        pad_z = max(0.4, float(spans[2]) * 0.18)
        return (
            float(mins[0] - pad_xy),
            float(maxs[0] + pad_xy),
            float(mins[1] - pad_xy),
            float(maxs[1] + pad_xy),
            float(mins[2] - pad_z),
            float(maxs[2] + pad_z),
        )

    def project_3d(
        xyz: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
    ) -> tuple[np.ndarray, np.ndarray]:
        pts = np.asarray(xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 3:
            return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=np.float32)
        pts = pts[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] <= 0:
            return np.empty((0, 2), dtype=np.int32), np.empty((0,), dtype=np.float32)
        min_x, max_x, min_y, max_y, min_z, max_z = bounds
        center = np.array(
            [(min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5],
            dtype=np.float32,
        )
        span = max(max_x - min_x, max_y - min_y, (max_z - min_z) * 2.4, 1e-3)
        p = pts - center
        yaw = math.radians(-42.0)
        pitch = math.radians(58.0)
        rz = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        rx = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(pitch), -math.sin(pitch)],
                [0.0, math.sin(pitch), math.cos(pitch)],
            ],
            dtype=np.float32,
        )
        q = (p @ rz.T) @ rx.T
        x0, y0, x1, y1 = rect
        scale = min(x1 - x0, y1 - y0) / (span * 1.25)
        px = ((x0 + x1) * 0.5 + q[:, 0] * scale).astype(np.int32)
        py = ((y0 + y1) * 0.55 - q[:, 1] * scale).astype(np.int32)
        return np.stack([px, py], axis=1), q[:, 2].astype(np.float32)

    def draw_points_3d(
        frame: np.ndarray,
        pts: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
        color: tuple[int, int, int],
        *,
        max_draw: int = 3500,
        radius: int = 1,
    ) -> None:
        pts = _sample_points(np.asarray(pts, dtype=np.float32), max_draw)
        if pts.size == 0:
            return
        pix, depth = project_3d(pts, rect, bounds)
        if pix.size == 0:
            return
        order = np.argsort(depth)
        x0, y0, x1, y1 = rect
        for px, py in pix[order]:
            if x0 <= int(px) <= x1 and y0 <= int(py) <= y1:
                if radius > 0:
                    cv2.circle(frame, (int(px), int(py)), radius, color, -1)
                else:
                    frame[int(py), int(px)] = color

    def draw_polyline_3d(
        frame: np.ndarray,
        xyz: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
        color: tuple[int, int, int],
    ) -> None:
        pts = np.asarray(xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] < 3:
            return
        pts = pts[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] < 2:
            return
        pix, _depth = project_3d(pts, rect, bounds)
        for a, b in zip(pix[:-1], pix[1:]):
            cv2.line(frame, tuple(int(v) for v in a), tuple(int(v) for v in b), color, 2)

    def draw_axes_3d(
        frame: np.ndarray,
        rect: tuple[int, int, int, int],
        bounds: tuple[float, float, float, float, float, float],
    ) -> None:
        axis = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        pix, _depth = project_3d(axis, rect, bounds)
        if pix.shape[0] != 4:
            return
        origin = tuple(int(v) for v in pix[0])
        axes = [
            (pix[1], (60, 80, 245), "x"),
            (pix[2], (90, 220, 90), "y"),
            (pix[3], (245, 170, 80), "z"),
        ]
        for point, color, label in axes:
            end = tuple(int(v) for v in point)
            cv2.line(frame, origin, end, color, 2)
            cv2.putText(frame, label, end, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    map_bounds3d = bounds3d_for("map")
    traj_bounds3d = bounds3d_for("trajectory")
    sim_traj: list[list[float]] = []
    odom_traj: list[list[float]] = []
    sim_truth_xy: list[list[float]] = []
    scene_rect = (24, 56, 624, 356)
    map_rect = (656, 56, 1256, 356)
    traj_rect = (24, 398, 624, 696)
    info_rect = (656, 398, 1256, 696)

    for idx, sample in enumerate(samples):
        frame = np.full((height, width, 3), (18, 20, 24), dtype=np.uint8)
        t_s = float(sample.get("t_s") or 0.0)
        cv2.putText(
            frame,
            "LingTu MuJoCo + MID360 + Fast-LIO live validation",
            (24, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            text,
            2,
        )
        cv2.putText(
            frame,
            f"t={t_s:05.1f}s  sample={idx + 1}/{len(samples)}  SE2 rmse={alignment.get('rmse_m') or 0.0:.2f}m",
            (900, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            text,
            2,
        )

        draw_panel(frame, scene_rect, "MuJoCo environment render")
        overview = sample.get("overview_rgb")
        if is_numpy_array(overview) and overview.size:
            x0, y0, x1, y1 = scene_rect
            rgb = np.asarray(overview, dtype=np.uint8)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb.ndim == 3 else rgb
            bgr = cv2.resize(bgr, (x1 - x0 - 2, y1 - y0 - 2), interpolation=cv2.INTER_AREA)
            frame[y0 + 1 : y1 - 1, x0 + 1 : x1 - 1] = bgr
            cv2.putText(
                frame,
                "chase MuJoCo render, camera follows robot",
                (x0 + 14, y1 - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                text,
                1,
            )
        else:
            cv2.putText(frame, "MuJoCo renderer unavailable", (48, 206), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text, 2)

        inset = (scene_rect[2] - 232, scene_rect[3] - 132, scene_rect[2] - 12, scene_rect[3] - 12)
        cv2.rectangle(frame, (inset[0], inset[1]), (inset[2], inset[3]), (20, 22, 26), -1)
        cv2.rectangle(frame, (inset[0], inset[1]), (inset[2], inset[3]), (110, 118, 130), 1)
        raw_for_inset = sample.get("raw_points", np.empty((0, 3)))
        draw_points(frame, raw_for_inset, inset, sim_bounds, (80, 85, 95), max_draw=800)
        sim_xy_for_inset = sample.get("sim_xy")
        if sim_xy_for_inset is not None and np.isfinite(sim_xy_for_inset).all():
            sim_truth_xy.append([float(sim_xy_for_inset[0]), float(sim_xy_for_inset[1])])
        draw_polyline(frame, np.asarray(sim_truth_xy, dtype=np.float32), inset, sim_bounds, blue)
        if sim_truth_xy:
            px, py = world_to_px(sim_truth_xy[-1][0], sim_truth_xy[-1][1], inset, sim_bounds)
            cv2.circle(frame, (px, py), 5, (60, 60, 245), -1)
            cv2.putText(frame, "robot", (px + 7, py - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 245), 1)
        cv2.putText(
            frame,
            "MuJoCo truth trail",
            (inset[0] + 8, inset[1] + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            text,
            1,
        )

        draw_panel(
            frame,
            map_rect,
            f"3D Fast-LIO map cloud ({TOPICS.map_cloud}, {SIM_FASTLIO_LIVE_MAP_FRAME_ID})",
        )
        draw_axes_3d(frame, map_rect, map_bounds3d)
        draw_points_3d(
            frame,
            sample.get("map_points", np.empty((0, 3))),
            map_rect,
            map_bounds3d,
            green,
            max_draw=4500,
            radius=1,
        )
        raw_points = sample.get("raw_points", np.empty((0, 3)))
        raw_points_aligned = transform_sim_points(raw_points)
        draw_points_3d(
            frame,
            raw_points_aligned,
            map_rect,
            map_bounds3d,
            yellow,
            max_draw=1200,
            radius=1,
        )
        moving_points = sample.get("moving_obstacle_points", np.empty((0, 3)))
        moving_points_aligned = transform_sim_points(moving_points)
        draw_points_3d(
            frame,
            moving_points_aligned,
            map_rect,
            map_bounds3d,
            red,
            max_draw=1200,
            radius=2,
        )
        cv2.putText(
            frame,
            "green=Fast-LIO accumulated  yellow=current MID360 scan  red=dynamic obstacle returns",
            (map_rect[0] + 14, map_rect[3] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            text,
            1,
        )

        draw_panel(frame, traj_rect, "3D trajectory consistency")
        draw_axes_3d(frame, traj_rect, traj_bounds3d)
        sim_xy = sample.get("sim_xy")
        odom_xy = sample.get("odom_xy")
        sim_xyz = sample.get("sim_xyz")
        odom_xyz = sample.get("odom_xyz")
        if sim_xyz is not None and np.isfinite(sim_xyz).all():
            aligned = transform_sim_xyz(sim_xyz)
            if np.isfinite(aligned).all():
                sim_traj.append([float(aligned[0]), float(aligned[1]), float(aligned[2])])
        elif sim_xy is not None and np.isfinite(sim_xy).all():
            aligned_xy = transform_sim_xy(sim_xy)
            if np.isfinite(aligned_xy).all():
                sim_traj.append([float(aligned_xy[0]), float(aligned_xy[1]), 0.0])
        if odom_xyz is not None and np.isfinite(odom_xyz).all():
            odom_traj.append([float(odom_xyz[0]), float(odom_xyz[1]), float(odom_xyz[2])])
        elif odom_xy is not None and np.isfinite(odom_xy).all():
            odom_traj.append([float(odom_xy[0]), float(odom_xy[1]), 0.0])
        draw_polyline_3d(frame, np.asarray(sim_traj, dtype=np.float32), traj_rect, traj_bounds3d, blue)
        draw_polyline_3d(frame, np.asarray(odom_traj, dtype=np.float32), traj_rect, traj_bounds3d, red)
        cv2.putText(
            frame,
            "blue=MuJoCo truth -> odom display fit  red=Fast-LIO odom",
            (42, 682),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            text,
            1,
        )

        draw_panel(frame, info_rect, "Gate evidence")
        lines = [
            f"SLAM state: {sample.get('slam_state', 'UNKNOWN')}",
            f"raw points/frame: {int(sample.get('raw_point_count') or 0)}",
            f"map cloud samples: {int(sample.get('map_cloud_count') or 0)}",
            f"sim moved: {float(sample.get('sim_moved_m') or 0.0):.2f} m",
            f"Fast-LIO moved: {float(sample.get('odom_moved_m') or 0.0):.2f} m",
            f"map XY area: {float(sample.get('map_area_m2') or 0.0):.1f} m2",
            f"traj SE2 rmse/max: {alignment.get('rmse_m') or 0.0:.2f}/{alignment.get('max_error_m') or 0.0:.2f} m",
            f"nav outputs: odom={int(sample.get('nav_odom_count') or 0)} map={int(sample.get('nav_map_count') or 0)}",
            f"moving obstacles visible: {int(sample.get('moving_obstacle_box_count') or 0)}",
            f"frames: odom={sample.get('nav_odom_frame', 'odom')} map_cloud={sample.get('nav_map_frame', 'odom')}",
            "hardware cmd_vel: disabled",
        ]
        y = 438
        for line in lines:
            cv2.putText(frame, line, (678, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, text, 2)
            y += 30
        writer.write(frame)

    writer.release()
    report["video_path"] = str(out_path)
    report["video_frame_count"] = len(samples)
