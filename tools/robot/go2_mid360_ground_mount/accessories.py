"""V6 PLA conformal shim surfaces and the D435i envelope, in millimetres."""

import numpy as np
from assembly_geometry import read_collada
from generate import ASSETS, extrude, inside_convex, rounded_rectangle


def head_heights(points):
    body = read_collada(ASSETS / "dae/base.dae") * 1000
    keep = np.all((body[:, :, :2].min(axis=1) <= points.max(axis=0)) &
                  (body[:, :, :2].max(axis=1) >= points.min(axis=0)), axis=1)
    body = body[keep]
    lower, upper = body[:, :, :2].min(axis=1), body[:, :, :2].max(axis=1)
    heights = []
    for point in points:
        candidates = body[np.all((lower <= point) & (upper >= point), axis=1)]
        a, b, c = candidates[:, 0], candidates[:, 1], candidates[:, 2]
        e1, e2, delta = b - a, c - a, point - a[:, :2]
        determinant = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        valid = np.abs(determinant) > 1e-9
        u, v = np.zeros(len(a)), np.zeros(len(a))
        u[valid] = (delta[valid, 0] * e2[valid, 1] - delta[valid, 1] * e2[valid, 0]) / determinant[valid]
        v[valid] = (e1[valid, 0] * delta[valid, 1] - e1[valid, 1] * delta[valid, 0]) / determinant[valid]
        valid &= (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8)
        if not valid.any():
            raise ValueError(f"No head surface beneath support at {point}")
        heights.append(np.max((a[:, 2] + u * e1[:, 2] + v * e2[:, 2])[valid]))
    return np.array(heights)


def support_pads(p):
    parts = {}
    outline = rounded_rectangle(p["saddle_size_mm"], 4)
    outline[:, 0] += p["base_center_x_mm"]
    outline = np.vstack([np.linspace(a, b, max(1, int(np.ceil(np.linalg.norm(b - a) / 2))), endpoint=False)
                         for a, b in zip(outline, np.roll(outline, -1, axis=0))])
    grid = np.array([(x, y) for x in np.arange(-94, 85, 2) for y in np.arange(-8, 9, 2)])
    grid = grid[inside_convex(grid, outline) & (np.linalg.norm(grid, axis=1) > 3)]
    template, faces = extrude(outline, [(0, 0, p["head_clearance_diameter_mm"] / 2)], 1, grid)
    count = len(template) // 2
    for side, sign in [("left", 1), ("right", -1)]:
        vertices = template.copy()
        vertices[:, 1] += sign * p["head_hole_pitch_mm"] / 2
        world_xy = vertices[:count, :2] + np.array(p["mount_xyz_m"][:2]) * 1000
        surface = head_heights(world_xy) - p["mount_xyz_m"][2] * 1000
        pad_bottom = surface + p["saddle_fit_allowance_mm"]
        vertices[:count, 2] = pad_bottom
        vertices[count:, 2] = pad_bottom + p["saddle_pad_thickness_mm"]
        parts[f"{side}_pad"] = (vertices, faces)
    return parts


def camera_parts(p):
    # Body envelope only; rear-centre origin with X forward, Y left, Z up.
    map_yz = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    camera, faces = extrude(rounded_rectangle([90, 25], 5), [], 25.05)
    return {"d435i_envelope": (camera @ map_yz.T, faces)}
