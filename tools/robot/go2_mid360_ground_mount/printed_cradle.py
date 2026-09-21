"""Bounded-memory implicit CSG for the single-piece PLA cradle; units are mm.

Uses scikit-image already present in the local conda environment. The sampling
pitch is an explicit CAD approximation, not a printer or fit tolerance.
"""

from itertools import pairwise

import numpy as np
from accessories import head_heights, support_pads
from generate import circle, extrude, rounded_rectangle
from scipy.ndimage import map_coordinates
from skimage.measure import marching_cubes


def rounded2(x, y, cx, cy, hx, hy, radius):
    qx, qy = np.abs(x - cx) - hx + radius, np.abs(y - cy) - hy + radius
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - radius


def slab(value, low, high):
    return np.maximum(low - value, value - high)


def join(a, b, radius=2):
    h = np.maximum(radius - np.abs(a - b), 0) / radius
    return np.minimum(a, b) - h * h * radius / 4


def build(p, deck_position, deck_rotation, sensor_holes):
    spacing = p["implicit_mesh_pitch_mm"]
    axes = [np.arange(a, b + spacing, spacing, dtype=np.float32)
            for a, b in [(-102, 100), (-52, 52), (-48, 94)]]
    x, y, z = axes[0][:, None, None], axes[1][None, :, None], axes[2][None, None, :]
    c, s = deck_rotation[0, 0], deck_rotation[0, 2]
    u = c * (x - deck_position[0]) - s * (z - deck_position[2])
    w = s * (x - deck_position[0]) + c * (z - deck_position[2])
    # Two long conformal runners replace the rectangular floor and box walls.
    gx, gy = np.arange(-100, 97, 2), np.arange(-42, 43, 2)
    points = np.array([(a, b) for a in gx for b in gy], dtype=float)
    points += np.array(p["mount_xyz_m"][:2]) * 1000
    height = head_heights(points).reshape(len(gx), len(gy)) - p["mount_xyz_m"][2] * 1000
    xx, yy = np.meshgrid((axes[0] - gx[0]) / 2, (axes[1] - gy[0]) / 2, indexing="ij")
    surface = map_coordinates(height, [xx, yy], order=1, mode="nearest").astype(np.float32)[:, :, None]
    bottom = surface + p["saddle_fit_allowance_mm"] + p["saddle_pad_thickness_mm"]
    top = surface + 16
    solid = np.full((len(axes[0]), len(axes[1]), len(axes[2])), 1000, dtype=np.float32)
    for sign in [-1, 1]:
        rail = np.maximum(rounded2(x, y, -5, sign * 29.25, 90, 10, 7), np.maximum(bottom - z, z - top))
        solid = join(solid, rail, 4)
    # Only three cross bridges: the open centre follows the selected concept.
    for bridge_x, half_length in [(-83, 10), (0, 12), (77, 9)]:
        bridge_top = np.maximum(top, -.5) if bridge_x == 0 else top
        bridge = np.maximum(rounded2(x, y, bridge_x, 0, half_length, 39, 5), np.maximum(bottom - z, z - bridge_top))
        solid = join(solid, bridge, 4)
    # Swept rounded ribs form large triangular openings, not perforated walls.
    path = [(-70, -20), (-39, -7), (-22, 16), (-10, 42), (0, 66),
            (12, 76), (28, 73), (47, 60), (56, 22), (83, -1)]
    rib = np.full((len(axes[0]), 1, len(axes[2])), 1000, dtype=np.float32)
    for (ax, az), (bx, bz) in pairwise(path):
        t = np.clip(((x - ax) * (bx - ax) + (z - az) * (bz - az)) / ((bx - ax)**2 + (bz - az)**2), 0, 1)
        rib = np.minimum(rib, np.hypot(x - ax - t * (bx - ax), z - az - t * (bz - az)) - 8)
    for sign in [-1, 1]:
        side = np.maximum(rib, np.abs(y - sign * 40) - 6)
        side = np.maximum(side, np.maximum(bottom - z, w - 30))
        solid = join(solid, side, 5)
    # The compact seat is integral with the two ribs, removing lid fasteners.
    seat = np.maximum(rounded2(u, y, 0, 0, 39, 44, 8), slab(w, -8, 0))
    solid = join(solid, seat, 2)

    # The lower camera carrier is fused and filleted, with no welded joint.
    rear = (np.array(p["camera_rear_xyz_m"]) - p["mount_xyz_m"]) * 1000
    angle = np.radians(p["camera_pitch_deg"])
    ca, sa = np.cos(angle), np.sin(angle)
    cx, cz = ca * (x - rear[0]) - sa * (z - rear[2]), sa * (x - rear[0]) + ca * (z - rear[2])
    carrier = np.maximum(slab(cx, -8, 0), rounded2(y, cz, 0, 0, 48, 16, 4))
    solid = join(solid, carrier, 3)

    # Clearance bores for the original head screws; no printed screw threads.
    for sign in [-1, 1]:
        bore = np.hypot(x, y - sign * p["head_hole_pitch_mm"] / 2) - p["head_clearance_diameter_mm"] / 2
        solid = np.maximum(solid, -bore)
        # Clear the adjacent wall for the wide washer and provide a flat seat.
        washer_well = np.maximum(np.hypot(x, y - sign * p["head_hole_pitch_mm"] / 2) - 8.5, -.5 - z)
        solid = np.maximum(solid, -washer_well)
    # Sensor screws enter through the open centre from below the integral seat.
    for screw_x, screw_y, radius in sensor_holes:
        bore = np.maximum(np.hypot(u - screw_x, y - screw_y) - radius, slab(w, -15, 2))
        solid = np.maximum(solid, -bore)

    # Rear camera screws insert through counterbores accessible from the rear.
    for sign in [-1, 1]:
        bore = np.maximum(np.hypot(y - sign * 22.5, cz) - 1.8, slab(cx, -12, 1))
        seat = np.maximum(np.hypot(y - sign * 22.5, cz) - 3.8, slab(cx, -12, -4))
        solid = np.maximum(solid, -np.minimum(bore, seat))

    vertices, faces, _, _ = marching_cubes(solid.astype(np.float32), -.037,
                                          spacing=(spacing,) * 3, allow_degenerate=False)
    vertices = vertices.astype(float) + np.array([axis[0] for axis in axes])
    parts = {"body_shell": (vertices.astype(float), faces)}
    parts.update(support_pads(p))
    outline = rounded_rectangle(p["deck_size_mm"], 8)
    lid_holes = []
    for sign in [-1, 1]:
        washer, washer_faces = extrude(circle(0, sign * p["head_hole_pitch_mm"] / 2, 7), [(0, sign * p["head_hole_pitch_mm"] / 2, 1.7)], 3)
        washer[:, 2] -= .5
        parts[f"load_washer_{'left' if sign > 0 else 'right'}"] = (washer, washer_faces)
    return parts, outline, lid_holes
