"""Generate a six-piece bolted, flat-print fit prototype in millimetres."""

import json
import sys
from pathlib import Path

import numpy as np
import trimesh
from skimage.measure import marching_cubes

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
sys.path.insert(0, str(HERE.parent))
from accessories import head_heights
from generate import extrude, rounded_rectangle, ry
from generate_box import polygon_distance

PITCH = 0.25
ROT = ry(35)
OPTICAL = np.array([50.0, 0.0, 101.62])
SEAT = OPTICAL - ROT @ np.array([0.0, 0.0, 47.0])
BASE_BOTTOM = -9.0
BASE_TOP = -5.0
M4_RADIUS = 2.25
parts = {}
report = {
    "status": "fit_prototype_not_load_or_field_qualified",
    "sensor_optical_mount_mm": OPTICAL.tolist(),
    "sensor_bottom_mount_mm": SEAT.tolist(),
    "sensor_pitch_deg": 35,
    "base_bottom_z_mm": BASE_BOTTOM,
    "base_top_z_mm": BASE_TOP,
    "head_hole_pitch_mm": 58.5,
    "parts": {},
}


def interval(value, lo, hi):
    return np.maximum(lo - value, value - hi)


def segment_distance(x, z, a, b):
    dx, dz = b - a
    t = np.clip(((x - a[0]) * dx + (z - a[1]) * dz) / (dx * dx + dz * dz), 0, 1)
    return np.hypot(x - a[0] - t * dx, z - a[1] - t * dz)


def hex_field(a, b):
    result = np.maximum(a, -a) - 3.8
    for t in np.arange(1, 6) * np.pi / 3:
        result = np.maximum(result, np.cos(t) * a + np.sin(t) * b - 3.8)
    return result


def side_field(x, y, z):
    x, y, z = np.asarray(x), np.asarray(y), np.asarray(z)
    u = ROT[0, 0] * (x - SEAT[0]) - ROT[0, 2] * (z - SEAT[2])
    w = ROT[0, 2] * (x - SEAT[0]) + ROT[0, 0] * (z - SEAT[2])
    front = SEAT + ROT @ np.array([55, 0, -5])
    back = SEAT + ROT @ np.array([-55, 0, -5])
    outline = np.array([[-55, BASE_TOP], [front[0], BASE_TOP], front[[0, 2]], back[[0, 2]]])
    d = polygon_distance(x, z, outline)
    frame = np.maximum(d, -(d + 8))
    # Two diagonal strips retain load paths within the flat printed layer plane.
    for a, b in [((-44, BASE_TOP + 6), (front[0] - 6, front[2] - 7)),
                 ((front[0] - 8, BASE_TOP + 6), (back[0] + 3, back[2] - 9))]:
        frame = np.minimum(frame, np.maximum(d, segment_distance(x, z, np.array(a), np.array(b)) - 3.5))
    solid = np.maximum(frame, interval(y, 46, 50))
    # Thick joint pads grow inward from the same bed-facing surface as the sheet.
    for a in (-45, 45):
        pad = np.maximum(np.abs(u - a) - 9, interval(w, -23, -5))
        solid = np.minimum(solid, np.maximum(pad, interval(y, 34, 50)))
    for a in (-40, 40):
        pad = np.maximum(np.abs(x - a) - 10, interval(z, BASE_TOP, BASE_TOP + 18))
        solid = np.minimum(solid, np.maximum(pad, interval(y, 34, 50)))
    # Two positioning tongues enter matching slots in the base plate.
    for a in (-27, 27):
        tongue = np.maximum(np.abs(x - a) - 2.5, interval(z, BASE_TOP - 2, BASE_TOP + 2))
        solid = np.minimum(solid, np.maximum(tongue, interval(y, 46, 50)))
    for a in (-45, 45):
        bore = np.maximum(np.hypot(u - a, y - 42) - M4_RADIUS, interval(w, -25, -3))
        pocket = hex_field(u - a, y - 42)
        entrance = np.maximum(np.abs(u - a) - 3.8, interval(y, 32, 42))
        pocket = np.maximum(np.minimum(pocket, entrance), interval(w, -14, -10.4))
        solid = np.maximum(solid, -np.minimum(bore, pocket))
    for a in (-40, 40):
        bore = np.maximum(np.hypot(x - a, y - 42) - M4_RADIUS, interval(z, BASE_TOP - 1, BASE_TOP + 20))
        pocket = hex_field(x - a, y - 42)
        entrance = np.maximum(np.abs(x - a) - 3.8, interval(y, 32, 42))
        pocket = np.maximum(np.minimum(pocket, entrance), interval(z, BASE_TOP + 6, BASE_TOP + 9.6))
        solid = np.maximum(solid, -np.minimum(bore, pocket))
    return solid


def base_field(x, y, z):
    solid = np.maximum(np.maximum(np.abs(x) - 55, np.abs(y) - 50.5), interval(z, BASE_BOTTOM, BASE_TOP))
    holes = [(0, a, 1.8) for a in (-29.25, 29.25)]
    holes += [(a, b, M4_RADIUS) for a in (-40, 40) for b in (-42, 42)]
    holes += [(a, 0, 17) for a in (-28, 28)]
    for a, b, radius in holes:
        solid = np.maximum(solid, -(np.hypot(x - a, y - b) - radius))
    for a in (-27, 27):
        for b in (-48, 48):
            slot = np.maximum(np.abs(x - a) - 2.7, np.abs(y - b) - 2.2)
            solid = np.maximum(solid, -slot)
    return solid


def mesh_field(field, bounds):
    # Avoid sampling exactly on intersecting zero-level planes.
    axes = [np.arange(lo + 0.037, hi + PITCH / 2, PITCH, dtype=np.float32) for lo, hi in bounds]
    values = field(axes[0][:, None, None], axes[1][None, :, None], axes[2][None, None, :])
    v, f, _, _ = marching_cubes(values.astype(np.float32), 0, spacing=(PITCH,) * 3, allow_degenerate=False)
    v += np.array([a[0] for a in axes])
    mesh = trimesh.Trimesh(v, f, process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def add(name, mesh, print_matrix):
    mesh.remove_unreferenced_vertices()
    if not mesh.is_watertight or not mesh.is_winding_consistent or len(mesh.split()) != 1:
        raise ValueError(f"{name}: mesh closure/connectivity failed")
    parts[name] = mesh
    np.savez_compressed(HERE / f"{name}.npz", vertices=mesh.vertices, faces=mesh.faces)
    printed = mesh.copy()
    transform = np.eye(4)
    transform[:3, :3] = print_matrix
    printed.apply_transform(transform)
    printed.apply_translation(-printed.bounds[0])
    printed.export(HERE / "print" / f"{name}.stl")
    report["parts"][name] = {"print_size_mm": printed.extents.round(3).tolist(),
                             "solid_mass_g": round(mesh.volume * 0.00124, 2),
                             "watertight": True, "components": 1}


def main():
    (HERE / "print").mkdir(parents=True, exist_ok=True)
    side = mesh_field(side_field, [(-56, 67), (33, 51), (-8, 93)])
    add("left_side", side, np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]))
    other = side.copy()
    mirror = np.eye(4)
    mirror[1, 1] = -1
    other.apply_transform(mirror)
    add("right_side", other, np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]))
    add("base", mesh_field(base_field, [(-56, 56), (-51.5, 51.5), (-10, -4)]), np.eye(3))
    holes = [(a, b, 1.7) for a in (-24, 24) for b in (-18, 18)]
    holes += [(a, b, M4_RADIUS) for a in (-45, 45) for b in (-42, 42)]
    holes += [(a, b, 4.5) for a in (-12, 0, 12) for b in (-6, 6)]
    holes += [(0, a, 6) for a in (-42, 42)] + [(a, 0, 6) for a in (-45, 45)]
    v, f = extrude(rounded_rectangle([110, 100], 6), holes, 5)
    v[:, 2] -= 5
    lid = trimesh.Trimesh(v @ ROT.T + SEAT, f, process=True)
    add("lid", lid, ROT.T)
    outline = rounded_rectangle([70, 14], 3)
    grid = np.array([(x, y) for x in np.arange(-32, 33, 2) for y in np.arange(-5, 6, 2) if np.hypot(x, y) > 3])
    for name, sign in [("left_foot", 1), ("right_foot", -1)]:
        v, f = extrude(outline, [(0, 0, 1.8)], 1, grid)
        n = len(v) // 2
        v[:, 1] += sign * 29.25
        bottom = head_heights(v[:n, :2] + [190, 0]) - 101
        if bottom.max() > BASE_BOTTOM - 1.5:
            raise ValueError("Foot leaves insufficient thickness")
        v[:n, 2] = bottom
        v[n:, 2] = BASE_BOTTOM
        add(name, trimesh.Trimesh(v, f, process=True), np.diag([1, -1, -1]))
    # Sample known failure modes: screw paths, bolt pocket access and head clearance.
    for a in (-45, 45):
        w = np.linspace(-23, -5, 150)
        pts = np.column_stack((np.full_like(w, a), np.full_like(w, 42), w)) @ ROT.T + SEAT
        if np.any(side_field(*pts.T) < -0.05):
            raise ValueError("Lid M4 screw channel obstructed")
    for a in (-40, 40):
        z = np.linspace(BASE_TOP, BASE_TOP + 18, 150)
        if np.any(side_field(a, np.full_like(z, 42), z) < -0.05):
            raise ValueError("Base M4 screw channel obstructed")
    head_xy = np.array([(x, y) for x in np.arange(-55, 56, 2) for y in np.arange(-40, 41, 2)])
    highest_head = float((head_heights(head_xy + [190, 0]) - 101).max())
    if highest_head >= BASE_BOTTOM:
        raise ValueError("Base plate intersects nominal head surface")
    all_v = np.vstack([m.vertices for m in parts.values()])
    sensor_v = (all_v - OPTICAL) @ ROT
    clearance = -np.tan(np.radians(7)) * np.linalg.norm(sensor_v[:, :2], axis=1) - sensor_v[:, 2]
    if clearance.min() <= 0:
        raise ValueError("Structure enters nominal optical cone")
    report.update(nominal_head_clearance_mm=round(BASE_BOTTOM - highest_head, 3),
                  minimum_nominal_scan_cone_clearance_mm=float(clearance.min()),
                  checked_screw_paths="four lid M4 and four base M4 channels sampled clear",
                  note="Physical Go2 interface, stiffness and thermal performance remain unverified")
    (HERE / "geometry_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
