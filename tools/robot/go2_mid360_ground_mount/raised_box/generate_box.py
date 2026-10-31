"""Generate a separate raised, ribbed box prototype. All geometry is in mm.

Uses the existing conda mesh dependencies and existing Go2 nominal head surface.
No runtime calibration or original cradle assets are changed.
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import map_coordinates
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from skimage.measure import marching_cubes

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from accessories import head_heights
from generate import extrude, mass_properties, rounded_rectangle, ry, stl
from printed_cradle import rounded2, slab


def polygon_distance(x, z, polygon):
    distance = np.full(np.broadcast_shapes(x.shape, z.shape), np.inf, dtype=np.float32)
    inside = np.ones(distance.shape, dtype=bool)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        dx, dz = b - a
        t = np.clip(((x - a[0]) * dx + (z - a[1]) * dz) / (dx * dx + dz * dz), 0, 1)
        distance = np.minimum(distance, np.hypot(x - a[0] - t * dx, z - a[1] - t * dz))
        inside &= dx * (z - a[1]) - dz * (x - a[0]) >= 0
    return np.where(inside, -distance, distance)


def generate(directory=HERE, lightweight=False):
    p = json.loads((directory / "parameters.json").read_text())
    out = directory / "print"
    out.mkdir(parents=True, exist_ok=True)
    rotation = ry(p["pitch_deg"])
    origin = np.array(p["mount_xyz_m"]) * 1000
    optical = np.array(p["optical_xyz_m"]) * 1000 - origin
    seat = optical - rotation @ np.array([0, 0, p["optical_above_bottom_mm"]])
    spacing = p["mesh_pitch_mm"]
    axes = [np.arange(a, b + spacing, spacing, dtype=np.float32) for a, b in [(-71, 76), (-56, 56), (-65, 112)]]
    x, y, z = axes[0][:, None, None], axes[1][None, :, None], axes[2][None, None, :]
    c, s = rotation[0, 0], rotation[0, 2]
    u = c * (x - seat[0]) - s * (z - seat[2])
    w = s * (x - seat[0]) + c * (z - seat[2])
    gx, gy = np.arange(-72, 79, 2), np.arange(-42, 43, 2)
    sample = np.array([(a, b) for a in gx for b in gy], dtype=float) + origin[:2]
    heights = head_heights(sample).reshape(len(gx), len(gy)) - origin[2]
    xx, yy = np.meshgrid((axes[0] - gx[0]) / 2, (axes[1] - gy[0]) / 2, indexing="ij")
    surface = map_coordinates(heights, [xx, yy], order=1, mode="nearest")[:, :, None]
    bottom = surface + p["fit_allowance_mm"] + p["pad_mm"]
    lid_thickness = p["lid_mm"][2]
    front = seat + rotation @ np.array([55, 0, -lid_thickness])
    back = seat + rotation @ np.array([-55, 0, -lid_thickness])
    polygon = np.array([[-65, -60], [70, -60], front[[0, 2]], back[[0, 2]]])
    d = polygon_distance(x, z, polygon)
    outer = np.maximum(d, np.abs(y) - 50)
    outer = np.maximum(outer, bottom - z)
    cavity = np.maximum(d + p["wall_mm"], np.abs(y) - (50 - p["wall_mm"]))
    cavity = np.maximum(cavity, bottom + p["floor_mm"] - z)
    # Open the top of the inner cavity; the separate lid supplies roof stiffness.
    cavity = np.minimum(cavity, np.maximum(rounded2(u, y, 0, 0, 45, 40, 4), slab(w, -12, 2)))
    solid = np.maximum(outer, -cavity)
    if lightweight:
        # Open panels before adding the existing diagonal ribs and screw bosses.
        side_window = np.maximum(d + 14, bottom + p["floor_mm"] + 8 - z)
        side_window = np.maximum(side_window, w + lid_thickness + 10)
        solid = np.maximum(solid, -side_window)
        end_window = np.maximum(np.abs(y) - 30, bottom + p["floor_mm"] + 8 - z)
        end_window = np.maximum(end_window, w + lid_thickness + 10)
        solid = np.maximum(solid, -end_window)
        # Retain the central transverse mounting bridge and perimeter rails.
        for center_x in (-30, 30):
            floor_window = np.maximum(rounded2(x, y, center_x, 0, 12, 22, 4), d + 10)
            floor_window = np.maximum(floor_window, z - bottom - p["floor_mm"] - 1)
            solid = np.maximum(solid, -floor_window)
    # Diagonal side ribs are inside the box, joining the base to the roof corners.
    for a, b in [((-48, -8), (front[0] - 5, front[2] - 10)), ((52, -8), (back[0] + 5, back[2] - 10))]:
        dx, dz = b[0] - a[0], b[1] - a[1]
        t = np.clip(((x - a[0]) * dx + (z - a[1]) * dz) / (dx * dx + dz * dz), 0, 1)
        rib = np.maximum(np.hypot(x - a[0] - t * dx, z - a[1] - t * dz) - 4, np.abs(np.abs(y) - 45) - 5)
        solid = np.minimum(solid, np.maximum(rib, outer))
    # Two nominal Go2 mounting bosses remain accessible while the lid is off.
    for sign in (-1, 1):
        radius = np.hypot(x, y - sign * p["head_pitch_mm"] / 2)
        boss_top = float(head_heights(np.array([[origin[0], origin[1] + sign * p["head_pitch_mm"] / 2]]))[0])
        boss_top += -origin[2] + p["fit_allowance_mm"] + p["pad_mm"] + 13
        solid = np.minimum(solid, np.maximum(radius - 9, np.maximum(bottom - z, z - boss_top)))
        solid = np.maximum(solid, -np.maximum(radius - 1.8, z - boss_top - 2))
        solid = np.maximum(solid, -np.maximum(radius - 8, boss_top - z))
    # Four lid bosses; captive M4 nuts are inserted from the open underside.
    for a in (-45, 45):
        for b in (-42, 42):
            radius = np.hypot(u - a, y - b)
            solid = np.minimum(solid, np.maximum(radius - 9, slab(w, -lid_thickness - 19, -lid_thickness)))
            bore = np.maximum(radius - 2.25, slab(w, -lid_thickness - 22, 0))
            hexagon = np.maximum.reduce(
                [
                    np.broadcast_to(math.cos(t) * (u - a) + math.sin(t) * (y - b) - 3.8, solid.shape)
                    for t in np.arange(6) * math.pi / 3
                ]
            )
            pocket = np.maximum(hexagon, slab(w, -lid_thickness - 20, -lid_thickness - 11))
            solid = np.maximum(solid, -np.minimum(bore, pocket))
    # Small vents leave continuous corner columns and diagonal ribs.
    for vx in (-27, 22):
        vent = np.maximum(rounded2(x, z, vx, 16, 10, 5, 3), np.abs(y) - 57)
        solid = np.maximum(solid, -vent)
    # Low rear cable opening: routing stays underneath the optical window.
    cable = np.maximum(rounded2(y, z, 0, 10, 13, 7, 4), slab(x, -75, -20))
    solid = np.maximum(solid, -cable)
    vertices, faces, _, _ = marching_cubes(solid.astype(np.float32), 0, spacing=(spacing,) * 3, allow_degenerate=False)
    vertices += np.array([a[0] for a in axes])
    # Tangent CSG intersections can leave sub-grid closed specks. Remove only
    # components smaller than one sampling cell; structural splits still fail.
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph, directed=False)
    keep = np.ones(len(vertices), dtype=bool)
    removed_specks = 0
    for component in range(count):
        selected = labels == component
        if np.max(np.ptp(vertices[selected], axis=0)) < spacing:
            keep[selected] = False
            removed_specks += 1
    remap = np.cumsum(keep) - 1
    faces = remap[faces[np.all(keep[faces], axis=1)]]
    vertices = vertices[keep]
    parts = {"box": (vertices.astype(float), faces)}
    holes = [(a, b, 1.7) for a in (-24, 24) for b in (-18, 18)]
    holes += [(a, b, 2.25) for a in (-45, 45) for b in (-42, 42)]
    holes += [(a, b, 4.5) for a in (-12, 0, 12) for b in (-6, 6)]
    if lightweight:
        holes += [(0, a, 6) for a in (-42, 42)]
        holes += [(a, 0, 6) for a in (-45, 45)]
    lid, lid_faces = extrude(rounded_rectangle(p["lid_mm"][:2], 6), holes, lid_thickness)
    lid[:, 2] -= lid_thickness
    parts["lid"] = (lid @ rotation.T + seat, lid_faces)
    # Separate small conformal pads, using the same unverified nominal head mesh.
    outline = rounded_rectangle([110, 16], 4)
    interior = np.array([(a, b) for a in np.arange(-50, 51, 3) for b in np.arange(-5, 6, 3) if np.hypot(a, b) > 3])
    pad, pf = extrude(outline, [(0, 0, 1.8)], 1, interior)
    n = len(pad) // 2
    for name, sign in [("left_pad", 1), ("right_pad", -1)]:
        pv = pad.copy()
        pv[:, 1] += sign * p["head_pitch_mm"] / 2
        h = head_heights(pv[:n, :2] + origin[:2]) - origin[2] + p["fit_allowance_mm"]
        pv[:n, 2], pv[n:, 2] = h, h + p["pad_mm"]
        parts[name] = (pv, pf)
    gauge, gf = extrude(rounded_rectangle([16, 76], 3), [(0, a, 1.8) for a in (-29.25, 29.25)], 2)
    parts["hole_gauge"] = (gauge, gf)
    report = {
        "status": "geometry_checks_only_not_load_qualification",
        "parts": {},
        "seat_mount_mm": seat.tolist(),
        "optical_mount_mm": optical.tolist(),
        "sub_grid_csg_specks_removed": removed_specks,
        "variant": "open_frame_quick_fit" if lightweight else "enclosed_box",
    }
    for name, (v, f) in parts.items():
        tri = v[f]
        edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        if not np.all(counts == 2):
            raise ValueError(f"{name}: non-manifold mesh")
        graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(v), len(v)))
        components, labels = connected_components(graph, directed=False)
        if components != 1:
            for component in range(components):
                selected = v[labels == component]
                print(name, component, len(selected), selected.min(0), selected.max(0))
            raise ValueError(f"{name}: {components} disconnected parts")
        volume = np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6
        if volume < 0:
            f = f[:, ::-1]
        mass, _, _ = mass_properties(v, f, p["density_kg_m3"])
        # Side printing keeps the principal XZ load paths within layer planes.
        print_v = v @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]).T if name == "box" else v.copy()
        if name == "lid":
            print_v = (v - seat) @ rotation
        print_v -= print_v.min(axis=0)
        stl(out / f"{name}.stl", print_v, f)
        np.savez_compressed(directory / f"{name}.npz", vertices=v, faces=f)
        report["parts"][name] = {
            "size_mm": np.ptp(print_v, axis=0).round(2).tolist(),
            "solid_mass_g": round(mass * 1000, 1),
            "watertight_edge_counts": True,
            "connected_components": int(components),
        }
    # Structure must remain below the sensor's -7 degree boundary in sensor coordinates.
    all_v = np.vstack([v for k, (v, _) in parts.items() if k != "hole_gauge"])
    sensor_v = (all_v - optical) @ rotation
    clearance = -np.tan(np.radians(7)) * np.linalg.norm(sensor_v[:, :2], axis=1) - sensor_v[:, 2]
    report["minimum_nominal_scan_cone_clearance_mm"] = float(clearance.min())
    if clearance.min() <= 0:
        raise ValueError("Printed structure enters the nominal optical cone")
    report["ground_example"] = {
        str(a): round((p["body_height_example_m"] + p["optical_xyz_m"][2]) / math.tan(math.radians(a + 7)), 3)
        for a in [13, 25, 30, 35]
    }
    (directory / "geometry_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    generate()
