"""Generate a PLA dual-sensor cradle prototype and full Go2 URDF.

Uses the existing conda numpy/scipy/scikit-image environment. CAD/STL/DXF use mm; URDF/DAE use m.
The existing field RobotConfig and official Go2 assets are never modified.
"""

import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import Delaunay

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ASSETS = ROOT / "web/public/assets/robots/go2"
MESHES = ASSETS / "meshes/mid360_ground_mount"
CAD = HERE / "cad"


def ry(degrees):
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def extrude(outline, holes, thickness, interior=()):
    """Triangulate these convex plate outlines with separated circular holes."""
    points = [*outline, *interior]
    for x, y, radius in holes:
        points.extend((x + radius * math.cos(a), y + radius * math.sin(a))
                      for a in np.linspace(0, 2 * math.pi, 64, endpoint=False))
    points = np.unique(np.round(np.asarray(points, dtype=float), 8), axis=0)
    faces = []
    for triangle in Delaunay(points).simplices:
        center = points[triangle].mean(axis=0)
        if any(np.linalg.norm(center - [x, y]) < radius * (1 - 1e-8)
               for x, y, radius in holes):
            continue
        a, b, c = points[triangle]
        if np.linalg.det(np.stack((b - a, c - a))) < 0:
            triangle = triangle[::-1]
        faces.append(tuple(int(i) for i in triangle))
    count = len(points)
    vertices = np.vstack((np.column_stack((points, np.zeros(count))),
                          np.column_stack((points, np.full(count, thickness)))))
    result = [(c, b, a) for a, b, c in faces]
    result.extend((a + count, b + count, c + count) for a, b, c in faces)
    edges = set()
    for a, b, c in faces:
        for edge in [(a, b), (b, c), (c, a)]:
            reverse = edge[::-1]
            if reverse in edges:
                edges.remove(reverse)
            else:
                edges.add(edge)
    for a, b in sorted(edges):
        result.extend([(a, b, b + count), (a, b + count, a + count)])
    return vertices, np.asarray(result)


def rectangle(size):
    x, y = np.asarray(size) / 2
    return [(-x, -y), (x, -y), (x, y), (-x, y)]


def rounded_rectangle(size, radius):
    x, y = np.asarray(size) / 2 - radius
    points = []
    for center, start in [((x, y), 0), ((-x, y), 90), ((-x, -y), 180), ((x, -y), 270)]:
        for angle in np.linspace(start, start + 90, 9):
            radians = math.radians(angle)
            points.append((center[0] + radius * math.cos(radians), center[1] + radius * math.sin(radians)))
    return np.asarray(points)


def circle(x, y, radius):
    angles = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    return np.column_stack((x + radius * np.cos(angles), y + radius * np.sin(angles)))


def inside_convex(points, polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    delta = points[:, None, :] - polygon[None, :, :]
    cross = edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]
    return np.all(cross >= -1e-8, axis=1)


def sensor_housing():
    """Closed illustrative housing with optical origin at z=0; dimensions in mm."""
    angles = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    square = directions * (32.5 / np.abs(directions).max(axis=1))[:, None]
    rings = [np.column_stack((square, np.full(64, z))) for z in [-47, -7.5]]
    for angle in np.linspace(0, np.pi / 2, 9, endpoint=False):
        rings.append(np.column_stack((directions * 22 * np.cos(angle),
                                      np.full(64, -7.5 + 20.5 * np.sin(angle)))))
    vertices = np.vstack([*rings, [0, 0, -47], [0, 0, 13]])
    bottom, top = len(vertices) - 2, len(vertices) - 1
    faces = []
    for index in range(64):
        after = (index + 1) % 64
        faces.append((bottom, after, index))
        for layer in range(len(rings) - 1):
            a, b, c, d = layer * 64 + index, layer * 64 + after, (layer + 1) * 64 + after, (layer + 1) * 64 + index
            faces.extend([(a, b, c), (a, c, d)])
        faces.append(((len(rings) - 1) * 64 + index, (len(rings) - 1) * 64 + after, top))
    connector, connector_faces = extrude((directions * 6).tolist(), [], 8)
    connector = connector @ ry(-90).T + [-32.5, 0, -32.7]
    faces = np.vstack((faces, connector_faces + len(vertices)))
    return np.vstack((vertices, connector)), faces


def mass_properties(vertices, faces, density):
    triangles = vertices[faces].astype(np.float64) / 1000
    volumes = np.einsum("ij,ij->i", triangles[:, 0],
                        np.cross(triangles[:, 1], triangles[:, 2])) / 6
    sums = triangles.sum(axis=1)
    volume = volumes.sum()
    center = (volumes[:, None] * sums / 4).sum(axis=0) / volume
    second = np.einsum("ni,nj->nij", sums, sums)
    second += np.einsum("nki,nkj->nij", triangles, triangles)
    second = (volumes[:, None, None] * second / 20).sum(axis=0)
    second -= volume * np.outer(center, center)
    inertia = density * (np.trace(second) * np.eye(3) - second)
    return volume * density, center, inertia


def stl(path, vertices, faces):
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError(f"Degenerate face in {path.name}")
    records = np.zeros(len(faces), dtype=[("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    records["normal"] = normals / lengths[:, None]
    records["vertices"] = triangles
    with path.open("wb") as stream:
        stream.write(b"LingTu MID360 prototype; millimeters".ljust(80, b" "))
        stream.write(struct.pack("<I", len(faces)))
        stream.write(records.tobytes())


def dae(path, vertices, faces, color="0.13 0.15 0.17"):
    positions = " ".join(f"{v / 1000:.10g}" for v in vertices.flatten())
    indices = " ".join(str(int(i)) for i in faces.flatten())
    path.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-09-21T00:00:00Z</created><modified>2026-09-21T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_effects><effect id="gray"><profile_COMMON><technique sid="common"><lambert><diffuse><color>{color} 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="metal"><instance_effect url="#gray"/></material></library_materials>
<library_geometries><geometry id="part"><mesh>
<source id="positions"><float_array id="position-array" count="{len(vertices)*3}">{positions}</float_array><technique_common><accessor source="#position-array" count="{len(vertices)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="{len(faces)}" material="surface"><input semantic="VERTEX" source="#vertices" offset="0"/><p>{indices}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="part-node"><instance_geometry url="#part"><bind_material><technique_common><instance_material symbol="surface" target="#metal"/></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
''', encoding="utf-8")


def dxf(path, outline, holes):
    lines = ["0", "SECTION", "2", "HEADER", "9", "$INSUNITS", "70", "4",
             "0", "ENDSEC", "0", "SECTION", "2", "ENTITIES"]
    for a, b in zip(outline, np.roll(outline, -1, axis=0)):
        lines.extend(["0", "LINE", "8", "CUT", "10", str(a[0]), "20", str(a[1]),
                      "11", str(b[0]), "21", str(b[1])])
    for x, y, radius in holes:
        lines.extend(["0", "CIRCLE", "8", "CUT", "10", str(x), "20", str(y), "40", str(radius)])
    path.write_text("\n".join([*lines, "0", "ENDSEC", "0", "EOF", ""]), encoding="ascii")


def vector(values):
    return " ".join(f"{v:.12g}" for v in values)


def add_link(robot, name, parent, mesh, vertices, faces, density, xyz, pitch):
    link = ET.SubElement(robot, "link", name=name)
    mass, center, inertia = mass_properties(vertices, faces, density)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=vector(center), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.10g}")
    ET.SubElement(inertial, "inertia", **{
        key: f"{inertia[i,j]:.10g}" for key, i, j in
        [("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2),
         ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2)]})
    for kind in ["visual", "collision"]:
        geometry = ET.SubElement(ET.SubElement(link, kind), "geometry")
        ET.SubElement(geometry, "mesh", filename=f"package://go2_description/meshes/mid360_ground_mount/{mesh}.dae")
    joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=name)
    ET.SubElement(joint, "origin", xyz=vector(xyz), rpy=vector([0, math.radians(pitch), 0]))
    return mass


def generate():
    p = json.loads((HERE / "parameters.json").read_text())
    CAD.mkdir(exist_ok=True)
    MESHES.mkdir(parents=True, exist_ok=True)
    mount_rotation = ry(p["mount_pitch_deg"])
    new_rotation = ry(p["target_pitch_deg"])
    optical = np.array([0, 0, p["optical_origin_above_bottom_mm"]])
    mount_origin = np.asarray(p["mount_xyz_m"])
    new_bottom = np.array(p["target_lidar_xyz_m"]) - new_rotation @ optical / 1000
    deck_position = mount_rotation.T @ (new_bottom - mount_origin) * 1000
    delta = p["target_pitch_deg"] - p["mount_pitch_deg"]
    deck_rotation = ry(delta)
    holes = [(sx * p["lidar_hole_pitch_mm"][0] / 2,
              sy * p["lidar_hole_pitch_mm"][1] / 2,
              p["clearance_hole_diameter_mm"] / 2)
             for sx in [-1, 1] for sy in [-1, 1]]
    from accessories import camera_parts
    from printed_cradle import build

    parts, deck_outline, cover_holes = build(p, deck_position, deck_rotation, holes)
    parts["mid360_envelope"] = sensor_housing()
    parts["d435i_envelope"] = camera_parts(p)["d435i_envelope"]
    for sensor in ["mid360", "d435i"]:
        reference = np.load(HERE / "reference" / f"{sensor}_official.npz")
        dae(MESHES / f"{sensor}_official.dae", reference["vertices"], reference["faces"], "0.47 0.50 0.52")
    base_outline = rounded_rectangle([180, 79], 7) + np.array([-5, 0])
    base_holes = [(0, sign * p["head_hole_pitch_mm"] / 2, p["head_clearance_diameter_mm"] / 2) for sign in [-1, 1]]
    robot = ET.parse(ASSETS / "urdf/go2_description.urdf").getroot()
    robot.set("name", "go2_mid360_ground_mount_prototype")
    robot.insert(0, ET.Comment(" Generated prototype; no field calibration. See tools/robot/go2_mid360_ground_mount/README.md. "))
    ET.SubElement(robot, "link", name="sensor_seat_frame")
    seat_joint = ET.SubElement(robot, "joint", name="sensor_seat_joint", type="fixed")
    ET.SubElement(seat_joint, "parent", link="mid360_mount_base")
    ET.SubElement(seat_joint, "child", link="sensor_seat_frame")
    ET.SubElement(seat_joint, "origin", xyz=vector(deck_position / 1000), rpy=vector([0, math.radians(delta), 0]))
    total_mass = 0
    static_pitch_moment = 0
    for name, (vertices, faces) in parts.items():
        dae(MESHES / f"{name}.dae", vertices, faces, "0.47 0.50 0.52" if name in ("mid360_envelope", "d435i_envelope") else "0.13 0.15 0.17")
        stl(CAD / f"{name}.stl", vertices, faces)
        xyz, pitch, parent = [0, 0, 0], 0, "mid360_mount_base"
        link_name = f"mid360_mount_{name}"
        if name == "body_shell":
            xyz, pitch, parent, link_name = mount_origin, p["mount_pitch_deg"], "base", "mid360_mount_base"
        elif name == "mid360_envelope":
            xyz, parent, link_name = optical / 1000, "sensor_seat_frame", "livox_frame"
        elif name == "d435i_envelope":
            xyz = np.asarray(p["camera_rear_xyz_m"]) - mount_origin
            pitch, link_name = p["camera_pitch_deg"], "camera_link"
        density = p["pla_density_kg_m3"]
        if name in ("mid360_envelope", "d435i_envelope"):
            sensor_mass = .265 if name == "mid360_envelope" else p["camera_mass_kg"]
            density = sensor_mass / mass_properties(vertices, faces, 1)[0]
        mass = add_link(robot, link_name, parent, name, vertices, faces, density, xyz, pitch)
        if name in ("mid360_envelope", "d435i_envelope"):
            sensor = "mid360" if name == "mid360_envelope" else "d435i"
            robot.find(f"link[@name='{link_name}']/visual/geometry/mesh").set(
                "filename", f"package://go2_description/meshes/mid360_ground_mount/{sensor}_official.dae")
        _, center, _ = mass_properties(vertices, faces, density)
        if name == "mid360_envelope":
            center = deck_rotation @ (center + optical / 1000) + deck_position / 1000
        elif name == "d435i_envelope":
            center = ry(p["camera_pitch_deg"]) @ center + xyz
        world_center = mount_rotation @ center + mount_origin
        static_pitch_moment += mass * 9.81 * (world_center[0] - mount_origin[0])
        if name not in ("mid360_envelope", "d435i_envelope"):
            total_mass += mass
    ET.SubElement(robot, "link", name="camera_depth_optical_frame")
    joint = ET.SubElement(robot, "joint", name="camera_depth_optical_joint", type="fixed")
    ET.SubElement(joint, "parent", link="camera_link")
    ET.SubElement(joint, "child", link="camera_depth_optical_frame")
    ET.SubElement(joint, "origin", xyz="0.02075 0.0175 0", rpy=vector([-math.pi / 2, 0, -math.pi / 2]))
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(ASSETS / "urdf/go2_mid360_ground_mount.urdf", encoding="utf-8", xml_declaration=True)
    for name, outline, pattern in [("body_footprint", base_outline, base_holes),
                                   ("sensor_deck", deck_outline, [*holes, *cover_holes])]:
        dxf(CAD / f"{name}.dxf", outline, pattern)
    scad = ['// V6B concept-matched long saddle and swept ribs with low D435i. Edit parameters.json and regenerate.',
            '// All custom components are PLA; standard screws and nuts remain purchased fasteners.',
            'part = "assembly";']
    for name, (vertices, faces) in parts.items():
        if name in ("mid360_envelope", "d435i_envelope"):
            continue
        # OpenSCAD polyhedron faces use clockwise winding viewed from outside.
        scad.append(f'module {name}() {{ polyhedron(points={json.dumps(vertices.round(8).tolist())},faces={json.dumps(faces[:, ::-1].tolist())},convexity=10); }}')
    structural_parts = [name for name in parts if name not in ("mid360_envelope", "d435i_envelope")]
    scad.append('if (part == "assembly") { ' + " ".join(f"{name}();" for name in structural_parts))
    scad.append('}')
    for name in structural_parts:
        scad.append(f'else if (part == "{name}") {name}();')
    (CAD / "mount.scad").write_text("\n".join(scad) + "\n", encoding="utf-8")
    print_directory = HERE / "print"
    print_directory.mkdir(exist_ok=True)
    vertices, faces = parts["body_shell"]
    # Lay the left cheek on the bed: the main XZ load path lies within layers.
    print_vertices = vertices[:, [0, 2, 1]].copy()
    print_vertices[:, 1] *= -1
    print_vertices -= print_vertices.min(axis=0)
    stl(print_directory / "pla_cradle.stl", print_vertices, faces)
    for name, (vertices, faces) in parts.items():
        if name in ("body_shell", "mid360_envelope", "d435i_envelope"):
            continue
        stl(print_directory / f"{name}.stl", vertices - vertices.min(axis=0), faces)
    gauge_holes = [(0, sign * p["head_hole_pitch_mm"] / 2, 1.7) for sign in [-1, 1]]
    gauge, gauge_faces = extrude(rounded_rectangle([18, 76], 3), gauge_holes, 2)
    stl(print_directory / "head_hole_gauge.stl", gauge - gauge.min(axis=0), gauge_faces)
    dxf(CAD / "camera_rear_interface.dxf", rounded_rectangle([96, 32], 4),
        [(-22.5, 0, 1.8), (22.5, 0, 1.8)])
    imu_translation = np.asarray(p["target_lidar_xyz_m"]) - new_rotation @ [-.011, -.02329, .04412]
    report = {"status": p["status"], "bracket_mass_kg_excluding_fasteners": total_mass,
              "design_revision": "V6B concept-matched long saddle and swept ribs",
              "mass_assumption": "PLA treated as solid; slicer infill mass will differ",
              "implicit_mesh_pitch_mm": p["implicit_mesh_pitch_mm"],
              "static_gravity_pitch_moment_Nm_including_sensor": static_pitch_moment,
              "camera_mount_rear_xyz_m": p["camera_rear_xyz_m"],
              "camera_pitch_deg": p["camera_pitch_deg"],
              "camera_optical_frame_status": "vendor nominal left-imager frame; use device calibration on hardware",
              "saddle_support_length_mm": p["saddle_size_mm"][0],
              "mount_interface": "Go2 head two-hole pattern from community CAD; body registration is provisional",
              "mount_interface_xyz_m": mount_origin.tolist(),
              "head_hole_pitch_mm": p["head_hole_pitch_mm"],
              "deck_in_mount_xyz_mm": deck_position.tolist(), "deck_relative_pitch_deg": delta,
              "design_body_from_lidar_xyz_m": p["target_lidar_xyz_m"],
              "design_body_from_lidar_pitch_rad": math.radians(p["target_pitch_deg"]),
              "design_navigation_body_from_imu_rotation": new_rotation.flatten().tolist(),
              "design_navigation_body_from_imu_translation": imu_translation.tolist(),
              "flat_ground_front_edge_distance_m": {}}
    for label, angle, lidar_z, lidar_x in [
        ("original_13deg", p["original_pitch_deg"], p["original_lidar_xyz_m"][2], p["original_lidar_xyz_m"][0]),
        ("prototype_v6_35deg", p["target_pitch_deg"], p["target_lidar_xyz_m"][2], p["target_lidar_xyz_m"][0])]:
        report["flat_ground_front_edge_distance_m"][label] = (
            lidar_x + (p["nominal_body_height_m"] + lidar_z) / math.tan(math.radians(angle + 7)) - p["body_front_x_m"])
    (HERE / "design_values.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return p, parts, mount_origin, mount_rotation, deck_position, deck_rotation


if __name__ == "__main__":
    generate()
