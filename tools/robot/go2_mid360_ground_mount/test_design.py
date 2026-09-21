"""Local geometry contracts only; these do not qualify the physical assembly."""

import json
import math
import struct
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from assembly_geometry import read_collada
from generate import ASSETS, CAD, HERE, MESHES, mass_properties, ry


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * count
    records = np.frombuffer(data, dtype=[("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")], offset=84)
    triangles = records["vertices"]
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return vertices, indices.reshape(-1, 3)


def intersections(origin, direction, triangles):
    e1 = triangles[:, 1] - triangles[:, 0]
    e2 = triangles[:, 2] - triangles[:, 0]
    h = np.cross(np.broadcast_to(direction, e2.shape), e2)
    determinant = np.einsum("ij,ij->i", e1, h)
    valid = np.abs(determinant) > 1e-10
    inv = np.zeros(len(triangles))
    inv[valid] = 1 / determinant[valid]
    offset = origin - triangles[:, 0]
    u = inv * np.einsum("ij,ij->i", offset, h)
    q = np.cross(offset, e1)
    v = inv * (q @ direction)
    distance = inv * np.einsum("ij,ij->i", e2, q)
    return distance[valid & (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8) & (distance > 1e-8)]


class DesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.parse(ASSETS / "urdf/go2_mid360_ground_mount.urdf").getroot()
        cls.transforms = {"base": np.eye(4)}
        joints = list(cls.robot.findall("joint"))
        while joints:
            progressed = False
            for joint in joints[:]:
                parent = joint.find("parent").get("link")
                if parent not in cls.transforms:
                    continue
                origin = joint.find("origin")
                xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                roll, pitch, yaw = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)],
                               [0, math.sin(roll), math.cos(roll)]])
                rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                               [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
                local = np.eye(4)
                local[:3, :3] = rz @ ry(math.degrees(pitch)) @ rx
                local[:3, 3] = xyz
                cls.transforms[joint.find("child").get("link")] = cls.transforms[parent] @ local
                joints.remove(joint)
                progressed = True
            if not progressed:
                raise AssertionError("URDF contains an unresolved parent or cycle")

    def test_original_robot_links_and_joints_preserved(self):
        original = ET.parse(ASSETS / "urdf/go2_description.urdf").getroot()
        for element in list(original):
            if element.tag not in ("link", "joint"):
                continue
            actual = self.robot.find(f"{element.tag}[@name='{element.get('name')}']")
            self.assertIsNotNone(actual)
            for node in list(element.iter()) + list(actual.iter()):
                node.text = (node.text or "").strip()
                node.tail = (node.tail or "").strip()
            self.assertEqual(ET.tostring(element), ET.tostring(actual))

    def test_optical_frame_and_imu_composition(self):
        pose = self.transforms["livox_frame"]
        np.testing.assert_allclose(pose[:3, 3], [.24, 0, .18], atol=1e-10)
        np.testing.assert_allclose(pose[:3, :3], ry(35), atol=1e-10)
        values = json.loads((HERE / "design_values.json").read_text())
        rotation = np.array(values["design_navigation_body_from_imu_rotation"]).reshape(3, 3)
        position = values["design_navigation_body_from_imu_translation"]
        np.testing.assert_allclose(position + rotation @ [-.011, -.02329, .04412], pose[:3, 3], atol=1e-10)

    def test_meshes_closed_positive_and_matching_dae_units(self):
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        for path in sorted(CAD.glob("*.stl")):
            with self.subTest(part=path.stem):
                vertices, faces = read_stl(path)
                areas = np.linalg.norm(np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                                                vertices[faces[:, 2]] - vertices[faces[:, 0]]), axis=1)
                self.assertTrue(np.all(areas > 1e-10), path.stem)
                edges = np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
                _, inverse, counts = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True)
                self.assertTrue(np.all(counts == 2))
                winding = np.bincount(inverse, weights=np.sign(edges[:, 1] - edges[:, 0]))
                self.assertTrue(np.all(winding == 0))
                mass, _, inertia = mass_properties(vertices, faces, 2700)
                self.assertGreater(mass, 0)
                self.assertTrue(np.all(np.linalg.eigvalsh(inertia) > 0))
                collada = ET.parse(MESHES / f"{path.stem}.dae")
                positions = np.fromstring(collada.find(".//c:float_array", ns).text, sep=" ").reshape(-1, 3)
                np.testing.assert_allclose(positions.min(axis=0), vertices.min(axis=0) / 1000, atol=1e-8)
                np.testing.assert_allclose(positions.max(axis=0), vertices.max(axis=0) / 1000, atol=1e-8)
        for mesh in self.robot.findall(".//mesh"):
            self.assertTrue((ASSETS / mesh.get("filename").removeprefix("package://go2_description/")).is_file())

    def test_mounting_holes_are_open_and_surrounded_by_material(self):
        for name, holes in [("body_shell", [(0, -29.25), (0, 29.25)]),
                            ("sensor_deck", [(x, y) for x in [-24, 24] for y in [-18, 18]])]:
            vertices, faces = read_stl(CAD / "body_shell.stl")
            if name == "sensor_deck":
                values = json.loads((HERE / "design_values.json").read_text())
                vertices = (vertices - values["deck_in_mount_xyz_mm"]) @ ry(35)
            triangles = vertices[faces]
            for x, y in holes:
                hits = intersections(np.array([x, y, 1]), np.array([0, 0, -1]), triangles)
                self.assertFalse(np.any(hits < (12 if name == "sensor_deck" else 100)))
                self.assertGreater(len(intersections(np.array([x + (5 if name == "body_shell" else 3), y, 10]), np.array([0, 0, -1]), triangles)), 0)

    def test_full_resolution_body_surface_clearance_samples(self):
        body = read_collada(ASSETS / "dae/base.dae")
        lower = body[:, :, :2].min(axis=1)
        upper = body[:, :, :2].max(axis=1)
        minimum = float("inf")
        main_structure_minimum = float("inf")
        for link in self.robot.findall("link"):
            name = link.get("name")
            if not name.startswith("mid360_mount") and name not in ("livox_frame", "camera_link"):
                continue
            filename = link.find("visual/geometry/mesh").get("filename").removeprefix("package://go2_description/")
            pose = self.transforms[name]
            triangles = read_collada(ASSETS / filename) @ pose[:3, :3].T + pose[:3, 3]
            points = np.unique(np.vstack((triangles.reshape(-1, 3), triangles.mean(axis=1))), axis=0)
            # Bound surface sampling for the tessellated implicit cradle.
            if len(points) > 12000:
                _, sample = np.unique(np.round(points / .002).astype(int), axis=0, return_index=True)
                points = points[sample]
            nearby = np.all((lower <= points[:, :2].max(axis=0)) & (upper >= points[:, :2].min(axis=0)), axis=1)
            local_body, local_lower, local_upper = body[nearby], lower[nearby], upper[nearby]
            for point in points:
                candidates = np.all((local_lower <= point[:2]) & (local_upper >= point[:2]), axis=1)
                if not np.any(candidates):
                    continue
                hits = intersections(np.array([point[0], point[1], .5]), np.array([0, 0, -1]), local_body[candidates])
                if len(hits):
                    gap = point[2] - (.5 - hits.min())
                    minimum = min(minimum, gap)
                    if "spacer" not in name:
                        main_structure_minimum = min(main_structure_minimum, gap)
                    self.assertGreater(gap, 0, (name, point.tolist(), gap))
        print(f"Minimum sampled clearance to full-resolution body: {minimum*1000:.2f} mm")
        print(f"Minimum sampled clearance excluding provisional mounting spacers: {main_structure_minimum*1000:.2f} mm")

    def test_front_ground_corridor_in_fov_and_not_blocked_by_new_mount(self):
        triangles = []
        for link in self.robot.findall("link"):
            name = link.get("name")
            if not name.startswith("mid360_mount") and name != "camera_link":
                continue
            mesh = link.find("visual/geometry/mesh")
            pose = self.transforms[name]
            local = read_collada(ASSETS / mesh.get("filename").removeprefix("package://go2_description/"))
            triangles.extend(local @ pose[:3, :3].T + pose[:3, 3])
        triangles = np.asarray(triangles)
        origin = self.transforms["livox_frame"][:3, 3]
        for forward in [.5, 1, 2]:
            for lateral in [-.155, 0, .155]:
                direction = np.array([.38 + forward, lateral, -.30]) - origin
                local = ry(35).T @ direction
                elevation = math.degrees(math.atan2(local[2], math.hypot(local[0], local[1])))
                self.assertGreaterEqual(elevation, -7)
                self.assertLessEqual(elevation, 52)
                self.assertGreater(np.linalg.norm(direction), .5)
                hits = intersections(origin, direction, triangles)
                self.assertFalse(np.any(hits < 1))

    def test_camera_fov_clear_and_optical_axes(self):
        optical = self.transforms["camera_depth_optical_frame"]
        camera = self.transforms["camera_link"]
        np.testing.assert_allclose(optical[:3, :3] @ [0, 0, 1], camera[:3, :3] @ [1, 0, 0], atol=1e-10)
        triangles = []
        for link in self.robot.findall("link"):
            name = link.get("name")
            if not name.startswith("mid360_mount") and name != "livox_frame":
                continue
            mesh = link.find("visual/geometry/mesh")
            pose = self.transforms[name]
            local = read_collada(ASSETS / mesh.get("filename").removeprefix("package://go2_description/"))
            triangles.extend(local @ pose[:3, :3].T + pose[:3, 3])
        triangles = np.asarray(triangles)
        # Sample the depth frustum at 2 m, including its four corners.
        for horizontal in [-43.5, 0, 43.5]:
            for vertical in [-29, 0, 29]:
                direction = camera[:3, :3] @ np.array([2, 2 * math.tan(math.radians(horizontal)),
                                                       2 * math.tan(math.radians(vertical))])
                hits = intersections(optical[:3, 3], direction, triangles)
                self.assertFalse(np.any(hits < 1), (horizontal, vertical))

    def test_camera_rear_holes(self):
        triangles = read_collada(MESHES / "body_shell.dae")
        camera = self.transforms["camera_link"]
        mount = self.transforms["mid360_mount_base"]
        world = triangles @ mount[:3, :3].T + mount[:3, 3]
        for lateral in [-.0225, .0225]:
            origin = camera[:3, 3] + camera[:3, :3] @ [.005, lateral, 0]
            direction = camera[:3, :3] @ [-1, 0, 0]
            self.assertFalse(np.any(intersections(origin, direction, world) < .015))
            solid_origin = origin + camera[:3, :3] @ [0, .003, 0]
            self.assertTrue(np.any(intersections(solid_origin, direction, world) < .015))

    def test_official_sensor_interfaces_register_to_bracket(self):
        reference = json.loads((HERE / "reference/mount_interfaces.json").read_text())
        raw = np.array(reference["mid360_bottom_centers_source_mm"])
        rotation = np.array(reference["mid360_source_to_optical_rotation"])
        points = raw @ rotation.T + reference["mid360_source_to_optical_translation_mm"]
        self.assertEqual(len(points), 4)
        expected = np.array(sorted([[x, y, -47] for x in [-24, 24] for y in [-18, 18]]))
        np.testing.assert_allclose(np.array(sorted(points.tolist())), expected, atol=1e-7)
        cylinders = np.array(json.loads((HERE / "reference/d435i_cylinders_m.json").read_text()))
        holes = cylinders[(np.abs(cylinders[:, 2] + .02505) < 1e-9) &
                          (np.abs(cylinders[:, 6] - .00125) < 1e-9)]
        self.assertEqual(len(holes), 2)
        points = holes[:, :3] * 1000 @ np.array(reference["d435i_source_to_rear_rotation"]).T
        points += reference["d435i_source_to_rear_translation_mm"]
        np.testing.assert_allclose(np.array(sorted(points.tolist())), [[0, -22.5, 0], [0, 22.5, 0]], atol=1e-7)

    def test_raised_ribs_do_not_enter_lidar_housing(self):
        vertices, faces = read_stl(CAD / "body_shell.stl")
        points = np.vstack([vertices, vertices[faces].mean(axis=1)]) / 1000
        mount, lidar = self.transforms["mid360_mount_base"], self.transforms["livox_frame"]
        points = points @ mount[:3, :3].T + mount[:3, 3]
        local = (points - lidar[:3, 3]) @ lidar[:3, :3]
        inside = ((np.abs(local[:, 0]) < .0325) & (np.abs(local[:, 1]) < .0325) &
                  (local[:, 2] > -.0469) & (local[:, 2] < -.0075))
        self.assertFalse(inside.any(), "Raised ribs enter the nominal sensor housing")

    def test_head_washer_seats_clear_the_adjacent_wall(self):
        vertices, faces = read_stl(CAD / "body_shell.stl")
        for side in [-1, 1]:
            for angle in np.linspace(0, 2 * np.pi, 12, endpoint=False):
                point = np.array([6 * np.cos(angle), side * 29.25 + 6 * np.sin(angle), 8])
                hits = intersections(point, np.array([0, 0, -1]), vertices[faces])
                self.assertGreater(len(hits), 0)
                self.assertAlmostEqual(point[2] - hits.min(), -.5, delta=.12)

    def test_entire_added_geometry_below_lidar_lower_cone(self):
        lidar = self.transforms["livox_frame"]
        minima = {}
        for link in self.robot.findall("link"):
            name = link.get("name")
            if not name.startswith("mid360_mount") and name != "camera_link":
                continue
            mesh = link.find("visual/geometry/mesh")
            pose = self.transforms[name]
            vertices = np.unique(read_collada(ASSETS / mesh.get("filename").removeprefix("package://go2_description/")).reshape(-1, 3), axis=0)
            vertices = vertices @ pose[:3, :3].T + pose[:3, 3]
            local = (vertices - lidar[:3, 3]) @ lidar[:3, :3]
            margin = -(local[:, 2] + math.tan(math.radians(7)) * np.linalg.norm(local[:, :2], axis=1)) * math.cos(math.radians(7))
            minima[name] = float(margin.min())
            # The below-cone set is convex, so checking all mesh vertices covers
            # every point of every triangle, rather than selected rays alone.
            self.assertGreater(margin.min(), .003, name)
        camera = self.transforms["camera_link"]
        plug = np.array([[x, y, z] for x in [.005, .025] for y in [-.070, -.045] for z in [-.008, .008]])
        plug = plug @ camera[:3, :3].T + camera[:3, 3]
        local = (plug - lidar[:3, 3]) @ lidar[:3, :3]
        margin = -(local[:, 2] + math.tan(math.radians(7)) * np.linalg.norm(local[:, :2], axis=1)) * math.cos(math.radians(7))
        self.assertGreater(margin.min(), .003)
        print(f"D435i full-envelope cone clearance: {minima['camera_link'] * 1000:.2f} mm; USB reserve: {margin.min() * 1000:.2f} mm")

    def test_printed_cradle_single_component_and_print_orientation(self):
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components

        vertices, faces = read_stl(CAD / "body_shell.stl")
        edge = np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
        graph = coo_matrix((np.ones(len(edge)), (edge[:, 0], edge[:, 1])), shape=(len(vertices), len(vertices)))
        self.assertEqual(connected_components(graph, directed=False, return_labels=False), 1)
        printed, printed_faces = read_stl(HERE / "print/pla_cradle.stl")
        np.testing.assert_allclose(printed.min(axis=0), 0, atol=1e-6)
        mass, _, _ = mass_properties(vertices, faces, 1240)
        printed_mass, _, _ = mass_properties(printed, printed_faces, 1240)
        self.assertAlmostEqual(mass, printed_mass, places=6)


if __name__ == "__main__":
    unittest.main()
