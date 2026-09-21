"""Read the vendored Collada geometry for model-level fit checks."""

import xml.etree.ElementTree as ET

import numpy as np
from generate import ASSETS


def read_collada_batches(path):
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(path).getroot()
    effects = {effect.get("id"): np.fromstring(effect.find(".//c:diffuse/c:color", ns).text, sep=" ")[:3]
               for effect in root.findall(".//c:effect", ns) if effect.find(".//c:diffuse/c:color", ns) is not None}
    materials = {material.get("id"): effects.get(material.find("c:instance_effect", ns).get("url")[1:], [.5, .5, .5])
                 for material in root.findall(".//c:library_materials/c:material", ns)}
    batches = []
    for node in root.findall(".//c:visual_scene/c:node", ns):
        matrix = node.find("c:matrix", ns)
        transform = np.eye(4) if matrix is None else np.fromstring(matrix.text, sep=" ").reshape(4, 4)
        for instance in node.findall("c:instance_geometry", ns):
            bindings = {item.get("symbol"): materials.get(item.get("target")[1:], [.5, .5, .5])
                        for item in instance.findall(".//c:instance_material", ns)}
            geometry = root.find(f".//c:geometry[@id='{instance.get('url')[1:]}']/c:mesh", ns)
            sources = {source.get("id"): np.fromstring(source.find("c:float_array", ns).text, sep=" ")
                       for source in geometry.findall("c:source", ns)}
            position_id = geometry.find("c:vertices/c:input[@semantic='POSITION']", ns).get("source")[1:]
            vertices = sources[position_id].reshape(-1, 3)
            vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            for triangles in geometry.findall("c:triangles", ns):
                stride = max(int(item.get("offset")) for item in triangles.findall("c:input", ns)) + 1
                offset = int(triangles.find("c:input[@semantic='VERTEX']", ns).get("offset"))
                indices = np.fromstring(triangles.find("c:p", ns).text, sep=" ", dtype=int).reshape(-1, stride)[:, offset]
                batches.append((vertices[indices].reshape(-1, 3, 3), bindings.get(triangles.get("material"), [.5, .5, .5])))
    return batches


def read_collada(path):
    return np.concatenate([triangles for triangles, _ in read_collada_batches(path)])


if __name__ == "__main__":
    from test_design import intersections

    body = read_collada(ASSETS / "dae/base.dae")
    for x in [.11, .13, .15, .17, .19, .21, .23, .25, .27, .29, .31]:
        heights = []
        for y in [0, .018, .040]:
            hit = intersections(np.array([x, y, .5]), np.array([0, 0, -1]), body)
            heights.append(round((.5 - hit.min()) * 1000, 2) if len(hit) else None)
        print(x, heights)
