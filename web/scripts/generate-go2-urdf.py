"""Extract the browser's Go2 visual tree from the vendored, unchanged URDF."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


WEB = Path(__file__).resolve().parents[1]
SOURCE = WEB / "public/assets/robots/go2/urdf/go2_description.urdf"
TARGET = WEB / "src/components/scene3d/robot/go2UrdfSpec.ts"


def vector(element, name, default="0 0 0"):
    return [float(value) for value in (element.get(name, default) if element is not None else default).split()]


def origin(element):
    return {"xyz": vector(element, "xyz"), "rpy": vector(element, "rpy")}


def generate():
    robot = ET.parse(SOURCE).getroot()
    links = []
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError(f"Unsupported Go2 visual geometry: {link.get('name')}")
            filename = mesh.get("filename")
            prefix = "package://go2_description/"
            if not filename.startswith(prefix):
                raise ValueError(filename)
            visuals.append({"mesh": filename.removeprefix(prefix), "origin": origin(visual.find("origin")),
                            "scale": vector(mesh, "scale", "1 1 1")})
        links.append({"name": link.get("name"), "visuals": visuals})
    joints = [{"name": joint.get("name"), "type": joint.get("type"),
               "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
               "origin": origin(joint.find("origin")), "axis": vector(joint.find("axis"), "xyz", "1 0 0")}
              for joint in robot.findall("joint")]
    rows = ["// Generated from public/assets/robots/go2/urdf/go2_description.urdf.",
            "// Regenerate with: python web/scripts/generate-go2-urdf.py", "export const GO2_URDF = {", "  links: ["]
    rows.extend("    " + json.dumps(value, separators=(",", ":")) + "," for value in links)
    rows.extend(["  ],", "  joints: ["])
    rows.extend("    " + json.dumps(value, separators=(",", ":")) + "," for value in joints)
    rows.extend(["  ],", "} as const", ""])
    return "\n".join(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    generated = generate()
    if args.check:
        if TARGET.read_text(encoding="utf-8") != generated:
            raise SystemExit("Go2 visual tree differs from its source URDF; regenerate it.")
        print("Go2 visual tree matches the official URDF.")
    else:
        TARGET.write_text(generated, encoding="utf-8", newline="\n")
        print(TARGET)
