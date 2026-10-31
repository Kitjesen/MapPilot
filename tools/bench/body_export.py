"""Export fixed-pose URDF collision primitives for the native body probe.

MuJoCo performs offline forward kinematics only. Primitive dimensions and OBB
broad-phase bounds require no MuJoCo dependency in their consumer.
This snapshot does not describe gait or a field-qualified installation.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_URDF = ROOT / "web/public/assets/robots/go2/urdf/go2_mid360_flatpack.urdf"
sys.path.insert(0, str(ROOT))

from tools.bench.body_model import BodyModel


def native_body_text(parts: list[dict]) -> str:
    """Retain primitives and ground-contact labels; the final BODY2 bit is reserved."""
    lines = [f"BODY2 {len(parts)}"]
    for part in parts:
        values = [*part["center"], *(value for row in part["rotation"] for value in row),
                  *part["half_size"]]
        numeric = " ".join(format(float(value), ".17g") for value in values)
        lines.append(f"{part['link']} {numeric} {part['source_type']} "
                     f"{part['radius_m']:.17g} {part['half_length_m']:.17g} "
                     f"{int(part['support_contact'])} 0")
    return "\n".join(lines) + "\n"


def export_body(
    urdf: Path,
    output: Path,
    joint_angles: dict[str, float] | None = None,
    support_contact_links: tuple[str, ...] = (),
) -> dict:
    """Write each source primitive and conservative bounds at the supplied pose."""
    urdf, output = Path(urdf), Path(output)
    if output.suffix.lower() == ".json":
        raise ValueError("The native text output and its .json metadata need distinct paths")
    body = BodyModel(urdf, [], joint_angles=joint_angles)
    geometry = body.geometry([0.0, 0.0, 0.0, 0.0])
    parts = []
    missing_support = set(support_contact_links) - {geom["link"] for geom in geometry}
    if missing_support:
        raise ValueError(f"Support contact links have no collision geometry: {sorted(missing_support)}")
    for geom in geometry:
        link = geom["link"]
        if not link or any(character.isspace() for character in link):
            raise ValueError("Native text part labels require link names without whitespace")
        source_type, size = geom["type"], geom["size"]
        if source_type == "box":
            half_size = list(size)
        elif source_type == "cylinder":
            half_size = [size[0], size[0], size[1]]
        elif source_type == "sphere":
            half_size = [size[0]] * 3
        else:
            raise ValueError(f"Unsupported collision geometry {source_type!r}; it was not discarded")
        # A sphere is invariant under rotation. Keeping an arbitrary sphere
        # frame would rotate its enclosing cube and inflate axis bounds again.
        rotation = ([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                    if source_type == "sphere" else geom["rotation"])
        part = {
            "link": link,
            "center": geom["center"],
            "rotation": rotation,
            "half_size": half_size,
            "source_type": source_type,
            "radius_m": float(size[0]) if source_type in {"sphere", "cylinder"} else 0.0,
            "half_length_m": float(size[1]) if source_type == "cylinder" else 0.0,
            "support_contact": link in support_contact_links,
            "source_rotation": geom["rotation"],
            "axes_policy": ("URDF-root identity axes; sphere orientation is immaterial"
                            if source_type == "sphere" else "source collision axes"),
        }
        parts.append(part)

    model_name = ET.parse(urdf).getroot().get("name", "")
    report = {
        "urdf": body.report["urdf"],
        "model_name": model_name,
        "primitive_count": len(parts),
        "representation": "source primitive with conservative per-part OBB broad phase",
        "support_contact_links": list(support_contact_links),
        "frame": "URDF root, metres; center and rotation at the recorded joint pose",
        "rotation_layout": "row-major local part axes to URDF root",
        "sphere_obb_axes": "URDF-root identity; source_rotation retained for traceability",
        "geometry_margin_m": 0.0,
        "pose_scope": "fixed joint-pose snapshot; runtime forward kinematics and gait are not included",
        "joint_angles": body.report["joint_angles"],
        "model_report": body.report,
        "known_missing_geometry": (["external D435i and its bracket"]
                                   if model_name.startswith("go2_mid360")
                                   and not any("d435" in part["link"].lower() for part in parts) else []),
        "installation_qualified": False,
        "parts": parts,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(native_body_text(parts), encoding="utf-8")
    output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--joints", type=Path, help="JSON object of named joint-angle overrides in radians")
    parser.add_argument("--support-contact-link", action="append", default=[],
                        help="Explicit collision link allowed and required to touch observed support")
    args = parser.parse_args()
    joints = json.loads(args.joints.read_text(encoding="utf-8")) if args.joints else None
    if joints is not None and not isinstance(joints, dict):
        parser.error("--joints must contain a JSON object of joint names and radians")
    report = export_body(args.urdf, args.output, joints, tuple(args.support_contact_link))
    print(json.dumps({"output": str(args.output), "metadata": str(args.output.with_suffix('.json')),
                      "primitive_count": report["primitive_count"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
