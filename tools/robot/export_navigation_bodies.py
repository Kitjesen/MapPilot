"""Regenerate the selected navigation models from checked-in robot assets.

Run with the existing development MuJoCo environment. MuJoCo is an offline
export tool here, not a dependency of native field navigation.
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from tools.bench.body_export import export_body, native_body_text
from tools.bench.body_model import BodyModel
from tools.bench.body_physics import export_body as export_mjcf_body

FEET = tuple(f"{leg}_foot" for leg in ("FL", "FR", "RL", "RR"))
GO2_BODY = Path("config/robots/unitree/go2/body/go2_raised_nominal.body2")
THUNDER_BODY = Path("config/robots/doso/thunder_v4/body/thunderv4_navigation.body2")
THUNDER_SIM_BODY = Path("sim/packages/robots/doso/thunder_v4/collision/thunderv4_navigation.body2")
REGENERATION = "python tools/robot/export_navigation_bodies.py"


def write_report(output: Path, report: dict) -> None:
    output.write_text(native_body_text(report["parts"]), encoding="utf-8")
    output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def export_go2(output: Path) -> dict:
    """Fit a nominal standing pose to the configured, previously observed height."""
    source = ROOT / "web/public/assets/robots/go2/urdf/go2_mid360_raised_frame.urdf"
    config_path = Path("config/robots/unitree/go2/robot.yaml")
    geometry = yaml.safe_load((ROOT / config_path).read_text(encoding="utf-8"))["geometry"]
    reference = BodyModel(source, [])

    def lowest(angle: float) -> float:
        for leg in ("FL", "FR", "RL", "RR"):
            for name, value in ((f"{leg}_thigh_joint", angle), (f"{leg}_calf_joint", -2 * angle)):
                joint = reference.model.joint(name)
                reference.data.qpos[reference.model.jnt_qposadr[joint.id]] = value
        return min(part["center"][2] - part["size"][0]
                   for part in reference.geometry([0, 0, 0, 0]) if part["link"] in FEET)

    low, high = .1, 1.2
    height = float(geometry["support_height"])
    for _ in range(50):
        middle = (low + high) * .5
        if lowest(middle) < -height:
            low = middle
        else:
            high = middle
    angle = (low + high) * .5
    joints = {f"{leg}_{part}_joint": value for leg in ("FL", "FR", "RL", "RR")
              for part, value in (("hip", 0.), ("thigh", angle), ("calf", -2 * angle))}
    report = export_body(source, output, joints, FEET)
    # Nominal lower-leg/foot geometry is not the moving robot's stepping envelope.
    excluded = sorted({part["link"] for part in report["parts"]
                       if "_calf" in part["link"] or "_foot" in part["link"]})
    report["parts"] = [part for part in report["parts"] if part["link"] not in excluded]
    report["support_contact_links"] = []
    report["excluded_navigation_links"] = excluded

    # Preserve the declared upper payload space without claiming measured camera CAD.
    bottom, top = .057, float(geometry["collision_clearance_above"])
    report["parts"].append({
        "link": "unmeasured_payload_cover", "center": [0., 0., (bottom + top) / 2],
        "rotation": np.eye(3).tolist(),
        "half_size": [geometry["vehicle_length"] / 2, geometry["vehicle_width"] / 2, (top - bottom) / 2],
        "source_type": "box", "radius_m": 0., "half_length_m": 0.,
        "support_contact": False,
    })
    report.update(
        primitive_count=len(report["parts"]),
        planning_scope="upper body, hips, thighs and payload; lower legs and feet excluded; not full-body collision clearance",
        pose_scope="fixed nominal pose fitted to configured support height; not measured joints or gait",
        nominal_support_height_m=height,
        nominal_fit={"source": f"{config_path.as_posix()}:geometry.support_height",
                     "constraint": "symmetric thigh=a, calf=-2a, hip=0; raw foot sphere bottoms at -support_height",
                     "thigh_rad": angle, "foot_bottom_body_z_m": lowest(angle)},
        conservative_covers=[{
            "link": "unmeasured_payload_cover", "source": "existing declared upper navigation envelope",
            "source_config": config_path.as_posix(), "body_z_interval_m": [bottom, top],
            "xy_size_m": [geometry["vehicle_length"], geometry["vehicle_width"]],
            "meaning": "coverage inside previously declared upper envelope only; not measured D435i or bracket CAD; actual installed extent remains unverified",
        }],
        runtime_margin_source="RobotConfig collision_hard_margin; not baked into raw primitives",
        model_name="go2_raised_nominal", regeneration=REGENERATION,
    )
    report["model_report"]["joint_state_source"] = "nominal Go2 pose fitted to configured support height, not measured"
    write_report(output, report)
    return report


def _bounds(parts: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    """Project the source primitives themselves, including round cylinder ends."""
    minimum, maximum = [], []
    for part in parts:
        rotation = np.asarray(part["rotation"])
        if part["source_type"] == "box":
            extent = np.abs(rotation) @ np.asarray(part["half_size"])
        elif part["source_type"] == "sphere":
            extent = np.repeat(part["radius_m"], 3)
        else:
            axial = np.abs(rotation[:, 2]) * part["half_length_m"]
            if part["source_type"] == "capsule":
                extent = axial + part["radius_m"]
            else:
                extent = axial + part["radius_m"] * np.linalg.norm(rotation[:, :2], axis=1)
        center = np.asarray(part["center"])
        minimum.append(center - extent)
        maximum.append(center + extent)
    return np.min(minimum, axis=0), np.max(maximum, axis=0)


def _navigation_box(name: str, parts: list[dict], height_parts: list[dict] | None = None) -> dict:
    minimum, maximum = _bounds(parts)
    height_parts = parts if height_parts is None else height_parts
    height_minimum, height_maximum = _bounds(height_parts)
    minimum[2], maximum[2] = height_minimum[2], height_maximum[2]
    return {
        "link": name, "center": ((minimum + maximum) / 2).tolist(),
        "rotation": np.eye(3).tolist(), "half_size": ((maximum - minimum) / 2).tolist(),
        "source_type": "box", "radius_m": 0., "half_length_m": 0.,
        "support_contact": False,
        "footprint_source_parts": [part["link"] for part in parts],
        "height_source_parts": [part["link"] for part in height_parts],
    }


def export_thunder(output: Path, sim_output: Path) -> dict:
    """Keep the chassis outline and four nominal leg footprints in eight parts."""
    source = ROOT / "sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml"
    model = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "v4_nominal_stand")
    if key < 0:
        raise ValueError("Thunder source has no v4_nominal_stand keyframe")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)
    base = model.body("base_link").id
    state = SimpleNamespace(position=np.array(data.xpos[base]),
                            orientation=np.array(data.xquat[base])[[1, 2, 3, 0]])
    engine = SimpleNamespace(model=model, data=data, sim_time=0., get_robot_state=lambda: state)
    output.parent.mkdir(parents=True, exist_ok=True)
    exported = export_mjcf_body(engine, output, FEET)
    source_parts = exported["parts"]
    chassis = [part for part in source_parts if part["body"] == "base_link"]
    # The first three source boxes form the central chassis. Keep the narrow
    # nose, tail and raised sensor separate so their empty corners stay empty.
    parts = [_navigation_box("chassis", chassis[:3])]
    parts.extend(_navigation_box(name, [part]) for name, part in
                 zip(("nose", "tail", "sensor"), chassis[3:], strict=True))
    for leg in ("FR", "FL", "RR", "RL"):
        footprint = [part for part in source_parts if part["body"].startswith(f"{leg}_")]
        upper = [part for part in footprint if part["body"].endswith(("_hip", "_thigh"))]
        # Preserve the entire nominal leg/wheel XY occupation without treating
        # fixed wheel-ground intersections as rigid upper-body collisions.
        parts.append(_navigation_box(f"{leg}_clearance", footprint, upper))
    upper = [part for part in source_parts
             if part["body"] == "base_link" or part["body"].endswith(("_hip", "_thigh"))]
    minimum, maximum = _bounds(parts)
    source_bounds = []
    for part in source_parts:
        low, high = _bounds([part])
        source_bounds.append({"link": part["link"], "body": part["body"],
                              "minimum": low.tolist(), "maximum": high.tolist()})
    report = dict(
        robot=exported["robot"], primitive_count=len(parts), parts=parts,
        source_mjcf=source.relative_to(ROOT).as_posix(), model_name="thunderv4_navigation",
        source_keyframe="v4_nominal_stand", source_qpos=data.qpos.tolist(),
        joint_angles={model.joint(j).name: float(data.qpos[model.jnt_qposadr[j]])
                      for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE},
        source_primitive_count=len(source_parts),
        footprint_source_bodies=sorted({part["body"] for part in source_parts}),
        height_source_bodies=sorted({part["body"] for part in upper}),
        excluded_height_bodies=sorted({part["body"] for part in source_parts if part not in upper}),
        source_bounds=source_bounds,
        envelope_bounds={"minimum": minimum.tolist(), "maximum": maximum.tolist(),
                         "size": (maximum - minimum).tolist()},
        support_contact_links=[], frame="base_link, metres; fixed parts placed with query yaw",
        pose_scope="nominal XY footprint and chassis/hip/thigh height from MJCF keyframe",
        planning_scope="four chassis parts and four leg-clearance proxies; full nominal XY coverage, upper-body Z; not full-body 3D or gait clearance",
        physical_drive="offline forward kinematics only; no ONNX controller or physics rollout",
        geometry_margin_m=0.,
        runtime_margin_source="RobotConfig collision_hard_margin; not baked into raw primitives",
        representation="eight-part compound navigation model", installation_qualified=False,
        known_missing_geometry=[],
        source_asset_boundary="checked-in MJCF nominal stance; calves/wheels contribute XY only; low obstacles, gait and wheel clearance remain unqualified",
        regeneration=REGENERATION,
    )
    write_report(output, report)
    sim_output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(output, sim_output)
    shutil.copyfile(output.with_suffix(".json"), sim_output.with_suffix(".json"))
    return report


def main() -> None:
    go2 = export_go2(ROOT / GO2_BODY)
    thunder = export_thunder(ROOT / THUNDER_BODY, ROOT / THUNDER_SIM_BODY)
    print(json.dumps({"go2_parts": go2["primitive_count"],
                      "go2_foot_bottom_body_z_m": go2["nominal_fit"]["foot_bottom_body_z_m"],
                      "thunder_parts": thunder["primitive_count"]}, indent=2))


if __name__ == "__main__":
    main()
