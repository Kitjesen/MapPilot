"""Validate the native leg diagnostic against a source URDF and MuJoCo kinematics.

This is a local model/interface check. It neither detects physical contact nor
calibrates hardware, supplies independent odometry covariance or updates SLAM.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
GO2_URDF = ROOT / "web/public/assets/robots/go2/urdf/go2_description.urdf"
SDK_LEGS = ("FR", "FL", "RR", "RL")
SDK_NAMES = tuple(f"{leg}_{joint}_joint" for leg in SDK_LEGS for joint in ("hip", "thigh", "calf"))
DEFAULT_PROBE = ROOT / "build/leg-kinematics-windows/Release/leg_kinematics_probe.exe"


@dataclass(frozen=True)
class LegModel:
    joint_names: tuple[str, ...]
    origins: tuple[np.ndarray, ...]
    axes: tuple[np.ndarray, ...]
    last_link_from_tip: np.ndarray
    tip_link: str
    robot_model: str
    frame_id: str


def _origin(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    transform = np.eye(4)
    if origin is not None:
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
            raise ValueError("invalid URDF origin")
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
    return transform


def load_leg(urdf: Path, tip_link: str, *, robot_model: str, frame_id: str, base_link: str = "base") -> LegModel:
    """Collapse fixed joints while retaining each named revolute joint frame."""
    robot = ET.parse(urdf).getroot()
    by_child = {}
    for joint in robot.findall("joint"):
        child = joint.find("child").get("link")
        if child in by_child:
            raise ValueError(f"multiple parents for URDF link {child}")
        by_child[child] = joint
    path, current = [], tip_link
    while current != base_link:
        if current not in by_child or current in {child for child, _ in path}:
            raise ValueError(f"no acyclic {base_link} to {tip_link} chain")
        joint = by_child[current]
        path.append((current, joint))
        current = joint.find("parent").get("link")
    names, origins, axes = [], [], []
    pending = np.eye(4)
    for _, joint in reversed(path):
        pending = pending @ _origin(joint)
        if joint.get("type") == "fixed":
            continue
        if joint.get("type") not in {"revolute", "continuous"}:
            raise ValueError("only fixed/revolute leg chains are supported")
        axis_element = joint.find("axis")
        axis = np.fromstring(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0", sep=" ")
        if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis), 1):
            raise ValueError("invalid joint axis")
        names.append(joint.get("name"))
        origins.append(pending)
        axes.append(axis)
        pending = np.eye(4)
    if not names:
        raise ValueError("leg chain has no revolute joints")
    return LegModel(tuple(names), tuple(origins), tuple(axes), pending, tip_link, robot_model, frame_id)


def map_joint_sample(model: LegModel, sample: dict) -> dict:
    """Receive-time samples permit FK but explicitly cannot authorize velocity."""
    if sample.get("robot_model") != model.robot_model:
        raise ValueError("robot model mismatch")
    if sample.get("frame_id") != model.frame_id:
        raise ValueError("unknown joint reference frame")
    names = sample.get("names", [])
    if len(names) != len(set(names)):
        raise ValueError("duplicate joint name")
    position = np.asarray(sample.get("position", []), dtype=float)
    velocity = np.asarray(sample.get("velocity", []), dtype=float)
    if position.shape != (len(names),) or velocity.shape != (len(names),):
        raise ValueError("joint arrays do not match names")
    if not np.isfinite(position).all() or not np.isfinite(velocity).all():
        raise ValueError("nonfinite joint state")
    if not set(model.joint_names).issubset(names):
        raise ValueError("missing required joint")
    basis = sample.get("time_basis")
    if basis not in {"aligned_measurement", "receive_time"}:
        raise ValueError("unknown joint time basis")
    stamp = float(sample.get("stamp_s", math.nan))
    if not math.isfinite(stamp):
        raise ValueError("invalid joint time")
    indices = [names.index(name) for name in model.joint_names]
    return {"q": position[indices], "dq": velocity[indices], "joint_stamp_s": stamp,
            "timestamps_aligned": basis == "aligned_measurement"}


def probe_cases(probe: Path, cases: list[dict]) -> list[dict]:
    """Batch local cases; query defaults only define offline freshness checks."""
    if not probe.is_file():
        raise FileNotFoundError(f"native leg probe is unavailable: {probe}")
    lines = []
    for case in cases:
        model = case["model"]
        contact = case.get("contact", "unknown")
        aligned = case.get("timestamps_aligned", False)
        if contact == "stance" and aligned and not {"omega", "gyro_stamp_s", "joint_stamp_s"}.issubset(case):
            raise ValueError("aligned stance diagnostic requires an explicit gyro measurement and joint timestamp")
        values = [len(model.joint_names)]
        for origin, axis in zip(model.origins, model.axes):
            values.extend(origin.ravel())
            values.extend(axis)
        for item in (model.last_link_from_tip, case.get("point_tip", np.zeros(3)), case["q"], case["dq"],
                     case.get("omega", np.zeros(3)), case.get("origin", np.zeros(3)), case.get("predicted", np.zeros(3))):
            values.extend(np.asarray(item).ravel())
        stamp = case.get("joint_stamp_s", 10.0)
        values.extend((stamp, case.get("gyro_stamp_s", stamp), case.get("query_stamp_s", stamp + .001),
                       case.get("max_age_s", .02), case.get("max_skew_s", .002),
                       int(aligned)))
        lines.append(" ".join(format(float(value), ".17g") for value in values) + " " + contact)
    completed = subprocess.run([str(probe)], input="\n".join(lines) + "\n", capture_output=True,
                               text=True, check=True, timeout=30)
    results = [json.loads(line) for line in completed.stdout.splitlines()]
    if len(results) != len(cases):
        raise ValueError("native probe returned an incomplete batch")
    return results


class MujocoOracle:
    def __init__(self, urdf: Path):
        import mujoco

        self.mujoco = mujoco
        robot = ET.parse(urdf).getroot()
        # Keep every physical joint/link transform. Visual meshes are irrelevant
        # to FK and their DAE package URLs are unavailable to the URDF importer.
        for link in robot.findall("link"):
            for visual in link.findall("visual"):
                link.remove(visual)
        extension = robot.find("mujoco")
        if extension is None:
            extension = ET.SubElement(robot, "mujoco")
        compiler = extension.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(extension, "compiler")
        compiler.set("fusestatic", "false")
        compiler.set("discardvisual", "true")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
        self.data = mujoco.MjData(self.model)

    def evaluate(self, leg: LegModel, q: np.ndarray, point_tip: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.data.qpos[:] = self.model.qpos0
        columns = []
        for name, angle in zip(leg.joint_names, q):
            joint = self.model.joint(name)
            self.data.qpos[int(joint.qposadr[0])] = angle
            columns.append(int(joint.dofadr[0]))
        self.mujoco.mj_forward(self.model, self.data)
        body = self.model.body(leg.tip_link).id
        point = self.data.xpos[body] + self.data.xmat[body].reshape(3, 3) @ point_tip
        jacobian = np.zeros((3, self.model.nv))
        rotation_jacobian = np.zeros_like(jacobian)
        self.mujoco.mj_jac(self.model, self.data, jacobian, rotation_jacobian, point, body)
        return point.copy(), jacobian[:, columns]


def validate(probe: Path, urdf: Path = GO2_URDF, samples_per_leg: int = 12, real_snapshot: Path | None = None) -> dict:
    if samples_per_leg < 1:
        raise ValueError("at least one model sample is required")
    oracle = MujocoOracle(urdf)
    rng = np.random.default_rng(20261021)
    cases, references = [], []
    step = 1e-6
    for leg_name in SDK_LEGS:
        leg = load_leg(urdf, f"{leg_name}_foot", robot_model="go2", frame_id="body")
        angles = [np.array([0., .8, -1.6])]
        angles += [rng.uniform([-.5, .2, -2.3], [.5, 1.3, -.7]) for _ in range(samples_per_leg - 1)]
        for q in angles:
            # Both frame origin and a separate material point are checked;
            # neither is asserted to be the actual ground-contact point.
            for point_tip in (np.zeros(3), np.array([-.002, .007, -.022])):
                position, jacobian = oracle.evaluate(leg, q, point_tip)
                finite_difference = np.column_stack([
                    (oracle.evaluate(leg, q + np.eye(3)[i] * step, point_tip)[0] -
                     oracle.evaluate(leg, q - np.eye(3)[i] * step, point_tip)[0]) / (2 * step)
                    for i in range(3)])
                dq, omega = rng.uniform(-.5, .5, 3), rng.uniform(-.4, .4, 3)
                origin = np.array([.071, -.026, .044])
                predicted = -np.cross(omega, position - origin) - jacobian @ dq + np.array([.07, -.03, .02])
                # Independent time derivative of the moving body's material
                # point, using MuJoCo FK and a rotating velocity reference.
                moving_points = []
                for time in (-step, step):
                    rotation = Rotation.from_rotvec(omega * time).as_matrix()
                    translation = predicted * time - rotation @ origin + origin
                    moving_points.append(translation + rotation @ oracle.evaluate(leg, q + dq * time, point_tip)[0])
                contact_derivative = (moving_points[1] - moving_points[0]) / (2 * step)
                cases.append({"model": leg, "q": q, "dq": dq, "point_tip": point_tip,
                              "omega": omega, "origin": origin, "predicted": predicted,
                              "joint_stamp_s": 10., "gyro_stamp_s": 10.,
                              "timestamps_aligned": True, "contact": "stance"})
                references.append((position, jacobian, finite_difference, contact_derivative))
    results = probe_cases(probe, cases)
    maxima = {"fk_error_m": 0., "jacobian_error_m_per_rad": 0., "jacobian_fd_error_m_per_rad": 0.,
              "contact_time_derivative_error_mps": 0.}
    for result, reference in zip(results, references):
        position, jacobian, finite_difference, derivative = reference
        if not result["usable"]:
            raise AssertionError(f"aligned synthetic stance case rejected: {result}")
        errors = (np.max(np.abs(np.asarray(result["point"]) - position)),
                  np.max(np.abs(np.asarray(result["jacobian"]) - jacobian)),
                  np.max(np.abs(np.asarray(result["jacobian"]) - finite_difference)),
                  np.max(np.abs(np.asarray(result["residual"]) - derivative)))
        for key, error in zip(maxima, errors):
            maxima[key] = max(maxima[key], float(error))
    limits = (1e-11, 1e-11, 2e-8, 2e-8)
    for (name, value), limit in zip(maxima.items(), limits):
        if value > limit:
            raise AssertionError(f"{name}={value} exceeds {limit}")
    report = {"passed": True, "scope": "local_model_and_input_contract_only", "urdf": str(urdf.resolve()),
              "model_source": str(urdf.parent.parent / "README.md"), "probe": str(probe.resolve()),
              "oracle": f"MuJoCo {oracle.mujoco.__version__} mj_jac + finite differences",
              "legs": list(SDK_LEGS), "cases": len(cases), "samples_per_leg": samples_per_leg,
              "max_errors": maxima, "selected_points": ["foot_frame_origin", "explicit_tip_material_point"],
              "hardware_calibration_validated": False, "physical_contact_detected": False,
              "dynamics_or_gait_validated": False, "filter_fusion_enabled": False}
    if real_snapshot is not None:
        raw = json.loads(real_snapshot.read_text(encoding="utf-8"))
        if raw.get("order") != "FR/FL/RR/RL:hip/thigh/calf":
            raise ValueError("unknown acquisition motor order")
        sample = {"robot_model": "go2", "frame_id": "body", "names": list(SDK_NAMES),
                  "position": raw["position"], "velocity": raw["velocity"],
                  "stamp_s": raw["sample_receive_stamp_s"], "time_basis": "receive_time"}
        real_cases = []
        for leg_name in SDK_LEGS:
            leg = load_leg(urdf, f"{leg_name}_foot", robot_model="go2", frame_id="body")
            mapped = map_joint_sample(leg, sample)
            real_cases.append({"model": leg, **mapped, "contact": "unknown"})
        real_results = probe_cases(probe, real_cases)
        errors = []
        for case, result in zip(real_cases, real_results):
            expected = oracle.evaluate(case["model"], case["q"], np.zeros(3))[0]
            errors.append(float(np.max(np.abs(np.asarray(result["point"]) - expected))))
            if errors[-1] > limits[0]:
                raise AssertionError(f"real joint snapshot FK error {errors[-1]} exceeds {limits[0]}")
            if result["usable"] or "velocity" in result:
                raise AssertionError("receive-time sample with no contact/gyro fabricated a velocity")
        report["existing_real_snapshot"] = {"path": str(real_snapshot.resolve()), "max_fk_error_m": max(errors),
            "captured_samples": raw["received"], "retained_joint_samples": 1,
            "velocity_usable": False, "reason": "receive timestamp; no measured contact or gyro sample",
            "foot_frame_positions_body_m": {leg: result["point"] for leg, result in zip(SDK_LEGS, real_results)}}
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, default=DEFAULT_PROBE)
    parser.add_argument("--urdf", type=Path, default=GO2_URDF)
    parser.add_argument("--samples-per-leg", type=int, default=12)
    parser.add_argument("--real-snapshot", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.probe, args.urdf, args.samples_per_leg, args.real_snapshot)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
