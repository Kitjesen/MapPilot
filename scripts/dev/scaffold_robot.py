#!/usr/bin/env python3
"""Scaffold a new robot skeleton for MuJoCo simulation in LingTu.

Usage:
    python scripts/dev/scaffold_robot.py <name> --type quadruped
    python scripts/dev/scaffold_robot.py <name> --type wheeled
    python scripts/dev/scaffold_robot.py <name> --type quadruped --policy path/to/policy.onnx
    python scripts/dev/scaffold_robot.py <name> --type wheeled --meshes path/to/meshes
"""

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path

def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


REPO_ROOT = str(find_repo_root(Path(__file__).resolve().parent))
ROBOTS_DIR = os.path.join(REPO_ROOT, "sim", "robots")

QUADRUPED_MJCF_TEMPLATE = """<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" iterations="50" solver="PGS" gravity="0 0 -9.81">
    <flag frictionloss="enable" sensor="enable"/>
  </option>
  <worldbody>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" rgba="0.2 0.3 0.4 1"/>
    <body name="base_link" pos="0 0 0.45">
      <inertial pos="0 0 0.05" mass="12.0" diaginertia="0.4 0.4 0.2"/>
      <joint name="floating_base_joint" type="free"/>
      <geom name="torso" type="box" size="0.25 0.12 0.06" pos="0 0 0.05" rgba="0.75 0.75 0.75 1"/>
      <camera name="front_camera" pos="0.30 0 0.12" xyaxes="0 -1 0 0 0 1" fovy="60"/>
      <body name="lidar_link" pos="0 0 0.16">
        <geom name="lidar" type="cylinder" size="0.05 0.03" rgba="0.1 0.1 0.1 1"/>
        <site name="lidar_site" pos="0 0 0" size="0.001"/>
      </body>
{legs}
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
  <sensor>
    <framequat name="orientation" objtype="site" objname="imu_site" noise="0.001"/>
    <framepos name="position" objtype="site" objname="imu_site" noise="0.001"/>
    <gyro name="angular_velocity" site="imu_site" noise="0.005"/>
    <velocimeter name="linear_velocity" site="imu_site" noise="0.001"/>
    <accelerometer name="linear_acceleration" site="imu_site" noise="0.005"/>
  </sensor>
</mujoco>
"""

QUADRUPED_LEG_TPL = """\
      <body name="{side}_hip" pos="{hx} {hy} 0.03">
        <inertial pos="0.04 0 0" mass="0.8" diaginertia="0.008 0.008 0.002"/>
        <joint name="{side}_hip_joint" pos="0 0 0" axis="-1 0 0" range="-0.4 0.4"/>
        <geom name="{side}_hip" type="cylinder" size="0.04 0.018" pos="0.04 0 0" rgba="0.7 0.7 0.7 1"/>
        <body name="{side}_thigh" pos="0.07 0 0">
          <inertial pos="-0.10 0 0" mass="1.2" diaginertia="0.02 0.02 0.002"/>
          <joint name="{side}_thigh_joint" pos="0 0 0" axis="0 1 0" range="-1.57 0.52"/>
          <geom name="{side}_thigh" type="box" size="0.11 0.02 0.025" pos="-0.10 0 0" rgba="0.7 0.7 0.7 1"/>
          <body name="{side}_calf" pos="-0.20 0 0">
            <inertial pos="-0.10 0 0" mass="0.6" diaginertia="0.01 0.01 0.001"/>
            <joint name="{side}_calf_joint" pos="0 0 0" axis="0 1 0" range="-2.09 0"/>
            <geom name="{side}_calf" type="box" size="0.10 0.015 0.02" pos="-0.10 0 0" rgba="0.7 0.7 0.7 1"/>
            <body name="{side}_foot" pos="-0.20 0 0">
              <joint name="{side}_foot_joint" pos="0 0 0" axis="0 1 0" limited="false"/>
              <geom name="{side}_foot" type="sphere" size="0.025" rgba="0.3 0.3 0.3 1"/>
            </body>
          </body>
        </body>
      </body>"""

QUADRUPED_LEGS = [
    ("FR",  0.20, -0.09),
    ("FL",  0.20,  0.09),
    ("RR", -0.20, -0.09),
    ("RL", -0.20,  0.09),
]

ACTUATOR_TPL = '    <position name="{side}_{seg}" joint="{side}_{seg}_joint" kp="{kp}" kv="5" ctrlrange="{cr}" forcerange="-120 120"/>'

LEG_ACTUATORS = [
    ("hip",   65, "-0.8 0.8"),
    ("thigh", 95, "-2.0 2.0"),
    ("calf", 120, "-2.5 2.5"),
    ("foot",  30, "-0.5 0.5"),
]

WHEELED_MJCF_TEMPLATE = """<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002" iterations="40" solver="PGS" gravity="0 0 -9.81"/>
  <asset>
    <material name="chassis" rgba="0.12 0.38 0.68 1"/>
    <material name="wheel" rgba="0.15 0.15 0.15 1"/>
  </asset>
  <worldbody>
    <geom name="ground" type="plane" size="0 0 1" pos="0 0 0" rgba="0.2 0.3 0.4 1"/>
    <body name="base_link" pos="0 0 0.15">
      <inertial pos="0 0 0.05" mass="8.0" diaginertia="0.2 0.2 0.1"/>
      <joint name="floating_base_joint" type="free"/>
      <geom name="chassis" type="box" size="0.25 0.15 0.08" pos="0 0 0.08" material="chassis"/>
      <camera name="front_camera" pos="0.30 0 0.18" xyaxes="0 -1 0 0 0 1" fovy="60"/>
      <body name="lidar_link" pos="0 0 0.22">
        <geom name="lidar" type="cylinder" size="0.04 0.025" rgba="0.1 0.1 0.1 1"/>
        <site name="lidar_site" pos="0 0 0" size="0.001"/>
      </body>
{wheels}
    </body>
  </worldbody>
  <actuator>
{actuators}
  </actuator>
</mujoco>
"""

WHEEL_TPL = """\
      <body name="{side}_wheel" pos="{wx} {wy} -0.04">
        <joint name="{side}_wheel_joint" type="hinge" axis="0 1 0" range="-1e4 1e4"/>
        <geom name="{side}_wheel" type="cylinder" size="0.06 0.025" pos="0 0 0" material="wheel" condim="3"/>
      </body>"""

WHEELS = [
    ("FL",  0.20,  0.14),
    ("FR",  0.20, -0.14),
    ("RL", -0.20,  0.14),
    ("RR", -0.20, -0.14),
]

WHEEL_ACTUATOR_TPL = '    <velocity name="{side}_wheel" joint="{side}_wheel_joint" kv="1" ctrlrange="-10 10" forcerange="-20 20"/>'


def _valid_name(name: str) -> bool:
    """Validate robot name.

    Only lowercase alphanumeric and underscores are allowed.
    Hyphens are rejected because the directory name becomes a Python
    package name for ``import_module("sim.robots.<name>")``, and hyphens
    are invalid in Python identifiers (PEP 8).
    """
    return bool(re.match(r"^[a-z][a-z0-9_]*$", name))


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _build_mjcf(name: str, robot_type: str) -> str:
    if robot_type == "quadruped":
        legs_xml = "\n".join(
            QUADRUPED_LEG_TPL.format(side=side, hx=f"{hx:.2f}", hy=f"{hy:.2f}")
            for side, hx, hy in QUADRUPED_LEGS
        )
        actuators_xml = "\n".join(
            ACTUATOR_TPL.format(side=side, seg=seg, kp=kp, cr=cr)
            for side, _, _ in QUADRUPED_LEGS
            for seg, kp, cr in LEG_ACTUATORS
        )
        return QUADRUPED_MJCF_TEMPLATE.format(name=name, legs=legs_xml, actuators=actuators_xml)
    else:
        wheels_xml = "\n".join(
            WHEEL_TPL.format(side=side, wx=f"{wx:.2f}", wy=f"{wy:.2f}")
            for side, wx, wy in WHEELS
        )
        actuators_xml = "\n".join(
            WHEEL_ACTUATOR_TPL.format(side=side)
            for side, _, _ in WHEELS
        )
        return WHEELED_MJCF_TEMPLATE.format(name=name, wheels=wheels_xml, actuators=actuators_xml)


def _build_init(name: str, robot_type: str) -> str:
    return f'''"""Robot scaffold for {name} ({robot_type})."""

# TODO: import register_robot from sim framework when available
# from sim.registry import register_robot


def register():
    """Register {name} with the simulation framework.

    Call this from the simulation setup to load {name} assets.
    """
    # Example:
    # register_robot(
    #     name="{name}",
    #     mjcf="sim/robots/{name}/{name}.xml",
    #     robot_type="{robot_type}",
    # )
    pass
'''


def _build_manifest(policy_path: str) -> str:
    sha = _sha256(policy_path)
    fname = os.path.basename(policy_path)
    return json.dumps(
        {
            "schema_version": "lingtu.sim_policy_manifest.v1",
            "asset": fname,
            "sha256": sha,
            "source_note": "Scaffolded by scripts/dev/scaffold_robot.py",
            "simulation_only": True,
            "real_robot_motion": False,
        },
        indent=2,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Scaffold a new robot skeleton.")
    parser.add_argument("name", type=str, help="Robot name (alphanumeric, underscores, hyphens)")
    parser.add_argument("--type", required=True, choices=["quadruped", "wheeled"],
                        help="Robot type")
    parser.add_argument("--meshes", type=str, default=None,
                        help="Path to mesh/stl directory (copied as-is reference)")
    parser.add_argument("--policy", type=str, default=None,
                        help="Path to ONNX policy file")
    args = parser.parse_args()

    # --- Validate inputs ---
    if not _valid_name(args.name):
        print(f"Error: '{args.name}' is not a valid robot name. "
              f"Use lowercase alphanumeric and underscores only "
              f"(hyphens break Python module imports).", file=sys.stderr)
        sys.exit(1)

    dest = os.path.join(ROBOTS_DIR, args.name)
    if os.path.exists(dest):
        print(f"Error: '{dest}' already exists.", file=sys.stderr)
        sys.exit(1)

    if args.meshes and not os.path.isdir(args.meshes):
        print(f"Error: --meshes '{args.meshes}' is not a directory.", file=sys.stderr)
        sys.exit(1)

    if args.policy:
        if not os.path.isfile(args.policy):
            print(f"Error: --policy '{args.policy}' is not a file.", file=sys.stderr)
            sys.exit(1)
        if not args.policy.endswith(".onnx"):
            print(f"Warning: '{args.policy}' does not end in .onnx — continuing anyway.",
                  file=sys.stderr)

    # --- Create directory ---
    os.makedirs(dest, exist_ok=False)

    # --- Write MJCF ---
    mjcf = _build_mjcf(args.name, args.type)
    mjcf_path = os.path.join(dest, f"{args.name}.xml")
    with open(mjcf_path, "w", encoding="utf-8") as f:
        f.write(mjcf)
    print(f"Created  {mjcf_path}")

    # --- Write __init__.py ---
    init_path = os.path.join(dest, "__init__.py")
    with open(init_path, "w", encoding="utf-8") as f:
        f.write(_build_init(args.name, args.type))
    print(f"Created  {init_path}")

    # --- Copy meshes reference ---
    if args.meshes:
        meshes_dest = os.path.join(dest, "meshes")
        try:
            os.symlink(os.path.abspath(args.meshes), meshes_dest)
            print(f"Meshes:  symlink {args.meshes} -> {meshes_dest}")
        except (OSError, NotImplementedError, AttributeError) as e:
            # Fallback: copy the mesh directory
            import shutil
            shutil.copytree(args.meshes, meshes_dest, symlinks=True)
            print(f"Meshes:  copied {args.meshes} -> {meshes_dest} (symlink not available: {e})")

    # --- Write policy manifest ---
    if args.policy:
        manifest = _build_manifest(args.policy)
        manifest_path = os.path.join(dest, "policy_manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as f:
            f.write(manifest)
        print(f"Created  {manifest_path}")
        print(f"SHA256:  {json.loads(manifest)['sha256']}")

    # --- Checklist ---
    print()
    print("Next steps:")
    print(f"  1. Edit {mjcf_path} — adjust masses, inertias, geometry for your robot.")
    print(f"  2. Edit {init_path} — wire up the registration call.")
    if args.type == "quadruped":
        print("  3. Add a gait config: sim/configs/<name>_gait.json")
        print("  4. Test in simulation: python lingtu.py sim --robot " + args.name)
    else:
        print("  3. Set wheel friction/motor gains in the MJCF for your surface.")
        print("  4. Test in simulation: python lingtu.py sim --robot " + args.name)
    if args.meshes:
        print(f"  5. Update mesh paths in {mjcf_path} to point to sim/robots/{args.name}/meshes/")
    print()


if __name__ == "__main__":
    main()
