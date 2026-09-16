"""Promote a reviewed native campus export into an exact-version WorldPackage.

This copies the authored mesh/lightmap pairs and relinks the MJCF inside the
package. It does not rebuild geometry, change physics, or launch a Product.
"""

from __future__ import annotations

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def install_campus(source: Path, package_dir: Path) -> dict:
    source, package_dir = source.resolve(), package_dir.resolve()
    if package_dir.exists():
        raise FileExistsError(f"World version already exists: {package_dir}")
    scene = ET.parse(source / "native/campus.xml")
    for geom in scene.getroot().findall("./worldbody/geom"):
        if geom.get("group") == "3" and geom.get("contype") == "0" and geom.get("conaffinity") == "0":
            # Runtime group 3 is reserved for hidden robot collision geometry.
            geom.set("group", "2")
    files = []
    for asset in scene.getroot().find("asset"):
        filename = asset.get("file")
        if filename:
            origin = source / "native" / filename
            if not origin.is_file():
                raise FileNotFoundError(origin)
            files.append(origin)
            asset.set("file", "../visual/native/" + origin.name)
    package_dir.mkdir(parents=True)
    physics = package_dir / "physics"
    visual = package_dir / "visual/native"
    physics.mkdir()
    visual.mkdir(parents=True)
    for origin in files:
        shutil.copy2(origin, visual / origin.name)
    scene.getroot().set("model", "DOSO factory workshop 2.0.0")
    ET.indent(scene, space="  ")
    scene.write(physics / "campus.xml", encoding="utf-8", xml_declaration=True)
    option = scene.getroot().find("option")
    manifest = {
        "schema": "lingtu.sim.world-package.v1",
        "id": "factory_workshop", "version": "2.0.0", "kind": "world",
        "description": "DOSO 120 x 90 m campus with an 80 x 48 m factory, full second floor at 6 m, roof at 12 m, two stairs, retained machinery and exterior grounds. Native MuJoCo baked visuals; robot stair capability is not qualified.",
        "physics": {
            "mjcf": "physics/campus.xml",
            "global_policy": {
                "timestep_s": float(option.get("timestep")),
                "integrator": option.get("integrator").lower(),
                "solver": option.get("solver").lower(),
                "iterations": int(option.get("iterations")),
                "gravity_mps2": [float(v) for v in option.get("gravity").split()],
            },
        },
        "visual": {"binding": "WorldVisual:FactoryWorkshop", "level": "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"},
        "entities": [],
    }
    (package_dir / "world.package.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    record = {
        "world": "factory_workshop@2.0.0", "authoring_revision": "v8-r4-detail-study",
        "source": "artifacts/factory_workshop/v8-r4-detail-study/native/campus.xml",
        "asset_count": len(files), "asset_bytes": sum(f.stat().st_size for f in files),
        "collision_geometry_changed": False, "robot_motion_verified": False,
        "scope": "Native MuJoCo world; no Unreal scene or navigation qualification implied.",
    }
    (package_dir / "INSTALLATION.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--package-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(install_campus(args.source, args.package_dir), indent=2))


if __name__ == "__main__":
    main()
