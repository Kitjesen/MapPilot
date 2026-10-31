"""Run the bounded FactoryPark editor art pass, without physics or a build."""

import argparse
import json
import os
import subprocess
from datetime import datetime

from sim.runtime.process_owner import ProcessTreeOwner
from sim.tools.worlds.factory_park_hf.lookdev import ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--lighting-only", action="store_true")
    args = parser.parse_args()
    run = ROOT / "build/factory-park-lookdev" / datetime.now().strftime("%Y%m%d-%H%M%S")
    run.mkdir(parents=True, exist_ok=False)
    project = ROOT / "sim/runtime/visual/RobotSimUE"
    script = project / "Scripts/refine_factory_park.py"
    argv = [
        "D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor.exe",
        str(project / "RobotSimUE.uproject"),
        "-unattended",
        "-NoSound",
        "-NoCompile",
        "-NoSplash",
        "-RenderOffScreen",
        "-ResX=1920",
        "-ResY=1080",
        "-ForceRes",
        f"-ExecutePythonScript={script}",
        "-DDC=InstalledNoZenLocalFallback",
        f"-LocalDataCachePath={ROOT / 'build/unreal-ddc'}",
        "-DisablePlugins=ModelContextProtocol,AllToolsets,Terminal,Tripo3DUEBridge",
        f"-abslog={run / 'Unreal.log'}",
    ]
    env = dict(
        os.environ,
        UE_SKIP_UBT_SDK_SETUP="1",
        LINGTU_FACTORY_LOOKDEV_RUN=str(run),
        LINGTU_FACTORY_LOOKDEV_APPLY="1" if args.apply or args.lighting_only else "0",
        LINGTU_FACTORY_LOOKDEV_LIGHTING_ONLY="1" if args.lighting_only else "0",
    )
    (run / "launch.json").write_text(
        json.dumps(
            {
                "argv": argv,
                "apply": args.apply or args.lighting_only,
                "lighting_only": args.lighting_only,
                "physics": False,
            },
            indent=2,
        )
    )
    print(f"RUN_DIR={run}", flush=True)
    owner = ProcessTreeOwner()
    try:
        with (run / "console.log").open("wb") as stream:
            process = subprocess.Popen(
                argv,
                cwd=ROOT,
                env=env,
                stdout=stream,
                stderr=stream,
                **owner.popen_options(creationflags=subprocess.CREATE_NO_WINDOW),
            )
            owner.attach(process)
            code = process.wait(timeout=600)
            owner.close_after_exit()
        passed = code == 0 and (run / "success.json").exists() and not (run / "error.txt").exists()
        print(f"EXIT={code} PASSED={passed}", flush=True)
        return 0 if passed else 1
    finally:
        owner.close()


if __name__ == "__main__":
    raise SystemExit(main())
