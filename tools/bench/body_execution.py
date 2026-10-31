"""Repeat the four saved body-route fixtures through the native execution probe.

These frozen geometry fixtures do not resolve a robot Product. The body branch
uses native default control settings; legacy retains its historical tuning.
Comparing the two is not a geometry-only A/B test. Use body_physics for models
sharing one resolved Thunder sim configuration and actual policy physics.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--reference", type=Path,
                        default=Path("build/native-body-routes-20261022"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", default=["detour", "align", "corner", "too_narrow"])
    parser.add_argument("--modes", nargs="+", choices=["legacy", "body"], default=["legacy", "body"])
    parser.add_argument("--seconds", type=float, default=45)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for case in args.cases:
        scene = json.loads((args.reference / "runs" / case / "scene.json").read_text())
        route = args.out / f"{case}-request.txt"
        route.write_text("2\n" + "\n".join(" ".join(map(str, scene[key]))
                                         for key in ("start", "goal")) + "\n")
        for mode in args.modes:
            folder = args.out / case / mode
            folder.mkdir(parents=True, exist_ok=True)
            if (folder / "summary.json").exists():
                raise FileExistsError(f"Keep the previous result; choose a fresh output: {folder}")
            command = [str(args.native.resolve()), "--map",
                       str((args.reference / "runs" / case / "map.ot").resolve()),
                       "--body", str((args.reference / "model.txt").resolve()),
                       "--route", str(route.resolve()), "--out", str(folder.resolve()),
                       "--seconds", str(args.seconds)]
            if mode == "legacy":
                command.append("--legacy")
            (folder / "invocation.json").write_text(json.dumps(command, indent=2))
            with (folder / "native.log").open("w", encoding="utf-8") as stream:
                result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                        check=False, timeout=max(180, args.seconds * 5))
            if result.returncode:
                raise RuntimeError(f"Native probe failed ({result.returncode}): {folder}")
            print(case, mode, (folder / "summary.json").read_text().strip(), flush=True)


if __name__ == "__main__":
    main()
