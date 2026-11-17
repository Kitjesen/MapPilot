"""Probe the nominal-height cross-slope boundary without changing product parameters."""
import json
import math
import runpy
from pathlib import Path

import numpy as np

planner = runpy.run_path("build/terrain-exploration-20261117/explore.py")
generate = planner["generate"]
run = planner["run"]
out = Path("result/terrain-exploration-20261117/data/slope-boundary-fine.json")
rows = []

for angle in (46, 48, 50, 52, 53, 54, 55, 56):
    case = {
        "id": f"cross_{angle}_boundary",
        "title": f"横坡 {angle}° 边界",
        "fn": lambda x, y, a=angle: np.tan(np.radians(a)) * y + np.zeros_like(x),
        "length": 8,
        "width": 2.4,
        "feature": "",
        "expected": "probe",
        "diagonal": False,
    }
    scene = generate(case)
    base = scene["requests"][0]
    for direction, start, goal in (
        ("forward", base["start"], base["goal"]),
        ("reverse", base["goal"], base["start"]),
    ):
        for label, offset in (("nominal", 0.0), ("raised05", 0.05)):
            request = {
                "id": f"{label}_{direction}",
                "start": [start[0], start[1], start[2] + offset],
                "goal": [goal[0], goal[1], goal[2] + offset],
                "start_yaw": math.atan2(goal[1] - start[1], goal[0] - start[0]),
                "expected": "probe",
            }
            result = run(scene, request, 45)
            diagnostics = result.get("result", {}).get("diagnostics", {})
            rows.append(
                {
                    "angle_deg": angle,
                    "request": request["id"],
                    "status": result["status"],
                    "ok": result.get("result", {}).get("ok"),
                    "rust_failure_kind": diagnostics.get("rust_failure_kind"),
                    "rust_failure": diagnostics.get("rust_failure"),
                    "path_points": len(result.get("result", {}).get("path", [])),
                    "elapsed_s": result.get("elapsed_s"),
                }
            )

out.write_text(json.dumps(rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(out)
