#!/usr/bin/env python3
"""Run the paper-aligned text-to-native-preview water-bottle slice."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_slice(
    graph: Path,
    instruction: str,
    *,
    robot_position: dict[str, float] | None = None,
    standoff_m: float = 0.8,
    map_path: Path | None = None,
    planner: Path | None = None,
    verifier: Path | None = None,
    start: list[float] | None = None,
    options: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Execute parse -> instance query -> optional native preview."""
    query_module = _load("lingtu_water_query", ROOT / "tools" / "datasets" / "3dssg_goal_query.py")
    query = query_module.resolve(
        graph,
        instruction,
        robot_position=robot_position,
        standoff_m=standoff_m,
    )
    report: dict[str, Any] = {
        "schema_version": "lingtu.water_bottle_slice.v1",
        "paper_method": "OVI-MAP: geometry-first instance query with semantic evidence",
        "instruction": instruction,
        "query": query,
        "motion_authorization": False,
    }
    if query["semantic_match"]["status"] != "candidate":
        report["status"] = query["semantic_match"]["status"]
        report["preview"] = None
        return report
    if map_path is None or planner is None:
        report["status"] = "candidate_generated"
        report["preview"] = None
        return report

    preview_module = _load("lingtu_semantic_goal_preview", ROOT / "tools" / "bench" / "semantic_goal_preview.py")
    query_path = graph.parent / f"{graph.stem}_water_query.json"
    query_path.write_text(json.dumps(query, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["preview"] = preview_module.preview(
        query_path,
        map_path,
        planner,
        start or [0.0, 0.0, 0.3],
        options or {},
        verifier=verifier,
    )
    report["status"] = report["preview"]["status"]
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("graph", type=Path)
    parser.add_argument("--instruction", default="帮我找一瓶水")
    parser.add_argument("--robot-x", type=float, default=0.0)
    parser.add_argument("--robot-y", type=float, default=0.0)
    parser.add_argument("--robot-z", type=float, default=0.3)
    parser.add_argument("--standoff-m", type=float, default=0.8)
    parser.add_argument("--map", dest="map_path", type=Path)
    parser.add_argument("--planner", type=Path)
    parser.add_argument("--verifier", type=Path)
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--options", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    options = json.loads(args.options.read_text(encoding="utf-8")) if args.options else {}
    report = run_slice(
        args.graph,
        args.instruction,
        robot_position={"x": args.robot_x, "y": args.robot_y, "z": args.robot_z},
        standoff_m=args.standoff_m,
        map_path=args.map_path,
        planner=args.planner,
        verifier=args.verifier,
        start=args.start,
        options=options,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "motion_authorization": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
