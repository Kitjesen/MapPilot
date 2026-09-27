from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
RUNNER = HERE / "compare_maps.py"


class CompareMapsTest(unittest.TestCase):
    def test_three_groups_use_same_source_config_and_plan(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            (source / "patches").mkdir(parents=True)
            for name in ("poses.txt", "scan_origin.txt", "patch_bundle.manifest"):
                (source / name).write_text(name, encoding="utf-8")
            (source / "patches" / "scan_000000.pcd").write_text("patch", encoding="utf-8")
            (source / "map.pcd.preclean").write_text("original", encoding="utf-8")
            (source / "map.pcd").write_text("cleaned", encoding="utf-8")

            config = {
                "planner_options": {"robot_radius": 0.2},
                "rois": [
                    {"name": "body", "role": "body_clearance", "min": [0, 0, 0], "max": [1, 1, 1]},
                    {"name": "ground", "role": "ground_support", "min": [0, 0, -1], "max": [1, 1, 0]},
                ],
                "labels": [
                    {"name": "wall", "kind": "wall", "expected": "occupied", "point": [0, 0, 0]},
                    {"name": "post", "kind": "post", "expected": "occupied", "point": [0, 0, 0]},
                    {"name": "ghost", "kind": "residue", "expected": "not_occupied", "point": [0, 0, 0]},
                ],
                "plans": [{
                    "name": "fixed",
                    "start": [1, 2, 3],
                    "goal": [4, 5, 6],
                    "expected_ok": True,
                }],
            }
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")

            prune = root / "prune.py"
            prune.write_text(
                "import json,sys\nprint(json.dumps({'ok':True,'map_dir':sys.argv[2]}))\n",
                encoding="utf-8",
            )
            tool = root / "tool.py"
            tool.write_text(
                "import json,pathlib,sys\n"
                "if sys.argv[1]=='replay':\n"
                " pathlib.Path(sys.argv[3]).write_text('octomap')\n"
                " print(json.dumps({'valid_endpoints':3,'retained_endpoints':2,'dropped_endpoints':1}))\n"
                "else:\n"
                " print(json.dumps({'rois':[],'labels':[{'matches':True},{'matches':True},{'matches':True}]}))\n",
                encoding="utf-8",
            )
            planner = root / "planner.py"
            planner.write_text(
                "import json,sys\nrequest=json.load(sys.stdin)\nprint(json.dumps({'ok':True,'request':request}))\n",
                encoding="utf-8",
            )

            work = root / "work"
            command = [
                sys.executable, str(RUNNER), "--source", str(source), "--work-dir", str(work),
                "--config", str(config_path), "--tool", str(tool), "--old-prune", str(prune),
                "--new-prune", str(prune), "--old-replay", str(tool), "--new-replay", str(tool),
                "--planner", str(planner), "--code-sha", "abc123", "--content-epoch", "epoch-7",
            ]
            completed = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stderr or completed.stdout)
            report = json.loads((work / "comparison.json").read_text(encoding="utf-8"))
            self.assertTrue(report["acceptance_ready"])
            self.assertEqual(len(report["groups"]), 3)
            for group in report["groups"]:
                staged = Path(group["candidate_dir"])
                self.assertEqual((staged / "map.pcd").read_text(encoding="utf-8"), "original")
                request = group["plans"][0]["stdout_json"]["request"]
                self.assertEqual(request["start"], [1, 2, 3])
                self.assertEqual(request["goal"], [4, 5, 6])
                self.assertEqual(request["options"], {"robot_radius": 0.2})

    def test_incomplete_source_is_reported_without_staging_candidates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            (source / "map.pcd").write_text("map", encoding="utf-8")
            config = root / "config.json"
            config.write_text('{"rois":[],"labels":[],"plans":[]}', encoding="utf-8")
            missing = root / "missing"
            work = root / "work"
            command = [
                sys.executable, str(RUNNER), "--source", str(source), "--work-dir", str(work),
                "--config", str(config), "--tool", str(missing), "--old-prune", str(missing),
                "--new-prune", str(missing), "--old-replay", str(missing), "--new-replay", str(missing),
                "--planner", str(missing), "--code-sha", "abc123", "--content-epoch", "epoch-7",
            ]
            completed = subprocess.run(command, capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 2)
            report = json.loads((work / "comparison.json").read_text(encoding="utf-8"))
            self.assertEqual(report["reason"], "source_snapshot_incomplete")
            self.assertFalse(report["groups"])


if __name__ == "__main__":
    unittest.main()
