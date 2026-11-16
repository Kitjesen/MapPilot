import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def find_headless() -> Path:
    configured = os.environ.get("LINGTU_HEADLESS_EXE")
    default = ("build/artifact-check/Release/octoplanner3d_headless.exe" if os.name == "nt"
               else "build/nav_endpoint/octoplanner3d_runtime/octoplanner3d_headless")
    candidates = [Path(configured)] if configured else [ROOT / default]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise unittest.SkipTest("build octoplanner3d_headless or set LINGTU_HEADLESS_EXE")


def invoke(headless: Path, request: dict) -> tuple[int, dict]:
    completed = subprocess.run(
        [str(headless)], input=json.dumps(request), text=True,
        capture_output=True, timeout=120, check=False,
    )
    return completed.returncode, json.loads(completed.stdout)


class HeadlessBenchmarkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.headless = find_headless()

    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def test_invalid_samples(self) -> None:
        for samples in [0, -1, 1.5, 2147483648]:
            with self.subTest(samples=samples):
                code, result = invoke(self.headless, {
                    "map_path": str(self.directory / "missing.ot"),
                    "start": [0, 0, 0], "goal": [1, 0, 0],
                    "benchmark": {"samples": samples,
                                  "cache_directory": str(self.directory / "cache")},
                })
                self.assertEqual(code, 1)
                self.assertEqual(result["error"], "benchmark samples must be a positive integer")

    def test_warm_query_matches_normal_run(self) -> None:
        check_warm_query(self.headless, self.directory)


def check_warm_query(headless: Path, tmp_path: Path) -> None:
    suffix = headless.suffix
    maker = headless.with_name("octoplanner3d_make_test_octomap" + suffix)
    assert maker.is_file(), "build octoplanner3d_make_test_octomap for the fixture"
    map_path = tmp_path / "map.bt"
    subprocess.run(
        [str(maker), "--output", str(map_path)],
        capture_output=True, text=True, check=True, timeout=30,
    )
    request = {
        "map_path": str(map_path), "start": [-4.1, -.1, .45], "goal": [-3.1, -.1, .45],
        "options": {"robot_radius": .1, "support_height_m": .35,
                    "search_resolution_m": .2,
                    "support_height_tolerance_m": .1, "max_slope": .7},
    }
    code, normal = invoke(headless, request)
    assert code == 0, normal
    assert normal["diagnostics"]["used_rust_graph"]
    assert "benchmark" not in normal
    request["benchmark"] = {"samples": 3, "cache_directory": str(tmp_path / "cache")}
    code, measured = invoke(headless, request)
    assert code == 0, measured
    assert measured["path"] == normal["path"]
    assert measured["reached_goal"] == normal["reached_goal"]
    assert measured["diagnostics"]["used_rust_graph"] == normal["diagnostics"]["used_rust_graph"]
    benchmark = measured["benchmark"]
    assert benchmark["same_result"]
    assert benchmark["samples"] == len(benchmark["query_ms"]) == 3
    assert benchmark["prepare_count"] == benchmark["map_load_count"] == 1
    assert benchmark["cache_load_count"] == 0
    assert benchmark["restored_prepare_count"] == benchmark["restored_cache_load_count"] == 1
    assert 0 <= benchmark["warm_query_median_ms"] <= benchmark["warm_query_p95_ms"]

    request["benchmark"]["cache_directory"] = str(tmp_path / "failed-cache")
    request["goal"] = [100, 100, 100]
    code, failed = invoke(headless, request)
    assert code == 2
    assert not failed["ok"]
    assert failed["error"]
    assert failed["benchmark"]["samples"] == 3
    assert failed["benchmark"]["same_result"]


if __name__ == "__main__":
    unittest.main()
