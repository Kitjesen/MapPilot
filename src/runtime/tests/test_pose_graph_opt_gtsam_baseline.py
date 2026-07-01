from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
FIXTURE_SCRIPT = ROOT / "tools" / "bench" / "pose_graph_opt_fixture.py"
BASELINE_SCRIPT = ROOT / "tools" / "bench" / "pose_graph_opt_gtsam_baseline.py"


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_gtsam_baseline_runner_generates_optional_cmake_project():
    module = _load_module(BASELINE_SCRIPT, "pose_graph_opt_gtsam_baseline")

    cmake = module.generate_cmake()

    assert "find_package(GTSAM REQUIRED)" in cmake
    assert "add_executable(gtsam_fixture_runner" in cmake
    assert "target_link_libraries(gtsam_fixture_runner PRIVATE gtsam)" in cmake
    assert "GTSAM_LIBRARIES" in cmake


def test_gtsam_baseline_runner_generates_pgo_pose3_factor_graph():
    fixture_module = _load_module(FIXTURE_SCRIPT, "pose_graph_opt_fixture")
    module = _load_module(BASELINE_SCRIPT, "pose_graph_opt_gtsam_baseline")
    fixture = fixture_module.generate_fixture("pgo_loop", 4)

    source = module.generate_runner_source(fixture)

    assert "gtsam::PriorFactor<gtsam::Pose3>" in source
    assert "gtsam::BetweenFactor<gtsam::Pose3>" in source
    assert "gtsam::noiseModel::Gaussian::Information" in source
    assert "gtsam::LevenbergMarquardtOptimizer" in source
    assert "raw_graph.error(initial)" in source
    assert "raw_graph.error(result)" in source
    assert "strongAnchorInformation" in source
    assert "appendResidualValues" in source
    assert "final_residual_rms" in source
    assert "final_residual_max" in source
    assert "accepted_steps" in source
    assert "rejected_steps" in source
    assert '<<"case":"pgo_loop",' not in source
    assert "case" in source
    assert "pgo_loop" in source
    assert "optimized_poses" in source
    assert "params.setlambdaInitial" in source
    assert "optimizer.iterations()" in source


def test_gtsam_baseline_runner_keeps_hba_between_only():
    fixture_module = _load_module(FIXTURE_SCRIPT, "pose_graph_opt_fixture_hba")
    module = _load_module(BASELINE_SCRIPT, "pose_graph_opt_gtsam_baseline_hba")
    fixture = fixture_module.generate_fixture("hba_full_info", 5)

    source = module.generate_runner_source(fixture)

    assert "raw_graph.add(gtsam::PriorFactor" not in source
    assert "opt_graph.add(gtsam::PriorFactor<gtsam::Pose3>" in source
    assert "raw_graph.add(gtsam::BetweenFactor<gtsam::Pose3>" in source
    assert "opt_graph.add(gtsam::BetweenFactor<gtsam::Pose3>" in source
    assert "hba_full_info" in source


def test_gtsam_baseline_suite_aggregates_fixture_results(tmp_path, monkeypatch):
    fixture_module = _load_module(FIXTURE_SCRIPT, "pose_graph_opt_fixture_suite_fixture")
    module = _load_module(BASELINE_SCRIPT, "pose_graph_opt_gtsam_baseline_suite")
    first = fixture_module.generate_fixture("pgo_loop", 4)
    second = fixture_module.generate_fixture("hba_full_info", 4)
    fixture_module.write_json(tmp_path / "pgo_loop_4.json", first)
    fixture_module.write_json(tmp_path / "hba_full_info_4.json", second)

    monkeypatch.setattr(module.shutil, "which", lambda executable: executable)

    def fake_build_and_run(fixture_data, build_root, *, cmake, config):
        result = {
            "benchmark": "legacy_cpp_gtsam_fixture",
            "unit": "milliseconds",
            "cases": [
                {
                    "case": fixture_data["case"],
                    "poses": len(fixture_data["poses"]),
                    "factors": len(fixture_data["priors"]) + len(fixture_data["betweens"]),
                    "status": 0,
                    "converged": True,
                    "written": len(fixture_data["poses"]),
                    "elapsed_ms": 1.0,
                    "final_cost": 0.0,
                }
            ],
            "metadata": {"build_dir": str(build_root)},
        }
        return module.attach_fixture_identity(result, fixture_data)

    monkeypatch.setattr(module, "_build_and_run", fake_build_and_run)

    result = module.run_gtsam_baseline_suite(
        tmp_path,
        build_dir=tmp_path / "build",
    )

    assert result["benchmark"] == "legacy_cpp_gtsam_fixture_suite"
    assert result["metadata"]["fixture_count"] == 2
    assert [case["case"] for case in result["cases"]] == ["hba_full_info", "pgo_loop"]
    assert all("fixture_hash" in case for case in result["cases"])
    assert all("fixture_path" in case for case in result["cases"])
    assert len(result["fixtures"]) == 2
