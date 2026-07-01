from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pose_graph_opt_fixture.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pose_graph_opt_fixture", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pgo_loop_fixture_matches_neutral_schema():
    module = _load_module()

    fixture = module.generate_fixture("pgo_loop", 8)

    assert fixture["schema"] == module.SCHEMA
    assert fixture["schema_version"] == 1
    assert fixture["case"] == "pgo_loop"
    assert fixture["pose_format"] == "t_xyz_q_wxyz"
    assert fixture["tangent_order"] == ["rx", "ry", "rz", "tx", "ty", "tz"]
    assert fixture["information_upper_order"] == "row_major_upper_6x6"
    assert fixture["information_packing"] == "upper_triangle_row_major_6x6"
    assert fixture["config"]["fixed_pose_index"] == 0
    assert len(fixture["poses"]) == 8
    assert len(fixture["priors"]) == 1
    assert fixture["priors"][0]["index"] == 0
    assert len(fixture["priors"][0]["information_upper"]) == 21
    assert len(fixture["betweens"]) == 8
    assert fixture["betweens"][-1]["from_index"] == 0
    assert fixture["betweens"][-1]["to_index"] == 7
    assert fixture["expected"]["pose_count"] == 8
    assert fixture["expected"]["factor_count"] == 9
    assert fixture["baseline_tolerances"]["final_cost_delta_abs_max"] == 1e-6
    module.validate_fixture(fixture)


def test_hba_full_info_fixture_uses_between_only_full_information():
    module = _load_module()

    fixture = module.generate_fixture("hba_full_info", 8)

    assert fixture["case"] == "hba_full_info"
    assert fixture["priors"] == []
    assert len(fixture["betweens"]) > len(fixture["poses"])
    upper = fixture["betweens"][0]["information_upper"]
    assert len(upper) == 21
    assert 0.8 in upper
    assert -0.6 in upper
    module.validate_fixture(fixture)


def test_fixture_writer_roundtrips_json(tmp_path):
    module = _load_module()
    path = tmp_path / "pgo_loop_4.json"
    fixture = module.generate_fixture("pgo_loop", 4)

    module.write_json(path, fixture)

    assert json.loads(path.read_text(encoding="utf-8")) == fixture


def test_run_fixture_suite_aggregates_case_results(tmp_path, monkeypatch):
    module = _load_module()
    first = module.generate_fixture("pgo_loop", 4)
    second = module.generate_fixture("hba_full_info", 4)
    module.write_json(tmp_path / "pgo_loop_4.json", first)
    module.write_json(tmp_path / "hba_full_info_4.json", second)

    def fake_run_fixture(fixture_data, *, release=True):
        identity = module.fixture_identity(fixture_data)
        return {
            "benchmark": "lingtu_pose_graph_opt_fixture",
            "fixture_hash": identity["hash"],
            "fixture_identity": identity,
            "unit": "milliseconds",
            "cases": [
                {
                    "case": fixture_data["case"],
                    "poses": len(fixture_data["poses"]),
                    "factors": len(fixture_data["priors"]) + len(fixture_data["betweens"]),
                    "fixture_hash": identity["hash"],
                    "fixture_identity": identity,
                    "status": 0,
                }
            ],
        }

    monkeypatch.setattr(module, "run_fixture", fake_run_fixture)

    result = module.run_fixture_suite(tmp_path)

    assert result["benchmark"] == "lingtu_pose_graph_opt_fixture_suite"
    assert result["metadata"]["fixture_count"] == 2
    assert [case["case"] for case in result["cases"]] == ["hba_full_info", "pgo_loop"]
    assert all("fixture_hash" in case for case in result["cases"])
    assert all("fixture_path" in case for case in result["cases"])
    assert len(result["fixtures"]) == 2


def test_fixture_identity_hash_changes_when_graph_changes():
    module = _load_module()
    fixture = module.generate_fixture("pgo_loop", 4)
    changed = json.loads(json.dumps(fixture))
    changed["betweens"][0]["information_upper"][0] += 1.0

    identity = module.fixture_identity(fixture)

    assert identity["case"] == "pgo_loop"
    assert identity["poses"] == 4
    assert identity["factors"] == 5
    assert identity["hash"] == module.fixture_hash(fixture)
    assert module.fixture_hash(fixture) != module.fixture_hash(changed)


def test_fixture_residual_stats_are_zero_for_consistent_prior_and_between():
    module = _load_module()
    odom = module.rpy_pose(1.0, 0.2, -0.1, 0.01, -0.02, 0.03)
    poses = [module.identity_pose(), odom]
    fixture = {
        "poses": poses,
        "priors": [
            {
                "index": 0,
                "pose": module.identity_pose(),
                "information_upper": module.diagonal_information_upper([1.0] * 6),
            }
        ],
        "betweens": [
            {
                "from_index": 0,
                "to_index": 1,
                "pose_from_to": odom,
                "information_upper": module.diagonal_information_upper([1.0] * 6),
            }
        ],
    }

    rms, max_abs = module.factor_residual_stats(fixture, poses)

    assert rms == pytest.approx(0.0, abs=1e-12)
    assert max_abs == pytest.approx(0.0, abs=1e-12)


def test_fixture_validation_rejects_bad_information_size():
    module = _load_module()
    fixture = module.generate_fixture("pgo_loop", 4)
    fixture["betweens"][0]["information_upper"] = [1.0]

    with pytest.raises(ValueError, match="information_upper"):
        module.validate_fixture(fixture)
