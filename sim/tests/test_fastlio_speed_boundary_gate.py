import pytest

pytestmark = [pytest.mark.sim]

from __future__ import annotations

import json
from pathlib import Path

from sim.scripts import fastlio_speed_boundary_gate


def _write_fastlio_control_report(
    path: Path,
    *,
    speed_mps: float,
    lateral_mps: float = 0.0,
    angular_z_radps: float = 0.0,
    ok: bool,
    z_error_m: float,
    motion_ok: bool = True,
    yaw_ok: bool = True,
    tuned: bool = False,
    remaining_gaps: list[str] | None = None,
) -> Path:
    config = {
        "imu_static_acc_thresh": 0.0,
        "imu_static_gyro_thresh": 0.0,
        "zupt_min_static_frames": 1000000,
    }
    if tuned:
        config.update(
            {
                "lidar_filter_num": 2,
                "scan_resolution": 0.10,
                "map_resolution": 0.20,
                "near_search_num": 8,
                "ieskf_max_iter": 8,
                "lidar_cov_inv": 500.0,
            }
        )
    payload = {
        "ok": ok,
        "nav_data_source": "fastlio2",
        "commanded_sim_velocity": {
            "source": "fixed",
            "linear_x": speed_mps,
            "linear_y": lateral_mps,
            "angular_z": angular_z_radps,
        },
        "fastlio2_sim_config": config,
        "fastlio2_z_consistency": {
            "checked": True,
            "ok": z_error_m <= 1.0,
            "z_delta_error_m": z_error_m,
            "max_allowed_z_drift_m": 1.0,
        },
        "fastlio2_motion_consistency": {
            "checked": True,
            "ok": motion_ok,
            "sim_moved_m": speed_mps * 20.0,
            "fastlio2_moved_m": speed_mps * 20.0 * (1.0 if motion_ok else 4.0),
        },
        "fastlio2_yaw_consistency": {
            "checked": True,
            "ok": yaw_ok,
            "yaw_delta_error_rad": 0.02 if yaw_ok else 1.8,
            "max_allowed_yaw_drift_rad": 0.5,
        },
        "sim_time_s": 12.0,
        "wall_time_s": 18.0,
        "duration_clock": "sim",
        "remaining_gaps": (
            remaining_gaps
            if remaining_gaps is not None
            else [
                "Fast-LIO odometry Z drift exceeded limit"
                if z_error_m > 1.0
                else "runtime motion defect"
            ]
            if not ok
            else []
        ),
        "runtime_fault_events": (
            []
            if ok
            else [
                {
                    "kind": "z" if z_error_m > 1.0 else "motion",
                    "confirmed": True,
                    "time_alignment": {
                        "time_aligned": True,
                        "first_dt_s": 0.0,
                        "last_dt_s": 0.0,
                    },
                }
            ]
        ),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_fastlio_speed_boundary_gate_reports_green_and_red_speed_boundary(
    tmp_path: Path,
) -> None:
    reports = [
        _write_fastlio_control_report(
            tmp_path / "vx005.json",
            speed_mps=0.05,
            ok=True,
            z_error_m=0.029,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx010.json",
            speed_mps=0.10,
            ok=False,
            z_error_m=1.65,
            motion_ok=False,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx025.json",
            speed_mps=0.25,
            ok=False,
            z_error_m=31.5,
            motion_ok=False,
            yaw_ok=False,
        ),
    ]

    summary = fastlio_speed_boundary_gate.evaluate_reports(reports)

    assert summary["ok"] is False
    assert summary["boundary_characterized"] is True
    assert summary["algorithm_pass"] is False
    assert summary["claim_allowed"] is False
    assert summary["boundary"]["green_speed_mps"] == 0.05
    assert summary["boundary"]["first_red_speed_mps"] == 0.1
    assert summary["boundary"]["red_speeds_mps"] == [0.1, 0.25]
    assert summary["boundary"]["minimal_red_defect"]["blocking_subsystem"] == "slam_localization"


def test_fastlio_speed_boundary_gate_keeps_tuned_control_red_until_threshold_clears(
    tmp_path: Path,
) -> None:
    reports = [
        _write_fastlio_control_report(
            tmp_path / "vx025_baseline.json",
            speed_mps=0.25,
            ok=False,
            z_error_m=31.5,
            motion_ok=False,
            yaw_ok=False,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx025_tuned_a.json",
            speed_mps=0.25,
            ok=False,
            z_error_m=1.1448,
            tuned=True,
        ),
    ]

    summary = fastlio_speed_boundary_gate.evaluate_reports(reports)

    assert summary["boundary_characterized"] is True
    finding = summary["config_findings"][0]
    assert finding["config_label"] == "tuned"
    assert finding["classification"] == "red_defect"
    assert finding["changed_boundary"] is False
    assert finding["z_delta_error_m"] == 1.1448
    assert "scan timing" in summary["recommended_next_step"]


def test_fastlio_speed_boundary_gate_treats_default_fastlio_config_as_baseline(
    tmp_path: Path,
) -> None:
    baseline = _write_fastlio_control_report(
        tmp_path / "vx006_default_config.json",
        speed_mps=0.06,
        ok=False,
        z_error_m=1.49,
    )
    payload = json.loads(baseline.read_text(encoding="utf-8"))
    payload["fastlio2_sim_config"].update(
        {
            "config_path": "artifacts/mujoco_fastlio2_live/fastlio2_mujoco_live.yaml",
            "lidar_filter_num": 4,
            "scan_resolution": 0.15,
            "map_resolution": 0.30,
            "near_search_num": 5,
            "ieskf_max_iter": 5,
            "lidar_cov_inv": 1000.0,
        }
    )
    baseline.write_text(json.dumps(payload), encoding="utf-8")
    tuned = _write_fastlio_control_report(
        tmp_path / "vx010_tuned.json",
        speed_mps=0.10,
        ok=False,
        z_error_m=1.68,
        tuned=True,
    )

    summary = fastlio_speed_boundary_gate.evaluate_reports([baseline, tuned])

    assert summary["boundary_characterized"] is True
    assert summary["boundary"]["baseline_case_count"] == 1
    assert summary["boundary"]["first_red_speed_mps"] == 0.06
    assert summary["config_findings"][0]["speed_mps"] == 0.1


def test_fastlio_speed_boundary_gate_surfaces_non_monotonic_red_boundary(
    tmp_path: Path,
) -> None:
    reports = [
        _write_fastlio_control_report(
            tmp_path / "vx005_pass.json",
            speed_mps=0.05,
            ok=True,
            z_error_m=0.03,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx006_fail.json",
            speed_mps=0.06,
            ok=False,
            z_error_m=1.49,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx015_pass.json",
            speed_mps=0.15,
            ok=True,
            z_error_m=0.5,
        ),
    ]

    summary = fastlio_speed_boundary_gate.evaluate_reports(reports)

    assert summary["boundary"]["state"] == "non_monotonic_or_unstable"
    assert summary["boundary"]["green_speed_mps"] == 0.15
    assert summary["boundary"]["first_red_speed_mps"] == 0.06
    assert summary["boundary"]["minimal_red_defect"]["speed_mps"] == 0.06
    assert summary["boundary"]["red_speeds_mps"] == [0.06]


def test_fastlio_speed_boundary_gate_groups_turning_controls_by_angular_rate(
    tmp_path: Path,
) -> None:
    reports = [
        _write_fastlio_control_report(
            tmp_path / "vx005_wz000_pass.json",
            speed_mps=0.05,
            angular_z_radps=0.0,
            ok=True,
            z_error_m=0.03,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx005_wz025_fail.json",
            speed_mps=0.05,
            angular_z_radps=0.25,
            ok=False,
            z_error_m=1.24,
        ),
        _write_fastlio_control_report(
            tmp_path / "vx010_wz025_fail.json",
            speed_mps=0.10,
            angular_z_radps=0.25,
            ok=False,
            z_error_m=2.1,
            yaw_ok=False,
        ),
    ]

    summary = fastlio_speed_boundary_gate.evaluate_reports(reports)

    turning = summary["turning_boundary"]
    assert turning["turning_case_count"] == 2
    assert turning["turning_failing_count"] == 2
    assert turning["first_red_turning_case"]["speed_mps"] == 0.05
    assert turning["first_red_turning_case"]["angular_z_radps"] == 0.25
    assert turning["first_red_turning_case"]["abs_angular_z_radps"] == 0.25
    straight_group, turning_group = turning["angular_rate_groups"]
    assert straight_group["angular_z_radps"] == 0.0
    assert straight_group["case_count"] == 1
    assert straight_group["passing_count"] == 1
    assert straight_group["minimal_red_defect"] == {}
    assert turning_group["angular_z_radps"] == 0.25
    assert turning_group["case_count"] == 2
    assert turning_group["failing_count"] == 2
    assert turning_group["first_red_speed_mps"] == 0.05
    assert turning_group["red_speeds_mps"] == [0.05, 0.1]
    assert turning_group["minimal_red_defect"]["blocking_subsystem"] == "slam_localization"
    assert turning_group["minimal_red_defect"]["primary_metric"] == "fastlio2_z_consistency"
    assert turning_group["minimal_red_defect"]["z_delta_error_m"] == 1.24
    assert "yaw-rate" in summary["recommended_next_step"]


def test_fastlio_speed_boundary_gate_surfaces_lateral_command_shape(
    tmp_path: Path,
) -> None:
    report = _write_fastlio_control_report(
        tmp_path / "vx010_vy008_wz045_fail.json",
        speed_mps=0.10,
        lateral_mps=0.08,
        angular_z_radps=0.45,
        ok=False,
        z_error_m=2.4,
        motion_ok=False,
        yaw_ok=False,
    )

    summary = fastlio_speed_boundary_gate.evaluate_reports([report])

    case = summary["cases"][0]
    assert case["speed_mps"] == 0.1
    assert case["lateral_y_mps"] == 0.08
    assert case["abs_lateral_y_mps"] == 0.08
    assert case["planar_speed_mps"] == 0.128062
    defect = summary["turning_boundary"]["first_red_turning_slam_case"]
    assert defect["lateral_y_mps"] == 0.08
    assert defect["planar_speed_mps"] == 0.128062


def test_fastlio_speed_boundary_gate_separates_wall_timeout_from_slam_failure(
    tmp_path: Path,
) -> None:
    report = _write_fastlio_control_report(
        tmp_path / "spin_wall_timeout.json",
        speed_mps=0.0,
        angular_z_radps=0.25,
        ok=False,
        z_error_m=0.01,
        motion_ok=True,
        yaw_ok=True,
        remaining_gaps=["gate wall timeout after 90.0s (limit=90.0s)"],
    )

    summary = fastlio_speed_boundary_gate.evaluate_reports([report])

    case = summary["cases"][0]
    assert case["wall_timeout"] is True
    assert case["blocking_subsystem"] == "validation_runtime"
    assert summary["turning_boundary"]["first_red_turning_case"]["primary_metric"] == "gate_wall_timeout"
    assert summary["turning_boundary"]["first_red_turning_case"]["blocking_subsystem"] == "validation_runtime"
    assert summary["turning_boundary"]["first_red_turning_slam_case"] == {}


def test_fastlio_speed_boundary_gate_rejects_missing_fixed_controls(tmp_path: Path) -> None:
    report = _write_fastlio_control_report(
        tmp_path / "nav_cmd.json",
        speed_mps=0.05,
        ok=True,
        z_error_m=0.03,
    )
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["commanded_sim_velocity"]["source"] = "nav_cmd_vel"
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = fastlio_speed_boundary_gate.evaluate_reports([report])

    assert summary["boundary_characterized"] is False
    assert "no fixed-drive Fast-LIO controls provided" in summary["blockers"]


def test_fastlio_speed_scan_plan_preserves_diagnostic_variables() -> None:
    plan = Path("sim/scripts/fastlio_speed_scan_plan.sh")
    text = plan.read_text(encoding="utf-8")

    assert 'git -C "${SCRIPT_DIR}" rev-parse --show-toplevel' in text
    assert 'sim/scripts/mujoco_fastlio2_live_gate.py' in text
    assert '[[ -d "${candidate}/src" ]]' in text
    assert "source_setup_if_present()" in text
    assert "set +u" in text
    assert "source_setup_if_present /opt/ros/humble/setup.bash" in text
    assert 'source_setup_if_present "${ROOT}/install/setup.bash"' in text
    assert 'LINGTU_SPEED_SCAN_PYTHONPATH="src:."' in text
    assert '${PYTHONPATH:-}' in text
    assert 'PYTHONPATH="${LINGTU_SPEED_SCAN_PYTHONPATH}" python3' in text
    assert 'read -r -a SPEED_SCAN_VX_VALUES <<< "${LINGTU_SPEED_SCAN_VX_VALUES:-0.060 0.075 0.090 0.100}"' in text
    assert 'read -r -a SPEED_SCAN_WZ_VALUES <<< "${LINGTU_SPEED_SCAN_WZ_VALUES:-0.000 0.250}"' in text
    assert 'for vx in "${SPEED_SCAN_VX_VALUES[@]}"; do' in text
    assert 'for wz in "${SPEED_SCAN_WZ_VALUES[@]}"; do' in text
    assert 'out="${ROOT}/artifacts/server_sim_closure/diagnosis_matrix/speed_boundary_refined/fixed_vx_${vx}_wz_${wz}"' in text
    assert '--drive-vx "${vx}"' in text
    assert '--drive-wz "${wz}"' in text
    assert "for profile in physical_rolling synthetic_rolling instantaneous" in text
    assert 'out="${ROOT}/artifacts/server_sim_closure/diagnosis_matrix/scan_timing/vx010_${profile}"' in text
    assert '--scan-time-profile "${profile}"' in text
    assert "--fastlio-lidar-filter-num 2" in text
    assert "--fastlio-lidar-cov-inv 500" in text
    assert 'read -r -a COMMAND_SHAPE_VX_VALUES <<< "${LINGTU_COMMAND_SHAPE_VX_VALUES:-0.000 0.100}"' in text
    assert 'read -r -a COMMAND_SHAPE_VY_VALUES <<< "${LINGTU_COMMAND_SHAPE_VY_VALUES:-0.000 0.080}"' in text
    assert 'read -r -a COMMAND_SHAPE_WZ_VALUES <<< "${LINGTU_COMMAND_SHAPE_WZ_VALUES:-0.000 0.250 0.450}"' in text
    assert 'for vy in "${COMMAND_SHAPE_VY_VALUES[@]}"; do' in text
    assert 'command_shape_boundary/fixed_vx_${vx}_vy_${vy}_wz_${wz}' in text
    assert '--drive-vy "${vy}"' in text
    assert 'diagnosis_matrix/command_shape_boundary/*/report.json' in text
    assert "fastlio_speed_boundary_gate.py" in text
    assert "fastlio_controls/*/*/report.json" in text
    assert "fastlio_speed_boundary/refined_report.json" in text
    assert "--strict || true" in text
