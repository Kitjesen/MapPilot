from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import hashlib
import json
from pathlib import Path


def _write_active_same_source_tomogram(map_root: Path) -> Path:
    active_dir = map_root / "active"
    active_dir.mkdir(parents=True)
    map_path = active_dir / "map.pcd"
    tomogram_path = active_dir / "tomogram.pickle"
    map_path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 1",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 1",
                "DATA ascii",
                "0.0 0.0 0.0",
            ]
        )
        + "\n",
        encoding="ascii",
    )
    tomogram_path.write_bytes(b"product-field-active-tomogram")
    map_sha = hashlib.sha256(map_path.read_bytes()).hexdigest()
    tomogram_sha = hashlib.sha256(tomogram_path.read_bytes()).hexdigest()
    (active_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "source_profile": "real_s100p",
                "data_source": "real_s100p",
                "slam_source": "fastlio2",
                "localization_source": "fastlio2",
                "mapping_source": "fastlio2",
                "frame_id": "map",
                "created_at": "2026-05-25T00:00:00Z",
                "artifacts": {
                    "map_pcd": {
                        "path": "map.pcd",
                        "sha256": map_sha,
                        "source_profile": "real_s100p",
                        "data_source": "real_s100p",
                        "slam_source": "fastlio2",
                        "frame_id": "map",
                        "point_count": 1,
                    },
                    "tomogram": {
                        "path": "tomogram.pickle",
                        "sha256": tomogram_sha,
                        "source_map_sha256": map_sha,
                        "source_profile": "real_s100p",
                        "data_source": "real_s100p",
                        "frame_id": "map",
                        "shape": [1, 1, 1],
                    },
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return active_dir


def _gateway_acceptance(*, ok: bool = True, mode: str = "field") -> dict:
    blockers = [] if ok else ["field acceptance requires passing real-runtime-evidence"]
    return {
        "ok": ok,
        "mode": mode,
        "blockers": blockers,
        "advisories": [],
        "checks": {
            "gateway_contract": {"ok": True},
            "runtime_mode": {
                "ok": True,
                "command_sink": "hardware_driver_after_cmd_vel_mux",
            },
            "module_first_dataflow": {
                "ok": True,
                "arbitrary_publish_supported": False,
                "missing_command_interfaces": [],
                "unexpected_command_interfaces": [],
            },
            "frontier_preview": {
                "ok": True,
                "required": True,
                "read_only": True,
                "live": True,
                "candidate_source": "traversable_frontier",
                "candidate": {
                    "source": "traversable_frontier",
                    "centroid_3d": [1.0, 2.0, 0.25],
                    "reachable_score": 0.82,
                    "support_type": "flat",
                    "reasons": [],
                    "preview": True,
                    "command_published": False,
                },
                "command_published": False,
                "blockers": [],
            },
            "stage_evidence": {
                "ok": True,
                "required": True,
                "required_stages": [
                    "slam_or_relayed_localization_map",
                    "traversable_frontier_preview",
                    "global_planning",
                    "local_planning_and_following",
                    "command_boundary",
                ],
                "live_stages": [
                    "slam_or_relayed_localization_map",
                    "traversable_frontier_preview",
                    "global_planning",
                    "local_planning_and_following",
                    "command_boundary",
                ],
                "missing_stages": [],
                "not_live_stages": [],
                "non_observable_stages": [],
                "missing_tokens": {},
            },
            "readiness": {"ok": True},
            "localization": {"ok": True},
            "navigation": {"ok": True, "can_send_goal": True},
            "routecheck_latest": {
                "ok": True,
                "non_motion": True,
                "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
            },
            "real_runtime_evidence": {
                "ok": True,
                "data_flow_ok": True,
                "cmd_vel_sent_to_hardware": True,
                "report_age_s": 12.0,
                "runtime_contract": "real_s100p",
            },
        },
    }


def _algorithm_gate(
    *,
    ok: bool = True,
    missing_or_failed: list[str] | None = None,
    product_profiles: dict | None = None,
) -> dict:
    missing = missing_or_failed or []
    blockers = [] if ok and not missing else ["algorithm validation claim_allowed is not true"]
    return {
        "ok": ok and not missing,
        "read_only": True,
        "ros2_topic_required": False,
        "publishes": [],
        "claim_allowed": ok and not missing,
        "missing_or_failed": missing,
        "summary_path": "artifacts/server_sim_closure/summary_dimos_benchmark_24h.json",
        "report_age_s": 42.0,
        "max_age_s": 86400.0,
        "reason": None if ok and not missing else "algorithm_benchmark_not_passing",
        "blockers": blockers,
        "claim_boundary": {"product_claim": "algorithm gate"},
        "blocking_categories": {},
        "source": "server_sim_closure",
        "preset": "dimos_benchmark",
        "active_product_profile": "inspection_mvp",
        "strict_benchmark_profile": "dimos_benchmark",
        "product_profiles": product_profiles or {},
    }


def _runtime_switch_plan(*, ok: bool = True, dry_run: bool = True) -> dict:
    return {
        "ok": ok,
        "read_only": True,
        "dry_run": dry_run,
        "motion": False,
        "publishes": [],
        "lifecycle": "dry_run_preflight",
        "from": {
            "profile": "sim_mujoco_live",
            "endpoint": "mujoco_live",
            "data_source": "mujoco_fastlio2_live",
            "runtime_contract": "mujoco_fastlio2_live",
            "command_sink": "mujoco_velocity_adapter",
            "simulation_only": True,
        },
        "to": {
            "profile": "explore",
            "endpoint": "real_s100p",
            "data_source": "real_s100p",
            "runtime_contract": "real_s100p",
            "command_sink": "hardware_driver_after_cmd_vel_mux",
            "simulation_only": False,
        },
        "changed": ["command_sink", "data_source", "endpoint"],
        "blockers": [] if ok and dry_run else ["runtime switch preflight failed"],
    }


def test_product_field_check_passes_with_gateway_and_map_evidence():
    from core.product_field_check import build_product_field_check

    payload = build_product_field_check(
        _gateway_acceptance(),
        map_gate={
            "ok": True,
            "map_dir": "maps/active",
            "artifacts": {
                "tomogram": {"exists": True, "sha256_ok": True},
                "occupancy_grid": {"exists": True, "sha256_ok": True},
            },
            "blockers": [],
        },
        algorithm_gate=_algorithm_gate(),
        switch_plan=_runtime_switch_plan(),
    )

    assert payload["ok"] is True
    assert payload["summary"] == "PASS"
    assert payload["map"] == {
        "active": "maps/active",
        "provenance": "PASS",
        "tomogram": "PASS",
        "occupancy": "PASS",
    }
    assert payload["runtime"]["command_boundary"] == "PASS"
    assert payload["runtime"]["frontier_preview"] == "PASS"
    assert payload["runtime"]["runtime_switch"] == "PASS"
    assert payload["runtime"]["stages"] == "PASS"
    assert payload["stage_evidence"]["live_stages"] == [
        "slam_or_relayed_localization_map",
        "traversable_frontier_preview",
        "global_planning",
        "local_planning_and_following",
        "command_boundary",
    ]
    assert payload["frontier_preview"]["status"] == "PASS"
    assert payload["frontier_preview"]["candidate_source"] == "traversable_frontier"
    assert payload["frontier_preview"]["command_published"] is False
    assert payload["navigation"]["route_preview"] == "PASS"
    assert payload["navigation"]["route_preview_non_motion"] is True
    assert payload["navigation"]["route_preview_published"] == {
        "goal_pose": 0,
        "cmd_vel": 0,
        "stop_cmd": 0,
    }
    assert payload["evidence"]["real_s100p"] == "PASS"
    assert payload["algorithm"]["strict_benchmark"]["status"] == "PASS"
    assert payload["algorithm"]["strict_benchmark"]["ros2_topic_required"] is False
    assert payload["algorithm"]["strict_benchmark"]["publishes"] == []
    assert payload["algorithm"]["active_product_profile"] == "inspection_mvp"
    assert payload["runtime_switch"]["status"] == "PASS"
    assert payload["runtime_switch"]["read_only"] is True
    assert payload["runtime_switch"]["dry_run"] is True
    assert payload["runtime_switch"]["motion"] is False
    assert payload["runtime_switch"]["publishes"] == []
    assert payload["runtime_switch"]["from"]["endpoint"] == "mujoco_live"
    assert payload["runtime_switch"]["to"]["endpoint"] == "real_s100p"


def test_product_field_check_rejects_stateful_runtime_switch_preflight():
    from core.product_field_check import build_product_field_check

    switch_plan = _runtime_switch_plan(dry_run=False)

    payload = build_product_field_check(
        _gateway_acceptance(),
        algorithm_gate=_algorithm_gate(),
        switch_plan=switch_plan,
    )

    assert payload["ok"] is False
    assert payload["runtime"]["runtime_switch"] == "FAIL"
    assert payload["runtime_switch"]["status"] == "FAIL"
    assert "runtime switch preflight is not dry-run/read-only" in payload["blockers"]


def test_collect_product_field_check_defaults_to_active_map_provenance(
    monkeypatch,
    tmp_path,
):
    import core.gateway_runtime_acceptance as acceptance_mod
    from core.product_field_check import collect_product_field_check

    active_dir = _write_active_same_source_tomogram(tmp_path / "maps")
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path / "maps"))
    monkeypatch.setattr(
        acceptance_mod,
        "collect_gateway_runtime_acceptance",
        lambda **_: _gateway_acceptance(mode="non_motion"),
    )

    payload = collect_product_field_check(
        gateway_url="http://robot:5050",
        timeout_sec=1.0,
        mode="non_motion",
        map_dir=None,
        require_tomogram=True,
    )

    assert payload["map"]["active"] == str(active_dir)
    assert payload["map"]["provenance"] == "PASS"
    assert payload["map"]["tomogram"] == "PASS"
    assert payload["evidence"]["map"]["ok"] is True
    assert payload["evidence"]["map"]["artifacts"]["tomogram"]["sha256_ok"] is True
    assert "map provenance not checked" not in "\n".join(payload["advisories"])


def test_product_field_check_fails_field_mode_without_route_or_real_evidence():
    from core.product_field_check import build_product_field_check

    acceptance = _gateway_acceptance()
    acceptance["checks"]["routecheck_latest"]["ok"] = False
    acceptance["checks"]["real_runtime_evidence"]["ok"] = False

    payload = build_product_field_check(acceptance, algorithm_gate=_algorithm_gate())

    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert "route preview is not passing or unavailable" in payload["blockers"]
    assert "real S100P field evidence is not passing" in payload["blockers"]
    assert payload["map"]["provenance"] == "UNCHECKED"
    assert "map provenance not checked" in "\n".join(payload["advisories"])


def test_product_field_check_fails_field_command_boundary_on_non_hardware_sink():
    from core.product_field_check import build_product_field_check

    acceptance = _gateway_acceptance(ok=False)
    acceptance["blockers"] = [
        "field acceptance requires hardware command sink after CmdVelMux"
    ]
    acceptance["checks"]["runtime_mode"] = {
        "ok": False,
        "command_sink": "mujoco_velocity_adapter",
    }

    payload = build_product_field_check(acceptance, algorithm_gate=_algorithm_gate())

    assert payload["ok"] is False
    assert payload["runtime"]["command_boundary"] == "FAIL"
    assert "field acceptance requires hardware command sink after CmdVelMux" in (
        payload["blockers"]
    )


def test_product_field_check_formats_one_screen_summary():
    from cli.runtime_display import format_product_field_check
    from core.product_field_check import build_product_field_check

    payload = build_product_field_check(
        _gateway_acceptance(),
        algorithm_gate=_algorithm_gate(),
        switch_plan=_runtime_switch_plan(),
    )

    output = format_product_field_check(payload)

    assert "LingTu Field Ready: PASS" in output
    assert "Map: active=unchecked provenance=UNCHECKED" in output
    assert "Runtime: gateway=PASS readiness=PASS localization=PASS" in output
    assert (
        "stages=PASS command_boundary=PASS frontier_preview=PASS "
        "runtime_switch=PASS"
    ) in output
    assert (
        "Stage evidence: "
        "live=slam_or_relayed_localization_map,traversable_frontier_preview,"
        "global_planning,"
        "local_planning_and_following,command_boundary not_live=none missing=none"
    ) in output
    assert "Frontier preview: status=PASS source=traversable_frontier command_published=false" in output
    assert (
        "Runtime switch: status=PASS dry_run=true motion=false publishes=none "
        "from=mujoco_live to=real_s100p"
    ) in output
    assert "Navigation: can_send_goal=PASS route_preview=PASS" in output
    assert "Evidence: real_s100p=PASS age=12.0s mode=field" in output
    assert (
        "Algorithm: strict_benchmark=PASS claim_allowed=true missing=none"
        in output
    )


def test_product_field_check_field_mode_requires_algorithm_benchmark():
    from core.product_field_check import build_product_field_check

    payload = build_product_field_check(
        _gateway_acceptance(),
        algorithm_gate=_algorithm_gate(
            ok=False,
            missing_or_failed=["large_loop_closure", "moving_obstacle_sweep"],
        ),
    )

    strict = payload["algorithm"]["strict_benchmark"]
    assert payload["ok"] is False
    assert payload["summary"] == "FAIL"
    assert strict["status"] == "FAIL"
    assert strict["read_only"] is True
    assert strict["ros2_topic_required"] is False
    assert strict["publishes"] == []
    assert strict["missing_or_failed"] == [
        "large_loop_closure",
        "moving_obstacle_sweep",
    ]
    assert "algorithm strict benchmark is not passing" in payload["blockers"]


def test_product_field_check_preserves_product_algorithm_profile():
    from core.product_field_check import build_product_field_check

    payload = build_product_field_check(
        _gateway_acceptance(mode="simulation"),
        algorithm_gate=_algorithm_gate(
            ok=False,
            missing_or_failed=["large_loop_closure"],
            product_profiles={
                "inspection_mvp": {
                    "ok": True,
                    "status": "PASS",
                    "missing_or_failed": [],
                    "required_gate_sequence": [
                        "routecheck_preflight",
                        "large_terrain",
                        "fastlio2_dynamic_inspection",
                    ],
                }
            },
        ),
    )

    assert payload["algorithm"]["active_product_profile"] == "inspection_mvp"
    assert payload["algorithm"]["product_profiles"]["inspection_mvp"]["ok"] is True
    assert payload["algorithm"]["strict_benchmark"]["status"] == "FAIL"


def test_product_field_check_non_motion_keeps_algorithm_as_advisory():
    from core.product_field_check import build_product_field_check

    payload = build_product_field_check(
        _gateway_acceptance(mode="non_motion"),
        algorithm_gate=_algorithm_gate(ok=False, missing_or_failed=["large_terrain"]),
    )

    assert payload["ok"] is True
    assert "algorithm strict benchmark is not passing" not in payload["blockers"]
    assert "algorithm strict benchmark is not currently passing" in payload["advisories"]
