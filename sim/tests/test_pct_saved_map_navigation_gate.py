import argparse
import hashlib
import json
import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.sim]


def _write_saved_map_contract_assets(
    tmp_path: Path,
    *,
    route_tomogram: Path | None = None,
) -> tuple[Path, Path, Path, Path]:
    map_dir = tmp_path / "same_source_map"
    map_dir.mkdir(parents=True, exist_ok=True)
    scene_xml = map_dir / "scene.xml"
    scene_xml.write_text("<mujoco><worldbody/></mujoco>\n", encoding="utf-8")
    map_pcd = map_dir / "map.pcd"
    map_pcd.write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n0 0 0\n1 0 0\n",
        encoding="ascii",
    )
    tomogram = map_dir / "tomogram.pickle"
    tomogram.write_bytes(b"saved map contract tomogram")
    map_sha = hashlib.sha256(map_pcd.read_bytes()).hexdigest()
    tomogram_sha = hashlib.sha256(tomogram.read_bytes()).hexdigest()
    metadata = map_dir / "metadata.json"
    metadata.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.saved_map_artifacts.v1",
                "world": str(scene_xml),
                "artifacts": {
                    "map_pcd": {
                        "path": str(map_pcd),
                        "sha256": map_sha,
                        "point_count": 2,
                    },
                    "tomogram": {
                        "path": str(tomogram),
                        "sha256": tomogram_sha,
                        "source_map_sha256": map_sha,
                    },
                },
            }
        ),
        encoding="utf-8",
    )
    relocalize = tmp_path / "relocalize.json"
    relocalize.write_text(
        json.dumps(
            {
                "ok": True,
                "runtime_relocalization_validated": True,
                "map_pcd": str(map_pcd),
                "service": {"success": True},
                "localizer": {
                    "latest_health_state": "LOCKED",
                    "saved_map_cloud_points_latest": 1500,
                },
            }
        ),
        encoding="utf-8",
    )
    source = tmp_path / "source_report.json"
    source.write_text(
        json.dumps(
            {
                "validation_level": "saved_map_contract_test",
                "cases": [
                    {
                        "route": "saved_map_internal",
                        "assets": {
                            "scene_xml": str(scene_xml),
                            "tomogram": str(route_tomogram or tomogram),
                            "map_pcd": str(map_pcd),
                            "metadata": str(metadata),
                            "map_metadata": str(metadata),
                            "start": [0.0, 0.0, 0.0],
                            "goal": [1.0, 0.0, 0.0],
                        },
                        "selection": {
                            "primary_planner": "pct",
                            "selected_planner": "pct",
                            "fallback_used": False,
                            "selected_route_ok": True,
                        },
                        "path_safety": {"ok": True},
                        "planning": [
                            {
                                "planner": "pct",
                                "planner_class": "PCTPlanner",
                                "pct_planner_runtime": {
                                    "runtime": "rust_process",
                                    "ok": True,
                                },
                                "pct_optimizer_enabled": False,
                                "pct_planner_path_mode": "astar_raw_path",
                                "plan_ms": 2.5,
                                "path_safety": {"ok": True},
                                "goal": [1.0, 0.0, 0.0],
                                "path": [
                                    [0.0, 0.0, 0.5],
                                    [1.0, 0.0, 0.5],
                                ],
                            }
                        ],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    return relocalize, tomogram, scene_xml, source


def _saved_map_full_args(
    *,
    tmp_path: Path,
    relocalize: Path,
    tomogram: Path,
    scene_xml: Path,
) -> argparse.Namespace:
    return argparse.Namespace(
        tomogram=tomogram,
        relocalize_report=relocalize,
        scene_xml=scene_xml,
        source_report=None,
        contract_only=False,
        run_dir=tmp_path / "run",
        json_out=tmp_path / "full_report.json",
        route_name="saved_map_internal",
        ros_domain_id="pytest",
        timeout_s=1.0,
        min_route_progress_ratio=0.9,
        near_field_stop_distance=0.35,
        goal_threshold_m=0.50,
        goal_clear_range=0.45,
        waypoint_threshold_m=0.25,
        waypoint_safety_margin=0.12,
        waypoint_detour_margin=0.18,
        waypoint_collision_sample_step=0.05,
        sim_vehicle="omni_cart",
        start=None,
        goal=[1.0, 0.0],
        video_out=None,
        video_layout="scene_overlay",
        video_width=640,
        video_height=360,
        video_fps=20.0,
        strict=True,
    )


def _pct_full_result() -> list[list[float]]:
    return [
        [0.0, 0.0, 0.5],
        [1.0, 0.0, 0.5],
    ]


def _pct_preview_report() -> dict:
    return {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "status": {
            "ok": True,
            "code": "SUCCESS",
            "reached_goal": True,
            "returned_path_blocked_sample_count": 0,
            "obstacle_clear": True,
            "kinematics": {"ok": True},
        },
        "status_code": "SUCCESS",
        "planner": "pct",
        "path_count": 2,
        "start": [0.0, 0.0, 0.5],
        "goal": [1.0, 0.0, 0.5],
        "first": [0.0, 0.0, 0.5],
        "last": [1.0, 0.0, 0.5],
        "path": {
            "count": 2,
            "finite": True,
            "samples": {
                "by_index": [
                    {"index": 0, "point": [0.0, 0.0, 0.5]},
                    {"index": 1, "point": [1.0, 0.0, 0.5]},
                ]
            },
        },
        "diagnostics": {
            "last_optimizer_enabled": False,
            "last_optimizer_attempted": False,
            "last_optimizer_accepted": None,
            "last_optimizer_reject_reason": "",
            "last_optimizer_blocked_sample_count": 0,
            "last_path_mode": "rust_astar_raw_path",
            "optimize_trajectory": False,
        },
        "runtime": {
            "runtime": "rust_process",
            "planner_impl_class": "TomogramPlanner",
            "official_native_pct_impl": False,
            "lib_dir": "/rust/pct",
            "arch": "x86_64",
            "python": "py310",
        },
    }


def test_pct_saved_map_defaults_use_open_saved_map_route() -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    args = gate._build_parser().parse_args([])

    assert args.goal == [4.5, 3.0]
    assert args.near_field_stop_distance == 0.35
    assert args.goal_threshold_m == 0.50


def test_run_plan_preview_uses_canonical_runtime_with_optimizer_disabled(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    tomogram = tmp_path / "tomogram.pickle"
    tomogram.write_bytes(b"trusted test tomogram")
    full_result = [
        [0.0, 0.0, 0.5],
        [0.5, 0.0, 0.5],
        [1.0, 0.0, 0.5],
    ]
    captured: dict[str, object] = {}

    class Planner:
        def plan(self, start_xy, goal_xy, start_z, goal_z):
            captured["plan_args"] = (start_xy.tolist(), goal_xy.tolist(), start_z, goal_z)
            return full_result

    runtime = SimpleNamespace(
        planner=Planner(),
        runtime_paths=SimpleNamespace(
            lib_dir=tmp_path / "runtime",
            canonical_arch="x86_64",
            python_tag="py310",
        ),
    )
    canonical_report = {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "planner": "pct",
        "path_count": len(full_result),
        "diagnostics": {
            "last_optimizer_enabled": False,
            "last_path_mode": "rust_astar_raw_path",
        },
        "runtime": {"planner_impl_class": "TomogramPlanner"},
    }

    def fake_load(path, *, repo_root, planner_config):
        captured["load"] = (path, repo_root)
        assert planner_config.planner.optimize_trajectory is False
        return runtime

    def fake_build_preview_report(**kwargs):
        captured["preview_kwargs"] = kwargs
        assert kwargs["result"] is full_result
        return canonical_report

    def fail_subprocess(*_args, **_kwargs):
        raise AssertionError("plan preview must not launch a subprocess")

    monkeypatch.setattr(gate, "load_pct_planner_runtime", fake_load, raising=False)
    monkeypatch.setattr(gate, "build_preview_report", fake_build_preview_report, raising=False)
    monkeypatch.setattr(gate.subprocess, "run", fail_subprocess)

    report, result = gate._run_plan_preview(
        argparse.Namespace(
            start=[0.0, 0.0, 0.5],
            goal=[1.0, 0.0, 0.5],
        ),
        tomogram=tomogram,
        out_path=tmp_path / "preview.json",
    )

    assert captured["load"] == (tomogram, gate.ROOT)
    assert captured["plan_args"] == ([0.0, 0.0], [1.0, 0.0], 0.5, 0.5)
    preview_kwargs = captured["preview_kwargs"]
    assert isinstance(preview_kwargs, dict)
    assert preview_kwargs["planner"] is runtime.planner
    assert preview_kwargs["runtime_paths"] is runtime.runtime_paths
    assert preview_kwargs["result"] is full_result
    assert preview_kwargs["start"].tolist() == [0.0, 0.0, 0.5]
    assert preview_kwargs["goal"].tolist() == [1.0, 0.0, 0.5]
    assert preview_kwargs["tomogram_path"] == tomogram
    assert report is canonical_report
    assert result is full_result
    assert json.loads((tmp_path / "preview.json").read_text(encoding="utf-8")) == canonical_report


def test_run_plan_preview_uses_active_last_pose_when_start_is_omitted(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    map_dir = tmp_path / "map"
    active_dir = map_dir / "active"
    active_dir.mkdir(parents=True)
    tomogram = map_dir / "tomogram.pickle"
    tomogram.write_bytes(b"trusted test tomogram")
    (map_dir / "last_pose.txt").write_text("9.0 9.0 0.0\n", encoding="utf-8")
    (active_dir / "last_pose.txt").write_text("1.25 2.5 0.75\n", encoding="utf-8")
    captured: dict[str, object] = {}

    class Planner:
        @staticmethod
        def get_surface_height(point):
            captured.setdefault("height_points", []).append(point.tolist())
            return 0.4

        @staticmethod
        def plan(start_xy, goal_xy, start_z, goal_z):
            captured["plan_args"] = (start_xy.tolist(), goal_xy.tolist(), start_z, goal_z)
            return [[1.25, 2.5, 0.4], [3.0, 4.0, 0.4]]

    planner = Planner()
    runtime = SimpleNamespace(
        planner=planner,
        runtime_paths=SimpleNamespace(
            lib_dir=tmp_path / "runtime",
            canonical_arch="x86_64",
            python_tag="py310",
        ),
    )
    canonical_report = {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "planner": "pct",
        "path_count": 2,
        "diagnostics": {
            "last_optimizer_enabled": False,
            "last_path_mode": "rust_astar_raw_path",
        },
        "runtime": {"planner_impl_class": "TomogramPlanner"},
    }

    monkeypatch.setattr(
        gate,
        "load_pct_planner_runtime",
        lambda *_args, **_kwargs: runtime,
    )
    monkeypatch.setattr(
        gate,
        "build_preview_report",
        lambda **_kwargs: canonical_report,
    )

    gate._run_plan_preview(
        argparse.Namespace(start=None, goal=[3.0, 4.0]),
        tomogram=tomogram,
        out_path=tmp_path / "preview.json",
    )

    assert captured["height_points"] == [[1.25, 2.5], [3.0, 4.0]]
    assert captured["plan_args"] == ([1.25, 2.5], [3.0, 4.0], 0.4, 0.4)


@pytest.mark.parametrize("last_pose_contents", [None, "invalid pose\n", "1.0 2.0\n", "nan 2.0 0.0\n"])
def test_run_plan_preview_fails_closed_without_a_valid_start(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    last_pose_contents: str | None,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    tomogram = tmp_path / "map" / "tomogram.pickle"
    tomogram.parent.mkdir(parents=True)
    tomogram.write_bytes(b"trusted test tomogram")
    if last_pose_contents is not None:
        (tomogram.parent / "last_pose.txt").write_text(
            last_pose_contents,
            encoding="utf-8",
        )

    def fail_load(*_args, **_kwargs):
        raise AssertionError("runtime must not load without a valid start")

    monkeypatch.setattr(gate, "load_pct_planner_runtime", fail_load)

    with pytest.raises((FileNotFoundError, ValueError), match="last_pose.txt"):
        gate._run_plan_preview(
            argparse.Namespace(start=None, goal=[3.0, 4.0]),
            tomogram=tomogram,
            out_path=tmp_path / "preview.json",
        )


def test_run_native_gate_forwards_control_params(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    captured: dict[str, list[str]] = {}

    class Completed:
        returncode = 0
        stdout = "native ok"
        stderr = ""

    def fake_run(cmd, **_kwargs):
        captured["cmd"] = [str(part) for part in cmd]
        out_path = Path(captured["cmd"][captured["cmd"].index("--json-out") + 1])
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps({"ok": True}), encoding="utf-8")
        return Completed()

    monkeypatch.setattr(gate.subprocess, "run", fake_run)
    args = argparse.Namespace(
        route_name="saved_map_internal",
        ros_domain_id="75",
        timeout_s=80.0,
        sim_vehicle="omni_cart",
        min_route_progress_ratio=0.91,
        near_field_stop_distance=0.35,
        goal_threshold_m=0.5,
        goal_clear_range=0.45,
        waypoint_threshold_m=0.25,
        waypoint_safety_margin=0.12,
        waypoint_detour_margin=0.18,
        waypoint_collision_sample_step=0.05,
        video_out=None,
    )

    report = gate._run_native_gate(
        args,
        source_report=tmp_path / "source.json",
        out_path=tmp_path / "native" / "report.json",
    )

    cmd = captured["cmd"]

    def value_after(flag: str) -> str:
        return cmd[cmd.index(flag) + 1]

    assert report["ok"] is True
    assert value_after("--timeout-s") == "80.0"
    assert value_after("--min-route-progress-ratio") == "0.91"
    assert value_after("--near-field-stop-distance") == "0.35"
    assert value_after("--goal-threshold-m") == "0.5"
    assert value_after("--goal-clear-range") == "0.45"
    assert value_after("--waypoint-threshold-m") == "0.25"
    assert value_after("--waypoint-safety-margin") == "0.12"
    assert value_after("--waypoint-detour-margin") == "0.18"
    assert value_after("--waypoint-collision-sample-step") == "0.05"


def test_resolve_tomogram_prefers_relocalized_same_source_map(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalized_map = tmp_path / "same_source_map" / "map.pcd"
    relocalized_tomogram = relocalized_map.parent / "tomogram.pickle"
    newer_unrelated = tmp_path / "unrelated" / "same_source_map" / "tomogram.pickle"
    for path in (relocalized_map, relocalized_tomogram, newer_unrelated):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset")

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    assert gate._resolve_tomogram(
        None,
        relocalize_report={"map_pcd": str(relocalized_map)},
    ) == relocalized_tomogram


def test_resolve_tomogram_rejects_global_fallback_when_relocalized_tomogram_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalized_map = tmp_path / "same_source_map" / "map.pcd"
    unrelated_tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/mujoco_fastlio2_live/run/same_source_map/tomogram.pickle"
    )
    for path in (relocalized_map, unrelated_tomogram):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset")

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    with pytest.raises(
        FileNotFoundError,
        match="relocalize_report.map_pcd sibling tomogram missing",
    ):
        gate._resolve_tomogram(
            None,
            relocalize_report={"map_pcd": str(relocalized_map)},
        )


def test_resolve_tomogram_rejects_explicit_path_from_different_map(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalized_map = tmp_path / "same_source_map_a" / "map.pcd"
    relocalized_tomogram = relocalized_map.parent / "tomogram.pickle"
    mismatched_tomogram = tmp_path / "same_source_map_b" / "tomogram.pickle"
    for path in (relocalized_map, relocalized_tomogram, mismatched_tomogram):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"asset")

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    with pytest.raises(ValueError, match="explicit tomogram does not match"):
        gate._resolve_tomogram(
            mismatched_tomogram,
            relocalize_report={"map_pcd": str(relocalized_map)},
        )


def test_resolve_tomogram_rejects_missing_explicit_same_source_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalized_map = tmp_path / "same_source_map" / "map.pcd"
    relocalized_map.parent.mkdir(parents=True, exist_ok=True)
    relocalized_map.write_bytes(b"asset")
    expected_tomogram = relocalized_map.parent / "tomogram.pickle"

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    with pytest.raises(FileNotFoundError, match="tomogram not found"):
        gate._resolve_tomogram(
            expected_tomogram,
            relocalize_report={"map_pcd": str(relocalized_map)},
        )


def test_select_preview_case_reports_canonical_preview_failure() -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    with pytest.raises(RuntimeError, match="PCT preview failed: NO_PATH"):
        gate._select_preview_case(
            {
                "schema": "lingtu.pct.preview.actual.v2",
                "ok": False,
                "planner": "pct",
                "path_count": 0,
                "status_code": "NO_PATH",
                "error": "pct returned no path",
            }
        )


def test_build_source_report_uses_full_planner_result(
    tmp_path: Path,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    preview = _pct_preview_report()
    preview["diagnostics"]["last_path_mode"] = "rust_astar_raw_path"
    preview["runtime"] = {
        "runtime": "rust_process",
        "planner_impl_class": "TomogramPlanner",
        "lib_dir": "/rust/pct",
        "arch": "x86_64",
        "python": "py310",
    }
    preview["path_count"] = 4
    preview["path"]["count"] = 4
    preview["path"]["samples"]["by_index"] = [
        {"index": 0, "point": [0.0, 0.0, 0.5]},
        {"index": 3, "point": [1.5, 0.0, 0.5]},
    ]
    full_result = [
        [0.0, 0.0, 0.5],
        [0.5, 0.0, 0.5],
        [1.0, 0.0, 0.5],
        [1.5, 0.0, 0.5],
    ]
    tomogram = tmp_path / "tomogram.pickle"
    scene_xml = tmp_path / "scene.xml"
    obstacle_metadata = tmp_path / "obstacles.json"
    output = tmp_path / "source.json"

    source = gate._build_source_report(
        preview=gate._select_preview_case(preview),
        result=full_result,
        tomogram=tomogram,
        scene_xml=scene_xml,
        map_pcd=None,
        map_metadata=None,
        obstacle_metadata=obstacle_metadata,
        output=output,
        route_name="saved_map_internal",
    )

    planning = source["cases"][0]["planning"][0]
    assert planning["path"] == full_result
    assert planning["path"] != [
        sample["point"] for sample in preview["path"]["samples"]["by_index"]
    ]
    assert planning["planner_class"] == "TomogramPlanner"
    assert planning["pct_planner_runtime"]["ok"] is True
    assert planning["pct_planner_runtime"]["runtime"] == "rust_process"
    assert planning["pct_optimizer_enabled"] is False
    assert planning["pct_planner_path_mode"] == "astar_raw_path"
    assert "native_runtime_used" not in planning
    assert "native_runtime" not in planning
    assert json.loads(output.read_text(encoding="utf-8")) == source


@pytest.mark.parametrize(
    "raw_path_mode",
    [
        "native_optimized_trajectory",
        "rust_optimized_trajectory",
    ],
)
def test_build_source_report_normalizes_runtime_optimized_path_modes(
    tmp_path: Path,
    raw_path_mode: str,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    preview = _pct_preview_report()
    preview["diagnostics"].update(
        {
            "last_optimizer_enabled": True,
            "last_optimizer_attempted": True,
            "last_optimizer_accepted": True,
            "last_path_mode": raw_path_mode,
        }
    )

    source = gate._build_source_report(
        preview=gate._select_preview_case(preview),
        result=_pct_full_result(),
        tomogram=tmp_path / "tomogram.pickle",
        scene_xml=tmp_path / "scene.xml",
        map_pcd=None,
        map_metadata=None,
        obstacle_metadata=tmp_path / "obstacles.json",
        output=tmp_path / "source.json",
        route_name="saved_map_internal",
    )

    planning = source["cases"][0]["planning"][0]
    assert planning["pct_planner_path_mode"] == "optimized_trajectory"


def test_pct_saved_map_contract_only_validates_source_binding(
    tmp_path: Path,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, source = _write_saved_map_contract_assets(tmp_path)
    out = tmp_path / "contract_report.json"

    report = gate.run_gate(
        argparse.Namespace(
            tomogram=tomogram,
            relocalize_report=relocalize,
            scene_xml=scene_xml,
            source_report=source,
            contract_only=True,
            run_dir=tmp_path / "run",
            json_out=out,
            route_name="saved_map_internal",
        )
    )

    assert report["ok"] is True
    assert report["execution_mode"] == "contract_only"
    assert report["validation_only"] is True
    assert report["claim_boundary"] == "contract_only_no_pct_preview_or_mujoco_motion"
    assert report["relocalization"]["ok"] is True
    assert report["plan_preview"]["source"] == "source_report_contract_only"
    assert report["native_gate"]["claim_boundary"] == "contract_only_no_mujoco_motion"
    assert report["contract_checks"] == {
        "relocalization_locked": True,
        "source_report_loads": True,
        "tomogram_matches_relocalization": True,
        "map_pcd_matches_relocalization": True,
        "scene_xml_matches_saved_map": True,
        "pct_no_fallback": True,
        "pct_planner_runtime_selected": True,
        "pct_planner_runtime_ok": True,
        "pct_optimizer_disabled": True,
        "pct_astar_raw_path": True,
        "same_source_map_artifact": True,
        "same_source_hash_identity": True,
    }
    assert report["same_source_hash_identity"]["ok"] is True
    assert all(report["same_source_hash_identity"]["checks"].values())
    assert json.loads(out.read_text(encoding="utf-8"))["ok"] is True


def test_pct_saved_map_contract_only_rejects_stale_relocalize_report(
    tmp_path: Path,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, source = _write_saved_map_contract_assets(tmp_path)
    old = time.time() - 120.0
    os.utime(relocalize, (old, old))

    report = gate.run_gate(
        argparse.Namespace(
            tomogram=tomogram,
            relocalize_report=relocalize,
            scene_xml=scene_xml,
            source_report=source,
            contract_only=True,
            run_dir=tmp_path / "run",
            json_out=tmp_path / "contract_report.json",
            route_name="saved_map_internal",
            max_relocalize_report_age_s=1.0,
        )
    )

    assert report["ok"] is False
    assert report["relocalize_report_freshness"]["fresh"] is False
    assert any("report_age_s" in blocker for blocker in report["blockers"])


def test_pct_saved_map_contract_only_rejects_tomogram_mismatch(
    tmp_path: Path,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    mismatched = tmp_path / "other" / "tomogram.pickle"
    mismatched.parent.mkdir(parents=True, exist_ok=True)
    mismatched.write_bytes(b"other tomogram")
    relocalize, tomogram, scene_xml, source = _write_saved_map_contract_assets(
        tmp_path,
        route_tomogram=mismatched,
    )

    report = gate.run_gate(
        argparse.Namespace(
            tomogram=tomogram,
            relocalize_report=relocalize,
            scene_xml=scene_xml,
            source_report=source,
            contract_only=True,
            run_dir=tmp_path / "run",
            json_out=tmp_path / "contract_report.json",
            route_name="saved_map_internal",
        )
    )

    assert report["ok"] is False
    assert any(
        "source report tomogram does not match" in blocker
        for blocker in report["blockers"]
    )
    assert any(
        "source_tomogram_file_sha256_matches_metadata" in blocker
        for blocker in report["blockers"]
    )


def test_pct_saved_map_contract_only_writes_report_when_tomogram_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    monkeypatch.setattr(gate, "ROOT", tmp_path)

    map_dir = tmp_path / "same_source_map"
    map_dir.mkdir(parents=True)
    map_pcd = map_dir / "map.pcd"
    map_pcd.write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="ascii",
    )
    unrelated_tomogram = (
        tmp_path
        / "artifacts/server_sim_closure/mujoco_fastlio2_live/run/same_source_map/tomogram.pickle"
    )
    unrelated_tomogram.parent.mkdir(parents=True, exist_ok=True)
    unrelated_tomogram.write_bytes(b"unrelated tomogram")
    relocalize = tmp_path / "relocalize.json"
    relocalize.write_text(
        json.dumps(
            {
                "ok": True,
                "runtime_relocalization_validated": True,
                "map_pcd": str(map_pcd),
                "service": {"success": True},
                "localizer": {
                    "latest_health_state": "LOCKED",
                    "saved_map_cloud_points_latest": 1500,
                },
            }
        ),
        encoding="utf-8",
    )
    out = tmp_path / "contract_report.json"

    report = gate.run_gate(
        argparse.Namespace(
            tomogram=None,
            relocalize_report=relocalize,
            scene_xml=None,
            source_report=None,
            contract_only=True,
            run_dir=tmp_path / "run",
            json_out=out,
            route_name="saved_map_internal",
        )
    )

    assert report["ok"] is False
    assert report["execution_mode"] == "contract_only"
    assert report["claim_boundary"] == "contract_only_no_pct_preview_or_mujoco_motion"
    expected_tomogram = map_dir / "tomogram.pickle"
    assert report.get("tomogram", "") != str(unrelated_tomogram)
    assert report["blockers"] == [
        f"relocalize_report.map_pcd sibling tomogram missing: {expected_tomogram}"
    ]
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["ok"] is False
    assert saved["blockers"] == report["blockers"]


def test_pct_saved_map_full_run_validates_source_identity_before_native(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, _source = _write_saved_map_contract_assets(tmp_path)
    preview = _pct_preview_report()
    native_called: dict[str, Path] = {}

    def fake_preview(args, *, tomogram, out_path):
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(preview), encoding="utf-8")
        return preview, _pct_full_result()

    def fake_native(args, *, source_report, out_path):
        native_called["source_report"] = source_report
        assert source_report.exists()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        return {
            "ok": True,
            "selected_planner": "pct",
            "fallback_used": False,
            "reached_goal": True,
            "pct_planner_runtime": {
                "runtime": "rust_process",
                "ok": True,
            },
            "pct_planner_runtime_ok": True,
            "pct_path_count": 2,
            "path_count": 1,
            "max_path_poses": 2,
            "cmd_count_nonzero": 2,
            "moved_m": 1.0,
        }

    monkeypatch.setattr(gate, "_run_plan_preview", fake_preview)
    monkeypatch.setattr(gate, "_run_native_gate", fake_native)

    report = gate.run_gate(
        _saved_map_full_args(
            tmp_path=tmp_path,
            relocalize=relocalize,
            tomogram=tomogram,
            scene_xml=scene_xml,
        )
    )

    assert report["ok"] is True
    assert native_called["source_report"] == tmp_path / "run" / "pct_saved_map_source_report.json"
    assert report["same_source_hash_identity"]["ok"] is True
    assert all(report["same_source_hash_identity"]["checks"].values())
    assert report["plan_preview"]["pct_optimizer_enabled"] is False
    assert report["plan_preview"]["pct_planner_path_mode"] == "astar_raw_path"
    source_report = json.loads(native_called["source_report"].read_text(encoding="utf-8"))
    planning = source_report["cases"][0]["planning"][0]
    assert planning["pct_optimizer_enabled"] is False
    assert planning["pct_planner_path_mode"] == "astar_raw_path"
    assert report["source_planning_contract"]["pct_optimizer_enabled"] is False
    assert (
        report["source_planning_contract"]["pct_planner_path_mode"]
        == "astar_raw_path"
    )
    assert report["contract_checks"]["relocalization_locked"] is True
    assert report["contract_checks"]["same_source_hash_identity"] is True
    assert report["native_gate"]["ok"] is True


def test_pct_saved_map_full_run_rejects_missing_pct_path_mode_before_native(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, _source = _write_saved_map_contract_assets(tmp_path)
    preview = json.loads(json.dumps(_pct_preview_report()))
    preview["diagnostics"].pop("last_optimizer_enabled")
    preview["diagnostics"].pop("last_path_mode")

    def fake_preview(args, *, tomogram, out_path):
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(preview), encoding="utf-8")
        return preview, _pct_full_result()

    def fail_native(args, *, source_report, out_path):
        raise AssertionError("native gate must not run without PCT mode evidence")

    monkeypatch.setattr(gate, "_run_plan_preview", fake_preview)
    monkeypatch.setattr(gate, "_run_native_gate", fail_native)

    report = gate.run_gate(
        _saved_map_full_args(
            tmp_path=tmp_path,
            relocalize=relocalize,
            tomogram=tomogram,
            scene_xml=scene_xml,
        )
    )

    assert report["ok"] is False
    assert report["blockers"] == ["unsupported PCT planner path mode: ''"]


def test_pct_saved_map_full_run_rejects_optimized_trajectory_mode_before_native(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, _source = _write_saved_map_contract_assets(tmp_path)
    preview = json.loads(json.dumps(_pct_preview_report()))
    preview["diagnostics"]["last_optimizer_enabled"] = True
    preview["diagnostics"]["last_path_mode"] = "optimized_trajectory"

    def fake_preview(args, *, tomogram, out_path):
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(preview), encoding="utf-8")
        return preview, _pct_full_result()

    def fail_native(args, *, source_report, out_path):
        raise AssertionError("native gate must not run for optimized trajectory mode")

    monkeypatch.setattr(gate, "_run_plan_preview", fake_preview)
    monkeypatch.setattr(gate, "_run_native_gate", fail_native)

    report = gate.run_gate(
        _saved_map_full_args(
            tmp_path=tmp_path,
            relocalize=relocalize,
            tomogram=tomogram,
            scene_xml=scene_xml,
        )
    )

    assert report["ok"] is False
    assert "PCT optimizer mode evidence is not disabled" in report["blockers"]
    assert "PCT planner path mode is not astar_raw_path" in report["blockers"]


def test_pct_saved_map_full_run_skips_preview_when_relocalization_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, _source = _write_saved_map_contract_assets(tmp_path)
    payload = json.loads(relocalize.read_text(encoding="utf-8"))
    payload["ok"] = False
    payload["runtime_relocalization_validated"] = False
    relocalize.write_text(json.dumps(payload), encoding="utf-8")

    def fail_preview(*_args, **_kwargs):
        raise AssertionError("PCT preview must not run before relocalization validates")

    def fail_native(*_args, **_kwargs):
        raise AssertionError("native gate must not run before relocalization validates")

    monkeypatch.setattr(gate, "_run_plan_preview", fail_preview)
    monkeypatch.setattr(gate, "_run_native_gate", fail_native)

    report = gate.run_gate(
        _saved_map_full_args(
            tmp_path=tmp_path,
            relocalize=relocalize,
            tomogram=tomogram,
            scene_xml=scene_xml,
        )
    )

    assert report["ok"] is False
    assert report["relocalization"]["ok"] is False
    assert report["plan_preview"]["skipped"] is True
    assert report["plan_preview"]["reason"] == "saved_map_relocalization_prerequisite_failed"
    assert report["native_gate"]["skipped"] is True
    assert report["native_gate"]["reason"] == "saved_map_relocalization_prerequisite_failed"
    assert report["contract_checks"]["source_report_loads"] is False
    assert "saved-map relocalization prerequisite failed" in report["blockers"]


def test_pct_saved_map_full_run_rejects_bad_source_identity_before_native(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import pct_saved_map_navigation_gate as gate

    relocalize, tomogram, scene_xml, _source = _write_saved_map_contract_assets(tmp_path)
    metadata = tomogram.parent / "metadata.json"
    payload = json.loads(metadata.read_text(encoding="utf-8"))
    payload["artifacts"]["tomogram"]["sha256"] = "0" * 64
    metadata.write_text(json.dumps(payload), encoding="utf-8")
    preview = _pct_preview_report()

    def fake_preview(args, *, tomogram, out_path):
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(preview), encoding="utf-8")
        return preview, _pct_full_result()

    def fail_native(args, *, source_report, out_path):
        raise AssertionError("native gate must not run with mismatched source identity")

    monkeypatch.setattr(gate, "_run_plan_preview", fake_preview)
    monkeypatch.setattr(gate, "_run_native_gate", fail_native)

    report = gate.run_gate(
        _saved_map_full_args(
            tmp_path=tmp_path,
            relocalize=relocalize,
            tomogram=tomogram,
            scene_xml=scene_xml,
        )
    )

    assert report["ok"] is False
    assert report["native_gate"]["skipped"] is True
    assert report["native_gate"]["reason"] == "pre_native_contract_failed"
    assert report["same_source_hash_identity"]["ok"] is False
    assert report["contract_checks"]["same_source_hash_identity"] is False
    assert any(
        "same-source hash identity failed" in blocker
        for blocker in report["blockers"]
    )
