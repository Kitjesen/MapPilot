from __future__ import annotations

from pathlib import Path

from sim.scripts.mujoco import long_range_navigation_acceptance as acceptance


def _make_stub_report(
    *,
    ok: bool,
    reached: bool,
    distance: float = 5.0,
    path: float = 5.0,
    blockers=None,
    video_ok: bool | None = True,
    video_path: str | None = "attempt_01/native_navigation.mp4",
) -> dict:
    if blockers is None:
        blockers = []
    return {
        "ok": ok,
        "phases": {
            "motion": {
                "goal_metrics": {
                    "native_goal_reached": reached,
                    "distance_reduction_m": distance,
                    "sim_path_length_xy_m": path,
                },
                "video": {
                    "requested": True,
                    "ok": video_ok,
                    **({"path": video_path} if video_path is not None else {}),
                },
                "blockers": blockers,
            }
        },
    }


def test_run_single_navigation_goal_resolves_video_path(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")

    video_file = tmp_path / "attempt_01" / "native_navigation.mp4"
    video_file.parent.mkdir(parents=True, exist_ok=True)
    video_file.write_text("dummy", encoding="utf-8")

    def run_stub(args) -> dict:
        return _make_stub_report(
            ok=True,
            reached=True,
            distance=3.0,
            path=4.0,
            video_path="native_navigation.mp4",
        )

    report = acceptance._run_single_navigation_goal(
        attempt_index=0,
        manifest_path=manifest,
        out_base=tmp_path,
        goal=[50.0, 0.0, 0.30, 0.0],
        domain_id_base=220,
        phase="motion",
        record_video=True,
        run_fn=run_stub,
    )

    assert report["video_path"] == str(video_file)


def test_run_exports_summary_html(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")

    def run_stub(args) -> dict:
        return _make_stub_report(ok=True, reached=True, distance=7.0, path=9.0, video_path="native_navigation.mp4")

    # generate report in function path
    report = acceptance.run_long_range_acceptance(
        manifest=manifest,
        attempts=1,
        min_distance_m=50.0,
        max_distance_m=50.0,
        phase="motion",
        artifact_dir=tmp_path / "artifacts",
        strict=True,
        run_fn=run_stub,
    )

    html = acceptance._build_summary_html(report)
    assert "Long-range native navigation acceptance" in html
    assert "attempt" in html

def test_build_long_range_goals_matches_50_to_70_meter_sequence() -> None:
    goals = acceptance._build_long_range_goals(count=3, min_distance_m=50.0, max_distance_m=70.0)

    assert goals == [
        [50.0, -0.15, 0.30, 0.0],
        [60.0, 0.0, 0.30, 0.0],
        [70.0, 0.15, 0.30, 0.0],
    ]


def test_run_single_navigation_goal_records_goal_and_summary_fields(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")

    calls: list[tuple[str, str, int]] = []

    def run_stub(args) -> dict:
        calls.append((args.mode, args.out_dir, int(args.domain_id)))
        return _make_stub_report(ok=True, reached=True, distance=7.0, path=9.0)

    report = acceptance._run_single_navigation_goal(
        attempt_index=0,
        manifest_path=manifest,
        out_base=tmp_path,
        goal=[58.0, 0.0, 0.30, 0.0],
        domain_id_base=220,
        phase="motion",
        record_video=False,
        run_fn=run_stub,
    )

    assert len(calls) == 1
    assert calls[0][0] == "motion"
    assert calls[0][1] == str(tmp_path / "attempt_01")
    assert calls[0][2] == 220
    assert report["attempt"] == 1
    assert report["goal"] == [58.0, 0.0, 0.30, 0.0]
    assert report["ok"] is True
    assert report["goal_reached"] is True
    assert report["goal_distance_reduction_m"] == 7.0
    assert report["sim_path_length_xy_m"] == 9.0
    assert report["video"]["ok"] is True

    artifacts = list((tmp_path / "manifests").glob("attempt_01.json"))
    assert len(artifacts) == 1
    assert "58.0" in artifacts[0].read_text(encoding="utf-8")


def test_long_range_acceptance_stops_on_first_failure_with_strict_mode(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")
    call_count = 0

    def run_stub(args) -> dict:
        nonlocal call_count
        call_count += 1
        ok = call_count != 2
        return _make_stub_report(
            ok=ok,
            reached=False if not ok else True,
            distance=5.0 if ok else 1.0,
            video_ok=True if ok else None,
            video_path="attempt_01/native_navigation.mp4" if ok else None,
        )

    report = acceptance.run_long_range_acceptance(
        manifest=manifest,
        attempts=5,
        min_distance_m=50.0,
        max_distance_m=70.0,
        phase="motion",
        artifact_dir=tmp_path / "artifacts",
        strict=True,
        run_fn=run_stub,
    )

    assert call_count == 2
    assert report["attempts"] == 2
    assert report["required_attempts"] == 5
    assert report["successes"] == 1
    assert report["failures"] == 1
    assert report["ok"] is False
    assert report["all_success"] is False
    assert report["attempts_report"][1]["ok"] is False
    assert report["attempts_report"][1]["video"]["requested"] is True
    assert report["attempts_report"][1]["video"]["ok"] is None
    assert "path" not in report["attempts_report"][1].get("video", {})


def test_long_range_acceptance_runs_all_attempts_without_strict_and_reports_summary(tmp_path: Path) -> None:
    manifest = tmp_path / "manifest.json"
    manifest.write_text("{}", encoding="utf-8")
    call_count = 0

    def run_stub(args) -> dict:
        nonlocal call_count
        call_count += 1
        return _make_stub_report(
            ok=bool(call_count % 2),
            reached=bool(call_count % 2),
            distance=6.0,
            blockers=[] if call_count % 2 else ["mock_blocker"],
            video_ok=bool(call_count % 2),
            video_path="attempt_01/native_navigation.mp4",
        )

    report = acceptance.run_long_range_acceptance(
        manifest=manifest,
        attempts=4,
        min_distance_m=50.0,
        max_distance_m=70.0,
        phase="motion",
        artifact_dir=tmp_path / "artifacts",
        strict=False,
        run_fn=run_stub,
    )

    assert call_count == 4
    assert report["attempts"] == 4
    assert report["successes"] == 2
    assert report["failures"] == 2
    assert report["ok"] is False
    assert report["all_success"] is False
    assert len(report["attempts_report"]) == 4
