# ruff: noqa: S101
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from runtime.algorithm_gates import (
    DIMOS_BENCHMARK_REQUIRED_GATES,
    G4_SERVER_FULL_SIM_REQUIRED_GATES,
    INSPECTION_MVP_REQUIRED_GATES,
)
from sim.diagnostics import summary as sim_diagnostics

pytestmark = pytest.mark.sim

CURRENT_GATES = {
    "gateway_runtime_acceptance",
    "navigation_replay_deviation",
    "saved_map_relocalize",
    "bbs3d_kidnapped_relocalize",
}


def _write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _evaluate_ok(report: dict) -> tuple[bool, list[str], dict]:
    ok = report.get("ok") is True
    return ok, [] if ok else ["report.ok is not true"], {"value": report.get("value")}


def _spec(
    name: str = "alpha",
    *,
    requirements: tuple[str, ...] = sim_diagnostics.LOCAL_NON_MOTION_HOST_REQUIREMENTS,
) -> sim_diagnostics.GateSpec:
    return sim_diagnostics.GateSpec(
        name=name,
        description=f"{name} test gate",
        default_patterns=(f"artifacts/{name}/report.json",),
        evaluator=_evaluate_ok,
        host_requirements=requirements,
    )


def _install_specs(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    *specs: sim_diagnostics.GateSpec,
) -> None:
    monkeypatch.setattr(sim_diagnostics, "ROOT", tmp_path)
    monkeypatch.setattr(sim_diagnostics, "SRC", tmp_path / "src")
    monkeypatch.setattr(sim_diagnostics, "GATES", tuple(specs))


def test_gate_registry_is_the_current_native_set():
    assert {spec.name for spec in sim_diagnostics.GATES} == CURRENT_GATES


def test_gate_specs_are_unique_and_well_formed():
    names = [spec.name for spec in sim_diagnostics.GATES]
    assert len(names) == len(set(names))
    for spec in sim_diagnostics.GATES:
        assert spec.description
        assert spec.default_patterns
        assert callable(spec.evaluator)
        assert sim_diagnostics._expected_report_path(spec)


@pytest.mark.parametrize("spec", sim_diagnostics.GATES, ids=lambda spec: spec.name)
def test_current_gate_evaluators_reject_missing_evidence(spec: sim_diagnostics.GateSpec):
    ok, blockers, evidence = spec.evaluator({})

    assert ok is False
    assert blockers
    assert isinstance(evidence, dict)


def test_presets_share_runtime_gate_constants_and_only_registered_gates():
    assert sim_diagnostics.ALGORITHM_PRESETS["inspection_mvp"] == INSPECTION_MVP_REQUIRED_GATES
    assert sim_diagnostics.ALGORITHM_PRESETS["dimos_benchmark"] == DIMOS_BENCHMARK_REQUIRED_GATES
    assert sim_diagnostics.ALGORITHM_PRESETS["g4_server_full_sim"] == G4_SERVER_FULL_SIM_REQUIRED_GATES
    assert "full_algorithm" not in sim_diagnostics.ALGORITHM_PRESETS
    for names in sim_diagnostics.ALGORITHM_PRESETS.values():
        assert set(names) <= CURRENT_GATES


def test_ordered_gate_names_preserves_public_presets():
    for names in (
        INSPECTION_MVP_REQUIRED_GATES,
        DIMOS_BENCHMARK_REQUIRED_GATES,
        G4_SERVER_FULL_SIM_REQUIRED_GATES,
    ):
        assert sim_diagnostics._ordered_gate_names(set(names)) == list(names)


def test_parser_exposes_current_cli_contract():
    parser = sim_diagnostics._build_parser()
    destinations = {action.dest for action in parser._actions}
    assert {
        "gateway_runtime_acceptance_report",
        "navigation_replay_deviation_report",
        "saved_map_relocalize_report",
        "bbs3d_kidnapped_relocalize_report",
        "required",
        "preset",
        "host_preflight",
        "required_only",
        "strict",
        "dds_domain_id",
        "saved_map_pcd",
    } <= destinations
    assert "multifloor_exploration_report" not in destinations
    preset_action = next(action for action in parser._actions if action.dest == "preset")
    assert set(preset_action.choices) == set(sim_diagnostics.ALGORITHM_PRESETS)


def test_required_from_args_prefers_preset_and_accepts_csv():
    parser = sim_diagnostics._build_parser()
    preset = parser.parse_args(["--preset", "inspection_mvp", "--required", "saved_map_relocalize"])
    csv = parser.parse_args(["--required", "gateway_runtime_acceptance, saved_map_relocalize"])

    assert sim_diagnostics._required_from_args(preset) == set(INSPECTION_MVP_REQUIRED_GATES)
    assert sim_diagnostics._required_from_args(csv) == {
        "gateway_runtime_acceptance",
        "saved_map_relocalize",
    }
    assert sim_diagnostics._required_from_args(parser.parse_args([])) == CURRENT_GATES


def test_summarize_reports_missing_required_gate(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    alpha = _spec()
    _install_specs(monkeypatch, tmp_path, alpha)

    summary = sim_diagnostics.summarize(report_overrides={}, required={"alpha"})

    assert summary["ok"] is False
    assert summary["execution_mode"] == "summary_only"
    assert summary["missing_or_failed"] == ["alpha"]
    assert summary["gates"]["alpha"]["status"] == "missing"
    assert summary["missing_reports"][0]["name"] == alpha.name
    assert summary["simulation_only"] is True
    assert summary["real_robot_motion"] is False
    assert summary["cmd_vel_sent_to_hardware"] is False


def test_summarize_accepts_override_and_preserves_evidence(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(monkeypatch, tmp_path, _spec())
    report = _write_json(tmp_path / "override.json", {"ok": True, "value": 7})

    summary = sim_diagnostics.summarize(
        report_overrides={"alpha": report}, required={"alpha"}, include_optional=False
    )

    assert summary["ok"] is True
    assert summary["required_gate_sequence"] == ["alpha"]
    assert summary["gates"]["alpha"]["status"] == "passed"
    assert summary["gates"]["alpha"]["evidence"] == {"value": 7}


@pytest.mark.parametrize(
    ("contents", "status"),
    [({"ok": False}, "failed"), (None, "invalid")],
)
def test_summarize_records_failed_or_invalid_report(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, contents: dict | None, status: str
):
    _install_specs(monkeypatch, tmp_path, _spec())
    report = tmp_path / "report.json"
    if contents is None:
        report.write_text("not json", encoding="utf-8")
    else:
        _write_json(report, contents)

    summary = sim_diagnostics.summarize(
        report_overrides={"alpha": report}, required={"alpha"}, include_optional=False
    )

    assert summary["ok"] is False
    assert summary["gates"]["alpha"]["status"] == status


def test_summarize_separates_and_can_omit_optional_gates(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(monkeypatch, tmp_path, _spec("alpha"), _spec("beta"))
    report = _write_json(tmp_path / "alpha.json", {"ok": True})

    complete = sim_diagnostics.summarize(
        report_overrides={"alpha": report}, required={"alpha"}, include_optional=True
    )
    required_only = sim_diagnostics.summarize(
        report_overrides={"alpha": report}, required={"alpha"}, include_optional=False
    )

    assert complete["ok"] is True
    assert complete["optional_missing_or_failed"] == ["beta"]
    assert set(required_only["gates"]) == {"alpha"}
    assert required_only["optional_missing_or_failed"] == []


def test_summarize_rejects_stale_required_report(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(monkeypatch, tmp_path, _spec())
    report = _write_json(tmp_path / "old.json", {"ok": True})
    monkeypatch.setattr(sim_diagnostics.time, "time", lambda: report.stat().st_mtime + 20.0)

    summary = sim_diagnostics.summarize(
        report_overrides={"alpha": report},
        required={"alpha"},
        max_report_age_s=10.0,
        include_optional=False,
    )

    assert summary["ok"] is False
    assert summary["gates"]["alpha"]["is_fresh"] is False
    assert "max_report_age_s" in summary["remaining_gaps"][0]


def test_best_match_uses_latest_report(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    spec = _spec()
    _install_specs(monkeypatch, tmp_path, spec)
    failed = _write_json(tmp_path / "failed.json", {"ok": False})
    passed = _write_json(tmp_path / "passed.json", {"ok": True})
    monkeypatch.setattr(sim_diagnostics, "_candidate_matches", lambda _patterns: [failed, passed])

    assert sim_diagnostics._best_match(spec) == failed


def test_host_preflight_marks_local_gate_runnable(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(monkeypatch, tmp_path, _spec())

    report = sim_diagnostics.host_preflight(
        required={"alpha"},
        platform_system="Windows",
        env={},
        module_available=lambda _name: False,
        path_exists=lambda _path: False,
    )

    assert report["ok"] is True
    assert report["execution_mode"] == "host_preflight_only"
    assert report["runnable_gates"] == ["alpha"]
    assert report["gates"]["alpha"]["checks"]["local_non_motion"]["ok"] is True


def test_host_preflight_blocks_native_gate_when_assets_are_missing(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(
        monkeypatch,
        tmp_path,
        _spec(requirements=sim_diagnostics.NATIVE_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS),
    )

    report = sim_diagnostics.host_preflight(
        required={"alpha"},
        platform_system="Linux",
        env={"LINGTU_DDS_DOMAIN_ID": "231", "MUJOCO_GL": "egl"},
        module_available=lambda name: name == "mujoco",
        path_exists=lambda _path: False,
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["alpha"]
    assert report["gates"]["alpha"]["failed_checks"] == [
        "mid360_pattern",
        "mujoco_world_asset",
        "saved_map_input",
        "native_localization_runtime",
    ]


def test_host_preflight_checks_native_assets_without_ros(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(
        monkeypatch,
        tmp_path,
        _spec(requirements=sim_diagnostics.NATIVE_MUJOCO_LOCALIZATION_HOST_REQUIREMENTS),
    )

    saved_map = tmp_path / "map.pcd"
    saved_map.write_text("pcd", encoding="utf-8")
    report = sim_diagnostics.host_preflight(
        required={"alpha"},
        platform_system="Linux",
        env={"LINGTU_DDS_DOMAIN_ID": "231", "MUJOCO_GL": "egl"},
        saved_map_pcd=saved_map,
        module_available=lambda name: name == "mujoco",
        path_exists=lambda _path: True,
    )

    checks = report["gates"]["alpha"]["checks"]
    assert report["ok"] is True
    assert checks["isolated_dds_domain"]["ok"] is True
    assert checks["native_localization_runtime"]["ok"] is True
    assert not any(name.startswith("ros2") for name in checks)


def test_host_preflight_marks_relocalization_gate_input_blocked_without_saved_map(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _install_specs(monkeypatch, tmp_path, next(spec for spec in sim_diagnostics.GATES if spec.name == "saved_map_relocalize"))

    report = sim_diagnostics.host_preflight(
        required={"saved_map_relocalize"},
        platform_system="Linux",
        env={"MUJOCO_GL": "egl"},
        dds_domain_id=77,
        module_available=lambda name: name == "mujoco",
        path_exists=lambda path: str(path).endswith(("mid360.npy", "industrial_park_scene.xml", "slamd", "slamctl", ".yaml", "lingtu_mujoco_sensor_publisher")),
    )

    gate = report["gates"]["saved_map_relocalize"]
    assert report["ok"] is False
    assert report["input_blocked_gates"] == ["saved_map_relocalize"]
    assert gate["status"] == "input_blocked"
    assert gate["checks"]["saved_map_input"]["ok"] is False
    assert report["current_host"]["LINGTU_DDS_DOMAIN_ID"] == "77"


def test_host_preflight_rejects_unknown_gate():
    with pytest.raises(ValueError, match="unknown required gate"):
        sim_diagnostics.host_preflight(required={"retired_gate"})


class _FakeProductControl:
    def __init__(self) -> None:
        self.calls: list[tuple[str, object]] = []

    def switch(self, product: str, *, state_dir: Path | None = None) -> dict:
        self.calls.append(("switch", product))
        return {"ok": True, "status": "active", "env": "sim", "product": product}

    def status(self, *, state_dir: Path | None = None) -> dict:
        self.calls.append(("status", state_dir))
        return {"ok": True, "status": "active", "env": "sim", "product": "nav"}

    def stop(self, *, state_dir: Path | None = None) -> dict:
        self.calls.append(("stop", state_dir))
        return {"ok": True, "status": "stopped", "env": "sim", "product": "nav"}


def test_product_diagnostics_uses_product_control(tmp_path: Path):
    control = _FakeProductControl()

    result = sim_diagnostics.run_product_diagnostics(
        "nav",
        collect=lambda: {"ok": True, "gates": {}},
        control=control,
        state_dir=tmp_path,
    )

    assert result["ok"] is True
    assert [name for name, _value in control.calls] == ["switch", "status", "stop"]
    assert result["product_control"]["stop"]["status"] == "stopped"


def test_product_diagnostics_stops_after_collection_failure(tmp_path: Path):
    control = _FakeProductControl()

    def fail() -> dict:
        raise RuntimeError("diagnostic collection failed")

    result = sim_diagnostics.run_product_diagnostics(
        "nav",
        collect=fail,
        control=control,
        state_dir=tmp_path,
    )

    assert result["ok"] is False
    assert result["error"] == "diagnostic collection failed"
    assert [name for name, _value in control.calls] == ["switch", "status", "stop"]


@pytest.mark.parametrize(
    ("arguments", "message"),
    [
        (["--required", "unknown"], "unknown required gate"),
        (
            [
                "--navigation-replay-deviation-report",
                "report.json",
                "--navigation-replay-deviation-topic-jsonl",
                "topics.jsonl",
            ],
            "cannot be combined",
        ),
        (
            ["--host-preflight", "--navigation-replay-deviation-topic-jsonl", "topics.jsonl"],
            "cannot be combined",
        ),
    ],
)
def test_main_rejects_invalid_cli_combinations(
    monkeypatch: pytest.MonkeyPatch, arguments: list[str], message: str
):
    monkeypatch.setattr(sys, "argv", ["sim.diagnostics", *arguments])

    with pytest.raises(SystemExit, match=message):
        sim_diagnostics.main()


def test_main_host_preflight_writes_json(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    output = tmp_path / "preflight.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "sim.diagnostics",
            "--host-preflight",
            "--required",
            "gateway_runtime_acceptance",
            "--json-out",
            str(output),
            "--strict",
        ],
    )

    assert sim_diagnostics.main() == 0
    report = json.loads(output.read_text(encoding="utf-8"))
    assert report["execution_mode"] == "host_preflight_only"
    assert report["runnable_gates"] == ["gateway_runtime_acceptance"]
    assert report["real_robot_motion"] is False
