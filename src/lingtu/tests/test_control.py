from __future__ import annotations

import copy
import json
from functools import lru_cache
from pathlib import Path

import pytest
from sim.catalog import CatalogResolver

from lingtu.control import ExpectedProductMismatch, ProductControl
from lingtu.control import main as control_main
from lingtu.real.systemd import ProcessReport
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.switch_contracts import SwitchRequest
from runtime.graph import (
    ProcessArtifact,
    ProcessCommand,
    ProcessReadiness,
    ProcessSpec,
)


class FakeRunner:
    def __init__(self) -> None:
        self.calls = []

    def restart(self, product, process_name: str, *, dry_run: bool = False):
        self.calls.append((product, process_name, dry_run))
        return ProcessReport(
            product=product.product,
            env=product.env,
            action="restart",
            ok=True,
            status="active",
        )

    def apply(self, product, *, dry_run: bool = False):
        self.calls.append((product, "apply", dry_run))
        return ProcessReport(
            product=product.product,
            env=product.env,
            action="apply",
            ok=True,
            status="active",
        )

    def apply_deferred(self, product, *, dry_run: bool = False):
        return self.apply(product, dry_run=dry_run)

    def stop(self, product, *, dry_run: bool = False):
        self.calls.append((product, "stop", dry_run))
        return ProcessReport(
            product=product.product,
            env=product.env,
            action="stop",
            ok=True,
            status="stopped",
        )

    def quiesce(self, product, *, dry_run: bool = False):
        self.calls.append((product, "quiesce", dry_run))
        return ProcessReport(
            product=product.product,
            env=product.env,
            action="quiesce",
            ok=True,
            status="stopped",
        )


class FakeStopBackend:
    def __init__(self) -> None:
        self.stopped_products: list[str | None] = []
        self.removed_plans = []

    def current_product(self) -> str:
        raise AssertionError("current Product must come from the committed RunPlan")

    def stop_motion(self, current_product: str | None) -> None:
        self.stopped_products.append(current_product)

    def remove_session(self, plan) -> None:
        self.removed_plans.append(plan)


class FakeSimulationRunner:
    def __init__(self) -> None:
        self.calls: list[tuple[str, Path]] = []
        self.product_session_ids: list[str] = []

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ):
        assert timeout_s is None
        self.calls.append(("apply", run_plan_path))
        self.product_session_ids.append(product_session_id)
        plan = RunPlan.load(run_plan_path)
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            ok=True,
            status="active",
        )

    def quiesce(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ):
        assert timeout_s is None
        self.calls.append(("quiesce", run_plan_path))
        self.product_session_ids.append(product_session_id)
        plan = RunPlan.load(run_plan_path)
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="quiesce",
            ok=True,
            status="stopped",
        )

    def stop(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ):
        assert timeout_s is None
        self.calls.append(("stop", run_plan_path))
        self.product_session_ids.append(product_session_id)
        plan = RunPlan.load(run_plan_path)
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action="stop",
            ok=True,
            status="stopped",
        )


def test_product_control_help_names_the_field_product_selector(capsys) -> None:
    with pytest.raises(SystemExit) as exc_info:
        control_main(["--help"])

    assert exc_info.value.code == 0
    help_text = capsys.readouterr().out
    assert "{switch,status,stop}" in help_text
    assert "[product]" in help_text
    assert "product               Field Product name" in help_text
    assert "[profile]" not in help_text
    assert "--local-planner {cmu,scan}" in help_text
    assert "--backend" in help_text
    assert "--viewer" in help_text
    assert "stop-session" not in help_text
    assert "reapply" not in help_text
    assert "quiesce" not in help_text
    assert "restart" not in help_text


@pytest.mark.parametrize("action", ("reapply", "quiesce", "restart"))
def test_product_control_rejects_retired_actions(action: str) -> None:
    with pytest.raises(SystemExit) as exc_info:
        control_main([action])

    assert exc_info.value.code == 2


def test_product_control_requires_explicit_active_product() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )

    try:
        control._resolve()
    except RuntimeError as exc:
        assert str(exc) == "LINGTU_PRODUCT is required for Product control"
    else:
        raise AssertionError("missing active Product must fail closed")


def test_product_control_requires_robot_to_resolve_or_switch(tmp_path: Path) -> None:
    control = ProductControl(env="sim", process_env={})

    with pytest.raises(ValueError, match=r"^Robot is required$"):
        control._resolve("teleop")
    with pytest.raises(ValueError, match=r"^Robot is required$"):
        control.switch("teleop", state_dir=tmp_path, dry_run=True)


def test_product_control_resolves_the_same_product_deterministically() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        process_env={
            "LINGTU_PRODUCT": "nav",
            "LINGTU_ENV": "real",
        },
    )

    assert control._resolve() == control._resolve()


def test_product_control_selects_scan_in_sim_without_changing_product_identity() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
    )

    plan = control._resolve("nav", local_planner="scan")
    teleop = control._resolve("teleop")

    assert plan.product == "nav"
    assert plan.product_variant is None
    assert plan.native_nav["local_planner"] == "scan"
    assert plan.native_process_environment["LINGTU_NAV_LOCAL_PLANNER_BACKEND"] == "scan"
    assert teleop.product == "teleop"
    assert teleop.native_nav["local_planner"] == "cmu"


def test_product_control_rejects_unqualified_scan_for_teleop_avoid() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
    )

    with pytest.raises(
        ValueError,
        match="does not support local planner 'scan'",
    ):
        control._resolve("teleop_avoid", local_planner="scan")


def test_product_control_rejects_unqualified_scan_on_real_robot() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )

    with pytest.raises(ValueError, match="has not qualified local planner 'scan'"):
        control._resolve("nav", local_planner="scan")


def test_product_control_resolves_explore_variants_separately() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )

    live = control._resolve("explore", product_variant="live")
    mapped = control._resolve("explore", product_variant="map")

    assert live == control._resolve("explore", product_variant="live")
    assert live == control._resolve("explore")
    assert mapped == control._resolve("explore", product_variant="map")
    assert live.product_variant == "live"
    assert mapped.product_variant == "map"
    assert live != mapped


@pytest.mark.parametrize(
    ("map_name", "expected_variant"),
    (
        (None, "live"),
        ("yard", "map"),
    ),
)
def test_product_control_selects_explore_variant_from_map_request(
    map_name: str | None,
    expected_variant: str,
) -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )

    report = control._switch(
        SwitchRequest(
            target_product="explore",
            map_name=map_name,
        ),
        dry_run=True,
    )

    assert report.ok is True
    assert report.product_variant == expected_variant


def test_product_control_resolves_robot_and_env_together() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        process_env={
            "LINGTU_PRODUCT": "teleop",
            "LINGTU_ENV": "sim",
        },
    )

    plan = control._resolve()

    assert plan.env == "sim"
    assert plan.robot == "doso/thunder_v4"
    assert plan.process_control == "subprocess"
    assert plan.host_config["_env_backend"] == "mujoco"


def test_product_control_public_switch_hides_the_internal_plan(tmp_path: Path) -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        env="sim",
        process_env={"LINGTU_SESSION_ROOT": str(tmp_path)},
    )

    result = control.switch(
        "teleop",
        state_dir=tmp_path,
        dry_run=True,
    )

    assert result == {
        "ok": True,
        "status": "planned",
        "robot": "doso/thunder_v4",
        "env": "sim",
        "product": "teleop",
        "product_variant": None,
        "local_planner": "cmu",
        "product_session_id": None,
        "phases": ["preflight"],
        "cleanup": [],
        "readiness": None,
        "error": None,
    }
    assert not (tmp_path / "current.json").exists()
    assert list(tmp_path.glob("plan-*.json")) == []


def test_product_control_status_is_read_only_when_stopped(tmp_path: Path) -> None:
    control = ProductControl(env="sim", process_env={})

    before = list(tmp_path.iterdir())
    first = control.status(state_dir=tmp_path)
    second = control.status(state_dir=tmp_path)

    assert first == second == {
        "ok": True,
        "status": "stopped",
        "robot": "",
        "env": "sim",
        "product": None,
        "local_planner": None,
        "error": None,
    }
    assert list(tmp_path.iterdir()) == before


def test_product_control_public_stop_returns_simple_state(tmp_path: Path) -> None:
    simulation_runner = FakeSimulationRunner()
    control = ProductControl(
        robot="doso/thunder_v4",
        env="sim",
        process_env={},
        simulation_runner=simulation_runner,
    )
    control._switch(SwitchRequest(target_product="teleop"), state_dir=tmp_path)

    result = control.stop(state_dir=tmp_path)

    assert result == {
        "ok": True,
        "status": "stopped",
        "robot": "doso/thunder_v4",
        "env": "sim",
        "product": "teleop",
        "stopped": [],
        "stop_evidence": {},
        "error": None,
    }
    assert not (tmp_path / "current.json").exists()


def test_product_control_public_stop_is_idempotent(tmp_path: Path) -> None:
    control = ProductControl(env="sim", process_env={})

    assert control.stop(state_dir=tmp_path) == {
        "ok": True,
        "status": "stopped",
        "robot": "",
        "env": "sim",
        "product": None,
        "error": None,
    }


def test_product_control_executes_published_mujoco_teleop_run_plan(
    tmp_path: Path,
) -> None:
    simulation_runner = FakeSimulationRunner()
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={"LINGTU_SESSION_ROOT": str(tmp_path)},
        simulation_runner=simulation_runner,
    )

    report = control._switch(
        SwitchRequest(target_product="teleop"),
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert report.status == "active"
    assert len(simulation_runner.calls) == 1
    action, run_plan_path = simulation_runner.calls[0]
    assert action == "apply"
    assert report.product_session_id is not None
    assert run_plan_path == tmp_path / f"plan-{report.product_session_id}.json"
    assert RunPlan.load(run_plan_path).product == "teleop"
    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert current["product"] == "teleop"
    assert current["product_session_id"] == simulation_runner.product_session_ids[0]


def test_product_control_switches_and_stops_exact_mujoco_explore_live_plan(
    tmp_path: Path,
) -> None:
    simulation_runner = FakeSimulationRunner()
    systemd_runner = FakeRunner()
    control = ProductControl(
        systemd_runner,  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={"LINGTU_SESSION_ROOT": str(tmp_path)},
        simulation_runner=simulation_runner,
    )

    switched = control._switch(
        SwitchRequest(target_product="explore"),
        state_dir=tmp_path,
    )
    assert switched.product_session_id is not None
    run_plan_path = tmp_path / f"plan-{switched.product_session_id}.json"
    plan = RunPlan.load(run_plan_path)

    assert switched.ok is True
    assert switched.status == "active"
    assert plan.product == "explore"
    assert plan.product_variant == "live"
    assert plan.process("explore").name == "explore_runtime"
    assert simulation_runner.calls == [("apply", run_plan_path)]
    assert systemd_runner.calls == []
    product_session_id = simulation_runner.product_session_ids[0]

    stopped = control._stop(
        state_dir=tmp_path,
        expected_product="explore",
    )

    assert stopped.ok is True
    assert stopped.status == "stopped"
    assert simulation_runner.calls == [
        ("apply", run_plan_path),
        ("stop", run_plan_path),
    ]
    assert simulation_runner.product_session_ids == [product_session_id, product_session_id]
    assert not (tmp_path / "current.json").exists()


def test_product_control_sim_compare_and_stop_rejects_replaced_same_product_session(
    tmp_path: Path,
) -> None:
    simulation_runner = FakeSimulationRunner()
    control = ProductControl(
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
        simulation_runner=simulation_runner,
    )
    control._switch(SwitchRequest(target_product="teleop"), state_dir=tmp_path)
    first_session = simulation_runner.product_session_ids[-1]
    control._switch(SwitchRequest(target_product="teleop"), state_dir=tmp_path)
    second_session = simulation_runner.product_session_ids[-1]

    assert first_session != second_session
    with pytest.raises(RuntimeError, match="Product session"):
        control._stop(
            state_dir=tmp_path,
            expected_product="teleop",
            expected_product_session_id=first_session,
        )

    assert simulation_runner.calls[-1][0] == "apply"
    current = json.loads((tmp_path / "current.json").read_text(encoding="utf-8"))
    assert current["product_session_id"] == second_session


def test_product_control_routes_exact_mujoco_map_plan_through_simulation_runner(
    tmp_path: Path,
) -> None:
    simulation_runner = FakeSimulationRunner()
    systemd_runner = FakeRunner()
    control = ProductControl(
        systemd_runner,  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={"LINGTU_SESSION_ROOT": str(tmp_path)},
        simulation_runner=simulation_runner,
    )

    switched = control._switch(
        SwitchRequest(target_product="map"),
        state_dir=tmp_path,
    )
    assert switched.product_session_id is not None
    run_plan_path = tmp_path / f"plan-{switched.product_session_id}.json"
    plan = RunPlan.load(run_plan_path)

    assert switched.ok is True
    assert switched.status == "active"
    assert plan.product == "map"
    assert plan.lifecycle["slam_mode"] == "mapping"
    assert plan.process("camera") is not plan.process("lidar")
    assert plan.process("camera").name == "camera_publisher"
    assert simulation_runner.calls == [("apply", run_plan_path)]
    assert systemd_runner.calls == []
    product_session_id = simulation_runner.product_session_ids[0]

    stopped = control._stop(
        state_dir=tmp_path,
        expected_product="map",
    )

    assert stopped.ok is True
    assert stopped.status == "stopped"
    assert simulation_runner.calls == [
        ("apply", run_plan_path),
        ("stop", run_plan_path),
    ]
    assert simulation_runner.product_session_ids == [
        product_session_id,
        product_session_id,
    ]
    assert not (tmp_path / "current.json").exists()


def test_product_control_exposes_only_product_operations() -> None:
    internal = {
        "resolve",
        "reapply_current",
        "quiesce_current",
        "restart_current",
        "stop_current",
        "stop_session",
        "submit_switch",
        "write_plan",
        "apply",
        "apply_plan",
        "stop_before_start",
        "quiesce_plan",
        "restart",
        "restart_process",
    }

    public = {
        name
        for name, value in ProductControl.__dict__.items()
        if callable(value) and not name.startswith("_")
    }

    assert internal.isdisjoint(dir(ProductControl))
    assert public == {"switch", "status", "stop"}


def test_product_control_does_not_expose_run_plans_as_products() -> None:
    assert not hasattr(ProductControl, "apply_product")
    assert not hasattr(ProductControl, "stop_product")
    assert not hasattr(ProductControl, "restart_product")
    assert not hasattr(ProductControl, "quiesce_product")


def test_product_control_cli_plans_switch_without_field_side_effects(capsys) -> None:
    exit_code = control_main(
        [
            "switch",
            "teleop",
            "--robot",
            "unitree/go2",
            "--env",
            "real",
            "--dry-run",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload == {
        "ok": True,
        "status": "planned",
        "robot": "unitree/go2",
        "env": "real",
        "product": "teleop",
        "product_variant": None,
        "local_planner": "cmu",
        "product_session_id": None,
        "phases": ["preflight"],
        "cleanup": [],
        "readiness": None,
        "error": None,
    }


def test_product_control_cli_accepts_explicit_sim_backend(capsys) -> None:
    exit_code = control_main(
        [
            "switch",
            "teleop_avoid",
            "--robot",
            "doso/thunder_v4",
            "--env",
            "sim",
            "--backend",
            "mujoco",
            "--dry-run",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["ok"] is True
    assert payload["product"] == "teleop_avoid"
    assert payload["env"] == "sim"


def test_product_control_cli_human_output_names_robot(capsys) -> None:
    exit_code = control_main(
        [
            "switch",
            "teleop",
            "--robot",
            "unitree/go2",
            "--env",
            "real",
            "--dry-run",
        ]
    )

    assert exit_code == 0
    assert capsys.readouterr().out.strip() == (
        "planned: teleop on unitree/go2 in real"
    )


def test_product_control_cli_rejects_caller_current_product() -> None:
    with pytest.raises(SystemExit) as exc_info:
        control_main(
            [
                "switch",
                "teleop",
                "--robot",
                "unitree/go2",
                "--current",
                "teleop",
                "--env",
                "real",
                "--dry-run",
            ]
        )

    assert exc_info.value.code == 2


def _write_current(
    tmp_path: Path,
    run_plan_path: Path,
    *,
    product: str = "nav",
    product_variant: str | None = None,
    env: str = "real",
    product_session_id: str = "1" * 32,
) -> None:
    (tmp_path / "current.json").write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": product,
                "product_variant": product_variant,
                "env": env,
                "run_plan_path": str(run_plan_path),
                "product_session_id": product_session_id,
            }
        ),
        encoding="utf-8",
    )


def test_product_control_status_reads_current_without_writing(tmp_path: Path) -> None:
    control = ProductControl(robot="unitree/go2", env="real", process_env={})
    plan = control._resolve("teleop")
    plan_path = plan.write(tmp_path / "plan.json")
    _write_current(
        tmp_path,
        plan_path,
        product="teleop",
    )
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}

    result = control.status(state_dir=tmp_path)

    assert result == {
        "ok": True,
        "status": "active",
        "robot": "unitree/go2",
        "env": "real",
        "product": "teleop",
        "local_planner": "cmu",
        "error": None,
    }
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


def test_product_control_rejects_current_for_another_robot(tmp_path: Path) -> None:
    control = ProductControl(robot="unitree/go2", env="real", process_env={})
    plan = control._resolve("teleop")
    payload = plan.as_dict()
    payload["identity"]["robot"] = "another/robot"
    other_robot_plan = RunPlan.from_dict(payload)
    plan_path = other_robot_plan.write(tmp_path / "plan.json")
    _write_current(
        tmp_path,
        plan_path,
        product="teleop",
    )

    with pytest.raises(RuntimeError, match="belongs to Robot"):
        control.status(state_dir=tmp_path)

    assert control.robot == "unitree/go2"


def test_real_current_plan_returns_the_exact_product_session_id(tmp_path: Path) -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, plan_path)

    current, exact_path, product_session_id = control._current_plan_and_path(tmp_path)

    assert current == plan
    assert exact_path == plan_path.resolve()
    assert product_session_id == "1" * 32


REPO_ROOT = Path(__file__).resolve().parents[3]
SIMULATION_SESSION = "sim/scenarios/catalog/thunder_omni_contract/session.yaml"


@lru_cache(maxsize=1)
def _resolved_simulation_snapshot() -> dict[str, object]:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(REPO_ROOT / SIMULATION_SESSION)
    return {
        "schema": "lingtu.run_plan.simulation.v1",
        "session_source": SIMULATION_SESSION,
        "session": resolved.session,
        "physics_plan": resolved.physics_plan,
        "visual_plan": resolved.visual_plan,
        "sensor_plan": resolved.sensor_plan,
        "control_plan": resolved.control_plan,
        "transport_intent": resolved.transport_intent,
        "scenario_plan": resolved.scenario_plan,
    }


def _simulation_snapshot() -> dict[str, object]:
    return copy.deepcopy(_resolved_simulation_snapshot())


def _sim_subprocess_plan(product: str = "teleop_avoid") -> RunPlan:
    artifact = ProcessArtifact(
        path="sim/scripts/mujoco/product_acceptance.py",
    )
    process = ProcessSpec(
        name="sim_runtime",
        manager="direct",
        target=f"{product}-runtime",
        order=10,
        timeout_s=5,
        lifecycle="mode",
        command=ProcessCommand(
            argv=("python", artifact.path),
            cwd=".",
            env=(),
            artifact=artifact,
            readiness=ProcessReadiness("process"),
        ),
        provides=("sim_runtime",),
    )
    return RunPlan.create(
        product=product,
        env="sim",
        robot="doso/thunder_v4",
        process_control="subprocess",
        modules=(),
        processes=(process,),
        available_processes=(process,),
        stop_before_start=(process.target,),
        contracts=(f"lingtu.product.{product}.v1",),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={"product": product},
        simulation=_simulation_snapshot(),
        native_nav={
            "control_mode": "test",
            "global_planner": "astar",
            "publish_cmd_vel": False,
            "check_obstacle": False,
            "use_traversability_cost": False,
            "allow_teleop_takeover": False,
            "teleop_local_planner": False,
        },
    )


def test_product_control_stops_only_the_committed_current_plan(tmp_path: Path) -> None:
    runner = FakeRunner()
    backend = FakeStopBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)

    report = control._stop(
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert runner.calls == [(plan, "stop", False)]
    assert backend.removed_plans == [plan]
    assert not (tmp_path / "current.json").exists()


def test_product_control_expected_product_mismatch_has_no_stop_side_effects(
    tmp_path: Path,
) -> None:
    runner = FakeRunner()
    backend = FakeStopBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)

    with pytest.raises(ExpectedProductMismatch) as exc_info:
        control._stop(
            expected_product="explore",
            backend=backend,  # type: ignore[arg-type]
            state_dir=tmp_path,
        )

    assert exc_info.value.reason == "current_product_mismatch"
    assert exc_info.value.expected_product == "explore"
    assert exc_info.value.current_product == "nav"
    assert runner.calls == []
    assert backend.stopped_products == []
    assert backend.removed_plans == []
    assert (tmp_path / "current.json").is_file()


def test_product_control_expected_product_match_stops_current_plan(
    tmp_path: Path,
) -> None:
    runner = FakeRunner()
    backend = FakeStopBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("explore")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(
        tmp_path,
        run_plan_path,
        product="explore",
        product_variant=plan.product_variant,
    )

    report = control._stop(
        expected_product="explore",
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert runner.calls == [(plan, "stop", False)]
    assert backend.stopped_products == ["explore"]
    assert backend.removed_plans == [plan]
    assert not (tmp_path / "current.json").exists()


def test_product_control_stops_motion_before_systemd_and_session_cleanup(
    tmp_path: Path,
) -> None:
    events: list[str] = []

    class OrderedRunner(FakeRunner):
        def stop(self, product, *, dry_run: bool = False):
            events.append("systemd.stop")
            return super().stop(product, dry_run=dry_run)

    class OrderedBackend(FakeStopBackend):
        def stop_motion(self, current_product: str | None) -> None:
            events.append("backend.stop_motion")
            super().stop_motion(current_product)

        def remove_session(self, plan) -> None:
            events.append("backend.remove_session")
            super().remove_session(plan)

    runner = OrderedRunner()
    backend = OrderedBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)

    report = control._stop(
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert events == [
        "backend.stop_motion",
        "systemd.stop",
        "backend.remove_session",
    ]
    assert backend.stopped_products == ["nav"]


def test_product_control_does_not_stop_systemd_when_motion_stop_fails(
    tmp_path: Path,
) -> None:
    events: list[str] = []

    class FailingBackend(FakeStopBackend):
        def stop_motion(self, current_product: str | None) -> None:
            events.append("backend.stop_motion")
            raise RuntimeError("motion stop was not confirmed")

    runner = FakeRunner()
    backend = FailingBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)
    current_path = tmp_path / "current.json"

    with pytest.raises(RuntimeError, match="motion stop was not confirmed"):
        control._stop(
            backend=backend,  # type: ignore[arg-type]
            state_dir=tmp_path,
        )

    assert events == ["backend.stop_motion"]
    assert runner.calls == []
    assert backend.removed_plans == []
    assert current_path.is_file()


def test_product_control_removes_session_after_successful_stop_current(
    tmp_path: Path,
) -> None:
    runner = FakeRunner()
    backend = FakeStopBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)

    report = control._stop(
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert runner.calls == [(plan, "stop", False)]
    assert backend.removed_plans == [plan]
    assert not (tmp_path / "current.json").exists()


def test_product_control_keeps_session_when_stop_current_fails(
    tmp_path: Path,
) -> None:
    class FailingRunner(FakeRunner):
        def stop(self, product, *, dry_run: bool = False):
            self.calls.append((product, "stop", dry_run))
            raise RuntimeError("stop failed")

    runner = FailingRunner()
    backend = FakeStopBackend()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)

    with pytest.raises(RuntimeError, match="stop failed"):
        control._stop(
            backend=backend,  # type: ignore[arg-type]
            state_dir=tmp_path,
        )

    assert backend.removed_plans == []


def test_product_control_sim_stop_current_rejects_override_without_deleting_external(
    tmp_path: Path,
) -> None:
    plan = _sim_subprocess_plan()
    external_root = (tmp_path.parent / f"{tmp_path.name}-external-stop").resolve()
    external_root.mkdir()
    external_plan_path = plan.write(external_root / f"plan-{'1' * 32}.json")
    external_current = external_root / "current.json"
    _write_current(
        external_root,
        external_plan_path,
        product=plan.product,
        product_variant=plan.product_variant,
        env="sim",
    )
    before = external_current.read_bytes()
    simulation_runner = FakeSimulationRunner()
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="doso/thunder_v4",
        simulation_runner=simulation_runner,
        env="sim",
        process_env={"LINGTU_CURRENT_FILE": str(external_current)},
    )

    with pytest.raises(RuntimeError, match="LINGTU_CURRENT_FILE"):
        control._stop(state_dir=tmp_path)

    assert simulation_runner.calls == []
    assert external_current.read_bytes() == before
    assert not (tmp_path / "current.json").exists()


def test_product_control_cli_stop_reports_expected_product_mismatch(
    tmp_path: Path,
    capsys,
) -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        robot="unitree/go2",
        env="real",
        process_env={},
    )
    plan = control._resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path)

    exit_code = control_main(
        [
            "stop",
            "--robot",
            "unitree/go2",
            "--expected-product",
            "explore",
            "--state-dir",
            str(tmp_path),
            "--dry-run",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code != 0
    assert payload == {
        "ok": False,
        "reason": "current_product_mismatch",
        "error": "expected current Product 'explore', found 'nav'",
        "expected_product": "explore",
        "current_product": "nav",
    }
    assert (tmp_path / "current.json").is_file()
