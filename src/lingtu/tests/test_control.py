# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from lingtu.control import ExpectedProductMismatch, ProductControl
from lingtu.control import main as control_main
from lingtu.product_switch import SwitchRequest
from lingtu.run_plan import CURRENT_RUN_SCHEMA
from lingtu.systemd import ProcessReport


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
        return "nav"

    def stop_motion_and_session(self, current_product: str | None) -> None:
        self.stopped_products.append(current_product)

    def remove_session(self, plan) -> None:
        self.removed_plans.append(plan)


def test_product_control_help_names_the_field_product_selector(capsys) -> None:
    with pytest.raises(SystemExit) as exc_info:
        control_main(["--help"])

    assert exc_info.value.code == 0
    help_text = capsys.readouterr().out
    assert "[product]" in help_text
    assert "product               Field Product name" in help_text
    assert "[profile]" not in help_text


def test_product_control_compiles_env_once_per_operation(tmp_path: Path) -> None:
    runner = FakeRunner()
    control = ProductControl(
        runner,  # type: ignore[arg-type]
        process_env={
            "LINGTU_PRODUCT": "nav",
            "LINGTU_ENV": "real",
            "LINGTU_SESSION_ROOT": str(tmp_path),
        },
    )

    report = control.restart("slam")

    product, process_name, dry_run = runner.calls[0]
    assert product.product == "nav"
    assert product.env == "real"
    assert process_name == "slam"
    assert dry_run is False
    assert report.ok is True


def test_product_control_requires_explicit_active_product() -> None:
    control = ProductControl(FakeRunner(), env="real", process_env={})  # type: ignore[arg-type]

    try:
        control.restart("slam")
    except RuntimeError as exc:
        assert str(exc) == "LINGTU_PRODUCT is required for Product control"
    else:
        raise AssertionError("missing active Product must fail closed")


def test_product_control_reuses_one_compiled_product() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        process_env={
            "LINGTU_PRODUCT": "nav",
            "LINGTU_ENV": "real",
        },
    )

    assert control.resolve() is control.resolve()


def test_product_control_caches_explore_variants_separately() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        env="real",
        process_env={},
    )

    live = control.resolve("explore", product_variant="live")
    mapped = control.resolve("explore", product_variant="map")

    assert live is control.resolve("explore", product_variant="live")
    assert live is control.resolve("explore")
    assert mapped is control.resolve("explore", product_variant="map")
    assert live is not mapped
    assert live.product_variant == "live"
    assert mapped.product_variant == "map"
    assert live.fingerprint != mapped.fingerprint


@pytest.mark.parametrize(
    ("map_name", "expected_variant", "slam_mode", "requires_map"),
    (
        (None, "live", "mapping", False),
        ("yard", "map", "localization", True),
    ),
)
def test_product_control_selects_explore_variant_from_map_request(
    map_name: str | None,
    expected_variant: str,
    slam_mode: str,
    requires_map: bool,
) -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        env="real",
        process_env={},
    )

    report = control.switch(
        SwitchRequest(
            target_product="explore",
            current_product="teleop",
            map_name=map_name,
        ),
        dry_run=True,
    )

    assert report.ok is True
    assert report.run_plan["identity"]["product_variant"] == expected_variant
    assert report.run_plan["launch"]["session"]["slam_mode"] == slam_mode
    assert report.run_plan["launch"]["session"]["requires_map"] is requires_map


def test_product_control_fixes_sim_backend_from_process_environment() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        process_env={
            "LINGTU_PRODUCT": "nav",
            "LINGTU_ENV": "sim",
            "LINGTU_ENV_BACKEND": "mujoco_native",
        },
    )

    plan = control.resolve()

    assert plan.env == "sim"
    assert plan.process_control == "acceptance_runner"
    assert plan.host_config["_env_backend"] == "mujoco_native"


def test_product_control_sim_switch_is_plan_only_without_a_sim_executor() -> None:
    control = ProductControl(
        FakeRunner(),  # type: ignore[arg-type]
        env="sim",
        env_config={"backend": "mujoco_native"},
        process_env={},
    )
    request = SwitchRequest(target_product="nav")

    plan = control.switch(request, dry_run=True)

    assert plan.ok is True
    assert plan.status == "planned"
    assert plan.env == "sim"
    assert plan.run_plan["launch"]["controller"] == "acceptance_runner"
    with pytest.raises(RuntimeError, match="acceptance_runner simulation runner"):
        control.switch(request)


def test_product_control_does_not_own_compatibility_service_operations() -> None:
    legacy_methods = {name for name in dir(ProductControl) if name.startswith("legacy_")}

    assert legacy_methods == set()


def test_product_control_does_not_expose_run_plans_as_products() -> None:
    assert not hasattr(ProductControl, "apply_product")
    assert not hasattr(ProductControl, "stop_product")
    assert not hasattr(ProductControl, "restart_product")
    assert not hasattr(ProductControl, "quiesce_product")


@pytest.mark.parametrize("product_name", ("map", "nav"))
def test_product_control_rejects_direct_apply_for_field_product(product_name: str) -> None:
    runner = FakeRunner()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    product = control.resolve(product_name)

    try:
        control.apply_plan(product)
    except RuntimeError as exc:
        assert "requires ProductControl.switch" in str(exc)
    else:
        raise AssertionError("field Product must not bypass the switch transaction")

    assert runner.calls == []


def test_product_control_cli_plans_switch_without_field_side_effects(capsys) -> None:
    exit_code = control_main(
        [
            "switch",
            "teleop",
            "--current",
            "teleop",
            "--env",
            "real",
            "--dry-run",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["status"] == "planned"
    assert payload["target_product"] == "teleop"
    assert payload["env"] == "real"
    assert payload["phases"] == ["preflight"]


def test_product_control_owns_out_of_process_switch_submission(tmp_path: Path) -> None:
    invocations: list[list[str]] = []

    def run(command, **_kwargs):
        invocations.append(list(command))
        return SimpleNamespace(returncode=0, stdout="submitted", stderr="")

    control = ProductControl(
        env="real",
        process_env={
            "LINGTU_ENV": "real",
            "LINGTU_PYTHON": "python3",
            "HOME": "/home/sunrise",
            "USER": "sunrise",
        },
        operation_runner=run,
    )

    submission = control.submit_switch(
        SwitchRequest(
            target_product="nav",
            current_product="teleop",
            map_name="yard",
            relocalize=True,
            initial_pose=(1.0, 2.0, 0.5),
        ),
        request_id="web/request 42",
        state_dir=tmp_path,
    )

    command = invocations[0]
    assert command[:3] == ["sudo", "-n", "systemd-run"]
    assert "--unit=lingtu-product-control-switch.service" in command
    assert "--no-block" in command
    assert "LINGTU_SYSTEMD_UNIT" not in " ".join(command)
    assert "endpoint" not in " ".join(command).lower()
    assert command[command.index("--") + 1 :] == [
        "python3",
        "-m",
        "lingtu.control",
        "switch",
        "nav",
        "--env",
        "real",
        "--json",
        "--current",
        "teleop",
        "--map",
        "yard",
        "--relocalize",
        "--initial-pose",
        "1.0",
        "2.0",
        "0.5",
        "--state-dir",
        str(tmp_path.resolve()),
    ]
    assert submission["accepted"] is True
    assert submission["status"] == "submitted"
    assert submission["operation_id"] == "web-request-42"
    assert submission["product"] == "nav"
    assert submission["env"] == "real"


def _write_current(
    tmp_path: Path,
    run_plan_path: Path,
    *,
    fingerprint: str,
    product: str = "nav",
    product_variant: str | None = None,
) -> None:
    (tmp_path / "current.json").write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": product,
                "product_variant": product_variant,
                "env": "real",
                "run_plan_path": str(run_plan_path),
                "fingerprint": fingerprint,
            }
        ),
        encoding="utf-8",
    )


def test_product_control_reapplies_only_the_committed_current_plan(tmp_path: Path) -> None:
    runner = FakeRunner()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    report = control.reapply_current(state_dir=tmp_path)

    assert report.ok is True
    assert runner.calls == [(plan, "apply", False)]


def test_product_control_rejects_current_variant_mismatch(tmp_path: Path) -> None:
    runner = FakeRunner()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("explore", product_variant="map")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(
        tmp_path,
        run_plan_path,
        fingerprint=plan.fingerprint,
        product="explore",
        product_variant="live",
    )

    with pytest.raises(RuntimeError, match="current Product variant"):
        control.reapply_current(state_dir=tmp_path)

    assert runner.calls == []


def test_product_control_restarts_only_the_committed_current_plan(tmp_path: Path) -> None:
    runner = FakeRunner()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    report = control.restart_current("slam", state_dir=tmp_path)

    assert report.ok is True
    assert runner.calls == [(plan, "slam", False)]


def test_product_control_quiesces_without_removing_current_run(tmp_path: Path) -> None:
    runner = FakeRunner()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    report = control.quiesce_current(state_dir=tmp_path)

    assert report.ok is True
    assert runner.calls == [(plan, "quiesce", False)]
    assert (tmp_path / "current.json").is_file()


def test_product_control_stops_only_the_committed_current_plan(tmp_path: Path) -> None:
    runner = FakeRunner()
    backend = FakeStopBackend()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    report = control.stop_current(
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
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    with pytest.raises(ExpectedProductMismatch) as exc_info:
        control.stop_current(
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
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("explore")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(
        tmp_path,
        run_plan_path,
        fingerprint=plan.fingerprint,
        product="explore",
        product_variant=plan.product_variant,
    )

    report = control.stop_current(
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
    monkeypatch,
    tmp_path: Path,
) -> None:
    events: list[str] = []

    class OrderedRunner(FakeRunner):
        def stop(self, product, *, dry_run: bool = False):
            events.append("systemd.stop")
            return super().stop(product, dry_run=dry_run)

    class OrderedBackend(FakeStopBackend):
        def stop_motion_and_session(self, current_product: str | None) -> None:
            events.append("backend.stop_motion_and_session")
            super().stop_motion_and_session(current_product)

        def remove_session(self, plan) -> None:
            events.append("backend.remove_session")
            super().remove_session(plan)

    runner = OrderedRunner()
    backend = OrderedBackend()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = SimpleNamespace(product="nav", env="real")
    monkeypatch.setattr(control, "_current_plan", lambda _root: plan)
    (tmp_path / "current.json").write_text("{}", encoding="utf-8")

    report = control.stop_current(
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert events == [
        "backend.stop_motion_and_session",
        "systemd.stop",
        "backend.remove_session",
    ]
    assert backend.stopped_products == ["nav"]


def test_product_control_does_not_stop_systemd_when_motion_stop_fails(
    monkeypatch,
    tmp_path: Path,
) -> None:
    events: list[str] = []

    class FailingBackend(FakeStopBackend):
        def stop_motion_and_session(self, current_product: str | None) -> None:
            events.append("backend.stop_motion_and_session")
            raise RuntimeError("motion stop was not confirmed")

    runner = FakeRunner()
    backend = FailingBackend()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = SimpleNamespace(product="nav", env="real")
    monkeypatch.setattr(control, "_current_plan", lambda _root: plan)
    current_path = tmp_path / "current.json"
    current_path.write_text("{}", encoding="utf-8")

    with pytest.raises(RuntimeError, match="motion stop was not confirmed"):
        control.stop_current(
            backend=backend,  # type: ignore[arg-type]
            state_dir=tmp_path,
        )

    assert events == ["backend.stop_motion_and_session"]
    assert runner.calls == []
    assert backend.removed_plans == []
    assert current_path.is_file()


def test_product_control_removes_session_after_successful_stop_current(
    monkeypatch,
    tmp_path: Path,
) -> None:
    runner = FakeRunner()
    backend = FakeStopBackend()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = SimpleNamespace(product="nav", env="real")
    monkeypatch.setattr(control, "_current_plan", lambda _root: plan)
    (tmp_path / "current.json").write_text("{}", encoding="utf-8")

    report = control.stop_current(
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert report.ok is True
    assert runner.calls == [(plan, "stop", False)]
    assert backend.removed_plans == [plan]
    assert not (tmp_path / "current.json").exists()


def test_product_control_keeps_session_when_stop_current_fails(
    monkeypatch,
    tmp_path: Path,
) -> None:
    class FailingRunner(FakeRunner):
        def stop(self, product, *, dry_run: bool = False):
            self.calls.append((product, "stop", dry_run))
            raise RuntimeError("stop failed")

    runner = FailingRunner()
    backend = FakeStopBackend()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = SimpleNamespace(product="nav", env="real")
    monkeypatch.setattr(control, "_current_plan", lambda _root: plan)

    with pytest.raises(RuntimeError, match="stop failed"):
        control.stop_current(
            backend=backend,  # type: ignore[arg-type]
            state_dir=tmp_path,
        )

    assert backend.removed_plans == []


def test_product_control_reapply_rejects_uncommitted_plan(tmp_path: Path) -> None:
    runner = FakeRunner()
    control = ProductControl(runner, env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint="0" * 64)

    with pytest.raises(RuntimeError, match="current RunPlan fingerprint"):
        control.reapply_current(state_dir=tmp_path)

    assert runner.calls == []


def test_product_control_cli_reapplies_the_current_plan_in_dry_run(
    tmp_path: Path,
    capsys,
) -> None:
    control = ProductControl(FakeRunner(), env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    exit_code = control_main(
        [
            "reapply",
            "--state-dir",
            str(tmp_path),
            "--dry-run",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["action"] == "apply"
    assert payload["status"] == "planned"
    assert payload["product"] == "nav"


def test_product_control_cli_stop_reports_expected_product_mismatch(
    tmp_path: Path,
    capsys,
) -> None:
    control = ProductControl(FakeRunner(), env="real", process_env={})  # type: ignore[arg-type]
    plan = control.resolve("nav")
    run_plan_path = plan.write(tmp_path / "plan.json")
    _write_current(tmp_path, run_plan_path, fingerprint=plan.fingerprint)

    exit_code = control_main(
        [
            "stop",
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


def test_product_control_stop_session_uses_switch_safety_boundary(tmp_path: Path) -> None:
    backend = FakeStopBackend()
    control = ProductControl(FakeRunner(), env="real", process_env={})  # type: ignore[arg-type]

    payload = control.stop_session(
        backend=backend,  # type: ignore[arg-type]
        state_dir=tmp_path,
    )

    assert backend.stopped_products == ["nav"]
    assert payload == {
        "ok": True,
        "action": "stop-session",
        "status": "stopped",
    }
