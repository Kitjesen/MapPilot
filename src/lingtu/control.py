"""Switch, report, and stop one Product for one Robot in real or sim."""

from __future__ import annotations

import argparse
import json
import os
from collections.abc import Iterator, Mapping
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any

import lingtu.sim.switch as sim_switch
from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.native_nav import local_planner_name
from lingtu.product_lock import (
    CURRENT_RUN_FILE_NAME,
    ProductControlLock,
    resolve_current_run_path,
)
from lingtu.products import product_name
from lingtu.real.backend import FieldBackend, SwitchBackend
from lingtu.real.switch import execute_switch
from lingtu.real.systemd import SystemdRunner
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.sim.daemon import ensure_sim_supervisor
from lingtu.switch_contracts import (
    ProcessFailed,
    ProcessReport,
    SwitchFailed,
    SwitchReport,
    SwitchRequest,
    is_product_session_id,
)

_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_SIM_SUPERVISOR_START_TIMEOUT_S = 30.0


class _CurrentProductNotFound(RuntimeError):
    pass


class ExpectedProductMismatch(RuntimeError):
    """The committed Product did not match a guarded lifecycle request."""

    reason = "current_product_mismatch"

    def __init__(self, *, expected_product: str, current_product: str) -> None:
        self.expected_product = expected_product
        self.current_product = current_product
        super().__init__(
            f"expected current Product {expected_product!r}, found {current_product!r}"
        )


class ProductControl:
    """Public Product lifecycle for one fixed Robot and Env."""

    def __init__(
        self,
        runner: SystemdRunner | None = None,
        *,
        robot: str | None = None,
        env: str | None = None,
        env_config: Mapping[str, Any] | None = None,
        process_env: Mapping[str, str] | None = None,
        simulation_runner: sim_switch.SimSwitchRunner | None = None,
    ) -> None:
        self._runner_impl = runner
        self._simulation_runner_impl = simulation_runner
        self._process_env = process_env if process_env is not None else os.environ
        self.env = _env_name(env or self._process_env.get("LINGTU_ENV") or "real")
        self.robot = str(
            robot or self._process_env.get("LINGTU_ROBOT") or ""
        ).strip()
        self._env_config = dict(env_config or {})
        if self.env == "sim" and "backend" not in self._env_config:
            env_backend = str(
                self._process_env.get("LINGTU_ENV_BACKEND")
                or self._process_env.get("LINGTU_SIM_BACKEND")
                or ""
            ).strip()
            if env_backend:
                self._env_config["backend"] = env_backend
        self._mutation_state_dir: ContextVar[Path | None] = ContextVar(
            f"lingtu_product_control_state_dir_{id(self)}",
            default=None,
        )

    @contextmanager
    def _mutation(
        self,
        state_dir: str | Path | None = None,
    ) -> Iterator[Path]:
        """Serialize lifecycle effects across callers and nested switch callbacks."""

        inherited = self._mutation_state_dir.get()
        lock = ProductControlLock(
            state_dir if state_dir is not None else inherited,
            environment=self._process_env,
        )
        token = self._mutation_state_dir.set(lock.state_dir)
        try:
            with lock:
                yield lock.state_dir
        finally:
            self._mutation_state_dir.reset(token)

    def _resolve(
        self,
        product: str | None = None,
        *,
        product_variant: str | None = None,
        local_planner: str | None = None,
        parameter_overrides: Mapping[str, Any] | None = None,
    ) -> RunPlan:
        """Resolve one Product inside this control plane's fixed Env."""

        requested_product = str(
            product or self._process_env.get("LINGTU_PRODUCT") or ""
        ).strip()
        if not requested_product:
            raise RuntimeError("LINGTU_PRODUCT is required for Product control")
        selected_product = product_name(requested_product)
        selected_variant = (
            str(product_variant).strip() if product_variant is not None else None
        )
        selected_local_planner = (
            local_planner_name(local_planner) if local_planner is not None else None
        )
        plan = compile_run_plan(
            selected_product,
            self.env,
            robot=self.robot or None,
            local_planner=selected_local_planner,
            env_config=self._env_config or None,
            product_variant=selected_variant,
            parameter_overrides=parameter_overrides,
        )
        self._require_plan_robot(plan)
        return plan

    def switch(
        self,
        product: str,
        *,
        map_name: str | None = None,
        relocalize: bool = True,
        initial_pose: tuple[float, float, float] | None = None,
        local_planner: str | None = None,
        parameter_overrides: Mapping[str, Any] | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        """Switch to one Product and return the operator-facing state."""

        selected_local_planner = (
            local_planner_name(local_planner) if local_planner is not None else None
        )
        report = self._switch(
            SwitchRequest(
                target_product=product_name(product),
                map_name=map_name,
                relocalize=relocalize,
                initial_pose=initial_pose,
                local_planner=selected_local_planner,
                parameter_overrides=dict(parameter_overrides or {}),
            ),
            state_dir=state_dir,
            dry_run=dry_run,
        )
        return {
            "ok": report.ok,
            "status": report.status,
            "robot": self.robot,
            "env": report.env,
            "product": report.target_product,
            "product_variant": report.product_variant,
            "local_planner": report.local_planner,
            "product_session_id": report.product_session_id,
            "phases": list(report.phases),
            "cleanup": list(report.cleanup),
            "readiness": (
                dict(report.readiness) if report.readiness is not None else None
            ),
            "error": report.error,
        }

    def _switch(
        self,
        request: SwitchRequest,
        *,
        backend: SwitchBackend | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> SwitchReport:
        """Apply one complete fail-closed Product transition."""

        plan = self._resolve(
            request.target_product,
            product_variant=request.product_variant,
            local_planner=request.local_planner,
            parameter_overrides=request.parameter_overrides,
        )
        if plan.process_control == "subprocess":
            if self.env != "sim" or plan.env != "sim":
                raise RuntimeError(
                    "subprocess Product switch requires ProductControl env=sim"
                )
            if backend is not None:
                raise RuntimeError(
                    "sim subprocess switch does not accept a field SwitchBackend"
                )
            if dry_run:
                return sim_switch._plan_switch(request, resolved_plan=plan)
            with self._mutation(state_dir) as root:
                return sim_switch._execute_locked_switch(
                    request,
                    runner=self._simulation_runner(root),
                    environment=self._process_env,
                    state_root=root,
                    resolved_plan=plan,
                )
        if plan.process_control != "systemd":
            if dry_run:
                return SwitchReport(
                    current_product=None,
                    target_product=product_name(plan.product),
                    product_variant=plan.product_variant,
                    local_planner=plan.native_nav.get("local_planner"),
                    env=plan.env,
                    dry_run=True,
                    ok=True,
                    status="planned",
                    phases=["preflight"],
                )
            raise RuntimeError(
                "ProductControl.switch cannot execute a RunPlan controlled by "
                f"{plan.process_control!r}"
            )
        if dry_run:
            return execute_switch(
                self,
                request,
                backend=backend,
                environment=self._process_env,
                state_dir=state_dir,
                dry_run=True,
                resolved_plan=plan,
            )
        with self._mutation(state_dir):
            return execute_switch(
                self,
                request,
                backend=backend,
                environment=self._process_env,
                state_dir=state_dir,
                dry_run=dry_run,
                resolved_plan=plan,
            )

    def status(
        self,
        *,
        state_dir: str | Path | None = None,
    ) -> dict[str, Any]:
        """Return the current Product without changing runtime state."""

        root = ProductControlLock(
            state_dir,
            environment=self._process_env,
        ).state_dir
        try:
            plan, _plan_path, _product_session_id = self._current_plan_and_path(root)
        except _CurrentProductNotFound:
            return {
                "ok": True,
                "status": "stopped",
                "robot": self.robot,
                "env": self.env,
                "product": None,
                "local_planner": None,
                "error": None,
            }
        return {
            "ok": True,
            "status": "active",
            "robot": self.robot,
            "env": plan.env,
            "product": plan.product,
            "local_planner": plan.native_nav.get("local_planner"),
            "error": None,
        }

    def stop(
        self,
        *,
        expected_product: str | None = None,
        expected_product_session_id: str | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        """Stop the current Product, optionally guarded by its exact run identity."""

        try:
            report = self._stop(
                expected_product=expected_product,
                expected_product_session_id=expected_product_session_id,
                state_dir=state_dir,
                dry_run=dry_run,
            )
        except _CurrentProductNotFound:
            if expected_product is not None or expected_product_session_id is not None:
                raise RuntimeError("expected Product session is not active") from None
            return {
                "ok": True,
                "status": "stopped",
                "robot": self.robot,
                "env": self.env,
                "product": None,
                "error": None,
            }
        return {
            "ok": report.ok,
            "status": report.status,
            "robot": self.robot,
            "env": report.env,
            "product": report.product,
            "stopped": list(report.stopped),
            "stop_evidence": dict(report.stop_evidence),
            "error": report.error,
        }

    def _apply_plan_for_switch(
        self,
        path: str | Path,
        *,
        previous_plan: RunPlan | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Internal switch primitive for the resolved preflight artifact."""

        plan = RunPlan.load(path)
        runner = self._systemd_runner()
        if previous_plan is None:
            return runner.apply_deferred(plan, dry_run=dry_run)
        return runner.transition(
            previous_plan,
            plan,
            dry_run=dry_run,
            defer_rollback=True,
        )

    def _quiesce_plan_for_switch(
        self,
        plan: RunPlan,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Internal rollback primitive for one resolved switch target."""

        if plan.process_control == "subprocess":
            raise RuntimeError(
                "subprocess lifecycle requires the committed current RunPlan"
            )
        with self._mutation(state_dir):
            return self._systemd_runner().quiesce(plan, dry_run=dry_run)

    def _stop(
        self,
        *,
        expected_product: str | None = None,
        expected_product_session_id: str | None = None,
        backend: SwitchBackend | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Safely stop the committed Product and remove its session config.

        The optional expected identity fields form one compare-and-stop guard.
        The trusted current Product is loaded, compared, and stopped while
        holding the same mutation lock, so callers cannot stop a replacement
        session even when it uses the same Product.
        """

        expected = (
            product_name(str(expected_product).strip())
            if expected_product is not None
            else None
        )
        expected_session = (
            str(expected_product_session_id)
            if expected_product_session_id is not None
            else None
        )
        if expected_session is not None and not is_product_session_id(expected_session):
            raise ValueError("expected Product session ID is invalid")

        with self._mutation(state_dir) as root:
            plan, plan_path, product_session_id = self._current_plan_and_path(root)
            current = product_name(plan.product)
            if expected is not None and current != expected:
                raise ExpectedProductMismatch(
                    expected_product=expected,
                    current_product=current,
                )
            if expected_session is not None and product_session_id != expected_session:
                raise RuntimeError(
                    "current Product session does not match the compare-and-stop identity"
                )
            if plan.process_control == "subprocess" and plan.env == "sim":
                if backend is not None:
                    raise RuntimeError(
                        "sim subprocess stop does not accept a field SwitchBackend"
                    )
                if dry_run:
                    return ProcessReport(
                        product=plan.product,
                        env=plan.env,
                        action="stop",
                        dry_run=True,
                        ok=True,
                        status="planned",
                        planned=[
                            process.target for process in plan.managed_processes
                        ],
                    )
                report = self._simulation_runner(root).stop(
                    plan_path,
                    product_session_id=product_session_id,
                )
                if report.ok:
                    (root / CURRENT_RUN_FILE_NAME).unlink(missing_ok=True)
                    try:
                        plan_path.unlink(missing_ok=True)
                    except OSError:
                        # The stopped Product is already gone; a stale plan is harmless.
                        pass
                return report
            if plan.process_control != "systemd":
                raise RuntimeError(
                    "stop requires process_control=systemd or sim subprocess"
                )
            active_backend: SwitchBackend | None = None
            if not dry_run:
                field_environment = dict(self._process_env)
                field_environment["LINGTU_SESSION_ROOT"] = str(root)
                active_backend = backend or FieldBackend(environment=field_environment)
                active_backend.stop_motion(current)
            report = self._systemd_runner().stop(plan, dry_run=dry_run)
            if report.ok and active_backend is not None:
                active_backend.remove_session(plan)
                resolve_current_run_path(
                    root,
                    environment=self._process_env,
                ).unlink(missing_ok=True)
            return report

    def _systemd_runner(self) -> SystemdRunner:
        """Return a SystemdRunner that evaluates the same release environment."""

        return self._runner_impl or SystemdRunner(environment=self._process_env)

    def _simulation_runner(self, state_root: Path) -> sim_switch.SimSwitchRunner:
        """Return the private path-bound runner for the locked sim session."""

        return self._simulation_runner_impl or ensure_sim_supervisor(
            state_root,
            _REPOSITORY_ROOT,
            timeout_s=_SIM_SUPERVISOR_START_TIMEOUT_S,
        )

    def _current_plan_and_path(
        self,
        state_dir: Path,
    ) -> tuple[RunPlan, Path, str]:
        """Load one validated snapshot of the committed RunPlan identity."""

        if self.env == "sim":
            committed = sim_switch._load_committed_plan(
                state_dir,
                self._process_env,
            )
            if committed is None:
                raise _CurrentProductNotFound(
                    f"current run record not found: {state_dir / CURRENT_RUN_FILE_NAME}"
                )
            self._require_plan_robot(committed.plan)
            return committed.plan, committed.path, committed.product_session_id

        current = _load_current_run(state_dir, self._process_env)
        plan_path = Path(_current_text(current, "run_plan_path")).expanduser()
        if not plan_path.is_absolute():
            raise RuntimeError("current RunPlan path must be absolute")
        recorded_env = _current_text(current, "env")
        if recorded_env != self.env:
            raise RuntimeError(
                f"current RunPlan belongs to Env {recorded_env!r}, not {self.env!r}"
            )
        exact_plan_path = plan_path.resolve()
        plan = RunPlan.load(exact_plan_path)
        if _current_text(current, "product") != plan.product:
            raise RuntimeError("current Product does not match RunPlan")
        recorded_variant = current.get("product_variant")
        if recorded_variant is not None:
            if not isinstance(recorded_variant, str) or not recorded_variant.strip():
                raise RuntimeError("current Product variant is invalid")
            recorded_variant = recorded_variant.strip()
        if recorded_variant != plan.product_variant:
            raise RuntimeError("current Product variant does not match RunPlan")
        if _current_text(current, "env") != plan.env:
            raise RuntimeError("current Env does not match RunPlan")
        self._require_plan_robot(plan)
        product_session_id = _current_text(current, "product_session_id")
        if not is_product_session_id(product_session_id):
            raise RuntimeError("current Product session id is invalid")
        return plan, exact_plan_path, product_session_id

    def _require_plan_robot(self, plan: RunPlan) -> None:
        recorded = plan.robot
        if self.robot and recorded != self.robot:
            raise RuntimeError(
                f"current Product belongs to Robot {recorded!r}, not {self.robot!r}"
            )
        self.robot = recorded

def _load_current_run(
    state_dir: Path,
    environment: Mapping[str, str],
) -> Mapping[str, Any]:
    path = resolve_current_run_path(state_dir, environment=environment)
    try:
        raw = path.read_bytes()
    except FileNotFoundError as exc:
        raise _CurrentProductNotFound(f"current run record not found: {path}") from exc
    payload = _decode_current_run(raw, path)
    return payload


def _decode_current_run(raw: bytes, path: Path) -> Mapping[str, Any]:
    try:
        payload = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_strict_object_pairs,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise RuntimeError(f"current run record is invalid JSON: {path}") from exc
    if not isinstance(payload, Mapping):
        raise RuntimeError(f"current run record must be a JSON object: {path}")
    if payload.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError("current run record has unsupported schema")
    return payload


def _strict_object_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if not isinstance(key, str) or key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> Any:
    raise ValueError(f"invalid JSON constant: {value}")


def _current_text(payload: Mapping[str, Any], field: str) -> str:
    value = payload.get(field)
    if not isinstance(value, str) or not value.strip():
        raise RuntimeError(f"current run record requires {field}")
    return value.strip()


def _env_name(value: Any) -> str:
    env = str(value or "").strip()
    if env not in {"real", "sim"}:
        raise ValueError(f"Env must be 'real' or 'sim', received {env!r}")
    return env


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=(
            "switch",
            "status",
            "stop",
        ),
    )
    parser.add_argument("product", nargs="?", help="Field Product name")
    parser.add_argument(
        "--robot",
        default=os.environ.get("LINGTU_ROBOT"),
        help="Robot model, for example doso/thunder_v4 or unitree/go2",
    )
    parser.add_argument(
        "--env",
        choices=("real", "sim"),
        default=os.environ.get("LINGTU_ENV") or "real",
    )
    parser.add_argument(
        "--backend",
        help="Concrete simulation backend, for example mujoco",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open the MuJoCo viewer for a simulation Product",
    )
    parser.add_argument(
        "--local-planner",
        choices=("cmu", "scan"),
        help="Select the local-planner backend without changing the Product",
    )
    parser.add_argument("--map", dest="map_name")
    parser.add_argument(
        "--expected-product",
        help=(
            "Stop only if this Product is still current; the comparison and "
            "stop run under one ProductControl lock"
        ),
    )
    parser.add_argument("--state-dir", type=Path)
    parser.add_argument("--initial-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Override one validated session parameter",
    )
    relocalize = parser.add_mutually_exclusive_group()
    relocalize.add_argument("--relocalize", dest="relocalize", action="store_true")
    relocalize.add_argument("--no-relocalize", dest="relocalize", action="store_false")
    parser.set_defaults(relocalize=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    return parser


def _print(payload: Mapping[str, Any], *, json_output: bool) -> None:
    if json_output:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return
    status = str(payload.get("status") or "planned")
    product = str(
        payload.get("product")
        or payload.get("target_product")
        or "unknown"
    )
    robot = str(payload.get("robot") or "unknown")
    env = str(payload.get("env") or "unknown")
    print(f"{status}: {product} on {robot} in {env}")


def main(argv: list[str] | None = None) -> int:
    """Run Product control outside the managed Host process."""

    args = _parser().parse_args(argv)
    control: ProductControl | None = None
    try:
        if args.local_planner and args.action != "switch":
            raise ValueError("--local-planner is only valid with switch")
        if args.backend and (args.action != "switch" or args.env != "sim"):
            raise ValueError("--backend is only valid with switch --env sim")
        if args.viewer and (args.action != "switch" or args.env != "sim"):
            raise ValueError("--viewer is only valid with switch --env sim")
        env_config: dict[str, Any] = {}
        if args.backend:
            env_config["backend"] = args.backend
        if args.viewer:
            env_config["viewer"] = True
        control = ProductControl(
            robot=args.robot,
            env=args.env,
            env_config=env_config or None,
        )
        if args.expected_product and args.action != "stop":
            raise ValueError("--expected-product is only valid with stop")
        if args.action == "switch":
            initial_pose = tuple(args.initial_pose) if args.initial_pose is not None else None
            requested_product = str(
                args.product or os.environ.get("LINGTU_PRODUCT") or ""
            ).strip()
            if not requested_product:
                raise ValueError("switch requires PRODUCT or LINGTU_PRODUCT")
            payload = control.switch(
                requested_product,
                map_name=args.map_name,
                relocalize=bool(args.relocalize),
                initial_pose=initial_pose,
                local_planner=args.local_planner,
                parameter_overrides=_parameter_overrides(args.set),
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            )
        elif args.action == "status":
            if args.product:
                raise ValueError("status does not accept a Product selector")
            payload = control.status(state_dir=args.state_dir)
        elif args.action == "stop":
            if args.product:
                raise ValueError("stop does not accept a Product selector")
            if args.expected_product:
                payload = control._stop(
                    expected_product=args.expected_product,
                    state_dir=args.state_dir,
                    dry_run=args.dry_run,
                ).as_dict()
            else:
                payload = control.stop(
                    state_dir=args.state_dir,
                    dry_run=args.dry_run,
                )
        else:
            raise AssertionError(f"unsupported ProductControl action: {args.action}")
    except SwitchFailed as exc:
        report = exc.report
        print(
            json.dumps(
                {
                    "ok": False,
                    "status": report.status,
                    "robot": control.robot if control is not None else args.robot,
                    "env": report.env,
                    "product": report.target_product,
                    "error": report.error,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 1
    except ProcessFailed as exc:
        report = exc.report
        print(
            json.dumps(
                {
                    "ok": False,
                    "status": report.status,
                    "robot": control.robot if control is not None else args.robot,
                    "env": report.env,
                    "product": report.product,
                    "error": report.error,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 1
    except ExpectedProductMismatch as exc:
        print(
            json.dumps(
                {
                    "ok": False,
                    "reason": exc.reason,
                    "error": str(exc),
                    "expected_product": exc.expected_product,
                    "current_product": exc.current_product,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 3
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2
    _print(payload, json_output=args.json)
    return 0


def _parameter_overrides(values: list[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for value in values:
        key, separator, raw = value.partition("=")
        if not separator or not key.strip() or not raw.strip():
            raise ValueError(f"invalid parameter override: {value!r}")
        overrides[key.strip()] = raw.strip()
    return overrides


if __name__ == "__main__":
    raise SystemExit(main())
