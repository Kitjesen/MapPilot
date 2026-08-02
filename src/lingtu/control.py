"""Resolve Product intent and operate exact RunPlan artifacts."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import uuid
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from typing import Any

from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import compile_run_plan
from lingtu.product_lock import ProductControlLock, resolve_current_run_path
from lingtu.product_switch import (
    FieldBackend,
    SwitchBackend,
    SwitchFailed,
    SwitchReport,
    SwitchRequest,
    execute_switch,
)
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.systemd import ProcessFailed, ProcessReport, SystemdRunner
from runtime.profiles.product_lifecycle import product_name


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
    """Operate the current product without reconstructing process targets."""

    def __init__(
        self,
        runner: SystemdRunner | None = None,
        *,
        env: str | None = None,
        env_config: Mapping[str, Any] | None = None,
        process_env: Mapping[str, str] | None = None,
        operation_runner: Callable[..., Any] | None = None,
    ) -> None:
        self._runner_impl = runner
        self._process_env = process_env if process_env is not None else os.environ
        self.env = _env_name(env or self._process_env.get("LINGTU_ENV") or "real")
        self._env_config = dict(env_config or {})
        self._operation_runner = operation_runner or subprocess.run
        if self.env == "sim" and "backend" not in self._env_config:
            env_backend = str(self._process_env.get("LINGTU_ENV_BACKEND") or "").strip()
            if env_backend:
                self._env_config["backend"] = env_backend
        self._plans: dict[tuple[str, str | None], RunPlan] = {}
        self._mutation_state_dir: ContextVar[Path | None] = ContextVar(
            f"lingtu_product_control_state_dir_{id(self)}",
            default=None,
        )

    def submit_switch(
        self,
        request: SwitchRequest,
        *,
        request_id: str | None = None,
        state_dir: str | Path | None = None,
    ) -> dict[str, Any]:
        """Submit a switch outside the managed Host process.

        The transient unit is deliberately owned here rather than by Gateway.
        A field switch replaces ``lingtu.service`` itself, so executing the
        transaction inside Gateway would terminate the caller before the
        ProductControl transaction can commit or roll back.
        """

        plan = self.resolve(
            request.target_product,
            product_variant=request.product_variant,
        )
        if plan.process_control != "systemd":
            raise RuntimeError(
                "ProductControl.submit_switch requires process_control=systemd"
            )
        operation_id = _operation_id(request_id)
        unit = "lingtu-product-control-switch.service"
        command = self._switch_command(request, state_dir=state_dir)
        invocation = self._systemd_submission_command(
            command,
            unit=unit,
            operation_id=operation_id,
            state_dir=state_dir,
        )
        result = self._operation_runner(
            invocation,
            check=False,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
        if int(getattr(result, "returncode", 1)) != 0:
            detail = str(
                getattr(result, "stderr", "")
                or getattr(result, "stdout", "")
                or "systemd rejected the ProductControl operation"
            ).strip()
            raise RuntimeError(f"ProductControl switch submission failed: {detail}")
        return {
            "schema_version": "lingtu.product_control_submission.v1",
            "ok": True,
            "accepted": True,
            "status": "submitted",
            "operation_id": operation_id,
            "unit": unit,
            "product": plan.product,
            "product_variant": plan.product_variant,
            "env": plan.env,
            "run_plan_fingerprint": plan.fingerprint,
        }

    def _switch_command(
        self,
        request: SwitchRequest,
        *,
        state_dir: str | Path | None,
    ) -> list[str]:
        python = str(self._process_env.get("LINGTU_PYTHON") or sys.executable).strip()
        command = [
            python,
            "-m",
            "lingtu.control",
            "switch",
            str(request.target_product),
            "--env",
            self.env,
            "--json",
        ]
        backend = str(self._env_config.get("backend") or "").strip()
        if backend:
            command.extend(["--backend", backend])
        if request.current_product is not None:
            command.extend(["--current", str(request.current_product)])
        if request.map_name:
            command.extend(["--map", str(request.map_name)])
        command.append("--relocalize" if request.relocalize else "--no-relocalize")
        if request.initial_pose is not None:
            command.extend(
                ["--initial-pose", *(str(value) for value in request.initial_pose)]
            )
        for key, value in sorted(request.parameter_overrides.items()):
            command.extend(["--set", f"{key}={value}"])
        if state_dir is not None:
            command.extend(["--state-dir", str(Path(state_dir).expanduser().resolve())])
        return command

    def _systemd_submission_command(
        self,
        command: list[str],
        *,
        unit: str,
        operation_id: str,
        state_dir: str | Path | None,
    ) -> list[str]:
        repo_root = Path(__file__).resolve().parents[2]
        user = str(self._process_env.get("USER") or "sunrise").strip()
        group = str(self._process_env.get("GROUP") or user).strip()
        home = str(
            self._process_env.get("HOME")
            or ("/root" if user == "root" else f"/home/{user}")
        ).strip()
        python_path = str(repo_root / "src")
        existing_python_path = str(self._process_env.get("PYTHONPATH") or "").strip()
        if existing_python_path:
            python_path = f"{python_path}{os.pathsep}{existing_python_path}"
        operation_env = {
            "HOME": home,
            "USER": user,
            "LOGNAME": user,
            "PYTHONPATH": python_path,
            "LINGTU_ENV": self.env,
            "LINGTU_CONTROL_OPERATION_ID": operation_id,
            "GW": str(self._process_env.get("GW") or "http://localhost:5050"),
        }
        backend = str(self._env_config.get("backend") or "").strip()
        if backend:
            operation_env["LINGTU_ENV_BACKEND"] = backend
        configured_state = state_dir or self._process_env.get("LINGTU_SESSION_ROOT")
        if configured_state is not None:
            operation_env["LINGTU_SESSION_ROOT"] = str(
                Path(configured_state).expanduser().resolve()
            )
        for name in ("NAV_MAP_DIR",):
            value = str(self._process_env.get(name) or "").strip()
            if value:
                operation_env[name] = value
        env_args = [
            f"--setenv={name}={value}"
            for name, value in operation_env.items()
        ]
        return [
            "sudo",
            "-n",
            "systemd-run",
            f"--unit={unit}",
            "--collect",
            "--no-block",
            "--service-type=exec",
            f"--working-directory={repo_root}",
            f"--property=User={user}",
            f"--property=Group={group}",
            "--property=EnvironmentFile=-/etc/lingtu/gateway.env",
            "--property=EnvironmentFile=-/opt/lingtu/current/config/release-runtime.env",
            f"--description=LingTu ProductControl switch {operation_id}",
            *env_args,
            "--",
            *command,
        ]

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

    def resolve(
        self,
        product: str | None = None,
        *,
        product_variant: str | None = None,
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
        cache_key = (selected_product, selected_variant)
        cached = self._plans.get(cache_key)
        if cached is not None:
            return cached
        resolved = resolve_product_host_runtime(
            selected_product,
            self.env,
            env_config=self._env_config or None,
            product_variant=selected_variant,
        )
        effective_key = (selected_product, resolved.product_variant)
        cached = self._plans.get(effective_key)
        if cached is not None:
            self._plans[cache_key] = cached
            return cached
        plan = compile_run_plan(
            resolved.product,
            resolved.env,
            resolved.config,
            env_config=self._env_config or None,
            environment=self._process_env,
            product_variant=resolved.product_variant,
        )
        self._plans[cache_key] = plan
        self._plans[effective_key] = plan
        return plan

    def write_plan(
        self,
        path: str | Path,
        *,
        product: str | None = None,
        product_variant: str | None = None,
        state_dir: str | Path | None = None,
    ) -> RunPlan:
        """Compile once and atomically publish the resulting contract."""

        with self._mutation(state_dir):
            plan = self.resolve(product, product_variant=product_variant)
            plan.write(path)
            return plan

    def switch(
        self,
        request: SwitchRequest,
        *,
        backend: SwitchBackend | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> SwitchReport:
        """Apply one complete fail-closed Product transition."""

        plan = self.resolve(
            request.target_product,
            product_variant=request.product_variant,
        )
        if plan.process_control != "systemd":
            if dry_run:
                return SwitchReport(
                    current_product=request.current_product,
                    target_product=product_name(plan.product),
                    env=plan.env,
                    dry_run=True,
                    ok=True,
                    status="planned",
                    phases=["preflight"],
                    run_plan=plan.as_dict(),
                    fingerprint=plan.fingerprint,
                )
            raise RuntimeError(
                "ProductControl.switch cannot execute a RunPlan controlled by "
                f"{plan.process_control!r}; use the corresponding "
                f"{plan.process_control} simulation runner"
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

    def apply(
        self,
        *,
        product: str | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Compile and apply one Product."""

        return self.apply_plan(
            self.resolve(product),
            state_dir=state_dir,
            dry_run=dry_run,
        )

    def apply_plan(
        self,
        plan: RunPlan,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Apply one already resolved RunPlan."""

        if plan.process_control == "systemd":
            raise RuntimeError(
                "field Product requires ProductControl.switch; "
                "direct apply would bypass plan publication and runtime staging"
            )
        with self._mutation(state_dir):
            return self._systemd_runner().apply(plan, dry_run=dry_run)

    def _apply_plan_for_switch(
        self,
        path: str | Path,
        *,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Internal switch primitive for the fingerprinted preflight artifact."""

        plan = RunPlan.load(path)
        return self._systemd_runner().apply(plan, dry_run=dry_run)

    def stop(
        self,
        *,
        product: str | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Compile and stop one Product's mode-owned processes."""

        return self.stop_plan(
            self.resolve(product),
            state_dir=state_dir,
            dry_run=dry_run,
        )

    def stop_plan(
        self,
        plan: RunPlan,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Stop mode-owned processes declared by one RunPlan."""

        with self._mutation(state_dir):
            return self._systemd_runner().stop(plan, dry_run=dry_run)

    def quiesce_plan(
        self,
        plan: RunPlan,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Stop every mode process that may conflict with this plan."""

        with self._mutation(state_dir):
            return self._systemd_runner().quiesce(plan, dry_run=dry_run)

    def restart(
        self,
        process_name: str,
        *,
        product: str | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Compile a Product and restart one declared process."""

        return self.restart_process(
            self.resolve(product),
            process_name,
            state_dir=state_dir,
            dry_run=dry_run,
        )

    def restart_process(
        self,
        plan: RunPlan,
        process_name: str,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Restart one process declared by one RunPlan."""

        with self._mutation(state_dir):
            return self._systemd_runner().restart(
                plan,
                process_name,
                dry_run=dry_run,
            )

    def reapply_current(
        self,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Reapply the exact RunPlan committed as current."""

        with self._mutation(state_dir) as root:
            plan = self._current_plan(root)
            if plan.process_control != "systemd":
                raise RuntimeError("reapply requires process_control=systemd")
            return self._systemd_runner().apply(plan, dry_run=dry_run)

    def quiesce_current(
        self,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Stop current mode processes while preserving the run identity.

        Release activation uses this boundary so the exact committed RunPlan
        remains available for reapply or rollback after the release symlink is
        switched.
        """

        with self._mutation(state_dir) as root:
            plan = self._current_plan(root)
            return self._systemd_runner().quiesce(plan, dry_run=dry_run)

    def restart_current(
        self,
        process_name: str,
        *,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Restart one logical process in the committed RunPlan."""

        with self._mutation(state_dir) as root:
            plan = self._current_plan(root)
            return self._systemd_runner().restart(
                plan,
                process_name,
                dry_run=dry_run,
            )

    def stop_current(
        self,
        *,
        expected_product: str | None = None,
        backend: SwitchBackend | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Safely stop the committed Product and remove its session config.

        ``expected_product`` is an optional compare-and-stop guard.  The
        committed Product is loaded, compared, and stopped while holding the
        same ProductControl mutation lock, so callers cannot accidentally stop
        a Product that replaced the one they intended to stop.
        """

        expected = (
            product_name(str(expected_product).strip())
            if expected_product is not None
            else None
        )

        with self._mutation(state_dir) as root:
            plan = self._current_plan(root)
            current = product_name(plan.product)
            if expected is not None and current != expected:
                raise ExpectedProductMismatch(
                    expected_product=expected,
                    current_product=current,
                )
            active_backend: SwitchBackend | None = None
            if not dry_run:
                field_environment = dict(self._process_env)
                field_environment["LINGTU_SESSION_ROOT"] = str(root)
                active_backend = backend or FieldBackend(environment=field_environment)
                active_backend.stop_motion_and_session(current)
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

    def _current_plan(self, state_dir: Path) -> RunPlan:
        current = _load_current_run(state_dir, self._process_env)
        plan_path = Path(_current_text(current, "run_plan_path")).expanduser()
        if not plan_path.is_absolute():
            raise RuntimeError("current RunPlan path must be absolute")
        plan = RunPlan.load(plan_path.resolve())
        if _current_text(current, "fingerprint") != plan.fingerprint:
            raise RuntimeError("current RunPlan fingerprint does not match its record")
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
        if plan.env != self.env:
            raise RuntimeError(
                f"current RunPlan belongs to Env {plan.env!r}, not {self.env!r}"
            )
        return plan

    def stop_session(
        self,
        *,
        backend: SwitchBackend | None = None,
        state_dir: str | Path | None = None,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        """Cancel native navigation and end the Gateway session without switching Product."""

        with self._mutation(state_dir) as root:
            if not dry_run:
                field_environment = dict(self._process_env)
                field_environment["LINGTU_SESSION_ROOT"] = str(root)
                active_backend = backend or FieldBackend(environment=field_environment)
                active_backend.stop_motion_and_session(active_backend.current_product())
        return {
            "ok": True,
            "action": "stop-session",
            "status": "planned" if dry_run else "stopped",
        }


def _load_current_run(
    state_dir: Path,
    environment: Mapping[str, str],
) -> Mapping[str, Any]:
    path = resolve_current_run_path(state_dir, environment=environment)
    if not path.is_file():
        raise RuntimeError(f"current run record not found: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise RuntimeError(f"current run record must be a JSON object: {path}")
    if payload.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError("current run record has unsupported schema")
    return payload


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


def _operation_id(value: str | None) -> str:
    raw = str(value or uuid.uuid4().hex).strip()
    normalized = re.sub(r"[^A-Za-z0-9_.-]+", "-", raw).strip("-.")
    if not normalized:
        raise ValueError("ProductControl operation id must contain a safe character")
    return normalized[:96]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action",
        choices=(
            "switch",
            "reapply",
            "quiesce",
            "restart",
            "stop",
            "stop-session",
        ),
    )
    parser.add_argument("product", nargs="?", help="Field Product name")
    parser.add_argument(
        "--env",
        choices=("real", "sim"),
        default=os.environ.get("LINGTU_ENV") or "real",
    )
    parser.add_argument(
        "--backend",
        default=os.environ.get("LINGTU_ENV_BACKEND"),
        help="Required implementation backend for --env sim (default: LINGTU_ENV_BACKEND)",
    )
    parser.add_argument("--current")
    parser.add_argument("--map", dest="map_name")
    parser.add_argument("--process")
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
    env = str(payload.get("env") or "unknown")
    print(f"{status}: {product} in {env}")


def main(argv: list[str] | None = None) -> int:
    """Run Product control outside the managed Host process."""

    args = _parser().parse_args(argv)
    try:
        env_config = {"backend": args.backend} if args.backend else None
        control = ProductControl(env=args.env, env_config=env_config)
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
                SwitchRequest(
                    target_product=product_name(requested_product),
                    current_product=(
                        product_name(args.current)
                        if args.current
                        else None
                    ),
                    map_name=args.map_name,
                    relocalize=bool(args.relocalize),
                    initial_pose=initial_pose,
                    parameter_overrides=_parameter_overrides(args.set),
                ),
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            ).as_dict()
        elif args.action == "reapply":
            if args.product:
                raise ValueError("reapply does not accept a Product selector")
            payload = control.reapply_current(
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            ).as_dict()
        elif args.action == "restart":
            if args.product:
                raise ValueError("restart does not accept a Product selector")
            if not args.process:
                raise ValueError("restart requires --process <logical-name>")
            payload = control.restart_current(
                args.process,
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            ).as_dict()
        elif args.action == "quiesce":
            if args.product:
                raise ValueError("quiesce does not accept a Product selector")
            payload = control.quiesce_current(
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            ).as_dict()
        elif args.action == "stop":
            if args.product:
                raise ValueError("stop does not accept a Product selector")
            payload = control.stop_current(
                expected_product=args.expected_product,
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            ).as_dict()
        else:
            if args.product:
                raise ValueError("stop-session does not accept a Product selector")
            payload = control.stop_session(
                state_dir=args.state_dir,
                dry_run=args.dry_run,
            )
    except SwitchFailed as exc:
        print(json.dumps(exc.report.as_dict(), ensure_ascii=False, indent=2))
        return 1
    except ProcessFailed as exc:
        print(json.dumps(exc.report.as_dict(), ensure_ascii=False, indent=2))
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
