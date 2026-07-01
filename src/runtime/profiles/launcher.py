"""Runtime process and external launcher context helpers.

CLI commands decide how to present and execute a runtime. This module owns the
runtime-layer facts needed for that execution: run spec, environment, launcher
path, and command arguments.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from runtime.profiles.catalog.endpoints import RuntimeEndpointError, RuntimeRunSpec
from runtime.profiles.endpoints import resolve_runtime_run_spec


@dataclass(frozen=True)
class RuntimeProcessContext:
    """Resolved run spec plus the process environment for a runtime."""

    spec: RuntimeRunSpec
    env: Mapping[str, str]


@dataclass(frozen=True)
class RuntimeLaunchContext(RuntimeProcessContext):
    """External launcher context resolved from a runtime run spec."""

    launcher_path: Path
    command: tuple[str, ...]


def resolve_runtime_process_context(
    profile: str,
    config: Mapping[str, Any],
    *,
    record: bool = False,
    extra_args: tuple[str, ...] = (),
    base_env: Mapping[str, str] | None = None,
) -> RuntimeProcessContext:
    """Resolve runtime spec and overlay its environment on an optional base."""

    spec = resolve_runtime_run_spec(
        profile,
        config,
        record=record,
        extra_args=extra_args,
    )
    env = dict(base_env or {})
    env.update(spec.env)
    return RuntimeProcessContext(spec=spec, env=env)


def build_external_launch_context(
    profile: str,
    config: Mapping[str, Any],
    *,
    repo_root: Path,
    record: bool = False,
    extra_args: tuple[str, ...] = (),
    base_env: Mapping[str, str] | None = None,
) -> RuntimeLaunchContext:
    """Resolve the command and environment for an external runtime launcher."""

    process = resolve_runtime_process_context(
        profile,
        config,
        record=record,
        extra_args=extra_args,
        base_env=base_env,
    )
    spec = process.spec
    if not spec.launcher:
        raise RuntimeEndpointError("external profile missing _external_launcher")
    return RuntimeLaunchContext(
        spec=spec,
        env=process.env,
        launcher_path=(Path(repo_root) / spec.launcher).resolve(),
        command=tuple(spec.as_command()),
    )
