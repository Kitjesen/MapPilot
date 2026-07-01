"""Execution boundary for OctoPlanner3D runtimes."""

from __future__ import annotations

import importlib
import json
import os
import shlex
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_EXECUTABLE_NAMES = (
    "octoplanner3d_headless",
    "octoplanner3d_headless.exe",
)
ENV_EXECUTABLE = "LINGTU_OCTOPLANNER3D_EXECUTABLE"
ENV_NATIVE_MODULE = "LINGTU_OCTOPLANNER3D_NATIVE_MODULE"
ENV_WSL_EXECUTABLE = "LINGTU_OCTOPLANNER3D_WSL_EXECUTABLE"
ENV_TIMEOUT = "LINGTU_OCTOPLANNER3D_TIMEOUT_S"
DEFAULT_NATIVE_MODULES = (
    "nav.services.plan.global_planner.algorithm._native",
    "_octoplanner3d_core",
)


@dataclass(frozen=True)
class RuntimeResult:
    stage: str
    result: dict[str, Any] | None = None
    error_message: str = ""
    error_type: str = ""
    returncode: int | None = None
    stdout: str = ""
    stderr: str = ""
    result_type: str = ""

    def diagnostics(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        if self.error_type:
            out["error_type"] = self.error_type
        if self.error_message:
            out["error_message"] = self.error_message
        if self.returncode is not None:
            out["returncode"] = self.returncode
        if self.stderr:
            out["stderr"] = self.stderr[-2000:]
        if self.stdout:
            out["stdout"] = self.stdout[-2000:]
        if self.result_type:
            out["result_type"] = self.result_type
        return out


class OctoPlanner3DRuntime:
    """Resolve and run either the in-process module or headless executable."""

    def __init__(
        self,
        executable_path: str | os.PathLike[str] | None = None,
        timeout_s: float | None = None,
    ) -> None:
        self.native_kernel, self.native_module_name = self.resolve_native_kernel()
        self.executable_path = self.resolve_executable(executable_path)
        self.wsl_executable_path = self.resolve_wsl_executable(self.executable_path)
        self.timeout_s = self.resolve_timeout(timeout_s)

    @property
    def mode(self) -> str:
        if self.native_kernel is not None:
            return "in_process_native"
        return "headless_executable"

    @property
    def process_boundary(self) -> str:
        if self.native_kernel is not None:
            return "in_process"
        return "headless_subprocess"

    @property
    def has_headless_executable(self) -> bool:
        return bool(self.executable_path or self.wsl_executable_path)

    def validate_map(self, map_path: str, supported_extensions: tuple[str, ...]) -> list[str]:
        errors: list[str] = []
        if self.native_kernel is None:
            if self.wsl_executable_path:
                if not (shutil.which("wsl.exe") or shutil.which("wsl")):
                    errors.append(
                        "OctoPlanner3D WSL executable configured but wsl.exe was not found"
                    )
            elif not self.executable_path:
                errors.append(
                    "OctoPlanner3D native kernel/headless executable not configured; set "
                    f"{ENV_NATIVE_MODULE}, {ENV_EXECUTABLE}/{ENV_WSL_EXECUTABLE}, "
                    "or pass executable_path"
                )
            elif not Path(self.executable_path).is_file():
                errors.append(f"OctoPlanner3D executable not found: {self.executable_path}")

        if not map_path:
            errors.append(
                "OctoPlanner3D map path not configured; provide an OctoMap "
                ".bt/.ot/.octomap file or a .pcd point cloud"
            )
            return errors

        if not (
            self.wsl_executable_path and OctoPlanner3DRuntime.looks_like_wsl_path(map_path)
        ) and not Path(map_path).exists():
            errors.append(f"OctoPlanner3D map path not found: {map_path}")

        suffix = Path(map_path).suffix.lower()
        if suffix and suffix not in supported_extensions:
            accepted = ", ".join(supported_extensions)
            errors.append(f"OctoPlanner3D map format unsupported: {suffix}; accepted: {accepted}")
        return errors

    def runtime_map_path(self, map_path: str) -> str:
        if self.wsl_executable_path:
            return self.to_wsl_path(map_path)
        return map_path

    def command(self) -> list[str]:
        if self.wsl_executable_path:
            launcher = shutil.which("wsl.exe") or shutil.which("wsl") or "wsl.exe"
            return [
                launcher,
                "bash",
                "-lc",
                "exec " + shlex.quote(self.wsl_executable_path),
            ]
        return [self.executable_path]

    def run(self, payload: dict[str, Any]) -> RuntimeResult:
        if self.native_kernel is not None:
            return self._run_native(payload)
        return self._run_headless(payload)

    def _run_native(self, payload: dict[str, Any]) -> RuntimeResult:
        try:
            raw_result = self.call_native_kernel(payload)
        except Exception as exc:  # pragma: no cover - exact native exceptions vary
            return RuntimeResult(
                stage="in_process_native_exception",
                error_message=f"octoplanner3d native kernel failed: {exc}",
                error_type=type(exc).__name__,
            )

        result = self.normalize_native_result(raw_result)
        if result is None:
            return RuntimeResult(
                stage="in_process_native_result",
                error_message="octoplanner3d native kernel returned unsupported result",
                result_type=type(raw_result).__name__,
            )
        return RuntimeResult(stage="in_process_native_result", result=result)

    def _run_headless(self, payload: dict[str, Any]) -> RuntimeResult:
        try:
            proc = subprocess.run(  # noqa: S603  # path is operator/config selected
                self.command(),
                input=json.dumps(payload, ensure_ascii=False),
                text=True,
                encoding="utf-8",
                errors="replace",
                capture_output=True,
                timeout=self.timeout_s,
                check=False,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return RuntimeResult(
                stage="subprocess_exception",
                error_message=f"octoplanner3d executable failed: {exc}",
                error_type=type(exc).__name__,
            )

        result = self.parse_result(proc.stdout)
        if proc.returncode != 0:
            return RuntimeResult(
                stage="native_plan_failed" if result else "subprocess_returncode",
                result=result,
                error_message=str(
                    (result or {}).get("error")
                    or (result or {}).get("message")
                    or f"octoplanner3d executable exited with code {proc.returncode}"
                ),
                returncode=proc.returncode,
                stdout=proc.stdout or "",
                stderr=proc.stderr or "",
            )

        if result is None:
            return RuntimeResult(
                stage="parse_stdout",
                error_message="octoplanner3d executable did not return JSON",
                stdout=proc.stdout or "",
            )
        return RuntimeResult(stage="native_plan_result", result=result)

    def call_native_kernel(self, payload: dict[str, Any]) -> Any:
        kernel = self.native_kernel
        if kernel is None:
            raise RuntimeError("OctoPlanner3D native kernel is not loaded")
        for method_name in ("plan", "plan_octoplanner3d"):
            method = getattr(kernel, method_name, None)
            if callable(method):
                return method(payload)
        if callable(kernel):
            return kernel(payload)
        raise TypeError("OctoPlanner3D native kernel must expose plan(payload)")

    @staticmethod
    def normalize_native_result(raw_result: Any) -> dict[str, Any] | None:
        if isinstance(raw_result, dict):
            return dict(raw_result)
        if isinstance(raw_result, list):
            return {"ok": True, "path": raw_result}
        return None

    @staticmethod
    def parse_result(stdout: str) -> dict[str, Any] | None:
        text = (stdout or "").strip()
        if not text:
            return None
        candidates = [text]
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        if lines:
            candidates.append(lines[-1])
        for candidate in candidates:
            try:
                parsed = json.loads(candidate)
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, dict):
                return parsed
        return None

    @staticmethod
    def resolve_native_kernel() -> tuple[Any | None, str]:
        env_value = os.environ.get(ENV_NATIVE_MODULE, "").strip()
        if env_value.lower() in {"0", "none", "off", "disabled"}:
            return None, ""
        candidates = [env_value] if env_value else list(DEFAULT_NATIVE_MODULES)
        for module_name in candidates:
            if not module_name:
                continue
            try:
                module = importlib.import_module(module_name)
            except ImportError:
                if env_value:
                    return None, module_name
                continue
            kernel = getattr(module, "kernel", module)
            return kernel, module_name
        return None, ""

    @staticmethod
    def resolve_executable(executable_path: str | os.PathLike[str] | None) -> str:
        if executable_path:
            return str(executable_path)
        env_value = os.environ.get(ENV_EXECUTABLE, "").strip()
        if env_value:
            return env_value

        for local_candidate in OctoPlanner3DRuntime.local_build_candidates():
            if local_candidate.is_file():
                return str(local_candidate)

        for name in DEFAULT_EXECUTABLE_NAMES:
            found = shutil.which(name)
            if found:
                return found
        return ""

    @staticmethod
    def resolve_wsl_executable(executable_path: str) -> str:
        env_value = os.environ.get(ENV_WSL_EXECUTABLE, "").strip()
        if env_value:
            return OctoPlanner3DRuntime.restore_embedded_wsl_path(env_value)
        restored = OctoPlanner3DRuntime.restore_embedded_wsl_path(executable_path)
        if OctoPlanner3DRuntime.looks_like_wsl_path(restored):
            return restored
        if not executable_path:
            for local_candidate in OctoPlanner3DRuntime.local_wsl_build_candidates():
                if local_candidate.is_file():
                    return OctoPlanner3DRuntime.to_wsl_path(str(local_candidate))
        return ""

    @staticmethod
    def restore_embedded_wsl_path(path: str) -> str:
        value = str(path or "").replace("\\", "/")
        marker = "/mnt/"
        idx = value.find(marker)
        if idx > 0 and len(value) > idx + len(marker) and value[idx + len(marker)].isalpha():
            return value[idx:]
        return value

    @staticmethod
    def looks_like_wsl_path(path: str) -> bool:
        value = OctoPlanner3DRuntime.restore_embedded_wsl_path(path)
        return value.startswith("/mnt/") or value.startswith("/home/")

    @staticmethod
    def to_wsl_path(path: str) -> str:
        value = OctoPlanner3DRuntime.restore_embedded_wsl_path(path)
        if OctoPlanner3DRuntime.looks_like_wsl_path(value):
            return value
        if len(value) >= 3 and value[1] == ":" and value[2] == "/":
            drive = value[0].lower()
            return f"/mnt/{drive}/{value[3:]}"
        if len(value) >= 3 and value[0] == "/" and value[2] == "/" and value[1].isalpha():
            return f"/mnt/{value[1].lower()}/{value[3:]}"
        return value

    @staticmethod
    def local_build_candidates() -> list[Path]:
        repo_root = Path(__file__).resolve().parents[7]
        build_root = repo_root / "build" / "octoplanner3d_headless"
        candidates: list[Path] = []
        for name in DEFAULT_EXECUTABLE_NAMES:
            candidates.append(build_root / name)
            candidates.append(build_root / "Release" / name)
            candidates.append(build_root / "Debug" / name)
        return candidates

    @staticmethod
    def local_wsl_build_candidates() -> list[Path]:
        repo_root = Path(__file__).resolve().parents[7]
        build_roots = (
            repo_root / "build" / "octoplanner3d_headless_wsl",
            repo_root / "build" / "octoplanner3d_headless_wsl_pcl",
        )
        candidates: list[Path] = []
        for build_root in build_roots:
            for name in DEFAULT_EXECUTABLE_NAMES:
                candidates.append(build_root / name)
                candidates.append(build_root / "Release" / name)
                candidates.append(build_root / "Debug" / name)
        return candidates

    @staticmethod
    def resolve_timeout(timeout_s: float | None) -> float:
        if timeout_s is not None:
            return max(0.1, float(timeout_s))
        env_value = os.environ.get(ENV_TIMEOUT, "").strip()
        if env_value:
            try:
                return max(0.1, float(env_value))
            except ValueError:
                return 10.0
        return 10.0
