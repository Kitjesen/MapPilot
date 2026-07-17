"""Save-map optimization option resolution."""

from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Any


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class MapOpt:
    """Resolve native map optimization options for MapsServiceCore."""

    def __init__(
        self,
        *,
        strategy: str | None = None,
        command: Any = None,
        timeout_sec: float = 120.0,
        required: bool | None = None,
    ) -> None:
        # The bundled save-time PGO/HBA frontends currently have no independent
        # loop/geometric constraints.  Rebuilding a map from the same odometry
        # poses can only discard an upstream SLAM correction, so optimization is
        # opt-in until the native optimizer can prove a useful constraint set.
        self.strategy = self._strategy(strategy or os.environ.get("LINGTU_MAP_OPT", "off"))
        self.command = command
        self.timeout_sec = float(os.environ.get("LINGTU_MAP_OPT_TIMEOUT", timeout_sec))
        self.required = _as_bool(
            os.environ.get("LINGTU_MAP_OPT_REQUIRED"),
            bool(required) if required is not None else False,
        )

    def source_options(self, strategy: str | None = None) -> dict[str, Any]:
        """Return optimizer options to pass into native SaveMap source options."""
        resolved_strategy = self._strategy(strategy or self.strategy)
        command = None
        if resolved_strategy not in {"", "none", "off", "disabled", "false", "0"}:
            command = self._resolve_command(resolved_strategy)
        return {
            "strategy": resolved_strategy,
            "required": self.required,
            "command": command,
            "timeout_sec": self.timeout_sec,
        }

    @staticmethod
    def _strategy(value: str | None) -> str:
        normalized = str(value or "").strip().lower()
        if normalized in {"auto", "default"}:
            return "pgo"
        return normalized

    def _resolve_command(self, strategy: str) -> Any | None:
        if isinstance(self.command, dict):
            selected = self.command.get(strategy) or self.command.get("default")
            if selected:
                return selected
        elif self.command:
            return self.command

        specific = os.environ.get(f"LINGTU_{strategy.upper()}_CMD")
        if specific:
            return specific
        generic = os.environ.get("LINGTU_MAP_OPT_CMD")
        if generic:
            return generic

        candidates = [
            Path.home() / ".local" / "bin",
            Path("/opt/lingtu/current/build/native-runtime"),
        ]
        repo = os.environ.get("LINGTU_REPO")
        if repo:
            candidates.append(Path(repo) / "build" / "native-runtime")

        for name in (f"lt_{strategy}", f"lingtu_{strategy}"):
            found = shutil.which(name)
            if found:
                return [found]
            for directory in candidates:
                candidate = directory / name
                if candidate.is_file() and os.access(candidate, os.X_OK):
                    return [str(candidate)]
        return None
