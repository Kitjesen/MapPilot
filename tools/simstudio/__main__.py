"""Loopback launcher for the simulation-local SimStudio service."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Sequence

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765


def _repository_markers(root: Path) -> bool:
    """Return whether ``root`` has the minimum controlled repository shape."""

    return (
        root.is_dir()
        and (root / "pyproject.toml").is_file()
        and (root / "sim").is_dir()
        and (root / "config").is_dir()
    )


def discover_repo_root(candidate: Path | None = None) -> Path:
    """Find a controlled LingTu repository root without consulting environment state."""

    if candidate is not None:
        root = Path(candidate).expanduser().resolve(strict=True)
        if not root.is_dir() or not _repository_markers(root):
            raise ValueError(f"not a LingTu repository root: {root}")
        return root

    module_root = Path(__file__).resolve().parents[2]
    if _repository_markers(module_root):
        return module_root

    current = Path.cwd().resolve()
    for root in (current, *current.parents):
        if _repository_markers(root):
            return root
    raise RuntimeError("unable to discover a controlled LingTu repository root")


def build_app(repo_root: Path | None = None) -> Any:
    """Construct the local FastAPI app without binding sockets or starting processes."""

    from tools.simstudio.http import create_app
    from tools.simstudio.service.application import SimulationStudioService

    service = SimulationStudioService.from_repository(discover_repo_root(repo_root))
    return create_app(service)


def _validate_port(value: str) -> int:
    port = int(value)
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("Studio port must be between 1 and 65535")
    return port


def build_server(repo_root: Path | None = None, *, port: int = DEFAULT_PORT) -> Any:
    """Construct one Uvicorn server fixed to the local loopback interface."""

    if not 1 <= port <= 65535:
        raise ValueError("Studio port must be between 1 and 65535")
    import uvicorn

    config = uvicorn.Config(
        build_app(repo_root),
        host=DEFAULT_HOST,
        port=port,
        workers=1,
        reload=False,
    )
    return uvicorn.Server(config)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the local LingTu SimStudio service")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="optional controlled local repository root; auto-discovered by default",
    )
    parser.add_argument(
        "--port",
        type=_validate_port,
        default=DEFAULT_PORT,
        help=f"SimStudio HTTP port only (default: {DEFAULT_PORT})",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    """Run the single local SimStudio Uvicorn server."""

    args = _parser().parse_args(argv)
    build_server(args.repo_root, port=args.port).run()


if __name__ == "__main__":  # pragma: no cover - exercised by the launcher itself
    main()


__all__ = [
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "build_app",
    "build_server",
    "discover_repo_root",
    "main",
]
