"""Trusted local CLI for RobotSimUE Windows distribution operations."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

from .core import DistributionError, WindowsDistribution


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m sim.distribution.windows",
        description=(
            "Preflight, plan, or produce the pinned RobotSimUE Win64 distribution. "
            "Executable paths and UAT arguments are intentionally not configurable."
        ),
    )
    parser.add_argument("operation", choices=("preflight", "dry-run", "package"))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run one trusted local Windows distribution operation."""

    args = _parser().parse_args(argv)
    repo_root = Path(__file__).resolve().parents[3]
    try:
        result = WindowsDistribution(repo_root=repo_root).execute(args.operation)
    except DistributionError as exc:
        print(f"Windows distribution failed closed: {exc}", file=sys.stderr)
        return 1
    output = {
        "operation": args.operation,
        "state": result.manifest["state"],
        "shipping_build_produced": result.manifest.get(
            "shipping_build_produced",
            result.manifest.get("claims", {}).get("shipping_build_produced", False),
        ),
        "manifest_path": (str(result.manifest_path) if result.manifest_path is not None else None),
        "unreal_root": str(result.unreal_root),
    }
    print(json.dumps(output, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
