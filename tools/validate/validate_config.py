#!/usr/bin/env python3
"""Validate a RobotConfig with the runtime's typed rules."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

from runtime.config import load_config, validate_config

DEFAULT_CONFIG = (
    REPO_ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml"
)


def main() -> None:
    """Validate the requested config and exit non-zero on typed errors."""
    parser = argparse.ArgumentParser(description="Validate a RobotConfig")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Retained for CLI compatibility; typed validation has no warnings",
    )
    parser.add_argument(
        "config",
        nargs="?",
        type=Path,
        default=DEFAULT_CONFIG,
        help="RobotConfig path (defaults to Unitree Go2)",
    )
    args = parser.parse_args()

    path = args.config.resolve()
    if not path.is_file():
        parser.error(f"RobotConfig not found: {path}")

    errors = validate_config(load_config(str(path)))
    for error in errors:
        print(f"ERROR  {error}")
    if errors:
        raise SystemExit(1)

    print(f"OK  {path}")


if __name__ == "__main__":
    main()
