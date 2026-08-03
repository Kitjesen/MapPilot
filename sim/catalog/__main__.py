"""Command-line entry point for deterministic simulation session resolution."""

from __future__ import annotations

import argparse
from pathlib import Path

from .resolver import CatalogResolver


def main() -> int:
    parser = argparse.ArgumentParser(description="Resolve a LingTu simulation session into a SessionBundle.")
    parser.add_argument("session", type=Path, help="path to a lingtu.sim.session.v1 YAML file")
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="repository root; defaults to the current working directory",
    )
    parser.add_argument("--output-dir", type=Path, help="write session.lock.json and physics.plan.json here")
    args = parser.parse_args()

    repo_root = (args.repo_root or Path.cwd()).resolve()
    resolved = CatalogResolver.from_repository(repo_root).resolve(args.session)
    if args.output_dir is not None:
        resolved.write_bundle(args.output_dir)
    print(resolved.session_digest)
    if args.output_dir is not None:
        print((args.output_dir / "session.lock.json").resolve())
        print((args.output_dir / "physics.plan.json").resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
