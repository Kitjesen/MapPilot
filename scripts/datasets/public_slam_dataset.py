#!/usr/bin/env python3
"""Inspect LingTu's public SLAM dataset catalog and create replay manifests."""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.evaluation.slam.public_datasets import (  # noqa: E402
    build_replay_manifest,
    load_public_dataset_catalog,
)


def _atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        text=True,
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_path, path)
    except BaseException:
        temp_path.unlink(missing_ok=True)
        raise


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="List public SLAM datasets or create an LTU1 replay manifest.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="list catalog entries")
    list_parser.add_argument(
        "--json",
        action="store_true",
        help="emit machine-readable JSON",
    )

    manifest_parser = subparsers.add_parser(
        "manifest",
        help="create a lingtu.slam.replay.v1 manifest",
    )
    manifest_parser.add_argument("dataset_id")
    manifest_parser.add_argument("source_path", type=Path)
    manifest_parser.add_argument("output_dir", type=Path)
    manifest_parser.add_argument("--sequence")
    manifest_parser.add_argument(
        "--write",
        type=Path,
        help="write JSON atomically instead of printing it",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    catalog = load_public_dataset_catalog()

    if args.command == "list":
        rows = [
            {
                "id": dataset.id,
                "sensor_model": dataset.sensor_model,
                "source_format": dataset.source_format,
                "license": dataset.license.name,
                "license_review_required": dataset.license.review_required,
                "recommended_uses": list(dataset.recommended_uses),
            }
            for dataset in catalog.datasets
        ]
        if args.json:
            print(json.dumps(rows, indent=2, sort_keys=True))
        else:
            for row in rows:
                review = " [license review required]" if row["license_review_required"] else ""
                print(f"{row['id']}: {row['sensor_model']} / {row['source_format']} / {row['license']}{review}")
        return 0

    dataset = catalog.require(args.dataset_id)
    manifest = build_replay_manifest(
        dataset,
        source_path=args.source_path,
        output_dir=args.output_dir,
        sequence=args.sequence,
    )
    if args.write is not None:
        _atomic_write_json(args.write, manifest)
        print(str(args.write))
    else:
        print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
