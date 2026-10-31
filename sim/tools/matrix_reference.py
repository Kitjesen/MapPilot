"""Stage a MATRiX distribution as a local reference, never an active package."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path
from typing import Any

from sim.catalog.importers.contracts import SourceIntakeResult
from sim.catalog.importers.intake import SourceIntake

_ASSET_SUFFIXES = {".stl", ".obj", ".fbx", ".uasset", ".xml", ".onnx", ".png"}


def stage_matrix_reference(source: Path, destination: Path, *, version: str) -> dict[str, Any]:
    """Copy/extract one Linux release and describe it without launching or promoting it.

    SourceIntake owns extraction and refuses an existing destination. Release
    files remain unmodified; the generated receipt is not a Catalog manifest.
    """
    owned_root = destination.resolve()
    intake = SourceIntake(
        max_file_bytes=2 * 1024**3,
        max_total_bytes=8 * 1024**3,
    ).materialize(source, owned_root)
    try:
        report = _describe_reference(intake, version=version)
        with (owned_root / "matrix-reference.json").open("x", encoding="utf-8") as output:
            json.dump(report, output, indent=2, ensure_ascii=False)
            output.write("\n")
        return report
    except Exception:
        # materialize() rejects existing destinations; only this new absolute root is owned here.
        shutil.rmtree(owned_root)
        raise


def _describe_reference(intake: SourceIntakeResult, *, version: str) -> dict[str, Any]:
    wrapped_root = intake.root / f"MATRiX_v{version}"
    distribution = wrapped_root if wrapped_root.is_dir() else intake.root
    readme = distribution / "README.md"
    if not readme.is_file() or readme.read_text(encoding="utf-8").partition("\n")[0] != f"# MATRiX v{version}":
        raise ValueError("MATRiX README does not identify the requested release")

    binary = distribution / "UeSim/Binaries/Linux/UeSim"
    with binary.open("rb") as stream:
        header = stream.read(64)
    if len(header) < 64 or header[:6] != b"\x7fELF\x02\x01" or int.from_bytes(header[18:20], "little") != 62:
        raise ValueError("This reference intake requires the Linux x86-64 ELF release")

    binary_relative = binary.relative_to(intake.root).as_posix()
    binary_record = next(item for item in intake.files if item.path == binary_relative)
    counts = Counter(
        suffix
        for item in intake.files
        if (suffix := Path(item.path).suffix.lower()) in _ASSET_SUFFIXES
    )
    report = {
        "schema": "lingtu.sim.matrix-reference.v1",
        "version": version,
        "source": {
            "kind": intake.source_kind,
            "sha256": intake.source_sha256,
            "file_count": len(intake.files),
            "unpacked_bytes": sum(item.size for item in intake.files),
        },
        "runtime": {
            "platform": "linux-x86_64",
            "entry": binary_relative,
            "bytes": binary_record.size,
            "sha256": binary_record.sha256,
            "windows_native": False,
        },
        "asset_counts": dict(sorted(counts.items())),
        "release_readme": readme.relative_to(intake.root).as_posix(),
        "license_files": sorted(
            item.path
            for item in intake.files
            if Path(item.path).stem.upper() in {"LICENSE", "LICENCE", "COPYING", "NOTICE"}
        ),
        "redistribution": "unverified",
        "catalog_registered": False,
        "launch_performed": False,
    }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--version", default="1.1.2")
    parser.add_argument("--destination", type=Path)
    args = parser.parse_args()
    destination = args.destination or (
        Path(__file__).resolve().parents[2] / "third_party/matrix" / f"runtime-v{args.version}"
    )
    report = stage_matrix_reference(args.source, destination, version=args.version)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
