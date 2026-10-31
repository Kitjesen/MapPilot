"""Materialize RobotLab robot sources and write a source-library inventory."""

from __future__ import annotations

import argparse
import ast
import json
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Any, Sequence

from sim.catalog.importers.intake import SourceIntake

REPOSITORY_URL = "https://github.com/fan-ziqi/robot_lab.git"
CONFIRMED_REVISION = "500399ed75f510aeaff28705a8ce736c514dbec3"
ROBOTS_PATH = PurePosixPath("source/robot_lab/data/Robots")
LICENSE_PATHS = (PurePosixPath("LICENSE"), PurePosixPath(".github/LICENSE_HEADER.txt"))
EXTERNAL_FAMILIES = (
    {
        "family": "ANYmal D",
        "status": "external_not_materialized",
        "provider": "isaaclab_assets.robots.anymal.ANYMAL_D_CFG",
        "evidence": (
            "source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/"
            "config/quadruped/anymal_d/rough_env_cfg.py"
        ),
    },
    {
        "family": "Unitree H1",
        "status": "external_not_materialized",
        "provider": "isaaclab_assets.robots.unitree.H1_MINIMAL_CFG",
        "evidence": (
            "source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/"
            "config/humanoid/unitree_h1/rough_env_cfg.py"
        ),
    },
)
PRESENTATION_METADATA = {
    "agibot/d1": ("agibot_d1", "Agibot D1", "urdf/edu.urdf"),
    "booster/t1_description": ("booster_t1", "Booster T1", "urdf/robot.urdf"),
    "ddt/tita_description": ("ddtrobot_tita", "DDTRobot Tita", "urdf/tita.urdf"),
    "deeprobotics/lite3_description": (
        "deeprobotics_lite3",
        "DeepRobotics Lite3",
        "urdf/lite3.urdf",
    ),
    "deeprobotics/m20_description": (
        "deeprobotics_m20",
        "DeepRobotics M20",
        "urdf/m20.urdf",
    ),
    "fftai/gr1t1_description": ("fftai_gr1t1", "FFTAI GR1T1", "urdf/GR1T1.urdf"),
    "fftai/gr1t2_description": ("fftai_gr1t2", "FFTAI GR1T2", "urdf/GR1T2.urdf"),
    "magiclab/magicbot-Gen1": (
        "magiclab_magicbot_gen1",
        "MagicLab MagicBot-Gen1",
        "urdf/MAGICBOT.urdf",
    ),
    "magiclab/magicbot-Z1": (
        "magiclab_magicbot_z1",
        "MagicLab MagicBot-Z1",
        "urdf/MagicBotZ1.urdf",
    ),
    "magiclab/magicdog": ("magiclab_magicdog", "MagicLab MagicDog", "urdf/magicdog.urdf"),
    "magiclab/magicdog_w": (
        "magiclab_magicdog_w",
        "MagicLab MagicDog-W",
        "urdf/magicdog_w.urdf",
    ),
    "openloong/loong_description": ("openloong_loong", "OpenLoong Loong", "urdf/loong.urdf"),
    "roboparty/atom01_description": (
        "roboparty_atom01",
        "RoboParty ATOM01",
        "urdf/atom01.urdf",
    ),
    "robotera/xbot_description": ("robotera_xbot", "RobotEra Xbot", "urdf/robot.urdf"),
    "unitree/a1_description": ("unitree_a1", "Unitree A1", "urdf/a1.urdf"),
    "unitree/b2_description": ("unitree_b2", "Unitree B2", "urdf/b2_description.urdf"),
    "unitree/b2w_description": (
        "unitree_b2w",
        "Unitree B2W",
        "urdf/b2w_description.urdf",
    ),
    "unitree/g1_description": (
        "unitree_g1",
        "Unitree G1",
        "urdf/g1_29dof_rev_1_0.urdf",
    ),
    "unitree/go2_description": ("unitree_go2", "Unitree Go2", "urdf/go2_description.urdf"),
    "unitree/go2w_description": (
        "unitree_go2w",
        "Unitree Go2W",
        "urdf/go2w_description.urdf",
    ),
    "zsibot/zsl1_description": ("zsibot_zsl1", "Zsibot ZSL1", "urdf/zsl1.urdf"),
    "zsibot/zsl1w_description": ("zsibot_zsl1w", "Zsibot ZSL1W", "urdf/zsl1w.urdf"),
}
_FIND_URI = re.compile(r"^file://\$\(find ([^)]+)\)/(.*)$")


class SourceImportError(RuntimeError):
    """Raised when the confirmed RobotLab source cannot be inventoried safely."""


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _git(upstream: Path, *arguments: str) -> str:
    completed = subprocess.run(
        ["git", "-C", str(upstream), *arguments],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise SourceImportError(f"git {' '.join(arguments)} failed: {detail}")
    return completed.stdout.strip()


def acquire_sparse_sources(upstream: Path) -> str:
    """Add the robot data and license notice directory to an existing sparse clone."""

    if not (upstream / ".git").exists():
        raise SourceImportError(f"expected an existing RobotLab git clone: {upstream}")
    _git(upstream, "sparse-checkout", "add", ROBOTS_PATH.as_posix(), ".github")
    revision = _git(upstream, "rev-parse", "HEAD")
    if not (upstream / ROBOTS_PATH).is_dir():
        raise SourceImportError(f"sparse checkout did not materialize {ROBOTS_PATH}")
    return revision


def _configured_urdfs(upstream: Path) -> set[str]:
    configured: set[str] = set()
    assets_root = upstream / "source/robot_lab/robot_lab/assets"
    for source in sorted(assets_root.glob("*.py")):
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
                continue
            marker = "/Robots/"
            if marker not in node.value or not node.value.lower().endswith(".urdf"):
                continue
            configured.add(node.value.split(marker, 1)[1])
    return configured


def _collapse(path: PurePosixPath) -> PurePosixPath | None:
    parts: list[str] = []
    for part in path.parts:
        if part in {"", "."}:
            continue
        if part == "..":
            if not parts:
                return None
            parts.pop()
        else:
            parts.append(part)
    return PurePosixPath(*parts)


def _resolve_reference(
    uri: str,
    *,
    urdf_path: PurePosixPath,
    model_root: PurePosixPath,
    package_roots: dict[str, PurePosixPath],
    exact_files: set[str],
    folded_files: dict[str, str],
) -> dict[str, str]:
    candidates: list[tuple[PurePosixPath, str]]
    if uri.startswith("package://"):
        package_and_path = uri.removeprefix("package://").split("/", 1)
        package_root = package_roots.get(package_and_path[0])
        candidates = (
            [(package_root / package_and_path[1], "package_uri")]
            if package_root is not None and len(package_and_path) == 2
            else []
        )
    elif match := _FIND_URI.match(uri):
        package_root = package_roots.get(match.group(1))
        candidates = [(package_root / match.group(2), "ros_find_uri")] if package_root is not None else []
    elif "://" in uri or PurePosixPath(uri).is_absolute():
        candidates = []
    else:
        relative_uri = PurePosixPath(uri.replace("\\", "/"))
        candidates = [
            (urdf_path.parent / relative_uri, "urdf_relative"),
            (model_root / relative_uri, "model_root_relative"),
        ]

    normalized_candidates = [
        (normalized, resolution)
        for candidate, resolution in candidates
        if (normalized := _collapse(candidate)) is not None
    ]
    if not normalized_candidates:
        return {"uri": uri, "status": "unresolved"}
    for normalized, resolution in normalized_candidates:
        path = normalized.as_posix()
        if path in exact_files:
            return {"uri": uri, "status": "resolved", "path": path, "resolution": resolution}
    for normalized, resolution in normalized_candidates:
        path = normalized.as_posix()
        case_match = folded_files.get(path.casefold())
        if case_match is not None:
            return {
                "uri": uri,
                "status": "case_mismatch",
                "path": path,
                "actual_path": case_match,
                "resolution": resolution,
            }
    return {"uri": uri, "status": "missing", "path": normalized_candidates[0][0].as_posix()}


def _urdf_inventory(
    urdf: Path,
    *,
    robots_root: Path,
    model_root: PurePosixPath,
    package_roots: dict[str, PurePosixPath],
    exact_files: set[str],
    folded_files: dict[str, str],
    configured: set[str],
) -> dict[str, Any]:
    relative = PurePosixPath(urdf.relative_to(robots_root).as_posix())
    root = ET.parse(urdf).getroot()
    references = {
        element.attrib["filename"]
        for element in root.iter()
        if element.tag in {"mesh", "texture"} and element.attrib.get("filename")
    }
    resolved = [
        _resolve_reference(
            uri,
            urdf_path=relative,
            model_root=model_root,
            package_roots=package_roots,
            exact_files=exact_files,
            folded_files=folded_files,
        )
        for uri in sorted(references)
    ]
    return {
        "path": (ROBOTS_PATH / relative).as_posix(),
        "robot_name": root.attrib.get("name"),
        "configured_by_robot_lab": relative.as_posix() in configured,
        "references": resolved,
        "reference_counts": {
            status: sum(item["status"] == status for item in resolved)
            for status in ("resolved", "missing", "case_mismatch", "unresolved")
        },
    }


def build_inventory(upstream: Path, *, revision: str) -> dict[str, Any]:
    """Inventory every local model directory, URDF variant, and URDF file reference."""

    robots_root = upstream / ROBOTS_PATH
    if not robots_root.is_dir():
        raise SourceImportError(f"RobotLab robot source tree is missing: {robots_root}")
    for notice in LICENSE_PATHS:
        if not (upstream / notice).is_file():
            raise SourceImportError(f"RobotLab license notice is missing: {notice}")

    all_files = sorted(path for path in robots_root.rglob("*") if path.is_file())
    source_files = [
        {"path": (ROBOTS_PATH / path.relative_to(robots_root)).as_posix(), "size_bytes": path.stat().st_size}
        for path in all_files
    ]
    relative_files = {path.relative_to(robots_root).as_posix() for path in all_files}
    folded_files: dict[str, str] = {}
    for path in sorted(relative_files):
        folded_files.setdefault(path.casefold(), path)

    urdfs = sorted(robots_root.rglob("*.urdf"))
    model_roots = sorted({urdf.parent.parent for urdf in urdfs})
    package_roots = {root.name: PurePosixPath(root.relative_to(robots_root).as_posix()) for root in model_roots}
    if len(package_roots) != len(model_roots):
        raise SourceImportError("model directory names are not unique enough for package:// resolution")
    configured = _configured_urdfs(upstream)

    version_file = upstream / "VERSION"
    release = version_file.read_text(encoding="utf-8").strip() if version_file.is_file() else "unknown"
    source_version = f"robot_lab_{release}"
    models: list[dict[str, Any]] = []
    presentation_entries: list[dict[str, str]] = []
    for model_root in model_roots:
        relative_model = PurePosixPath(model_root.relative_to(robots_root).as_posix())
        model_files = [path for path in all_files if path.is_relative_to(model_root)]
        entries = [
            _urdf_inventory(
                urdf,
                robots_root=robots_root,
                model_root=relative_model,
                package_roots=package_roots,
                exact_files=relative_files,
                folded_files=folded_files,
                configured=configured,
            )
            for urdf in urdfs
            if urdf.is_relative_to(model_root)
        ]
        models.append(
            {
                "id": relative_model.as_posix(),
                "source_directory": (ROBOTS_PATH / relative_model).as_posix(),
                "file_count": len(model_files),
                "size_bytes": sum(path.stat().st_size for path in model_files),
                "urdf_entries": entries,
                "source_license": {
                    "declared": "Apache-2.0",
                    "license_file": "LICENSE",
                    "scope": "upstream repository declaration; no per-model notice was present",
                },
            }
        )
        configured_entries = [entry for entry in entries if entry["configured_by_robot_lab"]]
        fallback_urdf = PurePosixPath(entries[0]["path"]).relative_to(ROBOTS_PATH / relative_model)
        default_id = relative_model.as_posix().replace("_description", "").replace("/", "_").lower()
        default_label = default_id.replace("_", " ").title()
        presentation_id, label, primary_urdf = PRESENTATION_METADATA.get(
            relative_model.as_posix(),
            (
                default_id,
                default_label,
                str(
                    PurePosixPath(configured_entries[0]["path"]).relative_to(ROBOTS_PATH / relative_model)
                    if configured_entries
                    else fallback_urdf
                ),
            ),
        )
        primary_path = ROBOTS_PATH / relative_model / primary_urdf
        if primary_path.as_posix() not in {entry["path"] for entry in entries}:
            raise SourceImportError(f"presentation URDF is not an inventoried variant: {primary_path}")
        presentation_entries.append(
            {
                "id": presentation_id,
                "version": source_version,
                "label": label,
                "source_path": (ROBOTS_PATH / relative_model).as_posix(),
                "urdf": primary_path.as_posix(),
                "status": "source_only",
            }
        )

    reference_records = [
        reference
        for model in models
        for urdf in model["urdf_entries"]
        for reference in urdf["references"]
    ]
    return {
        "schema": "lingtu.sim.robot-source-library.v1",
        "source": {
            "repository": REPOSITORY_URL,
            "revision": revision,
            "path": ROBOTS_PATH.as_posix(),
            "license": {
                "declared": "Apache-2.0",
                "files": [path.as_posix() for path in LICENSE_PATHS],
                "asset_scope_note": (
                    "RobotLab declares Apache-2.0 at repository level. No model-specific license files "
                    "were present beside the 22 robot directories, so downstream distribution still "
                    "requires model-origin license review."
                ),
            },
        },
        "summary": {
            "readme_family_count": 24,
            "local_model_count": len(models),
            "external_family_count": len(EXTERNAL_FAMILIES),
            "robot_file_count": len(all_files),
            "robot_size_bytes": sum(path.stat().st_size for path in all_files),
            "urdf_count": len(urdfs),
            "configured_urdf_count": sum(
                entry["configured_by_robot_lab"] for model in models for entry in model["urdf_entries"]
            ),
            "reference_count": len(reference_records),
            "resolved_reference_count": sum(item["status"] == "resolved" for item in reference_records),
            "missing_reference_count": sum(item["status"] == "missing" for item in reference_records),
            "case_mismatch_reference_count": sum(
                item["status"] == "case_mismatch" for item in reference_records
            ),
            "unresolved_reference_count": sum(item["status"] == "unresolved" for item in reference_records),
        },
        "external_families": list(EXTERNAL_FAMILIES),
        "presentation_entries": presentation_entries,
        "models": models,
        "source_files": source_files,
        "activation": {
            "active_robot_packages_created": False,
            "controller_qualification_claimed": False,
            "physics_authority_changed": False,
        },
    }


def _write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _presentation_document(inventory: dict[str, Any]) -> dict[str, Any]:
    return {
        "robots": [
            {key: entry[key] for key in ("id", "version", "label")}
            for entry in inventory["presentation_entries"]
        ]
    }


def _tree_matches_inventory(destination: Path, inventory: dict[str, Any]) -> bool:
    for record in inventory["source_files"]:
        target = destination / PurePosixPath(record["path"])
        if not target.is_file() or target.stat().st_size != record["size_bytes"]:
            return False
    for notice in LICENSE_PATHS:
        if not (destination / notice).is_file():
            return False
    presentation_path = destination / "robot-library.presentation.json"
    if not presentation_path.is_file() or json.loads(
        presentation_path.read_text(encoding="utf-8")
    ) != _presentation_document(inventory):
        return False
    actual = {
        path.relative_to(destination).as_posix()
        for path in destination.rglob("*")
        if path.is_file()
        and path.name not in {"source-library.json", "robot-library.presentation.json"}
    }
    expected = {record["path"] for record in inventory["source_files"]}
    expected.update(path.as_posix() for path in LICENSE_PATHS)
    return actual == expected


def materialize_source_library(
    upstream: Path,
    destination: Path,
    inventory_path: Path,
    *,
    revision: str,
    replace: bool = False,
) -> dict[str, Any]:
    """Copy the confirmed source subset into LingTu's inactive import workspace."""

    inventory = build_inventory(upstream, revision=revision)
    destination = destination.resolve()
    source = upstream.resolve()
    if destination == destination.parent or destination == source or destination in source.parents:
        raise SourceImportError("destination must not replace the source checkout or its parents")
    if destination.exists() and replace:
        marker = destination / "source-library.json"
        try:
            owned = json.loads(marker.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            raise SourceImportError("--replace requires an existing RobotLab source intake") from exc
        if owned.get("schema") != "lingtu.sim.robot-source-library.v1" or owned.get("source", {}).get("repository") != REPOSITORY_URL:
            raise SourceImportError("--replace requires an existing RobotLab source intake")
    if destination.exists() and not replace:
        existing_inventory = destination / "source-library.json"
        if existing_inventory.is_file():
            existing = json.loads(existing_inventory.read_text(encoding="utf-8"))
            if existing == inventory and _tree_matches_inventory(destination, inventory):
                _write_json(inventory_path, inventory)
                return inventory
        raise SourceImportError(
            f"destination already exists with different content: {destination}; rerun with --replace"
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{destination.name}.staging-", dir=destination.parent))
    try:
        SourceIntake(max_files=2_000).materialize(upstream / ROBOTS_PATH, staging / ROBOTS_PATH)
        for notice in LICENSE_PATHS:
            target = staging / notice
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(upstream / notice, target)
        _write_json(staging / "source-library.json", inventory)
        _write_json(staging / "robot-library.presentation.json", _presentation_document(inventory))
        if destination.exists():
            shutil.rmtree(destination)
        staging.rename(destination)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    _write_json(inventory_path, inventory)
    return inventory


def _parser() -> argparse.ArgumentParser:
    root = _repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--upstream",
        type=Path,
        default=root / "build/robot-library-import-20261016/upstream/robot_lab",
    )
    parser.add_argument("--destination", type=Path, default=root / "sim/.imports/robot_lab")
    parser.add_argument(
        "--inventory",
        type=Path,
        default=root / "build/robot-library-import-20261016/robot_lab-source-inventory.json",
    )
    parser.add_argument("--expected-revision", default=CONFIRMED_REVISION)
    parser.add_argument("--replace", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    upstream = args.upstream.resolve()
    revision = acquire_sparse_sources(upstream)
    if revision != args.expected_revision:
        raise SourceImportError(
            f"RobotLab revision mismatch: expected {args.expected_revision}, found {revision}"
        )
    inventory = materialize_source_library(
        upstream,
        args.destination,
        args.inventory,
        revision=revision,
        replace=args.replace,
    )
    print(json.dumps(inventory["summary"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
