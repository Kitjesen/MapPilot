#!/usr/bin/env python3
"""Audit GTSAM references after the pose_graph_opt Rust migration.

The PGO/HBA migration is complete only for the SE3 prior/between pose graph
surface. This check keeps that boundary explicit: any GTSAM reference inside
src/localization/pgo or src/localization/hba fails, while known remaining surfaces are reported
as separate migration work.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]

SEARCH_PATTERN = re.compile(
    r"libgtsam|libmetis-gtsam|find_package\s*\(\s*GTSAM|(?<![A-Za-z0-9_])gtsam(?![A-Za-z0-9_])",
    re.IGNORECASE,
)
FILENAME_PATTERN = re.compile(r"^(libgtsam|libmetis-gtsam)", re.IGNORECASE)
REMOVED_PCT_RUNTIME_ARTIFACT_ROOT = (
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64"
)
RUNTIME_ARTIFACT_PATTERN = re.compile(
    r"(\.so(\.|$)|\.pyd$|\.dll$|\.dylib$)",
    re.IGNORECASE,
)

TEXT_SUFFIXES = {
    "",
    ".bash",
    ".cfg",
    ".cmake",
    ".cpp",
    ".cc",
    ".cxx",
    ".h",
    ".hpp",
    ".hh",
    ".ini",
    ".json",
    ".launch",
    ".m",
    ".md",
    ".py",
    ".rst",
    ".sh",
    ".toml",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}

SKIP_DIR_NAMES = {
    ".git",
    ".codex",
    ".mypy_cache",
    ".omx",
    ".pytest_cache",
    ".qoder",
    ".ruff_cache",
    ".tmp",
    ".venv",
    "__pycache__",
    "artifacts",
    "build",
    "dist",
    "install",
    "node_modules",
    "target",
}

VENDORED_GTSAM_ROOT = "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1"

CATEGORY_DESCRIPTIONS = {
    "covered_surface_forbidden": "Migrated surface; GTSAM must not reappear here",
    "coverage_audit": "This validator and its tests",
    "covered_kernel_docs": "Rust kernel comments/docs describing the replaced surface",
    "docs_only": "Documentation or non-runtime notes",
    "optional_baseline_tool": "Optional legacy C++/GTSAM comparison tooling",
    "pose_graph_opt_tests": "Tests for the Rust replacement and optional baseline",
    "capability_surface_mismatch": "Classified dependency is not in the capability matrix",
    "removed_legacy_runtime_artifact": "Deleted legacy PCT runtime binary artifact; must not reappear",
    "remaining_dependency": "Known non-PGO/HBA GTSAM-dependent subsystem",
    "remaining_dependency_support": "Build/package support for a known remaining dependency",
    "tests_only": "Tests for other subsystems or packaging checks",
    "unknown": "Unclassified GTSAM reference; review required",
    "vendored_third_party": "Vendored upstream or third-party source bundle",
}

CAPABILITY_SURFACES: tuple[dict[str, object], ...] = (
    {
        "surface": "slam_pgo",
        "coverage": "covered_by_pose_graph_opt",
        "paths": ("src/localization/pgo",),
        "kernel": "src/kernels/slam/pose_graph_opt",
        "capabilities": (
            "SE3 Pose3",
            "prior factors",
            "between factors",
            "full 6x6 information matrices",
            "batch LM/GN smoothing with warm start",
        ),
    },
    {
        "surface": "slam_hba",
        "coverage": "covered_by_pose_graph_opt",
        "paths": ("src/localization/hba",),
        "kernel": "src/kernels/slam/pose_graph_opt",
        "capabilities": (
            "SE3 Pose3",
            "between-only anchored graphs",
            "full 6x6 information matrices",
            "batch LM/GN smoothing",
        ),
    },
    {
        "surface": "pct_gpmp_global_planning",
        "coverage": "remaining_dependency",
        "paths": (
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner",
            "src/nav/services/plan/global_planner/algorithm/pct/runtime",
        ),
        "kernel": None,
        "capabilities": (
            "vector-state nonlinear factor graphs",
            "custom one-key and two-key GPMP factors",
            "interpolation factors",
            "process-noise models",
            "explicit legacy native comparison surface",
        ),
    },
    {
        "surface": "camera_lidar_calibration",
        "coverage": "covered_by_camera_lidar_optimizer",
        "paths": ("calibration/camera_lidar/direct_visual_lidar_calibration",),
        "kernel": "src/kernels/calibration/camera_lidar_optimizer",
        "capabilities": (
            "CT-ICP/CT-GICP fixed-correspondence residual/Jacobian/Hessian kernels",
            "Rust-owned CT-GICP nearest-neighbor correspondence search",
            "CT-GICP two-pose LM optimizer",
            "Rust-owned dynamic CT-GICP correspondence rebuild and optimization loop",
            "stable C ABI",
            "continuous-time pose interpolation",
            "dynamic integrator Rust runtime path",
        ),
    },
    {
        "surface": "vendored_gtsam_source",
        "coverage": "vendored_third_party",
        "paths": (VENDORED_GTSAM_ROOT, "third_party/dimos"),
        "kernel": None,
        "capabilities": ("upstream or third-party source bundle",),
    },
)

DEPENDENCY_SUBSURFACES: tuple[dict[str, object], ...] = (
    {
        "surface": "pct_gpmp_global_planning",
        "subsurface": "pct_runtime_packaging",
        "priority": "P0",
        "paths": (
            "src/nav/services/plan/global_planner/algorithm/pct/runtime",
        ),
        "notes": (
            "Runtime loader, preview entry points, explicit legacy native "
            "comparison guards, and generated native artifact directory."
        ),
    },
    {
        "surface": "pct_gpmp_global_planning",
        "subsurface": "pct_gpmp_optimizer_core",
        "priority": "P1",
        "paths": (
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/gpmp_optimizer",
        ),
        "notes": "Vector-state GPMP factors, interpolators, models, and optimizer implementations.",
    },
    {
        "surface": "pct_gpmp_global_planning",
        "subsurface": "pct_python_binding_control",
        "priority": "P1",
        "paths": (
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/scripts/planner_wrapper.py",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/trajectory_optimization",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/ele_planner",
        ),
        "notes": "Python/binding control layer that can host a dual-track fallback.",
    },
    {
        "surface": "pct_gpmp_global_planning",
        "subsurface": "pct_build_packaging",
        "priority": "P2",
        "paths": (
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/CMakeLists.txt",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/CMakeLists.txt",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/CMakeLists.txt",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/CMakeLists.txt",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/build.sh",
            "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/build_thirdparty.sh",
        ),
        "notes": "Build scripts and CMake GTSAM linkage outside the algorithm runtime.",
    },
)


@dataclass(frozen=True)
class Hit:
    path: str
    line: int
    category: str
    text: str


@dataclass(frozen=True)
class AuditResult:
    root: str
    hits: list[Hit]
    vendored_tree_present: bool

    @property
    def category_counts(self) -> dict[str, int]:
        return dict(sorted(Counter(hit.category for hit in self.hits).items()))

    @property
    def violations(self) -> list[Hit]:
        violations = [
            hit
            for hit in self.hits
            if hit.category
            in {
                "covered_surface_forbidden",
                "removed_legacy_runtime_artifact",
                "unknown",
            }
        ]
        violations.extend(self.capability_surface_violations)
        return violations

    @property
    def capability_surface_violations(self) -> list[Hit]:
        return [
            Hit(
                path=hit.path,
                line=hit.line,
                category="capability_surface_mismatch",
                text=(
                    "remaining_dependency hit does not belong to any "
                    "remaining capability surface"
                ),
            )
            for hit in self.remaining_dependency_hits
            if hit.category == "remaining_dependency"
            and remaining_dependency_surface_for(hit.path) is None
        ]

    @property
    def remaining_dependency_hits(self) -> list[Hit]:
        return [
            hit
            for hit in self.hits
            if hit.category in {"remaining_dependency", "remaining_dependency_support"}
        ]

    @property
    def remaining_dependency_surface_summary(self) -> dict[str, dict[str, object]]:
        summary: dict[str, dict[str, object]] = {}
        for hit in self.remaining_dependency_hits:
            surface = dependency_surface_for_hit(hit)
            bucket = summary.setdefault(
                surface,
                {
                    "hit_count": 0,
                    "path_count": 0,
                    "paths": [],
                    "representative_paths": [],
                    "sample_hits": [],
                },
            )
            paths = set(bucket.pop("_path_set", set()))
            paths.add(hit.path)
            bucket["_path_set"] = paths
            path_records = dict(bucket.pop("_path_records", {}))
            record = dict(
                path_records.get(
                    hit.path,
                    {
                        "path": hit.path,
                        "hit_count": 0,
                        "first_line": hit.line,
                        "sample_text": hit.text,
                    },
                )
            )
            record["hit_count"] = int(record["hit_count"]) + 1
            if hit.line and (not record.get("first_line") or hit.line < int(record["first_line"])):
                record["first_line"] = hit.line
                record["sample_text"] = hit.text
            path_records[hit.path] = record
            bucket["_path_records"] = path_records
            bucket["hit_count"] = int(bucket["hit_count"]) + 1
            sample_hits = list(bucket["sample_hits"])
            if len(sample_hits) < 10:
                sample_hits.append(asdict(hit))
            bucket["sample_hits"] = sample_hits
        for bucket in summary.values():
            paths = sorted(bucket.pop("_path_set", set()))
            path_records = sorted(
                bucket.pop("_path_records", {}).values(),
                key=lambda item: (-int(item["hit_count"]), str(item["path"])),
            )
            bucket["path_count"] = len(paths)
            bucket["paths"] = paths[:25]
            bucket["representative_paths"] = path_records[:10]
            if len(paths) > 25:
                bucket["truncated_paths"] = len(paths) - 25
            if len(path_records) > 10:
                bucket["truncated_representative_paths"] = len(path_records) - 10
        return dict(sorted(summary.items(), key=lambda item: surface_sort_key(item[0])))

    @property
    def remaining_dependency_subsurface_summary(self) -> dict[str, dict[str, object]]:
        summary: dict[str, dict[str, object]] = {}
        for hit in self.remaining_dependency_hits:
            surface = dependency_surface_for_hit(hit)
            subsurface = dependency_subsurface_for_hit(hit)
            bucket = summary.setdefault(
                subsurface,
                {
                    "surface": surface,
                    "priority": dependency_subsurface_priority(subsurface),
                    "hit_count": 0,
                    "path_count": 0,
                    "paths": [],
                    "representative_paths": [],
                },
            )
            paths = set(bucket.pop("_path_set", set()))
            paths.add(hit.path)
            bucket["_path_set"] = paths
            path_records = dict(bucket.pop("_path_records", {}))
            record = dict(
                path_records.get(
                    hit.path,
                    {
                        "path": hit.path,
                        "hit_count": 0,
                        "first_line": hit.line,
                        "sample_text": hit.text,
                    },
                )
            )
            record["hit_count"] = int(record["hit_count"]) + 1
            if hit.line and (not record.get("first_line") or hit.line < int(record["first_line"])):
                record["first_line"] = hit.line
                record["sample_text"] = hit.text
            path_records[hit.path] = record
            bucket["_path_records"] = path_records
            bucket["hit_count"] = int(bucket["hit_count"]) + 1
        for bucket in summary.values():
            paths = sorted(bucket.pop("_path_set", set()))
            path_records = sorted(
                bucket.pop("_path_records", {}).values(),
                key=lambda item: (-int(item["hit_count"]), str(item["path"])),
            )
            bucket["path_count"] = len(paths)
            bucket["paths"] = paths[:25]
            bucket["representative_paths"] = path_records[:10]
            if len(paths) > 25:
                bucket["truncated_paths"] = len(paths) - 25
            if len(path_records) > 10:
                bucket["truncated_representative_paths"] = len(path_records) - 10
        return dict(sorted(summary.items(), key=lambda item: subsurface_sort_key(item[0])))

    def to_jsonable(self) -> dict:
        return {
            "root": self.root,
            "capability_surfaces": list(CAPABILITY_SURFACES),
            "dependency_subsurfaces": list(DEPENDENCY_SUBSURFACES),
            "category_counts": self.category_counts,
            "remaining_dependency_surface_summary": self.remaining_dependency_surface_summary,
            "remaining_dependency_subsurface_summary": self.remaining_dependency_subsurface_summary,
            "violations": [asdict(hit) for hit in self.violations],
            "remaining_dependency_hits": [
                asdict(hit) for hit in self.remaining_dependency_hits
            ],
            "vendored_tree_present": self.vendored_tree_present,
            "hits": [asdict(hit) for hit in self.hits],
        }


def normalize_repo_path(path: Path | str, root: Path = ROOT_DIR) -> str:
    candidate = Path(path)
    if candidate.is_absolute():
        try:
            candidate = candidate.relative_to(root)
        except ValueError:
            return candidate.as_posix()
    rel = candidate.as_posix()
    while rel.startswith("./"):
        rel = rel[2:]
    return rel


def path_matches(rel_path: str, prefix: str) -> bool:
    rel = rel_path.replace("\\", "/").rstrip("/")
    normalized_prefix = prefix.rstrip("/")
    return rel == normalized_prefix or rel.startswith(f"{normalized_prefix}/")


def is_removed_pct_runtime_artifact_path(rel_path: str) -> bool:
    rel = rel_path.replace("\\", "/")
    if not path_matches(rel, REMOVED_PCT_RUNTIME_ARTIFACT_ROOT):
        return False
    name = rel.rsplit("/", 1)[-1]
    return bool(RUNTIME_ARTIFACT_PATTERN.search(name))


def classify_path(path: Path | str, root: Path = ROOT_DIR) -> str:
    rel = normalize_repo_path(path, root)

    if is_removed_pct_runtime_artifact_path(rel):
        return "removed_legacy_runtime_artifact"
    if path_matches(rel, VENDORED_GTSAM_ROOT) or path_matches(rel, "third_party"):
        return "vendored_third_party"
    if path_matches(rel, "src/localization/pgo") or path_matches(rel, "src/localization/hba"):
        return "covered_surface_forbidden"
    if (
        path_matches(rel, "src/kernels/slam/pose_graph_opt")
        or path_matches(rel, "src/kernels/planning/gpmp_trajectory_optimizer")
        or rel == "src/kernels/catalog.py"
    ):
        return "covered_kernel_docs"
    if rel in {
        "tools/validate/validate_pct_gpmp_migration_contract.py",
        "tools/validate/validate_camera_lidar_calibration_migration_contract.py",
        "tools/validate/validate_kernel_migration_status.py",
        "tools/validate/validate_pose_graph_opt_coverage.py",
        "src/runtime/tests/test_kernel_migration_status.py",
        "src/runtime/tests/test_pct_gpmp_migration_contract.py",
        "src/runtime/tests/test_pose_graph_opt_coverage_audit.py",
    }:
        return "coverage_audit"
    if path_matches(rel, "tools/bench/pose_graph_opt_compare.py") or path_matches(
        rel,
        "tools/bench/pose_graph_opt_gtsam_baseline.py",
    ):
        return "optional_baseline_tool"
    if rel.startswith("src/runtime/tests/test_pose_graph_opt_") or rel == (
        "src/runtime/tests/test_compute_kernel_migration_catalog.py"
    ):
        return "pose_graph_opt_tests"
    if (
        rel in {".dockerignore", ".gitignore"}
        or path_matches(rel, "docs")
        or path_matches(rel, "sim/experiments")
        or rel.endswith("/README.md")
        or rel.endswith("/NOTICE")
        or rel.endswith("/LICENSE")
        or rel.endswith("/LICENSE.BSD")
        or "/docs/" in rel
        or rel
        in {
            "calibration/camera_lidar/README.md",
            "config/robot_config.yaml",
            "src/localization/README.md",
        }
    ):
        return "docs_only"
    if path_matches(rel, "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner") or path_matches(
        rel,
        "src/nav/services/plan/global_planner/algorithm/pct/runtime",
    ):
        return "remaining_dependency"
    if path_matches(rel, "calibration/camera_lidar/direct_visual_lidar_calibration"):
        return "covered_surface_forbidden"
    if rel in {
        ".github/workflows/slam-aarch64-build.yml",
        "scripts/build/build_ros_workspace.sh",
        "scripts/deploy/setup_server_ros_pct.sh",
    }:
        return "remaining_dependency_support"
    if "/tests/" in rel or rel.startswith("tests/") or Path(rel).name.startswith("test_"):
        return "tests_only"
    return "unknown"


def remaining_dependency_surface_for(path: Path | str, root: Path = ROOT_DIR) -> str | None:
    rel = normalize_repo_path(path, root)
    for surface in CAPABILITY_SURFACES:
        if surface["coverage"] != "remaining_dependency":
            continue
        for prefix in surface["paths"]:
            if path_matches(rel, str(prefix)):
                return str(surface["surface"])
    return None


def dependency_surface_for_hit(hit: Hit) -> str:
    if hit.category == "remaining_dependency_support":
        return "remaining_dependency_support"
    return remaining_dependency_surface_for(hit.path) or "unmapped_remaining_dependency"


def remaining_dependency_subsurface_for(path: Path | str, root: Path = ROOT_DIR) -> str | None:
    rel = normalize_repo_path(path, root)
    for subsurface in DEPENDENCY_SUBSURFACES:
        for prefix in subsurface["paths"]:
            if path_matches(rel, str(prefix)):
                return str(subsurface["subsurface"])
    return None


def dependency_subsurface_for_hit(hit: Hit) -> str:
    if hit.category == "remaining_dependency_support":
        return "remaining_dependency_support"
    surface = dependency_surface_for_hit(hit)
    subsurface = remaining_dependency_subsurface_for(hit.path)
    if subsurface is not None:
        return subsurface
    if surface == "pct_gpmp_global_planning":
        return "pct_other_remaining_dependency"
    if surface == "camera_lidar_calibration":
        return "camera_lidar_other_remaining_dependency"
    return f"{surface}_other"


def dependency_subsurface_priority(subsurface: str) -> str | None:
    if subsurface == "remaining_dependency_support":
        return "P0"
    for item in DEPENDENCY_SUBSURFACES:
        if item["subsurface"] == subsurface:
            return str(item["priority"])
    return None


def surface_sort_key(surface: str) -> tuple[int, str]:
    remaining_order = [
        str(item["surface"])
        for item in CAPABILITY_SURFACES
        if item["coverage"] == "remaining_dependency"
    ]
    if surface in remaining_order:
        return (remaining_order.index(surface), surface)
    if surface == "remaining_dependency_support":
        return (len(remaining_order), surface)
    if surface == "unmapped_remaining_dependency":
        return (len(remaining_order) + 1, surface)
    return (len(remaining_order) + 2, surface)


def subsurface_sort_key(subsurface: str) -> tuple[int, int, str]:
    surface = None
    priority = dependency_subsurface_priority(subsurface)
    for item in DEPENDENCY_SUBSURFACES:
        if item["subsurface"] == subsurface:
            surface = str(item["surface"])
            break
    priority_order = {"P0": 0, "P1": 1, "P2": 2, "P3": 3, None: 4}
    return (
        surface_sort_key(surface or "unmapped_remaining_dependency")[0],
        priority_order.get(priority, 4),
        subsurface,
    )


def iter_candidate_files(root: Path) -> list[Path]:
    files: list[Path] = []
    stack = [root]
    while stack:
        current = stack.pop()
        try:
            children = sorted(current.iterdir(), reverse=True)
        except OSError:
            continue
        for child in children:
            try:
                is_dir = child.is_dir()
            except OSError:
                continue
            if is_dir:
                rel = normalize_repo_path(child, root)
                if should_skip_dir(child, rel) or path_matches(rel, VENDORED_GTSAM_ROOT):
                    continue
                stack.append(child)
                continue
            rel = normalize_repo_path(child, root)
            if (
                child.suffix.lower() in TEXT_SUFFIXES
                or FILENAME_PATTERN.search(child.name)
                or is_removed_pct_runtime_artifact_path(rel)
            ):
                files.append(child)
    return files


def should_skip_dir(path: Path, rel: str) -> bool:
    if path.name == "build" and path_matches(rel, "scripts/build"):
        return False
    return path.name in SKIP_DIR_NAMES


def scan_repository(root: Path = ROOT_DIR) -> AuditResult:
    root = root.resolve()
    hits: list[Hit] = []
    vendored_root = root / VENDORED_GTSAM_ROOT
    vendored_tree_present = vendored_root.exists()
    if vendored_tree_present:
        hits.append(
            Hit(
                path=VENDORED_GTSAM_ROOT,
                line=0,
                category="vendored_third_party",
                text="vendored GTSAM source tree present",
            )
        )

    for path in iter_candidate_files(root):
        rel = normalize_repo_path(path, root)
        category = classify_path(path, root)
        if is_removed_pct_runtime_artifact_path(rel):
            hits.append(
                Hit(
                    path=rel,
                    line=0,
                    category=category,
                    text=f"removed PCT legacy runtime artifact: {path.name}",
                )
            )
            continue
        if FILENAME_PATTERN.search(path.name):
            hits.append(
                Hit(
                    path=rel,
                    line=0,
                    category=category,
                    text=f"file name matches GTSAM runtime artifact: {path.name}",
                )
            )
            if path.suffix.lower() not in TEXT_SUFFIXES:
                continue
        try:
            lines = path.read_text(encoding="utf-8-sig", errors="ignore").splitlines()
        except OSError:
            continue
        for line_number, line in enumerate(lines, start=1):
            if not SEARCH_PATTERN.search(line):
                continue
            hits.append(
                Hit(
                    path=rel,
                    line=line_number,
                    category=category,
                    text=line.strip()[:240],
                )
            )
    return AuditResult(
        root=str(root),
        hits=sorted(hits, key=lambda hit: (hit.category, hit.path, hit.line)),
        vendored_tree_present=vendored_tree_present,
    )


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def print_text_report(result: AuditResult, *, fail_on_remaining: bool) -> None:
    violations = list(result.violations)
    if fail_on_remaining:
        violations.extend(result.remaining_dependency_hits)

    status = "FAIL" if violations else "OK"
    print(f"Pose graph GTSAM coverage audit: {status}")
    for category, count in result.category_counts.items():
        description = CATEGORY_DESCRIPTIONS.get(category, "")
        print(f"  {category}: {count} ({description})")

    covered_surfaces = [
        str(surface["surface"])
        for surface in CAPABILITY_SURFACES
        if surface["coverage"] == "covered_by_pose_graph_opt"
    ]
    remaining_surfaces = [
        str(surface["surface"])
        for surface in CAPABILITY_SURFACES
        if surface["coverage"] == "remaining_dependency"
    ]
    print("\nCapability coverage:")
    print(f"  covered_by_pose_graph_opt: {', '.join(covered_surfaces)}")
    print(f"  remaining_dependency: {', '.join(remaining_surfaces)}")

    if result.remaining_dependency_hits:
        print("\nRemaining dependency summary:")
        subsurface_summary = result.remaining_dependency_subsurface_summary
        for surface, bucket in result.remaining_dependency_surface_summary.items():
            print(
                "  "
                f"{surface}: {bucket['hit_count']} hit(s), {bucket['path_count']} path(s)"
            )
            for subsurface, subsurface_bucket in subsurface_summary.items():
                if subsurface_bucket["surface"] != surface:
                    continue
                priority = subsurface_bucket.get("priority") or "unranked"
                print(
                    "    "
                    f"{subsurface} [{priority}]: "
                    f"{subsurface_bucket['hit_count']} hit(s), "
                    f"{subsurface_bucket['path_count']} path(s)"
                )
            for record in bucket.get("representative_paths", []):
                print(
                    "    - "
                    f"{record['path']} "
                    f"({record['hit_count']} hit(s), first line {record['first_line']})"
                )
        unique_paths = sorted({hit.path for hit in result.remaining_dependency_hits})
        print("\nKnown remaining GTSAM-dependent surfaces:")
        for path in unique_paths[:20]:
            print(f"  - {path}")
        if len(unique_paths) > 20:
            print(f"  - ... {len(unique_paths) - 20} more path(s)")

    if violations:
        print("\nViolations:")
        for hit in violations[:50]:
            print(f"  ERROR: {hit.path}:{hit.line}: [{hit.category}] {hit.text}")
        if len(violations) > 50:
            print(f"  ERROR: ... {len(violations) - 50} more violation(s)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate the GTSAM coverage boundary for Rust pose_graph_opt",
    )
    parser.add_argument("--root", type=Path, default=ROOT_DIR, help="Repository root")
    parser.add_argument("--json-out", type=Path, help="Write full audit JSON")
    parser.add_argument("--json", action="store_true", help="Print full audit JSON")
    parser.add_argument(
        "--fail-on-remaining",
        action="store_true",
        help="Also fail on known remaining GTSAM-dependent surfaces",
    )
    args = parser.parse_args()

    result = scan_repository(args.root)
    payload = result.to_jsonable()
    if args.json_out:
        write_json(args.json_out, payload)
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print_text_report(result, fail_on_remaining=args.fail_on_remaining)

    has_failure = bool(result.violations) or (
        args.fail_on_remaining and bool(result.remaining_dependency_hits)
    )
    if has_failure:
        sys.exit(1)


if __name__ == "__main__":
    main()
