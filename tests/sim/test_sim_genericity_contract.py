"""Static architecture gates for generic simulation production code."""

from __future__ import annotations

import ast
import re
from pathlib import Path

from sim.catalog.resolver import CatalogResolver

REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "sim"

CPP_SUFFIXES = {".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".inl"}
MODEL_SPECIFIC_SOURCES = {
    SIM_ROOT / "runtime" / "control" / "thunderv4.py",
    SIM_ROOT / "runtime" / "coordinator" / "playable_evidence.py",
    SIM_ROOT / "runtime" / "coordinator" / "playable_vertical_slice.py",
    SIM_ROOT
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
    / "Source"
    / "LingTuSimUI"
    / "Private"
    / "SLingTuSimRuntimeHUD.cpp",
}

PHYSICS_SOURCE_ROOTS = tuple(
    SIM_ROOT / "runtime" / "physics" / name for name in ("include", "src", "apps")
)
PYTHON_SOURCE_ROOTS = tuple(
    SIM_ROOT / "runtime" / name for name in ("coordinator", "scenario", "sensors", "recording")
)
UNREAL_SOURCE_ROOT = SIM_ROOT / "runtime" / "visual" / "RobotSimUE" / "Plugins" / "LingTuSim" / "Source"

FORBIDDEN_IDENTITIES = {
    "ThunderV4": re.compile(r"(?i)(?<![a-z0-9])thunder[\s_-]*v4(?![a-z0-9])"),
    "OmniCart": re.compile(r"(?i)(?<![a-z0-9])omni[\s_-]*cart(?![a-z0-9])"),
    "Go2": re.compile(r"(?i)(?<![a-z0-9])go[\s_-]*2(?![a-z0-9])"),
    "H1": re.compile(r"(?i)(?<![a-z0-9])h[\s_-]*1(?![a-z0-9])"),
    "OpenField": re.compile(r"(?i)(?<![a-z0-9])open[\s_-]*field(?![a-z0-9])"),
}
EXPECTED_BODY_COUNT = re.compile(r"\bEXPECTED_[A-Z0-9_]*BODY_COUNT\b", re.IGNORECASE)
EQUALS_21 = re.compile(r"==\s*21\b")
BODY_COUNT_21 = re.compile(r"\bbody_count\s*=\s*21\b", re.IGNORECASE)

LEGACY_MANIFEST_ROOTS = {"robots", "controllers", "sensor_rigs", "worlds"}
LEGACY_MANIFEST_NAMES = {
    "robot.package.yaml",
    "controller.package.yaml",
    "sensor-rig.package.yaml",
    "world.package.yaml",
}
LEGACY_MANIFEST_PATH = re.compile(
    r"(?i)(?:^|[\s\"'`()\[\],])(?:\.\.?[/\\])?sim[/\\]"
    r"(?:robots|controllers|sensor_rigs|worlds)(?:[/\\][^\s\"'`()\[\],]+)*[/\\]"
    r"(?:robot|controller|sensor-rig|world)\.package\.yaml(?:$|[^a-z0-9_.-])"
)


def _generic_source_files() -> list[Path]:
    files: list[Path] = []
    files.extend(path for path in (SIM_ROOT / "catalog").glob("*.py") if path.is_file())

    for root in PHYSICS_SOURCE_ROOTS:
        if root.is_dir():
            files.extend(path for path in root.rglob("*") if path.is_file() and path.suffix.lower() in CPP_SUFFIXES)

    for root in PYTHON_SOURCE_ROOTS:
        if root.is_dir():
            files.extend(path for path in root.rglob("*.py") if path.is_file())

    for module_root in UNREAL_SOURCE_ROOT.iterdir():
        for visibility in ("Public", "Private"):
            root = module_root / visibility
            if not root.is_dir():
                continue
            for path in root.rglob("*"):
                if not path.is_file() or path.suffix.lower() not in CPP_SUFFIXES:
                    continue
                if any(part.casefold() == "tests" for part in path.relative_to(root).parts):
                    continue
                files.append(path)

    model_specific = {item.resolve() for item in MODEL_SPECIFIC_SOURCES}
    files = [path for path in files if path.resolve() not in model_specific]
    return sorted(files)


def _path_chain(node: ast.AST) -> tuple[str, ...] | None:
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
        left = _path_chain(node.left)
        right = _path_chain(node.right)
        if left is None or right is None:
            return None
        return left + right
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return (node.value,)
    if isinstance(node, ast.Name):
        return ()
    return None


def _legacy_manifest_reference(text: str) -> bool:
    """Match old catalog roots only when the path names a legacy manifest."""

    return bool(LEGACY_MANIFEST_PATH.search(text.replace("\\", "/")))


def _legacy_manifest_references(path: Path) -> list[str]:
    references: list[str] = []
    text = path.read_text(encoding="utf-8")
    for line_number, line in enumerate(text.splitlines(), 1):
        if _legacy_manifest_reference(line):
            references.append(f"{path.relative_to(REPO_ROOT)}:{line_number}")

    if path.suffix.lower() == ".py":
        tree = ast.parse(text, filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.BinOp):
                continue
            chain = _path_chain(node)
            if chain is None or len(chain) < 4:
                continue
            normalized = "/".join(chain).replace("\\", "/")
            if (
                len(chain) >= 4
                and chain[0] == "sim"
                and chain[1] in LEGACY_MANIFEST_ROOTS
                and chain[-1].casefold() in LEGACY_MANIFEST_NAMES
            ):
                references.append(f"{path.relative_to(REPO_ROOT)}:{getattr(node, 'lineno', 1)}: {normalized}")
    return references


def test_generic_compiler_and_runtime_sources_are_model_and_world_neutral() -> None:
    violations: list[str] = []
    for path in _generic_source_files():
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            for label, pattern in FORBIDDEN_IDENTITIES.items():
                if pattern.search(line):
                    violations.append(f"{path.relative_to(REPO_ROOT)}:{line_number}: {label}")
            if EXPECTED_BODY_COUNT.search(line):
                violations.append(f"{path.relative_to(REPO_ROOT)}:{line_number}: EXPECTED_*BODY_COUNT")
            if EQUALS_21.search(line):
                violations.append(f"{path.relative_to(REPO_ROOT)}:{line_number}: explicit == 21")
            if BODY_COUNT_21.search(line):
                violations.append(f"{path.relative_to(REPO_ROOT)}:{line_number}: body_count = 21")

    assert not violations, "generic compiler/runtime source leaked model assumptions:\n" + "\n".join(violations)


def test_production_sources_use_canonical_manifests_and_keep_asset_roots_valid() -> None:
    """Only old manifest paths are forbidden; MJCF and mesh roots remain valid assets."""
    resolver = CatalogResolver.from_repository(REPO_ROOT)

    assert resolver.catalog_roots == ((SIM_ROOT / "packages").resolve(),)
    assert not any(_legacy_manifest_references(path) for path in _generic_source_files())
    assert not _legacy_manifest_reference("sim/packages/robots/doso/thunder_v4/mjcf/thunderv4.xml")
    assert not _legacy_manifest_reference("sim/packages/robots/doso/thunder_v4/meshes/base.stl")
