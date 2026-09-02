"""Static contract for the three LingTu simulation product surfaces."""

# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PRODUCT_DEFINITION = REPO_ROOT / "docs" / "product" / "simulation-studio-product.md"
EXECUTION_PLAN = REPO_ROOT / "docs" / "plans" / "ue5-playable-vertical-slice.md"
PREACCEPTANCE = (
    REPO_ROOT
    / "docs"
    / "07-testing"
    / "field-runs"
    / "2026-08-12-ue5-playable-preacceptance.md"
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _normalized(text: str) -> str:
    return " ".join(text.split())


def test_product_definition_keeps_three_distinct_surfaces() -> None:
    product = _read(PRODUCT_DEFINITION)
    plan = _read(EXECUTION_PLAN)

    assert "Status: **confirmed product boundary" in product
    assert "### 1. UE5 Runtime — primary playable product" in product
    assert "### 2. UE5 Create — immersive 3D authoring mode" in product
    assert "### 3. SimStudio — management and evidence product" in product
    assert "Pretending a static Web image is the UE live world." in product
    assert "Replacing UE with a Web “game” shell." in product
    assert "RobotSimUE Runtime is the playable product surface." in plan
    assert "RobotSimUE Create is the later immersive 3D authoring surface." in plan
    assert "It is not a substitute for a UE game viewport." in plan


def test_component_greens_cannot_qualify_the_playable_product() -> None:
    product = _normalized(_read(PRODUCT_DEFINITION))
    plan = _normalized(_read(EXECUTION_PLAN))
    evidence = _normalized(_read(PREACCEPTANCE))

    assert "The first playable slice is **not yet qualified**" in product
    assert "No Web preview, component test, offline screenshot" in product
    assert "Anything less remains a component capability" in plan
    assert "Status: **NO-GO for playable qualification" in evidence
    assert "It does not qualify the UE5 playable product" in evidence


def test_matrix_benchmark_is_commit_pinned_and_documentation_bounded() -> None:
    product = _normalized(_read(PRODUCT_DEFINITION))

    assert "6ec0b354b93b0dd0ccdfb2d1c012fb5cc3f52a30" in product
    assert "immutable documentation snapshot" in product
    assert "documentation-level" in product
    assert "does not contain the referenced UeSim project" in product
    assert "not as independently reproduced implementation or performance evidence" in product
