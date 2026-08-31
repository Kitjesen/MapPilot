import subprocess
import sys
from pathlib import Path

from tools.validate.validate_docs import validate_repository

ROOT = Path(__file__).resolve().parents[2]


def test_first_party_documentation_contract() -> None:
    """Keep maintained Markdown within the documented structure."""

    violations, scanned = validate_repository(ROOT)

    assert scanned > 0
    assert violations == [], "\n".join(violations)


def test_generated_api_documentation_is_current() -> None:
    """Generated route and MCP inventories must match current source."""

    result = subprocess.run(
        [sys.executable, "tools/docs/extract_api_docs.py", "--check"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr


def test_gateway_inventory_preserves_canonical_recording_routes() -> None:
    """The generated inventory must expose the canonical recording API."""

    inventory = (ROOT / "docs" / "api" / "gateway_rest.md").read_text(
        encoding="utf-8"
    )
    for action in ("start", "status", "stop"):
        assert f"/api/v1/recordings/{action}" in inventory
