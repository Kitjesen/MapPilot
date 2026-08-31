"""Product-documentation contracts for native recording and replay."""

# ruff: noqa: D103, S101 - pytest contracts use assertions by design.

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GUIDE = ROOT / "docs" / "04-deployment" / "native_recording.md"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_native_recording_has_one_truthful_operator_guide() -> None:
    guide = _read(GUIDE)

    for expected in (
        "C++",
        "CycloneDDS",
        "MCAP",
        "lingtu_recorder",
        "lingtu_dds_player",
        "--min-free-gib",
        "5 GiB",
        "DDS domain `84`",
        "session.json",
        ".mcap.tmp",
        "record-only",
    ):
        assert expected in guide

    assert "startup preflight" in guide.lower()
    assert "0" in guide and "disable" in guide.lower()
    assert "running quota" in guide.lower()
    assert "not yet" in guide.lower()



def test_recording_docs_expose_only_native_mcap_routes() -> None:
    guide = _read(GUIDE)

    for endpoint in (
        "/api/v1/recordings/start",
        "/api/v1/recordings/status",
        "/api/v1/recordings/stop",
    ):
        assert endpoint in guide

    assert "/api/v1/bag/" not in guide
